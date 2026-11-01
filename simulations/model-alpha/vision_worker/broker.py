"""One producer thread,one spawned estimator,latest pending RGB bundle only.

Publication/polling never wait for IPC locks or run inference. Memory copies and
OS scheduling are still work;this prototype does not guarantee an8msdeadline.
"""
import multiprocessing as mp,json,math,threading,uuid,os
from multiprocessing import shared_memory
from worker import loop,FREE,WRITING,READY

class Broker:
    def __init__(self,calibration,calibration_id,*,minimum_latency_s,max_age_s,width=640,height=480,backend='actual',processing_delay_s=0):
        if not(0<=minimum_latency_s<=max_age_s and max_age_s>0):raise ValueError('Explicit finite latency/age policy required')
        if not all(math.isfinite(v) for v in [minimum_latency_s,max_age_s,processing_delay_s]) or processing_delay_s<0:raise ValueError('Finite timing values required')
        self.owner=threading.get_ident();self.session=uuid.uuid4().hex;self.calibration_id=calibration_id;self.ids=tuple(c['id'] for c in calibration['cameras']);self.width=width;self.height=height
        if len(self.ids)!=2 or len(set(self.ids))!=2:raise ValueError('Two distinct fixed camera IDs required')
        self.minimum_latency=minimum_latency_s;self.max_age=max_age_s;self.clock=None;self.invalid=False;self.last_submitted=0;self.last_capture=-float('inf');self.last_seen=0;self.pending=None;self.closed=False;self.metrics=dict(submitted=0,replaced_pending=0,busy_publications=0,stale_results=0);ctx=mp.get_context('spawn');size=2*width*height*3;self.sizes=[size,size,65536];self.blocks=[shared_memory.SharedMemory(create=True,size=n) for n in self.sizes];self.state=ctx.Array('i',2,lock=False);self.headers=[ctx.Array('B',4096,lock=False) for _ in range(2)];self.header_lengths=ctx.Array('i',2,lock=False);self.input_lock=ctx.Lock();self.output_lock=ctx.Lock();self.output_length=ctx.Value('i',0,lock=False);self.stats=ctx.Array('q',4,lock=False);self.stop=ctx.Event();self.process=ctx.Process(target=loop,args=([b.name for b in self.blocks],self.sizes,self.state,self.headers,self.header_lengths,self.input_lock,self.output_lock,self.output_length,self.stats,self.stop,calibration,backend,processing_delay_s),daemon=True);self.process.start()
    def _clock(self,now):
        if threading.get_ident()!=self.owner:raise RuntimeError('Broker has one producer/poll thread')
        if self.closed:raise RuntimeError('Broker is closed')
        if not isinstance(now,(int,float)) or not math.isfinite(now):return False
        if self.clock is not None and now<self.clock:self.invalid=True;self.pending=None
        self.clock=now;return not self.invalid
    def publish(self,sequence,capture_times_s,rgb_frames,now_sim_s):
        if not self._clock(now_sim_s):return 'clock_invalid'
        if type(sequence)!=int or sequence<=self.last_submitted:return 'sequence_invalid'
        if len(capture_times_s)!=2 or not all(isinstance(t,(int,float)) and math.isfinite(t) for t in capture_times_s):return 'timestamp_invalid'
        if max(capture_times_s)>now_sim_s or min(capture_times_s)<=self.last_capture or max(capture_times_s)-min(capture_times_s)>.01:return 'timestamp_invalid'
        if now_sim_s-min(capture_times_s)>self.max_age:return 'stale_input'
        stride=self.width*self.height*3
        if len(rgb_frames)!=2 or any(type(a)!=bytes or len(a)!=stride for a in rgb_frames):return 'format_invalid'
        metadata=dict(sequence=sequence,capture_times_s=tuple(capture_times_s),camera_ids=self.ids,width=self.width,height=self.height,calibration_id=self.calibration_id,session=self.session);encoded=json.dumps(metadata,allow_nan=False).encode()
        if len(encoded)>4096:return 'metadata_too_large'
        if not self.input_lock.acquire(False):self.metrics['busy_publications']+=1;return 'busy'
        try:
            free=[i for i in range(2) if self.state[i] in [FREE,READY]]
            if not free:self.metrics['busy_publications']+=1;return 'busy'
            slot=free[0]
            for i in range(2):
                if self.state[i]==READY:self.state[i]=FREE;self.metrics['replaced_pending']+=1
            self.state[slot]=WRITING
        finally:self.input_lock.release()
        self.blocks[slot].buf[:stride]=rgb_frames[0];self.blocks[slot].buf[stride:2*stride]=rgb_frames[1];self.headers[slot][:len(encoded)]=encoded;self.header_lengths[slot]=len(encoded)
        # Commit without waiting. If contention occurs,discard this bundle.
        if not self.input_lock.acquire(False):self.state[slot]=FREE;self.metrics['busy_publications']+=1;return 'busy'
        try:self.state[slot]=READY
        finally:self.input_lock.release()
        self.last_submitted=sequence;self.last_capture=min(capture_times_s);self.metrics['submitted']+=1;return 'published'
    def poll(self,now_sim_s):
        if not self._clock(now_sim_s):return dict(status='clock_invalid',packet=None)
        if not self.process.is_alive():self.pending=None;return dict(status='worker_dead',packet=None)
        if not self.output_lock.acquire(False):return dict(status='busy',packet=None)
        try:
            n=self.output_length.value;data=bytes(self.blocks[2].buf[:n]) if n else None
        finally:self.output_lock.release()
        if data:
            try:r=json.loads(data)
            except Exception:self.pending=None;return dict(status='invalid_output',packet=None)
            if r['session']!=self.session or r['calibration_id']!=self.calibration_id or tuple(r['camera_ids'])!=self.ids:self.pending=None;return dict(status='invalid_output',packet=None)
            if r['sequence']>self.last_seen:self.last_seen=r['sequence'];self.pending=r
        if self.pending is None:return dict(status='waiting',packet=None)
        r=self.pending;capture=min(r['capture_times_s']);age=now_sim_s-capture
        if age<0:self.pending=None;return dict(status='future_output',packet=None)
        if age>self.max_age:self.pending=None;self.metrics['stale_results']+=1;return dict(status='stale',packet=None)
        if age<self.minimum_latency:return dict(status='scheduled_wait',packet=None)
        self.pending=None;r=dict(r,delivered_at_sim_s=now_sim_s,age_sim_s=age,minimum_scheduled_latency_s=self.minimum_latency);return dict(status='delivered',packet=r)
    def close(self):
        if self.closed:return
        self.stop.set();self.process.join(2)
        if self.process.is_alive():self.process.terminate();self.process.join(2)
        self.closed=True
        for b in self.blocks:b.close();b.unlink()
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
