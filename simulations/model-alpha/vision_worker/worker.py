"""Child process:immutable image copies,estimation and compact result IPC only."""
import json,time,os,hashlib
from multiprocessing import shared_memory
FREE,WRITING,READY,READING=0,1,2,3
def loop(names,sizes,state,headers,header_lengths,input_lock,output_lock,output_length,stats,stop,calibration,backend,delay):
    blocks=[shared_memory.SharedMemory(name=n) for n in names]
    try:
        estimator=None
        if backend=='actual':
            from pose_estimator import estimate
            estimator=estimate
        stats[0]=1
        while not stop.is_set():
            slot=None
            with input_lock:
                ready=[i for i in range(2) if state[i]==READY]
                if ready:slot=ready[-1];state[slot]=READING
            if slot is None:stop.wait(.001);continue
            try:
                metadata=json.loads(bytes(headers[slot][:header_lengths[slot]]));payload=bytes(blocks[slot].buf[:sizes[slot]]);stats[1]=metadata['sequence']
            finally:
                with input_lock:state[slot]=FREE
            start=time.perf_counter_ns()
            try:
                if delay:stop.wait(delay)
                if backend=='manufactured_error':raise RuntimeError('Declared manufactured estimator failure')
                if backend=='manufactured_exit':os._exit(17)
                if estimator:
                    stride=metadata['width']*metadata['height']*3;frames=[dict(camera_id=c,width=metadata['width'],height=metadata['height'],rgb=payload[i*stride:(i+1)*stride]) for i,c in enumerate(metadata['camera_ids'])];result=estimator(frames,calibration,'apriltag');compact=dict(status=result['status'],chosen=result['chosen'],calibrated_uncertainty=None,controller_usable=False,manufactured=False)
                else:compact=dict(status='manufactured_echo',chosen=None,calibrated_uncertainty=None,controller_usable=False,manufactured=True)
            except Exception as e:compact=dict(status='worker_error',chosen=None,error=type(e).__name__+': '+str(e),controller_usable=False,calibrated_uncertainty=None,manufactured=backend!='actual')
            compact.update(sequence=metadata['sequence'],session=metadata['session'],calibration_id=metadata['calibration_id'],capture_times_s=metadata['capture_times_s'],camera_ids=metadata['camera_ids'],worker_pid=os.getpid(),processing_host_ms=(time.perf_counter_ns()-start)/1e6,input_rgb_sha256=hashlib.sha256(payload).hexdigest());encoded=json.dumps(compact,allow_nan=False).encode();assert len(encoded)<sizes[2]
            with output_lock:blocks[2].buf[:len(encoded)]=encoded;output_length.value=len(encoded)
            stats[2]=metadata['sequence'];stats[3]+=1
    finally:
        stats[0]=0
        for b in blocks:b.close()
