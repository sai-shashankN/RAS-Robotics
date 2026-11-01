import unittest,time,json,os,threading
from broker import Broker
CAL=dict(cameras=[dict(id='left'),dict(id='right')])
def wait_for(predicate,timeout=8):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        value=predicate()
        if value:return value
        time.sleep(.005)
    raise AssertionError('Bounded worker observation timed out')
def ready(b):wait_for(lambda:b.stats[0]==1)
def publish(b,n,t):return b.publish(n,(t,t),(b'\0'*48,b'\1'*48),t)
class TransportChecks(unittest.TestCase):
    def make(self,**kwargs):return Broker(CAL,'manufactured-calibration',minimum_latency_s=kwargs.pop('minimum_latency_s',0),max_age_s=.5,width=4,height=4,backend=kwargs.pop('backend','manufactured'),**kwargs)
    def test_real_child_and_separate_simulator_latency(self):
        with self.make(minimum_latency_s=.08) as b:
            ready(b);self.assertEqual(publish(b,1,1.),'published');wait_for(lambda:b.stats[2]==1);self.assertEqual(b.poll(1.04)['status'],'scheduled_wait');r=b.poll(1.088);self.assertEqual(r['status'],'delivered');self.assertEqual(r['packet']['capture_times_s'],[1.,1.]);self.assertAlmostEqual(r['packet']['age_sim_s'],.088);self.assertNotEqual(r['packet']['worker_pid'],os.getpid());self.assertTrue(r['packet']['manufactured'])
    def test_latest_pending_overload_drops_instead_of_queues(self):
        with self.make(processing_delay_s=.15) as b:
            ready(b);self.assertEqual(publish(b,1,1.),'published');wait_for(lambda:b.stats[1]==1)
            for n in range(2,21):
                t=1.+n*.001;wait_for(lambda:publish(b,n,t)=='published')
            wait_for(lambda:b.stats[2]==20);r=b.poll(1.03);self.assertEqual(r['packet']['sequence'],20);self.assertEqual(b.stats[3],2);self.assertGreaterEqual(b.metrics['replaced_pending'],18)
    def test_busy_locks_return_without_waiting(self):
        with self.make() as b:
            ready(b);b.input_lock.acquire()
            try:self.assertEqual(publish(b,1,1.),'busy')
            finally:b.input_lock.release()
            b.output_lock.acquire()
            try:self.assertEqual(b.poll(1.)['status'],'busy')
            finally:b.output_lock.release()
    def test_stale_result_is_not_reissued(self):
        with self.make() as b:
            ready(b);publish(b,1,1.);wait_for(lambda:b.stats[2]==1);self.assertEqual(b.poll(1.6)['status'],'stale');self.assertIsNone(b.poll(1.61)['packet'])
    def test_invalid_or_replayed_inputs_reject(self):
        with self.make() as b:
            ready(b);self.assertEqual(b.publish(1,(1.,1.),(b'\0',b'\0'),1.),'format_invalid');self.assertEqual(publish(b,1,1.),'published');self.assertEqual(publish(b,1,1.),'sequence_invalid');self.assertEqual(b.publish(2,(2.,2.),(b'\0'*48,b'\0'*48),1.1),'timestamp_invalid')
    def test_clock_rollback_invalidates_session(self):
        with self.make() as b:
            ready(b);b.poll(2.);self.assertEqual(b.poll(1.9)['status'],'clock_invalid');self.assertEqual(publish(b,1,2.1),'clock_invalid')
    def test_estimator_error_is_explicit_no_old_pose(self):
        with self.make(backend='manufactured_error') as b:
            ready(b);publish(b,1,1.);wait_for(lambda:b.stats[2]==1);r=b.poll(1.1);self.assertEqual(r['packet']['status'],'worker_error');self.assertIsNone(r['packet']['chosen']);self.assertFalse(r['packet']['controller_usable'])
    def test_worker_death_is_detected(self):
        with self.make(backend='manufactured_exit') as b:
            ready(b);publish(b,1,1.);wait_for(lambda:not b.process.is_alive());self.assertEqual(b.poll(1.1)['status'],'worker_dead')
    def test_foreign_session_result_rejects(self):
        with self.make() as b:
            ready(b);encoded=json.dumps(dict(session='foreign',calibration_id=b.calibration_id,camera_ids=list(b.ids))).encode()
            with b.output_lock:b.blocks[2].buf[:len(encoded)]=encoded;b.output_length.value=len(encoded)
            self.assertEqual(b.poll(1.)['status'],'invalid_output')
    def test_one_producer_thread_enforced(self):
        with self.make() as b:
            caught=[]
            def other():
                try:publish(b,1,1.)
                except RuntimeError:caught.append(True)
            t=threading.Thread(target=other);t.start();t.join();self.assertEqual(caught,[True])
if __name__=='__main__':unittest.main()
