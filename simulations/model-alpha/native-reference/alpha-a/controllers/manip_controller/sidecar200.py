"""Passive five-ms pair, distinct producer/host clocks and audit-only witnesses."""
import json
from measurement import PassiveCapture
from buffer import SnapshotBuffer

class Alpha200:
    def __init__(self,robot,body,out,restart=False):
        self.robot=robot;self.body=body;self.restart=restart;self.queue=SnapshotBuffer();self.out=out
        self.a=robot.getDevice('raw200_accel');self.g=robot.getDevice('raw200_gyro')
        before=[self.a.getSamplingPeriod(),self.g.getSamplingPeriod()]
        if before!=[0,0]:raise ValueError('dedicated pair already configured')
        self.a.enable(5);self.g.enable(5)
        self.capture=PassiveCapture(self.a,self.g,period_ms=5,enabled_s=robot.getTime())
        self.files={n:(out/n).open('w',encoding='utf-8') for n in ['raw200-producer.jsonl','raw200-reference.jsonl','raw200-host.jsonl','raw200-actions.jsonl']}
        (out/'raw200-init.json').write_text(json.dumps(dict(read_s=robot.getTime(),before=before,after=[self.a.getSamplingPeriod(),self.g.getSamplingPeriod()]),allow_nan=False),encoding='utf-8')
        self.reference(robot.getTime())
    def write(self,name,row):self.files[name].write(json.dumps(row,allow_nan=False)+'\n')
    def reference(self,t):
        # Supervisor witnesses never enter the producer, transport or old control path.
        b=self.body
        self.write('raw200-reference.jsonl',dict(audit_only=True,time_s=t,position=list(b.getPosition()),orientation=list(b.getOrientation()),velocity=list(b.getVelocity())))
    def tick(self,t):
        ms=round(t*1000)
        if ms%5==0:
            row=dict(producer_read_s=t,samples=dict(identity=dict(snapshot=self.capture.read(t),accel_period_ms=self.a.getSamplingPeriod(),gyro_period_ms=self.g.getSamplingPeriod(),gyro_api=list(self.g.getValues()))))
            self.write('raw200-producer.jsonl',row);self.queue.push(row);self.reference(t)
            if self.restart and ms in [50000,50040]:
                action='disable' if ms==50000 else 'enable';before=self.a.getSamplingPeriod()
                if action=='disable':self.a.disable()
                else:self.a.enable(5)
                self.write('raw200-actions.jsonl',dict(after_producer_read_s=t,action=action,before_accel_period_ms=before,after_accel_period_ms=self.a.getSamplingPeriod(),gyro_period_ms=self.g.getSamplingPeriod()))
        if ms%8==0:self.write('raw200-host.jsonl',self.queue.read(t))
    def close(self,t):
        for f in self.files.values():f.close()
        (self.out/'raw200-completion.json').write_text(json.dumps(dict(time_s=t,pending=len(self.queue.pending),queue_fault=self.queue.fault,usable_for_feedback=False,hardware_driver_executed=False,acquisition_s=None),allow_nan=False),encoding='utf-8')
