"""Own only the added accelerometer configuration; shared gyro remains read only."""
class AccelOwner:
    def __init__(self,accel,gyro,enabled_read_s=0.):
        self.accel=accel;self.gyro=gyro
        before=accel.getSamplingPeriod()
        if before!=0 or gyro.getSamplingPeriod()!=8:raise ValueError('unowned initial configuration')
        accel.enable(8)
        self.initial_action=dict(after_sample_s=enabled_read_s,action='initial_enable',before_accel_period_ms=before,after_accel_period_ms=accel.getSamplingPeriod(),gyro_period_ms=gyro.getSamplingPeriod())
        if self.initial_action['after_accel_period_ms']!=8:raise ValueError('initial enable failed')
    def periods(self,t):return dict(read_s=t,accel_period_ms=self.accel.getSamplingPeriod(),gyro_period_ms=self.gyro.getSamplingPeriod())
    def after_read(self,t,loss):
        if not loss or not(abs(t-50.)<1e-9 or abs(t-50.040)<1e-9):return []
        action='disable' if t<50.020 else 'enable';before=self.accel.getSamplingPeriod()
        if action=='disable':self.accel.disable()
        else:self.accel.enable(8)
        return [dict(after_sample_s=t,action=action,before_accel_period_ms=before,after_accel_period_ms=self.accel.getSamplingPeriod(),gyro_period_ms=self.gyro.getSamplingPeriod())]
