import ctypes
from context import SOURCE
class VendorParser:
    def __init__(self):
        self.library=ctypes.CDLL(str(SOURCE/'build/fifo_probe.dll'));self.function=self.library.fifo_probe;self.function.argtypes=[ctypes.POINTER(ctypes.c_uint8),ctypes.c_uint32,ctypes.POINTER(ctypes.c_int32),ctypes.c_uint32];self.function.restype=ctypes.c_int
    def decode(self,data):
        raw=(ctypes.c_uint8*max(1,len(data)))(*data);capacity=9+6*(len(data)+1);out=(ctypes.c_int32*capacity)();r=self.function(raw,len(data),out,capacity)
        if r!=0:raise ValueError('vendor memory bridge error '+str(r))
        na,ng=out[2],out[3];acc=[list(out[9+3*i:12+3*i]) for i in range(na)];start=9+3*na;gyro=[list(out[start+3*i:start+3*i+3]) for i in range(ng)]
        return dict(accel=acc,gyro=gyro,accel_status=out[0],gyro_status=out[1],last_sensor_time_ticks=out[4],last_skipped_frame_count=out[5],accel_byte_index=out[6],gyro_byte_index=out[7],forbidden_io_calls=out[8])
