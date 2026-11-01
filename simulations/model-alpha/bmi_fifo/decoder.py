"""Restricted ordinary BMI2 FIFO header burst decoder;no sample-time reconstruction."""
import struct
class HeaderBurstDecoder:
    def __init__(self):self.fault=None
    def decode(self,data):
        events=[];fault=self.fault;offset=0
        if fault is None:
            try:
                if not isinstance(data,bytes) or len(data)>8192:raise ValueError('bounded complete bytes burst required')
                while offset<len(data):
                    begin=offset;header=data[offset];offset+=1;tag=header&3;kind=header&0xfc
                    lengths={0x84:6,0x88:6,0x8c:12,0x44:3,0x40:1,0x48:4,0x80:0}
                    if kind not in lengths:raise ValueError('unsupported header at byte '+str(begin))
                    n=lengths[kind]
                    if offset+n>len(data):raise ValueError('truncated frame at byte '+str(begin))
                    payload=data[offset:offset+n];offset+=n;row=dict(byte_offset=begin,header=header,interrupt_tag=tag,kind=None,accel_counts=None,gyro_counts=None,sensor_time_ticks=None,skipped_frames=None,input_config_bytes=None,acquisition_s=None,sample_sequence_proven=False,hardware_driver_executed=False,usable_for_feedback=False)
                    if kind==0x84:row.update(kind='accel',accel_counts=list(struct.unpack('<hhh',payload)))
                    elif kind==0x88:row.update(kind='gyro',gyro_counts=list(struct.unpack('<hhh',payload)))
                    elif kind==0x8c:row.update(kind='gyro_accel',gyro_counts=list(struct.unpack('<hhh',payload[:6])),accel_counts=list(struct.unpack('<hhh',payload[6:])))
                    elif kind==0x44:row.update(kind='sensor_time',sensor_time_ticks=int.from_bytes(payload,'little'))
                    elif kind==0x40:
                        row.update(kind='skip',skipped_frames=payload[0])
                        if payload[0]:fault=fault or 'unqualified_skipped_frames'
                    elif kind==0x48:row.update(kind='input_config',input_config_bytes=list(payload));fault=fault or 'unqualified_configuration_change'
                    else:
                        row['kind']='end_of_data'
                        if offset!=len(data):raise ValueError('unsupported trailing overread padding')
                    events.append(row)
                self.fault=fault
            except (ValueError,TypeError,struct.error) as error:self.fault=fault or type(error).__name__+':'+str(error)
        return dict(status='latched_unavailable' if self.fault else 'decoded_restricted_burst',fault=self.fault,events=events,bytes_consumed=offset,syntax_compatible=self.fault is None,acquisition_s=None,conversion_freshness_proven=False,sample_sequence_proven=False,hardware_driver_executed=False,usable_for_feedback=False)
