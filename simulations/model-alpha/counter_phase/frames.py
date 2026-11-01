"""Selected ordinary frames including documented80/00overread;no hardware clocks."""
import struct
class FrameError(ValueError):pass
def parse(data):
    if not isinstance(data,bytes) or len(data)>8192:raise FrameError('bounded complete bytes burst required')
    result=[];offset=0
    lengths={0x84:6,0x88:6,0x8c:12,0x44:3,0x40:1,0x48:4,0x80:1}
    while offset<len(data):
        begin=offset;header=data[offset];kind=header&0xfc;offset+=1
        if kind not in lengths:raise FrameError('unsupported header at '+str(begin))
        size=lengths[kind]
        if offset+size>len(data):raise FrameError('truncated frame at '+str(begin))
        payload=data[offset:offset+size];offset+=size
        r=dict(byte_offset=begin,byte_length=size+1,raw_header=header,interrupt_tag=header&3,kind=None,accel_counts=None,gyro_counts=None,sensor_time_ticks=None,skipped_frames=None,config_flags=None,config_next_frame_ticks=None)
        if kind==0x84:r.update(kind='accel',accel_counts=list(struct.unpack('<hhh',payload)))
        elif kind==0x88:r.update(kind='gyro',gyro_counts=list(struct.unpack('<hhh',payload)))
        elif kind==0x8c:r.update(kind='gyro_accel',gyro_counts=list(struct.unpack('<hhh',payload[:6])),accel_counts=list(struct.unpack('<hhh',payload[6:])))
        elif kind==0x44:r.update(kind='sensor_time',sensor_time_ticks=int.from_bytes(payload,'little'))
        elif kind==0x40:r.update(kind='skip',skipped_frames=payload[0])
        elif kind==0x48:
            if payload[0]&0xf0:raise FrameError('unselected auxiliary/reserved configuration flags')
            r.update(kind='input_config',config_flags=payload[0],config_next_frame_ticks=int.from_bytes(payload[1:],'little'))
        else:
            if header!=0x80 or payload!=b'\x00':raise FrameError('unsupported overread header/payload')
            r['kind']='overread'
        result.append(r)
    return result
def grammar(events):
    times=[i for i,e in enumerate(events) if e['kind']=='sensor_time'];skips=[i for i,e in enumerate(events) if e['kind']=='skip'];config=[i for i,e in enumerate(events) if e['kind']=='input_config'];over=[i for i,e in enumerate(events) if e['kind']=='overread']
    if len(skips)>1 or skips and skips[0]!=0:raise FrameError('skip must be a single first control')
    if skips and events[skips[0]]['skipped_frames']>0:raise FrameError('positive skipped frames')
    if len(over)>1 or over and over[0]!=len(events)-1:raise FrameError('only one final overread supported')
    if len(times)>1:raise FrameError('multiple timestamp controls')
    if times and times[0]!=(len(events)-2 if over else len(events)-1):raise FrameError('timestamp is not terminal')
    if times and not any(e['kind'] in ['accel','gyro','gyro_accel'] for e in events[:times[0]]):raise FrameError('timestamp-only drain unqualified')
    if len(config)>1:raise FrameError('multiple configuration changes unqualified')
    return dict(observed_terminal_read_time_ticks=events[times[0]]['sensor_time_ticks'] if times else None,drain_observation='terminal_read_time_control' if times else 'overread_control' if over else 'no_empty_marker_observed',regular_frames=sum(e['kind'] in ['accel','gyro','gyro_accel'] for e in events))
