"""Reversible sidecars for the older M2 controller's global initialization layout."""
GYRO='    Gyro { name "body_gyro" }\n'
ACCEL='    Accelerometer { name "body_accel" }\n'
INIT=';imu.enable(8);gyro.enable(8)\n'
ADDED_INIT='from measurement import PassiveCapture\nfrom timed_shadow import TimedShadowAttitude\nfrom audit_reference import capture as capture_raw_reference\nraw_accel=robot.getDevice("body_accel");raw_accel.enable(8);raw_capture=PassiveCapture(raw_accel,gyro);shadow_attitude=TimedShadowAttitude()\n'
STREAM=" as destination_stream:"
NEW_STREAM=" as destination_stream,(OUT/'raw-imu.jsonl').open('w',encoding='utf-8') as raw_imu_stream,(OUT/'raw-imu-reference.jsonl').open('w',encoding='utf-8') as raw_reference_stream,(OUT/'imu-shadow.jsonl').open('w',encoding='utf-8') as shadow_stream:"
WRITER='        writer=None\n'
INITIAL="        raw_reference_stream.write(json.dumps(capture_raw_reference(body,robot.getTime()),allow_nan=False)+'\\n')\n"
TICK=';last_angles=dict(a)\n'
ADDED_TICK="""            raw_observation=raw_capture.read(t)
            raw_imu_stream.write(json.dumps(raw_observation,allow_nan=False)+'\\n')
            shadow_stream.write(json.dumps(shadow_attitude.update(raw_observation),allow_nan=False)+'\\n')
            raw_reference_stream.write(json.dumps(capture_raw_reference(body,t),allow_nan=False)+'\\n')
"""
START=';started=time.perf_counter()\n'
ADDED_START='    prefix_end_reached=False\n'
STOP="            if hard or success or (stage=='recovery' and u>=5) or t>=400:break\n"
ADDED_STOP='            if t>=60. and not hard and not success:prefix_end_reached=True;break\n'
REPORT="    (OUT/'report.json').write_text("
ADDED_REPORT="    report.update(research_prefix=True,prefix_end_reached=prefix_end_reached,declared_prefix_endpoint_s=60.,original_controller_success=success)\n"

def controller(text):
    for n in [INIT,STREAM,WRITER,TICK,START,STOP,REPORT]:assert text.count(n)==1,n
    return text.replace(INIT,INIT+ADDED_INIT).replace(STREAM,NEW_STREAM).replace(WRITER,WRITER+INITIAL).replace(TICK,TICK+ADDED_TICK).replace(START,START+ADDED_START).replace(STOP,ADDED_STOP+STOP).replace(REPORT,ADDED_REPORT+REPORT)
def reverse_controller(text):
    for n in [ADDED_INIT,NEW_STREAM,INITIAL,ADDED_TICK,ADDED_START,ADDED_STOP,ADDED_REPORT]:assert text.count(n)==1,n
    return text.replace(ADDED_INIT,'').replace(NEW_STREAM,STREAM).replace(INITIAL,'').replace(ADDED_TICK,'').replace(ADDED_START,'').replace(ADDED_STOP,'').replace(ADDED_REPORT,'')
def world(text):
    assert text.count(GYRO)==1 and ACCEL not in text;return text.replace(GYRO,GYRO+ACCEL)
def reverse_world(text):
    assert text.count(ACCEL)==1;return text.replace(ACCEL,'')
