"""Exact reversible scheduler/device/projection additions to sealed219 source."""
WORLD='    Gyro { name "body_gyro" }\n'
DEVICES='    Accelerometer { name "raw200_accel" }\n    Gyro { name "raw200_gyro" }\n'
INIT='OWNED_ACCEL_LOSS=False\n'
IMPORT='from projection import project as project_pitch\nfrom sidecar200 import Alpha200\nalpha200=Alpha200(robot,body,OUT,RESTART_VALUE)\n'
STREAM=' as owned_shadow_stream:'
NEW_STREAM=" as owned_shadow_stream,(OUT/'imu-pitch-projection.jsonl').open('w',encoding='utf-8') as pitch_projection_stream:"
OLD="            owned_shadow_stream.write(json.dumps(owned_consumer.update(raw_observation,owner_metadata),allow_nan=False)+'\\n')\n"
NEW="""            owned_output=owned_consumer.update(raw_observation,owner_metadata)
            owned_shadow_stream.write(json.dumps(owned_output,allow_nan=False)+'\\n')
            pitch_projection_stream.write(json.dumps(project_pitch(owned_output,raw_observation),allow_nan=False)+'\\n')
"""
LOOP='        while robot.step(8)!=-1:\n'
NEW_LOOP='        while robot.step(1)!=-1:\n            alpha200.tick(robot.getTime())\n            if round(robot.getTime()*1000)%8:continue\n'
REPORT="    report=dict(case=CASE,"
CLOSE='    alpha200.close(robot.getTime())\n'
C_INCLUDE='#include "contact_observer.inc"\n'
C_ADDED='#include "raw200_native.inc"\n'
C_INIT='  ct_init(dir);\n'
C_STEP='double ms=dWebotsGetTime();ct_begin(ms);'
C_END='void webots_physics_cleanup(void){'

def controller(t,restart):
    for a in [INIT,STREAM,OLD,LOOP,REPORT]:assert t.count(a)==1,a
    return t.replace(INIT,INIT+IMPORT.replace('RESTART_VALUE',repr(restart))).replace(STREAM,NEW_STREAM).replace(OLD,NEW).replace(LOOP,NEW_LOOP).replace(REPORT,CLOSE+REPORT)
def reverse_controller(t,restart):
    for a in [IMPORT.replace('RESTART_VALUE',repr(restart)),NEW_STREAM,NEW,NEW_LOOP,CLOSE]:assert t.count(a)==1,a
    return t.replace(IMPORT.replace('RESTART_VALUE',repr(restart)),'').replace(NEW_STREAM,STREAM).replace(NEW,OLD).replace(NEW_LOOP,LOOP).replace(CLOSE,'')
def world(t):
    assert t.count(WORLD)==1 and DEVICES not in t
    return t.replace(WORLD,WORLD+DEVICES)
def reverse_world(t):
    assert t.count(DEVICES)==1
    return t.replace(DEVICES,'')
def native(t):
    for a in [C_INCLUDE,C_INIT,C_STEP,C_END]:assert t.count(a)==1,a
    return t.replace(C_INCLUDE,C_INCLUDE+C_ADDED).replace(C_INIT,C_INIT+'  raw200_init(dir);\n').replace(C_STEP,'double ms=dWebotsGetTime();raw200_log(ms);ct_begin(ms);').replace(C_END,C_END+'raw200_cleanup();')
def reverse_native(t):
    return t.replace(C_ADDED,'').replace('  raw200_init(dir);\n','').replace('double ms=dWebotsGetTime();raw200_log(ms);ct_begin(ms);',C_STEP).replace(C_END+'raw200_cleanup();',C_END)
