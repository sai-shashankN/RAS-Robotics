"""Reversible additions to the reviewed continuous sixty-second source."""
OLD_INIT='raw_accel=robot.getDevice("body_accel");raw_accel.enable(8);raw_capture=PassiveCapture(raw_accel,gyro);shadow_attitude=TimedShadowAttitude()\n'
NEW_INIT='from owner import AccelOwner\nfrom history import ApiHistory\nfrom consumer import HistoryConsumer\nraw_accel=robot.getDevice("body_accel");accel_owner=AccelOwner(raw_accel,gyro,robot.getTime());raw_capture=PassiveCapture(raw_accel,gyro);shadow_attitude=TimedShadowAttitude();api_history=ApiHistory(robot.getTime());owned_consumer=HistoryConsumer()\n'
STREAM=' as shadow_stream:'
NEW_STREAM=" as shadow_stream,(OUT/'imu-device-periods.jsonl').open('w',encoding='utf-8') as imu_period_stream,(OUT/'imu-device-actions.jsonl').open('w',encoding='utf-8') as imu_action_stream,(OUT/'imu-owner-history.jsonl').open('w',encoding='utf-8') as owner_history_stream,(OUT/'imu-owned-shadow.jsonl').open('w',encoding='utf-8') as owned_shadow_stream:"
INITIAL="        raw_reference_stream.write(json.dumps(capture_raw_reference(body,robot.getTime()),allow_nan=False)+'\\n')\n"
INIT_JOURNAL="        (OUT/'imu-owner-init.json').write_text(json.dumps(accel_owner.initial_action,allow_nan=False),encoding='utf-8')\n"
ANCHOR="            raw_reference_stream.write(json.dumps(capture_raw_reference(body,t),allow_nan=False)+'\\n')\n"
BLOCK="""            owner_period=accel_owner.periods(t)
            owner_actions=accel_owner.after_read(t,OWNED_ACCEL_LOSS)
            owner_metadata=api_history.update(raw_observation,owner_period,owner_actions)
            imu_period_stream.write(json.dumps(owner_period,allow_nan=False)+'\\n')
            for owner_event in owner_actions:imu_action_stream.write(json.dumps(owner_event,allow_nan=False)+'\\n')
            owner_history_stream.write(json.dumps(owner_metadata,allow_nan=False)+'\\n')
            owned_shadow_stream.write(json.dumps(owned_consumer.update(raw_observation,owner_metadata),allow_nan=False)+'\\n')
"""
def controller(t,loss):
    for a in [OLD_INIT,STREAM,INITIAL,ANCHOR]:assert t.count(a)==1,a
    return t.replace(OLD_INIT,NEW_INIT+'OWNED_ACCEL_LOSS='+repr(loss)+'\n').replace(STREAM,NEW_STREAM).replace(INITIAL,INITIAL+INIT_JOURNAL).replace(ANCHOR,ANCHOR+BLOCK)
def reverse(t,loss):
    for a in [NEW_INIT,NEW_STREAM,INIT_JOURNAL,BLOCK]:assert t.count(a)==1,a
    return t.replace(NEW_INIT+'OWNED_ACCEL_LOSS='+repr(loss)+'\n',OLD_INIT).replace(NEW_STREAM,STREAM).replace(INIT_JOURNAL,'').replace(BLOCK,'')
