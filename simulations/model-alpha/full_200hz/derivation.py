from prefix200_derivation import world,reverse_world,native,reverse_native
INIT='OWNED_ACCEL_LOSS=False\n'
ADDED='from full200 import FullObservedAlpha200\nfull200=FullObservedAlpha200(robot,body,OUT,False)\n'
LOOP='        while robot.step(8)!=-1:\n'
NEW_LOOP='        while robot.step(1)!=-1:\n            full200.tick(robot.getTime())\n            if round(robot.getTime()*1000)%8:continue\n'
REPORT='    report=dict(case=CASE,'
CLOSE='    full200.close(robot.getTime())\n'
def controller(t):
    for s in [INIT,LOOP,REPORT]:assert t.count(s)==1,s
    assert 'prefix_end_reached' not in t
    return t.replace(INIT,INIT+ADDED).replace(LOOP,NEW_LOOP).replace(REPORT,CLOSE+REPORT)
def reverse(t):
    for s in [ADDED,NEW_LOOP,CLOSE]:assert t.count(s)==1,s
    return t.replace(ADDED,'').replace(NEW_LOOP,LOOP).replace(CLOSE,'')
