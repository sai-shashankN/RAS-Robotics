"""Declared bounded reach, distinct from contact-conditioned preload."""
CAP=.105
FAST=.3
SLOW=.03
REACQUIRE=.004
SWITCH=-.01
def advance(q,touched,dt):
    if touched:return min(CAP,q+REACQUIRE*dt)
    if q<SWITCH-1e-12:return min(SWITCH,q+FAST*dt)
    return min(CAP,q+SLOW*dt)
