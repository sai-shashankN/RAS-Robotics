"""Privileged read-only witness, journal only; never a sensor or control input."""
def capture(body,time_s):
    return dict(audit_only=True,time_s=time_s,position=list(body.getPosition()),orientation=list(body.getOrientation()),velocity=list(body.getVelocity()))
