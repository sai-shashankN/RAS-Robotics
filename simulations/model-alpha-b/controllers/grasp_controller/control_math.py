"""Frame conventions: x forward, y left, z up; positive heading is left."""

import math


def finger_angles(p):
    """Palm-local radial placement; legacy files default to three fingers."""
    return tuple(p.get('finger_angles_rad', (0, 2*math.pi/3, 4*math.pi/3)))


def wheel_speeds(forward, yaw_rate, radius, track):
    """Return left/right rolling speeds in rad/s, separate from balance correction."""
    if radius <= 0 or track <= 0:
        raise ValueError("Wheel radius and track must be positive.")
    return ((forward - yaw_rate * track / 2) / radius,
            (forward + yaw_rate * track / 2) / radius)


def saturate_pair(left, right, limit):
    """Scale together to preserve the requested curvature when saturated."""
    scale = max(1.0, abs(left) / limit, abs(right) / limit)
    return left / scale, right / scale


def palm_position(p, pose):
    """Analytic palm center, in the body frame, for checking the scene's joint axes."""
    q, e = pose[p["arm_base_joint"]], pose["elbow"]
    radius = p["sleeve_outer_radius"] if p["has_sleeve"] else 0.0
    mount = p["arm_mount_offset"]
    neck = p.get('wrist_standoff', 0.0)
    claw_angle = q + e + pose.get('wrist', 0.0)
    return (mount[0] + (radius + p["upper_length"]) * math.sin(q)
            + p["forearm_length"] * math.sin(q + e) + neck * math.sin(claw_angle),
            mount[1],
            mount[2] + (radius + p["upper_length"]) * math.cos(q)
            + p["forearm_length"] * math.cos(q + e) + neck * math.cos(claw_angle))


def finger_tip_position(p, pose, index, proximal, distal):
    """Distal endpoint in the body frame, including each finger's radial axis."""
    phi = finger_angles(p)[index]
    radial = (.018 - p['finger_proximal_length']*math.sin(proximal)
              - p['finger_distal_length']*math.sin(proximal+distal))
    height = (.009 + p['finger_proximal_length']*math.cos(proximal)
              + p['finger_distal_length']*math.cos(proximal+distal))
    x, y = radial*math.cos(phi), radial*math.sin(phi)
    angle = pose[p['arm_base_joint']]+pose['elbow']+pose['wrist']
    palm = palm_position(p, pose)
    return (palm[0]+x*math.cos(angle)+height*math.sin(angle), palm[1]+y,
            palm[2]-x*math.sin(angle)+height*math.cos(angle))
