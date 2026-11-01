"""Mass accounting matching the generator's lumped body/link COM assumptions.

Identical symmetric radial fingers cancel their radial COM components. This is a
design calculation, not a measured hardware model or strength calculation.
"""

import math
from control_math import finger_angles


def mass_properties(p, pose_name="folded", joint_angles=None):
    # Encoder angles can replace named geometry poses during arm movement.
    pose = p["poses"][pose_name] if joint_angles is None else joint_angles
    q, e, w = pose[p["arm_base_joint"]], pose["elbow"], pose["wrist"]
    r = p["sleeve_outer_radius"] if p["has_sleeve"] else 0.0
    mount = p["arm_mount_offset"]
    layout = finger_angles(p)
    if abs(sum(math.cos(a) for a in layout))>1e-9 or abs(sum(math.sin(a) for a in layout))>1e-9:
        raise ValueError('Aggregate finger COM requires a symmetric layout')
    count = len(layout)

    def extend(point, distance, angle):
        return [point[0] + distance * math.sin(angle), point[1], point[2] + distance * math.cos(angle)]

    elbow = extend(mount, r + p["upper_length"], q)
    palm = extend(elbow, p["forearm_length"], q + e)
    palm = extend(palm, p.get('wrist_standoff', 0.0), q + e + w)
    palm[0] -= p.get("wrist_forward_offset_m", 0.) * math.cos(q+e+w)
    palm[2] += p.get("wrist_forward_offset_m", 0.) * math.sin(q+e+w)
    proximal_z = 0.009 + p["finger_proximal_length"] / 2 * math.cos(pose["finger"])
    distal_z = (0.009 + p["finger_proximal_length"] * math.cos(pose["finger"])
                + p["finger_distal_length"] / 2 * math.cos(pose["finger"] + pose["distal"]))
    masses = [
        ("inner_body", p["body_mass"], [p.get("body_com_x", 0.0), 0.0, p["body_com_z"]]),
        ("base_and_upper", p["sleeve_upper_mass"], extend(mount, r + p["upper_length"] * 0.35, q)),
        ("forearm", p["forearm_mass"], extend(elbow, p["forearm_length"] / 2, q + e)),
        ("palm", p["palm_mass"], palm),
        ("proximal_fingers", count * p["finger_segment_mass"], extend(palm, proximal_z, q + e + w)),
        ("distal_fingers", count * p["finger_segment_mass"], extend(palm, distal_z, q + e + w)),
        ("wheels", 2 * p["wheel_mass"], [0.0, 0.0, 0.0]),
    ]
    total = sum(mass for _, mass, _ in masses)
    moments = [sum(mass * point[i] for _, mass, point in masses) for i in range(3)]
    arm_x_moment = sum(mass * point[0] for name, mass, point in masses if name not in ["inner_body", "wheels"])
    return {
        "pose": pose_name,
        "mass_kg": total,
        "com_body_m": [moment / total for moment in moments],
        "lean_to_align_com_rad": -math.atan2(moments[0], moments[2]),
        "body_com_x_for_level_folded_pose_m": -arm_x_moment / p["body_mass"],
        "parts": [{"name": name, "mass_kg": mass, "com_body_m": point} for name, mass, point in masses],
    }
