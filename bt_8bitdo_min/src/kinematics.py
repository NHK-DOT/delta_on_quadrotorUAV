from math import acos, asin, atan2, cos, sin, sqrt

from config import robot_params


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_mul(a, scalar):
    return [a[0] * scalar, a[1] * scalar, a[2] * scalar]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def norm(a):
    return sqrt(dot(a, a))


def local_to_global(x_local, y_local, z_local, servo_angle):
    return [
        x_local * cos(servo_angle) + y_local * sin(servo_angle),
        -x_local * sin(servo_angle) + y_local * cos(servo_angle),
        z_local,
    ]


def platform_offset(radius, servo_angle):
    return [radius * cos(servo_angle), -radius * sin(servo_angle), 0.0]


def wrap_to_pi(value):
    return atan2(sin(value), cos(value))


def inverse_kinematics(x, y, z, params=None):
    params = params or robot_params()
    results = []
    for i in range(3):
        results.append(inverse_kinematics_single(x, y, z, params.servo_distribution[i], params))
    valid = all(item[1] for item in results)
    if not valid:
        return [0.0, 0.0, 0.0], False
    return [item[0] for item in results], True


def inverse_kinematics_single(xt, yt, zt, servo_angle, params):
    zt = zt - params.servo_offset_z
    x_rot = xt * cos(servo_angle) - yt * sin(servo_angle)
    y_rot = xt * sin(servo_angle) + yt * cos(servo_angle)
    arm_end_x = x_rot + params.l3
    under = params.l2 ** 2 - y_rot ** 2
    if under < 0:
        return 0.0, False
    l2p = sqrt(under)
    l2p_angle = asin(max(-1.0, min(1.0, y_rot / params.l2)))
    if abs(l2p_angle) >= params.ball_joint_angle_limit:
        return 0.0, False
    ext = sqrt(zt ** 2 + (params.servo_offset_x - arm_end_x) ** 2)
    if ext <= l2p - params.l1 or ext >= params.l1 + l2p:
        return 0.0, False
    cos_phi = (params.l1 ** 2 + ext ** 2 - l2p ** 2) / (2.0 * params.l1 * ext)
    cos_phi = max(-1.0, min(1.0, cos_phi))
    phi = acos(cos_phi)
    omega = atan2(zt, params.servo_offset_x - arm_end_x)
    theta = phi + omega
    if params.servo_angle_min <= theta <= params.servo_angle_max:
        return theta, True
    return 0.0, False


def forward_kinematics(theta1, theta2, theta3, params=None):
    params = params or robot_params()
    theta = [theta1, theta2, theta3]
    centers = []
    for i, servo_angle in enumerate(params.servo_distribution):
        elbow = local_to_global(
            params.servo_offset_x - params.l1 * cos(theta[i]),
            0.0,
            params.servo_offset_z + params.l1 * sin(theta[i]),
            servo_angle,
        )
        centers.append(vec_sub(elbow, platform_offset(params.l3, servo_angle)))
    point, ok = intersect_three_spheres(centers, params.l2)
    if not ok:
        return [0.0, 0.0, 0.0], False
    theta_check, ik_ok = inverse_kinematics(point[0], point[1], point[2], params)
    if not ik_ok:
        return [0.0, 0.0, 0.0], False
    max_error = max(abs(wrap_to_pi(theta_check[i] - theta[i])) for i in range(3))
    if max_error > 1e-4:
        return [0.0, 0.0, 0.0], False
    return point, True


def intersect_three_spheres(centers, radius):
    p1, p2, p3 = centers
    ex = vec_sub(p2, p1)
    d = norm(ex)
    if d < 1e-9:
        return [0.0, 0.0, 0.0], False
    ex = vec_mul(ex, 1.0 / d)
    p3_minus_p1 = vec_sub(p3, p1)
    i_val = dot(ex, p3_minus_p1)
    temp = vec_sub(p3_minus_p1, vec_mul(ex, i_val))
    temp_norm = norm(temp)
    if temp_norm < 1e-9:
        return [0.0, 0.0, 0.0], False
    ey = vec_mul(temp, 1.0 / temp_norm)
    ez = cross(ex, ey)
    j_val = dot(ey, p3_minus_p1)
    if abs(j_val) < 1e-9:
        return [0.0, 0.0, 0.0], False
    x_coord = d / 2.0
    y_coord = (i_val ** 2 + j_val ** 2 - 2.0 * i_val * x_coord) / (2.0 * j_val)
    z_sq = radius ** 2 - x_coord ** 2 - y_coord ** 2
    if z_sq < -1e-6:
        return [0.0, 0.0, 0.0], False
    z_coord = sqrt(max(z_sq, 0.0))
    base = vec_add(p1, vec_add(vec_mul(ex, x_coord), vec_mul(ey, y_coord)))
    candidate1 = vec_add(base, vec_mul(ez, z_coord))
    candidate2 = vec_sub(base, vec_mul(ez, z_coord))
    point = candidate1 if candidate1[2] >= candidate2[2] else candidate2
    return point, True


class DeltaRobot(object):
    def __init__(self, params=None):
        self.params = params or robot_params()
        self.current_position = list(self.params.home_position)
        self.current_theta, _ = inverse_kinematics(
            self.current_position[0],
            self.current_position[1],
            self.current_position[2],
            self.params,
        )

    def compute_ik(self, x, y, z):
        return inverse_kinematics(x, y, z, self.params)

    def compute_fk(self, theta1, theta2, theta3):
        return forward_kinematics(theta1, theta2, theta3, self.params)

    def get_workspace_bounds(self):
        return {
            "x_min": -self.params.workspace_xy_max,
            "x_max": self.params.workspace_xy_max,
            "y_min": -self.params.workspace_xy_max,
            "y_max": self.params.workspace_xy_max,
            "z_min": self.params.workspace_z_min,
            "z_max": self.params.workspace_z_max,
        }
