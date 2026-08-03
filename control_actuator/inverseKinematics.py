import numpy as np


class StewartPlatformIK:
    """Inverse kinematics for a six-actuator Stewart platform.

    Coordinates and distances are in metres. ``translation`` is an offset from
    the neutral platform centre and ``quaternion`` uses ROS ordering (x, y, z,
    w). The result is actuator extension from the retracted length in metres.
    """

    def __init__(self):
        self.home_position = np.array([0.0, 0.0, 0.5628])
        self.retracted_length = 0.570
        self.stroke_length = 0.202

        # Base attachment points expressed in the fixed base frame.
        self.base_points = np.array([
            [0.0440, -0.1642, -0.1642, 0.0440, 0.1202, 0.1202],
            [0.1642, 0.0440, -0.0440, -0.1642, -0.1202, 0.1202],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ])

        # Platform attachment points expressed relative to its centre.
        self.platform_points = np.array([
            [-0.0391, -0.0878, -0.0878, -0.0391, 0.1269, 0.1269],
            [0.1240, 0.0959, -0.0959, -0.1240, -0.0281, 0.0281],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ])

    @staticmethod
    def quaternion_to_matrix(quaternion):
        quaternion = np.asarray(quaternion, dtype=float)
        if quaternion.shape != (4,):
            raise ValueError('Quaternion must contain x, y, z, and w')
        if not np.all(np.isfinite(quaternion)):
            raise ValueError('Quaternion must contain only finite values')

        magnitude = np.linalg.norm(quaternion)
        if magnitude < 1e-12:
            raise ValueError('Quaternion magnitude must be nonzero')
        if not np.isclose(magnitude, 1.0, atol=1e-3):
            raise ValueError(
                'Quaternion must have unit magnitude; include a valid w value'
            )

        x, y, z, w = quaternion / magnitude
        return np.array([
            [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w),
             2.0*(x*z + y*w)],
            [2.0*(x*y + z*w), 1.0 - 2.0*(x*x + z*z),
             2.0*(y*z - x*w)],
            [2.0*(x*z - y*w), 2.0*(y*z + x*w),
             1.0 - 2.0*(x*x + y*y)],
        ])

    def leg_lengths(self, translation, quaternion):
        translation = np.asarray(translation, dtype=float)
        if translation.shape != (3,):
            raise ValueError('Translation must contain x, y, and z')
        if not np.all(np.isfinite(translation)):
            raise ValueError('Translation must contain only finite values')

        rotation = self.quaternion_to_matrix(quaternion)
        platform_center = self.home_position + translation

        # Vector from each base joint to its corresponding platform joint:
        # L_i = T + R * P_i - B_i
        leg_vectors = (
            platform_center[:, np.newaxis]
            + rotation @ self.platform_points
            - self.base_points
        )
        return np.linalg.norm(leg_vectors, axis=0)

    def solve(self, translation, quaternion):
        return self.leg_lengths(translation, quaternion) - self.retracted_length


# Backward-compatible name for other code in this package.
inv_kinematics = StewartPlatformIK
