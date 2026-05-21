import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):

    robot_pose = result.atPose2(X(4))

    robot_x = robot_pose.x()
    robot_y = robot_pose.y()
    robot_theta = robot_pose.theta()

    landmark_x = 4.0
    landmark_y = 2.0

    dx = landmark_x - robot_x
    dy = landmark_y - robot_y

    rotation = (math.atan2(dy, dx) - robot_theta) / np.pi * 180
    distance = np.sqrt(dx**2 + dy**2)

    graph.add(
        gtsam.BearingRangeFactor2D(
            X(4),
            L(2),
            gtsam.Rot2.fromDegrees(rotation),
            distance,
            MEASUREMENT_NOISE
        )
    )

    return graph