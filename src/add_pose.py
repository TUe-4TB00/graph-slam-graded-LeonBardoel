import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.1, 0.1, 0.05])
)

ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.2, 0.2, 0.1])
)

MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.05, 0.1])
)

def add_pose(graph, initial_estimate):

    # Define odometry measurement
    odometry = gtsam.Pose2(2.0, 0.0, math.pi / 2)

    # Add odometry factor between X3 and X4
    graph.add(
        gtsam.BetweenFactorPose2(
            X(3),
            X(4),
            odometry,
            ODOMETRY_NOISE
        )
    )

    # Get previous pose
    pose3 = initial_estimate.atPose2(X(3))

    # Compose new pose
    pose4 = pose3.compose(odometry)

    # Insert X4 initial estimate
    initial_estimate.insert(X(4), pose4)

    return graph, initial_estimate