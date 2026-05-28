import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    #doing the trig and maths on paper, for the coordinates system we should have the following displacement/rotations
    dx = math.sqrt(2)
    dy = math.sqrt(2)
    dtheta = math.pi / 2
    odometry_delta = gtsam.Pose2(dx, dy, dtheta)
    #the position wished for X4 and the stiffness of the link (X3-X4) is added to link between X3-X4
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), odometry_delta, ODOMETRY_NOISE))
    #in assigment, we know X3 is at (4,0). Now for X4 should be X3+ our delta gives exact true X4
    perfect_x4_guess = gtsam.Pose2(4.0 + dx, 0.0 + dy, dtheta)
    initial_estimate.insert(X(4), perfect_x4_guess)
    
    return graph, initial_estimate