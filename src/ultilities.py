import numpy as np
from scipy.optimize import minimize
from geometry_msgs.msg import Pose


def objective_func(theta, target_pose, start_pose, offset):
    ## transfer the ROS Pose to numpy array
    start_coord = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
    ## the circle with offset
    target_coord= np.array([
        target_pose.position.x + np.sin(theta)*offset,
        target_pose.position.y + np.cos(theta)*offset,
        target_pose.position.z])
    
    return np.sum((target_coord - start_coord) ** 2)
    
    
    
def offset_pose_optimiser(target_pose, start_pose, offset):
    """
    Optimizes the angular offset (theta) to minimize the distance between 
    a fixed position (start_pose) and a target position on a circle defined 
    by the target_pose and offset.

    Parameters:
    ----------
    target_pose : geometry_msgs.msg.Pose
        The base position defining the circle's center.
    start_pose : geometry_msgs.msg.Pose
        The fixed position to minimize the distance from.
    offset : float
        The radius of the circle.

    Returns:
    -------
    optimal_theta : float
        The angle (in radians) that minimizes the distance.
    optimal_distance : float
        The minimum distance (squared) between the positions.
    result_pose : geometry_msgs.msg.Pose
        The optimized position on the circle.
    """
    # initialize gues
    theta_initial_guess =  np.pi
    bounds = [(0,2*np.pi)]
    # optimizer
    result = minimize(objective_func, theta_initial_guess, args= (target_pose, start_pose, offset), bounds=bounds, method='L-BFGS-B')
    optimal_theta = result.x[0]

    ## get the optimized pose
    result_pose = Pose()
    result_pose.position.x = target_pose.position.x + np.sin(optimal_theta)*offset
    result_pose.position.y = target_pose.position.y + np.cos(optimal_theta)*offset
    result_pose.position.z = target_pose.position.z
    
    result_pose.orientation.x = start_pose.orientation.x
    result_pose.orientation.y = start_pose.orientation.y
    result_pose.orientation.z = start_pose.orientation.z
    result_pose.orientation.w = start_pose.orientation.w
    
    return optimal_theta, float(result.fun), result_pose


if __name__ == "__main__":
    
    fixed_pose = Pose()
    fixed_pose.position.x = 2.0
    fixed_pose.position.y = 1.0
    fixed_pose.position.z = 4.0
    fixed_pose.orientation.x = 1

    target_pose_base = Pose()
    target_pose_base.position.x = 2.0
    target_pose_base.position.y = 2.0
    target_pose_base.position.z = 4.0

    l = 1.0
    optimal_theta, optimal_distance, result_pose = offset_pose_optimiser(target_pose_base, fixed_pose, l)

    print("optimized theta:", optimal_theta)
    print("optimized distance:", optimal_distance)
    print("optimized offset pose:",result_pose)    