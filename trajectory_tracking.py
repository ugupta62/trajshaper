import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
import yaml
import utils

class TrajectoryTracker():
    def __init__(self, trajectory, time_step=0.5):

        self.time_step = time_step
        self.robot_pose = Odometry()  # odometry message can store both position and velocity
        self.actual_trajectory = np.zeros((0, trajectory.shape[1]))
        # publisher
        self.pub_nav = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        # subscribers (Odometry is not that accurate)
        rospy.Subscriber('/mavros/local_position/pose',
                         PoseStamped, self.current_pose,  queue_size=1)
        rospy.Subscriber('/mavros/local_position/velocity_local',
                         TwistStamped, self.current_vel,  queue_size=1)


        # define waypoints
        self.trajectory = trajectory
        self.index = 0 # initial waypoint index

    def current_pose(self, data):
        """
        The function updates the position of a robot's pose based on input data.
        
        :param data: It is a variable that contains the current pose data of the robot. It is a PoseStamped
        message that includes information such as the position and orientation of the robot in 3D
        space. The code is using this data to update the position of the robot in the `robot_pose` variable
        """
        self.robot_pose.pose.pose.position.x = data.pose.position.x
        self.robot_pose.pose.pose.position.y = data.pose.position.y
        self.robot_pose.pose.pose.position.z = data.pose.position.z

    def current_vel(self, data):
        """
        The function updates the current velocity of a robot and publishes its trajectory based on a time
        step and a navigation parameter.
        
        :param data: The "data" parameter is a variable that contains information about the current velocity
        of the robot. It is likely a TwistStamped that includes information such as linear and angular
        velocity. The function is using this information to update the robot's pose and to control its
        trajectory
        """
        self.robot_pose.twist.twist.linear.x = data.twist.linear.x
        self.robot_pose.twist.twist.linear.y = data.twist.linear.y
        self.robot_pose.twist.twist.linear.z = data.twist.linear.z
        
        is_navigation_started = rospy.get_param("/is_navigation_started")
        # time step 
        r = rospy.Rate(1/self.time_step) # should be >= 2hz
        r.sleep()
        self.pubRobot(self.trajectory[self.index])
        if is_navigation_started == False:
            self.index = 0
        else:
            self.index += 1
        if self.index >= len(self.trajectory):
            self.index = len(self.trajectory) - 1

    def pubRobot(self, waypoint):
        """
        The function `pubRobot` publishes a PositionTarget message to control the robot's position and
        velocity towards a given waypoint. It also keep a track of the actual trajectory followed by the quadrotor 
        
        :param waypoint: The input parameter to the `pubRobot` function, which is a tuple containing the
        desired x, y, z position and velocity of the robot
        """
        x, y, z, v = waypoint
        pose = PositionTarget()
        pose.coordinate_frame = 1  # FRAME LOCAL_NED
        pose.type_mask = 3520  # AFX+AFY+AFZ+YAW_RATE
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        root3 = np.sqrt(3)
        pose.velocity.x = v/root3
        pose.velocity.y = v/root3
        pose.velocity.z = v/root3
        pose.yaw = 0

        robot_cur_vel = np.sqrt(self.robot_pose.twist.twist.linear.x**2 + self.robot_pose.twist.twist.linear.y**2 + self.robot_pose.twist.twist.linear.z**2)
        
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2}, v: {3} | current position x: {4:.2f}, y: {5:.2f}, z: {6:.2f}, v: {7:.2f}".
            format(x, y, z, v, self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y , self.robot_pose.pose.pose.position.z, robot_cur_vel))
        
        self.actual_trajectory = np.vstack((self.actual_trajectory, np.array([x,y,z,v])))
        self.pub_nav.publish(pose)


if __name__ == '__main__':
    rospy.init_node('follow_waypoints', anonymous=True)

    with open("config.yaml") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    input_traj, output_traj = utils.get_trajectory(config['traj_num'])
    input_traj[:,:-1] = input_traj[:,:-1] + 1
    # starting point has zero velocity
    input_traj[0,-1] = 0
    rospy.set_param('/is_navigation_started', False)
    traj_tracker = TrajectoryTracker(input_traj, time_step=config['time_step'])
    rospy.spin()
    
    # save trajectory
    utils.save_trajectory(traj_tracker.actual_trajectory)

                  
   
