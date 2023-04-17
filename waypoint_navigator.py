import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry


class WaypointNavigator:
    def __init__(self, trajectory, time_step=0.5):

        self.time_step = time_step
        # publisher
        self.pub_nav = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        # subscribers (Odometry is not that accurate)
        rospy.Subscriber('/mavros/local_position/pose',
                         PoseStamped, self.current_pose,  queue_size=1)
        rospy.Subscriber('/mavros/local_position/velocity_local',
                         TwistStamped, self.current_vel,  queue_size=1)

        self.robot_pose = Odometry()  # odometry message can store both position and velocity

        # define waypoints
        self.trajectory = trajectory
        self.index = 0 # initial waypoint index

    def current_pose(self, data):
        self.robot_pose.pose.pose.position.x = data.pose.position.x
        self.robot_pose.pose.pose.position.y = data.pose.position.y
        self.robot_pose.pose.pose.position.z = data.pose.position.z

    def current_vel(self, data):
        self.robot_pose.twist.twist.linear.x = data.twist.linear.x
        self.robot_pose.twist.twist.linear.y = data.twist.linear.y
        self.robot_pose.twist.twist.linear.z = data.twist.linear.z
        
        # time step 
        r = rospy.Rate(1/self.time_step) # should be >= 2hz
        r.sleep()
        self.pubRobot(self.trajectory[self.index])
        self.index += 1
        if self.index >= len(self.trajectory):
            self.index = len(self.trajectory) - 1

    def pubRobot(self, waypoint):
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
        
        self.pub_nav.publish(pose)


if __name__ == '__main__':
    rospy.init_node('follow_waypoints', anonymous=True)
    initial_waypoint = [[1, 1, 1, 0],
                  [1.1, 1.1, 1.1, 0], [1.1, 1.1, 1.2, 0],
                  [1.2, 1.1, 1.2, 0], [1.3, 1.2, 1.2, 0]]
    waypoint_nav = WaypointNavigator(initial_waypoint)
    rospy.spin()
