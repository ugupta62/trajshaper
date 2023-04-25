import rospy
import numpy as np
from visualization_msgs.msg import Marker

if __name__ == '__main__':
    marker = Marker()
    marker.header.frame_id ="world"
    marker.type = 10 # for meshes
    # marker.action = 0
    scale = 1000
    waypoints = np.array(rospy.get_param("/waypoints"))

    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    # origin of building is the average of waypoints in x and y axes
    marker.pose.position.x = np.average(waypoints[:, 0]) 
    marker.pose.position.y = np.average(waypoints[:, 1]) 
    marker.pose.position.z = 42.291306    # adjust the ground level accordingly

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.mesh_resource = "package://hector_gazebo_worlds/Media/models/TEP.dae"
    marker.mesh_use_embedded_materials = False   # Need this to use textures for mesh
    pub = rospy.Publisher("/marker", Marker, queue_size = 1)
    rospy.init_node('visualisation', anonymous=False) # intializes node
    while not rospy.is_shutdown():
        pub.publish(marker)