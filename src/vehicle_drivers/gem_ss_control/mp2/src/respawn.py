import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import os
import sys

def spawn_car(model_name, urdf_path, x, y, z, roll, pitch, yaw):
    # Initialize ROS node
    rospy.init_node('spawn_gem_e2', anonymous=True)

    # Wait for the spawn service
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        # Convert roll, pitch, yaw to quaternion
        from tf.transformations import quaternion_from_euler
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        # Read the URDF content
        if not os.path.exists(urdf_path):
            rospy.logerr(f"URDF file not found: {urdf_path}")
            sys.exit(1)

        with open(urdf_path, 'r') as urdf_file:
            urdf_content = urdf_file.read()

        # Prepare the pose
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        # Call the spawn service
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        spawn_model(
            model_name=model_name,
            model_xml=urdf_content,
            robot_namespace="/e2",
            initial_pose=pose,
            reference_frame="world"
        )
        rospy.loginfo(f"Model '{model_name}' spawned successfully at ({x}, {y}, {z}).")

    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn model '{model_name}': {e}")

if __name__ == "__main__":
    # Replace with your URDF path
    urdf_file_path = os.popen("rospack find gem_description").read().strip() + "/urdf/gem_e2.urdf.xacro"

    # Spawn parameters (adjust as needed)
    model_name = "gem_e4"
    spawn_x = 0.0
    spawn_y = -98.0
    spawn_z = 1.0
    spawn_roll = 0.0
    spawn_pitch = 0.0
    spawn_yaw = 0.0

    spawn_car(model_name, urdf_file_path, spawn_x, spawn_y, spawn_z, spawn_roll, spawn_pitch, spawn_yaw)
