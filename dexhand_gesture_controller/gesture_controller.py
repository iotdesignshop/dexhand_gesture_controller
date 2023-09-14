from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class GestureController(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('gesture_controller')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0

        # Run loop @ 30 Hz
        loop_rate = self.create_rate(30)

        # Initialize a base message which we will re-use for each frame
        self.NUM_JOINTS = 23
        self.joint_state = JointState()
        self.joint_state.name=[
                    'wrist_pitch',
                    'wrist_yaw',
                    'index_yaw',
                    'middle_yaw',
                    'ring_yaw',
                    'pinky_yaw',
                    'index_pitch',
                    'index_knuckle',
                    'index_tip',
                    'middle_pitch',
                    'middle_knuckle',
                    'middle_tip',
                    'ring_pitch',
                    'ring_knuckle',
                    'ring_tip',
                    'pinky_pitch',
                    'pinky_knuckle',
                    'pinky_tip',
                    'thumb_yaw',
                    'thumb_roll',
                    'thumb_pitch',
                    'thumb_knuckle',
                    'thumb_tip']

        self.joint_state.position = [0.0] * self.NUM_JOINTS
        self.get_logger().info("{0} publishing {1} joint states".format(self.nodeName, self.NUM_JOINTS))
        self.get_logger().info("{0} joint names: {1}".format(self.nodeName, self.joint_state.name))
        self.get_logger().info("{0} joint positions: {1}".format(self.nodeName, self.joint_state.position))

        # Really simple animation system to simulate servo motors moving
        # Basically, the target position, and a rotational velocity. 
        # Move at the velocity toward the target position
        self.joint_state_animation_targets = [0.0] * self.NUM_JOINTS
        self.joint_state_animation_velocities = [0.0] * self.NUM_JOINTS
        self.ROTATIONAL_VELOCITY = 0.01

        
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # Animate and publish the joint state
                self.animate_and_publish()
                
                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


    # Performs a single frame of animation and publishes the joint state
    def animate_and_publish(self):
    
        # Animate all joints
        for i in range(self.NUM_JOINTS):
            # Calculate the difference between the target and current position
            diff = self.joint_state_animation_targets[i] - self.joint_state.position[i]
            # If the difference is greater than the rotational velocity, move the joint
            if abs(diff) > self.ROTATIONAL_VELOCITY:
                self.joint_state.position[i] += self.joint_state_animation_velocities[i]
            # Otherwise, we're close enough to the target, so just set the position
            else:
                self.joint_state.position[i] = self.joint_state_animation_targets[i]
                self.joint_state_animation_velocities[i] = 0.0
        


        # Timestamp
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        
        # Publish the new joint state
        self.joint_pub.publish(self.joint_state)

    

def main():
    node = GestureController()

if __name__ == '__main__':
    main()