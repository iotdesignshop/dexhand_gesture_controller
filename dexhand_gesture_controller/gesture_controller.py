from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import threading
from std_msgs.msg import String

class GestureController(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('gesture_controller')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # Subscribe to gesture messages
        self.gesture_sub = self.create_subscription(String, 'dexhand_gesture', self.gesture_callback, 10)
        self.extension_sub = self.create_subscription(String, 'dexhand_finger_extension', self.extension_callback, 10)

        # Hardware publisher
        self.hw_pub = self.create_publisher(String, 'dexhand_hw_command', 10)

        # Run animation loop @ 30 Hz
        timer_period = 0.033 # seconds
        self.timer = self.create_timer(timer_period, self.animate_and_publish)

        # Initialize a base message which we will re-use for each frame
        self.NUM_JOINTS = 24
        self.joint_state = JointState()
        self.joint_state.name=[
                    'wrist_pitch_lower',
                    'wrist_pitch_upper',
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

        # Utilities to make it easy to work with the fingers
        self.fingers = ['index', 'middle', 'ring', 'pinky', 'thumb']
        self.finger_joints = ['yaw', 'pitch', 'knuckle', 'tip']
        self.finger_joint_table = [
            ['index_yaw', 'index_pitch', 'index_knuckle', 'index_tip'],
            ['middle_yaw', 'middle_pitch', 'middle_knuckle', 'middle_tip'],
            ['ring_yaw', 'ring_pitch', 'ring_knuckle', 'ring_tip'],
            ['pinky_yaw', 'pinky_pitch', 'pinky_knuckle', 'pinky_tip'],
            ['thumb_yaw', 'thumb_pitch', 'thumb_knuckle', 'thumb_tip']
        ]
        self.finger_range_table = [
            [0.0, 75.0, 75.0, 75.0],    # index
            [0.0, 75.0, 75.0, 75.0],    # middle
            [0.0, 75.0, 75.0, 75.0],    # ring
            [0.0, 75.0, 75.0, 75.0],    # pinky
            [0.0, 60.0, 60.0, 60.0]]    # thumb


        # Intent lookup table - maps intent strings to functions
        self.intent_table = {
            "default": self.set_default_pose,
            "reset": self.set_default_pose,
            "wave": self.set_default_pose,
            "fist": self.set_fist_pose,
            "grab": self.set_grab_pose,
            "peace": self.set_peace_pose,
            "horns": self.set_horns_pose,
            "shaka": self.set_shaka_pose,
            "point": self.set_index_point_pose

        }

        # Really simple animation system to simulate servo motors moving
        # Basically, the target position, and a rotational velocity. 
        # Move at the velocity toward the target position
        self.joint_state_animation_targets = [0.0] * self.NUM_JOINTS
        self.joint_state_animation_velocities = [0.0] * self.NUM_JOINTS
        self.ROTATIONAL_VELOCITY = 0.05
        self.isAnimating = False
        
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

        except KeyboardInterrupt:
            pass

    # Sets the target position for a joint
    def set_joint_target_degrees(self, joint_name, target_position):
        if joint_name in self.joint_state.name:
            joint_index = self.joint_state.name.index(joint_name)
            self.joint_state_animation_targets[joint_index] = target_position * pi / 180.0

            # Set the velocity to move toward the target
            if (self.joint_state_animation_targets[joint_index] > self.joint_state.position[joint_index]):
                self.joint_state_animation_velocities[joint_index] = self.ROTATIONAL_VELOCITY
            else:
                self.joint_state_animation_velocities[joint_index] = -self.ROTATIONAL_VELOCITY
        else:
            self.get_logger().warn("{0} is not a valid joint name".format(joint_name))
    

    # Performs a single frame of animation and publishes the joint state
    def animate_and_publish(self):

        self.isAnimating = False
    
        # Animate all joints
        for i in range(self.NUM_JOINTS):
            # Calculate the difference between the target and current position
            diff = self.joint_state_animation_targets[i] - self.joint_state.position[i]
            # If the difference is greater than the rotational velocity, move the joint
            if abs(diff) > self.ROTATIONAL_VELOCITY:
                self.joint_state.position[i] += self.joint_state_animation_velocities[i]
                self.isAnimating = True # We have at least one joint moving
            # Otherwise, we're close enough to the target, so just set the position
            else:
                self.joint_state.position[i] = self.joint_state_animation_targets[i]
                self.joint_state_animation_velocities[i] = 0.0

        # Timestamp
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        
        # Publish the new joint state
        self.joint_pub.publish(self.joint_state)


    # Set default pose
    def set_default_pose(self):

        # All joints move to 0 degrees
        for joint_name in self.joint_state.name:
            self.set_joint_target_degrees(joint_name, 0.0)

    # Fist
    def set_fist_pose(self):
        self.set_default_pose()

        # Close the finger extensions
        for finger in self.fingers:
            self.set_finger_extension(finger, 0.0)

        # Rotate the yaw on the thumb to rotate the thumb around the palm
        self.set_finger_extension('thumb', 0.4)
        self.set_joint_target_degrees('thumb_yaw', 45.0)


    # Grab
    def set_grab_pose(self):
        self.set_default_pose()

        # Close the finger extensions partially
        for finger in self.fingers:
            self.set_finger_extension(finger, 0.45)

        # Rotate the yaw on the thumb to rotate the thumb around the palm
        self.set_joint_target_degrees('thumb_yaw', 40.0)

    # Peace
    def set_peace_pose(self):
        self.set_fist_pose()

        # Extend the index and middle fingers
        self.set_finger_extension('index', 1.0)
        self.set_finger_extension('middle', 1.0)

        # Spread the index and middle fingers
        self.set_joint_target_degrees('index_yaw', 15.0)
        self.set_joint_target_degrees('middle_yaw', -15.0)

    # Horns
    def set_horns_pose(self):
        self.set_fist_pose()

        # Extend the index and pinky fingers
        self.set_finger_extension('index', 1.0)
        self.set_finger_extension('pinky', 1.0)

    # Shaka
    def set_shaka_pose(self):
        self.set_default_pose()

        # Close all fingers except pinky and thumb
        for finger in self.fingers:
            if finger != 'pinky' and finger != 'thumb':
                self.set_finger_extension(finger, 0.0)

        # Angle pinky outward
        self.set_joint_target_degrees('pinky_yaw', -20.0)


    # Index finger point
    def set_index_point_pose(self):
        self.set_fist_pose()

        # Extend the index finger
        self.set_finger_extension('index', 1.0)

        

    # Sets the extension of a finger by name. 0.0 is closed (to fist) and 1.0 is open (straight)
    def set_finger_extension(self, finger_name, extension):
        if finger_name in self.fingers:
            finger_index = self.fingers.index(finger_name)

            # Clamp extension range from 0-1
            extension = min(max(extension, 0.0), 1.0)

            # Scale the range based on the extension value for each joint except yaw
            for i in range(1, len(self.finger_joints)):
                joint_name = self.finger_joint_table[finger_index][i]
                self.set_joint_target_degrees(joint_name, (1.0-extension) * self.finger_range_table[finger_index][i])
            
        else:
            self.get_logger().warn("{0} is not a valid finger name".format(finger_name))

    # Gesture message handler
    def gesture_callback(self, msg):
        self.get_logger().info('Received gesture: "%s"' % msg.data)

        # Look up the intent in the table and call the function
        if msg.data in self.intent_table:
            # Call sim function
            self.intent_table[msg.data]()

            # Publish the message to the hardware
            hw_msg = String()
            hw_msg.data = "gesture:{0}".format(msg.data)
            self.hw_pub.publish(hw_msg)

        else:
            self.get_logger().warn("{0} is not a known gesture".format(msg.data))

            
    # Finger extension message handler
    def extension_callback(self, msg):
        self.get_logger().info('Received finger extension: "%s"' % msg.data)

        # Parse the finger name and extension value from the message
        finger_name, extension = msg.data.split(':')
        extension = float(extension)

        # Set the finger extension in simulation
        if (finger_name in self.fingers):
            # Process sim finger
            self.set_finger_extension(finger_name, extension)

            # Hardware publisher operates in 0-100 range, so multiply by 100 and convert to int
            extension = int(extension * 100)

            # Hardware publisher user finger index and not name
            finger_index = self.fingers.index(finger_name)

            # Publish the message
            hw_msg = String()
            hw_msg.data = "fingerextension:{0}:{1}".format(finger_index, extension)
            self.hw_pub.publish(hw_msg)

        else:
            self.get_logger().warn("{0} is not a valid finger name".format(finger_name))
        
        
def main():
    node = GestureController()

if __name__ == '__main__':
    main()