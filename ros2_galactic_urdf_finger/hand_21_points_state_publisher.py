from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('hand_21_points_state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        tilt = 0.
        tinc = degree
        swivel = 0.
        angle = 0.
        height = 0.
        hinc = 0.005

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'wrist1'
        
        odom_trans0 = TransformStamped()
        odom_trans0.header.frame_id = 'wrist1'
        odom_trans0.child_frame_id = 'metacarpals1'
        
        odom_trans1 = TransformStamped()
        odom_trans1.header.frame_id = 'metacarpals1'
        odom_trans1.child_frame_id = 'proximal1'
        
        odom_trans2 = TransformStamped()
        odom_trans2.header.frame_id = 'proximal1'
        odom_trans2.child_frame_id = 'intermediate1'
        
        odom_trans3 = TransformStamped()
        odom_trans3.header.frame_id = 'intermediate1'
        odom_trans3.child_frame_id = 'distal1'
        
        odom_trans4 = TransformStamped()
        odom_trans4.header.frame_id = 'wrist1'
        odom_trans4.child_frame_id = 'thumb_cmc'
        
        odom_trans5 = TransformStamped()
        odom_trans5.header.frame_id = 'thumb_cmc'
        odom_trans5.child_frame_id = 'thumb_mcp'
        
        odom_trans6 = TransformStamped()
        odom_trans6.header.frame_id = 'thumb_mcp'
        odom_trans6.child_frame_id = 'thumb_ip'
        
        odom_trans7 = TransformStamped()
        odom_trans7.header.frame_id = 'thumb_ip'
        odom_trans7.child_frame_id = 'thumb_tip'
        
        odom_trans8 = TransformStamped()
        odom_trans8.header.frame_id = 'metacarpals1'
        odom_trans8.child_frame_id = 'middle_finger_mcp'
        
        odom_trans9 = TransformStamped()
        odom_trans9.header.frame_id = 'middle_finger_mcp'
        odom_trans9.child_frame_id = 'middle_finger_pip'
        
        odom_trans10 = TransformStamped()
        odom_trans10.header.frame_id = 'middle_finger_pip'
        odom_trans10.child_frame_id = 'middle_finger_dip'
        
        odom_trans11 = TransformStamped()
        odom_trans11.header.frame_id = 'middle_finger_dip'
        odom_trans11.child_frame_id = 'middle_finger_tip'
        
        odom_trans12 = TransformStamped()
        odom_trans12.header.frame_id = 'middle_finger_mcp'
        odom_trans12.child_frame_id = 'ring_finger_mcp'
        
        odom_trans13 = TransformStamped()
        odom_trans13.header.frame_id = 'ring_finger_mcp'
        odom_trans13.child_frame_id = 'ring_finger_pip'
        
        odom_trans14 = TransformStamped()
        odom_trans14.header.frame_id = 'ring_finger_pip'
        odom_trans14.child_frame_id = 'ring_finger_dip'
        
        odom_trans15 = TransformStamped()
        odom_trans15.header.frame_id = 'ring_finger_dip'
        odom_trans15.child_frame_id = 'ring_finger_tip'
        
        odom_trans16 = TransformStamped()
        odom_trans16.header.frame_id = 'ring_finger_mcp'
        odom_trans16.child_frame_id = 'pinky_mcp'
        
        odom_trans17 = TransformStamped()
        odom_trans17.header.frame_id = 'pinky_mcp'
        odom_trans17.child_frame_id = 'pinky_pip'
        
        odom_trans18 = TransformStamped()
        odom_trans18.header.frame_id = 'pinky_pip'
        odom_trans18.child_frame_id = 'pinky_dip'
        
        odom_trans19 = TransformStamped()
        odom_trans19.header.frame_id = 'pinky_dip'
        odom_trans19.child_frame_id = 'pinky_tip'
        
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['metacarpals_joint', 'fake_proximal_joint', 'proximal_joint', 'intermediate_joint']
                joint_state.position = [tilt, tilt, tilt, tilt]

                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = cos(angle)*2
                odom_trans.transform.translation.y = sin(angle)*2
                odom_trans.transform.translation.z = 0.7
                odom_trans.transform.rotation = \
                    self.euler_to_quaternion(0, 0, (angle + pi/2)) # roll,pitch,yaw
                    
                odom_trans0.header.stamp = now.to_msg()
                odom_trans0.transform.translation.x = cos(angle)*2
                odom_trans0.transform.translation.y = sin(angle)*2
                odom_trans0.transform.translation.z = 0.7
                odom_trans0.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw
                    
                odom_trans1.header.stamp = now.to_msg()
                odom_trans1.transform.translation.x = cos(angle)*2
                odom_trans1.transform.translation.y = sin(angle)*2
                odom_trans1.transform.translation.z = 0.7
                odom_trans1.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw
                    
                odom_trans2.header.stamp = now.to_msg()
                odom_trans2.transform.translation.x = cos(angle)*2
                odom_trans2.transform.translation.y = sin(angle)*2
                odom_trans2.transform.translation.z = 0.7
                odom_trans2.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw
                    
                odom_trans3.header.stamp = now.to_msg()
                odom_trans3.transform.translation.x = cos(angle)*2
                odom_trans3.transform.translation.y = sin(angle)*2
                odom_trans3.transform.translation.z = 0.7
                odom_trans3.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

                odom_trans4.header.stamp = now.to_msg()
                odom_trans4.transform.translation.x = cos(angle)*2+5
                odom_trans4.transform.translation.y = sin(angle)*2+5
                odom_trans4.transform.translation.z = 0.7
                odom_trans4.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans5.header.stamp = now.to_msg()
                odom_trans5.transform.translation.x = cos(angle)*2+5
                odom_trans5.transform.translation.y = sin(angle)*2+5
                odom_trans5.transform.translation.z = 0.7
                odom_trans5.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans6.header.stamp = now.to_msg()
                odom_trans6.transform.translation.x = cos(angle)*2+5
                odom_trans6.transform.translation.y = sin(angle)*2+5
                odom_trans6.transform.translation.z = 0.7
                odom_trans6.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans7.header.stamp = now.to_msg()
                odom_trans7.transform.translation.x = cos(angle)*2+5
                odom_trans7.transform.translation.y = sin(angle)*2+5
                odom_trans7.transform.translation.z = 0.7
                odom_trans7.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans8.header.stamp = now.to_msg()
                odom_trans8.transform.translation.x = cos(angle)*2+1
                odom_trans8.transform.translation.y = sin(angle)*2+1
                odom_trans8.transform.translation.z = 0.7
                odom_trans8.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans9.header.stamp = now.to_msg()
                odom_trans9.transform.translation.x = cos(angle)*2+1
                odom_trans9.transform.translation.y = sin(angle)*2+1
                odom_trans9.transform.translation.z = 0.7
                odom_trans9.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans10.header.stamp = now.to_msg()
                odom_trans10.transform.translation.x = cos(angle)*2+1
                odom_trans10.transform.translation.y = sin(angle)*2+1
                odom_trans10.transform.translation.z = 0.7
                odom_trans10.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans11.header.stamp = now.to_msg()
                odom_trans11.transform.translation.x = cos(angle)*2+1
                odom_trans11.transform.translation.y = sin(angle)*2+1
                odom_trans11.transform.translation.z = 0.7
                odom_trans11.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans12.header.stamp = now.to_msg()
                odom_trans12.transform.translation.x = cos(angle)*2+.5
                odom_trans12.transform.translation.y = sin(angle)*2+.5
                odom_trans12.transform.translation.z = 0.7
                odom_trans12.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans13.header.stamp = now.to_msg()
                odom_trans13.transform.translation.x = cos(angle)*2+.5
                odom_trans13.transform.translation.y = sin(angle)*2+.5
                odom_trans13.transform.translation.z = 0.7
                odom_trans13.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans14.header.stamp = now.to_msg()
                odom_trans14.transform.translation.x = cos(angle)*2+.5
                odom_trans14.transform.translation.y = sin(angle)*2+.5
                odom_trans14.transform.translation.z = 0.7
                odom_trans14.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans15.header.stamp = now.to_msg()
                odom_trans15.transform.translation.x = cos(angle)*2+.5
                odom_trans15.transform.translation.y = sin(angle)*2+.5
                odom_trans15.transform.translation.z = 0.7
                odom_trans15.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans16.header.stamp = now.to_msg()
                odom_trans16.transform.translation.x = cos(angle)*2+.5
                odom_trans16.transform.translation.y = sin(angle)*2+.5
                odom_trans16.transform.translation.z = 0.7
                odom_trans16.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans17.header.stamp = now.to_msg()
                odom_trans17.transform.translation.x = cos(angle)*2+.5
                odom_trans17.transform.translation.y = sin(angle)*2+.5
                odom_trans17.transform.translation.z = 0.7
                odom_trans17.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans18.header.stamp = now.to_msg()
                odom_trans18.transform.translation.x = cos(angle)*2+.5
                odom_trans18.transform.translation.y = sin(angle)*2+.5
                odom_trans18.transform.translation.z = 0.7
                odom_trans18.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                    
                odom_trans19.header.stamp = now.to_msg()
                odom_trans19.transform.translation.x = cos(angle)*2+.5
                odom_trans19.transform.translation.y = sin(angle)*2+.5
                odom_trans19.transform.translation.z = 0.7
                odom_trans19.transform.rotation = \
                    self.euler_to_quaternion(0, 0, angle + pi/2)
                # send the joint state and transform
                #self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)
                self.broadcaster.sendTransform(odom_trans0)
                self.broadcaster.sendTransform(odom_trans1)
                self.broadcaster.sendTransform(odom_trans2)
                self.broadcaster.sendTransform(odom_trans3)
                self.broadcaster.sendTransform(odom_trans4)
                self.broadcaster.sendTransform(odom_trans5)
                self.broadcaster.sendTransform(odom_trans6)
                self.broadcaster.sendTransform(odom_trans7)
                self.broadcaster.sendTransform(odom_trans8)
                self.broadcaster.sendTransform(odom_trans9)
                self.broadcaster.sendTransform(odom_trans10)
                self.broadcaster.sendTransform(odom_trans11)
                self.broadcaster.sendTransform(odom_trans12)
                self.broadcaster.sendTransform(odom_trans13)
                self.broadcaster.sendTransform(odom_trans14)
                self.broadcaster.sendTransform(odom_trans15)
                self.broadcaster.sendTransform(odom_trans16)
                self.broadcaster.sendTransform(odom_trans17)
                self.broadcaster.sendTransform(odom_trans18)
                self.broadcaster.sendTransform(odom_trans19)

                # Create new robot state
                tilt += tinc
                if tilt < -0.5 or tilt > 0.0:
                    tinc *= -1
                height += hinc
                if height > 0.2 or height < 0.0:
                    hinc *= -1
                swivel += degree
                angle += degree/4

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()