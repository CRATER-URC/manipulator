import numpy as np
import rclpy

from rclpy.node         import Node
from joint_control_msg.msg import JointControl
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Float64MultiArray

class JointControlNode(Node):
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        rate       = RATE
        
        self.dt = 1.0 / float(RATE)
        self.timer = self.create_timer(self.dt, self.sendcmd)

        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))
        self.get_logger().info("Running %s" % name)

        self.cmdsub = self.create_subscription(JointControl, '/cmd_joint', self.recvinput, 1)

        self.jointsub = self.create_subscription(JointState, '/joint_states', self.recvjoint, 1)

        self.velpub = self.create_publisher(Float64MultiArray, '/commands', 10)

        self.msg = Float64MultiArray()
        self.msg.data = [0.0, 0.0, 0.0, 0.0]

    def shutdown(self):
        self.destroy_node()

    def recvinput(self, msg):
        self.msg.data[0] = msg.joint1
        self.msg.data[1] = msg.joint2
        self.msg.data[2] = msg.joint3
        self.msg.data[3] = msg.joint4
        self.get_logger().info("Publishing message : " + str(self.msg.data))
        self.velpub.publish(self.msg)

    def recvjoint(self, msg):
        self.get_logger().info("Received joint state message : " + str(msg.velocity))
        

def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = JointControlNode('jointcontrol')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    