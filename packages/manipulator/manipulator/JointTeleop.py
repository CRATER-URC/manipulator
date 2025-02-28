#!/usr/bin/env python3
#
#   teleop.py
#
#   Continually (at 50Hz!) send body velocity commands for remote control.
#
#   Node:       /teleop
#   Publish:    /cmd_vel        geometry_msgs/Twist
#   Subscribe:  -nothing-
#
import curses
import numpy as np
import sys

# ROS Imports
import rclpy

from rclpy.node                 import Node
from rclpy.time                 import Time

from joint_control_msg.msg     import JointControl


#
#   Global Definitions
#
RATE = 50.0             # Hertz

VNOM = 0.25
WNOM = 0.50             # This gives a turning radius of v/w = 0.5m


#
#   Custom Node Class
#
class CustomNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Pull out the nominal forward/spin speeds from the parameters.
        if (len(sys.argv) == 1):
            self.get_logger().info("Usage: teleop.py forward_speed spin_speed")
            self.get_logger().info("Using default values...")
            self.vnom = VNOM
            self.wnom = WNOM
        elif (len(sys.argv) == 3):
            self.vnom = float(sys.argv[1])
            self.wnom = float(sys.argv[2])
        else:
            self.get_logger().info("Usage: teleop.py forward_speed spin_speed")
            self.get_logger().info("GOOD DEFAULTS: teleop.py %3.1f %3.1f" %
                                   (VNOM, WNOM))
            raise Exception("Illegal Arguments")

        # Initialize the (repeating) message data.
        self.msg = JointControl()
        self.msg.joint0 = 0.0
        self.msg.joint1 = 0.0
        self.msg.joint2 = 0.0
        self.msg.joint3 = 0.0
        self.msg.joint4 = 0.0
        self.msg.joint5 = 0.0
        self.msg.gripper = 0.0


        # Create a publisher to send twist commands.
        self.pub = self.create_publisher(JointControl, '/cmd_joint', 10)

        # Create a fixed rate to control the speed of sending commands.
        self.rate = self.create_rate(RATE)
        self.dt   = 1/RATE
        self.get_logger().info("Sending commands with dt %fs (%fHz)" %
                               (self.dt, 1.0 / self.dt))

        # Report the speeds and return.
        self.get_logger().info("Nominal fwd = %6.3fm/s, spin = %6.3frad/s"
                               % (self.vnom, self.wnom))
         
    # Shutdown
    def shutdown(self):
        # Destroy the rate and node.
        self.destroy_rate(self.rate)
        self.destroy_node()


    # Run the terminal input loop, send the commands, and ROS spin.
    def loop(self, screen):
        # Make the getchar() non-blocking and otherwise initialize.
        curses.curs_set(0)
        curses.flushinp()
        screen.nodelay(True)
        screen.erase()
        screen.addstr(0,  0, "!!! IGNORE JOINT 0, 5, Gripper FOR NOW !!!")
        screen.addstr(1,  0, "Joint 0: a z")
        screen.addstr(2,  0, "Joint 1: s x")
        screen.addstr(3,  0, "Joint 2: d c")
        screen.addstr(4,  0, "Joint 3: f v")
        screen.addstr(5,  0, "Joint 4: g b")
        screen.addstr(6,  0, "Joint 5: h n")
        screen.addstr(7,  0, "Gripper: j m")        
        screen.addstr(8,  0, "Commands last 0.5s")
        screen.addstr(9,  0, "Hit q to quit")
        screen.addstr(10,  0, "Nominal fwd = %6.3fm/s, spin = %6.3frad/s" %
                      (self.vnom, self.wnom))

        # Set up the map from keycodes (0..127) to velocities.
        # nom = [(x, y, z, r, p, y)]
        nom  = [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)] * 0x20
        nom[ord('a') & 0x1F] = (self.vnom, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # a = joint 0 +
        nom[ord('z') & 0x1F] = (-self.vnom, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # z = joint 0 -
        nom[ord('s') & 0x1F] = (0.0, self.vnom, 0.0, 0.0, 0.0, 0.0, 0.0)  # s = joint 1 +
        nom[ord('x') & 0x1F] = (0.0, -self.vnom, 0.0, 0.0, 0.0, 0.0, 0.0)  # x = joint 1 -
        nom[ord('d') & 0x1F] = (0.0, 0.0, self.vnom, 0.0, 0.0, 0.0, 0.0)  # d = joint 2 +
        nom[ord('c') & 0x1F] = (0.0, 0.0, -self.vnom, 0.0, 0.0, 0.0, 0.0)  # c = joint 2 -
        nom[ord('f') & 0x1F] = (0.0, 0.0, 0.0, self.vnom, 0.0, 0.0, 0.0)  # f = joint 3 +
        nom[ord('v') & 0x1F] = (0.0, 0.0, 0.0, -self.vnom, 0.0, 0.0, 0.0)  # v = joint 3 -
        nom[ord('g') & 0x1F] = (0.0, 0.0, 0.0, 0.0, self.vnom, 0.0, 0.0)  # g = joint 4 +
        nom[ord('b') & 0x1F] = (0.0, 0.0, 0.0, 0.0, -self.vnom, 0.0, 0.0)  # b = joint 4 -
        nom[ord('h') & 0x1F] = (0.0, 0.0, 0.0, 0.0, 0.0, self.vnom, 0.0)  # h = joint 5 +
        nom[ord('n') & 0x1F] = (0.0, 0.0, 0.0, 0.0, 0.0, -self.vnom, 0.0)  # n = joint 5 -
        nom[ord('j') & 0x1F] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.vnom)  # j = gripper +
        nom[ord('m') & 0x1F] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -self.vnom)  # m = gripper -
        
        map  = []
        # map += [(0.25*vx, 0.25*wz) for (vx, wz) in nom]  # Control   = slow
        # map += [(0.0 *vx, 0.0 *wz) for (vx, wz) in nom]  # Numbers   = nothing
        # map += [(2.0 *vx, 2.0 *wz) for (vx, wz) in nom]  # Uppercase = fast
        # map += [(1.0 *vx, 1.0 *wz) for (vx, wz) in nom]  # Lowercase = nominal

        map += [(0.25 * j0, 0.25 * j1, 0.25 * j2, 0.25 * j3, 0.25 * j4, 0.25 * j5, 0.25 * grip) for (j0, j1, j2, j3, j4, j5, grip) in nom]  # Control   = slow
        map += [(0.0 * j0, 0.0 * j1, 0.0 * j2, 0.0 * j3, 0.0 * j4, 0.0 * j5, 0.0 * grip) for (j0, j1, j2, j3, j4, j5, grip) in nom]  # Numbers   = nothing
        map += [(2.0 * j0, 2.0 * j1, 2.0 * j2, 2.0 * j3, 2.0 * j4, 2.0 * j5, 2.0 * grip) for (j0, j1, j2, j3, j4, j5, grip) in nom]  # Uppercase = fast
        map += [(1.0 * j0, 1.0 * j1, 1.0 * j2, 1.0 * j3, 1.0 * j4, 1.0 * j5, 1.0 * grip) for (j0, j1, j2, j3, j4, j5, grip) in nom]

        # Initialize the velocity and remaining active time.
        key = 0
        vel = (0.0, 0.0)
        Tactive = 0.0

        # Run the loop until shutdown.
        while rclpy.ok():
            # Reduce the active time and stop if necessary.
            Tactive -= self.dt
            if Tactive <= 0.0:
                vel = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                Tactive = 0.0

            # Process all pending keys.
            while True:
                keycode = screen.getch()
                if keycode == -1:
                    break

                # Reset the active time.
                key     = keycode
                Tactive = 0.5

                # Change the velocity based on the key.
                if not ((keycode >= 0) and (keycode < 0x80)):
                    vel = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                elif (keycode & 0x1F) == (ord('q') & 0x1F):
                    return
                else:
                    vel = map[keycode]

            # Report.
            if (key >= 0x20) and (key <= 0x80):
                s = "'" + str(chr(key)) + "'"
            else:
                s = "%d" % key
            screen.addstr(8, 0, "Last pressed 0x%02x = %s" % (key, s))
            screen.clrtoeol()
            screen.addstr(11, 0,
                          "Sending j0 = %6.3f, j1 = %6.3f, j2 = %6.3f, j3 = %6.3f, j4 = %6.3f, j5 = %6.3f, grip = %6.3f" % vel)

            # Update the message and publish.
            self.msg.joint0 = vel[0]
            self.msg.joint1 = vel[1]
            self.msg.joint2 = vel[2]
            self.msg.joint3 = vel[3]
            self.msg.joint4 = vel[4]
            self.msg.joint5 = vel[5]
            self.msg.gripper = vel[6]
            self.pub.publish(self.msg)

            # Spin once to process other items.
            rclpy.spin_once(self)

            # Wait for the next turn.
            self.rate.sleep()


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the node.
    node = CustomNode('JointTeleop')

    # Run the terminal input loop, which spins the ROS items.
    try:
        curses.wrapper(node.loop)
    except KeyboardInterrupt:
        pass

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
