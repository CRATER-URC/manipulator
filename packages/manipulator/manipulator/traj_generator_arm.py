#!/usr/bin/env python3
#
#   demo134.py
#
#   Demonstration node to interact with the HEBIs.
#
import numpy as np
import rclpy

from rclpy.node         import Node
from sensor_msgs.msg    import JointState
from geometry_msgs.msg import Pose, Point, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from manipulator.TrajectoryUtils import goto, goto5, spline, spline5
from manipulator.KinematicChain import KinematicChain
from manipulator.TransformHelpers import *



#
#   Definitions
#
RATE = 1000.0            # Hertz


#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)


        # self.trajpub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        # TODO: get feedback about actual motor states
        # Subscribe to the actual joint states, waiting for the first message.
        # self.actpos = None
        # self.statessub = self.create_subscription(JointState, "/joint_states", self.cb_states, 1)
        # while self.actpos is None:
        #     rclpy.spin_once(self)
        # self.get_logger().info("Initial positions: %r" % self.actpos)
        
        
        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)
        
        self.fbksub = self.create_subscription(Point, '/point', self.recvpoint, 10)
        self.fbksub = self.create_subscription(Pose, '/pose', self.recvpose, 10)
        
        self.chain = KinematicChain(self, 'world', 'tip', self.jointnames())
        
        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        
        self.dt = 1.0 / float(RATE)
        self.timer = self.create_timer(self.dt, self.sendcmd)

        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))
        self.get_logger().info("Running %s" % name)
        
        

        # # Create a temporary subscriber to grab the initial position.
        self.q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.p0, _, _, _ = self.chain.fkin(self.q0)
        self.R0 = Roty(np.pi/2)
        self.get_logger().info("Initial positions: %r" % self.q0)
        self.get_logger().info("Initial start: %r" % self.p0)


        self.change = [0.02, 0.005, 0.0025, 0.0025, 0.0025]
        self.limit_min = [-1, -0.5, -0.5, -0.5, -0.5]
        self.limit_max = [1, 0.5, 0.5, 0.5, 0.5]

        self.goalpos = None
        self.goalrot = None
        self.goalnext = None
        self.goalnextangle = None
        self.mid = None
        self.startmov = None
        self.endpos = None
        self.gamma = 0
        self.printed = False

        self.collision = False

        self.max_effort = [2, 3, 1, 0.5, 0.5]

        self.h = 0.04

        self.t = 0
        self.T = 1 # TODO need to change
        self.T2 = 2
        self.retT = 0.3
        # self.lam = 10
        self.lam = 0.1 / self.dt
        # self.lam = 10

        # Period of sinusoid
        self.waitpos = np.radians(np.array([95, 5, 95, 5, 5, 5]))
        self.startpos, self.R0, _, _ = self.chain.fkin(self.waitpos)
        self.startros = self.R0
        self.get_logger().info("start" + str(self.startpos))
        
        self.q = self.q0
        self.qdot = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pd = self.startpos
        self.Rd = self.R0
        
        self.x = 0
        self.y = 0
        self.z = 0
        
        
        self.cmdsub = self.create_subscription(Twist, '/cmd_vel', self.recvinput, 1)
        

    def cb_states(self, msg):
        # Save the actual position.
        self.actpos = msg.position
        self.acteff = msg.effort
    
    def jointnames(self):
        return ['joint0', 'joint1', 'joint2', 'joint3', 'joint4'] #, 'tiprotary']

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Grab a single feedback - do not call this repeatedly.
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return self.grabpos


    # Receive feedback - called repeatedly by incoming messages.
    def recvfbk(self, fbkmsg):
        # Just print the position (for now).
        # print(list(fbkmsg.position))
        pass
    
    def recvinput(self, msg):
        if self.goalpos is None and( msg.linear.x != 0 or msg.linear.y != 0 or msg.linear.z != 0 or msg.angular.x != 0 or msg.angular.y != 0 or msg.angular.z != 0): 
            p, R, _, _ = self.chain.fkin(self.q)
            self.startpos = p
            self.startros = R
            self.goalpos = np.array([p[0] + msg.linear.x, p[1] + msg.linear.y, p[2] + msg.linear.z])
            self.goalrot = R @ Rotz(msg.angular.z) @ Roty(msg.angular.y) @ Rotx(msg.angular.x)
            self.startmov = self.t
            self.get_logger().info("current" + str(p) + "\nreceived msg" + str(msg) + "\ngoing to " +str(self.goalpos))
        # elif ( msg.linear.x != 0 or msg.linear.y != 0 or msg.linear.z != 0 or msg.angular.x != 0 or msg.angular.y != 0 or msg.angular.z != 0):
        #     self.get_logger().info("curr goal " + str(self.goalpos) + "\nreceived msg" + str(msg))
            
            

    def recvpoint(self, pointmsg):
        self.printed = False
        if self.goalpos is None:
            self.goalpos = np.array([pointmsg.x, pointmsg.y, pointmsg.z])
            self.startmov = self.t
            self.endpos = None
        
        # Report.
            self.get_logger().info("Running point %r, %r, %r" % (pointmsg.x,pointmsg.y,pointmsg.z))
            
    def recvpose(self, posemsg):
        if self.goalpos is None:
            self.mid = np.array([posemsg.position.x, posemsg.position.y, 0.08])
            theta = np.arccos(posemsg.orientation.w) * 2 
            sin_theta = 2 * (posemsg.orientation.z) * (posemsg.orientation.w)
            cos_theta = (posemsg.orientation.w)**2 - (posemsg.orientation.z)**2

            theta2 = np.arcsin(sin_theta)
            theta3 = np.arccos(cos_theta)


            p1 = (self.mid[0] + (self.h/2 * cos_theta), self.mid[1] + (self.h/2 * sin_theta), 0)
            p2 = (self.mid[0] - (self.h / 2 * cos_theta), self.mid[1] - (self.h / 2 * sin_theta), 0) 

            self.goalnext = np.array(p2)
            self.goalpos = np.array(p1)
            self.startmov = self.t
            self.endpos = None
            
            
            self.get_logger().info("Got pose %r, %r" % (posemsg.position,posemsg.orientation))
            self.get_logger().info("Goals: %r, %r, %r, %r, %r, %r" % (self.goalnext, self.mid, self.goalpos, theta, theta2, theta3))
        
        # Report.
            self.get_logger().info("Running pose ") # %r, %r, %r" % (pointmsg.x,pointmsg.y,pointmsg.z))
    
    def getq(self, pd, vd, Rd, wd):
        """
        Calls inverse kinematics with given pd and vd to return the desired robot q
        """
        # pd = pd.reshape((-1,1))
        # vd = vd.reshape((-1,1))
        
        qlast  = self.q
        pdlast = self.pd
        Rdlast = self.Rd
        # self.get_logger().info("qlast" + str(qlast))
        (p, R, Jv, Jw) = self.chain.fkin(qlast)
        
        # self.get_logger().info("p" + str(p))
        
        vr    = vd + self.lam * ep(pdlast, p)
        wr    = wd + self.lam * eR(Rdlast, R)
        J     = np.vstack((Jv, Jw))
        xrdot = np.vstack((vr, wr))
        
        # self.get_logger().info("xrdot" + str(xrdot))
        self.get_logger().info("vr" + str(vr) + "\nvd" + str(vd) + "\nep" + str(ep(pdlast, p)))
        
        Jinv = np.transpose(J) @ np.linalg.pinv((J @ np.transpose(J) + self.gamma**2 * np.eye(len(J))))
        E = np.vstack((ep(pdlast, p), eR(Rdlast, R)))
        # self.get_logger().info("E" + str(E))
        qdot = (Jinv @ (xrdot + self.lam * E)).flatten()
        
        # self.get_logger().info("qdot" + str(qdot))
        
        # self.get_logger().info("qdot" + str(qdot) + "\nJ" + str(J) + "\nxrdot" + str(xrdot))

        # Integrate the joint position.
        q = qlast + self.dt * qdot
        # q = q[:, 0].reshape((1, -1))[0]

        # Save the joint value and desired values for next cycle.
        self.q  = q
        self.pd = pd
        self.Rd = Rd
        
        (p, R, Jv, Jw) = self.chain.fkin(q)
        # self.get_logger().info("p" + str(p) + "\npd" + str(pd))
        
        return qdot, q
    # Send a command - called repeatedly by the timer.
    def sendcmd(self):
        # Build up the message and publish.
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = self.jointnames()
        self.t += self.dt

        # initial move to wait position
        if self.t < self.retT:
            (self.q, velocity) = goto(self.t, self.retT, np.array(self.q0), np.array([self.q0[0], self.waitpos[1], self.q0[2], self.q0[3], self.q0[4], self.q0[5]]))

        elif self.t < 2*self.retT:
            (self.q, velocity) = goto(self.t-self.retT, self.retT, np.array([self.q0[0], self.waitpos[1], self.q0[2], self.q0[3], self.q0[4], self.q0[5]]), self.waitpos)

        # when goal is detected from from Point node
        elif self.goalpos is not None and len(self.goalpos) == 3:
            # for i in range(len(self.acteff)):
            #     if abs(self.acteff[i]) > abs(self.max_effort[i]) and not self.collision:
            #         self.endpos = self.position.copy()
            #         self.t += (self.T - (self.t - self.startmov))
            #         self.collision = True

            # going to goal     
            # self.get_logger().info("goalpos" + str(self.goalpos))
            if self.t < self.startmov + self.T:
                pd, vd = goto(self.t - self.startmov, self.T, self.startpos.reshape((-1,1)), self.goalpos.reshape((-1,1)))
                # self.get_logger().info("pd" + str(pd))
                # self.get_logger().info("vd" + str(vd))
                if self.goalrot is None:
                    Rd = self.Rd
                    wd = np.zeros((3,1))
                else:
                    s, sdot = goto(self.t - self.startmov, self.T, 0, 1)
                    Rd = Rinter(self.startros, self.goalrot, s)
                    wd = winter(self.startros, self.goalrot, sdot)
                    # self.get_logger().info("Rd" + str(Rd) + "\nwd" + str(wd))
                self.qdot, self.q = self.getq(pd, vd, Rd, wd)
            else:
                self.goalpos = None
                if not self.printed:
                    # self.get_logger().info("q : " + str(self.q))
                    (p, R, Jv, Jw) = self.chain.fkin(self.q)
                    # self.get_logger().info("final p : " + str(p))
                    self.printed = True
                    self.goalpos = None
        else:
            self.goalpos = None
        
        self.cmdmsg.position     = self.q.flatten().tolist()
        self.cmdmsg.velocity = self.qdot.flatten().tolist()

        self.cmdpub.publish(self.cmdmsg)


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = DemoNode('demo')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
