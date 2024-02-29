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
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from manipulator.TrajectoryUtils import goto, goto5, spline, spline5
from manipulator.KinematicChain import KinematicChain
from manipulator.TransformHelpers import *



#
#   Definitions
#
RATE = 100.0            # Hertz


#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization.
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
        
        self.inputsub = self.create_subscription(String, '/key_input', self.recvinput, 1)
        
        self.chain = KinematicChain(self, 'world', 'tip', self.jointnames())
        
        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        self.timer = self.create_timer(1/rate, self.sendcmd)

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
        self.goalnext = None
        self.goalnextangle = None
        self.mid = None
        self.startmov = None
        self.endpos = None
        self.gamma = 0.01

        self.collision = False

        self.max_effort = [2, 3, 1, 0.5, 0.5]

        self.h = 0.04

        self.t = 0
        self.T = 2
        self.T2 = 2
        self.retT = 2
        self.dt = 1.0 / float(RATE)
        self.lam = 0.1

        # Period of sinusoid
        self.waitpos = np.radians(np.array([0, 0, 90, 0, 0, 0]))
        self.startpos, _, _, _ = self.chain.fkin(self.waitpos)
        self.get_logger().info("start" + str(self.startpos))
        
        self.q = self.q0
        self.qdot = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pd = self.startpos
        self.Rd = self.R0
        
        self.x = 0
        self.y = 0
        self.z = 0
        

    def cb_states(self, msg):
        # Save the actual position.
        self.actpos = msg.position
        self.acteff = msg.effort
    
    def jointnames(self):
        return ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'tiprotary']
        # return ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'tiprotary']

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
    
    def recvinput(self, strmsg):
        if strmsg.data is not None and len(strmsg.data) > 0:
            self.get_logger().info(strmsg.data)

    def recvpoint(self, pointmsg):
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

    def gravity(self, pos):
        tau_shoulder = 2 * np.sin(pos[1]) + 0 * np.cos(pos[1])
        # TODO tau elbow
        return (0.0, -tau_shoulder, 0.0)
    
    def getq(self, pd, vd, Rd, wd):
        """
        Calls inverse kinematics with given pd and vd to return the desired robot q
        """
        # pd = pd.reshape((-1,1))
        # vd = vd.reshape((-1,1))
        
        qlast  = self.q
        pdlast = self.pd
        Rdlast = self.Rd
        self.get_logger().info("qlast" + str(qlast))
        (p, R, Jv, Jw) = self.chain.fkin(qlast)
        
        vr    = vd + self.lam * ep(pdlast, p)
        wr    = wd + self.lam * eR(Rdlast, R)
        J     = np.vstack((Jv, Jw))
        xrdot = np.vstack((vr, wr))
        
        # J = np.transpose(J) @ np.linalg.pinv((J @ np.transpose(J) + self.gamma**2 * np.eye(len(J))))
        # E = np.vstack((ep(pdlast, p), eR(Rdlast, R)))
        # qdot = (J @ xrdot + self.lam * E)[0]
        qdot = (np.linalg.inv(J) @ xrdot)

        # Integrate the joint position.
        q = qlast + self.dt * qdot
        q = q[:, 0].reshape((1, -1))[0]
        
        self.get_logger().info("qdot" + str(qdot) + "\nq : " + str(q) + "\nJ : " + str(J))
        
        self.pd = pd
        self.Rd = Rd
        
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
        
        elif self.x != 0:
            pd, Rd, _, _ = self.chain.fkin(self.q)
            self.get_logger().info("prev" + str(pd))
            pd[0] += np.sign(self.x) / (10 * RATE)
            vd = np.zeros((3, 1))
            vd[0] += np.sign(self.x) * RATE / 10
            wd = np.zeros((3,1))
            self.qdot, self.q = self.getq(pd, vd, Rd, wd)
            self.get_logger().info("next" + str(self.pd))
            
        elif self.y != 0 and self.t < 2*self.retT + 10/RATE:
            pd, Rd, _, _ = self.chain.fkin(self.q)
            self.get_logger().info("prev" + str(pd))
            pd[1] += np.sign(self.y) / ( 10 * RATE)
            vd = np.zeros((3, 1))
            vd[1] += np.sign(self.y) * RATE / 10
            wd = np.zeros((3,1))
            self.qdot, self.q = self.getq(pd, vd, Rd, wd)
            self.get_logger().info("next" + str(self.pd))
            
        elif self.z != 0:
            pd, Rd, _, _ = self.chain.fkin(self.q)
            self.get_logger().info("prev" + str(pd))
            pd[2] += np.sign(self.z) / (10 * RATE)
            vd = np.zeros((3, 1))
            vd[2] += np.sign(self.z) * RATE / 10
            wd = np.zeros((3,1))
            self.qdot, self.q = self.getq(pd, vd, Rd, wd)
            self.get_logger().info("next" + str(self.pd))

        # when goal is detected from from Point node
        elif self.goalpos is not None and len(self.goalpos) == 3:

            # for i in range(len(self.acteff)):
            #     if abs(self.acteff[i]) > abs(self.max_effort[i]) and not self.collision:
            #         self.endpos = self.position.copy()
            #         self.t += (self.T - (self.t - self.startmov))
            #         self.collision = True

            # going to goal     
            if self.t < self.startmov + self.T:
                pd, vd = goto(self.t - self.startmov, self.T, self.startpos.reshape((-1,1)), self.goalpos.reshape((-1,1)))

                Rd = self.Rd
                wd = np.zeros((3,1))
                self.qdot, self.q = self.getq(pd, vd, Rd, wd)

            # going back to waiting position, very similar to code from starting to wait pos. this is for point only
            elif self.t < self.startmov + self.T + 2*self.retT:
                if self.endpos is None:
                    self.endpos = self.q.copy()
                    
                if self.t < self.startmov + self.T + self.retT:
                    self.q, velocity = goto(self.t - self.startmov - self.T, self.retT, self.endpos, np.array([self.endpos[0], self.waitpos[1], self.endpos[2], self.endpos[3], self.endpos[4], self.endpos[5]]))
                else:
                    self.q, velocity = goto(self.t - self.startmov - self.T - self.retT, self.retT, np.array([self.endpos[0], self.waitpos[1], self.endpos[2], self.endpos[3], self.endpos[4], self.endpos[5]]), self.waitpos)

            else:
                self.get_logger().info("endpos" + str(self.endpos) + "end tip" + str(self.pd))
                self.collision = False
                self.endpos = None
                self.goalpos = None
                self.startmov = None 
                self.goalnext = None
                self.mid = None
                self.goalnextangle = None
        
        
        self.cmdmsg.position     = self.q.flatten().tolist()
        self.cmdmsg.velocity = self.qdot.flatten().tolist()
        # self.cmdmsg.effort = self.gravity(self.actpos)

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
