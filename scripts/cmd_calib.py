#!/usr/bin/env python

import rospy
import message_filters
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import tf
from tf import transformations as tx

def anorm(x):
    return (x+np.pi) % (2*np.pi) - np.pi
def adiff(a,b):
    return anorm(a-b)

class CmdCalib(object):
    """
    Learn the nonlinear mapping from current cmd_vel estimate to actual velocity.
    Note that caster wheel hysteresis is not modeled, which may cause erroneous calibration.
    For best results, it is suggested to hold the each command at a steady-state,
    to prevent transient behavior from dominating the calibration results.
    """

    def __init__(self):
        # get parameters
        self.slop_ = rospy.get_param('~slop', default=0.01)
        self.win_ = rospy.get_param('~win', default=0.1) # collection window, sec
        self.min_sample_ = rospy.get_param('~min_sample', default=10) # minimum number of samples for data entry
        self.min_var_    = rospy.get_param('~min_var', default=0.02)
        self.src_ = rospy.get_param('~src', default='pose') # [pose, odom, tf]

        # register ROS handles
        vel_sub = message_filters.Subscriber('/cmd_vel', Twist)
        pos_sub = message_filters.Subscriber('/slam_out_pose', PoseStamped)
        #pos_sub = message_filters.Subscriber('/odom', Odometry)
        # TODO : support getting position information from map -> base_link transform

        self.sub_ = message_filters.ApproximateTimeSynchronizer(
                [vel_sub, pos_sub], 10, self.slop_, allow_headerless=True)
        self.sub_.registerCallback(self.data_cb)

        # cache
        self.last_proc_ = rospy.Time.now()
        self.raw_ = []
        self.data_ = [] # == (cv,cw,ev,ew,sv,sw)

    @staticmethod
    def to_2d(pose):
        x=pose.position.x
        y=pose.position.x
        o=pose.orientation
        q=[o.x,o.y,o.z,o.w]
        h=tx.euler_from_quaternion(q)[2]
        return (x,y,h)

    def data_cb(self, cmd_vel, ps):
        cv,cw  = cmd_vel.linear.x, cmd_vel.angular.z

        if np.abs(cv) < 1e-6 and np.abs(cw) < 1e-6:
            # don't record zeros
            return

        ex,ey,eh = self.to_2d(ps.pose)
        t = ps.header.stamp.to_sec()
        #ev,ew  = odom.twist.twist.linear.x, odom.twist.twist.angular.z
        # standard deviation(weighing factor for later)
        #sv     = np.sqrt(odom.twist.covariance[0*6+0])
        #sw     = np.sqrt(odom.twist.covariance[5*6+5])

        entry = (cv,cw,ex,ey,eh,t)#ev,ew,sv,sw)
        rospy.loginfo_throttle(1.0, '{}'.format(entry))
        self.raw_.append(entry)

    def proc(self):
        # discretize ...
        rospy.loginfo_throttle(1.0, '#entry: {}'.format(len(self.data_)))
        pass

    def save(self):
        data = np.asarray(self.data_, dtype=np.float32)
        np.save('/tmp/data.npy', data)

    def step(self):
        now = rospy.Time.now()
        dt_proc = (now - self.last_proc_).to_sec()

        # filter by elapsed time and number of samples
        if (dt_proc < self.win_) or len(self.raw_) < self.min_sample_:
            return

        rospy.loginfo('dt,len : {},{}'.format(dt_proc, len(self.raw_)))

        raw = np.asarray(self.raw_, dtype=np.float32)
        v_cv, v_cw = np.var(raw[:,:2], axis=0)

        if v_cv < self.min_var_ and v_cw < self.min_var_:
            # filter by input variance
            # considered steady-state input (i.e. not in transition)

            c_dt = np.diff(raw[:,-1])
            dt_entry = float(raw[-1,-1] - raw[0,-1]) # == sum(c_dt)

            dist = np.sum(np.linalg.norm(np.diff(raw[:,2:4], axis=0),axis=-1))

            # ground truth lin.vel / ang.vel
            gv    = dist / dt_entry
            gw    = adiff(raw[-1,4], raw[0,4]) / dt_entry

            # commanded lin.vel / and.vel
            #cv, cw = np.mean(raw[:,:2], axis=0) - naive version
            cv = np.sum(raw[:-1,0]*c_dt) / dt_entry
            cw = np.sum(raw[:-1,1]*c_dt) / dt_entry

            # enter processed information into database
            self.data_.append( [gv,gw,cv,cw] )
        else:
            # no data is entered to the database.
            # however, self.raw_ and self.last_proc_ cache are cleared
            # to make room for next processing step.
            pass
        self.raw_ = []
        self.last_proc_ = now

    def run(self):
        rate = rospy.Rate(50)
        rospy.on_shutdown(self.save)
        while not rospy.is_shutdown():
            self.step()
            self.proc()
            rate.sleep()

def main():
    rospy.init_node('cmd_calib')
    app = CmdCalib()
    app.run()

if __name__ == '__main__':
    main()
