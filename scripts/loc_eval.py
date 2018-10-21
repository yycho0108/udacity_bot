#!/usr/bin/env python2

import numpy as np
import rospy
import message_filters
import sys

from tf_conversions import posemath as pm
from tf import transformations as tx

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from matplotlib import pyplot as plt

def anorm(x):
    return (x + np.pi) % (2*np.pi) - np.pi

def roll_rospose(x):
    txn, qxn = pm.toTf(pm.fromMsg(x))
    x,y,_ = txn
    rz = tx.euler_from_quaternion(qxn)[-1]
    res = [x,y,rz]
    return res

class LocEval(object):
    """ evaluate localization performance """
    def __init__(self):
        # unroll params
        self.plot_ = rospy.get_param('~plot', default=False)
        if(self.plot_):
            self.plot()
            sys.exit(0)

        self.slop_ = rospy.get_param('~slop', default=0.01)
        self.rate_ = rospy.get_param('~rate', default=50.0)

        pf_sub = message_filters.Subscriber('/amcl_pose', PoseWithCovarianceStamped)
        gt_sub = message_filters.Subscriber('/ground_truth_pose', Odometry)
        self.sub_ = message_filters.ApproximateTimeSynchronizer(
                [pf_sub, gt_sub], 10, self.slop_, allow_headerless=False)
        self.sub_.registerCallback(self.data_cb)

        self.err_ = []

        self.last_recv = rospy.Time.now()

    def data_cb(self, gt, pf):
        self.last_recv=gt.header.stamp
        t = gt.header.stamp.to_sec()
        gt_pose = roll_rospose(gt.pose.pose)
        pf_pose = roll_rospose(pf.pose.pose)
        dx,dy,dh = np.subtract(gt_pose, pf_pose)
        dh = anorm(dh) # normalize angle
        err = [t, dx,dy,dh]

        #print('gt:{} | pf:{}'.format(gt_pose,pf_pose))
        rospy.loginfo_throttle(0.5, 'err:{}'.format(err))
        self.err_.append(err)

    def run(self):
        rate=rospy.Rate(self.rate_)
        rospy.on_shutdown(self.save)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

    def stat(self):
        n = len(self.err_)
        if n <= 0:
            return 'None'
        else:
            t,dx,dy,dh = np.around(np.mean(np.abs(self.err_), axis=0), 3)
            return '[{}] Mean Error : (x:{},y:{},h:{})'.format(n, dx,dy,dh)

    def step(self):
        now = rospy.Time.now()
        dt = (now - self.last_recv).to_sec()

        if dt > 5.0:
            rospy.loginfo_throttle(1.0, 'No message received for {} seconds'.format(dt))

        rospy.loginfo_throttle(1.0, 'Data Stats: {}'.format(self.stat()))

    def plot(self):
        data = np.load('/tmp/err.npy')
        stamp = data[:,0] - data[0,0]
        d_pos = np.linalg.norm(data[:,1:3], axis=-1)
        d_rot = np.rad2deg(np.abs(data[:,-1]))
        fig, ax = plt.subplots(2,1, sharex=True)
        ax[0].plot(stamp, d_pos)
        ax[0].set_ylabel('Pos Error(m)')
        ax[0].grid()
        ax[0].set_title('Udacity Bot Localization Error Characterization')
        ax[1].plot(stamp, d_rot)
        ax[1].set_ylabel('Rot Error(deg)')
        ax[1].grid()
        ax[1].set_xlabel('Time (sec)')
        plt.show()

    def save(self):
        np.save('/tmp/err.npy', self.err_)

def main():
    rospy.init_node('loc_eval')
    node = LocEval()
    node.run()

if __name__ == "__main__":
    main()
