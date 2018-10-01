#!/usr/bin/env python2

import rospy
import tf

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from std_srvs.srv import Empty

def xq2p(x,q):
    """ convert POD to ROS datatyes """
    res = Pose()
    res.position.x = x[0]
    res.position.y = x[1]
    res.position.z = x[2]

    res.orientation.x = q[0]
    res.orientation.y = q[1]
    res.orientation.z = q[2]
    res.orientation.w = q[3]
    return res

class TrajectoryPublisher(object):
    """
    Plots trajectory of `target` frame w.r.t. `source` frame and publishes over `path` topic (nav_msgs/Path).
    WARNING: the source and target definitions are different from TransformListener().lookupTransform(...) arguments,
    because at least I think it's a confusing name.
    """
    
    def __init__(self):
        rospy.init_node('plot_trajectory')
        self._src_frame = rospy.get_param('~source', 'map')
        self._dst_frame = rospy.get_param('~target', 'robot_footprint')

        self._path = Path()
        self._path.header = Header(frame_id=self._src_frame, stamp=rospy.Time.now())
        self._path.header.frame_id = self._src_frame
        self._path.header.stamp = rospy.Time.now()

        self._tfl = tf.TransformListener()
        self._pub = rospy.Publisher('path', Path, queue_size=10)
        self._reset_srv = rospy.Service('reset', Empty, self.reset)

    def reset(self, *args, **kwargs):
        """ reset path cache """
        self._path.poses = []

    def run(self):
        """ main loop """
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                x, q = self._tfl.lookupTransform(self._src_frame, self._dst_frame, rospy.Time(0))
                self._path.poses.append(PoseStamped(
                    header=Header(frame_id=self._src_frame, stamp=rospy.Time.now()),
                    pose=xq2p(x,q)))
                self._pub.publish(self._path)
            except Exception as e:
                print e
            rate.sleep()

def main():
    app = TrajectoryPublisher()
    app.run()

if __name__ == '__main__':
    main()
