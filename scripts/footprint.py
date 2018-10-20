"""
short script to accurately determine the footprint of a simple robot.
The assumption here

"""

import numpy as np
from scipy.spatial import ConvexHull
from matplotlib import pyplot as plt

def R(x):
    c, s = np.cos(x), np.sin(x)
    return np.reshape([c,-s,s,c], (2,2))

def box_points(o, w, h, r=0.0):
    w2, h2 = w/2.0, h/2.0

    pts = [[w2,h2], [w2,-h2], [-w2,-h2], [-w2,h2]]
    # right-multiply rotation matrix
    return np.reshape(o, [-1,2]) + np.dot(pts, R(r).T)

def plot_closed(pts, *args, **kwargs):
    pts_wrap = np.concatenate([pts, pts[:1]], axis=0)
    plt.plot(pts_wrap[:,0], pts_wrap[:,1], *args, **kwargs)

def main():
    l_wheel = box_points([0.0, 0.15], 0.2, 0.05)
    r_wheel = box_points([0.0, -0.15], 0.2, 0.05)
    chassis = box_points([0.0,0.0], 0.4, 0.2)
    
    pts = np.concatenate([l_wheel, r_wheel, chassis])
    idx = ConvexHull(pts).vertices
    fpts = pts[idx]

    print('footprint output')
    print(repr(fpts))
    print('----------------')

    plot_closed(l_wheel, 'r--', label='l_wheel')
    plot_closed(r_wheel, 'g--', label='r_wheel')
    plot_closed(chassis, 'b--', label='chassis')
    plot_closed(fpts, 'k-', label='footprint')

    #plt.plot(l_wheel[:,0], l_wheel[:,1], 'k--', label='l_wheel')
    #plt.plot(r_wheel[:,0], r_wheel[:,1], 'k--', label='r_wheel')
    #plt.plot(chassis[:,0], chassis[:,1], 'k--', label='chassis')
    #plt.plot(fpts[:,0], fpts[:,1], label='footprint')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('footprint hull visualization')
    plt.grid()
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
