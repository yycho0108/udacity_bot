#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt

def main():
    m = np.load('/tmp/data.npy')
    m = m[:-4]
    gv, gw, cv, cw = m.T

    # find limits
    v = np.concatenate([gv,cv], axis=0)
    w = np.concatenate([gw,cw], axis=0)
    mn_v, mx_v = np.min(v), np.max(v)
    s_v = (mx_v - mn_v)
    avg_v = (mx_v + mn_v) / 2.0
    mn_w, mx_w = np.min(w), np.max(w)
    s_w = (mx_w - mn_w)
    avg_w = (mx_w + mn_w) / 2.0

    s = max(s_v,s_w)
    lim_v = [avg_v - s/2., avg_v + s/2.]
    lim_w = [avg_w - s/2., avg_w + s/2.]

    x = cv
    y = cw
    u = (gv - cv)
    v = (gw - cw)

    plt.quiver(x,y,u,v,color='k')

    plt.axvline(x=0.0, color='g', linestyle='-')
    plt.axhline(y=0.0, color='g', linestyle='-')

    plt.xlim(lim_v)
    plt.ylim(lim_w)


    plt.title('Command Velocity Discrepancy Field')
    plt.xlabel('v')
    plt.ylabel('w')
    plt.show()

if __name__ == '__main__':
    main()
