#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from argparse import ArgumentParser
#from scipy.optimize import curve_fit
#from sklearn.linear_model import LinearRegression
#
#def func(x,
#        a,b,c,d,
#        e,f,g,h
#        ):
#    v, w = x
#    v2 = a*v+b*w + e*v*v+f*w*w
#    w2 = c*v+d*w + g*v*v+h*w*w
#    return np.stack([v2,w2], axis=0).ravel()
#
#def optimize(src, dst):
#    popt, pcov = curve_fit(func, src, dst)
#    return popt

def optimize_v2(src, dst):
    v, w = src

    A = np.array([
        v, w,
        v*v, v*w, w*w]).T
    #A = np.array([v, w]).T
    B = dst.T
    c,r = np.linalg.lstsq(A,B,rcond=None)[0:2]
    print 'hmm', c
    print 'hmm-r', r

    vex = 'v\' = {}v + {}w + {}v.v + {}v.w + {}w.w'.format(*c[:,0])
    wex = 'w\' = {}v + {}w + {}v.v + {}v.w + {}w.w'.format(*c[:,1])

    #vex = 'v\' = {}v + {}w'.format(*c[:,0])
    #wex = 'w\' = {}v + {}w'.format(*c[:,1])
    print vex
    print wex

    return lambda v,w : c.T.dot([v,w,v*v,v*w,w*w])
    #return lambda v,w : c.T.dot([v, w])


def main():
    # arguments
    parser = ArgumentParser()
    parser.add_argument('-f', '--file', default='/tmp/data.npy', help='Calibration Data Path')

    parser.add_argument('-r', '--rel', dest='rel', action='store_true', help='Relative Discrepancy (default : absolute)')
    parser.add_argument('-i', '--inv', dest='inv', action='store_true', help='Invert quiver source (gt_vel -> cmd_vel)')

    parser.set_defaults(rel=False, inv=False)

    opt = parser.parse_args()
    print('options : {}'.format(opt))

    # load file
    m = np.load(opt.file)
    gv, gw, cv, cw = m.T

    mfun = optimize_v2(
            np.stack([gv,gw], axis=0),
            np.stack([cv,cw], axis=0))
    gv, gw = mfun(gv,gw)


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

    if opt.inv:
        x = gv
        y = gw
        u = (cv - gv)
        v = (cw - gw)
    else:
        x = cv
        y = cw
        u = (gv - cv)
        v = (gw - cw)

    #print np.sort(np.linalg.norm(
    #    np.stack([u,v],axis=-1),axis=-1))[::-1]

    print np.sum(np.linalg.norm(
            np.stack([u,v],axis=-1),axis=-1))

    if opt.rel:
        u /= (x + 0.01*np.sign(x))
        v /= (y + 0.01*np.sign(y))

    plt.quiver(x,y,u,v,color='k')
    plt.grid()
    plt.gca().set_axisbelow(True)

    plt.axvline(x=0.0, color='g', linestyle='-')
    plt.axhline(y=0.0, color='g', linestyle='-')

    plt.xlim(lim_v)
    plt.ylim(lim_w)


    plt.title('Command Velocity Discrepancy Field' + (' (Rel)' if opt.rel else '') )
    plt.xlabel('v')
    plt.ylabel('$\omega$')
    plt.show()

if __name__ == '__main__':
    main()
