

import numpy as np
from scipy.spatial import cKDTree as KDTree

# def cluster_data_KDTree(a, thr=0.1):
#     t = KDTree(a)
#     mask = np.ones(a.shape[:1], bool)
#     idx = 0
#     nxt = 1
#     while nxt:
#         mask[t.query_ball_point(a[idx], thr)] = False
#         nxt = mask[idx:].argmax()
#         mask[idx] = True
#         idx += nxt
#     return a[mask]


def cluster_data_KDTree(a, b, thr=0.1):
    t = KDTree(a)
    mask = np.ones(a.shape[:1], bool)
    idx = 0
    nxt = 1
    avg_list = []
    while nxt:
        inds = t.query_ball_point(a[idx], thr)
        avg_list.append(np.mean(b[inds]))
        mask[inds] = False
        nxt = mask[idx:].argmax()
        mask[idx] = True
        idx += nxt
    max_ind = np.argmax(avg_list)
    center = a[mask][max_ind]
    return center

data1 = np.random.rand(20,2)
b =  np.random.rand(20,1)
out1 = cluster_data_KDTree(data1, b,thr=0.5)
print(out1)