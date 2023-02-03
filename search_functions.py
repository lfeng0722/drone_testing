

import numpy as np
from scipy.spatial import cKDTree as KDTree

def cluster_data_KDTree(a, thr=0.1):
    t = KDTree(a)
    mask = np.ones(a.shape[:1], bool)
    idx = 0
    nxt = 1
    while nxt:
        mask[t.query_ball_point(a[idx], thr)] = False
        nxt = mask[idx:].argmax()
        mask[idx] = True
        idx += nxt
    return a[mask]


data1 = np.random.rand(10000,2)
out1 = cluster_data_KDTree(data1, thr=1)
print(out1)