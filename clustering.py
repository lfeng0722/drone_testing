import numpy as np
from sklearn.cluster import KMeans

# 构造数据
X = np.random.rand(5,2)
def clustering(threshold,coor_set,confi_set):

    # 设置聚类数目为1
    kmeans = KMeans(n_clusters=1, random_state=0).fit(coor_set)
    error = kmeans.inertia_

    # 循环不断增加聚类数目并计算误差，直到误差小于阈值
    while error > threshold:
        kmeans = KMeans(n_clusters=kmeans.n_clusters + 1, random_state=0).fit(coor_set)
        error = kmeans.inertia_

    # 获取聚类结果
    labels = kmeans.labels_

    print("Cluster labels:", labels)

    # 获取聚类中心
    cluster_centers = kmeans.cluster_centers_
    print("Cluster centers:", cluster_centers)