#!/usr/bin/env python

import pcl
import rospy
from constants import *
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray
import ros_numpy

CYLINDER_CLUSTERING = True # else box clustering

def pcl_cb(msg):
    rospy.loginfo("Got point cloud.")
    if CYLINDER_CLUSTERING:
        cluster_cylinders(msg)
    else:
        cluster_boxes(msg)

def cluster_cylinders(msg):
    # convert to pcl data
    pc = ros_numpy.numpify(msg)
    points = np.zeros((pc.shape[0], 3), dtype=np.float32)
    points[:, 0] = pc["x"]
    points[:, 1] = pc["y"]
    points[:, 2] = pc["z"]
    pc = pcl.PointCloud(np.array(points, dtype=np.float32))
    ne = pc.make_NormalEstimation()
    tree = pc.make_kdtree()
    ne.set_SearchMethod(tree)
    ne.set_KSearch(50)

    seg = pc.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_CYLINDER)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(10000)
    seg.set_distance_threshold(0.05)
    seg.set_radius_limits(0, 0.1)
    seg.set_axis([0, 0, 1])
    seg.set_eps_angle(0.523599)

    inliers, coefficients = seg.segment()
    print(coefficients)
    cloud_cylinder = pc.extract(inliers, negative=False)

    # convert to ROS message
    ros_cloud_cylinder = ros_numpy.msgify(PointCloud2, cloud_cylinder)
    ros_cloud_cylinder.header = msg.header
    # publish ROS message
    pub.publish(ros_cloud_cylinder)

def cluster_boxes(msg):
    vox = msg.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    downsampled = vox.filter()
    ec = downsampled.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    tree = downsampled.make_kdtree()
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()


def outlier_filtering(pcl_data, mean_k=50, std_dev=1.0):
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(std_dev)
    return outlier_filter.filter()

if __name__ == "__main__":
    rospy.init_node("clustering", anonymous=True)
    pcl_sub = rospy.Subscriber(PC_TOPIC, PointCloud2, pcl_cb, queue_size=1)
    pub = rospy.Publisher("/cylinder", PointCloud2, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()