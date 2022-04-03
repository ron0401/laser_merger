#!/usr/bin/python2.7

import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import ros_numpy
import tf
import time
import tf.transformations as tfs

lp = lg.LaserProjection()
class laser_scan():
    def __init__(self,topic,base_frame):
        self.base_frame = base_frame
        rospy.Subscriber(topic, LaserScan, self.scan_callback, queue_size=1)
    
    def scan_callback(self,scan):
        self.point_cloud = lp.projectLaser(scan)
        self.scan = scan
        self.point_cloud_ary = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.point_cloud)

    def get_transformed_points(self,transform):
        try:
            ary = self.point_cloud_ary
            n = len(ary)
            ary = np.hstack([ary,np.zeros(n).reshape(n,1)])
            trans = tfs.translation_matrix(transform[0])
            rot = tfs.quaternion_matrix(transform[1])
            transform_matrix = np.dot(trans,rot)
            result = np.apply_along_axis(lambda x: np.dot(transform_matrix,x),1, ary)[0:, 0:3]
            return result
        except:
            pass       
        
class scan_merger():
    def __init__(self):
        self.scan_topics = rospy.get_param("/laser_merger/scan_list")
        self.base_frame = rospy.get_param("/laser_merger/base_frame")
        self.remove_low_points = rospy.get_param("/laser_merger/remove_low_points")
        self.remove_low_points_th = rospy.get_param("/laser_merger/remove_low_points_th")
        self.scans = []
        rospy.init_node('laser_merger')
        self.__pc_pub = rospy.Publisher(rospy.get_param("/laser_merger/publish_pc2_topic"), PointCloud2, queue_size=1)
        self.tf_listener = tf.TransformListener()
        for i in self.scan_topics:
            self.scans.append(laser_scan(i["topic_name"],self.base_frame))
        rospy.Timer(rospy.Duration(1.0 / float(rospy.get_param("/laser_merger/frequency"))), self.timer_callback)
        rospy.spin()

    def timer_callback(self,t):
        lst = []
        for scan in self.scans:
            try:
                transform = self.tf_listener.lookupTransform(self.base_frame, scan.point_cloud.header.frame_id, rospy.Time(0))
                lst.append(scan.get_transformed_points(transform))
            except:
                pass
        if len(lst) == 0:
            return
        head = Header()
        head.frame_id = self.base_frame
        head.stamp = rospy.Time.now()
        data = np.concatenate(lst)
        if self.remove_low_points:
            data = np.delete(data, np.where(data[0:,2:3] < self.remove_low_points_th),axis=0)
        msg = pc2.create_cloud_xyz32(head,data)
        self.__pc_pub.publish(msg)

if __name__ == '__main__':
    try:
        scan_merger()
    except rospy.ROSInterruptException: pass