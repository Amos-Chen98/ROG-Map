#!/usr/bin/env python3
import math
import time
import unittest
import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from rog_map_msgs.msg import LocalMap
from std_srvs.srv import Empty

class LocalMapNodeTest(unittest.TestCase):
    def test_transform_invalid_scan_and_reset(self):
        maps = []
        sub = rospy.Subscriber('/map_test/rog_map/local_map', LocalMap, maps.append, queue_size=100)
        pub = rospy.Publisher('/map_test/cloud', PointCloud2, queue_size=1)
        deadline = time.monotonic()+10
        while not pub.get_num_connections() and time.monotonic()<deadline:
            time.sleep(.02)
        self.assertTrue(pub.get_num_connections())
        cloud = point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='map_test/cloud_frame'),[(.5,0,0)]*3+[(float('nan'),0,0),(0,float('inf'),0)])
        # Transformed obstacle at world x=1.5, not the raw coordinate x=.5.
        pub.publish(cloud)
        deadline = time.monotonic()+5
        while not maps and time.monotonic()<deadline:
            time.sleep(.02)
        self.assertTrue(maps)
        first=maps[-1]
        self.assertEqual(first.header.stamp,cloud.header.stamp)
        def occupied(m,x,y,z):
            ids=[math.floor((p-o)/m.resolution) for p,o in zip((x,y,z),(m.origin.x,m.origin.y,m.origin.z))]
            i=ids[0]+m.size[0]*(ids[1]+m.size[1]*ids[2])
            return bool(m.occupied_bits[i//8] & (1<<(i%8)))
        self.assertTrue(occupied(first,1.55,.05,.05))
        self.assertFalse(occupied(first,.55,.05,.05))
        count=len(maps)
        cloud.header.frame_id='missing_sensor_frame'
        pub.publish(cloud)
        time.sleep(.2)
        self.assertEqual(len(maps),count)
        rospy.wait_for_service('/map_test/rog_map/reset',timeout=5)
        rospy.ServiceProxy('/map_test/rog_map/reset',Empty)()
        deadline=time.monotonic()+3
        while len(maps)==count and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertGreater(maps[-1].epoch,first.epoch)
        self.assertFalse(any(maps[-1].occupied_bits))
        self.assertFalse(any(maps[-1].inflated_bits))
        cloud.header=Header(stamp=rospy.Time.now(),frame_id='map_test/cloud_frame')
        pub.publish(cloud)
        deadline=time.monotonic()+3
        while maps[-1].version==1 and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertTrue(occupied(maps[-1],1.55,.05,.05))

        # Both historical and newer transforms exist. The scan must use the
        # historical transform, even though a latest-pose lookup would succeed.
        broadcaster = tf2_ros.TransformBroadcaster()
        historical = TransformStamped()
        historical.header = Header(stamp=rospy.Time.now()-rospy.Duration(1),frame_id='world')
        historical.child_frame_id = 'map_test/moving_cloud'
        historical.transform.translation.x = -1
        historical.transform.rotation.w = 1
        newer = TransformStamped()
        newer.header = Header(stamp=historical.header.stamp+rospy.Duration(.5),frame_id='world')
        newer.child_frame_id = historical.child_frame_id
        newer.transform.translation.x = 1
        newer.transform.rotation.w = 1
        deadline = time.monotonic()+5
        while not broadcaster.pub_tf.get_num_connections() and time.monotonic()<deadline:
            time.sleep(.02)
        self.assertTrue(broadcaster.pub_tf.get_num_connections())
        broadcaster.sendTransform([historical,newer])
        time.sleep(1.1)
        cloud.header = Header(stamp=historical.header.stamp,frame_id=historical.child_frame_id)
        count = len(maps)
        pub.publish(cloud)
        deadline = time.monotonic()+3
        while len(maps)==count and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertGreater(len(maps),count)
        self.assertEqual(maps[-1].header.stamp,historical.header.stamp)
        self.assertTrue(occupied(maps[-1],-.45,.05,.05))
        self.assertFalse(occupied(maps[-1],1.55,.05,.05))

if __name__=='__main__':
    rospy.init_node('local_map_node_test')
    rostest.rosrun('rog_map','local_map_node_test',LocalMapNodeTest)
