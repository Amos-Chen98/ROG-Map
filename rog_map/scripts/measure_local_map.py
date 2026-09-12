#!/usr/bin/env python3
"""Measure scan-to-map latency on a running (preferably isolated) ROS graph."""
import argparse
import json
import statistics
import threading
import time
import xmlrpc.client
import rospy
import rosnode
import rosgraph
import psutil
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray
from rog_map_msgs.msg import LocalMap
from octomap_msgs.msg import Octomap

parser = argparse.ArgumentParser()
parser.add_argument('--duration', type=float, default=85)
parser.add_argument('--warmup', type=float, default=10)
parser.add_argument('--cloud', default='/dragon/cloud_denoised')
parser.add_argument('--map', default='/dragon/rog_map/local_map')
parser.add_argument('--node', default='/dragon/rog_map')
parser.add_argument('--old-octomap', action='store_true')
parser.add_argument('--output', required=True)
args = parser.parse_args()
rospy.init_node('measure_local_map', anonymous=True)
lock = threading.Lock()
started = time.monotonic()
clouds, maps, latency, sizes, diagnostics = [], [], [], [], []
scenes, scene_latency = [], []
receipt = {}

def cloud_cb(msg):
    now = time.monotonic()
    with lock:
        receipt[msg.header.stamp.to_nsec()] = now
        if len(receipt) > 3000:
            receipt.pop(next(iter(receipt)))
        if now-started >= args.warmup:
            clouds.append(now)

def map_cb(msg):
    now = time.monotonic()
    with lock:
        if now-started < args.warmup:
            return
        maps.append(now)
        before = receipt.get(msg.header.stamp.to_nsec())
        if before is not None:
            latency.append(1000*(now-before))
        sizes.append(len(msg.data) if args.old_octomap else len(msg.occupied_bits)+len(msg.inflated_bits))

def scene_cb(msg):
    now = time.monotonic()
    with lock:
        if now-started < args.warmup:
            return
        scenes.append(now)
        before = receipt.get(msg.stamp.to_nsec())
        if before is not None:
            scene_latency.append(1000*(now-before))

def diag_cb(msg):
    with lock:
        if time.monotonic()-started >= args.warmup:
            for status in msg.status:
                diagnostics.append({v.key: float(v.value) for v in status.values})

subscribers = [
    rospy.Subscriber('/dragon/planning/map_ready', Header, scene_cb, queue_size=10, tcp_nodelay=True),
    rospy.Subscriber(args.cloud, PointCloud2, cloud_cb, queue_size=10, buff_size=2**24, tcp_nodelay=True),
    rospy.Subscriber(args.map, Octomap if args.old_octomap else LocalMap, map_cb, queue_size=1, buff_size=2**26, tcp_nodelay=True),
    rospy.Subscriber('/dragon/rog_map/diagnostics', DiagnosticArray, diag_cb, queue_size=10),
    rospy.Subscriber('/dragon/planning/map_diagnostics', DiagnosticArray, diag_cb, queue_size=10),
]
process = None
cpu, rss = [], []
while not rospy.is_shutdown() and time.monotonic()-started < args.duration:
    if process is None:
        try:
            uri = rosnode.get_api_uri(rosgraph.Master(rospy.get_name()), args.node)
            pid = xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())[2]
            process = psutil.Process(pid)
            process.cpu_percent()
        except Exception:
            pass
    elif process.is_running():
        usage = process.cpu_percent()
        if time.monotonic()-started >= args.warmup:
            cpu.append(usage)
            rss.append(process.memory_info().rss)
    time.sleep(.5)

def distribution(values):
    if not values:
        return None
    ordered = sorted(values)
    return {'count': len(values), 'p50': statistics.median(values),
            'p95': ordered[min(len(ordered)-1, int(.95*len(ordered)))], 'max': max(values)}

def hz(values):
    return (len(values)-1)/(values[-1]-values[0]) if len(values)>1 else 0

with lock:
    result = {'process_node': args.node, 'cloud_hz': hz(clouds), 'map_hz': hz(maps), 'cloud_count': len(clouds),
              'map_count': len(maps), 'scene_hz': hz(scenes),
              'cloud_receive_to_scene_notification_ms': distribution(scene_latency), 'cloud_receive_to_map_receive_ms': distribution(latency),
              'payload_bytes': distribution(sizes), 'mapper_cpu_percent': distribution(cpu),
              'mapper_rss_bytes': distribution(rss), 'diagnostics': diagnostics}
with open(args.output, 'w') as output:
    json.dump(result, output, indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='diagnostics'}, indent=2))
