#!/usr/bin/env python3
import sys
sys.path.append('/home/innovation/robocon/src/robocon_node/scripts')

from ydlidar_x2 import YDLidarX2
import time

lidar = YDLidarX2()
if lidar.connect():
    print("✅ Connected to LiDAR")
    lidar.start_scan()
    time.sleep(1)

    data = lidar.get_data()
    print("📊 Sample data:", data[:10])

    lidar.stop()
    lidar.disconnect()
else:
    print("❌ Failed to connect")
