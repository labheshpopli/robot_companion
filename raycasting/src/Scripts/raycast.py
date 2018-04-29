#!/usr/bin/env python

"""
This code is not tested. Please test before use.
"""

import rospy
import tf
import math
import sys
import time
import os
import os.path

from geometry_msgs.msg import PoseStamped, Twist 
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionFeedback
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

class raycasting():

    def __init__(self):
        self._count = 0
        self._dist = 0
    def precasting(minx, miny, xw, yw, xyreso, yawreso):
    
        precast = [[] for i in range(int(round((math.pi * 2.0) / yawreso)) + 1)]
    
        for ix in range(xw):
            for iy in range(yw):
                px = ix * xyreso + minx
                py = iy * xyreso + miny
    
                d = math.sqrt(px**2 + py**2)
                angle = angleAtan(py, px)
                angleid = int(math.floor(angle / yawreso))
    
                pc = precastDB()
    
                pc.px = px
                pc.py = py
                pc.d = d
                pc.ix = ix
                pc.iy = iy
                pc.angle = angle
    
                precast[angleid].append(pc)
    
        return precast
    
    def calc_grid_map_config(ox, oy, xyreso):
        minx = round(min(ox) - EXTEND_AREA / 2.0)
        miny = round(min(oy) - EXTEND_AREA / 2.0)
        maxx = round(max(ox) + EXTEND_AREA / 2.0)
        maxy = round(max(oy) + EXTEND_AREA / 2.0)
        xw = int(round((maxx - minx) / xyreso))
        yw = int(round((maxy - miny) / xyreso))
    
        return minx, miny, maxx, maxy, xw, yw
        
    def angleAtan(y, x):
        angle = math.atan2(y, x)
        if angle < 0.0:
            angle += math.pi * 2.0
    
        return angle
        
    def generate_ray_casting_grid_map(ox, oy, xyreso, yawreso):
    
        minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(ox, oy, xyreso)
    
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]
    
        precast = precasting(minx, miny, xw, yw, xyreso, yawreso)
    
        for (x, y) in zip(ox, oy):
    
            d = math.sqrt(x**2 + y**2)
            angle = angleAtan(y, x)
            angleid = int(math.floor(angle / yawreso))
    
            gridlist = precast[angleid]
    
            ix = int(round((x - minx) / xyreso))
            iy = int(round((y - miny) / xyreso))
    
            for grid in gridlist:
                if grid.d > d:
                    pmap[grid.ix][grid.iy] = 0.5
    
            pmap[ix][iy] = 1.0
    
        return pmap, minx, maxx, miny, maxy, xyreso
    
            
if __name__ == "__main__":
    rospy.init_node('raycasting', anonymous=True)
    rate = rospy.Rate(1)
    raycast = raycasting()
    rospy.Subscriber('/nav_msgs/OccupancyGrid', OccupancyGridArray, raycast.check_status)
     
     xyreso = 0.25  # x-y grid resolution [m]
    yawreso = math.radians(10.0)  # yaw angle resolution [rad]

    for i in range(5):
        # ONLY FOR TESTING: Get these vals from grid map
        ox = (np.random.rand(4) - 0.5) * 10.0 
        oy = (np.random.rand(4) - 0.5) * 10.0
        while not rospy.is_shutdown():
            pmap, minx, maxx, miny, maxy, xyreso = raycast.generate_ray_casting_grid_map(ox, oy, xyreso, yawreso)
            rospy.publish(pmap)
    
    
    
        

