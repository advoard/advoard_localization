#!/usr/bin/env python
'''
__author__ = "Bekir Bostanci"
__license__ = "BSD"
__version__ = "0.0.1"
__maintainer__ = "Bekir Bostanci"
__email__ = "bekirbostanci@gmail.com"
'''

import rospy 


import numpy as np
import math
import time

import matplotlib.pyplot as plt
from scipy.spatial import distance


def error_calculation(ppoints, cpoints):
    spoint_count = 0 
    inds = 0

    ppoints = ppoints.T     #map data
    cpoints = cpoints.T     #lidar data
    
    #for each lidar data point 
    for i in cpoints:       
        #calculate the lidar point distance of map array 
        d =distance.cdist([(i)], ppoints, 'euclidean') 
        #get the minimum distance on the list  
        mind = min(d[0])
        inds += mind
        if mind<50:
            #matched point
            spoint_count += 1 
        
        
    return inds, spoint_count

def root_finding(ppoints,cx,cy, current_degree, robot_pos_x,robot_pos_y):
    motion = [robot_pos_x, robot_pos_y, np.deg2rad(current_degree)]  # movement [x[m],y[m],yaw[deg]]
    zx = [math.cos(motion[2]) * x - math.sin(motion[2]) * y + motion[0] for (x, y) in zip(cx, cy)]
    zy = [math.sin(motion[2]) * x + math.cos(motion[2]) * y + motion[1] for (x, y) in zip(cx, cy)]
    
    cpoints =np.vstack((zx,zy))
    
    error, spoint_count =error_calculation(ppoints,cpoints)
    
    """
    #visualization of lidar and map data
    plt.cla()
    plt.plot(ppoints[0, :], ppoints[1, :], ".r")
    plt.plot(cpoints[0, :], cpoints[1, :], ".b")
    plt.plot(0.0, 0.0, "xr")
    plt.axis("equal")
    plt.pause(0.1)
    """

    
    return error, spoint_count


def main(processed_lidar,processed_map,robot_pos_x,robot_pos_y):
    tic = time.clock()          #initial time 

    """
    # previous points
    px = processed_map[:,0]
    py = processed_map[:,1]

    
    ppoints =np.vstack((px,py))
    """
    ppoints =np.array(processed_map).T
    processed_lidar = np.array(processed_lidar).T
    cx = processed_lidar[0]     #lidar x coordinates value 
    cy = processed_lidar[1]     #lidar y coordinates value
   
    current_degree = 0 
    pre_spoint_count = -1 
    final_sponit = 0            #count of final successful point 
    final_degree = 0            
    spoint_count = 0            #count of successful point
    tpoint_count = len(cx)      #count of total point 

    for i in range(360):
        #current_degree = (max_degree+min_degree)/2
        current_val = 0 
         
        if i%10 == 0:
            current_degree = i 
            current_val , spoint_count =  root_finding(ppoints,cx,cy,i,robot_pos_x,robot_pos_y) 
        
        #compare the best one 
        if final_sponit< spoint_count : 
            final_sponit = spoint_count
            final_degree = i 

            """
            #show iteration result 
            print("")
            print("Iteration : "+ str(i))
            print("Current degree : "+str(current_val))
            print("Total point : "+str(tpoint_count))
            print("Sucessful point : "+str(spoint_count))
            """

        pre_spoint_count = spoint_count
        
    toc = time.clock()          #finish time

    result_log = [] 
    result_log.append("Final Result")
    result_log.append("Current rotated degree : "+str(final_degree))
    result_log.append("Total point : "+str(tpoint_count))
    result_log.append("Sucessful point : "+str(final_sponit))
    result_log.append("Time :"+str(toc - tic))
    rospy.loginfo(result_log)


    return final_degree

if __name__ == '__main__':
    """
    #use for saved data example
    processed_lidar = np.load('lidar.npy')
    processed_map = np.load('map.npy')
    robot_pos_x = -580
    robot_pos_y = -1364
    main(processed_lidar,processed_map,robot_pos_x,robot_pos_y)
    """
   