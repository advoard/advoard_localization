#!/usr/bin/env python3

import rospy 
import rospkg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import tf 

from pozyx_simulation.msg import  uwb_data
import math

import numpy as np
import scipy.stats

import time


rospy.init_node('particle_filter_UWB_node', anonymous=True)
pub = rospy.Publisher('localization_data_topic', PointStamped, queue_size=10)
r = rospy.Rate(1)

global particles
global weights


#add random seed for generating comparable pseudo random numbers
#np.random.seed(123)

def initialize_particles(num_particles, map_limits):
    # randomly initialize the particles inside the map limits

    created_particles = []

    for i in range(num_particles):
        particle = dict()

        # draw x,y and theta coordinate from uniform distribution
        # inside map limits
        particle['x'] = np.random.uniform(map_limits[0], map_limits[1])
        particle['y'] = np.random.uniform(map_limits[2], map_limits[3])
        #particle['theta'] = np.random.uniform(-np.pi, np.pi)
        particle['theta'] = 0 + np.random.uniform(-np.pi, np.pi) 

        created_particles.append(particle)
    
    return created_particles

def mean_pose():
    # calculate the mean pose of a particle set.
    #
    # for x and y, the mean position is the mean of the particle coordinates
    #
    # for theta, we cannot simply average the angles because of the wraparound 
    # (jump from -pi to pi). Therefore, we generate unit vectors from the 
    # angles and calculate the angle of their average 

    # save x and y coordinates of particles
    xs = []
    ys = []

    # save unit vectors corresponding to particle orientations 
    vxs_theta = []
    vys_theta = []

    toplam_nokta  = 0
    """
    for i in range(len(particles)):
        if weights[i]>0.001: 
            toplam_nokta = toplam_nokta +1  
            xs.append(particles[i]['x'])
            ys.append(particles[i]['y'])

            #make unit vector from particle orientation
            vxs_theta.append(np.cos(particles[i]['theta']))
            vys_theta.append(np.sin(particles[i]['theta']))
    print("toplam :" + str(toplam_nokta))
    """


    for particle in particles:
        xs.append(particle['x'])
        ys.append(particle['y'])

        #make unit vector from particle orientation
        vxs_theta.append(np.cos(particle['theta']))
        vys_theta.append(np.sin(particle['theta']))


    #calculate average coordinates
    mean_x = np.mean(xs)
    mean_y = np.mean(ys)
    mean_theta = np.arctan2(np.mean(vys_theta), np.mean(vxs_theta))

    return [mean_x, mean_y, mean_theta]

def sample_motion_model(Odometry):
    global particles
    # Samples new particle positions, based on old positions, the odometry measurements and the motion noise
    # (probabilistic motion models slide 27)


    delta_vel = Odometry.twist.twist.linear.x/30         # redefine r1                  odom=>twist=>linear=>x 
    delta_w = Odometry.twist.twist.angular.z/30                 # redefine t                     odom=>twist=>angular=>z
    

    # "move" each particle according to the odometry measurements plus sampled noise to generate new particle set

    new_particles = []
    for particle in particles:
        new_particle = dict()
        # sample noisy motions
        noise_x = (np.random.uniform() - 0.5) * abs(delta_vel)
        #noise_y = (np.random.uniform()-0.5) * abs(delta_vel)
        noise_theta = (np.random.uniform() - 0.5) * abs(delta_w)
        noisy_delta_vel = delta_vel + noise_x
        #noisy_delta_vel_y = delta_vel + noise_y
        noisy_delta_w = delta_w + noise_theta
        # calculate new particle pose
        new_particle['x'] = particle['x'] + noisy_delta_vel * np.cos(particle['theta']) 
        new_particle['y'] = particle['y'] + noisy_delta_vel * np.sin(particle['theta']) 
        new_particle['theta'] = particle['theta'] + noisy_delta_w  
        new_particles.append(new_particle)

    particles = new_particles
    


def eval_sensor_model():
    global weights
    global particles
    # Computes the observation likelihood of all particles, given the particle and landmark positions and sensor measurements
    # (probabilistic sensor models slide 33)
    #
    # The employed sensor model is range only.

    sigma_r = 0.5

    while not rospy.is_shutdown(): 
        #measured landmark ids and ranges
        ids = g_uwb_data.destination_id
        ranges = g_uwb_data.distance
        
        weights_new = []

        # rate each particle
        for particle in particles:
            all_meas_likelihood = 1.0  # for combining multiple measurements
            # loop for each observed landmark
            for i in range(len(ids)):
                lm_id = ids[i]
                meas_range = ranges[i]
                lx = sensor_pos[i][0]
                ly = sensor_pos[i][1]
                lz = sensor_pos[i][2]
                #calculate expected range 
                px = particle['x']
                py = particle['y']
                pz = 0
                ptheta = particle['theta']
                # calculate expected range measurement
                meas_range_exp = np.sqrt((lx - px) ** 2 + (ly - py) ** 2 + (lz - pz) ** 2)
                # evaluate sensor model (probability density function of normal distribution)
                meas_likelihood = scipy.stats.norm.pdf(meas_range, meas_range_exp, sigma_r)
                #print(meas_likelihood)
                # combine (independent) measurements
                all_meas_likelihood = all_meas_likelihood * meas_likelihood
            
            weights_new.append(all_meas_likelihood)

        #normalize weights_new
        normalizer = sum(weights_new)
        weights = weights_new / normalizer

        #print("max weights :" + str(max(weights)))
        resample_particles()
        [mean_x, mean_y, mean_theta] = mean_pose()
        publish_data(mean_x,mean_y)

def resample_particles():
    global particles
    global weights
    # Returns a new set of particles obtained by performing stochastic universal sampling, according to the particle weights.

    new_particles = []
    #new_weights = [] 
    # distance between pointers
    step = 1.0 / len(particles)
    # random start of first pointer
    u = np.random.uniform(0, step)
    # where we are along the weights
    c = weights[0]
    # index of weight container and corresponding particle
    i = 0
    # loop over all particle weights
    for particle in particles:
    # go through the weights until you find the particle to which the pointer points
        while u > c:
            i = i + 1
            c = c + weights[i]
        # add that particle
        new_particles.append(particles[i])
        #new_weights.append(weights[i])
        # increase the threshold
        u = u + step

    #weights = new_weights
    particles = new_particles

def subscribe_odom_data(Odometry):
    sample_motion_model(Odometry)

def subscribe_uwb_data(uwb_data):
    global g_uwb_data
    g_uwb_data = uwb_data

    #eval_sensor_model(uwb_data,sensor_pos)
    #particles = resample_particles()

def publish_data(pose_x,pose_y):
    robot_pos = PointStamped()
    robot_pos.header.stamp = rospy.Time.now() 
    robot_pos.header.frame_id = "map" 

    robot_pos.point.x = float(pose_x)
    robot_pos.point.y = float(pose_y)
    robot_pos.point.z = 0.0

    pub.publish(robot_pos)

def get_anchors_pos():
    max_anchor = 100
    sensor_pos = []   
    uwb_id = 'uwb_anchor_'
    listener = tf.TransformListener()
    
    for i in range(max_anchor):
        try:
            time.sleep(0.3)
            (trans,rot) = listener.lookupTransform('/map', uwb_id+str(i), rospy.Time(0))
            sensor_pos.append(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if(i == 0):
                rospy.INFO("Firstly have to run pozyx_simulation package uwb_simulation.py file")
            break

    return sensor_pos


if __name__ == "__main__":
    global particles
    global sensor_pos
    #get uwb anchors postion
    sensor_pos = get_anchors_pos()

    #1800 0
    map_limits = [-3,3,-3,3]
    particles = initialize_particles(1000, map_limits)


    rospy.Subscriber("odom", Odometry, subscribe_odom_data)
    rospy.Subscriber("uwb_data_topic", uwb_data, subscribe_uwb_data)

    time.sleep(2)
    eval_sensor_model()
    rospy.spin()

