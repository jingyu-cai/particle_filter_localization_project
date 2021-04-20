#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from likelihood_field import LikelihoodField

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample, choice
import math

from random import randint, random

from copy import deepcopy


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) 
    and returns yaw """

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for 
    gaussian and returns probability (likelihood) of observation """

    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist, 2)) / (2 * math.pow(sd, 2)))
    
    return prob


def draw_random_sample(n, list, prob):
    """ Draws a random sample of n elements from a given list of choices 
    and their specified probabilities. We recommend that you fill in this 
    function using random_sample. """

    # get an array of numbers that correspond to indices in the list with 
    #   a specific prob value
    prob_idxs = []
    for i in range(len(list)):
        if list[i] == prob:
            prob_idxs.append(int(i))

    # randomly sample n elements as indices to choose from prob_idxs
    random_nums = len(prob_idxs) * random_sample((n, ))
    random_nums = random_nums.astype(int)

    # return randomly chosen elements in prob_idxs as a new array, with
    #   each new element being the index in list that corresponds to coordinates
    #   with a specific color, such as light gray (inside the house)
    list_idxs = []
    for i in random_nums:
        list_idxs.append(prob_idxs[i])
    
    return list_idxs


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w
              

class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        

        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # create LikelihoodField object
        self.likelihood_field = LikelihoodField()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # sleep to get map data before initializing particle cloud
        rospy.sleep(1)

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True


    def get_map(self, data):

        self.map = data


    def initialize_particle_cloud(self):
        """ Initialize the particle cloud with random locations and 
        orientations throughout the house """
        
        # get map data and random indices that correspond to coordinates
        #   with a light gray color (inside the house)
        map_data = self.map.data
        random_indices = draw_random_sample(self.num_particles, map_data, 0)

        # initialize variables to convert from a particle's position to 
        #   its index on the Occupancy Grid 
        r = self.map.info.resolution
        x = self.map.info.origin.position.x
        y = self.map.info.origin.position.y
        w = self.map.info.width
        h = self.map.info.height
        
        for i in range(self.num_particles):
            # set pose data for particle
            p = Pose()
            p.position = Point()
            p.position.x = (random_indices[i] % w) * r + x
            p.position.y = (random_indices[i] // h) * r + y
            p.position.z = 0
            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, math.radians(360 * random_sample()))
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            # initialize the new particle, where all will have the same weight (1.0)
            new_particle = Particle(p, 1.0)
            
            # append the particle to the particle cloud
            self.particle_cloud.append(new_particle)

        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        """ Normalize the particle weights so they add up to 1 """

        # initialize sum variable to normalize
        sum = 0

        # add up all the particle weights into sum
        for part in self.particle_cloud:
            sum += part.w

        # reassign weights by dividing each particle's original weight by sum
        for part in self.particle_cloud:
            part.w = part.w / sum

        return


    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)


    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)


    def resample_particles(self):
        """ Resample particles with probabilities proportionate to their weights """

        # create an array of particle weights
        particle_weights = []
        for part in self.particle_cloud:
            particle_weights.append(part.w)

        # sample particles with probabilities proportinate to their weights
        new_particle_cloud = choice(self.particle_cloud, self.num_particles, p = particle_weights)

        # deepcopy these newly sampled particles into particle_cloud
        for i in range(self.num_particles):
            self.particle_cloud[i] = deepcopy(new_particle_cloud[i])
        
        return


    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return

        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose


    def update_estimated_robot_pose(self):
        """ Based on the particles within the particle cloud, update the 
        robot pose estimate """
        
        # initialize sum variables to average
        px = py = pz = 0 
        ox = oy = oz = ow = 0

        for part in self.particle_cloud:
            p = part.pose
            px += p.position.x
            py += p.position.y
            pz += p.position.z

            ox += p.orientation.x
            oy += p.orientation.y
            oz += p.orientation.z
            ow += p.orientation.w

        # update estimated robot pose by taking the average of all particle poses 
        num = len(self.particle_cloud)
        self.robot_estimate.position.x = px/num
        self.robot_estimate.position.y = py/num
        self.robot_estimate.position.z = pz/num
        self.robot_estimate.orientation.x = ox/num
        self.robot_estimate.orientation.y = oy/num
        self.robot_estimate.orientation.z = oz/num
        self.robot_estimate.orientation.w = ow/num

        return

    
    def update_particle_weights_with_measurement_model(self, data):
        """ Update the particle weights using the likelihood field for
        range finders model """
        
        # wait until initialization is complete
        if not(self.initialized):
            return

        # take into account 8 directions of data
        cardinal_directions_idxs = [0, 45, 90, 135, 180, 225, 270, 315]

        # compute the new probabilities for all particles based on model
        for part in self.particle_cloud:
            q = 1
            for ang in cardinal_directions_idxs:
                ztk = data.ranges[ang]
                if ztk > 3.5:
                    ztk = 3.5
                theta = get_yaw_from_pose(part.pose)
                x_ztk = part.pose.position.x + ztk * math.cos(theta + math.radians(ang))
                y_ztk = part.pose.position.y + ztk * math.sin(theta + math.radians(ang))
                dist = self.likelihood_field.get_closest_obstacle_distance(x_ztk, y_ztk)
                q = q * (1 * compute_prob_zero_centered_gaussian(dist, 0.1))
            if math.isnan(q) or q == 0:
                q = 1 * 10**(-100)
            part.w = q

        return
        

    def update_particles_with_motion_model(self):
        """ Calculates how much the robot has moved using odometry and
        move all the particles correspondingly by the same amount """

        # calculate how the robot has moved
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        dx = curr_x - old_x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        dy = curr_y - old_y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        dyaw = curr_yaw - old_yaw

        # move all the particles correspondingly
        for part in self.particle_cloud:          
            p = part.pose
            p.position.x += dx
            p.position.y += dy
            new_yaw = get_yaw_from_pose(p) + dyaw
            q = quaternion_from_euler(0.0, 0.0, new_yaw)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
        
        return


if __name__=="__main__":

    pf = ParticleFilter()

    rospy.spin()









