#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud
from mrs_project_simulation.msg import Neighbours
import numpy as np
from typing import List
import rospkg
import math
import random

class CalcNeighboursNode():
    def __init__(self):
        rospy.loginfo("Calc neighbours node started")
        
        PUB_RATE = 10

        self.num_robots = rospy.get_param('~num_robots', default=3) #int
        self.switch = rospy.get_param('~switch', default= False)
        self.radius_value = rospy.get_param('~radius', default=0.2)
        
        package = rospkg.RosPack().get_path('mrs_project2_simulation')
        #adjacency matrix
        adj_mat_file = rospy.get_param('~adjacency_params', default="empty_triangle_random_adj")
        self.adjacency_matrix = np.loadtxt(package + f'/adjacency_params/{adj_mat_file}.txt', delimiter=" ", comments="#")

        self.publisher_neighbours = [rospy.Publisher(f"/robot_{i}/neighbours", Neighbours, queue_size=PUB_RATE) for i in range(self.num_robots)]
        self.publisher_obstacles = [rospy.Publisher(f"/robot_{i}/obstacles", PointCloud, queue_size=PUB_RATE) for i in range(self.num_robots)]
        self.obstacles = PointCloud()
        self.odoms: List[Odometry] = [None] * self.num_robots
        self.r = 1

        [rospy.Subscriber(f'/robot_{robot_id}/odom', Odometry, self.robots_odom_callback, callback_args=robot_id) for robot_id in range(self.num_robots)] #callback_args (any) - additional arguments to pass to the callback. This is useful when you wish to reuse the same callback for multiple subscriptions.
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1) #tu calc_neighbours_node publisha odom susjeda od ovog node-a
        self.rate = rospy.Rate(PUB_RATE) #frekvencija kojom publisha poruke, nece affectat to da missas poruke koje dobivas, ovo utjece samo na publishanje

        

    def robots_odom_callback(self, robot_odom, robot_id):
        self.odoms[robot_id] = robot_odom

    def map_callback(self, grid: OccupancyGrid):
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        points = PointCloud()
        for y in range(height):
            for x in range(width):
                index = x + y * width
                if grid.data[index] > 0:
                    point = Point()
                    point.x = origin_x + x * resolution
                    point.y = origin_y + y * resolution
                    points.points.append(point)

        self.obstacles = points
        
    def run(self):
        
        while not rospy.is_shutdown():
            neighbours_dict = {f"robot_{i}": [] for i in range(self.num_robots)}
            nearest_obs = {f"robot_{i}": PointCloud() for i in range(self.num_robots)}
            
            if None not in self.odoms: #ako je popunjena lista sa ne None vrijednostima tj popunjena je odom podacima
                for i in range(self.num_robots):
                    for j in range(self.num_robots):
                        if i == j:
                            continue
                        if self.adjacency_matrix[i, j]: #ako i-ti šalje j-tom svoju odometriju (poziciju) (ako self.adjacency_matrix[i, j] nije 0)
                            
                            if self.switch:
                                position_i = self.odoms[i].twist.twist.linear
                                position_j = self.odoms[j].twist.twist.linear
                                euclidean_distance = math.sqrt( (position_i.x - position_j.x)**2 + (position_i.y - position_j.y)**2 )
                                if euclidean_distance < self.radius_value:
                                    random_number1 = random.randint(0,9)
                                    random_number2 = random.randint(0,9)
                                    if random_number1 == random_number2:
                                        # simulira izgubljenu konekciju/paket
                                        #print(f"robot {i} & robot {j} lost connection!")
                                        pass
                                    else:
                                        #print(f"robot {i} & robot {j} have connetion!")
                                        self.odoms[i].child_frame_id = str(i) #da bi mogao vidjet ko je to poslao i onda dobit od njega zetu, a da ne moram mjenjat poruku i nes dodatno komplicirat
                                        neighbours_dict[f'robot_{j}'].append(self.odoms[i]) #j-ti robot prima odometriju od i-tog i uskladuje se (mice se) s obzirom na to
                                else:
                                    #print(f"robot {i} & robot {j} does not have connection!")
                                    pass
                            
                            else:
                                self.odoms[i].child_frame_id = str(i) #da bi mogao vidjet ko je to poslao i onda dobit od njega zetu, a da ne moram mjenjat poruku i nes dodatno komplicirat
                                neighbours_dict[f'robot_{j}'].append(self.odoms[i]) #j-ti robot prima odometriju od i-tog i uskladuje se (mice se) s obzirom na to

                    curr_odom = self.odoms[i]
                    obstacles = PointCloud()
                    curr_x, curr_y = curr_odom.pose.pose.position.x, curr_odom.pose.pose.position.y 
                    for obst in self.obstacles.points:
                        d = np.sqrt((obst.x - curr_x)**2 + (obst.y - curr_y)**2)
                        if d <= self.r: #unutar kruga
                            obstacles.points.append(obst)

                    nearest_obs[f"robot_{i}"] = obstacles
                                                

            #publish neighours
            for i, pub in enumerate(self.publisher_neighbours):
                msg = Neighbours()
                msg.neighbours_odoms = neighbours_dict[f"robot_{i}"]
                pub.publish(msg)
            
            for i, pub in enumerate(self.publisher_obstacles):
                pub.publish(nearest_obs[f"robot_{i}"])

            self.rate.sleep()
            pass

if __name__ == '__main__':
    rospy.init_node('calc_neighbours_node')
    try:
        calc_neighbours_node = CalcNeighboursNode()
        calc_neighbours_node.run()
    except rospy.ROSInterruptException: pass
