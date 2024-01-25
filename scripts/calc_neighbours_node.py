#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from mrs_project_simulation.msg import Neighbours
import numpy as np
from typing import List
import rospkg

class CalcNeighboursNode():
    def __init__(self):
        rospy.loginfo("Calc neighbours node started")
        
        PUB_RATE = 10

        self.num_robots = rospy.get_param('~num_robots', default=3) #int
        package = rospkg.RosPack().get_path('mrs_project2_simulation')
        adj_mat_file = rospy.get_param('~adjacency_params', default="empty_triangle_random_adj")
        self.adjacency_matrix = np.loadtxt(package + f'/adjacency_params/{adj_mat_file}.txt', delimiter=" ", comments="#")

        self.publisher_neighbours = [rospy.Publisher(f"/robot_{i}/neighbours", Neighbours, queue_size=PUB_RATE) for i in range(self.num_robots)]
        self.odoms: List[Odometry] = [None] * self.num_robots

        [rospy.Subscriber(f'/robot_{robot_id}/odom', Odometry, self.robots_odom_callback, callback_args=robot_id) for robot_id in range(self.num_robots)] #callback_args (any) - additional arguments to pass to the callback. This is useful when you wish to reuse the same callback for multiple subscriptions.
        self.rate = rospy.Rate(PUB_RATE) #frekvencija kojom publisha poruke, nece affectat to da missas poruke koje dobivas, ovo utjece samo na publishanje


    def robots_odom_callback(self, robot_odom, robot_id):
        self.odoms[robot_id] = robot_odom
        
    def run(self):
        while not rospy.is_shutdown():
            neighbours_dict = {f"robot_{i}": [] for i in range(self.num_robots)}
            
            if None not in self.odoms: #ako je popunjena lista sa ne None vrijednostima tj popunjena je odom podacima
                for i in range(self.num_robots):
                    for j in range(self.num_robots):
                        if i == j:
                            continue

                        if self.adjacency_matrix[i, j]: #ako i-ti šalje j-tom svoju odometriju (poziciju) (ako self.adjacency_matrix[i, j] nije 0)
                            self.odoms[i].child_frame_id = str(i) #da bi mogao vidjet ko je to poslao i onda dobit od njega zetu, a da ne moram mjenjat poruku i nes dodatno komplicirat
                            neighbours_dict[f'robot_{j}'].append(self.odoms[i]) #j-ti robot prima odometriju od i-tog i uskladuje se (mice se) s obzirom na to


            #publish neighours
            for i, pub in enumerate(self.publisher_neighbours):
                msg = Neighbours()
                msg.neighbours_odoms = neighbours_dict[f"robot_{i}"]
                pub.publish(msg)


            self.rate.sleep()
            pass

if __name__ == '__main__':
    rospy.init_node('calc_neighbours_node')
    try:
        calc_neighbours_node = CalcNeighboursNode()
        calc_neighbours_node.run()
    except rospy.ROSInterruptException: pass