#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from mrs_project_simulation.msg import Neighbours
from mrs_project2_simulation.srv import SetGoalPosition
import numpy as np
import rospkg
import math

class AgentNode():
    def __init__(self):
        self.PUB_RATE = 10

        #ovo dobiva od odometrije
        self.x = 0 
        self.y = 0 
        self.vel_x = 0 
        self.vel_y = 0 

        self.initial_pos_set = False
        
        package = rospkg.RosPack().get_path('mrs_project2_simulation')
        formation_positions_param = rospy.get_param('~formation_positions_param', default="empty_triangle_random_form")
        robot_num = int(rospy.get_name()[-1]) #robot_0 -> 0
        self.formations = np.loadtxt(package + f"/formation_positions/{formation_positions_param}.txt", delimiter=" ", comments="#")
        self.formation_position_x = self.formations[robot_num, 0]
        self.formation_position_y = self.formations[robot_num, 1]
        self.avoidance_factor = 0.25

        self.is_leader = rospy.get_param('~is_leader', default=False)
        if self.is_leader:
            self.goal_position_x = self.formation_position_x
            self.goal_position_y = self.formation_position_y
            self.goal_pos_service = rospy.Service('set_goal_position', SetGoalPosition, self.handle_set_goal_position)

        self.neighbours_odoms = []
        self.obstacles = PointCloud()
        self.publisher_vel = rospy.Publisher(f"{rospy.get_name()}/cmd_vel", Twist, queue_size=self.PUB_RATE) #ovo uzima stage i pomice robota,, publisha svoj cmd_vel
        rospy.Subscriber(f"{rospy.get_name()}/odom", Odometry, self.odom_callback, queue_size=1) #stage publisha na ovaj topic
        rospy.Subscriber(f"{rospy.get_name()}/neighbours", Neighbours, self.neighbours_callback, queue_size=1) #tu calc_neighbours_node publisha odom susjeda od ovog node-a
        rospy.Subscriber(f"{rospy.get_name()}/obstacles", PointCloud, self.map_callback, queue_size=1)
        self.rate = rospy.Rate(self.PUB_RATE) #frekvencija kojom publisha poruke, nece affectat to da missas poruke koje dobivas, ovo utjece samo na publishanje

    def handle_set_goal_position(self, request):
        # Save the goal position from the service request
        self.goal_position_x = request.x
        self.goal_position_y = request.y
        return True

    def odom_callback(self, odom_msg):
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        self.vel_x = odom_msg.twist.twist.linear.x
        self.vel_y = odom_msg.twist.twist.linear.y

    def neighbours_callback(self, neighbours: Neighbours):
        self.neighbours_odoms = neighbours.neighbours_odoms #Neighbours je lista Odometry poruka

    def map_callback(self, obstacles: PointCloud):
        self.obstacles = obstacles

    def run(self):
        while not rospy.is_shutdown():
            vel_x = 0 #vel za izracunat i outputat, a self.vel_x je trenutni vel dobiven iz odometrij
            vel_y = 0
            
            if self.is_leader: #ako je leader, nece ni uzlazi u ovaj neighbours loop jer nema ni jednog susjeda,,, on ne prima pozicije od nikog i ne updatea svoju poziciju s obzirom na druge
                vel_x = self.goal_position_x - self.x
                vel_y = self.goal_position_y - self.y

            for neighbour in self.neighbours_odoms:
                neigh_x, neigh_y = neighbour.pose.pose.position.x, neighbour.pose.pose.position.y
                #print(neighbour.child_frame_id)
                neigh_index = int(neighbour.child_frame_id)
                vel_x += (neigh_x - self.x) - (self.formations[neigh_index, 0] - self.formation_position_x)
                vel_y += (neigh_y - self.y) - (self.formations[neigh_index, 1] - self.formation_position_y)

            if np.linalg.norm([vel_x, vel_y]) > 0.05: #inace ce djelit s 0 (ako je == 0, ovo da ne skalira bezveze ako je dosta mala brzina)
                desired_norm = 0.5
                s = desired_norm / np.linalg.norm([vel_x, vel_y])
                vel_x *= s
                vel_y *= s

            vel = Twist()
            vel.linear.x = vel_x
            vel.linear.y = vel_y

            vel_avoid = [0, 0]

            if len(self.obstacles.points) != 0:
                cordinate_difference = np.array([[self.x - obstacle.x, self.y - obstacle.y] for obstacle in self.obstacles.points])
                euclidean_distances = np.array([[math.dist([self.x,self.y],[obstacle.x, obstacle.y])] for obstacle in self.obstacles.points])

                for i,distance in enumerate(euclidean_distances):
                    vel_avoid +=  cordinate_difference[i] / abs(distance**3)

                vel_avoid /= len(self.obstacles.points)

            vel.linear.x += vel_avoid[0] * self.avoidance_factor
            vel.linear.y += vel_avoid[1] * self.avoidance_factor
            self.publisher_vel.publish(vel)

            self.rate.sleep()
            pass

if __name__ == '__main__':
    rospy.init_node('robot_0') #debug
    try:
        agent_node = AgentNode()
        agent_node.run()
    except rospy.ROSInterruptException: pass
