import random
from typing import List, Tuple
from sklearn.neighbors import KDTree
from path_planning import *
from dubins_py import *
from utils import *
import time
import cv2
import math
import numpy as np
train_list = [] #用来存放rrt
train_list2 = [] #用来存放rrt2


nl = 0 # 检测node_list 函数是否改变
nl2 = 0 # 检测第二棵树node_list函数是否改变
ci = 0.98 #检测最近树节点时，随机点和树节点距离所占比重
cj = 1-ci #检测最近树节点时，目标点和树节点距离所占比重 
kp = 0.3 #引力系数
krep = 3 #斥力系数

cal = 0
cals = 0

class  RRT:
    """RRT Motion Planning.
    """

    class Node:
        """RRT Node.
        """

        def __init__(self, x: float, y: float, angle:float,dis2obs = 1):
            self.x = x
            self.y = y
            self.angle = angle
            self.path_x = []
            self.path_y = []
            self.path_angle = []
            self.cost = 0.0   #用于计算路程花费的
            self.parent = None  # type:None | RRT.Node
            self.dis2obs = dis2obs
            self.dt = 0  #和最近障碍物的夹角
            
    def __init__(self, start: Tuple[int, int], goal: Tuple[int, int], obstacle_list: List[Tuple[int, int, int]],
                 rand_area: Tuple[int, int],
                 expand_dis=3.0, path_resolution=0.5, goal_sample_rate=5, max_iter=500 , radius=40):
        """Initialize the RRT Motion Planner.

        :param start: Start Position
        :param goal:  Goal Position
        :param obstacle_list: the list of obstacles
        :param rand_area: random sampling area
        :param expand_dis: distance of one expanding
        :param path_resolution: path resolution
        :param goal_sample_rate: gaol sample rate
        :param max_iter: max amount of iterations
        """
        self.phi = math.degrees(math.atan((goal[0] - start[0])/(goal[1] - start[1])))
        print('aaa',self.phi)
        self.start = self.Node(start[0], start[1],start[2])
        self.end = self.Node(goal[0], goal[1],goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        
        self.max_angle = self.phi + 25      
        self.min_angle = self.phi - 25
        self.expand_dis = expand_dis
        self.expand_dis_max = expand_dis+2
        self.expand_dis_min = expand_dis-2
        
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        
        
        self.obstacle_list = obstacle_list
        
        self.node_list = [self.start]
        self.expand = 1
        self.search_radius = radius
        
        
    
    
    def path_planning(self, img: np.ndarray, rrt2,signal): #关键
        """Find the solution of the maze.
        
        :param img: the image of maze
        :return: the solution path
        """
        obs=[]
        d_dynamic = 0
        global cal,cals
        
        for k in range(len(self.obstacle_list)):
            obs.append([self.obstacle_list[k][0],self.obstacle_list[k][1]])
        self.start.dis2obs,self.dt = self.near_obstacles_dis(obs, self.start)
        
        for i in range(self.max_iter):
            
            cal +=1 

            # Generate a sample node.
            rnd_node = self.get_random_node(signal)
            # Find the nearest node to the rnd_node.
            dist,nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node,self.end) #rnd为随机点,self.为已有路径
#            print('nearest_ind',nearest_ind)
            nearest_node = self.node_list[nearest_ind]
            
            #d为力障碍物的最近距离
            d = nearest_node.dis2obs
            #动态步长
            if d < self.expand_dis_min:
                self.expand_dis = self.expand_dis_min
            elif d > self.expand_dis_min and d < self.expand_dis_max:
                self.expand_dis = d
            else: self.expand_dis = self.expand_dis_max
            
            
            print('expand_dis', self.expand_dis)
            # Expand the tree
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis,15,obs)   #换成dubins，15为斥力场最大作用范围
            
           
            
            # When there are no collisions, add new_node to the tree.
            _ = self.dubins_ev(new_node, nearest_node,signal)
            if _ == False:
                print('delete cuz of dubins')
                continue

            #获得树节点到障碍物的最近距离及角度，用于斥力场的计算
            new_node.dis2obs,self.dt = self.near_obstacles_dis(obs, new_node)
            #判断两颗树是否相交
            dist2,ind2 = rrt2.get_nearest_node_index(rrt2.node_list, new_node,  self.end)
            if dist2 < 2*self.expand_dis:

                new_node2 = rrt2.Node(new_node.x, new_node.y,new_node.angle,new_node.dis2obs)
                if rrt2.dubins_ev(new_node2,rrt2.node_list[ind2],-signal): 
                    self.expand = 0 #检测是否已经在另外一个树
                    self.end = new_node
                    self.node_list.append(new_node)
                    tmp = self.node_list[-1]
                        
                    x, y = int(tmp.x), int(tmp.y)
                    
                    img[x][y] = 255
                    cv2.imshow("Processing", img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    
                        
                    rrt2.end = new_node2
                    rrt2.node_list.append(new_node2)
                    rrt2.end.parent = rrt2.node_list[ind2]
                    rrt2.expand = 0
                    cals +=1
                    print('cal cals',cal,cals)
                    break
                

            new_node = self.rrtstartsearch(new_node) #rrt星处理        
            cals +=1
            self.node_list.append(new_node)
                # Show the expanding process.
               

            tmp = self.node_list[-1]
#            for j in range(len(new_node.path_x)):
            x, y = int(tmp.x), int(tmp.y)
            if(x>=512 or y>=512 or x<0 or y<0):
                print('alert')
                break
            img[x][y] = 255
            cv2.imshow("Processing", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            self.expand = 1
            break 


    def steer(self, from_node: Node, to_node: Node, expand_length=float("inf"), ifldis=15, obs=List[Tuple[int, int]]):    #对其进行引力修改
        """Expand the tree from from_node to to_node.
        
        :param from_node: from which node to expand
        :param to_node: to which node to expand
        :param expand_length: expand length
        :return: the new node
        """
        
        new_node = self.Node(from_node.x, from_node.y, from_node.angle,from_node.dis2obs)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        _, theta2 = self.calc_distance_and_angle(new_node, self.end)

        pr = self.path_resolution
        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_angle = [new_node.angle]

        if expand_length > d:
            expand_length = d
        n_expand = math.floor(expand_length / self.path_resolution) #建立点的次数
        for _ in range(n_expand+1):
            d, _ = self.calc_distance_and_angle(new_node, to_node)
            if d <= self.path_resolution:
                pr = d
            pr = self.path_resolution
            _, theta2 = self.calc_distance_and_angle(new_node, self.end)
            dis2obs,dt = self.near_obstacles_dis(obs,new_node)
            if dis2obs < ifldis:
                new_node.x += pr  *(math.cos(theta) + kp * math.cos(theta2) + krep * (1/dis2obs - 1/ifldis) * math.cos(dt))
                new_node.y += pr  *(math.sin(theta) +  kp * math.sin(theta2) + krep * (1/dis2obs - 1/ifldis) * math.sin(dt))
            else:
                new_node.x += pr *(math.cos(theta) + kp * math.cos(theta2))
                new_node.y += pr *(math.sin(theta) + kp * math.sin(theta2))
            
            new_node.angle = to_node.angle
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
            new_node.path_angle.append(to_node.angle)
        new_node.parent = from_node
        return new_node

    def generate_final_course(self):
        """Generate the final path to Goal Position.

        :return: the final path to Goal Position
        """

        path = [(self.end.x, self.end.y, self.end.angle)]  # type:List[Tuple[float,float]]
        node = self.node_list[len(self.node_list) - 1]
#        print('node',node.parent)
        while node.parent is not None:
            path.append((node.x, node.y, node.angle))
            node = node.parent
            
        path.append((node.x, node.y, node.angle))
        return path[::-1]

    def calc_dist_to_goal(self, x: float, y: float):
        """Calculate the distance between node (x, y) and Goal Position.

        :param x: the x-coordinate of the node
        :param y: the y-coordinate of the node
        :return: the distance between node (x, y) and Goal Position
        """
        dx = x - self.end.x
        dy = y - self.end.y
        return math.sqrt(dx ** 2 + dy ** 2)

    def get_random_node(self,signal):
        """Generate a random RRT.Node

        :return: a random RRT.Node
        """
        
        if random.randint(0, 100) > self.goal_sample_rate:  # Do random sampling.
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_angle, self.max_angle))
            
        else:  # Do goal point sampling.
            rnd = self.Node(self.end.x, self.end.y, self.phi)
        return rnd

    @staticmethod
    
    """ 计算Dubins曲线距离
    """
    
    def dubins_dis(tree_node,rand_node):
        pt1 = Waypoint(tree_node.x,tree_node.y,tree_node.angle)
        pt2 = Waypoint(rand_node.x,rand_node.y,rand_node.angle)
        r = 10
        param = calcDubinsPath(pt1, pt2, r, 10)
        cost = (param.seg_final[0]+ param.seg_final[1]+ param.seg_final[2]) * r
        return cost
    
    def get_nearest_node_index(self,node_list: List[Node], rnd_node: Node,goal):  #同时调用会出现问题，用signal解决
        """寻找最近树节点

        :param node_list: the candidate nodes
        :param rnd_node: the target node
        :return: the nearest node to rnd_node in node_list
        """

#        distances = [ci*math.sqrt((node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2)+cj*math.sqrt((node.x -goal.x) ** 2 + (node.y - goal.y) ** 2) for node in node_list]
        distances = [ci*self.dubins_dis(node, rnd_node)+cj*math.sqrt((node.x -goal.x) ** 2 + (node.y - goal.y) ** 2) for node in node_list] 
        dmin = min(distances)
        nearest = distances.index(dmin)
        dmin = math.sqrt((node_list[nearest].x-rnd_node.x) ** 2 + (node_list[nearest].y-rnd_node.y) ** 2)
        return dmin,nearest
    
#   KD树（由于稳定性不佳在此处未采用）                
#            if(nl != len(node_list)):
#                for j in range(len(node_list)-1):
#                    cost_list.append([ci * self.dubins_dis(node_list[j],rnd_node),cj * distance(node_list[j].x,node_list[j].y,goal.x,goal.y)])
                
#                train_list.append([node_list[i].x,node_list[i].y, cj/ci * distance(node_list[i].x,node_list[i].y,goal.x,goal.y)])
#                nl = len(node_list)
#            i += 1
#            if (nl<10):
#        print('tl',train_list)
#            tree = KDTree(np.array(train_list))
            
#        print([[rnd_node.x,rnd_node.y]])
#            dists, indices = tree.query([[rnd_node.x,rnd_node.y,0]], k=1)
#        distances = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list] #采用其他数据结构
#        print('n',indices[0])

            
#            return dists[0][0],indices[0][0] #返回int值
            
#        else: 
#            if(nl2 != len(node_list)):
#                for k in range(i+1):
#                     cost_list.append([self.dubins_dis(node_list[k],rnd_node),cj/ci * distance(node_list[k].x,node_list[k].y,goal.x,goal.y)])
#                train_list2.append([node_list[i].x,node_list[i].y,cj/ci * distance(node_list[i].x,node_list[i].y,goal.x,goal.y)])
#                nl2 = len(node_list)
#            tree = KDTree(np.array(train_list2)) #error,需要array
            

#            dists, indices = tree.query([[rnd_node.x,rnd_node.y,0]], k=1)
            
#            return dists[0][0],indices[0][0] #返回int值
    

    @staticmethod
    def check_collision(node: Node, obstacles: List[Tuple[int, int, int]]):
        """Check the node whether collides with obstacles.

        :param node: a RRT.Node
        :param obstacles: obstacles list
        :return: whether the node collides with obstacles
        """
        
        
        for (ox, oy, size) in obstacles:
            d_list = [distance(ox,oy,x,y) for (x,y) in zip(node.path_x,node.path_y)]

            if min(d_list) <= size :
               print('collison_failed!')
               return False
        print('collison_succeed!')
        return True

    @staticmethod
    def calc_distance_and_angle(from_node: Node, to_node: Node):
        """Calculate the distance and angle between two nodes.

        :param from_node: a RRT.Node
        :param to_node: a RRT.Node
        :return: the distance and angle
        """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.sqrt(dx ** 2 + dy ** 2)
        theta = math.atan2(dy, dx)
        return d, theta

    def rrtstartsearch(self, new_node:Node):     
        """ rrt*重新寻找父节点
        
        """
        temp = []
        for i in range(len(self.node_list)):
            dis = distance(self.node_list[i].x,self.node_list[i].y,new_node.x,new_node.y)
            if dis < self.search_radius:
                temp.append((self.node_list[i].cost + dis,i))
        if temp != []:
            
            new_node.cost = min(temp)[0] 
            new_node.parent = self.node_list[min(temp)[1]]
 
            return new_node
        new_node.cost = new_node.parent.cost + self.dubins_dis(new_node.parent,new_node)
        return new_node
    
    
    def dubins_ev(self,new_node:Node, nearest_node:Node,signal):
        """ dubins路径碰撞检测
        
        """
        print('nna',new_node.angle,'\n')
        pt1 = Waypoint(nearest_node.x,nearest_node.y,nearest_node.angle)
        pt2 = Waypoint(new_node.x,new_node.y,new_node.angle)
        
        if signal == 1:
            param = calcDubinsPath(pt1, pt2, 10, 10)
        
        if signal == -1:
            param = calcDubinsPath(pt2, pt1, 10, 10)
        
        path_dubins = dubins_traj(param,1)  # 一堆点，接下来进行碰撞检测
        d_list = []
        
        for (ox, oy, size) in self.obstacle_list:
           for k in range(len(path_dubins)):
               d_list.append(distance(ox,oy,path_dubins[k][0],path_dubins[k][1]))

        if min(d_list) <= size :
#             print('no!dubins')
             return False
#        print('yes!dubins')
        return True
    
    def near_obstacles_dis(self, obs:List[Tuple[int, int]],near_node:Node):
        
        d_list = []
        for i in range(len(obs)):
            d_list.append((near_node.x - obs[i][0]) ** 2 + (near_node.y - obs[i][1]) ** 2)
        
        dmin = min(d_list)
        nearest = d_list.index(dmin)
        dmin = math.sqrt(dmin)
        dx = - obs[nearest][0] + near_node.x
        dy = - obs[nearest][1] + near_node.y
        dmin -= 10

        if dmin<1: dmin=1
        theta = math.atan2(dy, dx)
        return dmin,theta
    
    
    
    