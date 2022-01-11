from typing import List

from remote import *
from utils import *
from dubins_py import *
from pure_pursit import *
from scipy.interpolate import splev, splprep
import numpy as np, matplotlib.pyplot as plt
from scipy.interpolate import interp1d

#from sympy import * 
path_dubins = []
path_dubins_all=[]
obs=[]
sample=[]


def path_following(path: List[Tuple[int, int]]):
    """Follow the solution path.

    :param path: the solution path
    
    """
#    print(desiredSteeringAngle)
    global obs,path_dubins,path_dubins_all,sample
#    print(path)
    i = 0 #revise
    x = []
    y = []
    angle_list = []
    goal_angle = None
    tolerance = 2
    print(len(path))
    Wptz = len(path) * [0,]
    print('Wptz',Wptz)
#    print(path)
#    for i in range(len(path)):
#        x.append(path[i][0])
#        y.append(path[i][1])
#    f1 = np.polyfit(x, y, 5)
#    p1 = np.poly1d(f1) 
#    diff = p1.deriv(1)
#    i = 0
#    for i in range(len(path)):
#        k = diff(path[i][0])
#        print(k)
#        angle = math.atan(k)
#       goal_angle_list.append(math.degrees(angle))
#    i = 0
    
    
    while vrep.simxGetConnectionId(clientID) != -1:
        # When it reaches the Goal Position, we stop the scripts.
        if i == len(path)-1:
            set_velocity(0)
            break
        else:
#            print('i',i)
  #     if stage == 0:
            # Steer for specific angle.
            # Get current position.
  #         print('0')
  #          print(desiredWheelRotSpeed)
 #           _, cur = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
#           phi = math.atan2(path[i+1][1] - path[i][1], path[i+1][0] - path[i][0])           # 去掉负号
 #           print('goal_angle',goal_angle)
 #           if(i==0):
#                 cur_angle = get_beta_angle() + 180
 #                x = [path[i][0],path[i+1][0],path[i+2][0]]
 #                y = [path[i][1],path[i+1][1],path[i+2][1]]
 #                f1 = np.polyfit(x, y, 3)
 #                p1 = np.poly1d(f1)        #拟合函数
 #                diff = p1.deriv(1)
 #                k = diff(path[i+1][0])
 #                angle = math.atan(k)
 #                goal_angle = math.degrees(angle)
 #           else:
 #                x = [path[i-1][0],path[i][0],path[i+1][0]]
  #               y = [path[i-1][1],path[i][1],path[i+1][1]]
 #                f1 = np.polyfit(x, y, 3)
 #                p1 = np.poly1d(f1)        #拟合函数
 #                diff = p1.deriv(1)
 #                k = diff(path[i][0])
 #                k1 = diff(path[i+1][0])
#                cur_angle = math.atan2(path[i][1] - path[i-1][1], path[i][0] - path[i-1][0])
#                cur_angle = math.degrees(cur_angle) #去掉负号
 #           goal_angle = math.degrees(phi)        #dubins中angle是mathdegreees
 #                angle = math.atan(k)
 #                angle1 = math.atan(k1)
 #                cur_angle = math.degrees(angle)
 #                goal_angle = math.degrees(angle1)
#            angle_list.append(cur_angle)
#            cur_angle = goal_angle_list[i]
#            goal_angle = goal_angle_list[i+1]


            Wptz[i]=Waypoint(path[i][0],path[i][1],path[i][2])
#            print(Wptz[i])

            Wptz[i+1]=Waypoint(path[i+1][0],path[i+1][1],path[i+1][2])
#            print(Wptz[i+1])
            param = calcDubinsPath(Wptz[i], Wptz[i+1], 0.1, 20)  #倒数第二个参数为转弯半径，最后一个参数为最大转向角（暂时未设置关系式）
            path_dubins = dubins_traj(param,1)
   #     print(path_dubins)
            path_dubins_all.extend(path_dubins) #杜宾斯曲线上的每一个点
   #     print(path_dubins_all[0])
           
            
            plt.plot(Wptz[i].x,Wptz[i].y,'yx')
            plt.plot(Wptz[i+1].x,Wptz[i+1].y,'kx')
            plt.plot(path_dubins[:,0],path_dubins[:,1],'r-')
            i+=1
    
    for i in range(len(path_dubins_all)):
        x.append(path_dubins_all[i][0])
        y.append(path_dubins_all[i][1])
    x = np.array(x)
    y = np.array(y)    
    tck, u = splprep([x, y], s=0.2)         #s为关键参数
    
    new_points = (splev(u, tck))
    
    plt.plot(x,y,'r.', markersize=1)
    plt.plot(new_points[0],new_points[1], 'b-')
    plt.xticks([])
    plt.yticks([])
#    print('x,y',len(x),len(y))
#    f=interp1d(x,y,kind='cubic')
#    xnew = np.arange(min(x),max(x),0.05)
#    ynew = f(xnew)
    
    
    
    plt.grid(False)
#    plt.axis("off")
    plt.title('Path') 
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.rcParams['figure.dpi'] = 300
    plt.show()
    
    print('len',new_points)
    
    



    
#    new_angle_list = []
#    print('al',angle_list)
#    for i in range(len(angle_list)):
#        if i!=0 and i!=len(angle_list)-1:
#            new_angle_list.append((angle_list[i-1] + angle_list[i] + angle_list[i+1])/3)
#        else:
#            new_angle_list.append(angle_list[i])
#    print('nal',new_angle_list)
#    i=0
#    while(i<len(path_dubins_all)):
#        if(i%20==0):
#            plt.scatter(path_dubins_all[i][0],path_dubins_all[i][1])
#            sample.append([path_dubins_all[i][0],path_dubins_all[i][1],path_dubins_all[i][2]])
#        i += 1
    
    
    
    
    
    
    
    img = cv2.imread("maze.png", cv2.THRESH_BINARY)
    dubins=[]
#    print(np.array(path_dubins_all))
    for i in range(len(path_dubins_all)): 
        x,y = Sensor_coordinate([new_points[0][i],new_points[1][i]])
        dubins.append([new_points[0][i],new_points[1][i]])
    
    
#    np.savetxt("dubins", dubins)
#    cv2.imshow("path_dubins_all", draw_path(img, dubins))
    cv2.imwrite("path_dubins_all.png", draw_path(img, dubins))
    np.savetxt("path_dubins_all.txt", dubins) #key 承上启下
    
    


def main():
    """Path Following.
    """
    # Initialize the simulation.
    init()

 #   print(desiredSteeringAngle)
    # Load the solution path.
    solution = np.loadtxt("pruned_solution.txt")  #Pruned solution.txt
    # Convert the coordinates.
    print(solution)
    maze = get_image(vision_sensor) #revise
    obstacles = get_obstacles(maze)
    
    
    obs = []                                                        #用于展示使用
    i = 0
    x_list = []
    y_list = []
#    print(obstacles)
    plt.figure(figsize=(5,5))
    while i < len(obstacles):
        print(obstacles[i])
        obs = world_coordinate(obstacles[i]) #获得障碍世界坐标
        print(obs[0])
        x_list.append(obs[0])
        y_list.append(obs[1])
        print('i',i)
        i += 1
        plt.scatter(obs[0],obs[1],color='black', marker='.',s=10)   
    plt.xlim(min(x_list),2.1)
    
    print(solution)
 
    path = []
    for i in range(len(solution)): 
        solution[i][2] = 360-solution[i][2]
        path.append(world_coordinate(solution[i]))
        
    print('path',path)
    # Follow the solution path.
#    print(path)
    path_following(path)
    

if __name__ == '__main__':
    main()
