from typing import List
from remote import *
from utils import *
from dubins_py import *
from pure_pursit import *
from scipy.interpolate import splev, splprep
import numpy as np, matplotlib.pyplot as plt
from scipy.interpolate import interp1d
 
path_dubins = []
path_dubins_all=[]
sample=[]


def path_smooth(path: List[Tuple[int, int]],obs):
    """Follow the solution path.

    :param path: the solution path
    
    """
#    print(desiredSteeringAngle)
    global path_dubins,path_dubins_all,sample
    i = 0 #revise
    x = []
    y = []
    angle_list = []
    goal_angle = None
    tolerance = 2
    print(len(path))
    Wptz = len(path) * [0,]
    print('Wptz',Wptz)

    while vrep.simxGetConnectionId(clientID) != -1:
        # When it reaches the Goal Position, we stop the scripts.
        if i == len(path)-1:
            set_velocity(0)
            break
        else:

            Wptz[i]=Waypoint(path[i][0],path[i][1],path[i][2])
            Wptz[i+1]=Waypoint(path[i+1][0],path[i+1][1],path[i+1][2])
            param = calcDubinsPath(Wptz[i], Wptz[i+1], 0.1, 20)  #倒数第二个参数为转弯半径，最后一个参数为最大转向角（暂时未设置关系式）
            path_dubins = dubins_traj(param,1)
            path_dubins_all.extend(path_dubins) #杜宾斯曲线上的每一个点
            plt.plot(Wptz[i].x,Wptz[i].y,'yx')
            plt.plot(Wptz[i+1].x,Wptz[i+1].y,'kx')
            plt.plot(path_dubins[:,0],path_dubins[:,1],'r-')
            i+=1
     
    # 三次b样条插值算法
    d_list = []
    signal = 1
    
    for i in range(len(path_dubins_all)):
        x.append(path_dubins_all[i][0])
        y.append(path_dubins_all[i][1])
    x = np.array(x)
    y = np.array(y)  
    
    a = 0.5
    while a>0:
        print(a,x,y)
        d_list = []
        j = 0
        k = 0
        tck, u = splprep([x, y], s=a)         #s为关键参数
        new_points = (splev(u, tck))
        for j in range(len(obs)):
            for k in range(len(new_points[0])):
                
                
                d_list.append(distance(obs[j][0],obs[j][1],new_points[0][k],new_points[1][k]))
            
            if min(d_list)<0.04:
                signal = 0
                
                break
            
            else: signal = 1
            
        if signal == 1: 
            break
        a = a - 0.1 # 若前一次s值无法通过障碍物检测
    
    print('s值为',a,'路径点距离障碍物最近距离',min(d_list))
    plt.plot(x,y,'r.', markersize=1)
    plt.plot(new_points[0],new_points[1], 'b-')
    plt.xticks([])
    plt.yticks([])
    plt.grid(False)
#    plt.axis("off")
    plt.title('Path') 
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.rcParams['figure.dpi'] = 300
    plt.show()

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
    # Load the solution path.
    solution = np.loadtxt("pruned_solution.txt")  #Pruned solution.txt
    # Convert the coordinates.
    print(solution)
    maze = get_image(vision_sensor) #revise
    obstacles = get_obstacles(maze)
    obs = []                                                        #用于展示使用
    obs1 = []
    i = 0
    x_list = []
    y_list = []
#    print(obstacles)
    plt.figure(figsize=(5,5))
    while i < len(obstacles):
        print(obstacles[i])
        obs = world_coordinate(obstacles[i]) #获得障碍世界坐标
        obs1.append(obs)
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
    path_following2(path,obs1)
    

if __name__ == '__main__':
    main()
