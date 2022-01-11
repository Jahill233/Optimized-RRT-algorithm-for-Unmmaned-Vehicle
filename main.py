from path_following import path_following
from path_following2 import path_following2
from path_planning import path_planning
from path_pruning import *
from dubins_py import *
from remote import *
from utils import *
from tim import *
import time
from stanley import *
from adjust import *
import matplotlib.pyplot as plt


def main():
    """Find the solution and follow it.
    """
    # 车辆初始化.
    init()
    
    # 扫描地图.
    maze = get_image(vision_sensor)  
    obstacles = get_obstacles(maze)  #获得障碍物坐标
    
    
    cv2.imwrite("maze.png", maze)
    cv2.imshow("Maze", maze)
    print('\n地图扫描完成！\n')
    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 寻找路径.
    
    
    start = time.time()
    solution = path_planning(maze) # revision1 
    cost = time.time() - start
    print(f"It costs {cost} seconds to find the path.")
    print('\n路径已找到！\n')
    # Show the solution.
    cv2.imshow("Solution", draw_path(maze, solution))
    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    """Path Pruning.
    """
    
    img = cv2.imread("maze.png", cv2.THRESH_BINARY)  # type:np.ndarray

    # Load and draw the solution path.
    path = np.loadtxt("solution.txt")
    cv2.imshow("Original Solution", draw_path(img.copy(), path))

    # 剪枝.
    pruned_path = path_pruning(path, get_obstacles(img))
    cv2.imshow("Pruned Solution", draw_path(img.copy(), pruned_path))
    cv2.imwrite("pruned_solution.png", draw_path(img.copy(), pruned_path))

    for i in range(len(pruned_path)):
        print(world_coordinate(pruned_path[i]))
    print('\n剪枝完成！\n')
    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 变换坐标.
    path= []
    solution = np.loadtxt("pruned_solution.txt")
    for i in range(len(solution)):
        solution[i][2] = 360-solution[i][2]
        path.append(world_coordinate(solution[i]))
    
    # 绘制障碍物
    obs = []   
    obs_all = []                                                     
    i = 0
    x_list = []
    y_list = []
#    print(obstacles)
    plt.figure(figsize=(5,5))
    
    while i < len(obstacles):
        print(obstacles[i])
        obs = world_coordinate(obstacles[i]) #获得障碍世界坐标
        obs_all.append(obs)
        x_list.append(obs[0])
        y_list.append(obs[1])
        print('i',i)
        i += 1
        plt.scatter(obs[0],obs[1],color='black', marker='.',s=10)   
    plt.xlim(min(x_list),2.1)
    
    # Follow the path.
    path_smooth(path,obs_all) # 采用三次样条插值
    print('\n平滑曲线完成\n')
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # 跟随算法
    stanley(30)    #参数为车辆最大运行速度
    
    print("开始微调")
    # Reeds-shepp微调姿态
    adjust()
    
    print('\n 所有步骤已完成！\n')
    
 

if __name__ == '__main__':
    main()
