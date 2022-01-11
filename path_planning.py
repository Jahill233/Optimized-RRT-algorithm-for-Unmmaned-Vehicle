import time
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from sklearn.neighbors import KDTree
from RRT import *
from utils import get_obstacles, draw_path
from remote import *
tree1Expansion=1
tree2Expansion=1
#signal = 1 # 1代表rrt,-1代表rrt2

def path_planning(img: np.ndarray):
    """Find the solution of the maze.

    :param img: the image of the maze
    :return: the solution
    """
    global train_list,train_list2
    # 获取起始点.
    start = (Sensor_coordinate(position_start)[0],Sensor_coordinate(position_start)[1],get_beta_angle()) #?
    goal = (Sensor_coordinate(position_end)[0],Sensor_coordinate(position_end)[1],get_beta_angle2()) #?
    print(start)
    print(goal)
    img1 = img.copy()
    obstacles = get_obstacles(img)
    
    # 建立两棵RRT树
    rrt1 = RRT(start, goal, obstacles, (0, 512), expand_dis=10, goal_sample_rate=5, path_resolution=5, max_iter=50000)
    rrt2 = RRT(goal, start, obstacles, (0, 512), expand_dis=10, goal_sample_rate=5, path_resolution=5, max_iter=50000)
    

    while rrt1.expand != 0 or rrt2.expand !=0: 
        if rrt1.expand == 1:
            signal = 1
            rrt1.path_planning(img1,rrt2,signal)
            
            
        if rrt2.expand == 1:
            signal = -1
            rrt2.path_planning(img1,rrt1,signal)
            
    # 获取第一棵树路径点并画图   
    path1 = rrt1.generate_final_course()
    cv2.imwrite("Tree1.png", draw_path(img.copy(), path1))
    cv2.imshow("Tree1", draw_path(img.copy(), path1))

    path1 = path1[:-1]
    path2 = rrt2.generate_final_course()
    path2.reverse()
    path2 = path2[1:]
    path1.extend(path2)
    cost = 0
    print(path1,'\n')

    if path1 is None:
        raise Exception("Cannot find the path.")
    # Save the solution.
    np.savetxt("solution.txt", path1, fmt="%s")

    #获取第一棵树路径点并画图
    cv2.imwrite("Tree2.png", draw_path(img.copy(), path2))
    cv2.imshow("Tree2", draw_path(img.copy(), path2))
  
    return path1



def main():
    """Path Planning.
    """
    # Load the maze image.
    img = cv2.imread("maze.png",cv2.THRESH_BINARY)  # type:np.ndarray 
    cv2.imshow("Maze", img)
    

    # Find the solution.
    start = time.time()
    solution = path_planning(img)
    cost = time.time() - start
    print(f"It costs {cost} seconds to find the path.")

    # Show the solution.
    cv2.imshow("Solution", draw_path(img.copy(), solution))
    cv2.imwrite("solution.png", draw_path(img.copy(), solution))
    
    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    main()
