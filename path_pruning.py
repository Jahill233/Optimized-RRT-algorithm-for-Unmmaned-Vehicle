from typing import List
from dubins_py import *
import cv2

from utils import *


def path_pruning(path: np.ndarray, obstacles: List[Tuple[int, int, int]]):
    """Prune the solution path.

    :param path: the solution path
    :param obstacles: the list of obstacles
    :return: the pruned path
    """
    pruned_path = [path[0]]
    n, m = len(path), len(obstacles)
    cur = 0
    th = obstacles[0][2]
    print(th) # revise
    # 将连线进行 n 等分操作。
#    sz = 10 #7
    

    while True:
        if cur == n - 1:
            break
        to = cur + 1

        i = 0

        
        for j in range(cur + 1, min(cur +8, n)):

            ok = True
            
            pt1 = Waypoint(path[cur][0],path[cur][1],path[cur][2])
            pt2 = Waypoint(path[j][0],path[j][1],path[j][2])
            param = calcDubinsPath(pt1, pt2, 11, 20)
            path_dubins = dubins_traj(param,1)  # 一堆点，接下来进行碰撞检测
            
            d_list = []

            print('cur',cur)
            for (ox, oy, size) in obstacles:
                for k in range(len(path_dubins)):
                    d_list.append(distance(ox,oy,path_dubins[k][0],path_dubins[k][1]))
                    
            if min(d_list) <= size :
                print('no!dubins')
                ok = False
                continue
            else:
                print('yes!dubins')
                ok = True
                i = j     #i用来存放对应当前cur可行dubins曲线的最大index
                print('i',i)
        

        to = max(to,j)

        if i!=0:
            to = i
            pruned_path.append(path[i])
            cur = to
        elif i==0:
            for k in range(cur + 1, min(cur + 8, n)):
#                a.append(k)
                pruned_path.append(path[k])
        cur = to    

    print('pruned_path',pruned_path)   
    ret = np.array(pruned_path)
    cost = 0
    print('ret',ret)   
    np.savetxt("pruned_solution.txt", ret) #将数组保存到文档
    return ret
    plt.grid(True)
    plt.axis("equal")
    plt.title('Dubin\'s Curves Trajectory Generation')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()


def main():
    """Path Pruning.
    """
    # Load the maze image.
    img = cv2.imread("maze.png", cv2.THRESH_BINARY)  # type:np.ndarray

    # Load and draw the solution path.
    path = np.loadtxt("solution.txt")
    cv2.imshow("Original Solution", draw_path(img.copy(), path))

    # Prune the solution path.
    pruned_path = path_pruning(path, get_obstacles(img))
    cv2.imshow("Pruned Solution", draw_path(img.copy(), pruned_path))
    cv2.imwrite("pruned_solution.png", draw_path(img.copy(), pruned_path))

    for i in range(len(pruned_path)):
        print(world_coordinate(pruned_path[i]))

    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    main()
