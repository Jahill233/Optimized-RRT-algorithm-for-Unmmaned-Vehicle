from remote import *
from path_following import *
import time as tim  #避免与time变量产生冲突
import numpy as np
import math
import matplotlib.pyplot as plt
_, cur = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
_, euler_angles = vrep.simxGetObjectOrientation(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
#定义常数

kx = 0.1 # stanley 曲线参数
k = 0.1  # 预瞄参数
Lfc = 0.1  #预瞄参数
Kp = 0.5  # 对车辆速度进行比例控制时的参数kp（这里采用积分控制替代PID控制，简化运算流程，实际效果相近）
dt = 0.05  # [s] #刷新时间
L = 0.0632  # [m] wheel base of vehicle 2.9
d = 0.03213 # [m] 车轮直径
pos = [] # 用来传递小车在vrep中位置

show_animation = True


class VehicleState:# 定义一个类，用于调用车辆状态信息

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw  #车辆当前姿态角
        self.v = v
        self.L = 0.059  #车辆轴距
        self.d = 0.0561/2 #车辆轮距的一半
        self.Rwheel =3.6/2 #车辆轮胎的半径
        

def update(state, a, delta, target_ind:int):#更新车辆状态信息（反向控制）
    _, cur = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
    state.x = cur[0]#state.x + state.v * math.cos(state.yaw) * dt
    state.y = cur[1]#state.y + state.v * math.sin(state.yaw) * dt
    print('delta',math.degrees(delta))
    print('angle',get_beta_angle())
    state.v = state.v + a * dt

    set_velocity(state.v)

    turning(delta) #转指定角度
    state.yaw = get_beta_angle()*math.pi/180 + math.pi/2 #关键加90度
    return state


def PIDControl(target, current):#PID控制，定速巡航
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):# 纯跟踪控制器,pind是target_ind

    ind = calc_target_index(state, cx, cy)#找到最近点的函数，输出最近点位置

    if pind >= ind:
        ind = pind

    if ind < len(cx): #新ind比原target_ind更大，且比终点小
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
    
    

    if state.v < 0:  # 如果是倒车的话，就要反过来
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc  #车速

    delta = alpha + math.atan2( kx * distance(tx,ty,state.x,state.y),state.v ) #Stanely曲线核心公式
    
    return delta, ind


def calc_target_index(state, cx, cy):
# 找到与车辆当前位置最近点的序号

    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    Lf = k * state.v + Lfc
    dx_list = []
    dy_list = []
    l = []
    i=ind
    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]               #修改 1->5
        dy = cx[ind + 1] - cx[ind]
        
#        L = distance(cx[i],cy[i],cx[ind],cy[ind])
        L += math.sqrt(dx ** 2 + dy ** 2)
        
        ind += 1

    return ind

def main():
    #  target course ，随机出来一条sin函数曲线
    cx = []
    cy = []
    i = 0
    path_dubins_all = np.loadtxt("path_dubins_all.txt") #
    while( i<len(path_dubins_all)):
        
        cx.append(path_dubins_all[i][0])  #np.arange(0, 50, 0.1)
        
        cy.append(path_dubins_all[i][1])  #[math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
        if(path_dubins_all[i][0]>1):
            print(path_dubins_all[i][0],path_dubins_all[i][1]) #曲线角度，)
        i += 1
        
    target_speed = 2.5 / 3.6  # [m/s] 点的速度 原5

    T = 1000.0  # max simulation time（change）

    # initial state
    state = VehicleState(x=cur[0], y=cur[1], yaw=euler_angles[1], v=0) #
    
    
    
    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)
    print('ti',target_ind)
    # 不断执行更新操作
    i=0
    while T >= time and lastIndex > target_ind:
        i+=1
        ai = PIDControl(target_speed, state.v) #控制速度，返回k乘以离最大速度差值
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind) #di=delta, target_ind=ind
        state = update(state, ai, di, target_ind) #ai速度 di转动角度

        time = time + dt
        tim.sleep(0.08)
        
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        if(i%10==0):  
            if show_animation:
                plt.cla()
                plt.plot(cx, cy, ".r", label="course")
                plt.plot(x, y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target") #设置目标点
                
                plt.axis("equal")
                plt.grid(False)
                plt.title("Speed[km/h]:" + str(state.v * math.pi * d* 3.6 * 100)[:4]) #1：100为本模型放大倍数
                plt.pause(0.001)
                plt.show()
    count = 0
    dist=[]
    _, cur1 = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)      
    vrep.simxSetJointTargetPosition(clientID,steeringLeft,0,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,steeringRight,0,vrep.simx_opmode_oneshot)     
    while(distance(cx[lastIndex],cy[lastIndex],cur1[0],cur1[1])>0.05):  #有问题
          _, cur1 = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
          dist.append(distance(cx[lastIndex],cy[lastIndex],cur1[0],cur1[1]))
          
          print(dist)
          if(count!=0):
              if(dist[count]>dist[count-1]):
                  break
          count +=1        
          print(count)
        
    set_velocity(0)
    print('done')
    
def stanley(vmax:float):
    #  target course ，随机出来一条sin函数曲线
    cx = []
    cy = []
    i = 0
    path_dubins_all = np.loadtxt("path_dubins_all.txt") 
    while( i<len(path_dubins_all)):
        
        cx.append(path_dubins_all[i][0])  #np.arange(0, 50, 0.1)
        
        cy.append(path_dubins_all[i][1])  #[math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
        if(path_dubins_all[i][0]>1):
            print(path_dubins_all[i][0],path_dubins_all[i][1]) #曲线角度，
        i += 1
    
    vmax = vmax / (math.pi * d* 3.6 * 100)
    target_speed = vmax   # [m/s] 点的速度
    
    T = 1000.0  # max simulation time（change）

    # initial state
    state = VehicleState(x=cur[0], y=cur[1], yaw=euler_angles[1], v=0) #
    
    
    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)
    print('ti',target_ind)
    # 不断执行更新操作
    i=0
    while T >= time and lastIndex > target_ind:
        i+=1
        ai = PIDControl(target_speed, state.v) 
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind) #di=delta, target_ind=ind
        state = update(state, ai, di, target_ind) #ai速度 di转动角度
        tim.sleep(0.05)
        time = time + dt
        print(time)
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        
        if show_animation:
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target") #设置目标点
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * math.pi * d* 3.6 * 100)[:4])
            plt.pause(0.001)
            plt.show()
    
    count = 0
    dist=[]
    _, cur1 = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)      
    vrep.simxSetJointTargetPosition(clientID,steeringLeft,0,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,steeringRight,0,vrep.simx_opmode_oneshot)     
   
    while distance(cx[lastIndex],cy[lastIndex],cur1[0],cur1[1])>0.1:  #有问题
          _, cur1 = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
          dist.append(distance(cx[lastIndex],cy[lastIndex],cur1[0],cur1[1]))
          
          print(dist)
          if(count!=0):
              if(dist[count]>dist[count-1]):
                  break
          count +=1        
    set_velocity(0)
    print('done')
            
if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
