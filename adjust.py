# -*- coding: utf-8 -*-
"""
Created on Mon Apr 13 16:14:33 2020

@author: Jiaxi Liang
"""

import time
from remote import *
from demo import *
from numpy import *
r = 0.11  #小车最小转弯半径
r0 = 0.05
kp = 0.3
ki = 0.05
kd = 0.3
ke = 0.4
error_list = []
k = 1
vmin = 0.05


def adjust():

    _, start = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
    _, end = vrep.simxGetObjectPosition(clientID, bot2, -1, vrep.simx_opmode_oneshot_wait)
    _, euler_angles = vrep.simxGetObjectOrientation(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)

    x = []
    first = []
    sec = []
    third = []



    path = reeds(start,end ,r)   #获取reeds-shepp曲线


    temp = []

# 对车辆进行初始化
    set_velocity(0)
    vrep.simxSetJointTargetPosition(clientID,steeringLeft,0,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,steeringRight,0,vrep.simx_opmode_oneshot)
    initial_angle = get_beta_angle() + 90




    for i in range(len(path)):

    
        delta = []
        mean_delta = 0
        temp = 0
        pre = 0
        integral = 0
        total_time = 0
    
    
    
        vrep.simxSetJointTargetPosition(clientID,steeringLeft,0,vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,steeringRight,0,vrep.simx_opmode_oneshot)
        print('i:\n',i)
        _, cur = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
        clock = time.time()
    
        if path[i].steering == rs.Steering.LEFT:
            turning(30* math.pi / 180)        #转最大角度
            while abs(abs(zaroto2pi(get_beta_angle()+90-initial_angle))-math.degrees(path[i].param) )/(math.degrees(path[i].param)) > 0.005 :
                error = math.degrees(path[i].param) - abs(zaroto2pi(get_beta_angle()+90-initial_angle))
                initial = time.time()
                time.sleep(0.1)
            
        
                total_time =  time.time() - clock
            
                a = r *(ki*(integral+error)/total_time + kp * error + kd * (error-pre))/180
                delta_t = time.time()  - initial 
                v = a/(delta_t*2*math.pi*r0)
                if path[i].gear == rs.Gear.FORWARD:
                    print(v)
                    if abs(v)< vmin:
                        if v > 0:set_velocity(vmin)
                        else: set_velocity(-vmin) 
                    else:    
                        set_velocity(v) 
               
                else:
                    if abs(v)<vmin:
                        if v > 0:set_velocity(-vmin)
                        else: set_velocity(vmin) 
                    else:    
                        set_velocity(-v) 
                
                pre = error
            
            # get_beta_angle()获得的是小车坐标下的姿态角，与地图中角度存在90度差距，故在所有运算中都要加上
            # 后来发现其实计算时抵消了，但需要修改地方较多，保留，并且可以加深对两个坐标系之间转换的理解
                print('error',error)
                print('\n',zaroto2pi(get_beta_angle()+90-initial_angle), math.degrees(path[i].param), '\n') #param为前进长度
                print(path[i].gear,'left')
         
            first.append(["实际",abs(zaroto2pi(get_beta_angle()+90-initial_angle)),"目标",math.degrees(path[i].param),"left"])
            initial_angle = get_beta_angle() + 90 #用于测量转弯完成时车辆姿态角
            set_velocity(0)
            vrep.simxSetJointTargetPosition(clientID,steeringLeft,0,vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(clientID,steeringRight,0,vrep.simx_opmode_oneshot)
        elif path[i].steering == rs.Steering.RIGHT:
                turning(- 30* math.pi / 180) 
                while abs(abs(zaroto2pi(get_beta_angle()+90-initial_angle)) - math.degrees(path[i].param))/math.degrees(path[i].param) > 0.005  :
                    initial = time.time()
                    error = math.degrees(path[i].param) - abs(zaroto2pi(get_beta_angle()+90-initial_angle))
                    time.sleep(0.1)  
                    total_time =  time.time() - clock
            
                    a = r *(ki*(integral+error)/total_time + kp * error + kd * (error-pre))/180
           

                    delta_t = time.time()  - initial 
               
                    v = a/(delta_t*2*math.pi*r0)
                    if path[i].gear == rs.Gear.FORWARD:
               
              
                        print(v)
                        if abs(v)<vmin:
                            if v > 0:set_velocity(vmin)
                            else: set_velocity(-vmin) 
                        else:    
                            set_velocity(v) 
              
                    else:
                        if abs(v)<vmin:
                            if v > 0:set_velocity(-vmin)
                            else: set_velocity(vmin) 
                        else:    
                            set_velocity(-v) 
                    
                    pre = error
           
           

                    print(path[i].gear,'right')
                    print('\n',zaroto2pi(get_beta_angle()+90-initial_angle), math.degrees(path[i].param), '\n')
            
                sec.append(["实际",abs(zaroto2pi(get_beta_angle()+90-initial_angle)),"目标",math.degrees(path[i].param),"right"])
                initial_angle = get_beta_angle() + 90 
                set_velocity(0)
                vrep.simxSetJointTargetPosition(clientID,steeringLeft,0,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,steeringRight,0,vrep.simx_opmode_oneshot)
        elif path[i].steering == rs.Steering.STRAIGHT:
            initial_position = [cur[0],cur[1]] #直线形式时需要获取最初位置
#       turning(-1 * math.pi/180)
            while abs(abs(distance(cur[0],cur[1],initial_position[0],initial_position[1])) - path[i].param * r)/(path[i].param * r) > 0.005:   # 直线情况特殊
                print('error', zaroto2pi(get_beta_angle()+90)-initial_angle)
           
                initial = time.time()
                _, cur = vrep.simxGetObjectPosition(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
                error = path[i].param * r - abs(distance(cur[0],cur[1],initial_position[0],initial_position[1]))
                time.sleep(0.1) 
                total_time = time.time() - clock
                
                a = ki*(integral+error)/total_time + kp * error + kd * (error-pre)
#           print('a',a)
           
                delta_t = time.time()  - initial
               
                v = a/(delta_t*2*math.pi*r0)
           
                error2 = 0 - (zaroto2pi(get_beta_angle()+90-initial_angle)) 
                turning_angle = ke * error2 #采用比例控制转向误差
           
                if path[i].gear == rs.Gear.FORWARD:
#               if  zaroto2pi(get_beta_angle()+90)-initial_angle>0.5:
                    turning(turning_angle * math.pi / 180)
#               elif zaroto2pi(get_beta_angle()+90)-initial_angle<-0.5:
                    if abs(v)<vmin:
                        if v > 0:set_velocity(vmin)
                        else: set_velocity(-vmin) 
                    else:    
                        set_velocity(v) 
                else:
                    turning( -turning_angle * math.pi / 180)
                    if abs(v)<vmin:
                        if v > 0:set_velocity(-vmin)
                        else: set_velocity(vmin) 
                    else:    
                        set_velocity(-v) 
       
           
                pre = error
                print(path[i].gear,'straight')
                print(distance(cur[0],cur[1],initial_position[0],initial_position[1]),path[i].param * r)
           
            third.append(["实际",abs(distance(cur[0],cur[1],initial_position[0],initial_position[1])),"目标",path[i].param * r,"straight"])
            initial_angle = get_beta_angle() + 90
            set_velocity(0)
            vrep.simxSetJointTargetPosition(clientID,steeringLeft,0,vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(clientID,steeringRight,0,vrep.simx_opmode_oneshot)

    set_velocity(0)
    vrep.simxSetJointTargetPosition(clientID,steeringLeft,0,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,steeringRight,0,vrep.simx_opmode_oneshot)

    if first != []:
       print(first)
    if sec!= []:
        print(sec)
    if third != []:
        print(third)


if __name__ == '__main__':
    adjust()

