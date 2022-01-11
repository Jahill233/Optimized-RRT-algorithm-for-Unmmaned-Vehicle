import time
import cv2
import math
import numpy as np
import vrep
from utils import *

# Close all the connections.
vrep.simxFinish(-1)
# Connect the V-REP.
clientID = vrep.simxStart("127.0.0.1", 19997, True, True, 5000, 5)

steeringAngleDx=5*math.pi/180
desiredSteeringAngle=0
desiredWheelRotSpeed=0.3*math.pi
#steeringAngleDx=2*math.pi/180
#wheelRotSpeedDx=20*math.pi/180
count1 = 0
count2 = 0
count = 0
max1 = 0
max2 = 0

d=0.094/4 #2*d=distance between left and right wheels
l=0.0632 #l=distance between front and read wheels

# Get object handles.
_, bot = vrep.simxGetObjectHandle(clientID, 'MyBot', vrep.simx_opmode_oneshot_wait)
_, bot2 =vrep.simxGetObjectHandle(clientID, 'MyBot#0', vrep.simx_opmode_oneshot_wait)
_, vision_sensor = vrep.simxGetObjectHandle(clientID, 'Vision_Sensor', vrep.simx_opmode_oneshot_wait)
_, vision_sensor2 = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
_, path_start = vrep.simxGetObjectHandle(clientID, 'MyBot', vrep.simx_opmode_oneshot_wait)
_, path_end = vrep.simxGetObjectHandle(clientID, 'MyBot#0', vrep.simx_opmode_oneshot_wait)
_, Front_left_wheel = vrep.simxGetObjectHandle(clientID, 'Front_left_wheel', vrep.simx_opmode_oneshot_wait)
_, Front_right_wheel = vrep.simxGetObjectHandle(clientID, 'Front_right_wheel', vrep.simx_opmode_oneshot_wait)
_, MotorHandle_Left = vrep.simxGetObjectHandle(clientID,'Front_left_joint',vrep.simx_opmode_oneshot_wait)
_, MotorHandle_Right = vrep.simxGetObjectHandle(clientID,'Front_right_joint',vrep.simx_opmode_oneshot_wait)
_, steeringLeft = vrep.simxGetObjectHandle(clientID,'Steer_left_joint',vrep.simx_opmode_oneshot_wait)
_, steeringRight = vrep.simxGetObjectHandle(clientID,'Steer_right_joint',vrep.simx_opmode_oneshot_wait)
_, Tip_M = vrep.simxGetObjectHandle(clientID,'tip_M',vrep.simx_opmode_oneshot_wait)
#Get object poristion
_, position_start = vrep.simxGetObjectPosition(clientID,path_start,-1,vrep.simx_opmode_oneshot_wait)
_, position_end = vrep.simxGetObjectPosition(clientID,path_end,-1,vrep.simx_opmode_oneshot_wait)


if clientID == -1:
    raise Exception("Fail to connect remote API server.")


def init():
    """Initialize the simulation.
    """

    vrep.simxGetVisionSensorImage(clientID, vision_sensor, 0, vrep.simx_opmode_streaming)
    set_velocity(0)
    time.sleep(1)


def get_image(sensor):
    """Retrieve a binary image from Vision Sensor.

    :return: a binary image represented by numpy.ndarray from Vision Sensor
    """
    err, resolution, raw = vrep.simxGetVisionSensorImage(clientID, sensor, 0, vrep.simx_opmode_buffer)
    if err == vrep.simx_return_ok:
        img = np.array(raw, dtype=np.uint8)
        img.resize([resolution[1], resolution[0], 3])
        # Process the raw image.
        
        _, th1 = cv2.threshold(img, 210, 255, cv2.THRESH_BINARY)
        
        g = cv2.cvtColor(th1, cv2.COLOR_BGR2GRAY)
        
        _, th2 = cv2.threshold(g, 250, 255, cv2.THRESH_BINARY) #土堆情况特殊要使比较黑的都变白 所以250<-2
        
        # Find the edges using Canny.
        edge = cv2.Canny(th2, 230, 255)  # type: np.ndarray
        print(edge.shape)
        return edge
    else:
        return None


def move(v, o):
    """Move the robot.

    :param v: desired velocity
    :param o: desired angular velocity
    """
    wheel_radius = 0.027
    distance = 0.119
    v_l = v - o * distance
    v_r = v + o * distance
    o_l = v_l / wheel_radius
    o_r = v_r / wheel_radius
    vrep.simxSetJointTargetVelocity(clientID, left_motor, o_l, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, right_motor, o_r, vrep.simx_opmode_oneshot)


def get_beta_angle():
    """Return the degrees of Beta Euler Angle.

    :return: the degrees of Beta Euler Angle
    """
    _, euler_angles = vrep.simxGetObjectOrientation(clientID, bot, -1, vrep.simx_opmode_oneshot_wait)
    ret = math.degrees(euler_angles[1])
   
 #   print(ret)
    if euler_angles[0] <= 0 < ret:
        return 180 - ret
    if euler_angles[2] <= 0 and ret < 0:
        return -180 - ret
    return ret

def get_beta_angle2():
    """Return the degrees of Beta Euler Angle.

    :return: the degrees of Beta Euler Angle
    """
    _, euler_angles = vrep.simxGetObjectOrientation(clientID, bot2, -1, vrep.simx_opmode_oneshot_wait)
    ret = math.degrees(euler_angles[1])
    
 #   print(ret)
    if euler_angles[0] <= 0 < ret:
        return 180 - ret
    if euler_angles[2] <= 0 and ret < 0:
        return -180 - ret
    return ret


def turning(x:str,y:str):
    """基于阿克曼转向小车的转向参数变换
    
    """
    
    desiredWheelRotSpeed=0.3*math.pi
    global count1,count2,desiredSteeringAngle,steeringAngleDx,steeringLeft,steeringRight
    if (x == 'left' and y == 'front'): #left
        if (desiredSteeringAngle>=36.7*math.pi/180):
            desiredSteeringAngle=36.7*math.pi/180
            desiredWheelRotSpeed=0.6*math.pi
            max1 = 1
        else:
            desiredSteeringAngle=desiredSteeringAngle+0.1*steeringAngleDx;
            count1 = count1 - 1
            desiredWheelRotSpeed=0.3*math.pi
            max1 = 0
    elif ( x == 'right' and y == 'front' ): # right
 #      print(desiredSteeringAngle)
        if (desiredSteeringAngle<=-36.7*math.pi/180): 
            dSteeringAngle=-36.7*math.pi/180
            max2 = 1
            desiredWheelRotSpeed=0.6*math.pi
        else:
            desiredSteeringAngle = desiredSteeringAngle - 0.1*steeringAngleDx
            count1 = count1 + 1
            desiredWheelRotSpeed=0.3*math.pi
            max2 = 0
    elif ( x == 'left' and y == 'back'):
        if (desiredSteeringAngle>=36.7*math.pi/180):
            desiredSteeringAngle=36.7*math.pi/180
            desiredWheelRotSpeed=-0.6*math.pi
        else:
            desiredSteeringAngle=desiredSteeringAngle+0.1*steeringAngleDx;
            count1 = count1 - 1
            desiredWheelRotSpeed=0.3*math.pi   
    elif ( x == 'right' and y == 'back'):
        if (desiredSteeringAngle<=-36.7*math.pi/180): 
            dSteeringAngle=-36.7*math.pi/180
            max2 = 1
            desiredWheelRotSpeed=-0.6*math.pi
        else:
            desiredSteeringAngle = desiredSteeringAngle - 0.1*steeringAngleDx
            count1 = count1 + 1
            desiredWheelRotSpeed=-0.3*math.pi           
    
    steeringAngleLeft=math.atan(l/(-d+l/math.tan(desiredSteeringAngle)))
    steeringAngleRight=math.atan(l/(d+l/math.tan(desiredSteeringAngle)))  
    vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Left,desiredWheelRotSpeed,vrep.simx_opmode_oneshot) #设置非球关节固有目标速度
    vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Right,desiredWheelRotSpeed,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,steeringLeft,steeringAngleLeft,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,steeringRight,steeringAngleRight,vrep.simx_opmode_oneshot)


def slight_adjust():
    """在微调情况下，基于阿克曼转向小车的转向参数变换
    
    """
    global count,count1,count2,desiredSteeringAngle,steeringLeft,steeringRight,MotorHandle_Left,MotorHandle_Right
    print('count1',count1,'count2',count2)
    while(count1!=0 or count2!=0):
        count = count1 + count2
        print('count',count)
        if count1>0 and count2>0:
           desiredWheelRotSpeed = 0.1*math.pi
           desiredSteeringAngle = desiredSteeringAngle + 0.3*steeringAngleDx
           count1 = count1-1
           count2 = count2-1
           
        elif count1>0 and count2==0:
 #          print('jahill')
           desiredWheelRotSpeed = 0.1*math.pi
           desiredSteeringAngle = desiredSteeringAngle + 0.1*steeringAngleDx
           count1 = count1-1
           
  #     elif count1==0 && count2==0
  #         desiredWheelRotSpeed = 0.1*math.pi
        elif count1<0 and count2<0:
           desiredWheelRotSpeed = 0.1*math.pi
           desiredSteeringAngle = desiredSteeringAngle - 0.3*steeringAngleDx
           count1 = count1+1
           count2 = count2+1
           
        elif count1<0 and count2==0:
           desiredWheelRotSpeed = 0.1*math.pi
           desiredSteeringAngle = desiredSteeringAngle - 0.1*steeringAngleDx
           count1 = count1+1
        steeringAngleLeft=math.atan(l/(-d+l/math.tan(desiredSteeringAngle)))
        steeringAngleRight=math.atan(l/(d+l/math.tan(desiredSteeringAngle)))  
        vrep.simxSetJointTargetPosition(clientID,steeringLeft,steeringAngleLeft,vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,steeringRight,steeringAngleRight,vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Left,desiredWheelRotSpeed,vrep.simx_opmode_oneshot) #设置非球关节固有目标速度
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Right,desiredWheelRotSpeed,vrep.simx_opmode_oneshot)
        time.sleep(0.02)

def set_velocity(v:float):
    """设置非球关节固有目标速度
    
    """
    global MotorHandle_Left,MotorHandle_Right       
    print('v',v,'\n')
    vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Left,v,vrep.simx_opmode_oneshot)  
    vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Right,v,vrep.simx_opmode_oneshot)
    
def turning(desiredSteeringAngle:float): 
    """逆运算推转动角  
    
    """  
    global  steeringAngleLeft,steeringAngleRight
    
    if abs(desiredSteeringAngle) > 29.77 * math.pi / 180:
        if desiredSteeringAngle > 0: desiredSteeringAngle = 29.77 * math.pi / 180
        else : desiredSteeringAngle = - 29.77 * math.pi / 180
    
    steeringAngleLeft=math.atan(l/(-d+l/math.tan(desiredSteeringAngle)))
    steeringAngleRight=math.atan(l/(d+l/math.tan(desiredSteeringAngle)))
    print(math.degrees(steeringAngleLeft), math.degrees(steeringAngleRight))
    
    
    vrep.simxSetJointTargetPosition(clientID,steeringLeft,steeringAngleLeft,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,steeringRight,steeringAngleRight,vrep.simx_opmode_oneshot)
    
    
def zaroto2pi(x):
    
    """reeds-shepp曲线中的角度调整函数
    
    """  
    if x<-180: x = x+ 360
    elif x>180: x= x-360
    return x