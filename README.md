# Optimized-RRT-algorithm-for-UAV
程序设定路径追踪与Reeds-shepp曲线的姿态微调放在同一流程中，如果想要单独检视最终车辆姿态微调过程，可以点开adjust.py文件进行检测（上述的所有流程都可以进行单独检测。
文件夹下:	

RRT.py对应优化RRT*核心算法流程	

Path_Planning.py对应执行RRT算法环节

Path_Pruning.py对应剪枝环节

Path_Smooth.py对应路径平滑

Stanley.py对应路径追踪文件

adjust.py对应最终车辆姿态微调环节。这里需要注意的是adjust.py所进行的车辆姿态微调是没有设置障碍物检测的，所以需要在空旷区域进行验证，因为其实际应用背景是在Stanley追踪完成后，车辆距离最终目标无论是姿态还是具体都十分接近的情况下。

具体的软件说明可以见文件：软件说明书_ReadMe.docx
