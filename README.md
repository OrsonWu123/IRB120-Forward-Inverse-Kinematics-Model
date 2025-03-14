# IRB120-Forward-Inverse-Kinematics-Model

这是吴芝伟的IRB120机械臂MATLAB仿真程序。内容包括IRB120的建模、正运动学、逆运动学和基于五次多项式的轨迹规划。
若有需要，可联系作者邮箱：18980515825@163.com

另外，程序需注意事项：

1.该程序仿真环境为Robotics Toolbox，作者为Peter Corke。相关下载文件附在程序文件内。

2.程序运行时默认为弧度制，角度制下可能会导致失败。

3.主运行文件为IRB120.m，将需要运行的程序名数值改为1即可运行，同一时间只能运行一个程序。另外如果需要得到该仿真的不同数据，则需要在程序处更改标注内容。

-------------------------

This is a MATLAB simulation program of IRB120 robotic arm by Orson Wu. It covers the modeling of IRB120, forward kinematics, inverse kinematics and trajectory planning based on fifth degree polynomials.
If necessary, you can contact the author at 18980515825@163.com

In addition, the program requires precautions:
1. The simulation environment of this program is Robotics Toolbox, authored by Peter Corke, and the related download file is attached to the program file.
2. The program runs in radians by default, and may fail in angles.
3. The main runtime file is IRB120.m, change the value of the program name you need to 1 to run one, and you can only run one program at each time. If you need to get different data of the simulation, you need to change the trajectory setting data at the program.
