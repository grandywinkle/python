# coding = utf-8
import sys
import vrep
import time
import os
import numpy as np
c = np.cos
s = np.sin
pi = np.pi

class Env()


class Viewer(object):
    emptyBuff = bytearray()
    def __init__(self):
        self.clientID = self.connectVrep()
        self.ObjectName =  ['target','tip','tip2']
        self.robotName = 'UR5'
        self.targetName,self.tipName,self.tip2Name = self.ObjectName
        self.robotAxieNum = 6
        self.jointHandles,self.objectHandles =self.getHandles()
    # 连接Vrep客户端，这里使用的是默认端口号19997，
    # 如果不使用这个端口号，则需要在Vrep端使用remoterServerStart函数开启目标端口号
    def connectVrep(self):
        vrep.simxFinish(-1)
        '''默认19997'''
        clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        # 当clientID为-1的时候连接不上
        if clientID != -1:
            print('Connected to remote API server')
            vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        else:
            print('Connected failed!!!')
            sys.exit('Couldn\'t connect API server')
        return clientID
    # 获取Robot所有关节句柄、获取所有需要定位的Object句柄
    def getHandles(self):
        # 可能自由度过大的情况下，训练时间过长，因此调整为一个可变的数值，与action_dim对应上
        jointHandles = [-1]*self.robotAxieNum
        for i in range(self.robotAxieNum):
            errorCode,jointHandles[i] = vrep.simxGetObjectHandle(self.clientID,
                                    self.robotName+'_joint'+str(i+1),vrep.simx_opmode_oneshot_wait)
        # 获取ObjectName中的所有对象句柄
        objectHandles = [-1]*len(self.ObjectName)
        for j in range(len(self.ObjectName)):
            errorCode,objectHandles[j] = vrep.simxGetObjectHandle(self.clientID,
                                                    self.ObjectName[j], vrep.simx_opmode_oneshot_wait)
        return jointHandles,objectHandles
    # 由于python远程获取的位置不对，所以考虑使用通信方式，从本地Lua脚本获取
    def getObjectPosition(self):
        objectPosition = [-1]*len(self.ObjectName)
        for i in range(len(self.ObjectName)):
            res, resInts, objectPosition[i], retString, retBuffer = vrep.simxCallScriptFunction(self.clientID,
                                                                                          'remoteCommander',
                                                                                          vrep.sim_scripttype_childscript,
                                                                                          'getPositions',
                                                                                          [self.objectHandles[i]], [], [],
                                                                                          self.emptyBuff,
                                                                                          vrep.simx_opmode_oneshot_wait)
        # print(objectPosition)
        return objectPosition
    # 控制N个joint运动到指定角度
    def render(self,angles):
        time.sleep(0.0001)
        for i in range(len(self.jointHandles)):
            vrep.simxSetJointPosition(self.clientID, self.jointHandles[i], angles[i], vrep.simx_opmode_oneshot)

    # 单轴测试
    def test(self,jointNum,angle):
        vrep.simxSetJointPosition(self.clientID,self.jointHandles[jointNum-1],angle,vrep.simx_opmode_oneshot)

    # 在程序结束时停止仿真
    def __del__(self):
        vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)
'''精简版的FK计算'''
DH_UR5 =   {'d':  [0.0892,    0,      0.0,        0.110,   0.09475,   -0.01442],
          'theta':[0.0,     -pi/2,    0.0,       -pi/2,    0.0,        0.0],
          'a':    [0.0,    0.00,   -0.4251,    -0.39215,  0.0,       0.0],
          'alpha':[0.0,   pi/2,       0.0,        0.0,    pi/2,      -pi/2]}
class FK(object):
    def __init__(self,DH_parameter):
        self.DH = DH_parameter
    def DH_update(self, jointAngles):
        self.DH.update({'theta':[x for x in jointAngles]})
    def controlAngle_to_CalculateAngle(self,control_angle):
        calculate_angle = control_angle
        calculate_angle[1]-=pi/2
        if len(control_angle)>3:
            calculate_angle[3]-=pi/2
        return calculate_angle
    def TCalFormula(self,DH_link,DH):
        d = DH['d'][DH_link-1]
        t = DH['theta'][DH_link - 1]
        a = DH['a'][DH_link - 1]
        al = DH['alpha'][DH_link - 1]
        TFormula = np.array([[c(t),       -s(t),       0,         a ],
                             [s(t)*c(al), c(t)*c(al), -s(al), -d*s(al)],
                             [s(t)*s(al), c(t)*s(al), c(al),  d*c(al)],
                             [0,           0,           0,         1]])
        return TFormula
    # 根据DH参数计算每两个连杆的变换矩阵,初始值
    def TSingleCalculate(self, angle_dim,DH):
        T = []
        for i in range(1, angle_dim+1):
            T.append(self.TCalFormula(i, DH))
        return T

    '''计算世界坐标到末端坐标的变换矩阵'''
    def calT0_to_end(self,angle):
        joint_angle = self.controlAngle_to_CalculateAngle(angle)
        self.DH_update(joint_angle)
        TMatrix = self.TSingleCalculate(len(angle),self.DH)
        # print(TMatrix)
        T0_to_end = np.eye(4,4)
        for i in range(len(TMatrix)):
            T0_to_end = np.dot(T0_to_end,TMatrix[i])
        return T0_to_end
    def calTipPositions(self,angle,tip,tip2):
        T0_to_end = self.calT0_to_end(angle)
        tipPosition = np.dot(T0_to_end,(np.array([tip[0],tip[1],tip[2],1])))[0:3]
        tip2Position = np.dot(T0_to_end,(np.array([tip2[0],tip2[1],tip2[2],1])))[0:3]
        # print(tipPosition,tip2Position)
        return tipPosition,tip2Position
def testFK():
    fk = FK(DH_UR5)
    viewer = Viewer()
    viewer.getObjectPosition()
    for i in range(1000):
        test_angle = np.random.rand(6) * 2 * pi - pi
        viewer.render(test_angle)
        target, tip, tip2 = viewer.getObjectPosition()
        tip_cal, tip2_cal = fk.calTipPositions(test_angle, [0, 0, 0.15], [0, 0, 0.1])
        tip_minus = [x - y for x, y in zip(tip, tip_cal)]
        tip2_minus = [x - y for x, y in zip(tip2, tip2_cal)]
        for x, y in zip(tip_minus, tip2_minus):
            if abs(x) > 1e-5 or abs(y) > 1e-5:
                print('误差超过了容许范围，计算有误！！！')
                break
        # print('计算tip坐标与实际tip坐标差值为',tip_minus,tip2_minus)
        time.sleep(0.01)
if __name__ =='__main__':
    testFK()