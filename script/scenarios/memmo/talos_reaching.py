from hpp.gepetto import Viewer
from tools.display_tools import *
from hpp.corbaserver import ProblemSolver
import time
from math import sqrt


import argparse, numpy as np
from hpp import Quaternion, Transform
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import loadServerPlugin, createContext

from hpp.corbaserver.robot import Robot, HumanoidRobot


from tools.display_tools import *

HumanoidRobot.packageName = "talos_data"
HumanoidRobot.urdfName = "talos"
HumanoidRobot.urdfSuffix = "_full_v2"
HumanoidRobot.srdfSuffix = ""
rootJointType = "freeflyer"


robot = HumanoidRobot("talos", rootJointType)
robot.leftAnkle = "leg_left_6_joint"
robot.rightAnkle = "leg_right_6_joint"
robot.setJointBounds("root_joint", [-0, 2, -2, 2, 0, 2])
robot.setJointBounds("root_joint", [-0., 0.2, -0.64, -0.66, 1., 1.1])
robot.setJointBounds("root_joint", [-0., 0.2, -2, 2, 1., 1.1])

ps = ProblemSolver(robot)
ps.setErrorThreshold(1e-3)
ps.setMaxIterProjection(40)

#init config 

init_conf = [0, 0, 0, 0, 0, 0, 1, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0, 0.006761, 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
init_conf[0:7] = [0.1, -0.65, 1.0192720229567027, 0, 0, sqrt(2) / 2, sqrt(2) / 2]  # root_joint

#~ ps.resetConstraints()

ps.createOrientationConstraint(
        "waist_yaw", "", "root_joint", (0, 0, 0, 1), [True, True, True] )
    
ps.addPartialCom("talos", ["root_joint"])

robot.createStaticStabilityConstraint(
        "balance", "talos", robot.leftAnkle, robot.rightAnkle, init_conf)
    
def addCons(ps):
    #~ ps.addNumericalConstraints('c',['waist_yaw','balancerelative-com', 'balancepose-left-foot', 'balancepose-right-foot'])
    ps.addNumericalConstraints('c',['balancerelative-com', 'balancepose-left-foot', 'balancepose-right-foot'])
    #~ ps.addNumericalConstraints('c',['balancepose-left-foot', 'balancepose-right-foot'])
    

posIdx = 0

def genTarget(ps, effector):
    global posIdx
    ok = False
    while not ok:
        ps.resetConstraints()
        q = robot.shootRandomConfig()
        q [:7] = init_conf[:7]
        robot.setCurrentConfig(q)
        #~ v(q)
        p = robot.getJointPosition(effector) [:3]
        pointName = "point"+str(posIdx)
        ps.createPositionConstraint(
        pointName,
        '',
        effector,
        p,
        (0, 0, 0),
        (True, True, True))
        
        ps.addNumericalConstraints('pose',[pointName])
        addCons(ps)
        
        res = ps.applyConstraints(init_conf)
        ok = res[0] and robot.isConfigValid(res[1])[0]
        q = res[1]
        posIdx += 1
    #~ v(q)
    ps.resetConstraints()
    addCons(ps)
    return q


ps.loadPlugin("spline-gradient-based.so")
#~ ps.addPathOptimizer("SimpleTimeParameterization")



#viewer
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)
v = Viewer (ps, displayCoM = True)

#load obstacles
v.loadObstacleModel("gerard_bauzil", "rolling_table", "planning")

def plan(ps, effector):
    q1 = genTarget(ps, effector)
    q = genTarget(ps, effector)
    #~ addCons(ps)
    ps.setInitialConfig(q1)
    ps.addGoalConfig(q)
    #~ v(q)
    



import os

EXPORT_PATH = "/media/data/memmo/talos_reach0/"
DT = 0.01
EFFECTORS = ['gripper_right_fingertip_1_joint','gripper_left_fingertip_1_joint' ]
# ~ if not os.path.exists(EXPORT_PATH):
        # ~ os.makedirs(EXPORT_PATH)
N = 5
#create session dir
import datetime
session = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
sessionPath = EXPORT_PATH+session
os.makedirs(sessionPath)


from mlp.utils import wholebody_result as wr
from mlp.utils.util import SE3FromConfig, SE3toVec
import eigenpy
eigenpy.switchToNumpyMatrix()
def exportPath(pId, directoryPath):
    ln = ps.pathLength(pId)
    nIters = (int)(ln / DT)
    qs = []
    cs = []    
    ts = []    
    effectorTraj = {}
    for effector in EFFECTORS:
        effectorTraj[effector] = []
    for dt in range(nIters):
        t = dt * DT
        ts += [t]
        qs += [ps.configAtParam(pId,t)]; 
        v(qs[-1])
        cs += [robot.getCenterOfMass()]
        for effector in EFFECTORS:
            effectorTraj[effector] += [np.array(SE3toVec(SE3FromConfig(robot.getJointPosition(effector)))).reshape(-1,).tolist()]
    # ~ qs += [ps.configAtParam(pId,ln)]
    # ~ ts += [ln]
    # ~ v(qs[-1])
    # ~ cs += [robot.getCenterOfMass()]
    r = wr.Result(len(robot.getCurrentConfig()),len(robot.getCurrentVelocity()),DT,EFFECTORS,len(qs))
    r.q_t[:] = np.matrix(qs).T
    r.c_t[:] = np.matrix(cs).T
    r.t_t[:] = np.array(ts)    
    for effector in EFFECTORS:
        print (r.effector_trajectories[effector].shape)
        print (np.matrix(effectorTraj[effector]))
        r.effector_trajectories[effector][:] = np.matrix(effectorTraj[effector]).T
    r.phases_intervals = None
    fname = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
    r.exportNPZ(directoryPath,fname)
    return r



from hpp.gepetto import PathPlayer
pp = PathPlayer (v)

pps = []

for effector in EFFECTORS:
    directoryPath = sessionPath + "/" + effector
    for i in range(N):
        ps.resetGoalConfigs()
        plan(ps, effector)
        ps.solve()
        pp(ps.numberPaths()-1)
        pps += [ps.numberPaths()-1]
        r = exportPath(ps.numberPaths()-1, directoryPath)
