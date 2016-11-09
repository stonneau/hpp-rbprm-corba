from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
white=[1.0,1.0,1.0,1.0]
green=[0.23,0.75,0.2,0.5]
yellow=[0.85,0.75,0.15,1]
pink=[1,0.6,1,1]
orange=[1,0.42,0,1]
brown=[0.85,0.75,0.15,0.5]
blue = [0.0, 0.0, 0.8, 1.0]
grey = [0.7,0.7,0.7,1.0]
red = [0.8,0.0,0.0,1.0]


rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'robot_test_trunk'
urdfNameRom = ['robot_test_lleg_rom','robot_test_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()
rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-6,6, -3, 3, 0, 2.5])
rbprmBuilder.boundSO3([-0.1,0.1,-1,1,-1,1])
rbprmBuilder.setFilter(['robot_test_lleg_rom', 'robot_test_rleg_rom'])
rbprmBuilder.setAffordanceFilter('robot_test_lleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('robot_test_rleg_rom', ['Support'])
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(3)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0])


#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.loadObstacleModel (packageName, "ground_jump_easy", "planning", r)
afftool.visualiseAffordances('Support', r, brown)

q_init = rbprmBuilder.getCurrentConfig ();
#q_init[(len(q_init)-3):]=[0,0,1] # set normal for init / goal config
q_init [0:3] = [-4, 1, 0.9]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)


q_goal = q_init [::]
#q_goal [0:3] = [-2, 0, 0.9]; r (q_goal) # premiere passerelle
q_goal [0:3] = [3, 1, 0.9]; r (q_goal) # pont




#~ ps.addPathOptimizer("GradientBased")
ps.addPathOptimizer("RandomShortcut")
#ps.client.problem.selectSteeringMethod("SteeringDynamic")
ps.selectPathPlanner("ParabolaPlanner")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)

r(q_init)

#ps.client.problem.prepareSolveStepByStep()
#i = 0
#r.displayRoadmap("rm"+str(i),0.02)
#ps.client.problem.executeOneStep() ;i = i+1; r.displayRoadmap("rm"+str(i),0.02) ; r.client.gui.removeFromGroup("rm"+str(i-1),r.sceneName) ;
#t = ps.solve ()

r.solveAndDisplay("rm",1,0.02)


#t = ps.solve ()

#r.displayRoadmap("rm",0.02)


r.displayPathMap("rmPath",0,0.02)


from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)

pp.displayPath(0,r.color.lightGreen)
pp(0)


pp.displayPath(1,blue)
r.client.gui.setVisibility("path_0_root","ALWAYS_ON_TOP")

pp.displayPath(1,black)
pp (1)

#r.client.gui.removeFromGroup("rm",r.sceneName)
r.client.gui.removeFromGroup("rmPath",r.sceneName)
r.client.gui.removeFromGroup("path_1_root",r.sceneName)
#~ pp.toFile(1, "/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")


math.sqrt((np.linalg.norm(u)*np.linalg.norm(u)) * (np.linalg.norm(v)*np.linalg.norm(v))

from hpp import quaternion as Quaternion
q = Quaternion.Quaternion([1,0,0],[2,3,-1])

