from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer


packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = "_reduced"
srdfSuffix = ""

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-5, 5, -5, 5, -5, 5])

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( fullBody )
ps.addPathOptimizer("RandomShortcut")
ps.addPathOptimizer("GradientBased")
r = Viewer (ps)
r.loadObstacleModel ('hpp-rbprm-corba', "scene_wall", "car")

#~ AFTER loading obstacles
rLegId = '7rLeg'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,-0.105,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 20000, 0.01)

lLegId = '8lLeg'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,-0.105,0]
lLegNormal = [0,1,0]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 20000, 0.01)

rarmId = '3Rarm'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [-0.05,-0.050,-0.050]
rArmNormal = [1,0,0]
rArmx = 0.024; rArmy = 0.024
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 20000, 0.01)


#~ AFTER loading obstacles
larmId = '4Larm'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.05,-0.050,-0.050]
lArmNormal = [1,0,0]
lArmx = 0.024; lArmy = 0.024
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 20000, 0.01)

#~ rLegId = '5RKnee'
#~ rLeg = 'RLEG_JOINT0'
#~ rKnee = 'RLEG_JOINT3'
#~ rLegOffset = [0.105,0.055,0.017]
#~ rLegNormal = [-1,0,0]
#~ rLegx = 0.05; rLegy = 0.05
#~ fullBody.addLimb(rLegId, rLeg,rKnee,rLegOffset,rLegNormal, rLegx, rLegy, 5000, 0.01)

#~ lLegId = '6LKnee'
#~ lLeg = 'LLEG_JOINT0'
#~ lKnee = 'LLEG_JOINT3'
#~ lLegOffset = [0.105,0.055,0.017]
#~ lLegNormal = [-1,0,0]
#~ lLegx = 0.05; lLegy = 0.05
#~ fullBody.addLimb(lLegId,lLeg,lKnee,lLegOffset,lLegNormal, lLegx, lLegy, 5000, 0.01)
#~  	


q_0 = fullBody.getCurrentConfig (); r (q_0)

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('LLEG_JOINT3',[1.5])
#~ fullBody.client.basic.robot.setJointConfig('RLEG_JOINT3',[1.5])
#~ 
fullBody.client.basic.robot.setJointConfig('base_joint_SO3',[0.7316888688738209, 0, -0.6816387600233341, 0]); q_init = fullBody.getCurrentConfig (); r (q_init)

q_init = fullBody.getCurrentConfig (); r (q_init)
q_init [0:3] = [-0.1, 0, 0.3]; fullBody.setCurrentConfig (q_init); r (q_init)
q_0 [0:3] = [-0.2, 0, 0.3]; r (q_0)


q_goal = q_init [::]
#~ q_goal [0:3] = [0.2, -0.5, 0.3]

q_goal = [-0.58,
 0.0,
 0.6,
 1.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.8165569353186153,
 0.01904481617296847,
 0.5757001393867748,
 0.0349066,
 -0.5759795881447421,
 -0.8458895038625474,
 0.0,
 0.999644764477483,
 -0.27941899714496077,
 0.7887881502725462,
 -0.3842797425576481,
 -0.8266261474212095,
 -0.7322957359271716,
 0.0,
 -0.06084265166231969,
 5.531954151015379e-05,
 0.07882462690145475,
 0.9267587094966957,
 0.5652483098600809,
 -0.06099150171806746,
 -0.2825554499271455,
 0.0026828522588598323,
 0.07421013928536811,
 0.9328970238734515,
 0.5644449731454055,
 -0.2825451621281345]


ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
r(q_goal)
ps.solve ()

from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

#~ pp (1)
#~ 
q_init = fullBody.generateContacts(q_init, [0,0,-1])
r (q_init)

fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,-1])

fullBody.setStartState(q_init,[rLegId,lLegId,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId,larmId])
#~ 
configs = fullBody.interpolate(0.005)
#~ configs2 = fullBody.interpolate(0.05)
i = 0; 
r (configs[i]); i=i+1; i-1

#~ configs = fullBody.getContactSamplesIds(rLeg, q_init, [0,1,0])
#~ i = 0
#~ q_init = fullBody.getSample(rLeg, int(configs[i])); i = i+1;r(q_init)
#~ 
#~ fullBody.setCurrentConfig (q_init)
#~ q_init = fullBody.generateContacts(q_init, [-0.1,0,1]) ; r(q_init)
#~ r (q_init)
#~ 
#~ q_goal = q_init [::]
#~ q_goal [0:3] = [1, -0.5, 0.6]

#~ r (q_0)
#~ fullBody.setCurrentConfig (q_0)
#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('LLEG_JOINT3',[1.5])
#~ fullBody.client.basic.robot.setJointConfig('RLEG_JOINT3',[1.5])

#~ fullBody.client.basic.robot.setJointConfig('base_joint_SO3',[0.7316888688738209, 0, 0.6816387600233341, 0]); q_init = fullBody.getCurrentConfig (); r (q_init)
#~ q_init = fullBody.getCurrentConfig (); r (q_init)
#~ q_init [0:3] = [0, -0.5, 0.2]; fullBody.setCurrentConfig (q_init); r (q_init)
#~ q_init = fullBody.generateContacts(q_init, [-0.1,0,1]) ; r(q_init)
