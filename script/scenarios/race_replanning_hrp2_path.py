from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
import pickle
import imp
pTc = imp.load_source('pathToConfigs', '../tools/pathToConfigs.py')

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'hrp2_trunk_flexible'
urdfNameRoms =  ['hrp2_larm_rom','hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()

rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-5,2, -1, 1, 0.5, 2.2])
rbprmBuilder.setFilter(['hrp2_lleg_rom','hrp2_rleg_rom'])
#~ rbprmBuilder.setNormalFilter('hrp2_rarm_rom', [0,0,1], 0.5)
#~ rbprmBuilder.setNormalFilter('hrp2_lleg_rom', [0,0,1], 0.3)
#~ rbprmBuilder.setNormalFilter('hrp2_rleg_rom', [0,0,1], 0.3)
#~ rbprmBuilder.setNormalFilter('hyq_rhleg_rom', [0,0,1], 0.9)
rbprmBuilder.boundSO3([-0.,0,-1,1,-1.5,1.5])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)


q_init = rbprmBuilder.getCurrentConfig ();
q_init [0:3] = [0, -0.82, 0.648702]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
q_init [0:3] = [-4.89, -0.82, 0.62]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)

q_goal = q_init [::]
q_goal [3:7] = [ 0.98877108,  0.        ,  0.14943813,  0.        ]
q_goal [0:3] = [1.49, -0.65, 1.2]; r (q_goal)

ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
r.loadObstacleModel (packageName, "race_0", "planning")
r (q_init)
t = ps.solve ()
print ("solving time " + str(t));




from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp(1)
path0 = pTc.pathToConfigs(r,ps,1,0.1)
pickle.dump(path0, open( "race_replanning_hrp2_0_path_configs", "wb" ))
rbprmBuilder.exportPath (r, ps.client.problem, 1, 0.1, "race_replanning_hrp2_0_path.txt")
