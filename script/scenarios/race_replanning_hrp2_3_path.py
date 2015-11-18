from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
import imp
import pickle
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
rbprmBuilder.setJointBounds ("base_joint_xyz", [-1.6,2, -1.5, 0, 0.547, 1.3])
rbprmBuilder.setFilter(['hrp2_lleg_rom','hrp2_rleg_rom','hrp2_rarm_rom'])
rbprmBuilder.boundSO3([-0.,0,-1,1,-1.5,1.5])

rbprmBuilder.setNormalFilter('hrp2_lleg_rom', [0,0,1], 0.9)
rbprmBuilder.setNormalFilter('hrp2_rleg_rom', [0,0,1], 0.9)

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)

path0 = pickle.load(open( "race_replanning_hrp2_2_path_configs", "rb" ))

q_init = path0[38]

q_goal = q_init [::]
q_goal [0:3] = [1.49, -0.65, 1.25]; r (q_goal)

ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.5)
r.loadObstacleModel (packageName, "race_3", "planning")
r (q_init)
t = ps.solve ()

print ("solving time " + str(t));
f = open('log.txt', 'a')
f.write("path computation " + str(t) + "\n")
f.close()




from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp(1)
path3 = pTc.pathToConfigs(r,ps,1,0.5)
pickle.dump(path3, open( "race_replanning_hrp2_4_path_configs", "wb" ))
rbprmBuilder.exportPath (r, ps.client.problem, 1, 0.1, "race_replanning_hrp2_4_path.txt")
r (q_init)
