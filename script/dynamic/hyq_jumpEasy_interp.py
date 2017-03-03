#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer

#calling script darpa_hyq_path to compute root path
import hyq_jumpEasy_path as tp

from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"


packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"

#  Information to retrieve urdf and srdf files.
urdfName = "hyq"
urdfSuffix = ""
srdfSuffix = ""

#  This time we load the full body model of HyQ
fullBody = FullBody () 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.setJointBounds ("base_joint_xyz", [-5,5, -1.5, 1.5, 0, 2])

#  Setting a number of sample configurations used
nbSamples = 20000
dynamic=True

ps = tp.ProblemSolver(fullBody)
r = tp.Viewer (ps,viewerClient=tp.r.client)

rootName = 'base_joint_xyz'





def addLimbDb(limbId, heuristicName, loadValues = True, disableEffectorCollision = False):
	fullBody.addLimbDatabase(str(db_dir+limbId+'.db'), limbId, heuristicName,loadValues, disableEffectorCollision)

rLegId = 'rfleg'
lLegId = 'lhleg'
rarmId = 'rhleg'
larmId = 'lfleg'

addLimbDb(rLegId, "forward")
addLimbDb(lLegId, "forward")
addLimbDb(rarmId, "forward")
addLimbDb(larmId, "forward")

q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.ps.configAtParam(0,0.01)[0:7] # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.ps.configAtParam(0,tp.ps.pathLength(0))[0:7]
dir_init = tp.ps.configAtParam(0,0.01)[7:10]
acc_init = tp.ps.configAtParam(0,0.01)[10:13]
dir_goal = tp.ps.configAtParam(0,tp.ps.pathLength(0))[7:10]
acc_goal = tp.ps.configAtParam(0,tp.ps.pathLength(0))[10:13]
configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

fullBody.setStaticStability(False)
# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
q_init = fullBody.generateContacts(q_init,dir_init,acc_init)

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal)

# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = acc_goal[::]
# specifying the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[larmId,rLegId,rarmId,lLegId])
fullBody.setEndState(q_goal,[larmId,rLegId,rarmId,lLegId])


r(q_init)
# computing the contact sequence
# configs = fullBody.interpolate(0.12, 10, 10, True) #Was this (Pierre)
configs = fullBody.interpolate(0.001,pathId=0,robustnessTreshold = 3, filterStates = True)


print "configs size = ",len(configs)
r(configs[-1])



from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

from fullBodyPlayer import Player
player = Player(fullBody,pp,tp,configs,draw=False,optim_effector=False,use_velocity=dynamic,pathId = 0)

#~ player.displayContactPlan()
r(configs[-1])



#add final configuration
hyq_ref = [-1.652528468457493,
 0.06758953014152885,
 0.6938277139631803,
 0.9995752585582991,
 -0.016391515572502728,
 -0.011295242081121508,
 0.02128469407050025,
 0.17905666752078864,
 0.9253512562075908,
 -0.8776870832724601,
 0.11147422537786231,
 -0.15843632504615043,
 1.150049183494211,
 -0.1704998924604114,
 0.6859376445755911,
 -1.1831277202117043,
 0.06262698472369518,
 -0.42708925470675,
 1.2855999319965081]
 

#retrieve ballistic path
pp.client.problem.extractPath(0,1,1.1)
#now for the crazy interpolation
time_to = 0 #fullBody.getTimeAtState(len(configs[-1]))
time_land = ps.pathLength(1)

#first remove useless last state
fullBody.removeState(len(configs)-1)
configs=configs[:-1][:]

#then replace last config before jump
to_conf = configs[-1]
#project it onto ground
to_root_config = pp.client.problem.configAtParam(1,time_to)
to_com = to_root_config[0:3]
q_to= (fullBody.projectToCom(len(configs)-1,to_com))
#~ to_com = to_root_config[0:7]
#~ q_to = to_conf[:]; q_to[0:7] = to_com

#remove previous config and replace it with new one
fullBody.setStateConfig(len(configs)-1,q_to)
configs[-1]=q_to
#add state with no contact for interpolation
configs+=[q_to]
fullBody.addState(q_to,[],0)

#uncomment to interpolate on first steps
#~ player.interpolate(1,len(configs)-1)


#computing a suitable end configuration
q_land = configs[-1][:]
q_land[0:len(hyq_ref)] = hyq_ref
land_root_config = pp.client.problem.configAtParam(1,time_land)
land_com = land_root_config[0:3]
q_land[0:len(land_root_config)-6] = land_root_config[:-6]
q_land = fullBody.generateContacts(q_land,dir_init,acc_init)

#and project it to final com
state_land_id = len(configs)
fullBody.addState(q_land,[rLegId,lLegId,rarmId,larmId],time_land)
#~ projectToCom
#~ target_config = q_land[:];
target_config = (fullBody.projectToCom(state_land_id,land_com))
configs+=[target_config[:]]
fullBody.removeState(state_land_id)
fullBody.addState(target_config,[],time_land)

#extract flying part of path
#~ pp.client.problem.extractPath(0,time_to,time_land)
#~ fullBody.comRRT(state_land_id-1, state_land_id, 1, numOptim=0)

"""
comRRT(self, state1, state2, path, numOptim=10)
camera = [0.5681925415992737,
 -6.707448482513428,
 2.5206544399261475,
 0.8217507600784302,
 0.5693002343177795,
 0.020600343123078346,
 0.01408931240439415]
r.client.gui.setCameraTransform(0,camera)

"""






