#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer
#reference pose for hyq
from hyq_ref_pose import hyq_ref

from hpp.corbaserver.rbprm.state_alg import *
from numpy import array

#calling script darpa_hyq_path to compute root path
import mount_hyq_path as tp

from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"


from hpp.corbaserver import Client


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
fullBody.setJointBounds ("base_joint_xyz", [-4,6, -1, 1, 0.3, 4])

#  Setting a number of sample configurations used
nbSamples = 20000

ps = tp.ProblemSolver(fullBody)
r = tp.Viewer (ps, viewerClient=tp.r.client)

rootName = 'base_joint_xyz'

cType = "_3_DOF"
rLegId = 'rfleg'
rLeg = 'rf_haa_joint'
rfoot = 'rf_foot_joint'
offset = [0.,-0.021,0.]
normal = [0,1,0]
legx = 0.02; legy = 0.02

def addLimbDb(limbId, heuristicName, loadValues = True, disableEffectorCollision = False):
	fullBody.addLimbDatabase(str(db_dir+limbId+'.db'), limbId, heuristicName,loadValues, disableEffectorCollision)

fullBody.addLimb(rLegId,rLeg,rfoot,offset,normal, legx, legy, nbSamples, "jointlimits", 0.1, cType)

lLegId = 'lhleg'
lLeg = 'lh_haa_joint'
lfoot = 'lh_foot_joint'
fullBody.addLimb(lLegId,lLeg,lfoot,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)
#~ 
rarmId = 'rhleg'
rarm = 'rh_haa_joint'
rHand = 'rh_foot_joint'
fullBody.addLimb(rarmId,rarm,rHand,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)

larmId = 'lfleg'
larm = 'lf_haa_joint'
lHand = 'lf_foot_joint'
fullBody.addLimb(larmId,larm,lHand,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)

fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(rarmId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(larmId, "jointLimitsDistance", True)

#~ q_init = hyq_ref[:]; q_init[0:7] = tp.q_init[0:7]; 
#~ q_goal = hyq_ref[:]; q_goal[0:7] = tp.q_goal[0:7]; 
q_init = hyq_ref[:]; q_init[0:7] = tp.q_init[0:7]; q_init[2]=hyq_ref[2]+0.02
q_goal = hyq_ref[:]; q_goal[0:7] = tp.q_goal[0:7]; q_init[2]=hyq_ref[2]+0.02

# Randomly generating a contact configuration at q_init
#~ fullBody.setCurrentConfig (q_init)
#~ q_init = fullBody.generateContacts(q_init, [0,0,1])

# Randomly generating a contact configuration at q_end
#~ fullBody.setCurrentConfig (q_goal)
#~ q_goal = fullBody.generateContacts(q_goal, [0,0,1])

# specifying the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[rLegId,lLegId,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId,rarmId])
#~ fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId])


r(q_init)
configs = []


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)


from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj

	
	
#~ limbsCOMConstraints = { rLegId : {'file': "hyq/"+rLegId+"_com.ineq", 'effector' : rfoot},  
						#~ lLegId : {'file': "hyq/"+lLegId+"_com.ineq", 'effector' : lfoot},  
						#~ rarmId : {'file': "hyq/"+rarmId+"_com.ineq", 'effector' : rHand},  
						#~ larmId : {'file': "hyq/"+larmId+"_com.ineq", 'effector' : lHand} }
						
limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : rfoot},  
						lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : lfoot},
						rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand},
						larmId : {'file': "hrp2/LA_com.ineq", 'effector' : lHand} }



def initConfig():
	r.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	r(q_init)
	
def endConfig():
	r.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	r(q_goal)
	

def rootPath():
	r.client.gui.setVisibility("hyq", "OFF")
	tp.cl.problem.selectProblem("rbprm_path")
	tp.r.client.gui.setVisibility("toto", "OFF")
	r.client.gui.setVisibility("hyq", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "ON")
	tp.pp(0)
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	r.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	
def genPlan(stepsize=0.06):
	tp.cl.problem.selectProblem("default")
	r.client.gui.setVisibility("hyq", "ON")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	global configs
	start = time.clock() 
	configs = fullBody.interpolate(stepsize, 5, 5, True)
	end = time.clock() 
	print "Contact plan generated in " + str(end-start) + "seconds"
	
def contactPlan(step = 0.5):
	r.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	global configs
	for i in range(0,len(configs)):
		r(configs[i]);
		time.sleep(step)		
		
		
def a():
	print "initial configuration"
	initConfig()
		
def b():
	print "end configuration"
	endConfig()
		
def c():
	print "displaying root path"
	rootPath()
	
def d(step=0.06):
	print "computing contact plan"
	genPlan(step)
	
def e(step = 0.5):
	print "displaying contact plan"
	contactPlan(step)


from bezier_traj import go0, go2, init_bezier_traj, reset
from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import play_trajectory

import time

from hpp.corbaserver.rbprm.rbprmstate import State
from hpp.corbaserver.rbprm.state_alg  import addNewContact, isContactReachable, closestTransform, removeContact, addNewContactIfReachable, projectToFeasibleCom

path = []

def sc(ec):
    pass

def pl(iid = None):
    global path
    if iid == None:
        iid = len(path) -1 
    play_trajectory(fullBody,pp,path[iid])
    
def plc(ctx = 0, iid = None):
    sc(ctx)
    pl(iid)

def go():
    return go0(states, mu=0.6,num_optim=2, use_kin = context == 0)
    
def plall(first = 0):
    global path
    sc(first)
    for pId in range(len(path)):
        play_trajectory(fullBody,pp,path[pId])
        
        

from pickle import load, dump
def save(fname):
    sc(0)
    all_data=[[],[]]
    global states
    for s in states:
        all_data[0]+=[[s.q(), s.getLimbsInContact()]]
    f = open(fname, "w")
    dump(all_data,f)
    f.close()

def load_save(fname):
    f = open(fname, "r+")
    all_data = load (f)
    f.close()
    sc(0)
    global states
    states = []
    #~ for i in range(0,len(all_data[0]),2):
        #~ print "q",all_data[0][i]
        #~ print "lic",all_data[0][i+1]
        #~ states+=[State(fullBody,q=all_data[0][i], limbsIncontact = all_data[0][i+1]) ]
    for _, s in enumerate(all_data[0]):
        states+=[State(fullBody,q=s[0], limbsIncontact = s[1]) ]
	r(states[0].q())
    
def onepath(ol, ctxt=1, nopt=1, mu=1, effector = False):
    reset()
    sc(ctxt)
    global path
    global states
    print "ctxt", ctxt
    print "q", len(states[ol+1].q())
    s = max(norm(array(states[ol+1].q()) - array(states[ol].q())), 1.) * 0.4
    print "s",s
    if(ol > len(path) -1):
        path += [go0([states[ol],states[ol+1]], num_optim=nopt, mu=mu, use_kin = False, s=s, effector = effector)]
    else:
        path[ol]=go0([states[ol],states[ol+1]], num_optim=nopt, mu=mu, use_kin = False, s=s, effector = effector)
    all_paths[ctxt] = path
    
def onepath2(states_subset, ctxt=1, nopt=1, mu=1, effector = False):
    reset()
    sc(ctxt)
    global path
    global states
    #~ print "ctxt", ctxt
    #~ print "q", len(states[ol+1].q())
    #~ s = max(norm(array(states_subset[1].q()) - array(states_subset[0].q())), 1.) * 0.4
    #~ print "s",s
    #~ if(ol > len(path) -1):
    path = all_paths[ctxt][:]
    path += [go2(states_subset, num_optim=nopt, mu=mu, use_kin = False, s=None, effector = effector)]
    #~ else:
        #~ path[ol]=go2(states_subset, num_optim=nopt, mu=mu, use_kin = False, s=s, effector = effector)
    all_paths[ctxt] = path    
    sac()

def save_paths(fname):
    f = open(fname, "w")
    dump(all_paths,f)
    f.close()
    #now try with latest paths
    global all_path
    global path
    sc(0)
    all_paths[0] = path[:]
    f = open(fname+"all", "w")
    dump(all_paths,f)
    f.close()
    
def load_paths(fname):
    f = open(fname, "r")
    global all_paths
    all_paths = load (f)
    f.close()
    sc(0)
    global path
    path = all_paths[0][:]
    
def sh(ctxt, i):
    sc(ctxt)
    r(states[i].q())
    
def lc():
    load_save("19_06_s")
    load_paths("19_06_p")
    #~ save_paths("19_06_p_save")
    save("19_06_s_save")
    
def sac():
    save("19_06_s")
    save_paths("19_06_p")
    
init_bezier_traj(fullBody, r, pp, configs, limbsCOMConstraints)

all_paths = [[],[]]
from hpp.corbaserver.rbprm.state_alg import *
#~ d(0.07);e(0.01)
#~ i=0
#~ d(0.07); e(0.01); states = planToStates(fullBody,configs)

#~ onepath2(states [0:-5],nopt=0,mu=0.3,effector=False)
#~ e(0.01)
#~ 
lc()



from numpy import array, cross
from numpy.linalg import norm

def flat(pts):
    return [item for sublist in pts for item in sublist]

__EPS = 1e-5

def __filter_points(points):
    res = []
    for el in points:
        el_arr = array(el)
        add = True
        for al in res:
            if(norm(al - el_arr) < __EPS):
                add = False
                break
        if add:
            res += [array(el)]
    return res

def __normal(points):
    p1 = array(points[0])
    p2 = array(points[1])
    p3 = array(points[2])
    normal = cross((p2 - p1),(p3 - p1))
    normal /= norm(normal)
    return normal.tolist()
    
def __centroid(points):
    return sum(points) / len(points)

def __centroid_list(list_points):
    return [[__centroid(__filter_points(flat(pts))).tolist(), __normal(pts[0]) ]  for pts in list_points]

def computeAffordanceCentroids(afftool, affordances=["Support","Lean"]):
    all_points = []
    for _, el in enumerate(affordances):
        all_points += afftool.getAffordancePoints(el)
    return __centroid_list(all_points)

b_id = 0

def draw_centroid(gui, winId, pt, scene="n_name", color = [1,1,1,0.3]):
    p = pt[0]
    n = array(pt[0]) + 0.03 * array(pt[1])
    resolution = 0.01
    global b_id
    boxname = scene+"/"+str(b_id)
    boxnameN = scene+"/"+str(b_id)+"n"
    b_id += 1
    gui.addBox(boxname,resolution,resolution,resolution, color)
    gui.addBox(boxnameN,resolution,resolution,resolution, color)
    gui.applyConfiguration(boxname,[p[0],p[1],p[2],1,0,0,0])
    gui.applyConfiguration(boxnameN,[n[0],n[1],n[2],1,0,0,0])
    gui.addSceneToWindow(scene,winId)
    gui.refresh()

def draw_centroids(gui, winId, pts_lists, scene="n_name", color = [1,0,0,1]):
    gui.createScene(scene)
    for _, pt in enumerate(pts_lists):
        draw_centroid(gui, winId, pt, scene=scene, color = color)
    


        
from gen_data_from_rbprm import *
#~ 
#~ afftool = AffordanceTool ()
#~ afftool.setAffordanceConfig('Support', [1., 0.003, 0.00005])
#~ 
suppTargets = computeAffordanceCentroids(tp.afftool, ['Support']) 
#~ leanTargets = computeAffordanceCentroids(tp.afftool, ["Support", 'Lean']) 

def getClosestTarget(ePos):
    aE = array(ePos)
    minDist = 10000
    current = None
    global suppTargets
    for el in suppTargets:
        d = norm(array(el[0]) - aE)
        if (d < minDist):
            current = el[:]
            minDist = d
    return current
    
N = None    

def ptsNormal(p,N):
    nps = [[p[0]+i,p[1]+j,p[2]] for i in [-0.05, 0.05] for j in [-0.05, 0.05]]
    #~ nN = [N for _ in nps] 
    nN = [[0.,0.,1.] for _ in nps] 
    return (nps, nN)

def checkDynamic(com,state, ddcom = array([0.,0.,0.])):
    return fullBody.isConfigBalanced(fullBody.getCurrentConfig(), state.getLimbsInContact(), robustness = -10)
    #~ com = array(state.getCenterOfMass())
    #~ H, h = state.getContactCone(0.6)  
    ps = state.getContactPosAndNormals()
    p = ps[0][0]
    N = ps[1][0]
    global N
    print "PPP"
    print p
    print "PPP"
    nps = []
    nNs = []
    for el in p:
        (retP, retN) = ptsNormal(el,getClosestTarget(el)[1])
        nps += retP
        nNs += retN
    print "NEW PPP"
    print nps
    print "NEW PPP"
    print "NEW nNs"
    print nNs
    #~ N = [getClosestTarget(el)[1]  for el in p]
    # now duplicate waypoints
    # compute waps in local coordinates
    #~ H = compute_CWC(p, N, state.fullBody.client.basic.robot.getMass(), mu = 0.9, simplify_cones = False)
    H = compute_CWC(nps, nNs, state.fullBody.client.basic.robot.getMass(), mu = 0.6, simplify_cones = True)
    c = com 
    #~ print "ddcom", ddcom
    w = compute_w(c, ddcom)       
    if(H.dot( w )<= 50).all():
       return True
    else:
       return False #max(H.dot( w ))

def getCom(config):
    r(config)
    return fullBody.client.basic.robot.getCenterOfMass()

from hpp.corbaserver.rbprm.state_alg  import computeIntermediateState, isContactCreated
def getStatesInterm(s1,s2, configs):
    sInt = computeIntermediateState(s1,s2)
    #get moving limb
    mLimb = list(set(s1.getLimbsInContact()) - set(sInt.getLimbsInContact()))
    if len(mLimb) == 0:
        #~ print "no changes"
        return [s1 for _ in range(len(configs))]
    mLimb = mLimb[0]
    #find when limb is moving
    stemp = State(fullBody,q=s1.q(), limbsIncontact = s1.getLimbsInContact())
    acc = 0.
    posInit =array(s1.getContactPosAndNormalsForLimb(mLimb)[0][0][0])
    posEnd = array(s2.getContactPosAndNormalsForLimb(mLimb)[0][0][0])
    currentState = s1
    takeoff = False
    land = False
    res = []
    for q in configs:
        stemp.setQ(q)
        #~ print "effector pos ", mLimb
        #~ print "effector pos ", array(stemp.getEffectorPosition(mLimb)[0])
        if (not takeoff):
            if(norm(posInit - array(stemp.getEffectorPosition(mLimb)[0])) > 0.05):
                #~ print "takeoff "
                takeoff = True
                currentState = sInt
        elif not land:
            if(norm(posEnd - array(stemp.getEffectorPosition(mLimb)[0])) < 0.05):
                #~ print "landing "
                land = True
                currentState = s2
        #~ print "sid ", currentState.sId
        res +=[currentState]
    return res
            


def getAllEqDyn(first = 0, second = 1, dyn = False):
    coms = []
    sc(0);r(states[0].q());sc(1);r(states[0].q())
    global path
    sc(first)
    pIds = [i for i in range(len(path))]
    cs = [item for sublist in [[[first,i],[second,i]] for i  in [j for j in range(len(path))]] for item in sublist]
    i = 0
    for ctx, pId in cs:   
        #~ if ctx ==  0:   
        sc(ctx) 
        s1 = states[pId]       
        s2 = states[pId+1]
        newCom = [getCom(config) for config in  path[pId]]
        statesss = getStatesInterm(s1,s2, path[pId])
        if dyn:
            dc  = [24.*(array(newCom[i+1])-array(newCom[i])) for i in  range(len(newCom)-1)]
            ddc = [array(dc[i+1])-array(dc[i]) for i in  range(len(dc)-1)]
            #~ print "dyn on", ddc
            eq = [checkDynamic(newCom[i], statesss[i], ddc[i])  for i in  range(len(ddc))]
        else: #quasi static
            eq = [checkDynamic(newCom[i], statesss[i], array([0.,0.,0.])) for i in  range(len(newCom))]
        coms += [eq]
    return coms

b = flatten(getAllEqDyn(dyn = False))
b2 = flatten(getAllEqDyn(dyn = True))
c = [el for el in b if el == False]
c2 = [el for el in b2 if el == False]
if(len(c2) < len(c)):
    b = b2
fname = "validity_moutain_1"
f = open(fname, "w")
dump(b,f)
f.close()
