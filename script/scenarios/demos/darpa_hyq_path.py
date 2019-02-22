# Importing helper class for setting up a reachability planning problem
from hpp.corbaserver.rbprm.hyq_abstract import Robot

# Importing Gepetto viewer helper class
from hpp.gepetto import Viewer


# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
rbprmBuilder.setJointBounds ("root_joint", [-2,5, -1, 1, 0.3, 4])
# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact ...
rbprmBuilder.setFilter(['hyq_rhleg_rom', 'hyq_lfleg_rom', 'hyq_rfleg_rom','hyq_lhleg_rom'])
rbprmBuilder.setAffordanceFilter('hyq_rhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_rfleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('hyq_lhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_lfleg_rom', ['Support',])
# We also bound the rotations of the torso.
rbprmBuilder.boundSO3([-0.4,0.4,-0.3,0.3,-0.3,0.3])

# Creating an instance of HPP problem solver and the viewer
from hpp.corbaserver.problem_solver import ProblemSolver
ps = ProblemSolver( rbprmBuilder )
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)


from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
#~ afftool.loadObstacleModel (packageName, "darpa", "planning", r, reduceSizes=[0.05,0.,0.])
afftool.loadObstacleModel("hpp_environments", "multicontact/darpa", "planning", vf,reduceSizes=[0.1,0,0])
v = vf.createViewer()
#afftool.visualiseAffordances('Support', v, [0.25, 0.5, 0.5])



# Setting initial and goal configurations
q_init = rbprmBuilder.getCurrentConfig ();
q_init [0:3] = [-2, 0, 0.64]; rbprmBuilder.setCurrentConfig (q_init); v (q_init)
q_goal = q_init [::]
q_goal [0:3] = [3, 0, 0.64]; v (q_goal)
#~ q_goal [0:3] = [-1.5, 0, 0.75]; r (q_goal)

# Choosing a path optimizer
ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# Choosing RBPRM shooter and path validation methods.
# Note that the standard RRT algorithm is used.
ps.client.problem.selectConfigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)

# Solve the problem
#~ t = ps.solve ()
#~ t = 0.

from numpy import array, zeros
from numpy.linalg import norm
from scipy.spatial import ConvexHull

##############################################" affichage ##################################""

def toRotationMatrix(q):
        """
        Returns a (3*3) array (rotation matrix)
        representing the same rotation as the (normalized) quaternion.
        """
        rm=zeros((3,3))
        rm[0,0]=1-2*(q[2]**2+q[3]**2)
        rm[0,1]=2*q[1]*q[2]-2*q[0]*q[3]
        rm[0,2]=2*q[1]*q[3]+2*q[0]*q[2]
        rm[1,0]=2*q[1]*q[2]+2*q[0]*q[3]
        rm[1,1]=1-2*(q[1]**2+q[3]**2)
        rm[1,2]=2*q[2]*q[3]-2*q[0]*q[1]
        rm[2,0]=2*q[1]*q[3]-2*q[0]*q[2]
        rm[2,1]=2*q[2]*q[3]+2*q[0]*q[1]
        rm[2,2]=1-2*(q[1]**2+q[2]**2)
        return rm

boxId = 0
scene = "boxes"
def init_scene(gui, winId): #  v.client.gui
        boxid = gui.createScene(scene)
        gui.addSceneToWindow(scene,winId)
        gui.refresh()
      
def rot_quat_x(a):
        x = [1.,0.,0.]
        return rbprmBuilder.clientRbprm.rbprm.rotationQuaternion(x,a)
        
def rot_mat_x(a):
        x = [1.,0.,0.]
        q = rbprmBuilder.clientRbprm.rbprm.rotationQuaternion(x,a)
        return toRotationMatrix(q)
        
        

def display_box(gui,a,b,y,z):
        global scene
        global boxId
        ar_a = array(a)
        ar_b = array(b)
        x_len = norm(ar_b - ar_a)
        x_pos = ar_a + (ar_b  - ar_a) / 2
        boxname = scene+"/b"+str(boxId); boxId = boxId +1
        gui.addBox(boxname,x_len/ 2,y*x_len,z*x_len, [1.,0.,0.,1.])
        config = x_pos.tolist()+rot_quat_x( ((ar_b - ar_a) / x_len).tolist() )
        gui.applyConfiguration(boxname,config)
        gui.refresh()
        
gui = v.client.gui
winId = 0
init_scene(gui, winId)

def to_ineq(a,b,y_r,z_r):
        a_r = array(a); b_r = array(b);
        normba = norm(b_r - a_r)
        x_dir = (b_r - a_r) / normba
        x_pos = a_r + (b_r  - a_r) / 2
        
        x = normba / 2.
        y = y_r * normba;
        z = z_r * normba;
        
        points = [ [x,-y,z], [x,-y,z], [-x,-y,z], [x,-y,-z], [x,y,-z], [x,y,z], [-x,y,z], [-x,y,-z] ]
        #transform
        rot = rot_mat_x(x_dir.tolist())
        points = [rot.dot(array(el)) + x_pos for el in points]
        ineq = ConvexHull(points).equations
        return ineq[:,:-1],-ineq[:,-1]
          
        

##############################################" CALCUL ##################################""

# a et b sont les extremites du rrt
# y le poids initial alloue a la largeur (si la distance ab vaut 1, initialement la largeur vaut y)
# z le poids initial alloue a la hauteur (si la distance ab vaut 1, initialement la hauteur vaut y)
# sizeObject dimension de securite de l'effecteur
def large_col_free_box(a,b,y = 0.5 ,z = 0.2, sizeObject = 0.05, margin = 0.):
        # margin distance is not so good, better expand and then reduce box
        # d abord o nessaie de trouver une boite sans collsion
        collision = True
        a_r = array(a); b_r = array(b); y_r = y; z_r = z
        x_dir = (b_r - a_r) / norm(b_r - a_r)  
        distance = 0
        maxiter = 100
        while(collision and maxiter > 0):
                maxiter = maxiter -1
                distance = rbprmBuilder.clientRbprm.rbprm.isBoxAroundAxisCollisionFree(a_r.tolist(),b_r.tolist(),[y_r,z_r],margin)
                collision = not distance > 0
                if(collision):
                        y_r = y_r* 0.5; z_r = z_r* 0.5 #todo handle penetration to be smarter
        if collision:
                print "failed"
                return -1
        # now for the largest box possible
        else:           
                maxiter = 100
                while(not collision and distance > 0.01 and maxiter > 0):
                        maxiter = maxiter - 1
                        #find meaning of distance
                        x_len = norm(b_r - a_r)
                        x_dir = (b_r - a_r) / x_len
                        scale = x_len + (distance) /(2* x_len)
                        x_pos = a_r + (b_r  - a_r) / 2
                        tmp_a_r = (x_pos - x_dir * scale * x_len / 2.)
                        tmp_b_r = (x_pos + x_dir * scale * x_len / 2.)                 
                        distance2 = rbprmBuilder.clientRbprm.rbprm.isBoxAroundAxisCollisionFree(tmp_a_r.tolist(),tmp_b_r.tolist(),[y_r,z_r],margin)                        
                        collision = not distance2 > 0
                        if not collision:
                                break
                        else:              
                                if abs(distance2 - distance) < 0.01 or distance2 > distance:
                                        break
                                a_r = tmp_a_r[:]
                                b_r = tmp_b_r[:]          
                                distance = distance2
        # now we have reached maximum uniform scaling, so we play a bit along each axis.
        eps = 0.05
        
         
        
        maxiter = 20
        collision = False
        while(not collision and maxiter>0):
                maxiter =  maxiter -1;
                # start with b
                tmp_b_r = b_r + x_dir * eps
                # adapt scaling of y and z
                x_len = norm(b_r - a_r)
                tmp_y_r = (x_len * y_r) / (x_len + eps)
                tmp_z_r = (x_len * z_r) / (x_len + eps)
                distance = rbprmBuilder.clientRbprm.rbprm.isBoxAroundAxisCollisionFree(a_r.tolist(),tmp_b_r.tolist(),[tmp_y_r,tmp_z_r],margin)                        
                collision = not distance > 0
                if collision:
                        break
                else:
                        b_r = tmp_b_r[:]
                        y_r = tmp_y_r     
                        z_r = tmp_z_r    
        maxiter = 20
        collision = False
        while(not collision  and maxiter>0):
                maxiter =  maxiter -1;
                # start with a
                tmp_a_r = a_r - x_dir * eps
                x_len = norm(b_r - a_r)
                tmp_y_r = (x_len * y_r) / (x_len + eps)
                tmp_z_r = (x_len * z_r) / (x_len + eps)
                distance = rbprmBuilder.clientRbprm.rbprm.isBoxAroundAxisCollisionFree(tmp_a_r.tolist(),b_r.tolist(),[tmp_y_r,tmp_z_r],margin)                        
                collision = not distance > 0
                if collision:
                        break
                else:
                        a_r = tmp_a_r[:]  
                        y_r = tmp_y_r     
                        z_r = tmp_z_r       
                        
        
        maxiter = 50
        collision = False
        while(not collision  and maxiter>0):
                maxiter =  maxiter -1;
                # start with a
                tmp_y_r = y_r + y_r * 0.05
                distance = rbprmBuilder.clientRbprm.rbprm.isBoxAroundAxisCollisionFree(a_r.tolist(),b_r.tolist(),[tmp_y_r,z_r],margin)                        
                collision = not distance > 0
                if collision:
                        break
                else:
                        y_r = tmp_y_r     
                        
        maxiter = 50
        collision = False
        while(not collision  and maxiter>0):
                maxiter =  maxiter -1;
                # start with a
                tmp_z_r = z_r + z_r * 0.05
                distance = rbprmBuilder.clientRbprm.rbprm.isBoxAroundAxisCollisionFree(a_r.tolist(),b_r.tolist(),[tmp_y_r,z_r],margin)                        
                collision = not distance > 0
                if collision:
                        break
                else:
                        z_r = tmp_z_r     
        
        #removing offset
        
        a_r = (a_r + x_dir*sizeObject/2).tolist()
        b_r = (b_r - x_dir*sizeObject/2).tolist()        
        
        return (a_r, b_r, y_r, z_r), to_ineq(a_r, b_r, y_r, z_r)
        
##############################################" TEST ##################################""
(a, b, y, z),(H,h) = large_col_free_box([0.5,0.,1.1],[0.,0.,0.4],.1,.1, 0.)
display_box(gui,a,b,y,z)
(a, b, y, z),(H,h) = large_col_free_box([-1.5,0.,0.6],[-2.,0.,0.6],0.1,0.2, 0.05)
display_box(gui,a,b,y,z)
gui.refresh()
