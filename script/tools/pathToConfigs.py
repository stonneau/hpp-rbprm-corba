## Discretizes a path into a list of configurations
#
# \param robot the considered robot,
# \param problem the problem associated with the path computed for the robot
# \param stepsize increment along the path
# \param pathId if of the considered path
def pathToConfigs(viewer, problem, pathId, dt):
	length = problem.pathLength (pathId)
	t = 0
	tau = []
	#~ dt = stepsize / length
	while t < length :
		q = problem.configAtParam (pathId, t)
		tau.append(q)
		t += dt
	return tau
