import obstacle 
import math
import numpy as np
from heapq import heappush, heappop



class AStarAlgorithm():
	def __init__(self, source_location, goal_location, theta_step = 30, step_size = 1, goal_thres = 0.1,
		width = 10, height = 10, thresh = 0.5, r = 0.1, c = 0.1, wheel_length = 0.038, 
		Ur=2,Ul=2, wheel_radius=2, dt=0.1, dtheta=0, weight=1, isExploration=0, isPathDisplay=1):
		self.source_location = source_location
		self.goal_location = goal_location
		# node = [ x , y , a , cost ]
		self.nodeData = []
		self.weight = weight
		# [ cost , selfID , parentID ]
		# selfID and parentID are index in nodeData
		# [ cost , x , y , a ]
		self.Data = []
		self.allData = []
		self.theta_step = theta_step
		self.dt=dt
		self.dtheta=dtheta
		self.wheel_radius=wheel_radius
		self.wheel_length=wheel_length
		self.Ur=Ur
		self.Ul=Ul
		self.isExploration = isExploration
		self.isPathDisplay = isPathDisplay
		self.step_size = step_size
		self.goal_thres = goal_thres
		self.path = []
		self.trace_index = []
		self.goal_reached = False
		self.actions = [[	   0 , self.Ur],
						[self.Ul , 		 0],
						[	   0 , self.Ul],
						[self.Ur , 		 0],
						[self.Ul , self.Ur],
						[self.Ur , self.Ul],
						[self.Ur , self.Ur],
						[self.Ul , self.Ul]]
		self.actionSet = []
		self.obstacle = obstacle.Obstacle(width, height, r = r, c = c, thresh=thresh, 
			actions=self.actions, wheel_length = self.wheel_length, 
			wheel_radius = self.wheel_radius)

	def actionSpaceData(self, present_node):
		index = 0
		self.actionSet = []
		
		for action_vector in self.actions:
			t = 0
			dt = 0.1
			x, y, a = present_node[1], present_node[2], present_node[3]
			# a = angle
			a = 3.14*a/180.0 
			costToCome = 0
			for i in range(10):
				t = t+dt
				xnew = 0.5*(self.wheel_radius)*(action_vector[0]+action_vector[1])*math.cos(a)*dt
				ynew = 0.5*(self.wheel_radius)*(action_vector[0]+action_vector[1])*math.sin(a)*dt
				x += xnew       
				y += ynew      
				a += (self.wheel_radius/self.wheel_length)*(action_vector[1]-action_vector[0])*dt 
				costToCome += math.sqrt(xnew**2 + ynew**2)
			a = 180 * (a) / 3.14
			self.actionSet.append([x, y, a, costToCome, index])
			index += 1
		return

	def isValid(self):
	# condition for goal and source location with respect to obstacles
		if not self.obstacle.isObstacle(self.goal_location[0], self.goal_location[1]):
			print("given goal location is inside obstacle field")
			return False
		elif not self.obstacle.isObstacle(self.source_location[0], self.source_location[1]):
			print("given source location is inside obstacle field")
			return False
		else:
			# pushing initial data (self.Data, [0,0,0])
			cost = math.sqrt((self.source_location[0] - self.goal_location[0])**2 + (self.source_location[1] - self.goal_location[1])**2)
			heappush(self.Data, [cost, self.source_location[0], self.source_location[1], self.source_location[2], 0])
			# storing into allData list 
			self.nodeData.append([self.source_location[0], self.source_location[1], self.source_location[2], 0])
			return True

	#calculating euclidian distance between current node and goal location
	def heuristic_function(self, current): 
		h = self.weight * math.sqrt((current[1] - self.goal_location[0])**2 + (current[2] - self.goal_location[1])**2)
		return h

	def goalReached(self, current):  
		x, y = current[1], current[2]
		if (x - self.goal_location[0])**2 + (y - self.goal_location[1])**2 <= (self.goal_thres)**2:
			return True
		else:
			return False

	def backTracePath(self, present_node):
		track = []
		trace_index = []
		current_node = present_node[:4]
		track.append(current_node)
		trace_index.append(0)
		while current_node[1:] != self.source_location:
			l, ind = self.obstacle.getExploredList(current_node)
			current_node = list(l)
			track.append(current_node)
			trace_index.append(ind)
		print(":::::Back Trace  Path::::::")
		track.reverse()
		trace_index.reverse()
		return track, trace_index

	def computeOptimalPath(self):
		count = 0
		if self.isValid():
			while len(self.Data)>0:
				count+=1
				present_node = heappop(self.Data)
				previous_cost, previous_cost_to_come = present_node[0], present_node[4]
				if self.goalReached(present_node):
					self.goal_reached = True
					print("::::::Robot reached destination location::::::")
					self.path, self.trace_index = self.backTracePath(present_node)
					if self.isExploration:
						self.obstacle.explorationPlot()
					# plot back trace path if the flag is 1
					if self.isPathDisplay:
						self.obstacle.plotPath(self.path, self.trace_index)
					return
				self.actionSpaceData(present_node)
				for action_vector in self.actionSet:
					# node = [ x , y , a , cost]
					# Data = [ cost , selfID , parentID ]
					new_X = action_vector[0]
					new_Y = action_vector[1]
					new_A = action_vector[2]
					new_node = [0, new_X, new_Y, new_A, 0]
					new_cost_to_come = previous_cost_to_come + action_vector[3]
					new_node[4] = new_cost_to_come
					cost_to_go = self.heuristic_function(new_node)
					if self.obstacle.isObstacle(new_X, new_Y):
						if not self.obstacle.isExplored(new_node):
							# unvisted nodes are added to Data list
							present_node[0] = new_cost_to_come
							self.obstacle.exploredList(new_node, present_node[:4], action_vector[4])
							new_node[0] = new_cost_to_come + cost_to_go
							heappush(self.Data, new_node)

						# checking previous cost for visited nodes
						else: 
							previous_visited,_ = self.obstacle.getExploredList(new_node)
							previous_cost = previous_visited[0]
							if previous_cost > new_cost_to_come:
								present_node[0] = new_cost_to_come
								self.obstacle.exploredList(new_node, present_node[:4], action_vector[4])
		print("Error reaching the destination location") 
		return
