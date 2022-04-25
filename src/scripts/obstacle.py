
import math
import numpy as np
from heapq import heappush, heappop
import time
import matplotlib.pyplot as plt


class Obstacle():
	def __init__(self, width = 10, height = 10, r = 1, c = 1, thresh=0.01, 
			theta_step = 30, actions=None, wheel_length = 1, 
			wheel_radius = 1):
		self.thresh = thresh
		self.W = int(width/thresh) +1
		self.H = int(height/thresh) +1
		self.r = r
		self.c = c
		self.theta_step = theta_step
		self.explored = np.zeros([self.H, self.W, 4])
		self.actionIndexMatrix = np.zeros([self.H, self.W])
		### [ startX , startY , endX , endY ]
		self.plotData_X = []
		self.plotData_Y = []
		self.plotData_A = []
		self.plotData_U = []
		self.plotData_V = []
		self.whcihAction = []
		plt.ion()
		self.fig, self.ax = plt.subplots()
		plt.axis('square')
		self.plotSpace()
		self.actions = actions
		self.wheel_radius = wheel_radius
		self.wheel_length = wheel_length

	
	def plotSpace(self):

		centX, centY, radii = 2,2,1
		circle_1_X = [centX+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
		circle_1_Y = [centY+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
		centX, centY, radii = 2,8,1
		circle_2_X = [centX+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
		circle_2_Y = [centY+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]

		square_1_x, square_1_y = [0.25, 1.75, 1.75, 0.25, 0.25],[ 4.25,  4.25,  5.75,  5.75, 4.25]
		rectangle_2_x, rectangle_2_y = [3.75, 3.75, 6.25, 6.25, 3.75],[ 5.75,  4.25, 4.25, 5.75,  5.75]
		rectangle_3_x, rectangle_3_y = [7.25, 7.25, 8.75, 8.75, 7.25],[ 4, 2, 2, 4, 4]
		self.ax.plot(circle_1_X, circle_1_Y)
		self.ax.plot(circle_2_X, circle_2_Y)
		self.ax.plot(square_1_x, square_1_y)
		self.ax.plot(rectangle_2_x, rectangle_2_y)
		self.ax.plot(rectangle_3_x, rectangle_3_y)
		self.ax.set_xlim(0, 10)
		self.ax.set_ylim(0, 10)
		pass


	def checkObstcaleSpace(self):
		xx = np.arange(0,10,0.05)
		yy = np.arange(0,10,0.05)
		x_ = []
		y_ = []
		for x in xx:
			for y in yy:
				if self.isObstacle(x,y):
					x_.append(x)
					y_.append(y)
		plt.scatter(x_, y_, s=0.1)
		plt.show()
		pass

	def isObstacle(self, i, j):

		if self.checkBoundary(i,j):
			return False
		elif self.isCircle(i, j, (2,2), 1):
			return False
		elif self.isCircle(i, j, (2,8), 1):
			return False  
		elif self.isSquare(i, j, 0.25, 1.75, 5.75, 4.25):
			return False
		elif self.isSquare(i, j, 3.75, 6.25, 5.75, 2.75):
			return False
		elif self.isSquare(i, j, 7.25, 8.75, 4, 2):
			return False
		else:
			return True

	def isSquare(self, i, j, left, right, top, bottom):
		l_ = left - self.r - self.c
		r_ = right + self.r + self.c
		t_ = top + self.r + self.c
		b_ = bottom - self.r - self.c
		if (i<r_ and i>l_ and j<t_ and j>b_):
			return True
		else:
			return False

	def isCircle(self, i, j, center, radius):
		center_x, center_y = center[0], center[1]
		if ((i - center_x) ** 2 + (j - center_y) ** 2) <= (radius + self.r + self.c) ** 2:
			return True
		else:
			return False

	def checkInQuad(self, i, j, vertex1, vertex2, vertex3, vertex4):
		x1, y1 = vertex1[0], vertex1[1]
		x2, y2 = vertex2[0], vertex2[1]
		x3, y3 = vertex3[0], vertex3[1]
		x4, y4 = vertex4[0], vertex4[1]
		if (i>x1 and i<=x2) and (j>=y1 and j<=y2):
			return True
		return False

	def checkBoundary(self,i,j):
		if (i < (10 - self.r - self.c) and  
			i > (0 + self.r + self.c) and 
			j < (10 - self.r - self.c) and  
			j > (0 +self.r + self.c)):
			return False
		return True

	def getIdxs(self, node):
		x,y,a = node[1], node[2], node[3]
		shiftx, shifty = 0,0
		x += shiftx
		y = abs(shifty + y)
		# i , j , k ----- height , width , angle
		i = int(round(y/self.thresh))
		j = int(round(x/self.thresh))
		k = int(round(a/self.theta_step))
		return i,j,k

	def isExplored(self, node):
		# node = [ cost , x , y , angle ]
		i,j,k = self.getIdxs(node)
		if self.explored[i, j, 3] != 0:
			return True # True if it is visited
		else:
			return False # Flase if Not visited

	def discret(self,node):
		i,j,k = self.getIdxs(node)
		return [node[0], i, j, k]

	def getExploredList(self, node):
		i,j,k = self.getIdxs(node)
		return self.explored[i, j, :], self.actionIndexMatrix[i,j]

	def exploredList(self, node, parentNode, actionIndex):
		i,j,k = self.getIdxs(node)
		self.plotData_X.append(parentNode[1])
		self.plotData_Y.append(parentNode[2])
		self.plotData_A.append(parentNode[3])
		self.whcihAction.append(actionIndex)
		self.explored[i, j, :] = np.array(parentNode)
		self.actionIndexMatrix[i,j] = actionIndex
		return

	def plotPath(self, path, trace_index):
		print(len(trace_index), len(path))
		for i in range(len(path)):
			Xi = path[i][1]
			Yi = path[i][2]
			Thetai = path[i][3]
			actionIndex = int(trace_index[i])
			UL, UR = self.actions[actionIndex][0], self.actions[actionIndex][1]
			self.plotCurve(Xi, Yi, Thetai, UL, UR, color="red", lw=1.2)
			plt.pause(0.2)
		plt.ioff()
		plt.show(block=False)
		pass

	def explorationPlot(self):
		for i in range(len(self.plotData_X)):
			Xi = self.plotData_X[i]
			Yi = self.plotData_Y[i]
			Thetai = self.plotData_A[i]
			actionIndex = self.whcihAction[i]
			UL, UR = self.actions[actionIndex][0], self.actions[actionIndex][1]
			self.plotCurve(Xi, Yi, Thetai, UL, UR)
			if i%100==0:
				plt.pause(0.000001)
		pass

	def plotCurve(self, Xi, Yi, Thetai, UL, UR,color="blue",lw=0.5):
		r = self.wheel_radius
		L = self.wheel_length
		t = 0
		dt = 0.1
		Xn = Xi
		Yn = Yi
		Thetan = 3.14 * Thetai / 180
		x_s, x_n, y_s, y_n = [],[],[],[] 
		while t<1:
			t = t + dt
			Xs = Xn
			Ys = Yn
			Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
			Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
			Thetan += (r / L) * (UR - UL) * dt
			x_s.append(Xs)
			x_n.append(Xn)
			y_s.append(Ys)
			y_n.append(Yn)
		self.ax.plot([x_s, x_n], [y_s, y_n], color=color, linewidth=lw)
