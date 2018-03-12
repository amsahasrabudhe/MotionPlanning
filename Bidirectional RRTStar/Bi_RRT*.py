import pygame
import random
from math import sin, cos, atan2, sqrt
import time, sys
from Node import Node
from pygame.locals import *

image = 0									# Image object
screen = 0									# Canvas to draw RRT
env = []									# Environment (image) to run RRT/RRT*/Bi-directional RRT*
size = (0,0)								# Size of the environment

# Parameters that can be tweaked
K = 10										# Number of samples to generate at each iteration
stepSize = 10								# Step size for each node connected to the tree
nearestNodesRadius = 16

start = (150, 150)
goal = (410, 360)

extendTree = []
connectTree = []

red = (255, 0, 0)
green = (0, 255, 0)
black = (0, 0, 0)

count = 0

def readEnvironmentFromImage(filename):

	img = pygame.image.load(filename)
	return img, pygame.surfarray.array2d(img)

def initializePygameDisplay(dispSize, image):

	global size

	pygame.init()
	temp_screen = pygame.display.set_mode(dispSize)
	size = dispSize
	temp_screen.fill([255,255,255])
	temp_screen.blit(image, (0,0))
	pygame.display.set_caption('RRT* Assignment - Ashwin Sahasrabudhe')
	pygame.display.update()

	return temp_screen

def getDistance(node1, node2):
	return sqrt((node1[0]-node2[0])**2 + (node1[1]-node2[1])**2)

def pathToGoal(node):

	extendNodeIndex = findNodeInTree(node, extendTree)

	extendNode = extendTree[extendNodeIndex]
	extendCost = extendNode.cumulativeCost
	connectCost = node.cumulativeCost

	totalCost = extendCost+connectCost

	path = []
	currNode = node
	while currNode.parent!=None:
		path.append(currNode.XY)
		parentXY = currNode.parent.XY
		pygame.draw.line(screen, green, currNode.XY, parentXY, 3)
		currNode = currNode.parent
		pygame.display.update()

	currNode = extendNode
	while currNode.parent!=None:
		path.append(currNode.XY)
		parentXY = currNode.parent.XY
		pygame.draw.line(screen, green, currNode.XY, parentXY, 3)
		currNode = currNode.parent
		pygame.display.update()	

	print "Cost of found path:",totalCost
	print "Path length (samples):",len(path)
	print "Total samples generated:",len(extendTree)+len(connectTree)


def getNearNodes(newXY, tree, radius):

	nearNodes = []
	for node in tree:
		d = getDistance(newXY, node.XY)

		if d<radius:
			nearNodes.append(node)

	return nearNodes


def getBestParent(newXY, nearNodes):

	lowestCost = 99999999
	parent = None
	for node in nearNodes:
		d = node.cumulativeCost + getDistance(newXY, node.XY)
		if d<lowestCost:
			lowestCost = d
			parent = node

	dist = getDistance(newXY, parent.XY)

	return parent, dist


def findNodeInTree(node, tree):

	for i in xrange(len(tree)):
		if node.XY == tree[i].XY:
			return i

	return None

def rewire(newNode, nearNodes, tree):

	for node in nearNodes:

		if node.XY != newNode.parent.XY:
			nearCost = getDistance(newNode.XY, node.XY)
			cost = newNode.cumulativeCost + nearCost
			if cost < node.cumulativeCost:
				num = findNodeInTree(node, tree)
				
				pygame.draw.line(screen, black, node.parent.XY, node.XY)

				tree[num].parent = newNode
				tree[num].setStepCost(nearCost)
				tree[num].recomputeCost()

				pygame.draw.line(screen, red, newNode.XY, node.XY)
				pygame.display.update()


def extendRRT(env, screen):

	for e in pygame.event.get():
		if e.type ==QUIT or (e.type ==KEYUP and e.type == K_ESCAPE):
			sys.exit("Leaving")

	randX = (int)(random.uniform(0,499))
	randY = (int)(random.uniform(0,499))

	randXY = (randX, randY)
	if env[randXY[0]][randXY[1]]==0:
		minDist = 99999999
		nearestNodeXY = None

		for node in extendTree:

			d = getDistance(node.XY, randXY)
			
			if d<minDist:
				minDist = d
				nearestNodeIndex = extendTree.index(node)
				nearestNodeXY = node.XY

		# Create new node in the direction of the random node based on stepSize

		newXY = None
		
		if minDist>stepSize:

			theta = atan2(randXY[1]-nearestNodeXY[1], randXY[0]-nearestNodeXY[0])					
			x = (int) (nearestNodeXY[0] + stepSize*cos(theta))
			y = (int) (nearestNodeXY[1] + stepSize*sin(theta))

			newXY = (x,y)
			minDist = stepSize

		else:
			newXY = randXY


		if env[newXY[0]][newXY[1]]==0:

			nearNodes = getNearNodes(newXY, extendTree, nearestNodesRadius)
			parent, dist = getBestParent(newXY, nearNodes)

			newNode = Node(newXY, dist, parent)

			extendTree.append(newNode)
			pygame.draw.line(screen, red, newXY, parent.XY)
			
			rewire(newNode, nearNodes, extendTree)
			pygame.display.update()

			return newNode

		else:
			return None

	else:
		return None


def connectRRT(env, screen, extendNode):

	for e in pygame.event.get():
		if e.type ==QUIT or (e.type ==KEYUP and e.type == K_ESCAPE):
			sys.exit("Leaving")

	extendNodeXY = extendNode.XY

	minDist = 99999999
	nearestConnectNode = None

	for node in connectTree:

		d = getDistance(node.XY, extendNodeXY)
		
		if d<minDist:
			minDist = d
			nearestConnectNode = node
			nearestConnectNodeXY = node.XY

	# Create new node in the direction of the random node based on stepSize

	newXY = None
	dist = minDist
	
	while True:

		#print "stuck connectTree"
	
		if dist>stepSize:

			theta = atan2(extendNodeXY[1]-nearestConnectNodeXY[1], extendNodeXY[0]-nearestConnectNodeXY[0])					
			x = (int) (nearestConnectNodeXY[0] + stepSize*cos(theta))
			y = (int) (nearestConnectNodeXY[1] + stepSize*sin(theta))

			newXY = (x,y)
			dist = stepSize

		else:
			newXY = extendNodeXY


		if env[newXY[0]][newXY[1]]==0:

			nearNodes = getNearNodes(newXY, connectTree, nearestNodesRadius)
			parent, dist = getBestParent(newXY, nearNodes)

			newNode = Node(newXY, dist, parent)

			connectTree.append(newNode)
			pygame.draw.line(screen, red, newXY, parent.XY)
			
			rewire(newNode, nearNodes, connectTree)
			pygame.display.update()
			
			if newXY == extendNodeXY:
				return newNode

		else:
			return None

		dist = (int) (getDistance(newXY, extendNodeXY))
		nearestConnectNodeXY = newXY


def generateTrees(env, screen):

	connectionNode = None
	while connectionNode==None:
		
		extendNode = None
		while extendNode==None:
			extendNode = extendRRT(env, screen)

		connectionNode = connectRRT(env, screen, extendNode)

	pathToGoal(connectionNode)


def main():

	global screen

	random.seed(time.time())

	image, env = readEnvironmentFromImage('2.png')
	screen = initializePygameDisplay(env.shape, image)

	startNode = Node(start, 0, None)
	connectTree.append(startNode)

	goalNode = Node(goal, 0, None)
	extendTree.append(goalNode)	

	pygame.draw.line(screen, green, goal, goal, 2)

	startTime = time.time()
	generateTrees(env, screen)
	endTime = time.time()

	print "Time taken for finding path :",(endTime-startTime)

	time.sleep(5)

main()