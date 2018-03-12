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

	path = []
	currNode = node
	while currNode.parent!=None:
		path.append(currNode.XY)
		parentXY = currNode.parent.XY
		pygame.draw.line(screen, green, currNode.XY, parentXY, 3)
		currNode = currNode.parent
		pygame.display.update()


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

			goalDist = getDistance(newXY, goal)
			if goalDist<stepSize:
				
				goalNode = Node(goal, goalDist, extendTree[-1])
				extendTree.append(goalNode)
				pygame.draw.line(screen, red, newXY, goal)
				pygame.display.update()

				newNode = goalNode

				print "Found path of cost :",goalNode.cumulativeCost
				print "Total samples generated :",count

			return newNode

		else:
			return None

	else:
		return None


def generateTrees(env, screen):

	while True:
		
		extendNode = None
		while extendNode==None:
			extendNode = extendTree(env, screen)

		connectRRT(env, screen)


def main():

	global screen

	random.seed(time.time())

	image, env = readEnvironmentFromImage('1.png')
	screen = initializePygameDisplay(env.shape, image)

	startNode = Node(start, 0, None)
	extendTree.append(startNode)

	pygame.draw.line(screen, green, goal, goal, 2)

	#generateTrees(env, screen)

	while True:
		node = extendRRT(env, screen)
		if node!=None:
			if node.XY == goal:
				pathToGoal(node)
				pygame.display.update()
				break

	time.sleep(5)

main()