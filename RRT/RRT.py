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

start = (150, 150)
goal = (410, 360)

red = (255, 0, 0)
green = (0, 255, 0)

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


def generateRRT(env, screen, root, mode):

	global count

	tree = []
	tree.append(root)

	while True:

		for nodeNum in xrange(K):

			randX = (int)(random.uniform(0,499))
			randY = (int)(random.uniform(0,499))

			randXY = (randX, randY)
			if env[randXY[0]][randXY[1]]==0:
				minDist = 99999999
				nearestNodeXY = None

				for node in tree:

					d = getDistance(node.XY, randXY)
					
					if d<minDist:
						minDist = d
						nearestNodeIndex = tree.index(node)
						nearestNodeXY = node.XY

				# Create new node in the direction of the random node based on stepSize

				breakFlag = None

				dist = minDist
				newXY = None
				while True:
				
					if dist>stepSize:

						theta = atan2(randXY[1]-nearestNodeXY[1], randXY[0]-nearestNodeXY[0])					
						x = (int) (nearestNodeXY[0] + stepSize*cos(theta))
						y = (int) (nearestNodeXY[1] + stepSize*sin(theta))

						newXY = (x,y)
						dist = stepSize

					else:
						newXY = randXY
						breakFlag = "reached"

					if env[newXY[0]][newXY[1]]==0:
						newNode = Node(newXY, dist, tree[nearestNodeIndex])
						tree.append(newNode)
						pygame.draw.line(screen, red, newXY, nearestNodeXY)
						count += 1

						goalDist = getDistance(newXY, goal)
						if goalDist<stepSize:
							
							goalNode = Node(goal, goalDist, tree[-1])
							tree.append(goalNode)
							pygame.draw.line(screen, red, newXY, goal)
							pygame.display.update()
							count += 1

							print "Found path of cost :",goalNode.cumulativeCost
							print "Total samples generated :",count
							return pathToGoal(goalNode)

					else:
						nodeNum -= 1
						breakFlag = "obstacle"

					if mode=="extend":
						break
					elif mode=="connect":

						nearestNodeIndex = -1
						nearestNodeXY = newXY
						if breakFlag=="reached" or breakFlag=="obstacle":
							break

					dist = getDistance(randXY, nearestNodeXY)
			else:
				nodeNum -= 1

			for e in pygame.event.get():
				if e.type ==QUIT or (e.type ==KEYUP and e.type == K_ESCAPE):
					sys.exit("Leaving")

		pygame.display.update()


def main():

	global screen

	mode = ""

	if len(sys.argv)==1:
		print "Please specify the mode (extend/connect) as last argument!!"
		return

	if (sys.argv[1]!="extend" and sys.argv[1]!="connect"):
		print "\nInput RRT mode correctly!\nEither 'extend' or 'connect'\n"
		return

	random.seed(time.time())

	image, env = readEnvironmentFromImage('narrower.png')
	screen = initializePygameDisplay(env.shape, image)

	root = Node(start, 0, None)

	pygame.draw.line(screen, green, goal, goal, 2)
	generateRRT(env, screen, root, sys.argv[1])

	time.sleep(5)


main()