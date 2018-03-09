class Node:

	def __init__(self, xy, cost, parent):

		self.parent = parent

		self.XY = xy
		self.stepCost = cost

		if self.parent == None:
			self.cumulativeCost = 0
		else:
			self.cumulativeCost = self.parent.cumulativeCost + self.stepCost