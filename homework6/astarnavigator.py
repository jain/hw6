import sys, pygame, math, numpy, random, time, copy
from pygame.locals import * 

from constants import *
from utils import *
from core import *
from mycreatepathnetwork import *
from mynavigatorhelpers import *


###############################
### AStarNavigator
###
### Creates a path node network and implements the FloydWarshall all-pairs shortest-path algorithm to create a path to the given destination.
			
class AStarNavigator(NavMeshNavigator):

	def __init__(self):
		NavMeshNavigator.__init__(self)
		

	### Create the pathnode network and pre-compute all shortest paths along the network.
	### self: the navigator object
	### world: the world object
	def createPathNetwork(self, world):
		self.pathnodes, self.pathnetwork, self.navmesh = myCreatePathNetwork(world, self.agent)
		return None
		
	### Finds the shortest path from the source to the destination using A*.
	### self: the navigator object
	### source: the place the agent is starting from (i.e., it's current location)
	### dest: the place the agent is told to go to
	def computePath(self, source, dest):
		### Make sure the next and dist matricies exist
		if self.agent != None and self.world != None: 
			self.source = source
			self.destination = dest
			### Step 1: If the agent has a clear path from the source to dest, then go straight there.
			###   Determine if there are no obstacles between source and destination (hint: cast rays against world.getLines(), check for clearance).
			###   Tell the agent to move to dest
			### Step 2: If there is an obstacle, create the path that will move around the obstacles.
			###   Find the pathnodes closest to source and destination.
			###   Create the path by traversing the self.next matrix until the pathnode closes to the destination is reached
			###   Store the path by calling self.setPath()
			###   Tell the agent to move to the first node in the path (and pop the first node off the path)
			if clearShot(source, dest, self.world.getLines(), self.world.getPoints(), self.agent):
				self.agent.moveToTarget(dest)
			else:
				start = findClosestUnobstructed(source, self.pathnodes, self.world.getLinesWithoutBorders())
				end = findClosestUnobstructed(dest, self.pathnodes, self.world.getLinesWithoutBorders())
				if start != None and end != None:
					#print len(self.pathnetwork)
					newnetwork = unobstructedNetwork(self.pathnetwork, self.world.getGates())
					#print len(newnetwork)
					#print "ok"
					closedlist = []
					path, closedlist = astar(start, end, newnetwork)
					if path is not None and len(path) > 0:
						path = shortcutPath(source, dest, path, self.world, self.agent)
						self.setPath(path)
						if self.path is not None and len(self.path) > 0:
							first = self.path.pop(0)
							self.agent.moveToTarget(first)
		return None
		
	### Called when the agent gets to a node in the path.
	### self: the navigator object
	def checkpoint(self):
		myCheckpoint(self)
		return None

	### This function gets called by the agent to figure out if some shortcutes can be taken when traversing the path.
	### This function should update the path and return True if the path was updated.
	def smooth(self):
		return mySmooth(self)

	def update(self, delta):
		myUpdate(self, delta)


def unobstructedNetwork(network, worldLines):
	newnetwork = []
	for l in network:
		hit = rayTraceWorld(l[0], l[1], worldLines)
		if hit == None:
			newnetwork.append(l)
	return newnetwork


def insert(x, list, func = lambda x: x):
	for i in xrange(len(list)):
		if func(x) < func(list[i]):
			list.insert(i, x)
			return list
	list.append(x)
	return list
	

def astar(init, goal, network):
	path = []
	open = []
	closed = []
	### YOUR CODE GOES BELOW HERE ###
	init =(init ,0 ,distance (init ,goal ),None )#line:3
	OO000O0000O0000O0 =set ()#line:4
	OO0O000O00O0O000O =set ()#line:5
	O0000O0O00O00O0O0 =[init ]#line:6
	O000OOOO0OOOOOO00 =init #line:7
	while O000OOOO0OOOOOO00 is not None and O000OOOO0OOOOOO00 [0 ]!=goal and len (O0000O0O00O00O0O0 )>0 :#line:10
		OO000O0000O0000O0 .add (O000OOOO0OOOOOO00 [0 ])#line:11
		OO0O000O00O0O000O .add (O000OOOO0OOOOOO00 )#line:12
		O0000O0O00O00O0O0 .pop (0 )#line:13
		OOOOOOO0000000OO0 =foow (O000OOOO0OOOOOO00 ,network ,goal )#line:15
		for O0OOO0OO00OOOO0O0 in OOOOOOO0000000OO0 :#line:17
			if O0OOO0OO00OOOO0O0 [0 ]not in OO000O0000O0000O0 :#line:18
				insert (O0OOO0OO00OOOO0O0 ,O0000O0O00O00O0O0 ,lambda x :x [1 ]+x [2 ])#line:19
		if len (O0000O0O00O00O0O0 )>0 :#line:21
			O000OOOO0OOOOOO00 =O0000O0O00O00O0O0 [0 ]#line:22
		else :#line:23
			O000OOOO0OOOOOO00 =None #line:24
	if O000OOOO0OOOOOO00 is not None :#line:27
		while O000OOOO0OOOOOO00 [3 ]is not None :#line:28
			path .append (O000OOOO0OOOOOO00 [0 ])#line:29
			OO0O0000O00OO00OO =O000OOOO0OOOOOO00 [3 ]#line:30
			for OOOO00O00OOOOO0O0 in list (OO0O000O00O0O000O ):#line:31
				if OO0O0000O00OO00OO ==OOOO00O00OOOOO0O0 [0 ]:#line:32
					O000OOOO0OOOOOO00 =OOOO00O00OOOOO0O0 #line:33
					break #line:34
		path .append (O000OOOO0OOOOOO00 [0 ])#line:35
		path .reverse ()#line:36
	OO000O0000O0000O0 =list (OO000O0000O0000O0 )	
	### YOUR CODE GOES ABOVE HERE ###
	return path, closed
	
	
def foow (node ,network ,goal ):#line:1
	O00O00O00OO00O0OO =[]#line:2
	for O0000OO0OO0OOOO0O in network :#line:3
		if O0000OO0OO0OOOO0O [0 ]==node [0 ]:#line:4
			O00O00O00OO00O0OO .append ((O0000OO0OO0OOOO0O [1 ],node [1 ]+distance (O0000OO0OO0OOOO0O [0 ],O0000OO0OO0OOOO0O [1 ]),distance (O0000OO0OO0OOOO0O [1 ],goal ),node [0 ]))#line:5
		elif O0000OO0OO0OOOO0O [1 ]==node [0 ]:#line:6
			O00O00O00OO00O0OO .append ((O0000OO0OO0OOOO0O [0 ],node [1 ]+distance (O0000OO0OO0OOOO0O [0 ],O0000OO0OO0OOOO0O [1 ]),distance (O0000OO0OO0OOOO0O [0 ],goal ),node [0 ]))#line:7
	return O00O00O00OO00O0OO 
	

def myUpdate(nav, delta):
	### YOUR CODE GOES BELOW HERE ###
	if nav .getPath ()is not None :#line:3
		OOOOO0OOOOO0O00OO =nav .world .getGates ()#line:4
		O0OO0000O0OO0O0O0 =nav .agent .getLocation ()#line:14
		for O00OO000OO00O0O0O in nav .getPath ()+[nav .getDestination ()]:#line:15
			if O0OO0000O0OO0O0O0 is not None :#line:16
				OO0O0O00O0OOOOOOO =rayTraceWorld (O0OO0000O0OO0O0O0 ,O00OO000OO00O0O0O ,OOOOO0OOOOO0O00OO )#line:17
				if OO0O0O00O0OOOOOOO is not None :#line:18
					nav .setPath (None )#line:20
					nav .agent .stopMoving ()#line:21
					return None #line:22
			O0OO0000O0OO0O0O0 =O00OO000OO00O0O0O #line:23		
	### YOUR CODE GOES ABOVE HERE ###
	return None

def myCheckpoint(nav):
	### YOUR CODE GOES BELOW HERE ###

	### YOUR CODE GOES ABOVE HERE ###
	return None

### Returns true if the agent can get from p1 to p2 directly without running into an obstacle.
### p1: the current location of the agent
### p2: the destination of the agent
### worldLines: all the lines in the world
### agent: the Agent object
def clearShot(p1, p2, worldLines, worldPoints, agent):
	### YOUR CODE GOES BELOW HERE ###
	O0OOOO0OO0OOOO0O0 =agent .getRadius ()*4.0 #line:3
	O0O00O000OO0OO000 =rayTraceWorld (p1 ,p2 ,worldLines )#line:4
	if O0O00O000OO0OO000 ==None :#line:5
		O00O0OO0O00OOO000 =False #line:6
		for O0OOOOO0O000O0OO0 in worldPoints :#line:7
			if minimumDistance ((p1 ,p2 ),O0OOOOO0O000O0OO0 )<O0OOOO0OO0OOOO0O0 :#line:8
				O00O0OO0O00OOO000 =True #line:9
		if not O00O0OO0O00OOO000 :#line:10
			return True #line:11
	### YOUR CODE GOES ABOVE HERE ###
	return False


#===============================================================#
# Obfuscated by Oxyry Python Obfuscator (http://pyob.oxyry.com) #
#===============================================================#
