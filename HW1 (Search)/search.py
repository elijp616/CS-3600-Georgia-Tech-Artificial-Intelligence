# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the node state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    visited_stack = list()
    open_stack = util.Stack()

    node = {}
    node['state'] = problem.getStartState()
    node['direction'] = list()
    node ['cost'] = 0

    open_stack.push(node)
    while (open_stack.isEmpty() != True):
        node = open_stack.pop()
        if node['state'] in visited_stack:
            pass
        else:
            visited_stack.append(node['state'])
            if problem.isGoalState(node['state']):
                return node['direction']
            for state, direction, cost in problem.getSuccessors(node['state']):
                if state in visited_stack:
                    pass
                else:
                    aggregate = {}
                    aggregate['state'] = state
                    aggregate['direction'] = node['direction'] + [direction]
                    aggregate['cost'] = node['cost'] + cost
                    open_stack.push(aggregate)

    return node['direction']
    util.raiseNotDefined()



def breadthFirstSearch(problem):
    open_queue = util.Queue()
    state = problem.getStartState()
    state_tuple = (state, [])
    open_queue.push(state_tuple)
    node_list = []
    while open_queue.isEmpty() != True:
        node = open_queue.pop()
        if node[0] not in node_list:
            if problem.isGoalState(node[0]):
                return node[1]
            node_list.append(node[0])
            successors = problem.getSuccessors(node[0])
            for i in successors:
                open_queue.push((i[0], node[1] + [(i[1])]))

    util.raiseNotDefined()


def uniformCostSearch(problem):
    """
    Search the node of least total cost first.
    """
    closed = []
    parent_node = {}
    path = []
    node = (problem.getStartState(), "", 0)
    current = node
    open = util.PriorityQueue()
    open.push(current, 0)
    cost = {node: 0}

    # while not problem.isGoalState(current[0]):
    #     closed.append(current[0])
    #     successors = problem.getSuccessors(current[0])
    #     for successor in successors:
    #         open.push(successor, cost[current] + successor[2])
    #         cost[successor] = cost[current] + successor[2]
    #         if successor not in parent

    while not problem.isGoalState(current[0]):
        if problem.isGoalState(current[0]):
            break
        closed.append(current[0])
        successors = problem.getSuccessors(current[0])
        for i in successors:
            open.push(i, cost[current] + i[2])
            cost[i] = cost[current] + i[2]
            if i not in parent_node.keys():
                parent_node[i] = current
        while current[0] in closed:
            current = open.pop()
    parent = parent_node[current]
    path.append(current[1])

    while parent != node:
        if parent == node:
            break
        path.append(parent[1])
        parent = parent_node[parent]

    path.reverse()
    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the node state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    routes = {}
    costs = {}
    visited = {}
    search = util.PriorityQueue()

    start = problem.getStartState()
    search_tuple = (start, start, None)
    search.push(search_tuple, 0)
    routes[start] = []
    costs[start] = 0

    while search:
        node = search.pop()
        if (node[0] != start) & (node[0] not in routes):
            routes[node[0]] = routes[node[1]] + [node[2]]

        if problem.isGoalState(node[0]) == True:
            return routes[node[0]]

        if node[0] not in visited:
            visited[node[0]] = 1
            for successor in problem.getSuccessors(node[0]):
                if successor[0] not in visited:
                    costs[successor[0]] = costs[node[0]] + successor[2]
                    hCost = heuristic(successor[0], problem)
                    search.push((successor[0], node[0], successor[1]), costs[successor[0]] + hCost)

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
