# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    frontier = util.Stack()
    frontier.push([problem.getStartState()])
    actionItems = util.Stack()
    actionItems.push([])

    while not frontier.isEmpty():
        currentList = frontier.pop()
        currentState = currentList[-1]

        poppedAction = actionItems.pop()

        if (problem.isGoalState(currentState)):
            return poppedAction
        else:
            successors = problem.getSuccessors(currentState)
            for successor, action, _ in successors:
                newList = currentList.copy()
                newList.append(successor)
                newActionList = poppedAction.copy()
                newActionList.append(action)
                if successor in currentList:
                    continue
                else:
                    actionItems.push(newActionList)
                    frontier.push(newList)

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    frontier = util.Queue()
    frontier.push([problem.getStartState()])
    actionQueue = util.Queue()
    actionQueue.push([])
    CycleCheck = []
    CycleCheck.append(problem.getStartState())

    while(not frontier.isEmpty()):
        currentList = frontier.pop()
        currentState = currentList[-1]

        poppedAction = actionQueue.pop()

        if problem.isGoalState(currentState):
            return poppedAction
        else:
            successors = problem.getSuccessors(currentState)
            for successor, action, _ in successors:
                newList = currentList.copy()
                newList.append(successor)
                newActionList = poppedAction.copy()
                newActionList.append(action)
                if successor in CycleCheck:
                    continue
                else:
                    frontier.push(newList)
                    actionQueue.push(newActionList)
                    CycleCheck.append(successor)

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    frontier = util.PriorityQueue()
    frontier.push([problem.getStartState(), []], 0) # ({(a,b): 0}, 0)
    CycleCheck = []

    while(not frontier.isEmpty()):
        popped = frontier.pop()
        startState = popped[0]
        path = popped[1]
        CycleCheck.append(startState)
        if problem.isGoalState(startState):
            return path
        else:
            successors = problem.getSuccessors(startState) #(c,d) and (e,f)
            for successor, action, _ in successors: #(2,3)
                heap_successors = [nodes[2][0] for nodes in frontier.heap]
                if successor not in CycleCheck and successor not in heap_successors:
                    frontier.update([successor, path + [action]], problem.getCostOfActions(path + [action]))
                elif successor not in CycleCheck and successor in heap_successors:
                    for i in frontier.heap:
                        if i[2][0] == successor and problem.getCostOfActions(path + [action]) < i[0]:
                            frontier.update([successor, path + [action]], problem.getCostOfActions(path + [action]))



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    frontier = util.PriorityQueue()
    frontier.push([problem.getStartState(), []], heuristic(problem.getStartState(), problem)) # ({(a,b): 0}, 0)
    CycleCheck = []

    while(not frontier.isEmpty()):
        popped = frontier.pop()
        startState = popped[0]
        path = popped[1]
        CycleCheck.append(startState)
        if problem.isGoalState(startState):
            return path
        else:
            successors = problem.getSuccessors(startState)#(c,d) and (e,f)
            present_heap= frontier.heap
            heap_successors = [nodes[2][0] for nodes in present_heap]
            for successor, action, _ in successors: #(2,3)
                if successor not in CycleCheck and successor not in heap_successors:
                    frontier.update([successor, path + [action]], heuristic(successor, problem) + problem.getCostOfActions(path + [action]))
                elif successor not in CycleCheck and successor in heap_successors:
                    for i in present_heap:
                        if i[2][0] == successor and heuristic(successor, problem) + problem.getCostOfActions(path + [action]) < i[0]:
                            frontier.update([successor, path + [action]], heuristic(successor, problem) + problem.getCostOfActions(path + [action]))
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
