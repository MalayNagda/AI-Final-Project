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
    return  [w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    #The following lines are mine
    
    from game import Directions
    #import ipdb
    #Problem is an object of class SearchProblem
    start_state=problem.getStartState()
    state=start_state #-1 and -100 is to indicate that this is the start state
    
    fringe_stack=util.Stack()
    explored_states=util.Stack()
    is_state_in_fringe={start_state:True}
    successor_state_for_this_state={}
    
    successor_states=problem.getSuccessors(state)
    successor_state_for_this_state[state]=successor_states

    ancestors={state:[]} #Root state has no ancestors
    
    for i in successor_states:
        fringe_stack.push(i)
        is_state_in_fringe[i]=True
        ancestors[i[0]]=[start_state]

    state=fringe_stack.pop()
    #Explore all states to find the goal state. Don't expand the goal state itself
    while(not problem.isGoalState(state[0])):
        #ipdb.set_trace()
        
        #print state[0]
        successor_states=problem.getSuccessors(state[0])
        successor_state_for_this_state[state[0]]=successor_states
        #ipdb.set_trace()
        
        for i in successor_states:
            next_state=i[0]
            if(next_state in ancestors): #If its ancestors are already listed
                if(state[0] not in ancestors[next_state] and next_state not in ancestors[state[0]]): #Ancestors don't include current state, and no cycle is being formed
                    ancestors[next_state].append(state[0])
                pass
            else:
                ancestors[next_state]=[state[0]]
                ancestors[next_state].extend(ancestors[state[0]])
            
            if(problem.isGoalState(next_state)):
                fringe_stack.push(i)
                is_state_in_fringe[next_state]=True
                break #If there is a goal state, it should be the first to pop out of the stack

            if(next_state not in is_state_in_fringe or next_state not in ancestors[state[0]] ): #Add the next state in the fringe if and only if 
                fringe_stack.push(i)
                is_state_in_fringe[next_state]=True

        #print state    
        
        explored_states.push(state)
        state=fringe_stack.pop()
        noSuccessorIsGoal=True
        #pass
    #print "Goal Found at", state[0]
    #If loop has ended, that means state=GoalState. From explored state, backtrack to obtain the start state
    if(problem.isGoalState(state[0]) and noSuccessorIsGoal):
        explored_states.push(state) #Since it wasn't added in the loop

    planning_stack=util.Stack()
    #ipdb.set_trace()
    current_state=explored_states.pop()
    planning_stack.push(current_state)
    while(len(explored_states.list)>0):
        
        predecessor_state=explored_states.pop()
        successor_states=successor_state_for_this_state[predecessor_state[0]]

        for i in successor_states:
            #if(problem.isGoalState(predecessor_state[0])):
                #noSuccessorIsGoal=False
            if(i==current_state): #Both the predecessor state and the action are the same
                current_state=predecessor_state
                planning_stack.push(current_state)
    #Now the planning stack is full. The state next to the start state is on top of the stack, stored with the action that got it there
    action_sequence=[]
    while(len(planning_stack.list)>0):
        current_state=planning_stack.pop()
        action_sequence.append(current_state[1])

    return action_sequence
        
    
    #return [Directions.SOUTH]

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    #The following lines are mine
    
    from game import Directions
    #import ipdb
    
    #Problem is an object of class SearchProblem
    start_state=problem.getStartState()
    state=(start_state,-1,0) #-1 and -100 is to indicate that this is the start state
    
    fringe_queue=util.Queue()
    #fringe_queue.push([state],0)
    explored_states=util.Stack()
    
    successor_states=problem.getSuccessors(state[0])
    #successor_state_for_this_state[state]=successor_states

    
    for i in successor_states:
        temp=[state]
        temp.append(i)
        fringe_queue.push(temp)
        #costs[i[0]]=i[2]
    #ipdb.set_trace()    
        
    traj=fringe_queue.pop()
    curr_state=traj[-1]
    nodes_expanded=[state[0]]
    
    while(not problem.isGoalState(curr_state[0])):
        
        if(curr_state[0] not in nodes_expanded):
            nodes_expanded.append(curr_state[0])
            successor_states=problem.getSuccessors(curr_state[0])
              
            for next_state in successor_states:
                temp=traj[:]
                temp.append(next_state)
                cost=0
                if next_state[0] not in nodes_expanded:
                    
                    fringe_queue.push(temp)
        #if curr_state[0]==[5,1,1,1,1,1]:
        #    ipdb.set_trace()

        #ipdb.set_trace()   
        #print curr_state     
        traj=fringe_queue.pop()
        curr_state=traj[-1]
            
    #Loop has exited=> curr_state=goalState
    action_sequence=[]
    for i in traj[1:]:
        action_sequence.append(i[1])
    
    return action_sequence

    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #The following lines are mine
    
    from game import Directions
    #import ipdb
    
    #Problem is an object of class SearchProblem
    start_state=problem.getStartState()
    state=(start_state,-1,0) #-1 and -100 is to indicate that this is the start state
    
    fringe_queue=util.PriorityQueue()
    #fringe_queue.push([state],0)
    explored_states=util.Stack()
    
    successor_states=problem.getSuccessors(state[0])
    #successor_state_for_this_state[state]=successor_states

    
    for i in successor_states:
        temp=[state]
        temp.append(i)
        fringe_queue.update(temp,i[2])
        #costs[i[0]]=i[2]
    #ipdb.set_trace()    
        
    traj=fringe_queue.pop()
    curr_state=traj[-1]
    nodes_expanded=[state[0]]
    
    while(not problem.isGoalState(curr_state[0])):
        
        if(curr_state[0] not in nodes_expanded):
            nodes_expanded.append(curr_state[0])
            successor_states=problem.getSuccessors(curr_state[0])
              
            for next_state in successor_states:
                temp=traj[:]
                temp.append(next_state)
                cost=0
                if next_state[0] not in nodes_expanded:
                    #Calculate traj_cost
                    #ipdb.set_trace()    
                    for past_states in traj:
                        cost+=past_states[2]
                    fringe_queue.update(temp,cost+next_state[2])
            #ipdb.set_trace()        
        traj=fringe_queue.pop()
        curr_state=traj[-1]
            
    #Loop has exited=> curr_state=goalState
    action_sequence=[]
    for i in traj[1:]:
        action_sequence.append(i[1])
    
    return action_sequence


            

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    "*** YOUR CODE HERE ***"
    #The following lines are mine
    
    from game import Directions
    #import ipdb
    
    #Problem is an object of class SearchProblem
    start_state=problem.getStartState()
    
    
    state=(start_state,-1,0) #-1 and -100 is to indicate that this is the start state
    
    fringe_queue=util.PriorityQueue()
    #fringe_queue.push([state],0)
    explored_states=util.Stack()

    successor_states=problem.getSuccessors(state[0])
    #successor_state_for_this_state[state]=successor_states

    
    for i in successor_states:
        temp=[state]
        temp.append(i)
        #ipdb.set_trace()
        fringe_queue.update(temp,i[2]+heuristic(i[0],problem))#This is the only difference between UCS and A*    
        
    traj=fringe_queue.pop()
    curr_state=traj[-1]
    nodes_expanded=[state[0]]
    
    while(not problem.isGoalState(curr_state[0])):
        
        if(curr_state[0] not in nodes_expanded):
            nodes_expanded.append(curr_state[0])
            successor_states=problem.getSuccessors(curr_state[0])
              
            for next_state in successor_states:
                temp=traj[:]
                temp.append(next_state)
                cost=0
                if next_state[0] not in nodes_expanded:
                    #Calculate traj_cost
                    
                    for past_states in traj:
                        cost+=past_states[2]
                    fringe_queue.update(temp,cost+next_state[2]+heuristic(next_state[0],problem)) #This is the only difference between UCS and A*
            #ipdb.set_trace()        
        traj=fringe_queue.pop()
        curr_state=traj[-1]
            
    #Loop has exited=> curr_state=goalState
    action_sequence=[]
    for i in traj[1:]:
        action_sequence.append(i[1])
    
    return action_sequence
    util.raiseNotDefined()

def aStarSearchLocallyObservableTunnelVision(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    #The following lines are mine

    #Problem is an object of class SearchProblem. Since the goal is fixed at (1,1), this is a PositionSearchProblem
    current_state=problem.getStartState()
    path=[]
    while(not problem.isGoalState(current_state)):
        current_path=aStarSearch(problem,heuristic)
        #follow current path until a wall is detected.
        for action in current_path:
            if(not problem.senseWall(current_state,action)):
                path.append(action)
                current_state=problem.getNextState(current_state,action)
            else:
                break
        if(problem.isGoalState(current_state)):
            return path
        else:
            wall_location=problem.getNextState(current_state,action)
            problem.addWall(wall_location)
            problem.setStartState(current_state)
           
    return path

def aStarSearchLocallyObservable(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    #The following lines are mine

    #Problem is an object of class SearchProblem. Since the goal is fixed at (1,1), this is a PositionSearchProblem
    current_state=problem.getStartState()
    path=[]
    while(not problem.isGoalState(current_state)):
        current_path=aStarSearch(problem,heuristic)
        #follow current path until a wall is detected.
        for action in current_path:
            if(not problem.getNeighboringWalls(current_state)):
                path.append(action)
                current_state=problem.getNextState(current_state,action)
            else:
                break
        if(problem.isGoalState(current_state)):
            return path
        else:
            problem.setStartState(current_state)           
    return path


def dStarSearch(problem):

    def manhattanDistance(s1,s2):
        return abs(s1[0]-s2[0])+abs(s1[1]-s2[1])

    def CalculateKey(u):
        return [min(g[u],rhs[u])+manhattanDistance(start_state,u),min(g[u],rhs[u])]

    def UpdateVertex(u):
        if(not problem.isGoalState(u)): 
            rhs[u]=float('inf')
            for s,a,d in problem.getSuccessors(u):
                rhs[u]=min(rhs[u],g[s]+1)

        if(U.CheckPresence(u)):
            U.Remove(u)

        if(g[u]!=rhs[u]):
            U.update(u,CalculateKey(u))

    def ComputeShortestPath():
        while(U.TopKey()<CalculateKey(start_state) or rhs[start_state]!=g[start_state]):
            
            if(len(U.heap)==1):
                import ipdb
                #ipdb.set_trace()

            u=U.Top()
            k_old=U.TopKey()
            k_new=CalculateKey(u)

            U.pop()
            #print(rhs[start_state])

            if(k_old<k_new): #this means some edges have become unusable
                U.update(u,k_new)

            elif(g[u]>rhs[u]): #This means a shorter path has been found
                g[u]=rhs[u]
                for s,a,d in problem.getSuccessors(u):
                    UpdateVertex(s)
            else:  #This means g[u]=rhs[u] i.e vertex is locally consistent
                g_old=g[u]
                g[u]=float('inf')
                UpdateVertex(u)

                for s,a,d in problem.getSuccessors(u):
                    UpdateVertex(s)

    def senseWallAt(problem,s):
        return problem.actual_walls[s[0]][s[1]]

    def knowWallAt(problem,s):
        return problem.walls[s[0]][s[1]]

    from util import PriorityQueue
    
    #Initialize
    U=PriorityQueue()
    km=0
    start_state=problem.getStartState()
    last_state=start_state #A copy is created, not a reference
    problem.getNeighboringWalls(start_state) #Update the agent model
    rhs={}
    g={}

    for state in problem.AllStates():
        rhs[state]=float('inf')
        g[state]=float('inf')

    goal=(1,1) #Predefined
    rhs[goal]=0
    U.push(goal,[manhattanDistance(goal,start_state),0])

    ComputeShortestPath()
    
    actions=[]


    while(not problem.isGoalState(start_state)):
        min_successor_value=float('inf')
        current_action=[]

        for s,a,discombombulation in problem.getSuccessors(start_state):
           
            if(1+g[s]<min_successor_value):
                start_state=s
                current_action=a
                min_successor_value=1+g[s]

        actions.append(current_action)
        
        #Scan for edge-weight changes after moving to the new start_state
        changes=problem.getNeighboringWalls(start_state)
        
        if(changes):
            km+=manhattanDistance(last_state,start_state)
            last_state=start_state
            for wall in changes:
                g[wall]=float('inf') #Since there's a wall, it's impossible to reach from the goal node
                
                #When a wall is detected, upto 4 edges become infinitely weighted. Update all their successors (predecessors)
                successors=problem.getSuccessors(wall) #No edge weight has gone down
                for s1,a1,discombombulation in successors:
                    UpdateVertex(s1)

        ComputeShortestPath()

    return actions


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
astar2=aStarSearchLocallyObservable
astartv=aStarSearchLocallyObservableTunnelVision
dstar=dStarSearch