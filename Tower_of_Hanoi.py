#To run the file , the following command needs to be run on the console/command line with the necessary arguments
# runfile('~path/Tower_of_Hanoi.py', wdir='~directory', args='-m <m-value> --heuristic <Heuristic name> <n-value>')

#--help can be used to access values of arguments
#<n-value> : "Number of discs"
# "--heuristic": <m-value> :'disksNotOnRightmost','distancefromlastpeg','Blind','PDB', default="disksNotOnRightmost"
# "-m" : <m-value> : m<=n, default=0 , This argument is only needed for PDB

'''
The script prints the algorithm flow giving details as well as algorithm
results and statistics. 
'''
import argparse
import time
import heapq
from collections import defaultdict

class State(object):
    """
    Encapsulates per-state information for the tower world planning task.

    Variables:
    ---------
    
        tower_world: stores reference to the tower_world to be used
        which contains list of size 4 containing discs for each rod
        (disks are ordered in ascending order of diameter)        
    """
    def __init__(self, tower_world):
        self.tower_world = tower_world
        
    def __hash__(self):
        return hash(self.__tuple__())

    def __tuple__(self):
        return (self.tower_world)

    def __lt__(self, other):
        return type(self) == type(other) and self.__tuple__() < other.__tuple__()

    def __gt__(self, other):
        return type(self) == type(other) and self.__tuple__() > other.__tuple__()

    def __le__(self, other):
        return type(self) == type(other) and self.__tuple__() <= other.__tuple__()

    def __ge__(self, other):
        return type(self) == type(other) and self.__tuple__() >= other.__tuple__()

    def __eq__(self, other):
        return type(self) == type(other) and self.__tuple__() == other.__tuple__()        
        
class TowerWorld(object):
    """
    Stores tower world related information (i.e., the location of the discs on rod etc.). 
    Also implements the coin collection planning task semantics.

    Variables
    ---------
        no_of_rods:  the number of rods in tower grid
        no_of_discs: the number of discs in tower grid
        rod_discs: List of size no_of_rods storing discs 
        (in descending order of diameter) per-rod

    Functions
    ---------
        get_disc_count(): Returns the number of discs
        is_feasible_move(disc,rod): checks if it is feasible to move disc to rod
        has_disc(rod): checks if there is a disc at the given rod
        get_rod(disc, positions): Return the rod where the disc is placed in given
        positions Otherwise returns None.
        get_initial_state(): Returns the initial state.
        get_successor_states(state): Computes and returns the list of successor
                                     states.
    """

    def __init__(self, n, rods):
        #initializing the attributes of the class
        self.no_of_rods = rods
        self.no_of_discs = n
        self.rod_discs = [[] for i in range(rods)]
        self.rod_discs[0] = [i for i in range(n-1,-1,-1)]
        
    def get_disc_count(self):
        '''
        
        Returns
        -------
        TYPE integer
            Number of discs 

        '''
        return self.no_of_discs
        
    def has_disc(self, rod, positions):
        """
        Checks if there is a disc at a given rod
        """
        return len(positions[rod]) > 0
        
    def is_feasible_move(self,disc,rod, positions):
        """
        checks if it is feasible to move disc to rod
        """
        return positions[rod][-1] > disc if self.has_disc(rod, positions) else True
    
    def get_rod(self, disc, positions):
        """
        Return the rod where the disc is placed 
        Otherwise returns None.
        """
        for rod in range(self.no_of_rods):
            if disc in positions[rod]:
                return rod
        return None
    
    def get_initial_state(self):
        """
        Returns the initial state.
        """
        return State(self.rod_discs)

    def get_successor_states(self, state):
        '''      
        Computes and returns the list of successor states.
        
        Parameters
        ----------
        state : Class object
            current state of the tower world

        Returns
        -------
        result : list of lists
            Successor state

        '''
        result = []
        #list of rod_discs from the class Tower world
        current_positions = state.tower_world
        #Loop to get the current state of the tower world
        for rod in range(self.no_of_rods):
            discs_on_rod = current_positions[rod]
            if len(discs_on_rod) > 0:
                #get the disc to move from the current rod to get the successor state move
                disc_to_move = discs_on_rod[-1]
            else:
                continue
            for rod_next in range(self.no_of_rods):
                #Comparing the current rod with each rod in the current state and checking a feasible move
                if rod != rod_next and self.is_feasible_move(disc_to_move, rod_next, current_positions):
                    new_positions = current_positions[:]
                    #move the disc from the current rod to the new position and pop the disc from the current rod
                    new_positions[rod_next] = current_positions[rod_next] + [disc_to_move]
                    new_positions[rod] = new_positions[rod][:-1]
                    #store the new position of the disc for the successive state and append it.
                    succ_state = State(new_positions)
                    #append the rod in the result i.e list of lists 
                    result.append(succ_state)
        return result
    
class SearchNode(object):
    """
    Stores information of a search node in A*.

    Variables
    ---------
        state: referenced state
        parent: reference to SearchNode that was used by A* to generate this
                search node
        h: heuristic estimate
        g: A*'s g-value

    """
    def __init__(self, state, h, parent = None):
        #initializing the attributes of the class to find the states correctly
        #Here, the parent node is stored to keep track along with the h and g values 
        #At every level in the A* algo, the g value increases by 1
        assert(isinstance(state, State))
        assert(isinstance(h, int))
        assert(parent is None or isinstance(parent, SearchNode))
        self.state = state
        self.parent = parent
        self.h = h
        #the algorithm has unit cost 
        self.g = 0 if parent is None else parent.g + 1

    def extract_plan(self):
        plan = []
        p = self
        while p != None:
            plan.append(p.state)
            p = p.parent
        return list(reversed(plan))

    def set_flag(self, flags, val = True):
        flags[str(self.state.tower_world)] = True

    def check_if_flagged(self, flags):
        return flags[str(self.state.tower_world)]

    def get_f_value(self):
        return self.h + self.g
    
    def __lt__(self, other):
        return (self.get_f_value(), self.g, self.state) < (other.get_f_value(), other.g, other.state)

class SearchResult(object):
    """
    Stores A* related statistics and the final plan.
    """
    def __init__(self, expansions = 0, visited = 0, plan = None):
        
        self.expansions = expansions
        self.visited = visited
        self.plan = plan
        
def print_search_node(tower_world, node):
    """
    Prints a search node to the console.
    """
    for rod in range(tower_world.no_of_rods):
        print("{:<25} {:<25}".format(str(rod), str(node.state.tower_world[rod])))

def astar_search(tower_world, heuristic):
    """
    Implementation of the A* algorithm.
    """
    assert(isinstance(tower_world, TowerWorld))
    assert(isinstance(heuristic, Heuristic))
    #Get the initial state of the Tower of Hanoi. 
    initial_state = tower_world.get_initial_state()
    #Store the number of discs for which the ToH runs
    disc_count = tower_world.get_disc_count()
    rod_count = 4
    #Open list or the frontier where the nodes to be visited are stored
    open_list = [ SearchNode(initial_state, heuristic(initial_state)) ]
    expanded = defaultdict(lambda: False)
    visited = defaultdict(lambda: None)
    visited[str(initial_state.tower_world)] = open_list[0].h
    
    #Calling the A-star search and storing the result
    result = SearchResult()
            
    while len(open_list) > 0:
        #Pop the states from the open list and check for the successive/goal states
        node = heapq.heappop(open_list)
        if node.check_if_flagged(expanded):
            continue
        #For the node with the least amount of cost, expand it.
        print('Expanding node',str(node.state.tower_world),'with f =',node.get_f_value())
        result.expansions += 1
        #If the last rod has all the discs then goal is reached
        if len(node.state.tower_world[rod_count-1]) == disc_count:
            result.plan = node.extract_plan()
            print('Goal state reached',str(node.state.tower_world))
            print_search_node(tower_world, node)
            break
        #If goal not reached find successor nodes
        successors = tower_world.get_successor_states(node.state)
        for succ in successors:
            #If already expanded then move to the next successor
            if expanded[str(succ.tower_world)]:
                continue
            #Visit the node which is  not goal and not expaned to check its heuristic value
            if visited[str(succ.tower_world)] is None:
                print('visiting',str(succ.tower_world),'with h =',heuristic(succ))
                result.visited += 1
                visited[str(succ.tower_world)] = heuristic(succ)
            #Push the visited node into the frontier to check in the next iter if it can be expanded
            heapq.heappush(open_list, SearchNode(succ, visited[str(succ.tower_world)], node))
            
       
        node.set_flag(expanded)
        
    #return the result which has the complete plan
    return result

class Heuristic(object):
    """
    Heuristic base class.
    
    Variables:
        tower_world: stores a reference to the tower_world
        disc_count: stores the total count of discs i.e. n
        rod_count: stores the total number of rods in tower_world
    """
    def __init__(self, tower_world):
        assert(isinstance(tower_world, TowerWorld))
        self.tower_world = tower_world
        self.disc_count = tower_world.get_disc_count()
        self.rod_count = 4
        

    def __call__(self, state):
        assert(isinstance(state, State))
        raise NotImplementedError("call function has not been implemented yet")
            
class disksNotOnRightmostHeuristic(Heuristic):
    """
    The count of discs which are not on the rightmost peg.
    """
    def __init__(self, tower_world):
        super(disksNotOnRightmostHeuristic, self).__init__(tower_world)

    def __call__(self, state):
        assert(isinstance(state, State))
        # The actual heuristic computation.
        return self.disc_count - len(state.tower_world[self.rod_count-1])
    
class distancefromlastpegHeuristic(Heuristic):
    """
    The distance of the disc of the largest size as the number of 
    pegs it is away from the rightmost peg
    """
    def __init__(self, tower_world):
        super(distancefromlastpegHeuristic, self).__init__(tower_world)

    def __call__(self, state):
        assert(isinstance(state, State))
        # The actual heuristic computation.
        big_disc = self.disc_count-1;
        for position in range(self.rod_count):
            if big_disc in state.tower_world[position]:
                return self.rod_count - position - 1
                
        
    
class BlindHeuristic(Heuristic):
    """
    The blind heuristic. Returns 0 for every state. Using this heuristic will
    turn A* into simple Dijkstra search. In our unit-cost setting, A* with the
    blind heuristic boils down to a simple breadth-first search.
    """
    def __init__(self, tower_world):
        # Call the super constructor to initialize the grid and coin_location
        # variables:
        super(BlindHeuristic, self).__init__(tower_world)

    def __call__(self, state):
        # The actual heuristic computation. The blind heuristic will simply
        # always return 0.
        return 0
    
class PDBHeuristic(Heuristic):
    """
    The PDB heuristic creates abstract nodes for m out of n total discs.
    First the data base is created and then the search algorithm of A* is executed.
    Pattern database heuristic starts from the goal and builds down to the initial state.
    """
    def __init__(self, tower_world, m):
        super(PDBHeuristic, self).__init__(tower_world)
        self.m = m
        #creating the pattern database
        self.pdb = self.createPDB()
        
    def abstractLargestm(self, state):
        abstract_state = [[] for i in range(self.rod_count)]
        n = self.disc_count
        for i in range(len(state)):
            rodi_discs = state[i]
            for j in rodi_discs:
                if j >= n - (self.m):
                    abstract_state[i].append(j)
        return abstract_state
    
    def createPDB(self):
       
        pdb = dict()
        #starting from the goal state
        goal_state = [[] for i in range(self.rod_count)]
        goal_state[-1] = [i for i in range(self.disc_count-1,-1,-1)]
        
        open_list = [SearchNode(State(goal_state), 0)]
        expanded = defaultdict(lambda: False)
        visited = defaultdict(lambda: None)
        visited[str(goal_state)] = open_list[0].h
          
        #Performing BFS and storing values into PDB              
        while len(open_list) > 0:
            node = heapq.heappop(open_list)
            if node.check_if_flagged(expanded):
                continue
            #abstarcting the state before storing 
            abstract_state1 = self.abstractLargestm(node.state.tower_world)
            if str(abstract_state1) not in pdb.keys():
                pdb[str(abstract_state1)] = node.get_f_value()
            
            #Stopping BFS when initial state is reached
            if len(node.state.tower_world[0]) == self.disc_count:
                abstract_state1 = self.abstractLargestm(node.state.tower_world)
                if str(abstract_state1) not in pdb.keys():
                    pdb[str(abstract_state1)] = node.get_f_value()
                    break
            successors = tower_world.get_successor_states(node.state)
            for succ in successors:
                if expanded[str(succ.tower_world)]:
                    continue
                if visited[str(succ.tower_world)] is None:
                    visited[str(succ.tower_world)] = 0
                heapq.heappush(open_list, SearchNode(succ, visited[str(succ.tower_world)], node))
            node.set_flag(expanded)
            
        return pdb
        
    def getPDB(self):
        return self.pdb

    def __call__(self, state):
        # The actual heuristic computation. The blind heuristic will simply
        # always return 0.
        abstract_state1 = self.abstractLargestm(state.tower_world)
        return self.pdb[str(abstract_state1)]



if __name__ == "__main__":
    #Arugument parser for calling the function
    p = argparse.ArgumentParser()
    p.add_argument("n", help="Number of discs", nargs="?", default=3)
    p.add_argument("--heuristic", help="Which heuristic to use.The blind heuristic will turn A* into simple Breadth-First Search.", choices=['disksNotOnRightmost','distancefromlastpeg','Blind','PDB'], default="disksNotOnRightmost")
    p.add_argument("-m", help="Number of discs to consider for PDB", default=0)
    args = p.parse_args()
    
    n = int(args.n)
    #Object for tower world with 4 pegs and n discs
    tower_world = TowerWorld(n, 4)
    
    #For heuristic Data base the argument m is also taken into account, for all other heuristic only n is considered
    if args.heuristic == "PDB":
        t1 = time.time()
        h = globals()["%sHeuristic" % args.heuristic](tower_world, int(args.m))
        pdb_time = (time.time() - t1)
        print(h.getPDB())
    else:
        h = globals()["%sHeuristic" % args.heuristic](tower_world)
    
    #Time calculation and print statements
    t = time.time()
    
    print('--------------ALGORITHM FLOW---------------\n')
    #Calling the function a_star to run Tower of Hanoi 
    result = astar_search(tower_world, h)
    print("")
    print('--------------ALGORITHM RUNTIME AND RESULTS-----------------\n')
    
    if args.heuristic == "PDB":
        print("Total time to create PDB: %.3fs" % pdb_time)
    
    print("Total time for search algorithm:      %.3fs" % (time.time() - t))

    if result.plan is None:
        print("Search terminated without finding a solution!")
    else:
        print("Solution found!")
        print("Plan length:     %d" % (len(result.plan) - 1))
        
    print("States expanded: %d" % result.expansions)
    print("States visited:  %d" % result.visited)