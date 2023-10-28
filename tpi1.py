#STUDENT NAME:
#STUDENT NUMBER:

#DISCUSSED TPI-1 WITH: (names and numbers):


from tree_search import *

class OrderDelivery(SearchDomain):

    def __init__(self,connections, coordinates):
        self.connections = connections
        self.coordinates = coordinates
        # ANY NEEDED CODE CAN BE ADDED HERE

    def actions(self,state):
        city = state[0]
        actlist = []
        for (C1,C2,D) in self.connections:
            if (C1==city):
                actlist += [(C1,C2)]
            elif (C2==city):
               actlist += [(C2,C1)]
        return actlist 

    def result(self,state,action):
        #IMPLEMENT HERE
        pass

    def satisfies(self, state, goal):
        #IMPLEMENT HERE
        pass

    def cost(self, state, action):
        #IMPLEMENT HERE
        pass

    def heuristic(self, state, goal):
        #IMPLEMENT HERE
        pass


 
class MyNode(SearchNode):

    def __init__(self,state,parent,arg3=None,arg4=None,arg5=None,arg6=None):
        super().__init__(state,parent)
        #ADD HERE ANY CODE YOU NEED

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',maxsize=None):
        super().__init__(problem,strategy)
        #ADD HERE ANY CODE YOU NEED

    def astar_add_to_open(self,lnewnodes):
        #IMPLEMENT HERE
        pass

    def search2(self):
        #IMPLEMENT HERE
        pass

    def manage_memory(self):
        #IMPLEMENT HERE
        pass

    # if needed, auxiliary methods can be added here


# If needed, auxiliary functions can be added here



