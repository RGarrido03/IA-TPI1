# STUDENT NAME: Rúben Tavares Garrido
# STUDENT NUMBER: 107927

# DISCUSSED TPI-1 WITH: (names and numbers):


from tree_search import *


class OrderDelivery(SearchDomain):

    def __init__(self, connections, coordinates):
        self.connections = connections
        self.coordinates = coordinates
        # ANY NEEDED CODE CAN BE ADDED HERE

    def actions(self, state):
        city = state[0]
        actlist = []
        for (C1, C2, D) in self.connections:
            if C1 == city:
                actlist += [(C1, C2)]
            elif C2 == city:
                actlist += [(C2, C1)]
        return actlist

    def result(self, state, action):
        # IMPLEMENT HERE
        pass

    def satisfies(self, state, goal):
        # IMPLEMENT HERE
        pass

    def cost(self, state, action):
        # IMPLEMENT HERE
        pass

    def heuristic(self, state, goal):
        # IMPLEMENT HERE
        pass


class MyNode(SearchNode):

    def __init__(self, state, parent, depth=None, cost=None, heuristic=None, eval_arg=None):
        super().__init__(state, parent)
        self.depth = depth
        self.cost = cost
        self.heuristic = heuristic
        self.eval = eval_arg
        self.is_marked_for_deletion = False
        # ADD HERE ANY CODE YOU NEED


class MyTree(SearchTree):

    def __init__(self, problem, strategy='breadth', maxsize=None):
        super().__init__(problem, strategy)
        root = MyNode(problem.initial, None, 0, 0, 0, 0)
        self.open_nodes = [root]
        self.terminals = 1
        self.maxsize = maxsize
        self.count = 0
        # ADD HERE ANY CODE YOU NEED

    def astar_add_to_open(self, lnewnodes):
        self.open_nodes.extend(lnewnodes)
        self.open_nodes.sort(key=lambda n: (n.eval, n.state))

    def search2(self):
        while self.open_nodes:
            node = self.open_nodes.pop(0)
            self.terminals = len(self.open_nodes) + 1
            if self.problem.goal_test(node.state):
                self.solution = node
                return self.get_path(node)
            self.non_terminals += 1
            lnewnodes = []
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state, a)
                if newstate not in self.get_path(node):
                    newnode = MyNode(newstate, node, node.depth + 1,
                                     node.cost + self.problem.domain.cost(node.state, a),
                                     self.problem.domain.heuristic(newstate, self.problem.goal),
                                     node.cost + self.problem.domain.cost(node.state, a) +
                                     self.problem.domain.heuristic(newstate, self.problem.goal))
                    lnewnodes.append(newnode)
            self.add_to_open(lnewnodes)
            self.count = 0
            self.manage_memory()
        return None

    def manage_memory(self):
        self.count += 1
        if self.strategy != "A*" or self.maxsize is None or self.terminals + self.non_terminals <= self.maxsize:
            return

        print(f"Recursion count if true: {self.count}")
        self.open_nodes.sort(key=lambda n: (n.eval, n.state))
        for node in reversed(self.open_nodes):
            if not node.is_marked_for_deletion:
                node.is_marked_for_deletion = True
                parent_node = node.parent

                # Get node's siblings
                node_siblings = [n for n in self.open_nodes if n.parent == parent_node and n != node]

                print([sibling.is_marked_for_deletion for sibling in node_siblings])
                if all([sibling.is_marked_for_deletion for sibling in node_siblings]):
                    print("-> All matched")

                    # Remove node and siblings
                    self.open_nodes.remove(node)
                    for sibling in node_siblings:
                        self.open_nodes.remove(sibling)
                    self.open_nodes.remove(parent_node)

                    # Get minimum eval for each parent child
                    parent_node.eval = min(node.eval for node in node_siblings + [node])

                    self.open_nodes.append(parent_node)

                    # Sort again (is this needed?)
                    self.open_nodes.sort(key=lambda n: (n.eval, n.state))
                break

        # Repeat to check if size still exceeds the threshold
        # self.manage_memory()

    # if needed, auxiliary methods can be added here


def orderdelivery_search(domain, city, targetcities, strategy='breadth', maxsize=None):
    # IMPLEMENT HERE
    pass

# If needed, auxiliary functions can be added here
