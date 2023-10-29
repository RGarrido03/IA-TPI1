# STUDENT NAME: Rúben Tavares Garrido
# STUDENT NUMBER: 107927
import math
import time

# DISCUSSED TPI-1 WITH: (names and numbers):


from tree_search import *

is_sleep_on = False


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
        if is_sleep_on:
            time.sleep(0.1)

        print(f"\nRESULT()\nstate: {type(state).__name__} = {state}\naction: {type(action).__name__} = {action}")
        print(f"action: {action}")

        c1, c2 = action
        new = c1 if c1 == state[0] else c2

        if new in state[1]:
            state[1].remove(new)
        return new, state[1]

    def satisfies(self, state, goal):
        if is_sleep_on:
            time.sleep(0.1)

        print(f"\nSATISFIES()\nstate: {type(state).__name__} = {state}\ngoal: {type(goal).__name__} = {goal}")
        print(f"Current goal cities in state: {[city in state for city in goal]}")

        return state[0] == goal and len(state[1]) == 0

    def cost(self, state, action):
        if is_sleep_on:
            time.sleep(0.1)

        print(f"\nCOST()\nstate: {type(state).__name__} = {state}\naction: {type(action).__name__} = {action}")

        for (x1, x2, d) in self.connections:
            if (x1, x2) == action or (x2, x1) == action:
                print(f"Returning {d}")
                return d

    def heuristic(self, state, goal):
        if is_sleep_on:
            time.sleep(0.1)

        print(f"\nHEURISTIC()\nstate: {type(state).__name__} = {state}\ngoal: {type(goal).__name__} = {goal}")

        c1_x, c1_y = self.coordinates[state[0]]
        c2_x, c2_y = self.coordinates[goal]

        h = round(math.hypot(c1_x - c2_x, c1_y - c2_y))
        print(f"Returning {h}")
        return h


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
        # ADD HERE ANY CODE YOU NEED

    def astar_add_to_open(self, lnewnodes):
        self.open_nodes.extend(lnewnodes)
        self.open_nodes.sort(key=lambda n: (n.eval, n.state))

    def search2(self):
        while self.open_nodes:
            node = self.open_nodes.pop(0)
            if self.problem.goal_test(node.state):
                self.terminals = len(self.open_nodes) + 1
                self.solution = node
                return self.get_path(node)
            self.non_terminals += 1
            lnewnodes = []
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state, a)
                if newstate not in self.get_path(node):
                    cost = node.cost + self.problem.domain.cost(node.state, a)
                    heuristic = self.problem.domain.heuristic(newstate, self.problem.goal)

                    newnode = MyNode(newstate, node, node.depth + 1, cost, heuristic, cost + heuristic)
                    lnewnodes.append(newnode)
            self.add_to_open(lnewnodes)
            self.terminals = len(self.open_nodes)
            self.manage_memory()
        return None

    def manage_memory(self):
        if self.strategy != "A*" or self.maxsize is None or self.terminals + self.non_terminals <= self.maxsize:
            return

        self.open_nodes.sort(key=lambda n: (n.eval, n.state))
        for node in reversed(self.open_nodes):
            if not node.is_marked_for_deletion:
                node.is_marked_for_deletion = True

                parent_node = node.parent
                if parent_node is None:
                    return

                # Get node's parent children (node + its siblings)
                parent_children = [n for n in self.open_nodes if n.parent == parent_node]

                if all([sibling.is_marked_for_deletion for sibling in parent_children]):
                    # Remove node and siblings
                    for child in parent_children:
                        self.open_nodes.remove(child)

                    # Get minimum eval for each parent child, update the parent node and return it to the queue
                    parent_node.eval = min(child.eval for child in parent_children)
                    parent_node.is_marked_for_deletion = True
                    self.non_terminals -= 1

                    # for n in self.open_nodes:
                    #     n.is_marked_for_deletion = False
                break

        # Repeat to check if size still exceeds the threshold
        self.manage_memory()


def orderdelivery_search(domain, city, targetcities, strategy='breadth', maxsize=None):
    p = SearchProblem(domain, (city, targetcities), city)
    t = MyTree(p, strategy, maxsize)

    return t, [city for (city, unvisited) in t.search2()]

# If needed, auxiliary functions can be added here
