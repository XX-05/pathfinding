from queue import Queue, PriorityQueue


class Graph:
    def __init__(self, graph: list[tuple[str, str]]):
        self.graph = graph
        nodes = self.bfs(self.graph[0][0]).keys()
        self.costs = {n: 0 for n in nodes}

    def set_cost(self, node: str, cost: int):
        self.costs[node] = cost

    def find_connections(self, node: str):
        connections = []
        for edge in self.graph:
            if edge[0] == node:
                connections.append(edge[1])
        return connections

    @staticmethod
    def _generate_path(came_from: dict[str, str], start: str, target: str):
        path = []
        n = target
        while n != start:
            path.append(n)
            n = came_from[n]
        path.reverse()
        return path

    def bfs(self, start: str) -> dict[str, str]:
        """
        Performs a breadth first search over the graph
        expanding the frontier outwards from the start node.
        :param start: The node of the graph to begin the search from.
        :return: A `visited` dict mapping graph nodes to the frontier
            node it was found through.
        """
        frontier = Queue()
        frontier.put(start)
        came_from = {start: start}

        while not frontier.empty():
            node = frontier.get()
            for n in self.find_connections(node):
                if n not in came_from:
                    frontier.put(n)
                    came_from[n] = node

        return came_from

    def bfs_path(self, start: str, target: str):
        """
        Returns the shortest path between start and target found by breadth-first-search.
        """
        came_from = self.bfs(start)
        return self._generate_path(came_from, start, target)

    def uniform_cost_search(self, start: str, target: str):
        frontier = PriorityQueue()
        frontier.put((0, start))

        sum_costs = {start: 0}
        came_from = {}

        while not frontier.empty():
            node = frontier.get()[1]
            for n in self.find_connections(node):
                cost = sum_costs[node] + self.costs[n]
                if n not in sum_costs or cost < sum_costs[n]:
                    sum_costs[n] = cost
                    came_from[n] = node
                    frontier.put((cost, n))

        return self._generate_path(came_from, start, target)


if __name__ == "__main__":
    g = Graph([('a', 'b'),
               ('a', 'c'),
               ('b', 'c'),
               ('b', 'd'),
               ('c', 'e'),
               ('c', 'd'),
               ('d', 'g'),
               ('e', 'f'),
               ('g', 'f')])

    g.set_cost('c', 10)

    print(g.uniform_cost_search('a', 'f'))
    print(g.bfs_path('a', 'f'))
