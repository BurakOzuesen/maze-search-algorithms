import sys
import time

from heapq import heappush, heappop
from operator import lt

import random
import matplotlib.pyplot as plt
import math

sys.setrecursionlimit(2 ** 30)


class Vertex():
    visited = False
    row_number = 0
    column_number = 0
    connectedCells = []
    uniformCostDistance = 0
    manhattanDistance = 0
    euclideanDistance = 0
    parent = None

    def __lt__(self, other):
        multiplier = random.randint(100, 1000)
        secondMultiplier = random.randint(0, 10)
        return lt(self.row_number * multiplier < other.row_number * secondMultiplier, True)

    def __repr__(self):
        return "(" + str(self.row_number) + "," + str(self.column_number) + ")"

    def __init__(self, row_number, column_number, connectedCells=None):
        if connectedCells is None:
            connectedCells = []
        self.row_number = row_number
        self.column_number = column_number
        self.connectedCells = connectedCells


def distanceCalculator(matrix):
    row_count = len(matrix)
    column_count = len(matrix[0])
    for x in range(row_count):
        for y in range(column_count):
            x_dist = (row_count - x - 1) ** 2
            y_dist = (column_count - y - 1) ** 2
            matrix[x][y].euclideanDistance = math.sqrt(x_dist + y_dist)
            x_dist = (row_count - x - 1)
            y_dist = (column_count - y - 1)
            matrix[x][y].manhattanDistance = x_dist + y_dist
            matrix[x][y].uniformCostDistance = x_dist + y_dist


def randomUnvisitedNeighbour(vertex):
    randomArray = []

    if vertex.column_number != 0:
        leftNeighbour = grid[vertex.row_number][vertex.column_number - 1]
        if not leftNeighbour.visited:
            randomArray.append(leftNeighbour)

    if vertex.row_number != 0:
        topNeighbour = grid[vertex.row_number - 1][vertex.column_number]
        if not topNeighbour.visited:
            randomArray.append(topNeighbour)

    if vertex.column_number != cols - 1:
        rightNeighbour = grid[vertex.row_number][vertex.column_number + 1]
        if not rightNeighbour.visited:
            randomArray.append(rightNeighbour)

    if vertex.row_number != rows - 1:
        bottomNeighbour = grid[vertex.row_number + 1][vertex.column_number]
        if not bottomNeighbour.visited:
            randomArray.append(bottomNeighbour)

    if len(randomArray) == 0:
        return None
    random.shuffle(randomArray)

    return randomArray[0]


def createMaze():
    startVertex = grid[0][0]
    randomizedDFS(startVertex)
    return


def randomizedDFS(vertex):
    myStack = []
    vertex.visited = True
    myStack.append(vertex)
    while len(myStack) != 0:
        currentVertex = myStack[-1]
        del myStack[-1]
        nextVertex = randomUnvisitedNeighbour(currentVertex)
        if nextVertex is not None:
            myStack.append(currentVertex)
            currentVertex.connectedCells.append(nextVertex)
            nextVertex.connectedCells.append(currentVertex)

            x_of_edge = abs(nextVertex.row_number - currentVertex.row_number)
            y_of_edge = abs(nextVertex.column_number - currentVertex.column_number)
            x_of_edge += min(nextVertex.row_number, currentVertex.row_number) * 2
            y_of_edge += min(nextVertex.column_number, currentVertex.column_number) * 2

            printableMaze[x_of_edge][y_of_edge] = 1
            printableMaze[currentVertex.row_number * 2][currentVertex.column_number * 2] = 1
            printableMaze[nextVertex.row_number * 2][nextVertex.column_number * 2] = 1

            nextVertex.visited = True
            myStack.append(nextVertex)


def clearVisitedStatus(matrix):
    for x in range(rows):
        for y in range(cols):
            matrix[x][y].visited = False
            matrix[x][y].parent = None


def DLS(src, target, maxDepth):
    global expanded
    expanded += 1
    if src == target: return True
    if maxDepth <= 0: return False
    for i in src.connectedCells:
        if i.visited:
            continue
        i.visited = True
        i.parent = src

        if DLS(i, target, maxDepth - 1):
            return True
    return False


def IDDFS(src, target, maxDepth):
    print("Iterative Deepening Search")
    for i in range(maxDepth):
        clearVisitedStatus(grid)
        if DLS(src, target, i):
            print_path(src, target)

            return True
    return False


def uniform_cost_search(start, goal):
    global expanded
    print("")
    found, fringe, visited, came_from, cost_so_far = False, [(0, start)], {start}, {start: None}, {start: 0}
    while not found and len(fringe):
        _, current = heappop(fringe)
        if current == goal:
            found = True
            break

        for node in current.connectedCells:
            expanded += 1
            new_cost = cost_so_far[current] + 1
            if node.visited == False or cost_so_far[node] > new_cost:
                node.visited = True
                node.parent = current
                cost_so_far[node] = new_cost
                heappush(fringe, (new_cost, node))

    if found:
        print("")
        print("Uniform Cost Search")
        print_path(start, goal)
        print("")
        return came_from, cost_so_far[goal]


def a_star_search_with_Euclidean(start, goal):
    global expanded
    found, fringe, visited, came_from, cost_so_far = False, [(start.euclideanDistance, start)], {start}, {
        start: None}, {start: 0}
    while not found and len(fringe):
        _, current = heappop(fringe)
        if current == goal: found = True; break
        for node in current.connectedCells:
            new_cost = cost_so_far[current] + 1
            expanded += 1
            if node.visited == False or cost_so_far[node] > new_cost:
                node.visited = True
                node.parent = current
                cost_so_far[node] = new_cost
                heappush(fringe, (new_cost, node))
    if found:
        print("")
        print("A* Search With Euclidean Heuristic Values")
        print_path(start, goal)
        print("")
        return came_from, cost_so_far[goal]


def a_star_search_with_Manhattan(start, goal):
    global expanded
    found, fringe, visited, came_from, cost_so_far = False, [(start.manhattanDistance, start)], {start}, {
        start: None}, {start: 0}
    while not found and len(fringe):
        _, current = heappop(fringe)
        if current == goal: found = True; break
        for node in current.connectedCells:
            expanded += 1
            new_cost = cost_so_far[current] + 1
            if node.visited == False or cost_so_far[node] > new_cost:
                node.visited = True
                node.parent = current
                cost_so_far[node] = new_cost
                heappush(fringe, (new_cost, node))
    if found:
        print("")
        print("A* Search With Manhattan Heuristic Values")
        print_path(start, goal)
        print("")
        return came_from, cost_so_far[goal]


def print_path(start, goal):
    return True
    parent = goal.parent
    path = [goal]

    while parent != start:
        path.append(parent)
        parent = parent.parent
        # print(parent)
    path.append(start)
    path.reverse()
    for i in range(len(path) - 1):
        print(path[i], "->", end=" ")
    print(path[-1])


def printMaze(matrix):
    plt.pcolormesh(matrix)
    plt.xticks([])
    plt.yticks([])
    plt.gca().invert_yaxis()
    plt.title("{}x{}".format(rows, cols))


expanded = 0

rows = 2000
cols = 2000
size = rows * cols
grid = [[Vertex(j, i) for i in range(cols)] for j in range(rows)]

printableMaze = [[0 for i in range(cols * 2 - 1)] for j in range(rows * 2 - 1)]

createMaze()
printMaze(printableMaze)

distanceCalculator(grid)
clearVisitedStatus(grid)
start_time = time.time()
#IDDFS(grid[0][0], grid[rows - 1][cols - 1], size ** 2)
print("\nExpanded = ", expanded)
end_time = time.time()
print("\nIDDFS Time Length = ", end_time - start_time)
clearVisitedStatus(grid)

expanded = 0
start_time = time.time()
path, cost = uniform_cost_search(grid[0][0], grid[rows - 1][cols - 1])
print("\nExpanded = ", expanded)
print("cost = ", cost)
end_time = time.time()
print("\nUniform Cost Search Time Length = ", end_time - start_time)
clearVisitedStatus(grid)

expanded = 0
start_time = time.time()
path, cost = a_star_search_with_Manhattan(grid[0][0], grid[rows - 1][cols - 1])
print("\nExpanded = ", expanded)
print("cost", cost)
end_time = time.time()
print("\nA* Search With Manhattan Distance Time Length = ", end_time - start_time)
clearVisitedStatus(grid)

expanded = 0
start_time = time.time()
path, cost = a_star_search_with_Euclidean(grid[0][0], grid[rows - 1][cols - 1])
print("\nExpanded = ", expanded)
print("cost", cost)
end_time = time.time()
print("\nA* Search With Manhattan Distance Time Length = ", end_time - start_time)
clearVisitedStatus(grid)

#plt.show()
