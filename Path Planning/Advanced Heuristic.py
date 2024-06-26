import numpy as np


class Intersection:
    def __init__(self, position, name):
        self.position = position
        self.connections = []
        self.name = name
        self.arrived_by = None


# S,U,L,R Costs     - Optimal Arrived by
# S,U,L,R = 0       - 4
# S = 10            - 7
# S = 10, L = 100   - 4

# Rough Intersection Drawing.

#    5---------7
#    |         |
#    |         |
#    |         |
#    4         |
#    |         |
#    |         |
#    |         |
#    3         |
#    |         |
#    |         |
#    |         |
#    2-----1---6
#          |
#          |
#          |
#          0


# Create intersections
intersection0 = Intersection([0, 0], '0')
intersection1 = Intersection([3, 0], '1')
intersection2 = Intersection([3, -5], '2')
intersection3 = Intersection([8, -5], '3')
intersection4 = Intersection([13, -5], '4')
intersection5 = Intersection([18, -5], '5')
intersection6 = Intersection([3, 3], '6')
intersection7 = Intersection([18, 3], '7')

# Create Connections
intersection0.connections += [intersection1]
intersection1.connections += [intersection0, intersection2, intersection6]
intersection2.connections += [intersection1, intersection3]
intersection3.connections += [intersection2, intersection4]
intersection4.connections += [intersection3, intersection5]
intersection5.connections += [intersection4, intersection7]
intersection6.connections += [intersection1, intersection7]
intersection7.connections += [intersection6, intersection5]


class AStarSearch:
    def __init__(self, start, goal, direction):
        self.start = start
        self.goal = goal
        self.not_explored = {}
        self.direction = direction
        self.explored = {}
        self.location = start
        self.length_depth = 0

    def get_possible_moves(self):
        # Iterate through connections and add applicable to not_explored.
        for connection in self.location.connections:
            if connection in self.not_explored:
                h_current = self.not_explored[connection]
                distance = self.road_distance(self.location, connection)
                h_new = self.length_depth + distance
                h_new += self.intersection_cost(connection)
                h_new += self.heuristic(connection)
                if h_new < h_current:
                    self.not_explored[connection] = h_new
                    connection.arrived_by = self.location

            if connection not in self.not_explored and connection not in self.explored:
                # self.length_depth
                # self.road_distance(self.location, connection)
                # self.intersection_cost(connection)
                # self.heuristic(connection)
                distance = self.road_distance(self.location, connection)
                self.not_explored[connection] = self.length_depth + distance
                self.not_explored[connection] += self.intersection_cost(connection)
                self.not_explored[connection] += self.heuristic(connection)

                # Save direction arrived by to allow to track turn direction.
                connection.arrived_by = self.location

        # Add current position to explored set value to
        # self.length_depth + self.heuristic(self.location)
        self.explored[self.location] = self.length_depth + self.heuristic(self.location)

    def goal_found(self):
        if self.goal in self.not_explored:
            return True
        return False

    def explore_next_move(self):
        # Determine next move to explore.
        sorted_not_explored = sorted(
            self.not_explored,
            key=self.not_explored.get,
            reverse=False)

        # Determine the pos and depth of next move.
        self.direction_update(sorted_not_explored[0])
        self.location = sorted_not_explored[0]
        self.length_depth = sorted_not_explored.pop(self.location) - self.heuristic(self.location)
        return True

    def heuristic(self, connection):
        distance = (self.goal.position[0] - connection.position[0])**2
        distance += (self.goal.position[1] - connection.position[1])**2
        return round(np.sqrt(distance), 1)

    def road_distance(self, int1, int2):
        distance = abs(int1.position[0] - int2.position[0])
        distance += abs(int1.position[1] - int2.position[1])
        return distance

    def direction_update(self, new_location):
        y = new_location.position[0] - new_location.arrived_by.position[0]
        x = new_location.position[1] - new_location.arrived_by.position[1]
        a = np.array([y, x])
        a[a < -0.1] = -1
        a[a > 0.1] = 1
        self.direction = [a[0], a[1]]

    def intersection_cost(self, connection):
        # Deep copy
        dir = list(self.direction)
        y = connection.position[0] - self.location.position[0]
        x = connection.position[1] - self.location.position[1]
        a = np.array([y, x])
        dot_product = a * dir

        if sum(dot_product) > 0:
            # Straight
            return 10
        if sum(dot_product) < 0:
            # U Turn
            return 0
        # Rotate until oriented upwards.
        while dir != [1, 0]:
            dir = [-dir[1], dir[0]]
            a = [-a[1], a[0]]
        if a[1] < 0:
            # Left
            return 100
        if a[1] > 0:
            # Right
            return 0

    def print_path(self):
        print('Goal arrived by ' + self.goal.arrived_by.name)


astar = AStarSearch(intersection0, intersection5, [1, 0])

while True:
    astar.get_possible_moves()
    if astar.goal_found():
        break
    astar.explore_next_move()

try:
    astar.print_path()
except:
    print('Cant print path')

print('Explored')
for i in astar.explored:
    print(i.name + ": " + str(astar.explored[i]))
print('Not Explored')
for i in astar.not_explored:
    print(i.name + ": " + str(astar.not_explored[i]))
