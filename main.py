import matplotlib.pyplot as plt
from shapely import unary_union, distance
from shapely.geometry import Point, LineString, Polygon, MultiPolygon
from shapely.plotting import plot_points, plot_line, plot_polygon
from queue import PriorityQueue
from numpy import linspace
from random import choice, random

obstacle_coords = [
    [(0.8, 5.5), (3, 4.5), (4, 8), (3, 9), (1.5, 7.8)],
    [(4, 4.8), (4.75, 7.5), (5.5, 4.8)],
    [(5.5, 9), (7.4, 9.3), (8, 8.5), (6, 7)],
    [(7.7, 4.5), (9.5, 4.5), (9.5, 7.5), (7, 7)],
    [(10, 8.5), (11.5, 9.5), (12.5, 6.5), (11.5, 4.5), (10.5, 5.5)],
    [(6.8, 2), (6.5, 4.8), (8, 3)],
    [(9, 1), (11, 2), (11.5, 3), (10, 4), (8.5, 3.5), (8.5, 2)],
    [(1.5, 1.5), (6, 1.5), (6, 4), (1.5, 4)]
]

all_obstacle_points = [Point(point) for coords in obstacle_coords for point in coords]
obstacles = MultiPolygon([Polygon(coords) for coords in obstacle_coords])

def get_valid_successors(current_point, all_points, obstacles):
    accessible = set()
    all_polygons = unary_union(obstacles)

    for next_state in all_points:
        if next_state == current_point:
            continue
        line = LineString([current_point, next_state])
        if not line.crosses(all_polygons) and not line.within(all_polygons):
            accessible.add(next_state)
    
    return accessible

def heuristic(point, goal):
    return distance(Point(point), Point(goal))

def find_goal(starting_point, goal, all_points, algorithm):
    open_set = PriorityQueue()
    open_set.put((0, random(), starting_point))
    
    g_score = {point: float('inf') for point in all_points}
    g_score[starting_point] = 0
    f_score = {point: float('inf') for point in all_points}
    f_score[starting_point] = heuristic(starting_point.coords[0], goal.coords[0])
    
    came_from = {}
    visited = set()

    while not open_set.empty():
        _, _, current = open_set.get()

        if current.equals(goal):
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))

        visited.add(current)
        for neighbor in get_valid_successors(current, all_points, obstacles):
            if neighbor in visited:
                continue

            h_cost = heuristic(neighbor, goal) if algorithm == "A_star" else 0 # implement heuristic function if algorithm is A*, else it's identical to BFS
            tentative_g = g_score[current] + h_cost

            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + h_cost
                open_set.put((f_score[neighbor], random(), neighbor))

    return None

def generate_example(start, goal, algorithm):
    all_points = all_obstacle_points + [start, goal]

    plot_polygon(obstacles)
    plot_points(start, color='green', **{'label': 'Start'})
    plot_points(goal, color='purple', **{'label': 'Goal'})

    path = find_goal(start, goal, all_points, algorithm)
    if not path:
        print("Unable to find path")
    else:
        for i in range(len(path) - 1):
            current, next = path[i], path[i+1]
            plot_line(LineString([current, next]), color='red', add_points=False)
            if i != 0:
                plot_points(current, color='red')

    plt.legend()
    plt.draw()

grid_parameters = {
    'start': Point(0.5, 2.5),
    'goal': Point(12.5, 9),
    'algorithm': 'A_star',
    'should_draw': True
}

def on_key(event):
    if event.key.lower() == 'g':
        possible_x = linspace(start=0, stop=12, num=int((12 / 0.1) + 1))
        possible_y = linspace(start=0, stop=10, num=int((10 / 0.1) + 1))
        while(True):
            start = Point(choice(possible_x), choice(possible_y))
            goal = Point(choice(possible_x), choice(possible_y))
            if (not start.dwithin(obstacles, 0.5) and
                not goal.dwithin(obstacles, 0.5) and
                LineString([start, goal]).crosses(obstacles) and
                distance(start, goal) >= 5):
                break
        grid_parameters['start'] = start
        grid_parameters['goal'] = goal
        grid_parameters['should_draw'] = True
    
    if event.key.lower() == 'h':
        algorithm = 'BFS' if grid_parameters['algorithm'] == 'A_star' else 'A_star'
        grid_parameters['algorithm'] = algorithm
        print(f'Switched to {algorithm} algorithm')
        grid_parameters['should_draw'] = True

def main():
    prompt = '''
    Solution of exercise 3.15 in the AIMA book.

    The program generates starting and goal points and finds approximately the best path between them using the selected graph traversal algorithm

    User interface:
    Press 'G' to generate a new pair of start/goal points (first pair is set)
    Press 'H' to switch between informed search method (A*, default) and uninformed (BFS)
    (Note that BFS algorithm may still randomly generate the same path as A*)
    '''
    print(prompt)

    fig = plt.figure()
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.ion()
    plt.show()

    while(True):
        if grid_parameters['should_draw']:
            plt.clf()
            generate_example(grid_parameters['start'], grid_parameters['goal'], grid_parameters['algorithm'])
            grid_parameters['should_draw'] = False
        plt.pause(0.1)

main()