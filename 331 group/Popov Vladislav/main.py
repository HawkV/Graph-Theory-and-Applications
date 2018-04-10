import svgwrite
import osmium
import csv
import math
import time
import random

from collections import deque
from heapq import heappush, heappop
from functools import reduce

# Чтение данных
class Node:
    def __init__(self, latitude, longitude, id=-1):
        self.latitude = latitude
        self.longitude = longitude
        self.id = id

        self.is_end_node = False
        self.is_crossroad = False
        self.in_road = False

        self.is_drawn = False

    def is_not_deletable(self):
        return self.is_crossroad or self.is_end_node


class Road:
    def __init__(self, road_type):
        self.type = road_type
        self.one_way = False
        self.nodes_id = []


class OSMHandler(osmium.SimpleHandler):
        def add_adjacent_nodes(self, first, second, one_way):
            if not self.nodes[first].is_not_deletable() or not self.nodes[second].is_not_deletable():
                return

            first_index, second_index = self.id_to_index[first], self.id_to_index[second]

            self.adjacency_list[first].add(second)
            self.adjacency_matrix[first_index][second_index] = 1

            if not one_way:
                self.adjacency_list[second].add(first)
                self.adjacency_matrix[second_index][first_index] = 1

        def __init__(self):
            osmium.SimpleHandler.__init__(self)

            self.nodes = {}
            self.roads = []
            self.hospitals = []

            self.adjacency_matrix = []
            self.adjacency_list = {}
            self.id_to_index = {}

        def node(self, n):
            new_node = Node(n.location.lat, n.location.lon, n.id)
            self.nodes[n.id] = new_node

            if n.tags.get('amenity') == 'hospital' and len(self.hospitals) < 10:
                self.hospitals.append(new_node);

        def way(self, w):
            road_type = w.tags.get('highway')

            if road_type in ["motorway", "trunk", "primary", "secondary", "tertiary",
                             "unclassified", "residential", "living_street"]:
                
                new_road = Road(road_type)
                new_road.nodes_id = [way_node.ref for way_node in w.nodes]
                new_road.one_way = w.tags.get('oneway') == 'yes'

                for id in new_road.nodes_id:
                    self.nodes[id].is_crossroad = self.nodes[id].in_road
                    self.nodes[id].in_road = True

                self.nodes[new_road.nodes_id[0]].is_end_node = True
                self.nodes[new_road.nodes_id[-1]].is_end_node = True

                self.roads.append(new_road)

        def closest_node(self, target):
            closest = None
            dist = -1

            for id, n in self.nodes.items():
                if n.is_not_deletable():
                    temp = euclidean(target, n)

                    if not closest or dist > temp:
                        closest = n
                        dist = temp

            return closest


# Вызов чтения данных

handler = OSMHandler()
handler.apply_file("map.xml")

# Заполнение CSV

handler.adjacency_list = {id: set() for id, node in handler.nodes.items() if node.is_not_deletable()}
handler.id_to_index = {item: index for index, item in enumerate(handler.adjacency_list)}

count = len(handler.adjacency_list)

handler.adjacency_matrix = [[0 for i in range(count)] for i in range(count)]

for road in handler.roads:
    iterator = iter(road.nodes_id)

    for prev_id in iterator:
        if handler.nodes[prev_id].is_not_deletable():
            break

    for id in iterator:
        if handler.nodes[id].is_not_deletable():
            handler.add_adjacent_nodes(prev_id, id, road.one_way)

            prev_id = id

id_list = []

with open('list.csv', 'w+') as file:
    writer = csv.writer(file, delimiter=',')

    for id, row in handler.adjacency_list.items():
        id_list.append(id)
        writer.writerow([id] + list(row))

with open('matrix.csv', 'w+') as file:
    writer = csv.writer(file, delimiter=',')

    writer.writerow([""] + id_list)

    for index, row in enumerate(handler.adjacency_matrix):
        writer.writerow([id_list[index]] + row)


# Максимальные и минимальные широта и долгота, для масштабирования и ввода данных
min_lon = handler.nodes[id_list[0]].longitude
max_lon = handler.nodes[id_list[0]].longitude
min_lat = handler.nodes[id_list[0]].latitude
max_lat = handler.nodes[id_list[0]].latitude

for id in id_list:
    lon, lat = handler.nodes[id].longitude, handler.nodes[id].latitude
    
    if min_lon > lon:
        min_lon = lon
    elif max_lon < lon:
        max_lon = lon
    
    if min_lat > lat:
        min_lat = lat
    elif max_lat < lat:
        max_lat = lat

# Отрисовка

size = 3000  # Размер изображения в пикселях по обеим осям

lat_scale = (max_lat - min_lat) / size
lon_scale = (max_lon - min_lon) / size


def flip_and_center_points(points_array):
    return [
                (
                    size - (max_lon - point.longitude) / lon_scale,
                    (max_lat - point.latitude) / lat_scale
                ) for point in points_array
           ]


def choose_road_params(road_type):
    if road_type == "primary":
        return svgwrite.rgb(0, 0, 0, '%'), 3
    elif road_type == "secondary":
        return svgwrite.rgb(50, 50, 300, '%'), 2
    elif road_type == "tertiary":
        return svgwrite.rgb(50, 300, 50, '%'), 2

    return svgwrite.rgb(55, 55, 55, '%'), 1

drawing = svgwrite.Drawing('city.svg', size=(str(size) + 'px', str(size) + 'px'))


for road in handler.roads:
    points = flip_and_center_points(
        [
            (handler.nodes[id]) for id in road.nodes_id if handler.nodes[id].is_not_deletable()
        ]
    )

    params = choose_road_params(road.type)
    drawing.add(drawing.polyline(points, stroke=params[0], stroke_width=params[1], fill="none"))

    [drawing.add(drawing.circle(center=point, r=1, stroke="orange", stroke_width=1)) for point in points]

drawing.save()

# ------ Нахождение кратчайших путей ------

# Методы вычисления расстояния
def euclidean(first, second):
    return ((first.latitude - second.latitude) ** 2 + (first.longitude - second.longitude) ** 2) ** (1/2)

def manhattan(first, second):
    return abs(first.latitude - second.latitude) + abs(first.longitude - second.longitude)

def chebyshev(first, second):
    return max(abs(first.latitude - second.latitude), abs(first.longitude - second.longitude))

# Алгоритмы поиска пути
def dijkstra(nodes, adjacency_list, start):
    dist = {key: None for key in nodes.keys()}
    dist[start.id] = 0

    prev = {}

    visited = set()

    queue = []

    heappush(queue, (0, start.id))

    while queue:
        tuple = heappop(queue)

        x = tuple[1]

        if x in visited:
            continue

        visited.add(x)

        if not adjacency_list.get(x):
            continue

        for successor in adjacency_list[x]:
            new_dist = dist[x] + euclidean(nodes[x], nodes[successor])

            if dist[successor] is None or new_dist < dist[successor]:
                dist[successor] = new_dist
                prev[successor] = x

                heappush(queue, (new_dist, successor))

    return dist, prev

def levit(nodes, adjacency_list, start):
    dist = {key: None for key in nodes.keys()}
    dist[start.id] = 0

    prev = {}

    queue = deque([start.id])

    id = {key: 0 for key in nodes.keys()}

    while queue:
        v = queue.popleft()

        id[v] = 1

        if not adjacency_list.get(v):
            continue

        for successor in adjacency_list[v]:
            new_dist = dist[v] + euclidean(nodes[v], nodes[successor])

            if dist[successor] is None or new_dist < dist[successor]:
                dist[successor] = new_dist

                if id[successor] == 0:
                    queue.append(successor)
                elif id[successor] == 1:
                    queue.appendleft(successor)

                prev[successor] = v
                id[successor] = 1

    return dist, prev

def a_star(nodes, adjacency_list, start, end, heuristic='euclidean'):
    dist_function = euclidean

    if heuristic == 'manhattan':
        dist_function = manhattan
    elif heuristic == 'chebyshev':
        dist_function = chebyshev

    open_list = []
    closed_list = []

    g = 0
    f = 0 + dist_function(nodes[start.id], nodes[end.id])

    heappush(open_list, (f, g, [start.id]))

    while open_list:
        tuple = heappop(open_list)

        p = tuple[-1]
        g = tuple[1]
        x = p[-1]

        if x in closed_list:
            continue

        if x == end.id:
            return f, p

        closed_list.append(x)

        if not adjacency_list.get(x):
            continue

        for successor in adjacency_list[x]:
            ng = g + euclidean(nodes[successor], nodes[x])
            nf = ng + dist_function(nodes[successor], nodes[end.id])

            new_path = p[:]
            new_path.append(successor)

            heappush(open_list, (nf, ng, new_path))

    return {key: 100000 for key in nodes.keys()}, {}

# Ввод данных
position_longitude, position_latitude = None, None

while not position_longitude or not min_lon <= position_longitude <= max_lon:
    print("Введите долготу из промежутка: ", min_lon, max_lon)
    position_longitude = float(input())

while not position_latitude or not min_lat <= position_latitude <= max_lat:
    print("Введите широту из промежутка: ", min_lat, max_lat)
    position_latitude = float(input())

position = handler.closest_node(Node(position_latitude, position_longitude))

# Ближайшие узлы дорог к больницам
closest_nodes = [handler.closest_node(node) for node in handler.hospitals]
points = flip_and_center_points(closest_nodes)

# Применение алгоритмов
filtered_nodes = {id: node for id, node in handler.nodes.items() if node.is_not_deletable}

dijkstra_dist, dijkstra_prev = dijkstra(filtered_nodes, handler.adjacency_list, position)
levit_dist, levit_prev = levit(filtered_nodes, handler.adjacency_list, position)
a_star_prev = {hospital: a_star(filtered_nodes, handler.adjacency_list, position, hospital, 'chebyshev') for hospital in closest_nodes}

# Алгоритмы отрисовки путей
def draw_path(path, color, width):
    points = flip_and_center_points([handler.nodes[id] for id in path])

    drawing.add(drawing.polyline(points, stroke=color, stroke_width=width, fill="none"))

def reconstruct_path(end_point, prev_nodes):
        path = [end_point.id]

        prev = prev_nodes.get(end_point.id)

        while prev:
            path.append(prev)
            prev = prev_nodes.get(prev)

        return path

def draw_paths(prev_nodes, color, width):
    for node in closest_nodes:
        draw_path(reconstruct_path(node, prev_nodes), color, width)

draw_paths(dijkstra_prev, "red", 5)

# draw_paths(levit_prev, "red", 4)
# [draw_path(pair[1], "blue", 2) for node, pair in a_star_prev.items()]

closest = None
min_distance = -1

for hospital in closest_nodes:
   if not closest or min_distance > dijkstra_dist[hospital.id]:
       closest = hospital.id
       min_distance = dijkstra_dist[hospital.id]

draw_path(reconstruct_path(handler.nodes[closest], dijkstra_prev), "blue", 3)

[drawing.add(drawing.circle(center=point, r=1, stroke="green", stroke_width=8)) for point in points]
drawing.add(drawing.circle(center=flip_and_center_points([position])[0], r=1, stroke="orange", stroke_width=8))
drawing.add(drawing.circle(center=flip_and_center_points([handler.nodes[closest]])[0], r=1, stroke="blue", stroke_width=8))

drawing.save()

def degrees_to_kilometers(first_id, second_id):
    middle = (handler.nodes[first_id].latitude + handler.nodes[second_id].latitude)/2.0

    # Примерное количество метров в 1 градусе широты
    meters_in_lat = 111132.954 - 559.822 * math.cos(2 * middle) + 1.175 * math.cos(4 * middle)

    # Примерное количество метров в 1 градусе долготы
    meters_in_lon = 111132.954 * math.cos(middle)

    dist = ((abs(handler.nodes[first_id].latitude - handler.nodes[second_id].latitude) * meters_in_lat)**2 +
            (abs(handler.nodes[first_id].longitude - handler.nodes[second_id].longitude) * meters_in_lon)**2)**0.5

    return dist/1000

def get_path_time_length(path):
    length = 0

    for index, node in enumerate(path):
        if index != len(path) - 1:
            length += degrees_to_kilometers(node, path[index + 1])

    return length/40

starting_points = set()

while len(starting_points) != 100:
    starting_points.add(random.choice(list(handler.adjacency_list.keys())))

test = False

if test:
    sum = lambda x, y: x + y

    # dijkstra
    t = time.clock()

    timmmme = 0
    global_counter = 0

    for position in starting_points:
        dist, prev = dijkstra(filtered_nodes, handler.adjacency_list, handler.nodes[position])
        delta = reduce(sum, [get_path_time_length(reconstruct_path(hospital, prev)) for hospital in closest_nodes]) / len(closest_nodes)

        if delta:
            global_counter += 1
            timmmme += delta

    print('Среднее время работы алгоритма Дейкстры на каждой из {} точек {} c'.format(len(starting_points), (time.clock() - t)/len(starting_points)))
    print('Среднее время проезда к одной больнице = {} м'.format(60 * timmmme/global_counter))

    dijkstra_timmmme = 60 * timmmme/global_counter

    # levit
    t = time.clock()

    timmmme = 0
    global_counter = 0

    for position in starting_points:
        dist, prev = levit(filtered_nodes, handler.adjacency_list, handler.nodes[position])
        delta = reduce(sum, [get_path_time_length(reconstruct_path(hospital, prev)) for hospital in closest_nodes]) / len(closest_nodes)

        if delta:
            global_counter += 1
            timmmme += delta

    print('Среднее время работы алгоритма Левита на каждой из {} точек {} c'.format(len(starting_points), (time.clock() - t)/len(starting_points)))
    print('Среднее время проезда к одной больнице = {} м'.format(60 * timmmme/global_counter))

    # A*

    print('', 'Проверка алгоритма А*', '')

    for heuristic in ["euclidean", "chebyshev", "manhattan"]:
        t = time.clock()

        timmmme = 0
        global_counter = 0

        for position in starting_points:
            test = {hospital: a_star(filtered_nodes, handler.adjacency_list, handler.nodes[position], hospital, heuristic) for hospital in closest_nodes}

            counter = 0
            temp = 0

            for key, item in test.items():
                if item[1]:
                    counter += 1
                    temp += get_path_time_length(item[1])

            if counter:
                temp /= counter
                global_counter += 1
                timmmme += temp

        timmmme = timmmme * 60 / global_counter

        print('Среднее время работы алгоритма A* на каждой из {} точек = {} c'.format(len(starting_points), (time.clock() - t)/len(starting_points)))
        print('Среднее время проезда к одной больнице = {} м, эвристика: {}'.format(timmmme, heuristic))
        print('Отношение времени проезда в А* к времени проезда в алгоритме Дейкстры: {}'.format(timmmme/dijkstra_timmmme))
