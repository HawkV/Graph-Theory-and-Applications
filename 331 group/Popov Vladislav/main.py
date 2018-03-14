import svgwrite
import osmium
import csv

# Чтение данных


class Node:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

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

            self.adjacency_matrix = []
            self.adjacency_list = {}
            self.id_to_index = {}

        def node(self, n):
            new_node = Node(n.location.lat, n.location.lon)
            self.nodes[n.id] = new_node

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


handler = OSMHandler()
handler.apply_file("map.xml")

# Заполнение CSV

handler.adjacency_list = {id: set() for id, node in handler.nodes.items() if node.is_not_deletable()}
handler.id_to_index = {item: index for index, item in enumerate(handler.adjacency_list)}

count = len(handler.adjacency_list)

handler.adjacency_matrix = [[0 for i in range(count)] for i in range(count)]

for road in handler.roads:
    for index, id in enumerate(road.nodes_id[:-1]):
        handler.add_adjacent_nodes(id, road.nodes_id[index + 1], road.one_way)

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

# Отрисовка

# Размер изображения в пикселях по обеим осям
size = 3000

# Максимальные и минимальные широта и долгота, для масштабирования
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
    
    
print(min_lon, max_lon, min_lat, max_lat)

lat_scale = (max_lat - min_lat) / size
lon_scale = (max_lon - min_lon) / size

def flip_and_center_points(points_array):
    return [
                (
                    size - (max_lon - point[1]) / lon_scale,
                    (max_lat - point[0]) / lat_scale
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
            (
                handler.nodes[id].latitude,
                handler.nodes[id].longitude
            ) for id in road.nodes_id if handler.nodes[id].is_not_deletable()
        ]
    )

    params = choose_road_params(road.type)
    drawing.add(drawing.polyline(points, stroke = params[0], stroke_width=params[1], fill = "none"))

    [drawing.add(drawing.circle(center=point, r=1, stroke="orange", stroke_width = 1)) for point in points]

drawing.save()
