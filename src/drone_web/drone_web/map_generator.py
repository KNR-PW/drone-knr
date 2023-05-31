import numpy as np
import cv2
import math
import smallestenclosingcircle as sec

#according to the rules the final map should include: 

#- trees divided into categories
#- time, place and kind of shots fired
#- time of mission 
#- drone's route

class Tree():
    def __init__(self, north, east, color: str, time: str = None):
        self.location_local = (north,east)
        self.time = time
        self.map = None #geometric information about map in dronekit local frame

        if color == 'gold': #BGR (OpenCV format)
            self.color = (65, 159, 212)
        elif color == 'beige':
            self.color = (227, 246, 249)
        elif color == 'brown':
            self.color = (76, 107, 147)

class MapInfo():
    def __init__(self, corners, home, time):
        self.time = time
        self.corners = corners
        self.home = home

class MapGenerator():
    def __init__(self, trees, map_info: MapInfo, route=None):
        self.map_image = np.zeros((700, 850, 3), dtype=np.uint8)

        self.route = route
        self.trees = trees
        self.map_info = map_info
        self.__parse_map()

    def __parse_map(self):
        points = [self.map_info.home]
        for corners in self.map_info.corners:
            points.append(corners)
        x,y,r = sec.make_circle(points)
        self.map.center = (x,y)
        self.map.radius = r
    
    def __to_map_coords(self, point):
        image_x = (point[0]-self.map.center[0]) / self.map.radius * (self.map_image.shape[0]-50) + self.map_image.shape[0]//2
        image_y = (point[1]-self.map.center[1]) / self.map.radius * (self.map_image.shape[1]-200) + ((self.map_image.shape[0]-150)//2)
        return (image_x,image_y)

    def add_location_markings(self):
        for corner in self.map_info.corners:
            location = self.__to_map_coords(corner)
            cv2.circle(self.map_image, location, radius=6, color=(0, 0, 255), thickness=-1)
        
        location = self.__to_map_coords(self.map_info.home)
        cv2.circle(self.map_image, location, radius=8, color=(255, 0, 0), thickness=-1)
        size, _ = cv2.getTextSize('Start', cv2.FONT_HERSHEY_SIMPLEX, 2, 1)
        cv2.putText(self.map_image, 'Start', (location[0] - (size[0]//2), location[1]+10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 1, cv2.LINE_AA)

    def add_legend(self):
        pass

    def add_trees(self):
        for tree in self.trees:
            location = self.__to_map_coords(tree.location_local)
            cv2.circle(self.map_image, location, radius=4, color=(65, 159, 212), thickness=-1)
    
            # Display the date below the circle
            if tree.time != None:
                text_size, _ = cv2.getTextSize(tree.time, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                text_x = location[0] - (text_size[0] // 2)
                text_y = location[1] + 25
                cv2.putText(self.map_image, tree.time, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    
    def add_route(self):
        for i in range(len(self.route) - 1):
            cv2.line(self.map_image, self.__to_map_coords(route[i]), self.__to_map_coords(route[i + 1]), (255,255,255), 1)

    def add_mission_time(self):
        date_string = f"Mission start time: {self.map_info.time}"
        size, _ = cv2.getTextSize(date_string, cv2.FONT_HERSHEY_SIMPLEX, 2, 1)
        cv2.putText(self.map_image, date_string, (self.map_image.shape[0]-size[0],self.map_image.shape[1]-size[1]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 1, cv2.LINE_AA)
    
    def save_map(self):
        cv2.imwrite('map.png',self.map_image)

    def show_map(self):
        cv2.imshow('map',self.map_image)

#EXAMPLES of how the lists should look like:

corners = [(10,10),(150,15),(45,100),(150,100)]
route=[(70, 490), (210, 630),(490, 490),(630, 630), (210, 350), (70, 70)] 
home = (2,8)
map_data = MapInfo(corners,home,'12:31')

brown = Tree(75,75,'brown','13')
beige = Tree(40,80,'beige','27')
gold = Tree(100,20,'gold','50')

trees = [beige, brown, gold]

generator = MapGenerator(trees,map_data)