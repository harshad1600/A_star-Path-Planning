#!/usr/bin/env python
import sys
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid

lst = []
origin_x = 5.05
origin_y = 5
row_cells = 200
col_cells = 200
width_g = 10
height_g = 10
res = 0.05


def isempty(l):
    if len(l) == 0:
        return True
    else:
        return False

def extractpath(djs,start,goal):
    son = djs[goal]

    if son == start:
        path_raw.append(son)
        print(path_raw)
        return path_raw

    else:
        path_raw.append(son)

        extractpath(djs,start,son)
def euclideandist(x,t):
    i = x // col
    j = x % col
    t_x = t // col
    t_y = t % col
    h = np.sqrt((t_y -j)**2 + (t_x - i)**2)
    return h

def inpt():
    x0 = float((input("Enter start locationx: ")))
    y0 = float((input("Enter start locationy: ")))
    x0 = x0 + origin_x
    y0 = y0 + origin_y
    s1 = int((col_cells/width_g)*x0 )
    s2 = int((row_cells/height_g)*y0 )
    s = col_cells*s2 + s1
    x1 = float((input("Enter goal locationx: ")))
    y1 = float((input("Enter goal locationy: ")))
    x1 = x1 + origin_x
    y1 = y1 + origin_y
    t1 = int((col_cells/width_g)*x1 )
    t2 = int((row_cells/height_g)*y1 )
    t = col_cells*t2 + t1


    if s in obst or t in obst:

        print("hi")
        print("Invalid start or goal point")
        return None,None
    else:
        return s,t




class minheap:
    def __init__(self):
        self.d = dict()
    def insert(self,lst):
        i = len(self.d)
        i = i+1
        self.d[i] = lst

        while np.floor(i/2) and self.d[np.floor(i/2)][1] > self.d[i][1]:

            try:

                if self.d[i][1] < self.d[np.floor(i/2)][1]:
                    (self.d[i],self.d[np.floor(i/2)]) = (self.d[np.floor(i/2)] , self.d[i])
                    i = np.floor(i/2)
                #if self.d[2*i+1][1] < self.d[i][1]:
                   # (self.d[2*i+1], self.d[i]) = (self.d[i], self.d[2*i+1])

            except KeyError:
                pass
    def delete_min(self):
        i = len(self.d)
        min = self.d[1]
        if i == 1:
            self.d.pop(i)
        else:
            self.d[1] = self.d.pop(i)
        i = 1
        while 2*i <= len(self.d):

            try:
                if self.d[2*i+1]:

                    if self.d[2*i][1] <= self.d[2*i+1][1] and self.d[2*i][1] < self.d[i][1]:
                        (self.d[2*i], self.d[i]) = (self.d[i], self.d[2*i])
                        i = 2*i

                        continue
                    if self.d[2*i + 1][1] <= self.d[2*i][1] and self.d[2*i+1][1] < self.d[i][1]:
                        (self.d[2*i + 1], self.d[i]) = (self.d[i], self.d[2*i + 1])
                        i = 2*i+1
                        continue
                    else:
                        break
            except KeyError:
                if self.d[2*i][1] < self.d[i][1]:
                    (self.d[2*i], self.d[i]) = (self.d[i], self.d[2*i])
                    i = 2*i
                else:
                    i = 2*i
        return min
class Graph:
    def __init__(self):
        self.vertices = dict()
    def addedge(self,v1,v2,w):
        if v2>=0 and v2<max and v1 not in obst and v2 not in obst:
            if v1 not in self.vertices:
                self.vertices[v1] = []
            #if v2 not in self.vertices:
                #self.vertices[v2] = []
            self.vertices[v1].append([v2,w])
        else:
        	pass
            #self.vertices[edge.v2].append((edge.v1,edge.wght))
    
  

    def astar(self,s,t):
        global path_raw
        path_raw = []
        open = minheap()
        closed = set()
        pred = dict()
        open.insert([s,0])
        if s == None or t == None:

            return 0

        while not isempty((open.d)):
            u = open.delete_min()
            if u[0] == t:
            	print("path_found")
                return extractpath(pred,s,t)
            for v in self.vertices[u[0]]:
                if v[0] in closed:

                    continue
                uvcost = v[1]
                h_cost = euclideandist(v[0], t)

                for keys in open.d:

                    if v[0] == open.d[keys][0]:

                        if u[1] + uvcost + h_cost < open.d[keys][1]:
                            open.d[keys][1] = u[1] + uvcost + h_cost
                            pred[v[0]] = u[0]
                            break

                        else:
                            break
                else:

                    open.insert([v[0],u[1] + uvcost + h_cost])
                    pred[v[0]] = u[0]

            closed.add(u[0])
        return 0

def map_call(msg):
    for i in range(40000):
        lst.append(msg.data[i])
    
	
def map_data():
    rospy.init_node('node_astar',disable_signals = False)
    rospy.Subscriber('/map', OccupancyGrid, map_call)

    rospy.sleep(1)




def graph_fn():

    global g 
    g = Graph()
    global row
    row = row_cells
    global col
    col = col_cells
    global max
    max = row * col
    k = 0
    global obst
    obst = []
    for ele in lst:
        if ele == -1 or ele == 100:
            obst.append(k)
        else:
            pass
        k = k+1
    for i in range(row):
        for j in range(col):
            if j+i*col % col == 0 or (j+i*col % col) == col - 1:
                continue
            else:
                g.addedge(j+i*col,j+i*col+1,1)
                g.addedge(j+i*col,j+i*col-1, 1)
                g.addedge(j+i*col,j+(i+1)*col, 1)
                g.addedge(j+i*col,j+(i-1)*col, 1)
                g.addedge(j+i*col,j+1+(i+1)*col, 1.41)
                g.addedge(j+i*col,j-1+(i+1)*col, 1.41)
                g.addedge(j+i*col,j+1+(i-1)*col, 1.41)
                g.addedge(j+i*col,j-1+(i-1)*col, 1.41)
                
    for i in range(0,max,col):
        g.addedge(i,i+1,1)
        g.addedge(i,i+col,1)
        g.addedge(i,i-col,1)
        g.addedge(i, i + col + 1, 1.41)
        g.addedge(i, i - col + 1, 1.41)
    for i in range(col - 1,max,col):
        g.addedge(i, i - 1, 1)
        g.addedge(i, i + col, 1)
        g.addedge(i, i - col, 1)
        g.addedge(i, i + col - 1, 1.41)
        g.addedge(i, i - col - 1, 1.41)
    s,t = inpt()
    print(s,t)
    p = g.astar(s,t)
    if p == 0:
        print("path not found!")
    



def display_path():
    pub = rospy.Publisher("chatter",Path,queue_size = 1)
    rospy.init_node("node_astar")
    j = 0

    path = Path()
    path.header.seq = j
    path.header.stamp = rospy.get_rostime()
    path.header.frame_id = "map"
    j = 0
    for i in path_raw:
        obj = PoseStamped()
        obj.header.seq = j
        obj.header.stamp = rospy.get_rostime()
        obj.header.frame_id = "map"
        obj.pose.position.x = -origin_x + (res)*(i%col_cells)
        obj.pose.position.y = -origin_y + (res)*(i//col_cells)
        obj.pose.position.z = 0
        obj.pose.orientation.w = 1
        path.poses.append(obj)
        j = j+1
    while not rospy.is_shutdown():
        pub.publish(path)
        path_raw.reverse()
    rospy.sleep(1)
		
if __name__ == '__main__':
    try:
    	map_data()
    	graph_fn()
        display_path()
    except rospy.ROSInterruptException:
        pass
				
		

