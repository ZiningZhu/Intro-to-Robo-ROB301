import numpy as np
import math
import matplotlib.pyplot as plt

class PathPlanning:
    def __init__(self):
        self.MAP_SIZE = 35
        self.M = np.zeros([35,35])
        self.M[14:30, 27:30] = np.ones([16, 3])
        self.M[27:30, 24:27] = np.ones([3, 3])
        self.M[11:14, 18:22] = np.ones([3,4])
        self.M[20:26, 18:21] = np.ones([6,3])
        self.M[16:19, 13:17] = np.ones([3,4])

        self.M[8:14, 10:12] = np.ones([6,2])
        self.M[12:14, 6:10] = np.ones([2,4])

        for x in range(8, 12):
            for y in range(6, 10):
                if (x+y>16):
                    self.M[x,y]=1
        for x in range(22, 30):
            for y in range(8, 21):
                if (y-13*x/7 < 8-13*22/7):
                    self.M[x,y]=1

        self.dijkstra_res = None
        self.astar_res = None


    def plotMap(self):
        plt.clf()
        plt.close()

        # Wall
        wallsPts = []
        for x in range(self.MAP_SIZE):
            for y in range(self.MAP_SIZE):
                if (self.M[x][y] == 1):
                    wallsPts.append((x,y))
        plt_x = [pt[0] for pt in wallsPts]
        plt_y = [pt[1] for pt in wallsPts]
        plt.plot(plt_x, plt_y, 'ro')

        # Start and End
        plt.plot(0, 0, 'bo')
        plt.plot(34,34,'bo')

        # dijkstra
        if not (self.dijkstra_res == None):
            plt_x = [pt[0] for pt in self.dijkstra_res]
            plt_y = [pt[1] for pt in self.dijkstra_res]
            plt.plot(plt_x, plt_y, 'b*')

        if not (self.astar_res == None):
            plt_x = [pt[0] for pt in self.astar_res]
            plt_y = [pt[1] for pt in self.astar_res]
            plt.plot(plt_x, plt_y, 'b-')

        plt.show()

    def inRange(self, pt):
        return pt[0] >= 0 and pt[0] < self.MAP_SIZE and \
            pt[1] >= 0 and pt[1] < self.MAP_SIZE and \
            self.M[pt] == 0

    def dijkstra_step(self, curr, tgt, neighbours, distance, previous, unvisited):
        if (self.inRange(tgt) and tgt in unvisited):
            if (not tgt in neighbours):
                neighbours.append(tgt)
            tentative_dist = distance[curr] + 1
            if (tentative_dist < distance[tgt]):
                distance[tgt] = tentative_dist
                previous[tgt] = curr

    def dijkstra(self, start, end):
        # input: start and end are lists of len 2
        # returns a list of coordinates
        assert type(start) == type((0,0)) and len(start) == 2
        assert type(end) == type((0,0)) and len(end) == 2
        unvisited = {} # 2x1 list to double
        previous = {}
        distance = {}
        neighbours = [] # queue of 2x1 list
        for x in range(self.MAP_SIZE):
            for y in range(self.MAP_SIZE):
                if not self.inRange((x,y)):
                    continue
                distance[(x,y)] = np.inf
                unvisited[(x,y)] = True
                previous[(x,y)] = None

        distance[start] = 0
        neighbours.append(start)
        while (len(neighbours) > 0):
            mindist = np.inf
            mincoord = None
            for i in range(len(neighbours)):
                if distance[neighbours[i]] < mindist:
                    mincoord = i
            curr = neighbours.pop(i)
            unvisited.pop(curr)

            if (curr == end):
                respath = [end]
                x = end

                while(x != start):
                    
                    respath.insert(0, previous[x])
                    x = previous[x]
                    # assume no cycle specified bt the previous
                self.dijkstra_res = respath
                return

            tgt = (curr[0]-1, curr[1]-1)
            self.dijkstra_step(curr, tgt, neighbours, distance, previous, unvisited)

            tgt = (curr[0], curr[1]-1)
            self.dijkstra_step(curr, tgt, neighbours, distance, previous, unvisited)

            tgt = (curr[0]+1, curr[1]-1)
            self.dijkstra_step(curr, tgt, neighbours, distance, previous, unvisited)

            tgt = (curr[0]-1, curr[1])
            self.dijkstra_step(curr, tgt, neighbours, distance, previous, unvisited)

            tgt = (curr[0]+1, curr[1])
            self.dijkstra_step(curr, tgt, neighbours, distance, previous, unvisited)

            tgt = (curr[0]-1, curr[1]+1)
            self.dijkstra_step(curr, tgt, neighbours, distance, previous, unvisited)

            tgt = (curr[0], curr[1]+1)
            self.dijkstra_step(curr, tgt, neighbours, distance, previous, unvisited)

            tgt = (curr[0]+1, curr[1]+1)
            self.dijkstra_step(curr, tgt, neighbours, distance, previous, unvisited)




        self.dijkstra_res = "None"
        return

    def astar(self, start, end):
        pass


if __name__ == "__main__":
    p = PathPlanning()
    p.dijkstra((0,0), (34,34))
    p.plotMap()
