from  astar import AStar
import math
from mcppIns.MCPPinstance import MCPPInstance
# import drawEnv
from drawEnv import drawPic
import numpy as np

class MCMP_Solver(AStar):

    def __init__(self, ins:MCPPInstance):

        self._mat = ins._mat
        self._row = ins._row
        self._col = ins._col

    def heuristic_cost_estimate(self, n1, n2):
        """computes the 'direct' distance between two (x,y) tuples"""
        (x1, y1) = n1
        (x2, y2) = n2
        return math.hypot(x2 - x1, y2 - y1)

    def distance_between(self, n1, n2):
        """this method always returns 1, as two 'neighbors' are always adajcent"""
        return 1

    def neighbors(self, node):
        """ for a given coordinate in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        x, y = node
        return[(nx, ny) for nx, ny in[(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]if 0 <= nx < self._row and 0 <= ny < self._col and self._mat[nx][ny] == 0]

    def __str__(self):
        return 'mcmp_astar row = '  + str(self._row) + ' col = ' + str(self._col)

class STC_ASTAR(MCMP_Solver):
    def __init__(self,ins:MCPPInstance, graphIndLst = []):
        super(STC_ASTAR,self).__init__(ins)
        self._mat = np.ones((self._row,self._col))
        for grow,gcol in graphIndLst:
            self._mat[grow][gcol] = 0



if __name__ == '__main__':
    print('test mcmp astar')

    ins = MCPPInstance()
    ins.loadCfg('.\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    astar_solver  = MCMP_Solver(ins)
    start = (1, 1)
    goal = (5, 10)
    foundPathInd = list(astar_solver.astar(start,goal))
    print(foundPathInd)
    foundPathInd = []
    # if ins._mat[1][7] == 1:
    #     raise  Exception()
    drawPic(ins = ins, singlePathInd = foundPathInd)










