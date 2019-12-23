import random
import numpy as np
import networkx as nx
import readcfg as rd

# import drawPath
import matplotlib.pyplot as plt

def getNeighbor(envMat,lst = (0,0),row =20, col =20):
    resLst = []
    #left
    lstLeft = (lst[0]-1,lst[1])
    if(lstLeft[0]>=0):
        if(envMat[lstLeft[0]][lstLeft[1]]==1):
            resLst.append(lstLeft)
    #right
    lstRight = (lst[0]+1,lst[1])
    if(lstRight[0]<row):
        if(envMat[lstRight[0]][lstRight[1]]==1):
            resLst.append(lstRight)
    #top
    lstTop = (lst[0],lst[1]+1)
    if(lstTop[1]<col):
        if(envMat[lstTop[0]][lstTop[1]]==1):
            resLst.append(lstTop)
    #bottom
    lstBottom = (lst[0],lst[1]-1)
    if(lstBottom[1]>=0):
        if(envMat[lstBottom[0]][lstBottom[1]]==1):
            resLst.append(lstBottom)
    return resLst

class MCPPInstance(object):

    def __init__(self):
        pass

    def loadCfg(self, fileName: str):
        # print(fileName)
        # self._fileName = fileName
        read_cfg = rd.Read_Cfg(fileName)
        self._row = read_cfg.getSingleVal('row',dtype='int')
        self._col = read_cfg.getSingleVal('col',dtype='int')
        self._robRowLst = []
        self._robColLst = []

        read_cfg.get('robRow',self._robRowLst, dtype= 'int')
        read_cfg.get('robCol',self._robColLst, dtype= 'int')

        self._robPosLst = []

        for robID in range(len(self._robRowLst)):
            self._robPosLst.append((self._robRowLst[robID],self._robColLst[robID]))



        self._robNum = len(self._robColLst)
        self._robReachRowLst = []
        self._robReachColLst = []

        read_cfg.get('robReachRowLst',self._robReachRowLst, dtype= 'int')
        read_cfg.get('robReachColLst',self._robReachColLst, dtype= 'int')


        self._robUnReachRowLst = []
        self._robUnReachColLst = []

        read_cfg.get('robUnReachRowLst',self._robUnReachRowLst, dtype= 'int')
        read_cfg.get('robUnReachColLst',self._robUnReachColLst, dtype= 'int')

        self._mat = np.zeros([self._row,self._col])

        for i in range(len(self._robUnReachColLst)):
            self._mat[self._robUnReachRowLst[i]][self._robUnReachColLst[i]] = 1



    def setPara(self,row,col,obstacleLst,robPosLst):
        self._row = row
        self._col = col
        self._obstacleLst  = obstacleLst
        self._robNum = len(robPosLst)
        self._mat = np.zeros([self._row,self._col])
        # print(self._mat)
        ind = 0
        for rowInd in range(row):
            for colInd in range(col):
                self._mat[rowInd][colInd] = self._obstacleLst[ind]
                ind  = ind + 1
        # print(self._mat)

        self._robPosLst = robPosLst
        self._robRowLst = [x[0] for x in self._robPosLst]
        self._robColLst = [x[1] for x in self._robPosLst]
        sPntx = []
        sPnty = []
        tPntx = []
        tPnty = []
        G = nx.Graph()
        for i in range(self._row):
            for j in range(self._col):
                centre = (i, j)
                G.add_node((i, j))
                if (self._mat[i][j] == 1):
                    continue
                neiLst = getNeighbor(envMat=self._mat, lst=centre, row=row, col=col)
                for unit in neiLst:
                    sPntx.append(i + 0.5)
                    sPnty.append(j + 0.5)
                    tPntx.append(unit[0] + 0.5)
                    tPnty.append(unit[1] + 0.5)
                    G.add_edge(centre, unit)
        # component2 = nx.connected_components(G)
        # print(len(component2))
        # print('G = ',G.number_of_nodes())
        # print('num_edge',G.number_of_edges())

        # component = list(nx.connected_components(G))
        # # print('len comp',len(component))
        # # print(component)
        #
        # # if len(component) ==
        #
        # reachComponentLst = []
        # unReachCompLst = [n for n in range(len(component))]
        # print(unReachCompLst)
        # for i in range(self._robNum):
        #     for j in range(len(component)):
        #         if (reachComponentLst.count(j) != 1):
        #             if ((self._robRowLst[i], self._robColLst[i]) in component[j]):
        #                 reachComponentLst.append(j)
        #                 unReachCompLst.remove(j)
        #
        # # print('len1 = ',len(reachComponentLst))
        #
        # self._robReachRowLst = []
        # self._robReachColLst = []
        # for unit in reachComponentLst:
        #     for gridUnit in component[unit]:
        #         self._robReachRowLst.append(gridUnit[0])
        #         self._robReachColLst.append(gridUnit[1])
        #
        # print('lenMat = ',len(self._mat[0]))
        #
        # '''
        # xuyao genggai
        # '''
        # self._robReachRowLst = []
        # self._robReachColLst = []
        # for rowInd in range(len(self._mat)):
        #     for colInd in range(len(self._mat[0])):
        #         if self._mat[rowInd][colInd] == 0:
        #             self._robReachRowLst.append(rowInd)
        #             self._robReachColLst.append(colInd)
        #
        #
        # # print(len(self._robReachRowLst))
        # # for row in len(self._mat[0]):
        # #     for col in len(self._mat[])
        # # nx.draw(G)
        # # plt.show()
        # print('len = ',len(self._robReachColLst))


    def __str__(self):
        return 'robNum = ' + str(self._robNum) + ' row = ' + str(self._row) + ' col = ' + str(self._col)
if __name__ =='__main__':
    row = 20
    col = 20
    robNum = 2
    p = np.array([0.9,0.1])
    np.random.seed(1000)
    rob_x = np.random.randint(20,size = robNum)
    rob_y = np.random.randint(20,size = robNum)
    robPosLst = []
    for i in range(robNum):
        robPosLst.append((rob_x[i],rob_y[i]))
    # 0 means way
    # 1 means obstacles
    print(robPosLst)
    obstacleLst = []
    for rowInd in range(row):
        for colInd in range(col):
            if (rowInd,colInd) in robPosLst:
                obstacleLst.append(0)
            else:
                obstacleLst.append(np.random.choice([0,1],p =p.ravel()))
    ins = MCMPInstance(row,col,obstacleLst,robPosLst)
    # drawPath.drawPath(ins)
    # print(obstacleLst)
