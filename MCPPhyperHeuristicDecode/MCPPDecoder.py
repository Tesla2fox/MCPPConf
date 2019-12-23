from mcppIns.MCPPinstance import MCPPInstance
from mcppIns.stcMap import STC_Map,GridInd,STCGridInd,STCVertType,STCVert,STCVirtualVertType,\
    DirType,getDir
import sympy
from math import floor
from drawEnv import  drawSTCGraph
import numpy as np
import networkx as nx
import time
from  mcppIns.MCPPastar import MCMP_Solver
from mcppIns.stc2path import STC2Path
from collections import namedtuple
import random as rd
from enum import Enum
from functools import cmp_to_key

AEdge = namedtuple('AEdge',['sInd','tInd'])
AucVertPara = namedtuple('AucVertPara',['ind','reO','gO','lT'])


class VertexType(Enum):
    BLeaf = 1
    SLeaf = 2



def sortOrder(seq : list):
    if len(seq)==0:
        raise  Exception('getOrder bug')
    o_seq = sorted(seq, key = lambda x :x[1])
    ind_seq = dict()
    # orderInd = -1
    orderInd = 0
    orderFitness = - 1
    # print(o_seq)
    for i in range(len(o_seq)):
        unit,fit = o_seq[i]
    # for i,unit,fit in enumerate(o_seq):
        if fit > orderFitness:
            ind_seq[unit] = i
            orderInd = i
            orderFitness = fit
        else:
            ind_seq[unit] = orderInd
    return ind_seq


class MCPP_HH_Decoder(object):
    def __init__(self,ins:MCPPInstance):
        self._ins = ins
        self._row = ins._row
        self._col = ins._col
        self._ins = ins
        self._mat = ins._mat
        self._robNum = ins._robNum
        self._robPosLst = ins._robPosLst
        self._robRowLst = ins._robRowLst
        self._robColLst = ins._robColLst
        self._robReachRowLst = ins._robReachRowLst
        self._robReachColLst = ins._robReachColLst

        self._s_map = STC_Map(self._ins)
        self._stcGraph = self._s_map._stcGraph
        self._vitualIndSet = self._s_map._vitualIndSet
        self._stcVitualIndSet = self._s_map._stcVitualIndSet
        self._waySTCNodeNum = self._s_map._waySTCNodeNum
        self._stcConvert2Path = STC2Path(self._ins, self._s_map)

    def decode(self,x,y):
        print(x)
        if x[0] == 0:
            self._selectAuctioneer = self.selectAuctioneerMin
            print('selectinAuctioneer Min')
        elif x[0] == 1:
            self._selectAuctioneer = self.selectAuctioneerRandom
            print('selectinAuctioneer Rand')
        elif x[0] == 2:
            self._selectAuctioneer = self.selectAuctioneerInSeq
        else:
            raise Exception('selectAuction may be something wrong')
        if x[1] == 0 :
            pass
        print(y)
        self._robRepulsLst = [rd.random() for _ in range(self._robNum)]
        '''
        repulsion and grivitation 's weight
        '''
        self._robWGraviReLst = [rd.random() for _ in range(self._robNum)]

        '''
        cmp_auction_vertex
        '''
        if x[1] == 0:
            self._distBLeaf = True
        elif x[1] == 1:
            self._distBLeaf = False

        # self.initialAssignByDis()
        self._initialization()
        self._auction()

        return  0
        # exit()
        self._pathLst = None
        a_start = time.perf_counter()
        self.auction()
        a_end = time.perf_counter()
        print('aution used:',a_end - a_start)
        if len(self._assignSet) == self._waySTCNodeNum:
            makespan = self._stcConvert2Path.generatePath(self._robStartSTCIndLst,
                                                          self._robStreeLst,
                                                          self._robSetLst)
            self._pathLst = self._stcConvert2Path._pathLst
            return True,makespan
        else:
            return False,np.inf

    def _initialization(self):
        self._robEstCostLst = [0 for x in range(self._robNum)]
        self._robSetLst = [[] for x in range(self._robNum)]
        self._robNeiSetLst = [[] for x in range(self._robNum)]
        self._noBidSet = set()
        self._haveAuctionInd = dict()
        self._robSleepLst = [False for x in range(self._robNum)]
        self._robStreeLst = [nx.Graph() for x in range(self._robNum)]
        self._robEdgeLst = [[] for x in range(self._robNum)]
        self._assignSet = set()
        self._sleepDependLst = [[] for x in range(self._robNum)]
        pass

    def _auction(self):
        '''
        :return:
        '''


        '''
        参数的初始化
        '''
        self._robStartSTCIndLst = []
        for robID in range(self._robNum):
            ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))
            self._robStartSTCIndLst.append(ind)
            initCost = self.leafCost(ind)
            self._robEstCostLst[robID] =  initCost
            self._robSetLst[robID].append(ind)
            self._noBidSet.add(ind)
            self._robStreeLst[robID].add_node(ind)
            self.updateNeiGraphRobID(robID)
            self._haveAuctionInd[ind] = robID

        circleTime = 0
        while True:
            auctioneerRobID = self._selectAuctioneer(circleTime)

            auctionInd,allAssign = self._calAcutionVertID(auctioneerRobID)
            # print(auctionInd)
            # if auctionInd == STCGridInd(-1,-1,STCVirtualVertType.NoVir):
            #     pass
                # raise Exception('xx')
            if allAssign:
                print('xxxx')
                print(self._robEstCostLst)
                pass
                # break

            if False not in self._robSleepLst:
                break

            if self._robSleepLst[auctioneerRobID] == True:
                continue

            winnerRobID, sInd, minCost = self.maxBiddingRob(auctionInd)

            if allAssign == False:
                robWinnerSet = self._robSetLst[winnerRobID]
                robWinnerSet.append(auctionInd)
                sTree = self._robStreeLst[winnerRobID]
                sTree.add_edge(auctionInd,sInd)
                self._robEdgeLst[winnerRobID].append(AEdge(sInd= sInd, tInd = auctionInd))
                self._haveAuctionInd[auctionInd] = winnerRobID
                self._robEstCostLst[winnerRobID] = minCost
                self.updateNeiGraphRobID(winnerRobID)
                for sleepRobID in self._sleepDependLst[winnerRobID]:
                    self._robSleepLst[sleepRobID] = False
                self._sleepDependLst[winnerRobID].clear()
                # for robID in range(self._robNum):
                    # if
                pass
            else:
                if auctioneerRobID == winnerRobID:
                    '''
                    此处存在BUG 需要修复
                    '''
                    # raise  Exception('xx')
                    self.loserEarseVert(self._haveAuctionInd[auctionInd], auctionInd)
                    robWinnerSet = self._robSetLst[winnerRobID]
                    robWinnerSet.append(auctionInd)
                    sTree = self._robStreeLst[winnerRobID]
                    sTree.add_edge(auctionInd, sInd)
                    self._robEdgeLst[winnerRobID].append(AEdge(sInd = sInd, tInd= auctionInd))
                    self._haveAuctionInd[auctionInd] = winnerRobID
                    self._robEstCostLst[winnerRobID] = minCost
                    self.updateNeiGraphRobID(winnerRobID)
                else:
                    raise  Exception('xx')
            print(self._robEstCostLst)
            print('circleTime = ', circleTime)
            circleTime += 1
            # print('robSetLst = ', self._robSetLst)
            if circleTime >= 250:
                break
        '''
        for debug
        '''
        print(self._robEstCostLst)
    def selectAuctioneer(self):
        pass
    def selectAuctioneerMin(self, circleTime ):
        '''
        select a robot which has the min estCost
        :return:
        '''
        estCostLst = []
        for i in range(self._robNum):
            if self._robSleepLst[i] == False:
                estCostLst.append((i,self._robEstCostLst[i]))
        minEstCost = min(estCostLst, key = lambda  x: x[1])
        minRobID = minEstCost[0]
        '''
        for debug 
        '''
        print(self._robEstCostLst)
        print('auctioneerID = ',minRobID)
        return minRobID
    def selectAuctioneerRandom(self,circleTime):
        return np.random.randint(self._robNum)
    def selectAuctioneerInSeq(self,circleTime):
        return circleTime%self._robNum

    def _cmpAuction(self,a:AucVertPara,b:AucVertPara):
        if self._distBLeaf:
            if a.lT == b.lT:
                aFit = self._wGraviRe * a.gO + (1-self._wGraviRe) *a.reO
                bFit = self._wGraviRe * b.gO + (1-self._wGraviRe) *b.reO
                return  aFit - bFit
                # pass
            elif a.lT == VertexType.BLeaf:
                return -1
            elif a.lT == VertexType.SLeaf:
                return 1
        else:
            aFit = self._wGraviRe * a.gO + (1 - self._wGraviRe) * a.reO
            bFit = self._wGraviRe * b.gO + (1 - self._wGraviRe) * b.reO
            return aFit - bFit

    def _calAcutionVertID(self,auctioneerID):
        '''

        :param auctioneerID:
        :return: auction vertex Index
        auction vertex type : assigned vertex or unassigned vertex
        '''

        robNeiSet = self._robNeiSetLst[auctioneerID]
        if len(robNeiSet) == 0:
            return STCGridInd(-1,-1,STCVirtualVertType.NoVir), False

        robUnANeiLst = []
        robANeiLst = []
        for ind in robNeiSet:
            if ind in self._haveAuctionInd:
                robANeiLst.append(ind)
            else:
                robUnANeiLst.append(ind)

        robUnANeiReFitLst = []
        robUnANeiGFitLst = []
        robUnANeiTDic = dict()
        for ind in robUnANeiLst:
            _type,_reFit,_gravFit = self._calUnAFit(auctioneerID, ind)
            robUnANeiReFitLst.append((ind,_reFit))
            robUnANeiGFitLst.append((ind,_gravFit))
            robUnANeiTDic[ind] = _type
        robUnANeiReODic = sortOrder(robUnANeiReFitLst)
        robUnANeiGODic = sortOrder(robUnANeiGFitLst)
        # print(robUnANeiReODic)
        # print(robUnANeiGODic)
        robUnANeiOLst = []
        for ind in robUnANeiLst:
            robUnANeiOLst.append(AucVertPara(ind= ind, lT= robUnANeiTDic[ind],
                                             reO= robUnANeiReODic[ind], gO= robUnANeiGODic[ind]))
        self._wGraviRe = self._robWGraviReLst[auctioneerID]
        print(sorted(robUnANeiOLst,key = cmp_to_key(self._cmpAuction)))
        print(robUnANeiOLst)


        exit()
        # robUnANeiReFitLst = sorted(robUnANeiReFitLst, key = lambda x: x[1])
        # robUnANeiGFitLst = sorted((robUnANeiGFitLst, key = lambda  x: x[1]))
        # for ind in self.calOpPriority()

        print(robNeiSet)
        exit()

    def calAcutionVertID(self,auctioneerID):
        robNeiSet = self._robNeiSetLst[auctioneerID]
        if len(robNeiSet) == 0:
            return False, STCGridInd(-1,-1,STCVirtualVertType.NoVir)

        for ind in robNeiSet:
            inotherSet = False

        robOpNeiLst = []
        minFit = (True, -1)
        allAssign = True
        auctionInd = STCGridInd(-1,-1,STCVirtualVertType.NoVir)
        for ind in robNeiSet:
            inotherSet = False
            if ind in self._haveAuctionInd:
                inotherSet = True
                opRobID = self._haveAuctionInd[ind]
                robOpNeiLst.append([opRobID,ind,self._robEstCostLst[opRobID]])
            else:
                fit = self.calUnOpPriority(auctioneerID,ind)
                # print('fit = ',fit)
                if self.cmpUnOp(fit,minFit):
                    minFit = fit
                    allAssign = False
                    auctionInd = ind

        if allAssign == False:
            return auctionInd,allAssign

        for i in range(len(robOpNeiLst)):
            minInd,cost = self.calCost(auctioneerID,robOpNeiLst[i][1])
            robOpNeiLst[i][2] = robOpNeiLst[i][2] - cost

        robOpNeiLst = sorted(robOpNeiLst, key = lambda x :x [2], reverse= True)
        maxCandi =  robOpNeiLst[0][2]
        if maxCandi <= 0:
            self._robSleepLst[auctioneerID] = True
            # for i in range()
            for robOpUnit in robOpNeiLst:
                if auctioneerID in self._sleepDependLst[robOpUnit[0]]:
                    continue
                self._sleepDependLst[robOpUnit[0]].append(auctioneerID)
            return auctionInd, allAssign
        else:
            self._robSleepLst[auctioneerID] = False

        '''
        end the rule 1 
        '''
        print('end the rule 1')
        '''
        还需要判断最小值的数量
        '''
        for opRobID,ind,cost in robOpNeiLst:
        # for i in range(len(robOpNeiLst)):
            if cost <= 0:
                self._robSleepLst[auctioneerID] = True
                return auctionInd, allAssign
            else:
                auctionBool, pri = self.calOpPriority(auctioneerID,opRobID,ind)
                if auctionBool:
                    auctionInd = ind
                    return auctionInd,allAssign

        raise  Exception('end the rule 2')

    def maxBiddingRob(self,autionInd:STCGridInd):
        maxBidding = -1
        costLst = []
        biddingLst = []
        for robID in range(self._robNum):
            # if autionInd inself._robNeiSetLst[robID]
            sInd,cost = self.calCost(robID, autionInd)
            costLst.append((sInd,cost))
            bidding = 1/cost
            biddingLst.append(bidding)

        # print(biddingLst)
        maxElement = max(biddingLst)
        winnerRobID = biddingLst.index(maxElement)
        sInd = costLst[winnerRobID][0]
        cost = costLst[winnerRobID][1]
        return winnerRobID,sInd,cost
    def cmpUnOp(self,fit1,fit2):
        '''
        if fit1 have higher priority return true
        else return false
        :param fit1: the new fitnesss  FIT1[0] MEANS IS THE BETTER PERFORMANCE OF LEAFNODE FIT[1] MEANS THE DISTANCE.
        :param fit2: current min fitness
        :return:
        '''
        if fit1[0] == fit2[0]:
            if fit1[1] > fit2[1]:
                return True
            else:
                return False
        else:
            if fit1[0] == True:
                return False
            else:
                return True
        raise  Exception('cmpUnop bug')

    def calCost(self,robID, auctionInd : STCGridInd):
        cost = np.inf
        minInd = np.inf
        if auctionInd in self._robNeiSetLst[robID]:
            candValLst = []
            candinateLst =  self._stcGraph.neighbors(auctionInd)
            for candinateInd in candinateLst:
                if candinateInd in self._robSetLst[robID]:
                    candValLst.append((candinateInd,self.estAddCost(robID,auctionInd,candinateInd)))

            minInd, minCost = min(candValLst,key = lambda x: x[1])
            minCost += self._robEstCostLst[robID]
            return minInd, minCost
            # xx
        if auctionInd in self._robSetLst[robID]:
            minCost = self._robEstCostLst[robID]
            return minInd, minCost
            # raise Exception('have not finish')
            # xx
        return minInd,cost

    def estAddCost(self,robID, sInd: STCGridInd, tInd: STCGridInd):
        '''
        esatimate the add cost of rob path
        :param robID:
        :param svd: the svd means the vertex out of the graph
        :param tvd: the tvd means the vertex in the rob graph
        :return: the estimated cost
        '''
        if self._stcGraph.nodes[sInd]['vert']._type == STCVertType.single:
            cost = 6
        if self._stcGraph.nodes[sInd]['vert']._type == STCVertType.doubleSame:
            if self._s_map.verticalDouble(sInd,tInd):
                cost  = 2
            else:
                cost  = 4
        if self._stcGraph.nodes[sInd]['vert']._type == STCVertType.doubleDiff or \
                self._stcGraph.nodes[sInd]['vert']._type == STCVertType.triple:
            cost = 2
        if self._stcGraph.nodes[sInd]['vert']._type == STCVertType.wayVert:
            cost = 4

        if self._stcGraph.nodes[tInd]['vert']._type == STCVertType.doubleSame and self.degree(robID,tInd) == 1:
            cost += 2
        if self._stcGraph.nodes[tInd]['vert']._type == STCVertType.doubleDiff and self.degree(robID,tInd) == 1:
            cost += 2
        # print(self._stcGraph.nodes[sInd]['vert']._type)
        return cost
        # self.estAddCost()
    def calMakespan(self):
        maxPath = max(self._pathLst, key =  lambda x: len(x))
        return len(maxPath)


    def _calUnAFit(self,robID :int ,stcGridInd: STCGridInd):
        '''

        :param robID:
        :param stcGridInd:
        :return:
        '''
        '''
        calculate the fitness of robots' repulsion 
        '''
        reFitNess = 0
        for i in range(self._robNum):
            if i == robID:
                continue
            if False:
                robSet = self._robSetLst[i]
                for ind in robSet:
                    dis = self.distanceSTCInd(ind, stcGridInd) * self._robRepulsionLst[i]
                    fitNess += dis
            else:
                dis = self.dinstanceRob2STCInd(i,stcGridInd) * self._robRepulsLst[i]
                reFitNess += dis
        gravFitness = self.dinstanceRob2STCInd(i,stcGridInd)
        if self._stcGraph.nodes[stcGridInd]['vert']._type == STCVertType.doubleSame \
                or self._stcGraph.nodes[stcGridInd]['vert']._type == STCVertType.doubleDiff:
            betterLeaf = VertexType.BLeaf
        else:
            betterLeaf = VertexType.SLeaf
        return betterLeaf, reFitNess, gravFitness


    def calUnOpPriority(self,robID :int ,stcGridInd: STCGridInd):
        '''
        :param robID:
        :param stcGridInd:
        :return: betterLeaf  priority is lower than vertexes which are not fitting the leaf node
        '''
        # robSet = self._robSetLst[robID]
        fitNess =  0
        for i in range(self._robNum):
            robSet = self._robSetLst[i]
            if i == robID:
                continue
            for ind in robSet:
                dis = self.distanceSTCInd(ind,stcGridInd)
                fitNess += dis
        if self._stcGraph.nodes[stcGridInd]['vert']._type == STCVertType.doubleSame \
                or self._stcGraph.nodes[stcGridInd]['vert']._type == STCVertType.doubleDiff:
            betterLeaf = True
        else:
            betterLeaf = False
        return betterLeaf,fitNess



    def calOpPriority(self,auctioneerRobID: int, opRobID :int, opStcGridInd :STCGridInd):
        robSet = self._robSetLst[opRobID]
        if opStcGridInd in self._noBidSet:
            return False,np.inf

        stree = self._robStreeLst[opRobID]
        if stree.degree(opStcGridInd) == 1:
            return True, 0
        if stree.degree(opStcGridInd) != 1:
            print("xxx")
            # if stree.degree(opStcGridInd)
            # return False,0
        robNodeLst = [x for x in robSet]
        print('robNodeLst = ', robNodeLst)
        if opStcGridInd in robNodeLst:
            print('in it')
        else:
            print('xxx')
        robNodeLst.remove(opStcGridInd)
        print('robSet = ',robSet)
        print('robNodeLst = ', robNodeLst)
        if robSet == robNodeLst:
            raise Exception('XXX')
        for unit in robSet:
            print(unit)
        # for unit in robNodeLst
        if opStcGridInd not in robSet:
            raise Exception('xxxx')
        if opStcGridInd in robNodeLst:
            raise Exception('xxxxx')
        # raise  Exception('''X''')
        subGraph = stree.subgraph(robNodeLst)
        components = list(nx.connected_components(subGraph))
        if len(components) == 1:
            print('components num = ', len(components))
            print('opSTC_gridInd = ', opStcGridInd)
            # exit('bug is here')
            return True, 0
        else:
            print('components = ', len(components))
            componentsNum = len(components)
            if componentsNum == 2:
                largest_cc = max(components, key=len)

            # if
            exit()
            raise Exception('xx')

        raise Exception('xxxx')
        indLst = []
        for ind in robSet:
            if stcGridInd == ind:
                continue
            indLst.append(ind)
        if self._s_map.allConnected(indLst):
            '''
            此处和文章不符合 需要进行修改
            '''
            return True,0
        else:
            return False,np.inf


    def distanceSTCInd(self,sInd,tInd):
        vec1 = np.array([self._stcGraph.nodes[sInd]['vert']._pos_x,self._stcGraph.nodes[sInd]['vert']._pos_y])
        vec2 = np.array([self._stcGraph.nodes[tInd]['vert']._pos_x,self._stcGraph.nodes[tInd]['vert']._pos_y])
        '''
        欧式距离
        '''
        return np.linalg.norm(vec1 - vec2, ord = 2)

    def dinstanceRob2STCInd(self,robID,tInd):
        vec2 = np.array([self._stcGraph.nodes[tInd]['vert']._pos_x,self._stcGraph.nodes[tInd]['vert']._pos_y])
        vec1 = np.array([self._robPosLst[robID][0],self._robPosLst[robID][1]])
        return np.linalg.norm(vec1 - vec2, ord = 2)

    def degree(self,robID,ind):
        stree = self._robStreeLst[robID]
        return stree.degree(ind)

    def leafCost(self, ind):
        vertType = self._stcGraph.nodes[ind]['vert']._type
        cost = 1000
        if vertType == STCVertType.single:
            cost = 6
            return cost
        if vertType == STCVertType.doubleSame:
            cost = 2
            return  cost
        if vertType == STCVertType.doubleDiff:
            cost =  2
            return cost
        if vertType == STCVertType.wayVert:
            cost = 4
            return cost
        raise  Exception('cost is error')

    def updateNeiGraphRobID(self,robID):
        robSet = self._robSetLst[robID]
        robNeiSet = self._robNeiSetLst[robID]
        robNeiSet.clear()
        for ind in robSet:
            neiLst = self._stcGraph.neighbors(ind)
            for neiInd in neiLst:
                if neiInd not in robSet:
                    robNeiSet.append(neiInd)
        return False

    def loserEarseVert(self,loserID, auctionInd: STCGridInd):
        robLoserSet = self._robSetLst[loserID]
        robLoserSet.remove(auctionInd)
        robLoserLst = list(robLoserSet)
        # robLoserLst.remove(auctionInd)
        print('robLoserLst', robLoserLst)
        # exit(robLoserLst)
        stree = self._robStreeLst[loserID]
        subGraph = stree.subgraph(robLoserLst)
        components = list(nx.connected_components(subGraph))

        if len(components) == 1:
            self._robEstCostLst[loserID] = self._robEstCostLst[loserID] - self.leafCost(auctionInd)
            self._robStreeLst[loserID].remove_node(auctionInd)
        else:
            print(' auctionInd = ', auctionInd)
            for component in components:
                print(component)
                # print(components)
            # exit()
            raise  Exception('xx')
            stree = stree.subgraph()
        # raise Exception("xx")
        pass
    def __str__(self):
        return 'mcpp_hh_decode row = ' + str(self._row) + ' col = ' + str(self._col)

    def initialAssignByDis(self):
        self._robSetLst = [[] for x in range(self._robNum)]
        for ind in self._stcGraph:
            if self._stcGraph.nodes[ind]['vert']._type == STCVertType.obstacle:
                continue
            disLst = []
            for robID in range(self._robNum):
                disLst.append(self.dinstanceRob2STCInd(robID,ind))
            minRob = disLst.index(min(disLst))
            self._robSetLst[minRob].append(ind)



import os
if __name__ == '__main__':
    ins = MCPPInstance()
    ins.loadCfg(os.path.dirname(os.getcwd()) + '//benchmarkData//r4_r51_c51_s1bootybay.map')
    hhDecoder = MCPP_HH_Decoder(ins)
    rd.seed(1)
    x = [rd.randint(0,1) for _ in range(10)]
    # x[0] = 2
    y = [rd.random() for _ in range(ins._robNum)]
    hhDecoder.decode(x,y)
    stcGraphLst = []
    stcNeiGraphLst = []
    allEdgeLstPnt = []
    for robID in range(hhDecoder._robNum):
        _robSet = hhDecoder._robSetLst[robID]
        _stcGraph = []
        for stcGridInd in _robSet:
            _stcGraph.append((stcGridInd.row * 2, stcGridInd.col * 2))
        stcGraphLst.append(_stcGraph)
        # _robNeiSet = astc._robNeiSetLst[robID]
        # _stcNeiGraph = []
        # for stcGridInd in _robNeiSet:
        #     _stcNeiGraph.append((stcGridInd.row * 2, stcGridInd.col * 2))
        # stcNeiGraphLst.append(_stcNeiGraph)
        # stree = astc._robStreeLst[robID]
        # edgeLst = []
        # for edge in stree.edges():
        #     # print(edge)
        #     t_pos_x = astc._stcGraph.nodes[edge[0]]['vert']._pos_x
        #     t_pos_y = astc._stcGraph.nodes[edge[0]]['vert']._pos_y
        #
        #     s_pos_x = astc._stcGraph.nodes[edge[1]]['vert']._pos_x
        #     s_pos_y = astc._stcGraph.nodes[edge[1]]['vert']._pos_y
        #
        #     edgeLst.append((t_pos_x, t_pos_y, s_pos_x, s_pos_y))
        # allEdgeLstPnt.append(edgeLst)
        # stree.edges
    # print(stcGraphLst)
    # drawSTCGraph(ins,stcGraphLst,stcNeiGraphLst = stcNeiGraphLst)
    # print(stcGraphLst)
    # for stcGraph in stcNeiGraphLst:
    #     print(stcGraph)
    # exit()
    drawSTCGraph(ins, fileName=os.path.dirname(os.getcwd()) + '//fig//bootybay'
                 , stcGraphLst=stcGraphLst)
                 # stcNeiGraphLst=None, edgePntLst=allEdgeLstPnt, multiPath=astc._pathLst)



