from mcppIns.MCPPinstance import MCPPInstance
from mcppIns.stcMap import  STC_Map,GridInd,STCGridInd,STCVertType,STCVert,STCVirtualVertType,\
    DirType,getDir,VirConnectError
import  random
from enum import  Enum
from  drawEnv import drawPic,drawSTCPic,drawSTCGraph,drawEvalSTCGraph
from math import floor
from itertools import chain
import  networkx as nx
import  numpy as np
import  math
import  time
from mcppIns.MCPPastar import STC_ASTAR


class STC2Path(object):
    def __init__(self, ins, s_map):
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

        self._s_map = s_map
        self._stcGraph = s_map._stcGraph
        self._vitualIndSet = self._s_map._vitualIndSet
        self._stcVitualIndSet = self._s_map._stcVitualIndSet
        self._waySTCNodeNum = self._s_map._waySTCNodeNum


    def generatePath(self,robStartSTCIndLst, robStreeLst, robSetLst):
        self._robStartSTCIndLst = robStartSTCIndLst
        # print(self._robStartIndLst)
        self._robStreeLst = robStreeLst
        self._robSetLst = robSetLst
        self.generateVirtualPath()
        self.generateRealPath()
        makespan = self.calMakespan()
        return self.calMakespan()
        pass
    def generateRealPath(self):

        # _mcmp_astar = MCMP_Solver(self._ins)
        self._pathPntLst = [[] for x in range(self._robNum)]
        self._pathLst = [[] for x in range(self._robNum)]

        # print(self._leafVirSetLst)
        # raise Exception('xx')
        for robID in range(self._robNum):
            path = self._pathLst[robID]
            pathPnt = self._pathPntLst[robID]
            virPath = self._virPathLst[robID]

            leafVirLst = self._leafVirSetLst[robID]
            roadPath = []
            for vrow,vcol in virPath:
                if self._mat[vrow][vcol] == 0:
                    roadPath.append((vrow,vcol))
            stc_astar = STC_ASTAR(self._ins,roadPath)
            currentPos = virPath[0]
            path.append(currentPos)
            for i in range(1,len(virPath)):
                nextPos = virPath[i]
                if self._mat[nextPos.row][nextPos.col] == 1:
                    #is obstacle
                    continue

                neiLst = self._s_map.obMapNeighbors(currentPos)
                if len(leafVirLst) != 0:
                    removeBool = False
                    for leafInd in leafVirLst:
                        # print(leafInd)
                        if leafInd in neiLst:
                            # print(leafInd)
                            path.append(leafInd)
                            path.append(currentPos)
                            removeBool = True
                            break
                    if removeBool:
                        leafVirLst.remove(leafInd)

                if nextPos in neiLst:
                    currentPos = nextPos
                    path.append(nextPos)
                else:
                    a_path = list(stc_astar.astar(currentPos,nextPos))
                    currentPos = nextPos
                    path.extend(a_path)
        # print(self._pathLst)

    def generateVirtualPath(self):
        self._virPathPntLst = [[] for x in range(self._robNum)]
        self._virPathLst = [[] for x in range(self._robNum)]
        self._leafVirSetLst = [[] for x in range(self._robNum)]
        for robID in range(self._robNum):
            path = self._virPathLst[robID]
            pathPnt = self._virPathPntLst[robID]
            pos = self._robStartSTCIndLst[robID]
            rob_row = self._ins._robRowLst[robID]
            rob_col = self._ins._robColLst[robID]
            baseInd = GridInd(rob_row,rob_col)
            stree = self._robStreeLst[robID]

            # '''
            # 暂时调试方便
            # '''
            #

            self._reDefineNeighborAdd = dict()
            self._reDefineNeighborRemove = dict()
            robPathSet = set()
            for stcInd in self._robSetLst[robID]:
                if stcInd.virType == STCVirtualVertType.NoVir:
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LBInd)
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RBInd)
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LTInd)
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RTInd)
                else:
                    if stree.degree(stcInd) == 2:
                        # raise Exception('xx')
                        reDefineSp = False
                        for nei in stree.neighbors(stcInd):
                            if  nei.virType != STCVirtualVertType.NoVir:
                                stcNeiLst = list(stree.neighbors(stcInd))
                                stcNeiLst.remove(nei)
                                self.reDefineNeighborSp(stcInd,stcNeiLst[0])
                                reDefineSp = True
                                break
                        if not reDefineSp:
                            self.reDefineNeighbor(stcInd)
                        if stcInd.virType == STCVirtualVertType.VLB:
                            robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LBInd)
                        if stcInd.virType == STCVirtualVertType.VRB:
                            robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RBInd)
                        if stcInd.virType == STCVirtualVertType.VLT:
                            robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LTInd)
                        if stcInd.virType == STCVirtualVertType.VRT:
                            robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RTInd)
                    # if stree.degree(stcInd) == 1:
                    #     for nei in stree.neighbors(stcInd):
                    #         if nei in self._stcVitualIndSet and stree.degree(nei) == 2:
                    #             if stcInd.virType == STCVirtualVertType.VLB:
                    #                 robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LBInd)
                    #             if stcInd.virType == STCVirtualVertType.VRB:
                    #                 robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RBInd)
                    #             if stcInd.virType == STCVirtualVertType.VLT:
                    #                 robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LTInd)
                    #             if stcInd.virType == STCVirtualVertType.VRT:
                    #                 robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RTInd)
                    pass
            removeLst = []
            leafVirLst = self._leafVirSetLst[robID]
            for streeInd in stree:
                if streeInd in self._stcVitualIndSet and stree.degree(streeInd) == 1:
                    removeLst.append(streeInd)
                    # for nei in stree.neighbors(streeInd):
                    #     if nei in self._stcVitualIndSet and stree.degree(nei) == 2:
                    #         pass
                    #     else:
                    #         removeLst.append(streeInd)
            for streeInd in removeLst:
                stree.remove_node(streeInd)
                leafVirLst.append(self.getVirBaseInd(streeInd))
            # print('robid =',robID, '  ',self._reDefineNeighborAdd)
            cenDir = DirType.center

            while True:
                lastInd = baseInd
                lastDir = cenDir
                path.append(baseInd)
                pathPnt.append((baseInd.row + 0.5, baseInd.col + 0.5))
                # baseNeiLst = self._s_map.baseMapNeighbors(baseInd)
                stc_ind = self._s_map.gridInd2STCGridInd(baseInd)
                stcNeiLst = list (stree.neighbors(stc_ind))
                # if baseInd.row == 18 and baseInd.col == 3:
                #     pass
                    # print(baseInd)
                noIntersectLst = self.intersect(baseInd,stc_ind,stcNeiLst)

                chsSameMega = False
                candLst = []
                for ind in noIntersectLst:
                    if ind in path:
                        continue
                    if ind not in robPathSet:
                        continue
                    if self._s_map.inSameSTCMegaBox(baseInd,ind):
                        baseInd = ind
                        chsSameMega = True
                        break
                    else:
                        candLst.append(ind)
                if not chsSameMega:
                    if len(candLst) != 0:
                        # print(cenDir)
                        # print(candLst)
                        # print('baseInd', baseInd)
                        if len(candLst) == 1 :
                            baseInd = candLst[0]
                            # print('baseInd', baseInd)
                        else:
                            for cand in candLst:
                                candDir = getDir(baseInd, cand)
                                if lastDir == candDir:
                                    baseInd = cand
                                    # if lastDir ==
                                    break
                                if candDir == DirType.virConnect:
                                    baseInd = cand
                                    break
                            if lastDir == DirType.center:
                                baseInd = candLst[-1]
                                # print(baseInd)
                    else:
                        break
                        '''
                        end construct vitual path
                        '''
                # try:
                cenDir = getDir(lastInd,baseInd)
            # break
                # except VirConnectError as e:
                #     cenDir = DirType.virConnect
        # print(self._pathLst)
    def intersect(self, baseInd, stc_ind, stcNeiLst):
        noIntersectLst = []
        baseNeiDirLst = self._s_map.getNeighborDir(baseInd)
        if baseInd in self._reDefineNeighborAdd:
            if baseInd in self._reDefineNeighborRemove:
                removeBool = False
                for baseNei in baseNeiDirLst:
                    if baseNei[1] == self._reDefineNeighborRemove[baseInd]:
                        removeBool = True
                        break
                if removeBool:
                    baseNeiDirLst.remove(baseNei)
            baseNeiDirLst.append((DirType.virConnect, self._reDefineNeighborAdd[baseInd]))
            # print(baseNeiDirLst)
        stcNeiDirLst = []
        for stcNeiInd in stcNeiLst:
            stcNeiDir = getDir(stc_ind, stcNeiInd)
            stcNeiDirLst.append((stcNeiDir,stcNeiInd))
            # if stcNeiInd.virType != STCVirtualVertType.NoVir:
            #     print(stcNeiInd)
                # raise Exception('xx')
        for baseDir, baseNeiInd in baseNeiDirLst:
            if baseDir == DirType.left:
                intersectionBool = False
                for stcNeiDir,stcNeiInd in stcNeiDirLst:
                    if stcNeiDir ==  DirType.top:
                        if  stc_ind.col *2  + 1 < baseInd.col + 0.5< stcNeiInd.col *2 + 1:
                            if   baseNeiInd.row + 0.5 < stc_ind.row * 2 + 1 < baseInd.row + 0.5:
                                intersectionBool = True
                    if stcNeiDir == DirType.bottom:
                        if   stcNeiInd.col *2 + 1 < baseInd.col + 0.5 <stc_ind.col* 2 + 1:
                            if   baseNeiInd.row + 0.5 < stc_ind.row * 2 + 1 < baseInd.row + 0.5:
                                intersectionBool = True
                if not intersectionBool:
                    noIntersectLst.append(baseNeiInd)

            if baseDir == DirType.right:
                intersectionBool = False
                for stcNeiDir, stcNeiInd in stcNeiDirLst:
                    if stcNeiDir == DirType.top:
                        if stc_ind.col * 2 + 1 < baseInd.col + 0.5 < stcNeiInd.col * 2 + 1:
                            if  baseInd.row + 0.5 < stc_ind.row * 2 + 1 < baseNeiInd.row + 0.5:
                                intersectionBool = True
                    if stcNeiDir == DirType.bottom:
                        if stcNeiInd.col * 2 + 1 < baseInd.col + 0.5 < stc_ind.col * 2 + 1:
                            if  baseInd.row + 0.5 < stc_ind.row * 2 + 1 < baseNeiInd.row + 0.5:
                                intersectionBool = True
                if not intersectionBool:
                    noIntersectLst.append(baseNeiInd)

            if baseDir == DirType.top:
                intersectionBool = False
                for stcNeiDir, stcNeiInd in stcNeiDirLst:
                    if stcNeiDir == DirType.left:
                        if stcNeiInd.row*2 + 1 < baseInd.row + 0.5 < stc_ind.row * 2+ 1:
                            if baseInd.col + 0.5 < stc_ind.col*2 + 1 < baseNeiInd.col + 0.5:
                                intersectionBool = True
                    if stcNeiDir == DirType.right:
                        if  stc_ind.row * 2 + 1 < baseInd.row + 0.5 <  stcNeiInd.row * 2 + 1:
                            if baseInd.col + 0.5 < stc_ind.col * 2 + 1 < baseNeiInd.col + 0.5:
                                intersectionBool = True
                if not intersectionBool:
                    noIntersectLst.append(baseNeiInd)

            if baseDir == DirType.bottom:
                intersectionBool = False
                for stcNeiDir, stcNeiInd in stcNeiDirLst:
                    if stcNeiDir == DirType.left:
                        if stcNeiInd.row * 2 + 1 < baseInd.row + 0.5 < stc_ind.row * 2 + 1:
                            if  baseNeiInd.col  + 0.5 < stc_ind.col * 2 + 1 < baseInd.col + 0.5:
                                intersectionBool = True
                    if stcNeiDir == DirType.right:
                        if stc_ind.row * 2 + 1 < baseInd.row + 0.5< stcNeiInd.row * 2 + 1:
                            if  baseNeiInd.col + 0.5< stc_ind.col * 2 + 1 <  baseInd.col + 0.5:
                                intersectionBool = True
                if not intersectionBool:
                    noIntersectLst.append(baseNeiInd)

            if baseDir == DirType.virConnect:
                noIntersectLst.append(baseNeiInd)
        return noIntersectLst


    def reDefineNeighbor(self,stcInd: STCGridInd):
        if stcInd.virType ==  STCVirtualVertType.VLB:
            ltind = self._stcGraph.nodes[stcInd]['vert']._LTInd
            rbind = self._stcGraph.nodes[stcInd]['vert']._RBInd
            nltind = GridInd(ltind.row - 1 ,ltind.col)
            nrbind = GridInd(rbind.row, rbind.col - 1)
            self._reDefineNeighborAdd[nltind] = nrbind
            self._reDefineNeighborAdd[nrbind] = nltind
            self._reDefineNeighborRemove[nltind] = ltind
            self._reDefineNeighborRemove[nrbind] = rbind
        if stcInd.virType == STCVirtualVertType.VRT:
            ltind = self._stcGraph.nodes[stcInd]['vert']._LTInd
            rbind = self._stcGraph.nodes[stcInd]['vert']._RBInd
            nltind = GridInd(ltind.row, ltind.col + 1)
            nrbind = GridInd(rbind.row + 1, rbind.col)
            self._reDefineNeighborAdd[nltind] = nrbind
            self._reDefineNeighborAdd[nrbind] = nltind
            self._reDefineNeighborRemove[nltind] = ltind
            self._reDefineNeighborRemove[nrbind] = rbind

        if stcInd.virType == STCVirtualVertType.VLT:
            rtind = self._stcGraph.nodes[stcInd]['vert']._RTInd
            lbind = self._stcGraph.nodes[stcInd]['vert']._LBInd
            nrtind = GridInd(rtind.row, rtind.col + 1)
            nlbind = GridInd(lbind.row - 1, lbind.col)
            self._reDefineNeighborAdd[nrtind] = nlbind
            self._reDefineNeighborAdd[nlbind] = nrtind
            self._reDefineNeighborRemove[nrtind] = rtind
            self._reDefineNeighborRemove[nlbind] = lbind

        if stcInd.virType == STCVirtualVertType.VRB:
            rtind = self._stcGraph.nodes[stcInd]['vert']._RTInd
            lbind = self._stcGraph.nodes[stcInd]['vert']._LBInd
            nrtind = GridInd(rtind.row + 1, rtind.col)
            nlbind = GridInd(lbind.row, lbind.col - 1)
            self._reDefineNeighborAdd[nrtind] = nlbind
            self._reDefineNeighborAdd[nlbind] = nrtind
            self._reDefineNeighborRemove[nrtind] = rtind
            self._reDefineNeighborRemove[nlbind] = lbind

    def reDefineNeighborSp(self, stcInd: STCGridInd, neiInd: STCGridInd):
        if stcInd.virType == STCVirtualVertType.VRB:
            rbind = self._stcGraph.nodes[stcInd]['vert']._RBInd
            dir = getDir(stcInd,neiInd)
            if dir == DirType.right:
                rtind = self._stcGraph.nodes[stcInd]['vert']._RTInd
                nrtind = GridInd(rtind.row + 1, rtind.col)
                self._reDefineNeighborAdd[rbind] = nrtind
                self._reDefineNeighborAdd[nrtind] = rbind
            if dir == DirType.bottom:
                lbind = self._stcGraph.nodes[stcInd]['vert']._LBInd
                nlbind = GridInd(lbind.row, lbind.col - 1)
                self._reDefineNeighborAdd[rbind] = nlbind
                self._reDefineNeighborAdd[nlbind] = rbind

        if stcInd.virType == STCVirtualVertType.VLT:
            ltind = self._stcGraph.nodes[stcInd]['vert']._LTInd
            dir = getDir(stcInd,neiInd)
            if dir == DirType.top:
                rtind = self._stcGraph.nodes[stcInd]['vert']._RTInd
                nrtind = GridInd(rtind.row, rtind.col + 1)
                self._reDefineNeighborAdd[ltind] = nrtind
                self._reDefineNeighborAdd[nrtind] = ltind
            if dir == DirType.left:
                lbind = self._stcGraph.nodes[stcInd]['vert']._LBInd
                nlbind = GridInd(lbind.row - 1, lbind.col)
                self._reDefineNeighborAdd[ltind] = nlbind
                self._reDefineNeighborAdd[nlbind] = ltind


        if stcInd.virType == STCVirtualVertType.VLB:
            lbind= self._stcGraph.nodes[stcInd]['vert']._LBInd
            dir = getDir(stcInd,neiInd)
            if dir == DirType.left:
                ltind = self._stcGraph.nodes[stcInd]['vert']._LTInd
                nltind = GridInd(ltind.row - 1, ltind.col)
                self._reDefineNeighborAdd[lbind] = nltind
                self._reDefineNeighborAdd[nltind] = lbind
            if dir == DirType.bottom:
                rbind = self._stcGraph.nodes[stcInd]['vert']._RBInd
                nrbind = GridInd(rbind.row, rbind.col - 1)
                self._reDefineNeighborAdd[lbind] = nrbind
                self._reDefineNeighborAdd[nrbind] = lbind


        if stcInd.virType == STCVirtualVertType.VRT:
            rtind = self._stcGraph.nodes[stcInd]['vert']._RTInd
            dir = getDir(stcInd,neiInd)
            if dir == DirType.bottom:
                ltind = self._stcGraph.nodes[stcInd]['vert']._LTInd
                nltind = GridInd(ltind.row, ltind.col + 1)
                self._reDefineNeighborAdd[rtind] = nltind
                self._reDefineNeighborAdd[nltind] = rtind
            if dir == DirType.right:
                rbind = self._stcGraph.nodes[stcInd]['vert']._RBInd
                nrbind = GridInd(rbind.row + 1, rbind.col)
                self._reDefineNeighborAdd[rtind] = nrbind
                self._reDefineNeighborAdd[nrbind] = rtind

        # if stcInd.virType == STCVirtualVertType.VLT:
        #     ltind = self._stcGraph.nodes[stcInd]['vert']._LTInd
        #     dir = getDir(stcInd,neiInd)
        #     if dir == DirType.top:
        #         rtind = self._stcGraph.nodes[stcInd]['vert']._RTInd
        #         nrtind = GridInd(rtind.row, rtind.col + 1)
        #         self._reDefineNeighborAdd[ltind] = nrtind
        #         self._reDefineNeighborAdd[nrtind] = ltind
        #     if dir == DirType.left:
        #         lbind = self._stcGraph.nodes[stcInd]['vert']._LBInd
        #         nlbind = GridInd(lbind.row - 1, lbind.col)
        #         self._reDefineNeighborAdd[ltind] = nlbind
        #         self._reDefineNeighborAdd[nlbind] = ltind
        # if
        # if stcInd.virType == STCVirtualVertType.VLB:
        #     ltind = self._stcGraph.nodes[stcInd]['vert']._LTInd
        #     rbind = self._stcGraph.nodes[stcInd]['vert']._RBInd
        #     nltind = GridInd(ltind.row - 1, ltind.col)
        #     nrbind = GridInd(rbind.row, rbind.col - 1)
        #     self._reDefineNeighborAdd[nltind] = nrbind
        #     self._reDefineNeighborAdd[nrbind] = nltind
        #     self._reDefineNeighborRemove[nltind] = ltind
        #     self._reDefineNeighborRemove[nrbind] = rbind
        # if stcInd.virType == STCVirtualVertType.VRT:
        #     ltind = self._stcGraph.nodes[stcInd]['vert']._LTInd
        #     rbind = self._stcGraph.nodes[stcInd]['vert']._RBInd
        #     nltind = GridInd(ltind.row, ltind.col + 1)
        #     nrbind = GridInd(rbind.row + 1, rbind.col)
        #     self._reDefineNeighborAdd[nltind] = nrbind
        #     self._reDefineNeighborAdd[nrbind] = nltind
        #     self._reDefineNeighborRemove[nltind] = ltind
        #     self._reDefineNeighborRemove[nrbind] = rbind
        #
        # if stcInd.virType == STCVirtualVertType.VLT:
        #     rtind = self._stcGraph.nodes[stcInd]['vert']._RTInd
        #     lbind = self._stcGraph.nodes[stcInd]['vert']._LBInd
        #     nrtind = GridInd(rtind.row, rtind.col + 1)
        #     nlbind = GridInd(lbind.row - 1, lbind.col)
        #     self._reDefineNeighborAdd[nrtind] = nlbind
        #     self._reDefineNeighborAdd[nlbind] = nrtind
        #     self._reDefineNeighborRemove[nrtind] = rtind
        #     self._reDefineNeighborRemove[nlbind] = lbind
        #
        # if stcInd.virType == STCVirtualVertType.VRB:
        #     rtind = self._stcGraph.nodes[stcInd]['vert']._RTInd
        #     lbind = self._stcGraph.nodes[stcInd]['vert']._LBInd
        #     nrtind = GridInd(rtind.row + 1, rtind.col)
        #     nlbind = GridInd(lbind.row, lbind.col - 1)
        #     self._reDefineNeighborAdd[nrtind] = nlbind
        #     self._reDefineNeighborAdd[nlbind] = nrtind
        #     self._reDefineNeighborRemove[nrtind] = rtind
        #     self._reDefineNeighborRemove[nlbind] = lbind

            # self._reDefineNeighbor[ltind] =
    def getVirBaseInd(self,stc_ind:STCGridInd):
        if stc_ind.virType == STCVirtualVertType.VLB:
            return self._stcGraph.nodes[stc_ind]['vert']._LBInd
        if stc_ind.virType == STCVirtualVertType.VLT:
            return self._stcGraph.nodes[stc_ind]['vert']._LTInd
        if stc_ind.virType == STCVirtualVertType.VRB:
            return self._stcGraph.nodes[stc_ind]['vert']._RBInd
        if stc_ind.virType == STCVirtualVertType.VRT:
            return self._stcGraph.nodes[stc_ind]['vert']._RTInd


    def calMakespan(self):
        maxPath = max(self._pathLst, key =  lambda x: len(x))
        return len(maxPath)



if __name__ == '__main__':
    pass
