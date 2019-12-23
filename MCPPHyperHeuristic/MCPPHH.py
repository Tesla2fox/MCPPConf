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
AEdge = namedtuple('AEdge',['sInd','tInd'])


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


    def decode(self):


if __name__ == '__main__':
    pass



