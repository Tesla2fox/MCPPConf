import plotly.graph_objects as go
import plotly
import numpy as np
from mcppIns import MCPPinstance
import  colorlover as cl

class Pnt:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
    def pnt2dict(self):
        dic = dict(x = self.x, y = self.y)
        return dic
    def display(self):
        print('x = ', self.x, 'y = ', self.y)

class Rect:
    def __init__(self, pnt=Pnt(), width=0, height=0):
        self.x0 = pnt.x
        self.y0 = pnt.y
        self.x1 = self.x0 + width
        self.y1 = self.y0 + height

    def goShape(self):
        dic = go.layout.Shape()
        dic['type'] = 'rect'
        dic['x0'] = self.x0
        dic['y0'] = self.y0
        dic['x1'] = self.x1
        dic['y1'] = self.y1
        dic['line'] = dict(color='rgb(128, 0, 128)')
        return dic

    def rect2dict(self):
        dic = dict()
        dic['type'] = 'rect'
        dic['x0'] = self.x0
        dic['y0'] = self.y0
        dic['x1'] = self.x1
        dic['y1'] = self.y1
        dic['line'] = dict(color='rgb(128, 0, 128)')
        return dic
    def rect2Lst(self):
        lst_x = []
        lst_y = []
        lst_x.append(self.x0)
        lst_y.append(self.y0)
        lst_x.append(self.x0 + 1)
        lst_y.append(self.y0)
        lst_x.append(self.x0)
        lst_y.append(self.y0 + 1)
        lst_x.append(self.x0 + 1)
        lst_y.append(self.y0 + 1)
        lst_x.append(self.x0)
        lst_y.append(self.y0)
        return lst_x, lst_y


class Line:
    def __init__(self, pnt0=Pnt(), pnt1=Pnt()):
        self.x0 = pnt0.x
        self.y0 = pnt0.y
        self.x1 = pnt1.x
        self.y1 = pnt1.y

    def line2dict(self):
        dic = dict()
        dic['type'] = 'line'
        dic['x0'] = self.x0
        dic['y0'] = self.y0
        dic['x1'] = self.x1
        dic['y1'] = self.y1
        dic['line'] = dict(color='rgb(128, 0, 128)')
        return dic

class Env:
    def __init__(self, mat):
        self._mat = mat
        self._shapeLst= []
        self._scatterLst = []
        self._annotationsLst = []
    def addgrid(self):
        gridInd = 0
        g_color = 'blue'
        row = len(self._mat)
        for i in range(row):
            for j in range(len(self._mat[i])):
                # print(gridInd)
                gridInd += 1
                pnt = Pnt(i, j)
                rect = Rect(pnt, 1, 1)
                rectDic = rect.rect2dict()
                rectDic['line']['color'] = g_color
                rectDic['line']['width'] = 0.5
                #                rectDic['opacity'] =  1/(int(self.mat[i][j])+1)
                #                rectDic['fillcolor'] = colorLst[int(self.mat[i][j])]
                if (int(self._mat[i][j]) == 1):
                    rectDic['fillcolor'] = 'black'
                    rectDic['opacity'] = 0.8
                    self._shapeLst.append(rectDic)
                #                if(int(self.mat[i][j])==0):
                #                    rectDic['fillcolor'] = colorLst[int(self.mat[i][j])]
                #                getLevelColor(mat[i][j])
                else:
                    self._shapeLst.append(rectDic)

    def addRobotStartPnt(self, lst=[]):
        # print('xxxxx')
        for robID in range(len(lst)):
            startTrace = go.Scatter(x=[lst[robID][0]+0.5], y=[lst[robID][1]+0.5], mode='markers',
                                    marker=dict(symbol='cross-dot', size=20),
                                    name='Robot_' + str(robID +1))
            self._scatterLst.append(startTrace)
    def addEdges(self, lst=[]):
        mark_x = []
        mark_y = []
        for p in range(len(lst)):
            pnt0 = Pnt(lst[p][0] + 0.5, lst[p][1] + 0.5)
            pnt1 = Pnt(lst[p][2] + 0.5, lst[p][3] + 0.5)
            mark_x.append(pnt0.x)
            mark_x.append(pnt1.x)
            mark_y.append(pnt0.y)
            mark_y.append(pnt1.y)
            line = Line(pnt0, pnt1)
            lineDic = line.line2dict()
            #                print(randColor())
            lineDic['line']['color'] = 'darkred'
            lineDic['line']['width'] = 3
            self._shapeLst.append(lineDic)

        markTrace = go.Scatter(mode='markers',
                               x=mark_x,
                               y=mark_y,
                               marker=dict(size=3),
                               name='Spanning-Tree')
        self._scatterLst.append(markTrace)

    def addEdgesInPnt(self,lst = []):
        mark_x = []
        mark_y = []
        for p in range(len(lst)):
            pnt0 = Pnt(lst[p][0], lst[p][1])
            pnt1 = Pnt(lst[p][2], lst[p][3])
            mark_x.append(pnt0.x)
            mark_x.append(pnt1.x)
            mark_y.append(pnt0.y)
            mark_y.append(pnt1.y)
            line = Line(pnt0, pnt1)
            lineDic = line.line2dict()
            #                print(randColor())
            lineDic['line']['color'] = 'darkred'
            # lineDic['line']['color'] = 'rgba(15,15,15,0.5)'
            lineDic['line']['width'] = 3
            self._shapeLst.append(lineDic)

        markTrace = go.Scatter(mode='markers',
                               x=mark_x,
                               y=mark_y,
                               marker=dict(size=3),
                               name='Spanning-Tree')
        self._scatterLst.append(markTrace)

    def addMultiEdgesInPnt(self,edgePntLst = []):
        robNum = len(edgePntLst)
        # robNum = len(stcGraphLst)
        if robNum >2 :
            bupu = cl.scales[str(robNum)]['qual']['Dark2']
        else:
            bupu = cl.scales[str(3)]['qual']['Dark2']

        for robID in range(robNum):
            mark_x = []
            mark_y = []
            lst = edgePntLst[robID]
            for p in range(len(lst)):
                pnt0 = Pnt(lst[p][0], lst[p][1])
                pnt1 = Pnt(lst[p][2], lst[p][3])
                mark_x.append(pnt0.x)
                mark_x.append(pnt1.x)
                mark_y.append(pnt0.y)
                mark_y.append(pnt1.y)
                line = Line(pnt0, pnt1)
                lineDic = line.line2dict()
                #                print(randColor())
                lineDic['line']['color'] = bupu[robID]
                # lineDic['line']['color'] = 'rgba(15,15,15,0.5)'
                lineDic['line']['width'] = 3
                self._shapeLst.append(lineDic)

            markTrace = go.Scatter(mode='markers',
                                   x=mark_x,
                                   y=mark_y,
                                   marker=dict(size=5),
                                   name='Spanning Tree_'+ str(robID +1))
            self._scatterLst.append(markTrace)


    def addSinglePathInd(self,pathInd = []):

        x = [path_unit[0]+0.5 for path_unit in pathInd]
        y = [path_unit[1]+0.5 for path_unit in pathInd]

        markTrace = go.Scatter(mode='markers+lines',
                               x=x,
                               y=y,
                               # marker=dict(size=10),
                               name='Path_single' )
        self._scatterLst.append(markTrace)
    def addMultiPathInd(self, pathLst = []):
        # robNum
        for robID in range(len(pathLst)):
            path = pathLst[robID]
            x = [path_unit[0]+0.5 for path_unit in path]
            y = [path_unit[1]+0.5 for path_unit in path]

            markTrace = go.Scatter(mode='markers+lines',
                                   x=x,
                                   y=y,
                                   # marker=dict(size=10),
                                   name='Path_' + str(robID +1) )

            self._scatterLst.append(markTrace)
    def addMidPosLst(self, pathLst = []):
        for robID in range(len(pathLst)):
            path = pathLst[robID]
            x = [path_unit[0]+0.5 for path_unit in path]
            y = [path_unit[1]+0.5 for path_unit in path]
            markTrace = go.Scatter(mode='markers',
                                   x=x,
                                   y=y,
                                   marker=dict(symbol='square-dot', size=20),
                                   name='midPos_' + str(robID +1) )
            self._scatterLst.append(markTrace)
            if False:
                for i in range(len(x)):
                    self._annotationsLst.append(dict(showarrow=False,
                                                     x=x[i], y=y[i],
                                                     text=str(i)))

    def addSTCGraph(self,stcGraphLst):
        g_color = 'blue'
        robNum = len(stcGraphLst)
        if robNum >2 :
            bupu = cl.scales[str(robNum)]['qual']['Set3']
        else:
            bupu = cl.scales[str(3)]['qual']['Set3']

        for robID in range(robNum):
            stcGraph =  stcGraphLst[robID]
            for pos in stcGraph:
                pnt = Pnt(pos[0],pos[1])
                rect = Rect(pnt,2,2)
                rectDic = rect.rect2dict()
                rectDic['line']['color'] = g_color
                rectDic['line']['width'] = 1
                rectDic['fillcolor'] = bupu[robID]
                rectDic['opacity'] = 0.3
                self._shapeLst.append(rectDic)
                if False:
                    self._annotationsLst.append(dict(showarrow=False,
                                                     x=pos[0] + 1,
                                                     y=pos[1] + 1,
                                                     text= str(robID +1 )))

    def addSTCNeiGraph(self,stcGraphLst):
        g_color = 'blue'
        robNum = len(stcGraphLst)
        if robNum > 2:
            bupu = cl.scales[str(robNum)]['qual']['Paired']
        else:
            bupu = cl.scales[str(11)]['qual']['Paired']
        # print(bupu)
        for robID in range(robNum):
            stcGraph =  stcGraphLst[robID]
            for pos in stcGraph:
                pnt = Pnt(pos[0],pos[1])
                rect = Rect(pnt,2,2)
                rectDic = rect.rect2dict()
                rectDic['line']['color'] = g_color
                rectDic['line']['width'] = 1
                rectDic['fillcolor'] = bupu[robID]
                rectDic['opacity'] = 0.25
                self._shapeLst.append(rectDic)
                if True:
                    self._annotationsLst.append(dict(showarrow = False,
                                                     x = pos[0] + 1,
                                                     y = pos[1] + 1,
                                                     text = 'n'+ str(robID + 1)))

    def drawPic(self, fileName='env', titleName = None, showBoolean = True,saveBoolean = False,):
        layout = dict()
        layout['shapes'] = self._shapeLst
        # layout['xaxis'] = dict(showgrid=False)

        layout['xaxis'] = dict(
            autorange=True,
            showgrid=False,
            zeroline=False,
            showline=False,
            # autotick=True,
            ticks='',
            showticklabels=False)

        layout['yaxis'] = dict(
            scaleanchor="x",
            autorange=True,
            showgrid=False,
            zeroline=False,
            showline=False,
            # autotick=True,
            ticks='',
            showticklabels=False)
        layout['xaxis']['range'] = [0, len(self._mat[0])]
        layout['yaxis']['range'] = [0, len(self._mat)]

        layout['font'] = dict(
            family='sans-serif',
            size=25,
            color='#000'
        )
        layout['autosize'] = False
        layout['height'] = 1000
        layout['width'] = 1000
        layout['template'] = "plotly_white"
        layout['annotations'] = self._annotationsLst
        if titleName != None:
            layout['title'] = go.layout.Title(text = titleName)

        # layout['annotations'] = self.annotations
        #        print(layout)
        fig = go.Figure(data = self._scatterLst, layout = layout)

        if showBoolean:
            plotly.offline.plot(fig, filename= fileName +'.html')
            # fig.show()
        if saveBoolean:
            fig.write_image( fileName +'.pdf')




def drawPic(ins: MCPPinstance.MCPPInstance, singlePathInd = None, edgeLst = None, fileName='env', titleName = None
            , multiPath = None, midPosLst = None):
    env = Env(ins._mat)
    env.addgrid()
    env.addRobotStartPnt(ins._robPosLst)
    if singlePathInd != None:
        env.addSinglePathInd(singlePathInd)
        # pass
    if edgeLst != None:
        # raise Exception('xxx')
        env.addEdges(edgeLst)
    else:
        pass
    if multiPath != None:
        env.addMultiPathInd(multiPath)
    if midPosLst != None:
        env.addMidPosLst(midPosLst)
        # raise  Exception('ssss')
    env.drawPic(fileName = fileName, titleName = titleName, saveBoolean = True, showBoolean = True)

def drawSTCPic(ins: MCPPinstance.MCPPInstance, edgePntLst = None):
    env = Env(ins._mat)
    env.addgrid()
    env.addRobotStartPnt(ins._robPosLst)
    env.addEdgesInPnt(edgePntLst)
    env.drawPic(fileName= 'pic', saveBoolean = False)

def drawSTCGraph(ins: MCPPinstance.MCPPInstance, fileName = 'path' ,stcGraphLst = None,
                 stcNeiGraphLst = None, edgePntLst = None, multiPath = None):
    '''
    :param ins:
    :param stcGraphLst:
    :param stcNeiGraphLst:
    :return:
    '''
    env = Env(ins._mat)
    env.addgrid()
    env.addRobotStartPnt(ins._robPosLst)
    if stcGraphLst != None:
        env.addSTCGraph(stcGraphLst)
    if stcNeiGraphLst != None:
        env.addSTCNeiGraph(stcNeiGraphLst)
    if edgePntLst != None:
        env.addMultiEdgesInPnt(edgePntLst)
    if multiPath != None:
        env.addMultiPathInd(multiPath)
    env.drawPic(fileName= fileName)



def drawEvalSTCGraph(ins: MCPPinstance.MCPPInstance, stcGraphLst = None,
                     stcNeiGraphLst = None, edgePntLst = None, multiPath = None):
    '''
    :param ins:
    :param stcGraphLst:
    :param stcNeiGraphLst:
    :return:
    '''
    env = Env(ins._mat)
    env.addgrid()
    env.addRobotStartPnt(ins._robPosLst)
    if stcGraphLst != None:
        env.addSTCGraph(stcGraphLst)
    if stcNeiGraphLst != None:
        env.addSTCNeiGraph(stcNeiGraphLst)
    if edgePntLst != None:
        env.addMultiEdgesInPnt(edgePntLst)
    if multiPath != None:
        env.addMultiPathInd(multiPath)
    env.drawPic(fileName= 'realPath2')

if __name__ == '__main__':

    row = 40
    col = 40
    robNum = 2
    p = np.array([0.9,0.1])
    np.random.seed(1000)
    rob_x = np.random.randint(20,size = robNum)
    rob_y = np.random.randint(20,size = robNum)
    robPosLst = []
    for i in range(robNum):
        robPosLst.append((rob_x[i],rob_y[i]))
    # 1 means obstacles
    # 0 means way
    print(robPosLst)
    obstacleLst = []
    for rowInd in range(row):
        for colInd in range(col):
            if (rowInd,colInd) in robPosLst:
                obstacleLst.append(0)
            else:
                obstacleLst.append(np.random.choice([0,1],p =p.ravel()))
    _row = row
    _col = col
    _obstacleLst = obstacleLst
    _robNum = len(robPosLst)
    _mat = np.zeros([_row, _col])
    # print(self._mat)
    ind = 0
    for rowInd in range(row):
        for colInd in range(col):
            _mat[rowInd][colInd] = _obstacleLst[ind]
            ind = ind + 1

    env = Env(_mat)
    env.addgrid()
    env.addRobotStartPnt(robPosLst)
    env.drawSTCGraph(showBoolean= True, saveBoolean= False)

    print('ss')


