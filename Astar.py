#-*-coding:utf-8-*-
# Function: Astar algorithm
#? AStar算法实现模组
#TODO Version 0.1.20230714
#! 依赖项目：numba
#! 被引用：main.py


#* A*算法地图存储
class Map(object):
    def __init__(self,mapdata,startx,starty,endx,endy):
        self.data = mapdata
        self.startx = startx
        self.starty = starty
        self.endx = endx
        self.endy = endy


#* A*算法Node类
class MapNode(object): 
    #* 初始化节点信息
    def __init__(self,x,y,g,h,father):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.father = father
    #* 处理边界和障碍点 返回可以从当前节点到达的相邻节点列表
    def getNeighbor(self,mapdata,endx,endy):
        x = self.x
        y = self.y
        result = []
    #先判断是否在上下边界
    #上
        if(x!=0 and mapdata[x-1][y]!=0):
            upNode = MapNode(x-1,y,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(upNode)
    #下
        if(x!=len(mapdata)-1 and mapdata[x+1][y]!=0):
            downNode = MapNode(x+1,y,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(downNode)
    #左
        if(y!=0 and mapdata[x][y-1]!=0):
            leftNode = MapNode(x,y-1,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(leftNode)
    #右
        if(y!=len(mapdata[0])-1 and mapdata[x][y+1]!=0):
            rightNode = MapNode(x,y+1,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(rightNode)
        return result
    #* 判断当前节点是否在列表中
    def hasNode(self,worklist):
        for i in worklist:
            if(i.x==self.x and i.y ==self.y):
                return True
        return False
    #* 找到到达该节点的更好路径时更新节点的 g 值
    def changeG(self,worklist):
        for i in worklist:
            if(i.x==self.x and i.y ==self.y):
                if(i.g>self.g):
                    i.g = self.g


# element排序函数，已使用匿名函数改写
# def getKeyforSort(element:MapNode):
#     return element.g #element#不应该+element.h，


#* A*算法主函数 
def astar(workMap):
    startx,starty = workMap.startx,workMap.starty
    endx,endy = workMap.endx,workMap.endy
    startNode = MapNode(startx, starty, 0, 0, None)
    openList = [];lockList = []
    lockList.append(startNode)
    currNode = startNode
    while((endx,endy) != (currNode.x,currNode.y)):
        workList = currNode.getNeighbor(workMap.data,endx,endy)
        for i in workList:
            if (i not in lockList):
                if(i.hasNode(openList)):
                    i.changeG(openList)
                else:
                    openList.append(i)
        # openList.sort(key=getKeyforSort) # 关键步骤
        openList=sorted(openList,key=lambda element:element.g)
        currNode = openList.pop(0)
        lockList.append(currNode)
    result = []
    while(currNode.father!=None):
        result.append((currNode.x,currNode.y))
        currNode = currNode.father
    result.append((currNode.x,currNode.y))
    return result
