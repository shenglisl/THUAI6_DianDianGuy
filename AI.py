import PyAPI.structures as THUAI6
from PyAPI.Interface import IStudentAPI, ITrickerAPI, IAI
from typing import Union, Final, cast, List
from PyAPI.constants import Constants
import queue
import time


# === ---asdawej--- ===


from PyAPI.Interface import IAPI
from typing import Tuple
from enum import Enum
from math import pi, atan2, sqrt, sin, cos
import random


# =====================


class Setting:
    # 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新
    @staticmethod
    def asynchronous() -> bool:
        return True

    # 选手需要依次将player0到player4的职业都定义
    @staticmethod
    def studentType() -> List[THUAI6.StudentType]:
        return [THUAI6.StudentType.Athlete,
                THUAI6.StudentType.Teacher,
                THUAI6.StudentType.StraightAStudent,
                THUAI6.StudentType.Sunshine]

    @staticmethod
    def trickerType() -> THUAI6.TrickerType:
        return THUAI6.TrickerType.Assassin


# 辅助函数
numOfGridPerCell: Final[int] = 1000


class AssistFunction:

    @staticmethod
    def CellToGrid(cell: int) -> int:
        return cell*numOfGridPerCell+numOfGridPerCell//2

    @staticmethod
    def GridToCell(grid: int) -> int:
        return grid//numOfGridPerCell


# === ---asdawej--- ===

G2C = AssistFunction.GridToCell
C2G = AssistFunction.CellToGrid


# 可以走的方格
set_path: set = {THUAI6.PlaceType.NullPlaceType,
                 THUAI6.PlaceType.Land,
                 THUAI6.PlaceType.Grass,
                 THUAI6.PlaceType.Window}


# >>> 方向


# 方向枚举型
class Direction:
    l = -pi/2
    r = pi/2
    u = pi
    d = 0
    lu = -3*pi/4
    ld = -pi/4
    ru = 3*pi/4
    rd = pi/4


def direction(x1: int, y1: int, x2: int, y2: int) -> float:
    '按照grid向量计算方向'
    return atan2(y2-y1, x2-x1)


def antiDirection(dir: float) -> float:
    '取反方向'
    if dir == 0:
        return pi
    elif dir == pi:
        return 0
    elif dir < 0:
        return dir+pi
    elif dir > 0:
        return dir-pi


def midDirection(*dirs: float) -> float:
    '求夹在两个方向中间的方向'
    return atan2(sum([sin(y) for y in dirs]), sum([cos(x) for x in dirs]))


# <<<


def dist(x1: int, y1: int, x2: int, y2: int) -> float:
    'grid距离'
    return sqrt((x2-x1)**2+(y2-y1)**2)


def isLine(x1: int, y1: int, x2: int, y2: int) -> bool:
    '按照grid向量分析是否走直线'
    for i in range(x1, x2+1, numOfGridPerCell):
        for j in range(y1, y2+1, numOfGridPerCell):
            if not mapAbstract[G2C(i)]:
                return False
    return True


def randomWonder(api: IAPI) -> None:
    '随机走动'
    pass


# >>> 地图抽象


bool_mapAbstrct: bool = True
mapAbstract: List[List[bool]] = None


class MapBlocks:
    ClassRoom: List[Tuple[int]] = []
    Gate: List[Tuple[int]] = []
    HiddenGate: List[Tuple[int]] = []
    Window: List[Tuple[int]] = []
    Door3: List[Tuple[int]] = []
    Door5: List[Tuple[int]] = []
    Door6: List[Tuple[int]] = []
    Chest: List[Tuple[int]] = []


def _mapAbstract(api: IAPI) -> None:
    '''
    最开始时获取全地图并区分障碍和可以走的地方\n
    只运行一次
    '''
    global bool_mapAbstrct, mapAbstract
    map: List[List[THUAI6.PlaceType]] = api.GetFullMap()
    mapAbstract = []
    for i in range(Constants.rows):
        mapAbstract.append([])
        for j in range(Constants.cols):
            mapAbstract[i].append(map[i][j] in set_path)
            if map[i][j] == THUAI6.PlaceType.ClassRoom:
                MapBlocks.ClassRoom.append((i, j))
            elif map[i][j] == THUAI6.PlaceType.Gate:
                MapBlocks.Gate.append((i, j))
            elif map[i][j] == THUAI6.PlaceType.HiddenGate:
                MapBlocks.HiddenGate.append((i, j))
            elif map[i][j] == THUAI6.PlaceType.Window:
                MapBlocks.Window.append((i, j))
            elif map[i][j] == THUAI6.PlaceType.Door3:
                MapBlocks.Door3.append((i, j))
            elif map[i][j] == THUAI6.PlaceType.Door5:
                MapBlocks.Door5.append((i, j))
            elif map[i][j] == THUAI6.PlaceType.Door6:
                MapBlocks.Door6.append((i, j))
            elif map[i][j] == THUAI6.PlaceType.Chest:
                MapBlocks.Chest.append((i, j))
    bool_mapAbstrct = False


# <<<


def isPath(Cx: int, Cy: int) -> bool:
    '是否可以行走'
    if (Cx < 0 or Cx >= Constants.rows or
            Cy < 0 or Cy >= Constants.cols):
        return False
    return mapAbstract[Cx][Cy]


# >>> 墙检测


def isSurrWall(api: IAPI) -> bool:
    '是否周围有墙'
    Cx, Cy = G2C(api.GetSelfInfo().x), G2C(api.GetSelfInfo().y)
    walls = (not isPath(Cx, Cy-1), not isPath(Cx, Cy+1), not isPath(Cx-1, Cy), not isPath(Cx+1, Cy))
    return bool(len([x for x in walls if x]))


def dirSurrWindow(api: IAPI) -> float:
    '返回窗户方向'
    Cx, Cy = G2C(api.GetSelfInfo().x), G2C(api.GetSelfInfo().y)
    window = THUAI6.PlaceType.Window
    if api.GetFullMap()[Cx][Cy-1] == window:
        return Direction.l
    elif api.GetFullMap()[Cx][Cy+1] == window:
        return Direction.r
    elif api.GetFullMap()[Cx-1][Cy] == window:
        return Direction.u
    elif api.GetFullMap()[Cx+1][Cy] == window:
        return Direction.d
    else:
        return 114514


def dirWall(api: IAPI) -> float:
    '返回墙的方向向量'
    Cx, Cy = G2C(api.GetSelfInfo().x), G2C(api.GetSelfInfo().y)
    walls = (not isPath(Cx, Cy-1), not isPath(Cx, Cy+1), not isPath(Cx-1, Cy), not isPath(Cx+1, Cy))
    match len([x for x in walls if x]):
        case 1:
            return {(1, 0, 0, 0): Direction.l,
                    (0, 1, 0, 0): Direction.r,
                    (0, 0, 1, 0): Direction.u,
                    (0, 0, 0, 1): Direction.d}[walls]
        case 2:
            if walls == (1, 1, 0, 0):
                if api.GetSelfInfo().y//numOfGridPerCell < numOfGridPerCell//2:
                    return Direction.l
                else:
                    return Direction.r
            elif walls == (0, 0, 1, 1):
                if api.GetSelfInfo().x//numOfGridPerCell < numOfGridPerCell//2:
                    return Direction.u
                else:
                    return Direction.d
            else:
                return {(1, 0, 1, 0): Direction.lu,
                        (1, 0, 0, 1): Direction.ld,
                        (0, 1, 1, 0): Direction.ru,
                        (0, 1, 0, 1): Direction.rd}[walls]
        case 3:
            return {(1, 1, 1, 0): Direction.u,
                    (1, 1, 0, 1): Direction.d,
                    (1, 0, 1, 1): Direction.l,
                    (0, 1, 1, 1): Direction.r}[walls]


def outStuckCorner(api: IAPI) -> float:
    '墙角脱出'
    Cx, Cy = G2C(api.GetSelfInfo().x), G2C(api.GetSelfInfo().y)
    corners: List[Tuple[int]] = []
    if not isPath(Cx-1, Cy-1):
        corners.append((Cx-1, Cy-1))
    elif not isPath(Cx+1, Cy-1):
        corners.append((Cx+1, Cy-1))
    elif not isPath(Cx-1, Cy+1):
        corners.append((Cx-1, Cy+1))
    elif not isPath(Cx+1, Cy+1):
        corners.append((Cx+1, Cy+1))
    dir_corners = []
    if len(corners):
        for corn in corners:
            dir_corners.append(direction(C2G(corn[0]), C2G(corn[1]), C2G(Cx), C2G(Cy)))
            if api.GetSelfInfo().radius-dist(api.GetSelfInfo().x, api.GetSelfInfo().y, C2G(corn[0]), C2G(corn[1])) >= 0:
                api.Move(100/api.GetSelfInfo().speed*1000, dir_corners[-1])
                t0 = time.time_ns()
                while time.time_ns()-t0 <= 100/api.GetSelfInfo().speed*10**9:
                    print(time.time_ns())
        return midDirection(*dir_corners)
    else:
        return 114514


# <<<


# >>> 地图采样建图
pathDensity: Final[int] = 100           # 取样数
sampleNodes: List[Tuple[int]] = []      # 取样点集


def _sample() -> None:
    '''
    地图采样, 用于采样寻路
    '''
    global sampleNodes
    _n = 0
    while _n != pathDensity:
        _Cx, _Cy = random.randrange(0, Constants.rows), random.randrange(0, Constants.cols)
        if (not (_Cx, _Cy) in sampleNodes) and isPath(_Cx, _Cy):
            sampleNodes.append((_Cx, _Cy))
            _n += 1


class PathNode:
    def __init__(self, loc: Tuple[int], neighbours: List['PathNode'] = None):
        self.Cloc = loc
        self.loc = (C2G(loc[0]), C2G(loc[1]))
        if neighbours:
            self.neighbours = neighbours
        else:
            self.neighbours = []


neighbour_dist = numOfGridPerCell*sqrt((Constants.rows*Constants.cols)/(pathDensity*pi))    # 相邻间距


def isNeighbour(loc1: Tuple[int], loc2: Tuple[int]) -> bool:
    '根据grid距离确定是否相邻'
    return dist(C2G(loc1[0]), C2G(loc1[0]),
                C2G(loc2[0]), C2G(loc2[1])) <= neighbour_dist


# 取样点图
sampleGraph: List[PathNode] = [PathNode(node, [nd for nd in sampleNodes
                                               if (nd != node and
                                                   isLine(*node, *nd) and
                                                   isNeighbour(node, nd))])
                               for node in sampleNodes]


# <<<


# >>> 采样寻路


def samplePath(x: int, y: int, targetx: int, targety: int) -> List[Tuple[int]]:
    '''
    采样寻路算法, 输入grid坐标和目标grid坐标, 返回grid坐标队列
    '''
    pass


# <<<


# >>> 曼哈顿寻路


def MHTDist(Cx: int, Cy: int, targetCx: int, targetCy: int) -> int:
    return abs(Cx-targetCx)+abs(Cy-targetCy)


class MHTNode:
    def __init__(self, Cx: int, Cy: int,
                 targetCx: int, targetCy: int,
                 parent: 'MHTNode' = None):
        self.loc = (Cx, Cy)
        self.parent = parent
        if parent:
            self.g = self.parent.g+1
        else:
            self.g = 0
        self.h = MHTDist(Cx, Cy, targetCx, targetCy)+self.g

    def __eq__(self, other: 'MHTNode') -> bool:
        return self.loc == other.loc


def MHTPath(Cx: int, Cy: int, targetCx: int, targetCy: int) -> List[Tuple[int]]:
    '''
    曼哈顿算法, 返回cell坐标队列
    '''
    _open: List[MHTNode] = []
    _close: List[MHTNode] = [MHTNode(Cx, Cy, targetCx, targetCy)]
    _head = _close[0]
    while True:
        if MHTDist(*(_head.loc), targetCx, targetCy) == 1:
            break
        # 将四周纳入
        temp_list: List[MHTNode] = []
        if isPath(_head.loc[0], _head.loc[1]-1) and not (l_node := MHTNode(_head.loc[0], _head.loc[1]-1, targetCx, targetCy,  _head)) in _close:
            temp_list.append(l_node)
        if isPath(_head.loc[0], _head.loc[1]+1) and not (r_node := MHTNode(_head.loc[0], _head.loc[1]+1, targetCx, targetCy, _head)) in _close:
            temp_list.append(r_node)
        if isPath(_head.loc[0]-1, _head.loc[1]) and not (u_node := MHTNode(_head.loc[0]-1, _head.loc[1], targetCx, targetCy, _head)) in _close:
            temp_list.append(u_node)
        if isPath(_head.loc[0]+1, _head.loc[1]) and not (d_node := MHTNode(_head.loc[0]+1, _head.loc[1], targetCx, targetCy, _head)) in _close:
            temp_list.append(d_node)
        # 估价决策
        for _node in temp_list:
            flag = False
            for _open_node in _open:
                if _node == _open_node:
                    if _open_node.h > _node.h:
                        _open_node.parent = _node.parent
                        _open_node.g = _node.g
                        _open_node.h = _node.h
                    flag = True
                    break
            if flag:
                continue
            _open.append(_node)
        # _head移动
        _open.sort(key=lambda node: node.h)
        _head = _open.pop(0)
        _close.append(_head)
        if MHTDist(*(_head.loc), targetCx, targetCy) == 1:
            break
    # 生成路径
    res: list[Tuple[int]] = []
    while _head.parent:
        res.append(_head.loc)
        _head = _head.parent
    return list(reversed(res))


# <<<


def miniPath(x: int, y: int, targetx: int, targety: int) -> Tuple[float, Tuple[int]]:
    '''
    局部寻路算法, 返回下一步应走的最优方向
    '''
    Cx, Cy = G2C(x), G2C(y)
    targetCx, targetCy = G2C(targetx), G2C(targety)
    mini = MHTPath(Cx, Cy, targetCx, targetCy)
    _x, _y = [C2G(t) for t in mini[0]]
    return direction(x, y, _x, _y), (_x, _y)


# =====================


class AI(IAI):
    trickerWindowTime0 = time.time()
    trickerTarget = (3, 3)
    bool_INIT_loc = True
    Cx, Cy = 0, 0

    def __init__(self, pID: int):
        self.__playerID = pID

    def StudentPlay(self, api: IStudentAPI) -> None:
        if self.__playerID == 0:
            return
        elif self.__playerID == 1:
            return
        elif self.__playerID == 2:
            return
        elif self.__playerID == 3:
            return
        return

    def TrickerPlay(self, api: ITrickerAPI) -> None:
        # api.EndAllAction()
        selfInfo = api.GetSelfInfo()
        api.PrintSelfInfo()
        # 初始化扫描地图
        if bool_mapAbstrct:
            _mapAbstract(api)
            _sample()
            AI.trickerTarget = MapBlocks.Door6[-1]
        if AI.bool_INIT_loc:
            AI.Cx, AI.Cy = G2C(selfInfo.x), G2C(selfInfo.y)
            AI.bool_INIT_loc = False
        # 决策移动角
        # selfInfo.facingDirection, next_node = miniPath(selfInfo.x, selfInfo.y, C2G(AI.trickerTarget[0]), C2G(AI.trickerTarget[1]))
        # if not isLine(selfInfo.x, selfInfo.y, *next_node):
        #     if next_node[1]<selfInfo.y:

        #     elif next_node[1]>selfInfo.y:
        if dist(selfInfo.x, selfInfo.y, C2G(G2C(selfInfo.x)), C2G(G2C(selfInfo.y))) < 50:
            AI.Cx, AI.Cy = G2C(selfInfo.x), G2C(selfInfo.y)
        path = MHTPath(AI.Cx, AI.Cy, *AI.trickerTarget)
        selfInfo.facingDirection = direction(AI.Cx, AI.Cy, *path[0])
        # api.Move(dist(selfInfo.x, selfInfo.y, C2G(path[0][0]), C2G(path[0][1]))/selfInfo.speed, selfInfo.facingDirection)
        api.Move(10, selfInfo.facingDirection)
        time.sleep(0.01)
        # if isSurrWall(api):
        #     _dir_rv1 = antiDirection(dirWall(api))
        #     # if (_dir_rv2 := outStuckCorner(api)) != 114514:
        #     #     _dir_rv1 = midDirection(_dir_rv1, antiDirection(_dir_rv2))
        #     selfInfo.facingDirection = midDirection(selfInfo.facingDirection, _dir_rv1)
        # 移动
        # if (selfInfo.x, selfInfo.y) == AI.trickerLastLoc:
        #     api.EndAllAction()
        #     outStuckCorner(api)
        # AI.trickerLastLoc = (selfInfo.x, selfInfo.y)
        # # #
        # Cx, Cy = G2C(selfInfo.x), G2C(selfInfo.y)
        # path = MHTPath(Cx, Cy, *AI.trickerTarget)
        # print(path)
        # if path[0][1] < Cy:
        #     api.Move(50, Direction.l)
        #     time.sleep(0.05)
        # elif path[0][1] > Cy:
        #     api.Move(50, Direction.r)
        #     time.sleep(0.05)
        # elif path[0][0] < Cx:
        #     api.Move(50, Direction.u)
        #     time.sleep(0.05)
        # elif path[0][0] > Cx:
        #     api.Move(50, Direction.d)
        #     time.sleep(0.05)
        # # #
        if time.time()-AI.trickerWindowTime0 > 2:
            if dirSurrWindow(api) != 114514:
                api.SkipWindow()
                AI.trickerWindowTime0 = time.time()
        # # #
        # if dirSurrWindow(api) != 114514:
        #     if abs(selfInfo.facingDirection-dirSurrWindow(api)) <= pi/2:
        #         api.SkipWindow()
        return
