import PyAPI.structures as THUAI6
from PyAPI.Interface import IStudentAPI, ITrickerAPI, IAI
from typing import Union, Final, cast, List
from PyAPI.constants import Constants
import queue
import time


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


class AI(IAI):
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
        selfInfo = api.GetSelfInfo()
        api.PrintSelfInfo()
        return