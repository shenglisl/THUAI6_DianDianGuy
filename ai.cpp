#include <vector>
#include <thread>
#include <queue>
#include <array>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include "AI.h"
#include "constants.h"

#define PI 3.1415
// 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新
extern const bool asynchronous = false;

// 选手需要依次将player0到player4的职业在这里定义

extern const std::array<THUAI6::StudentType, 4> studentType = {
    THUAI6::StudentType::StraightAStudent,
    THUAI6::StudentType::StraightAStudent,
    THUAI6::StudentType::StraightAStudent,
    THUAI6::StudentType::Teacher };

extern const THUAI6::TrickerType trickerType = THUAI6::TrickerType::Klee;
// 记录函数

// 定义非常多的状态（有限状态机）
enum class status
{
    initial,
    watch,
    sorround,
    idle,
    reset,
    index,
    retreat,
    avoid,
    move
};

static status BotStatus = status::idle;
static status LastStatus = status::reset;
// 存储路径的常量
const int MAXN = 50;
int a[MAXN][MAXN]; // 存储地图
bool blank[MAXN][MAXN];
int vis[MAXN][MAXN];    // 标记数组，记录每个点是否被访问过
int dis[MAXN][MAXN];    // 记录每个点到起点的最短距离
int pre[MAXN][MAXN][2]; // 记录每个点的前驱节点
// 定义坐标结构体

struct Point
{
    int x, y;
    Point() {}
    Point(int x, int y) : x(x), y(y) {}
};
// 定义队列元素结构体
struct Node
{
    Point p;
    int dist;
    Node(int d) { dist = d; }
    Node(Point p, int dist) : p(p), dist(dist) {}
    bool operator<(const Node& other) const
    { // 定义优先级，距离更小的优先级更高
        return dist > other.dist;
    }
};
// 状态机函数
void playerBot(IStudentAPI& api);
void teacherBot(IStudentAPI& api);
void moveStatus(IStudentAPI& api);
void retreatStatus(IStudentAPI& api);
void initialStatus(IStudentAPI& api);
void idleStatus(IStudentAPI& api);

// 可以在AI.cpp内部声明变量与函数

// 函数

// 循迹相关
double Distance(Point, Point);
std::queue<Point> bfs(Point, Point);
void Goto(IStudentAPI&, double, double, double); // randAngle = 1，则取波动范围为-0.5pi-0.5pi
void InitMapForMove(IAPI&);
// void initHwGroup();
void arrayClear();
int pathLen(Point, Point);

// 状态检查类
bool isSurround(IStudentAPI&, double, double);
bool isArround(IStudentAPI&, int, int);
bool stuckCheck(IStudentAPI&, int); // 注意，n必须在2-10之间
bool progressStuckCheck(int, int);
bool isTrigger(StudentAPI&, Point); // 学生和目标点是否在九宫格内

// debug相关
void printPathType(IStudentAPI&, std::queue<Point>);
void printQueue(std::queue<Point> q);
void printPosition(IStudentAPI&);
void printPointVector(std::vector<Point>);

// 决策相关

// void groupJuan(IStudentAPI& api);
void closestJuan(IStudentAPI& api);
void graduate(IStudentAPI& api);

// 发送信息相关
void receiveMessage(IStudentAPI& api);
void sendAndReceiveMessage(IStudentAPI& api);

// 爬窗开关门相关
bool isWindowInPath(IStudentAPI& api, std::queue<Point> q);
bool isSurroundWindow(IStudentAPI& api);
bool isDelayedAfterWindow(IStudentAPI& api);
void isDoorClosed(IStudentAPI& api);
// 躲避tricker相关
Point findNearestPoint(IStudentAPI& api, int n);
double tricker_distance(IStudentAPI& api);
bool isTrickerInsight(IStudentAPI& api);
bool isStudentInsight(IStudentAPI& api);
bool isAvoid;
// 惩罚tricker相关
void gotoTricker(IStudentAPI& api);
void gotoFarestStudent(IStudentAPI& api);
void gotoNearestStudent(IStudentAPI& api);

// 辅助函数相关
int findMaxInArray(double a[], int size);
int findMinInArray(double a[], int size);
// 变量

// 循迹相关变量
static int64_t myPlayerID;
static THUAI6::PlaceType map[50][50];
static int steps;
static int hasinitmap;
static int hasBeenTo;
static bool hasInitMap = false;
static bool IHaveArrived = false;
static int lastX = 0, lastY = 0;
static int lastFrameCount = 0;
std::queue<Point> path;
double derectionBeforeRetreat;

// stuckCheck()相关变量
int32_t memoryX[10];
int32_t memoryY[10];
std::chrono::system_clock::time_point stuckCheckStartTime;

// progressStuckCheck()相关变量
int32_t memoryProgress[10];

// 目标坐标
Point targetP = Point(12, 3);

// 特殊点坐标
std::vector<Point> hw;
std::vector<Point> door;
std::vector<Point> window;
std::vector<Point> gate;
std::vector<Point> chest;
std::vector<Point> ClassRoom1;
std::vector<Point> ClassRoom2;
std::vector<Point> ClassRoom3;
// 决策相关变量
// 总决策函数相关变量
int decision;
static bool isReinitial;
static bool isReinitial2;
static bool isReinitial3;
/*
decision == 1 写作业
decision == 2 毕业
decision == 3 缓冲
decision == 4 勉励
decision == 5 去tricker身边
decision == 6 去学生身边
...
*/
// 躲避Tricker相关变量
static bool isReinitialForFarMove;
int avoidCount;
static bool formerAvoidState;
int formerState;
int tricker_x;
int tricker_y;
Point farthestHw;
// groupJuan()相关变量
/*
int hwGroup1Index[5] = { 0,1,3,5,7 };
int hwGroup2Index[5] = { 2,4,6,8,9 };
std::vector<Point> hwGroup1;
std::vector<Point> hwGroup2;
*/
// graduate()相关变量
bool isGraduate = false;
static int framecount = 0; // 计数帧数
// rouse()相关变量
int rouseTarget = -1;
// 爬窗相关变量
static bool isCrossingWindow = 0;
static int CrossWindowCount = 0;
static bool climbingCheck;
static int climbingFrameCount;
// 通信相关变量
static bool isSendMessage;
static bool hwIsFinished[10];
static int playerDecision[4];
static int playerState[4];
static int playerHP[4];
static Point playerPosition[4];
static Point trickerPosition;
static int trickerTypeInt;
// static int player
void AI::play(IStudentAPI& api)
{
    // 公共操作
    myPlayerID = this->playerID;
    if (this->playerID == 0)
    {
        // 玩家0执行操作
        playerBot(api);
    }
    else if (this->playerID == 1)
    {
        // 玩家1执行操作
        playerBot(api);
    }
    else if (this->playerID == 2)
    {
        // 玩家2执行操作
        playerBot(api);
    }
    else if (this->playerID == 3)
    {
        // 玩家3执行操作
        teacherBot(api);
    }
    // 当然可以写成if (this->playerID == 2||this->playerID == 3)之类的操作
    //  公共操作
}

void AI::play(ITrickerAPI& api)
{
    auto self = api.GetSelfInfo();
    api.PrintSelfInfo();
    // api.MoveLeft(1000);
}
void arrayClear()
{
    int i, j;
    for (i = 0; i < 50; i++)
    {
        for (j = 0; j < 50; j++)
        {
            vis[i][j] = 0;
            dis[i][j] = 0;
            pre[i][j][0] = 0;
            pre[i][j][1] = 0;
        }
    }
}
void searchingArrayClear()
{
    int i, j;
    for (i = 0; i < 50; i++)
    {
        for (j = 0; j < 50; j++)
        {
            vis[i][j] = 0;
            dis[i][j] = 0;
            pre[i][j][0] = 0;
            pre[i][j][1] = 0;
        }
    }
}
void InitMapForMove(IAPI& api)
{
    int i, j;
    for (i = 0; i < 50; i++)
    {
        for (j = 0; j < 50; j++)
        {
            a[i][j] = (int)api.GetPlaceType(i, j) - 1;
            if (a[i][j] >= 10)
                a[i][j] = 9;
            if (a[i][j] == 2)
                a[i][j] = 0;
            std::cout << a[i][j];
            if ((int)api.GetPlaceType(i, j) == 4)
            {
                hw.push_back(Point(i, j));
            }
            else if ((int)api.GetPlaceType(i, j) == 8)
            {
                door.push_back(Point(i, j));
            }
            else if ((int)api.GetPlaceType(i, j) == 9)
            {
                door.push_back(Point(i, j));
            }
            else if ((int)api.GetPlaceType(i, j) == 10)
            {
                door.push_back(Point(i, j));
            }
            else if ((int)api.GetPlaceType(i, j) == 7)
            {
                window.push_back(Point(i, j));
            }
            else if ((int)api.GetPlaceType(i, j) == 5)
            {
                gate.push_back(Point(i, j));
            }
            else if ((int)api.GetPlaceType(i, j) == 11)
            {
                chest.push_back(Point(i, j));
            }
        }
        std::cout << std::endl;
    }

    for (int i = 0; i < door.size(); i++)
    {
        a[door[i].x][door[i].y] = 0;
    }
    for (int i = 0; i < window.size(); i++)
    {
        a[window[i].x][window[i].y] = 0;
    }
    api.Wait();
}
/*
void initHwGroup()
{
    for (int i = 0; i < 5; i++)
    {
        hwGroup1.emplace_back(hw[hwGroup1Index[i]]);
        hwGroup2.emplace_back(hw[hwGroup2Index[i]]);
    }
}
*/
// 搜索最短路径
std::queue<Point> bfs(Point start, Point end)
{
    searchingArrayClear();
    std::queue<Point> path;
    std::priority_queue<Node> q;
    q.push(Node(start, 0));
    vis[start.x][start.y] = 1;
    std::cout << " check1 " << std::endl;
    while (!q.empty())
    {
        Node cur = q.top();
        q.pop();
        Point p = cur.p;
        int dist = cur.dist;
        if (p.x == end.x && p.y == end.y)
        { // 找到终点
            path.push(p);
            while (p.x != start.x || p.y != start.y)
            { // 回溯路径
                Point pre_p = Point(pre[p.x][p.y][0], pre[p.x][p.y][1]);
                path.push(pre_p);
                p = pre_p;
            }
            break;
        }
        if (p.x - 1 >= 0 && !vis[p.x - 1][p.y] && a[p.x - 1][p.y] == 0 && blank[p.x - 1][p.y] == 0)
        { // 向上搜索
            vis[p.x - 1][p.y] = 1;
            dis[p.x - 1][p.y] = dist + 1;
            pre[p.x - 1][p.y][0] = p.x;
            pre[p.x - 1][p.y][1] = p.y;
            q.push(Node(Point(p.x - 1, p.y), dist + 1));
        }
        if (p.x + 1 < MAXN && !vis[p.x + 1][p.y] && a[p.x + 1][p.y] == 0 && blank[p.x + 1][p.y] == 0)
        { // 向下搜索
            vis[p.x + 1][p.y] = 1;
            dis[p.x + 1][p.y] = dist + 1;
            pre[p.x + 1][p.y][0] = p.x;
            pre[p.x + 1][p.y][1] = p.y;
            q.push(Node(Point(p.x + 1, p.y), dist + 1));
        }
        if (p.y - 1 >= 0 && !vis[p.x][p.y - 1] && a[p.x][p.y - 1] == 0 && blank[p.x][p.y - 1] == 0)
        { // 向左搜索
            vis[p.x][p.y - 1] = 1;
            dis[p.x][p.y - 1] = dist + 1;
            pre[p.x][p.y - 1][0] = p.x;
            pre[p.x][p.y - 1][1] = p.y;
            q.push(Node(Point(p.x, p.y - 1), dist + 1));
        }
        if (p.y + 1 < MAXN && !vis[p.x][p.y + 1] && a[p.x][p.y + 1] == 0 && blank[p.x][p.y + 1] == 0)
        { // 向右搜索
            vis[p.x][p.y + 1] = 1;
            dis[p.x][p.y + 1] = dist + 1;
            pre[p.x][p.y + 1][0] = p.x;
            pre[p.x][p.y + 1][1] = p.y;
            q.push(Node(Point(p.x, p.y + 1), dist + 1));
        }
    }
    if (path.empty())
    {
        std::cout << "empty!!" << std::endl;
    }
    printQueue(path);
    return path;
}
void printQueue(std::queue<Point> q)
{
    // printing content of queue
    while (!q.empty())
    {
        std::cout << "(" << q.front().x << "," << q.front().y << ")->";
        q.pop();
    }
    std::cout << std::endl;
}
void printPointVector(std::vector<Point> v)
{
    for (int i = 0; i < v.size(); i++)
    {
        std::cout << "(" << v[i].x << "," << v[i].y << ")->";
    }
    std::cout << std::endl;
}
bool isSurround(IStudentAPI& api, double x, double y)
{
    double distance;
    auto self = api.GetSelfInfo();
    double sx = (double)(self->x) / 1000;
    double sy = (double)(self->y) / 1000;
    distance = sqrt((sx - x) * (sx - x) + (sy - y) * (sy - y));
    if (distance <= 0.3)
        return true;
    return false;
}
bool isArround(IStudentAPI& api, int x, int y)
{
    double distance;
    auto self = api.GetSelfInfo();
    auto sx = (self->x) / 1000;
    auto sy = (self->y) / 1000;
    distance = sqrt((sx - x) * (sx - x) + (sy - y) * (sy - y));
    if (distance <= 0.4)
        return true;
    return false;
}
bool isTrigger(IStudentAPI& api, Point p)
{
    auto self = api.GetSelfInfo();
    auto sx = (double)(self->x) / 1000;
    auto sy = (double)(self->y) / 1000;
    if (abs(sx - p.x - 0.5) <= 1.5 && abs(sy - p.y - 0.5) <= 1.5)
        return true;
    return false;
}
void Goto(IStudentAPI& api, double destX, double destY, double randAngle = 0)
{
    // std::printf("goto %d,%d\n", destX, destY);
    auto self = api.GetSelfInfo();

    int sx = self->x;
    int sy = self->y;
    auto delta_x = (double)(destX * 1000 - sx);
    auto delta_y = (double)(destY * 1000 - sy);
    std::cout << "dx" << delta_x << "dy" << delta_y << std::endl;
    double ang = 0;
    // 直接走
    ang = atan2(delta_y, delta_x);
    std::cout << "angle:" << ang << std::endl;
    if (delta_x != 0 || delta_y != 0)
        api.Move(300, ang + (std::rand() % 10 - 5) * PI / 10 * randAngle);
}
// 判断实际速度是否为0（防止卡墙上）
bool stuckCheck(IStudentAPI& api, int n)
{
    if (n >= 2 && n <= 10)
    {
        auto self = api.GetSelfInfo();
        auto sx = self->x;
        auto sy = self->y;
        for (int i = 0; i <= n - 2; i++)
        {
            memoryX[i] = memoryX[i + 1];
            memoryY[i] = memoryY[i + 1];
        }
        memoryX[n - 1] = sx;
        memoryY[n - 1] = sy;
        if (abs(memoryX[0] - sx) < 100 && abs(memoryY[0] - sy) < 100)
        {
            std::cout << "stuck!" << std::endl;
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}
bool progressStuckCheck(int progress, int n) // 需要更新！

{
    /*
    if (n >= 2 && n <= 10)
    {
        for (int i = 0; i <= n - 2; i++)
        {
            memoryProgress[i] = memoryProgress[i + 1];
        }
        memoryProgress[n - 1] = progress;
        if (memoryProgress[0] == progress)
        {
            std::cout << "progressStuck!" << std::endl;
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    */
    return false;
}
double Distance(Point a, Point b)
{
    return (sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)));
}
double angle(double x1, double y1, double x2, double y2)
{
    double dot_product = x1 * x2 + y1 * y2;
    double len1 = sqrt(x1 * x1 + y1 * y1);
    double len2 = sqrt(x2 * x2 + y2 * y2);
    double cos_theta = dot_product / (len1 * len2);
    return acos(cos_theta);
}
int pathLen(Point a, Point b)
{
    auto tempPath = bfs(a, b);
    return tempPath.size();
}
void printPathType(IStudentAPI& api, std::queue<Point> q)
{
    while (!q.empty())
    {
        std::cout << "(" << q.front().x << "," << q.front().y << ")->" << (int)api.GetPlaceType(q.front().x, q.front().y) << " ";
        q.pop();
    }
    std::cout << std::endl;
}
void printPosition(IStudentAPI& api)
{
    auto self = api.GetSelfInfo();
    auto sx = self->x;
    auto sy = self->y;
    std::cout << "position: (" << sx << "," << sy << ")" << std::endl;
}
/*
void groupJuan(IStudentAPI& api) //暂不使用
{
    std::vector <int> temp;
    std::vector <Point>hwGroup;
    if (myPlayerID % 2 == 0)
    {
        hwGroup = hwGroup1;
    }
    else
    {
        hwGroup = hwGroup2;
    }

    for (int i = 0; i < hwGroup.size(); i++)
    {
        temp.emplace_back(api.GetClassroomProgress(hwGroup[i].x, hwGroup[i].y));
    }
    for (int i = 0; i < hwGroup.size(); i++)
    {
        //std::cout<<"isTrigger:"<< isTrigger(api, hw[i])<<"progress:"<<temp[i]<<std::endl;
        if (isTrigger(api, hwGroup[i]) && temp[i] < 10000000)
        {
            std::cout << "doing hw:" << i << "progress:" << temp[i] << std::endl;
            api.StartLearning();
            if (progressStuckCheck(temp[i], 9))
            {
                api.Move(300, rand());
                BotStatus = status::initial;
                return;
            }

            return;
        }
    }
    for (int i = 0; i < hwGroup.size(); i++)
    {
        if (temp[i] < 10000000)
        {
            std::cout << "goto hw" << i << temp[i] << std::endl;
            targetP.x = hwGroup[i].x;
            targetP.y = hwGroup[i].y;
            BotStatus = status::initial;
            return;
        }
    }
    std::cout << "graduate!!" << std::endl;
    isGraduate = true;
}
*/
void closestJuan(IStudentAPI& api)
{
    double dis = 999999;
    std::vector<int> temp;
    auto self = api.GetSelfInfo();
    auto sx = self->x;
    auto sy = self->y;
    auto cellX = sx / 1000;
    auto cellY = sy / 1000;
    int hwCount = 0;
    for (int i = 0; i < hw.size(); i++)
    {
        temp.emplace_back(api.GetClassroomProgress(hw[i].x, hw[i].y));
        std::cout << "temp" << i << ":" << temp[i] << std::endl;
        if (hwIsFinished[i])
            hwCount++;
    }
    if (hwCount >= 7)
    {
		isGraduate = true;
        BotStatus = status::idle;
		return;
	}
    for (int i = 0; i < hw.size(); i++)
    {
        // std::cout<<"isTrigger:"<< isTrigger(api, hw[i])<<"progress:"<<temp[i]<<std::endl;
        if (hwIsFinished[i] == 1)
        {
            continue;
        }
        if (isTrigger(api, hw[i]) && temp[i] < 10000000 && hwIsFinished[i] == 0)
        {
            std::cout << "doing hw:" << i << "progress:" << temp[i] << std::endl;
            api.StartLearning();
            if(self->studentType == THUAI6::StudentType::StraightAStudent)
                api.UseSkill(0);
            if (progressStuckCheck(temp[i], 9))
            {
                api.Move(300, rand());
                BotStatus = status::initial;
                return;
            }
            return;
        }
        if (temp[i] == 10000000)
        {
            hwIsFinished[i] = 1;
            isSendMessage = 1;
            std::cout << "work finished!" << std::endl;
            std::cout << "now hwIsFinished:";
            for (int m = 0; m < 10; m++)
            {
                std::cout << hwIsFinished[m];
            }
        }
    }

    targetP = Point(0, 0);
    for (int i = 0; i < hw.size(); i++)
    {
        int tempDis = pathLen(hw[i], Point(cellX, cellY));
        if (temp[i] < 10000000 && tempDis < dis && hwIsFinished[i] == 0)
        {
            dis = tempDis;
            targetP.x = hw[i].x;
            targetP.y = hw[i].y;
        }
    }
    if (targetP.x == 0 && targetP.y == 0)
    {
        std::cout << "graduate!!" << std::endl;
        isGraduate = true;
    }
    else
    {
        BotStatus = status::initial;
        return;
    }
}
void graduate(IStudentAPI& api)
{
    auto self = api.GetSelfInfo();
    auto sx = self->x;
    auto sy = self->y;
    auto cellX = sx / 1000;
    auto cellY = sy / 1000;
    auto selfP = Point(cellX, cellY);
    auto myGate = findNearestPoint(api, 5);
    if (isTrigger(api, myGate) && api.GetGateProgress(myGate.x, myGate.y) < 18000)
    {
        std::cout << "graduating!"
            << "progress:" << api.GetGateProgress(myGate.x, myGate.y) << std::endl;
        api.StartOpenGate();
        api.Graduate();
        if (progressStuckCheck(api.GetGateProgress(myGate.x, myGate.y), 9))
        {
            api.Move(300, rand());
            BotStatus = status::initial;
            return;
        }
        return;
    }
    else if (isTrigger(api, myGate))
    {
        std::cout << "runle" << std::endl;
        api.Graduate();
    }
    else
    {
        targetP.x = myGate.x;
        targetP.y = myGate.y;
        BotStatus = status::initial;
    }
}
void rouse(IStudentAPI& api)
{
    auto self = api.GetSelfInfo();
    auto sx = self->x;
    auto sy = self->y;
    auto cellX = sx / 1000;
    auto cellY = sy / 1000;
    auto selfP = Point(cellX, cellY);
    auto infoInsight = api.GetStudents();
    int addictProgress = 0;
    if (rouseTarget == -1)
    {
        BotStatus = status::idle;
        return;
    }
    if (isTrigger(api, playerPosition[rouseTarget]) && playerState[rouseTarget] == 3)
    {
        api.StartRouseMate(rouseTarget);
        return;
    }
    else if (playerState[rouseTarget] == 3)
    {
        targetP.x = playerPosition[rouseTarget].x;
        targetP.y = playerPosition[rouseTarget].y;
        BotStatus = status::initial;
    }
    else
    {
        rouseTarget = -1;
        BotStatus = status::idle;
    }
}
// 躲避tricker相关
Point findNearestPoint(IStudentAPI& api, int n)
{
    bool flag = false; // 记录是否找到
    int x = api.GetSelfInfo()->x / 1000;
    int y = api.GetSelfInfo()->y / 1000;
    int trickerx = 0;
    int trickery = 0;
    if (isTrickerInsight(api) == 1)
    {
        std::vector<std::shared_ptr<const THUAI6::Tricker>> tricker_vector = api.GetTrickers();
        trickerx = tricker_vector.front()->x / 1000;
        trickery = tricker_vector.front()->y / 1000;
    }
    std::cout << "trickerx:" << trickerx << "trickery:" << trickery << std::endl;
    Point nearestPoint;
    double mindis = 999999;
    for (int i = 0; i < 50; i++)
    {
        for (int j = 0; j < 50; j++)
        {
            if ((int)api.GetPlaceType(i, j) == n)
            {
                if (Distance(Point(i, j), Point(x, y)) < mindis)
                {
                    mindis = Distance(Point(i, j), Point(x, y));
                    nearestPoint = Point(i, j);
                }
            }
        }
    }
    return nearestPoint;
}
// 躲避tricker相关
double tricker_distance(IStudentAPI& api)
{
    std::cout << "Getting tricker distance..." << std::endl;

    double distance = Constants::basicStudentAlertnessRadius;
    // dangerAlert
    double danger = api.GetSelfInfo()->dangerAlert;
    if (danger > 0)
        distance /= danger;
    // 可视
    std::vector<std::shared_ptr<const THUAI6::Tricker>> tricker_vector = api.GetTrickers();
    if (tricker_vector.size() > 0)
    {
        for (int i = 0; i < tricker_vector.size(); i++)
        {
            double temp_distance = sqrt(pow(tricker_vector[i]->x - api.GetSelfInfo()->x, 2) +
                pow(tricker_vector[i]->y - api.GetSelfInfo()->y, 2));
            distance = std::min(distance, temp_distance);
        }
    }
    return distance / 1000;
}
bool isTrickerInsight(IStudentAPI& api)
{
    std::vector<std::shared_ptr<const THUAI6::Tricker>> tricker_vector = api.GetTrickers();
    if (tricker_vector.size() > 0)
    {
        return true;
    }
    else
        return false;
}
bool isStudentInsight(IStudentAPI& api)
{
    std::vector<std::shared_ptr<const THUAI6::Student>> student_vector = api.GetStudents();
    int count = 0;
    for (int i = 0; i < 4; i++)
    {
        if (api.HaveView(student_vector[i]->x, student_vector[i]->y))
            count++;
    }
    if (count > 1)
        return true;
    else
        return false;
}
std::string arrayToString(bool a[], int size)
{
    std::string result;
    for (int i = 0; i < size; i++)
    {
        result += a[i];
    }
    return result;
}
void stringToArray(const std::string& str, bool a[], int size)
{
    for (int i = 0; i < size; i++)
    {
        a[i] = str[i];
    }
}
std::string arrayToString(int a[], int size)
{
    std::string result;
    for (int i = 0; i < size; i++)
    {
        result += a[i];
    }
    return result;
}
void stringToArray(const std::string& str, int a[], int size)
{
    for (int i = 0; i < size; i++)
    {
        a[i] = str[i];
    }
}
std::string intToString(int a)
{
    std::string result;
    result += a;
    return result;
}
void stringToInt(const std::string& str, int& a)
{
    a = str[0];
}
bool isWindowInPath(IStudentAPI& api, std::queue<Point> p)
{
    std::queue<Point> q = p;
    while (!q.empty())
    {
        if ((int)api.GetPlaceType(q.front().x, q.front().y) == 7)
        {
            std::cout << "window In Path!" << std::endl;
            return true;
        }
        std::cout << "(" << q.front().x << "," << q.front().y << ")";
        q.pop();
    }
    std::cout << std::endl;
    return false;
}
bool isSurroundWindow(IStudentAPI& api)
{
    auto self = api.GetSelfInfo();
    int x = self->x / 1000;
    int y = self->y / 1000;

    Point window = findNearestPoint(api, 7);
    if (Distance(Point(x, y), window) <= 1.2)
    {
        return true;
    }
    else
        return false;
}
void receiveMessage(IStudentAPI& api)
{
    while (api.HaveMessage())
    {
        std::cout << "receiving message!" << std::endl;
        auto receive = api.GetMessage();
        int buffer[99];
        int tempID;
        stringToArray(receive.second, buffer, 35);
        tempID = buffer[0];
        int* tempMessage = buffer + 1;
        std::cout << "check" << std::endl;
        for (int m = 0; m < 10; m++)
        {
            hwIsFinished[m] += tempMessage[m];
        }
        for (int m = 0; m < 4; m++)
        {
            if (m == tempID)
                playerDecision[m] = tempMessage[m + 10];
        }
        for (int m = 0; m < 4; m++)
        {
            if (m == tempID)
                playerState[m] = tempMessage[m + 14];
        }
        for (int m = 0; m < 4; m++)
        {
            if (m == tempID)
                playerPosition[m].x = tempMessage[m + 18];
        }
        for (int m = 0; m < 4; m++)
        {
            if (m == tempID)
                playerPosition[m].y = tempMessage[m + 22];
        }
        int trickerflag = tempMessage[26];
        if (trickerflag == 1)
        {
            trickerPosition.x = tempMessage[27];
            trickerPosition.y = tempMessage[28];
            trickerTypeInt = tempMessage[29];
        }
        for (int m = 0; m < 4; m++)
        {
            if (m == tempID)
                playerHP[m] = tempMessage[m + 30];
        }
        std::cout << "message received!" << std::endl;
        for (int m = 0; m < 34; m++)
        {
            std::cout << tempMessage[m] << "||";
        }
        std::cout << std::endl;
    }
}
// 爬完窗后不会重复爬
bool isDelayedAfterWindow(IStudentAPI& api)
{
    if ((int)api.GetSelfInfo()->playerState == 20)
    {
        climbingCheck = true;
        climbingFrameCount = 0;
    }
    climbingFrameCount++;
    if (climbingFrameCount > 20)
    {
        climbingCheck = false;
    }
    return !climbingCheck;
}
// 从vector中随机输出一个元素
Point random_element(std::vector<Point>& vec)
{
    srand(time(NULL));
    int index = rand() % vec.size();
    return vec[index];
}
bool isWalled(IStudentAPI& api, Point avoidPoint)
{
    int x = api.GetSelfInfo()->x / 1000;
    int y = api.GetSelfInfo()->y / 1000;
    int windowFlag = 0;
    std::queue tempPath = bfs(Point(x, y), avoidPoint);
    while (!tempPath.empty())
    {
        if ((int)api.GetPlaceType(tempPath.front().x, tempPath.front().y) == 7)
        {
            windowFlag = 1;
        }
        tempPath.pop();
    }
    if (windowFlag == 1)
    {
        return true;
    }
    else
        return false;
}
Point farestHw(IStudentAPI& api)
{
    int x = (api.GetSelfInfo()->x) / 1000;
    int y = (api.GetSelfInfo()->y) / 1000;
    double dis = 0;
    Point farestHW;
    for (int i = 0; i < hw.size(); i++)
    {
        if (Distance(Point(x, y), hw[i]) > dis)
        {
            farestHW = hw[i];
            dis = Distance(Point(x, y), hw[i]);
        }
    }
    std::cout << "farthest hw" << farestHW.x << "," << farestHW.y << std::endl;
    return farestHW;
}
Point findMinAngle(IStudentAPI& api, std::vector<Point> vec)
{
    double ang = 200;
    int x = (api.GetSelfInfo()->x) / 1000;
    int y = (api.GetSelfInfo()->y) / 1000;
    Point minAngle;
    if (!vec.empty())
    {
        for (int i = 0; i < vec.size(); i++)
        {
            if (angle((x - tricker_x), (y - tricker_y), vec[i].x - x, vec[i].y - y) * 180 / PI < ang)
            {
                ang = angle((x - tricker_x), (y - tricker_y), vec[i].x - x, vec[i].y - y) * 180 / PI;
                minAngle = vec[i];
            }
        }
    }
    return minAngle;
}
Point generateAvoidTarget(IStudentAPI& api)
{
    Point point = Point(2, 3);
    std::cout << "generating" << std::endl;
    int x = (api.GetSelfInfo()->x) / 1000;
    int y = (api.GetSelfInfo()->y) / 1000;
    auto tricker = api.GetTrickers();
    double distance = tricker_distance(api);
    /**/
    std::vector<Point> vec1;
    std::vector<Point> vec2;
    std::vector<Point> vec3;
    std::vector<Point> vec4;
    std::vector<Point> vecWindow;
    for (int i = 0; i < 50; i++)
    {
        for (int j = 0; j < 50; j++)
        {
            if ((int)api.GetPlaceType(i, j) == 7)
            {
                if (Distance(Point(i, j), Point(x, y)) < 8 && Distance(Point(i, j), Point(x, y)) >= 1)
                {
                    if (angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI < 90)
                    {
                        vecWindow.push_back(Point(i, j));
                    }
                }
            }
            if (a[i][j] == 0)
            {
                if (Distance(Point(i, j), Point(x, y)) < 7 && Distance(Point(i, j), Point(x, y)) > 2)
                {
                    if (angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI < 30 && api.HaveView(i * 1000, j * 1000) == 1)
                    {
                        std::cout << "Reachable points:";
                        std::cout << "(" << i << "," << j << ")" << std::endl;
                        vec1.push_back(Point(i, j));
                    }
                }
                if (Distance(Point(i, j), Point(x, y)) < 5 && Distance(Point(i, j), Point(x, y)) > 3)
                {
                    if (angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI > 30 && angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI < 60 && api.HaveView(i * 1000, j * 1000) == 1)
                    {
                        std::cout << "Less Reachable points:";
                        std::cout << "(" << i << "," << j << ")" << std::endl;
                        vec2.push_back(Point(i, j));
                    }
                }
                if (Distance(Point(i, j), Point(x, y)) < 6 && Distance(Point(i, j), Point(x, y)) > 3 && api.HaveView(i * 1000, j * 1000) == 1)
                {
                    if (angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI > 60 && angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI < 90)
                    {
                        std::cout << "Less Less Reachable points:";
                        std::cout << "(" << i << "," << j << ")" << std::endl;
                        vec3.push_back(Point(i, j));
                    }
                }
                if (Distance(Point(i, j), Point(x, y)) < 5 && Distance(Point(i, j), Point(x, y)) > 3 && api.HaveView(i * 1000, j * 1000) == 1)
                {
                    if (angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI > 90 && angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI < 180)
                    {
                        std::cout << "In a corner!:";
                        std::cout << "(" << i << "," << j << ")" << std::endl;
                        vec4.push_back(Point(i, j));
                    }
                }
            }
        }
    }
    if (!vecWindow.empty())
    {
        std::cout << "skip window while avoiding!" << std::endl;
        point = findMinAngle(api, vecWindow);
        return point;
    }
    else if (!vec1.empty())
    {
        std::cout << "vec1!" << std::endl;
        point = findMinAngle(api, vec1);
        return point;
    }
    else if (!vec2.empty())
    {
        std::cout << "vec2!" << std::endl;
        point = findMinAngle(api, vec2);
        return point;
    }
    else if (!vec3.empty())
    {
        std::cout << "vec3!" << std::endl;
        for (int i = 0; i < vec3.size(); i++)
        {
            if ((int)api.GetPlaceType(vec3[i].x, vec3[i].y) == 7)
            {
                return Point(vec3[i].x, vec3[i].y);
            }
        }
        point = findMinAngle(api, vec3);
        return point;
    }
    else if (!vec4.empty())
    {
        std::cout << "vec4!" << std::endl;
        point = findMinAngle(api, vec4);
        return point;
    }
    else
    {
        return Point(1, 1);
    }
}
void gotoTricker(IStudentAPI& api)
{
    targetP = trickerPosition;
    BotStatus = status::initial;
}
void gotoStudents(IStudentAPI& api)
{
    for (int i = 0; i < 2; i++)
    {
        if (playerState[i] != 4 && !isTrigger(api, playerPosition[i]))
        {
            targetP = playerPosition[i];
            BotStatus = status::initial;
        }
    }
}
void gotoFarestStudent(IStudentAPI& api)
{
    int selfID = api.GetSelfInfo()->playerID;
    playerPosition[selfID].x = api.GetSelfInfo()->x;
    playerPosition[selfID].y = api.GetSelfInfo()->y;
    double studentDistance[3];
    int k = 0;
    int k_list[3];
    for (int i = 0; i < 4; i++)
    {
        if (i != selfID)
        {
            k_list[k] = i;
            studentDistance[k++] = Distance(playerPosition[selfID], playerPosition[i]);
        }
    }
    int number = findMaxInArray(studentDistance, 3);
    targetP = playerPosition[k_list[number]];
    decision = 0;
    BotStatus = status::initial;
}
void gotoNearestStudent(IStudentAPI& api)
{
    int selfID = api.GetSelfInfo()->playerID;
    playerPosition[selfID].x = api.GetSelfInfo()->x;
    playerPosition[selfID].y = api.GetSelfInfo()->y;
    double studentDistance[3];
    int k = 0;
    int k_list[3];
    for (int i = 0; i < 4; i++)
    {
        if (i != selfID)
        {
            k_list[k] = i;
            studentDistance[k++] = Distance(playerPosition[selfID], playerPosition[i]);
        }
    }
    int number = findMinInArray(studentDistance, 3);
    targetP = playerPosition[k_list[number]];
    decision = 0;
    BotStatus = status::initial;
}
int findMaxInArray(double a[], int size)
{
    double temp_max = 0;
    int index = 0;
    for (int i = 0; i < size; i++)
    {
        if (a[i] > temp_max)
        {
            temp_max = a[i];
            index = i;
        }
    }
    return index;
}
int findMinInArray(double a[], int size)
{
    double temp_min = MAXN;
    int index = 0;
    for (int i = 0; i < size; i++)
    {
        if (a[i] < temp_min)
        {
            temp_min = a[i];
            index = i;
        }
    }
    return index;
}
Point generateFarMovePoint(IStudentAPI& api)
{
    Point point = Point(2, 3);
    std::cout << "generating" << std::endl;
    int x = (api.GetSelfInfo()->x) / 1000;
    int y = (api.GetSelfInfo()->y) / 1000;
    std::vector<Point> vec1;
    std::vector<Point> vec2;
    for (int i = 0; i < 50; i++)
    {
        for (int j = 0; j < 50; j++)
        {
            if (a[i][j] == 0)
            {
                if (Distance(Point(i, j), Point(x, y)) < 10 && Distance(Point(i, j), Point(x, y)) > 7)
                {
                    if (angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI < 91 && angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI > 60)
                    {
                        if (isWalled(api, Point(i, j)) == 0)
                        {
                            std::cout << "Reachable points:";
                            std::cout << "(" << i << "," << j << ")" << std::endl;
                            vec1.push_back(Point(i, j));
                        }
                    }
                }
            }
        }
    }
    if (!vec1.empty())
    {
        std::cout << "vec1 empty!" << std::endl;
        point = random_element(vec1);
        return point;
    }
    if (vec1.empty())
    {
        for (int i = 0; i < 50; i++)
        {
            for (int j = 0; j < 50; j++)
            {
                if (a[i][j] == 0)
                {
                    if (Distance(Point(i, j), Point(x, y)) < 6 && Distance(Point(i, j), Point(x, y)) > 3)
                    {
                        if (angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI < 60 && angle((x - tricker_x), (y - tricker_y), i - x, j - y) * 180 / PI > 20)
                        {
                            if (isWalled(api, Point(i, j)) == 0)
                            {
                                std::cout << "Reachable points:";
                                std::cout << "(" << i << "," << j << ")" << std::endl;
                                vec2.push_back(Point(i, j));
                            }
                        }
                    }
                }
            }
        }
        if (!vec2.empty())
        {
            std::cout << "vec1 empty!" << std::endl;
            point = random_element(vec2);
            return point;
        }
        else
        {
            return Point(x, y);
        }
    }
    return Point(x, y);
}
void useskill(IStudentAPI& api)
{
    for (int i = 0; i < hw.size(); i++)
    {
        int x = hw[i].x;
        int y = hw[i].y;
        double distance;
        auto self = api.GetSelfInfo();
        auto sx = (self->x) / 1000;
        auto sy = (self->y) / 1000;
        distance = sqrt((sx - x) * (sx - x) + (sy - y) * (sy - y));
        if (distance <= 1)
        {
            api.UseSkill(0);
            api.UseSkill(1);
            api.UseSkill(2);
        }
    }
}
void blankReset()
{
    for (int i = 0; i < MAXN; i++)
    {
        for (int j = 0; j < MAXN; j++)
        {
            blank[i][j] = 0;
        }
    }
}
void sendAndReceiveMessage(IStudentAPI& api)
{
    auto self = api.GetSelfInfo();
    std::cout << isTrickerInsight(api) << std::endl;
    playerDecision[self->playerID] = decision;
    playerState[self->playerID] = (int)self->playerState;

    int k = 0;
    if (api.GetSelfInfo()->studentType == THUAI6::StudentType::Teacher)
    {
        k = 100;
    }
    else if (api.GetSelfInfo()->studentType == THUAI6::StudentType::StraightAStudent)
    {
        k = 13;
    }

    if (api.GetSelfInfo()->addiction > (60000 * k / 10 - 1000))
    {
        playerState[self->playerID] = 4;
    }
    playerPosition[self->playerID].x = self->x / 1000;
    playerPosition[self->playerID].y = self->y / 1000;
    //print info

    std::cout << "isHwFinished: ";
    for (int i = 0; i < 10; i++)
    {
        std::cout << hwIsFinished[i] << "||";
    }
    std::cout << std::endl;
    std::cout << "playerDecision: ";
    for (int i = 0; i < 4; i++)
    {
        std::cout << playerDecision[i] << "||";
    }
    std::cout << std::endl;
    std::cout << "playerState: ";
    for (int i = 0; i < 4; i++)
    {
        std::cout << playerState[i] << "||";
    }
    std::cout << std::endl;
    std::cout << "playerPosition: ";
    for (int i = 0; i < 4; i++)
    {
        //print playerPosition
        std::cout << "(" << playerPosition[i].x << "," << playerPosition[i].y << ")" << "||";
    }
    std::cout << std::endl;

    if (isSendMessage == 1)
    {
        std::string tempstring;
        tempstring += api.GetSelfInfo()->playerID;
        tempstring += arrayToString(hwIsFinished, 10);
        tempstring += arrayToString(playerDecision, 4);
        tempstring += arrayToString(playerState, 4);
        for (int i = 0; i < 4; i++)
        {
            tempstring += playerPosition[i].x;
        }
        for (int i = 0; i < 4; i++)
        {
            tempstring += playerPosition[i].y;
        }
        tempstring += isTrickerInsight(api);
        auto trks = api.GetTrickers();
        if (trks.size() > 0)
        {
            trickerPosition.x = trks[0]->x / 1000;
            trickerPosition.y = trks[0]->y / 1000;
            trickerTypeInt = (int)trks[0]->trickerType;
        }
        tempstring += trickerPosition.x;
        tempstring += trickerPosition.y;
        tempstring += trickerTypeInt;
        for (int i = 0; i < 4; i++)
        {
            tempstring += playerHP[i];
        }
        for (int k = 0; k < 4; k++)
        {
            if (k != api.GetSelfInfo()->playerID)
            {
                api.SendTextMessage(k, tempstring);
                std::cout << "sending message!" << std::endl;
            }
        }
        isSendMessage = 0;
    }
}

void hwInAnotherClassRoom(IStudentAPI& api)
{
    int n = hw.size();
    int hwmap[11][11];
    std::cout << "initial hw" << std::endl;
    for (int i = 0; i < hw.size(); i++)
    {
        std::cout << hw[i].x << hw[i].y << std::endl;
    }
    for (int i = 0; i < hw.size(); i++)
    {
        for (int j = i; j < hw.size(); j++)
        {
            if (i != j)
            {
                std::queue path = bfs(hw[j], hw[i]);
                while (!path.empty())
                {
                    if ((int)api.GetPlaceType(path.front().x, path.front().y) == 7 || (int)api.GetPlaceType(path.front().x, path.front().y) == 8 || (int)api.GetPlaceType(path.front().x, path.front().y) == 9 || (int)api.GetPlaceType(path.front().x, path.front().y) == 10)
                    {
                        hwmap[i][j] = 1;
                    }
                }
                hwmap[i][i] = 1;
            }
        }
    }
}
void isDoorClosed(IStudentAPI& api)
{
    int doorFlag = 0;
    for (int itr = 0; itr < door.size(); itr++)
    {
        if (api.HaveView(door[itr].x, door[itr].y) == 1)
        {
            if (!api.IsDoorOpen(door[itr].x, door[itr].y))
            {
                a[door[itr].x][door[itr].y] = 1;
                doorFlag = 1;
            }
        }
    }
    if (doorFlag == 1)
    {
        InitMapForMove(api);
    }
}

void avoid(IStudentAPI& api)
{
    int x = (api.GetSelfInfo()->x) / 1000;
    int y = (api.GetSelfInfo()->y) / 1000;
    double volume;
    volume = api.GetSelfInfo()->dangerAlert;
    double trickerdistance = 0;
    if (isTrickerInsight(api) == 1)
    {
        auto tricker_vector = api.GetTrickers();
        tricker_x = tricker_vector.front()->x / 1000;
        tricker_y = tricker_vector.front()->y / 1000;
        trickerdistance = tricker_distance(api);
    }
    std::cout << "distance" << trickerdistance << std::endl;

    if ((isTrickerInsight(api) != 0 && trickerdistance <= 5) || volume > 3000)
    {
        if (isReinitial == 0)
        {
            decision = 3;
            std::cout << "distance:" << tricker_distance(api) << std::endl;
            Point temp;
            targetP = generateAvoidTarget(api);
            isReinitial = 1;
            BotStatus = status::initial;
            std::cout << "————————————avoid 1————————————" << std::endl;
        }
        if (isReinitial == 1)
        {
            if (isArround(api, targetP.x + 0.5, targetP.y + 0.5) == 1)
                isReinitial = 0;
            std::cout << "————————————avoid 2————————————" << std::endl;
        }
        return;
    }
    if (volume != 0 && trickerdistance > 5)
    {
        if (isReinitial2 == 0)
        {
            for (int i = tricker_x - 5; i <= tricker_x + 5; i++)
            {
                for (int j = tricker_y - 5; j <= tricker_y + 5; j++)
                {
                    if (i >= 1 && i <= 49 && j >= 1 && j <= 49)
                    {
                        blank[i][j] = 1;
                    }
                }
            }
            for (int i = x - 1; i <= x + 1; i++)
            {
                for (int j = y - 1; j <= y + 1; j++)
                {
                    blank[i][j] = 0;
                }
            }
            std::cout << "————————————avoid 3————————————" << std::endl;
            targetP = farestHw(api);
            isReinitial2 = 1;
            decision = 3;
            BotStatus = status::initial;
        }
        if (isReinitial2 == 1)
        {
            if (isArround(api, targetP.x + 0.5, targetP.y + 0.5) == 1)
                isReinitial2 = 0;
            std::cout << "————————————avoid 4————————————" << std::endl;
        }
    }
    if (isTrickerInsight(api) == 0)
    {
        if (isReinitial2 == 0)
        {
            for (int i = tricker_x - 5; i < tricker_x + 5; i++)
            {
                for (int j = tricker_y - 5; i < tricker_y + 5; j++)
                {
                    if (i >= 1 && i <= 49 && j >= 1 && j <= 49)
                    {
                        blank[i][j] = 1;
                    }
                }
            }
            std::cout << "————————————avoid 5————————————" << std::endl;
            std::cout << "isTrickerInsight=0" << std::endl;;
            targetP = farestHw(api);
            isReinitial3 = 1;
            decision = 3;
            BotStatus = status::initial;
        }
        if (isReinitial3 == 1)
        {
            if (isArround(api, targetP.x + 0.5, targetP.y + 0.5) == 1)
                isReinitial2 = 0;
            std::cout << "————————————avoid 6————————————" << std::endl;
        }
    }
}
void avoidInGrass(IStudentAPI& api)
{
    double volume = api.GetSelfInfo()->dangerAlert;
    if (volume != 0 && isReinitial == 0)
    {
        isAvoid = 1;
        targetP = findNearestPoint(api, 3);
        isReinitial = 1;
        decision = 3;
        BotStatus = status::initial;
        return;
    }
    if (isReinitial == 1 && volume == 0)
    {
        isAvoid = 0;
        isReinitial = 0;
    }
}

void botInit(IStudentAPI& api) // 状态机的初始化
{
    bool stuck = false;
    std::ios::sync_with_stdio(false);
    auto self = api.GetSelfInfo();
    blankReset();
    auto stus = api.GetStudents();
    for (int i = 0; i < stus.size(); i++)
    {
        if (stus[i]->playerID == self->playerID)
        {
            continue;
        }
        blank[stus[i]->x / 1000][stus[i]->y / 1000] = 1;
        if (stus[i]->x / 100 == self->x / 100 && stus[i]->y / 100 == self->y / 100)
        {
            stuck = true;
        }
    }
    auto trs = api.GetTrickers();
    bool isTrickerInsight = trs.size();
    for (int i = 0; i < trs.size(); i++)
    {
        blank[trs[i]->x / 1000][trs[i]->y / 1000] = 1;
        if (trs[i]->x / 100 == self->x / 100 && trs[i]->y / 100 == self->y / 100)
        {
            stuck = true;
        }
    }
    auto bullet = api.GetBullets();
    if (!bullet.empty())
    {
        for (int i = 0; i < bullet.size(); i++)
        {
            blank[bullet[i]->x / 1000][bullet[i]->y / 1000] = 1;
        }
    }
    int x = (api.GetSelfInfo()->x) / 1000;
    int y = (api.GetSelfInfo()->y) / 1000;
    blank[x][y] = 0;
    if (isTrickerInsight == 1)
    {
        trickerPosition.x = trs.front()->x / 1000;
        trickerPosition.y = trs.front()->y / 1000;
    }
    framecount++;
    if (framecount > 10)
    {
        trickerPosition.x = -1;
        trickerPosition.y = -1;
        receiveMessage(api);
        isSendMessage = 1;
        framecount = 0;
    }
    if (!hasInitMap)
    {
        InitMapForMove(api);
        farthestHw = farestHw(api);
        hasInitMap = true;
        // initHwGroup();
    }

    sendAndReceiveMessage(api);

    isDoorClosed(api);

    if (stuck)
    {
        BotStatus = status::retreat;
        return;
    }
    /*
    if (isTrickerInsight == 1)
    {
        isAvoid = 1;
    }
    if (isAvoid == 1)
    {
        std::cout << "avoiding" << std::endl;
        avoid(api);
        decision = 3;
    }
    if (isTrigger(api, targetP))
    {
        isAvoid = 0;
    }*/
    avoidInGrass(api);
    /*
    if (isTrickerInsight(api) == 1 && isReinitial == 0)
    {
        decision = 3;
        std::cout << "distance:" << tricker_distance(api) << std::endl;
        Point temp;
        targetP = generateAvoidTarget(api);
        isReinitial = 1;
        isReinitialForFarMove = 0;
        formerAvoidState = 0;
        BotStatus = status::initial;
    }
    if (isTrickerInsight(api) == 1 && isReinitial == 1)
    {
        std::vector<std::shared_ptr<const THUAI6::Tricker>> tricker_vector = api.GetTrickers();
        int trickerx = tricker_vector.front()->x / 1000;
        int trickery = tricker_vector.front()->y / 1000;
        if (isArround(api, targetP.x + 0.5, targetP.y + 0.5) == 1)
            isReinitial = 0;
        if (angle((targetP.x - x), (targetP.y - y), trickerx - x, trickery - y) * 180 / PI < 90)
            isReinitial = 0;
    }
    else if (isTrickerInsight(api) != 1 && formerAvoidState == 1)
    {
        isReinitial = 0;
        std::cout << "need to move further!" << std::endl;
        if (true)
        {
            targetP = farestHw(api);
            BotStatus = status::initial;
            formerAvoidState = 0;
        }
    }
    */
    if (isTrickerInsight != 1 && isAvoid == 0)
    {
        std::cout << "————————————not avoid 1————————————" << std::endl;
        isReinitial = 0;
        if (playerState[3] == 3)
        {
            bool jumpOut = false;
            for (int i = 0; i < api.GetSelfInfo()->playerID; i++)// Checking the ID before self whether dead or not.
            {
                if (playerState[i] != 3 && playerState[i] != 4 && playerState[i] != 5)
                {
                    jumpOut = true;
                    break;
                }
            }
            if (!jumpOut && decision != 4)
            {
                rouseTarget = 3;
                decision = 4;
                BotStatus = status::idle;
                return;
			}
        }
        else if (decision == 4)
            decision = 1;
        if (rouseTarget != -1)
        {
            rouseTarget = -1;
            BotStatus = status::idle;
        }

        if (!isGraduate)
        {
            decision = 1;
        }
        else if (isGraduate)
        {
            decision = 2;
        }
    }
    formerAvoidState = isTrickerInsight;
    formerState = (int)api.GetSelfInfo()->playerState;
}

void teacherBotInit(IStudentAPI& api)      //状态机的初始化
{


    bool stuck = false;
    std::cout << "decision:" << decision << std::endl;
    std::ios::sync_with_stdio(false);
    auto self = api.GetSelfInfo();
    blankReset();
    auto stus = api.GetStudents();
    for (int i = 0; i < stus.size(); i++)
    {
        if (stus[i]->playerID == self->playerID)
        {
            continue;
        }
        blank[stus[i]->x / 1000][stus[i]->y / 1000] = 1;
        if (stus[i]->x / 100 == self->x / 100 && stus[i]->y / 100 == self->y / 100)
        {
            stuck = true;
        }
    }
    auto trs = api.GetTrickers();
    bool isTrickerInsight = trs.size();
    for (int i = 0; i < trs.size(); i++)
    {
        blank[trs[i]->x / 1000][trs[i]->y / 1000] = 1;
        if (trs[i]->x / 100 == self->x / 100 && trs[i]->y / 100 == self->y / 100)
        {
            stuck = true;
        }
        trickerPosition.x = trs[i]->x / 1000;
        trickerPosition.y = trs[i]->y / 1000;
    }
    int x = (api.GetSelfInfo()->x) / 1000;
    int y = (api.GetSelfInfo()->y) / 1000;
    blank[x][y] = 0;

    framecount++;
    if (framecount > 10)
    {
        trickerPosition.x = 0;
        trickerPosition.y = 0;
        receiveMessage(api);
        isSendMessage = 1;
        framecount = 0;
    }
    if (!hasInitMap)
    {
        InitMapForMove(api);
        hasInitMap = true;
        //initHwGroup();
    }

    sendAndReceiveMessage(api);
    isDoorClosed(api);


    if (stuck)
    {
        BotStatus = status::retreat;
        return;
    }

    bool skillFlag = false;
    if (trs.size() > 0 && api.GetSelfInfo()->timeUntilSkillAvailable[0] == 0)
    {
        if (trs.front()->playerState == THUAI6::PlayerState::Climbing || trs.front()->playerState == THUAI6::PlayerState::Swinging || trs.front()->playerState == THUAI6::PlayerState::OpeningAGate)
            skillFlag = true;
        if (skillFlag && Distance(Point(self->x / 1000, self->y / 1000), Point(trs[0]->x / 1000, trs[0]->y / 1000)) < (double)self->viewRange / 3)
            api.UseSkill(0);
    }

    for (int i = 0; i < 4; i++)
    {
        if (playerState[i] == 3)
        {
            if (decision != 4)
                BotStatus = status::idle;
            decision = 4;
            rouseTarget = i;

            return;
        }
    }
    if (decision == 4)
        decision = 1;
    if (rouseTarget != -1)
    {
        rouseTarget = -1;
        BotStatus = status::idle;
    }

    if (isTrigger(api, targetP) && isAvoid == 1)
    {
          isAvoid = 0;
	}

    if (isTrickerInsight && !isStudentInsight(api))
    {
        isAvoid = 1;
        decision = 3;
        if (isAvoid)
        {
            std::cout << "avoiding" << std::endl;
            avoid(api);
        }
    }
    else if (isTrickerInsight && isStudentInsight(api))
    {
        isReinitial = 0;
        decision = 5;
        if (decision != 5)
        {
			BotStatus = status::idle;
			return;
		}
    }
    else if(!isAvoid)
    {
        isReinitial == 0;
            if (trickerPosition.x != 0 && trickerPosition.y != 0 && decision != 5)
            {
                decision = 5;
                BotStatus = status::idle;
                return;
            }
            else if (trickerPosition.x == 0 && trickerPosition.y == 0)
            {
                if (!isGraduate)
                {
                    decision = 1;
                }
                else if (isGraduate)
                {
                    decision = 6;
                }
            }
    }
}
// ——————————————状态机函数—————————————————————————————
void playerBot(IStudentAPI& api)
{

    if ((int)api.GetSelfInfo()->playerState == 4)
    {
        return;
    }
    std::cout << "Target:"
        << "(" << targetP.x << "," << targetP.y << ")" << std::endl;
    botInit(api);
    std::cout << "isSurroundWindow:    " << isSurroundWindow(api) << std::endl;
    switch (BotStatus)
    {
        // 有限状态机的core
    case status::initial:
    {
        initialStatus(api);
        break;
    }
    case status::move:
    {
        moveStatus(api);
        break;
    }
    case status::retreat:
    {
        retreatStatus(api);
        break;
    }
    case status::idle:
    {
        idleStatus(api);
        break;
    }
    }
}

void teacherBot(IStudentAPI& api)
{

    if ((int)api.GetSelfInfo()->playerState == 4)
    {
        return;
    }

    std::cout << "Target:" << "(" << targetP.x << "," << targetP.y << ")" << std::endl;
    teacherBotInit(api);
    std::cout << "decision:" << decision << std::endl;
    std::cout << "isSurroundWindow:    " << isSurroundWindow(api) << std::endl;
    switch (BotStatus)
    {
        // 有限状态机的core
    case status::initial:
    {
        initialStatus(api);
        break;
    }
    case status::move:
    {
        moveStatus(api);
        // printPosition(api);
        // printQueue(path);
        // printPathType(api, path);
        break;
    }
    case status::retreat:
    {
        retreatStatus(api);
        break;
    }
    case status::idle:
    {
        idleStatus(api);
        break;
    }
    }
    api.Wait();
}

void moveStatus(IStudentAPI& api)
{
    auto self = api.GetSelfInfo();
    int x = self->x / 1000;
    int y = self->y / 1000;
    std::cout << "move!" << std::endl;
    if (isWindowInPath(api, path))
    {
        isCrossingWindow = 1;
    }
    else
        isCrossingWindow = 0;
    std::cout << "isCrossingWindow" << isCrossingWindow << std::endl;
    std::cout << "isSurroundWindow:" << isSurroundWindow(api) << std::endl;
    if (isCrossingWindow == 1)
    {
        if (isDelayedAfterWindow(api) == 1 && isSurroundWindow(api) == 1)
        {
            std::cout << "my skipping window!" << std::endl;
            api.SkipWindow();
            api.SkipWindow();
            if (!path.empty())
            {
                path.pop();
            }
        }
    }
    if (!path.empty())
    {
        if (isSurround(api, path.front().x + 0.5, path.front().y + 0.5) && (!path.empty()))
            path.pop();
    }
    else
    {
        BotStatus = status::initial;
    }

    if (blank[targetP.x][targetP.y] !=0 || a[targetP.x][targetP.y] != 0)
    {
        if (!path.empty() && (!isTrigger(api, targetP) || !api.HaveView(targetP.x * 1000 + 500, targetP.y * 1000 + 500)))
        {
            // std::cout << path.front().x << path.front().y << std::endl;
            Goto(api, path.front().x + 0.5, path.front().y + 0.5);
        }
        else
        {
            BotStatus = status::idle;
        }
    }
    else
    {
        if (!path.empty())
        {
            // std::cout << path.front().x << path.front().y << std::endl;
            Goto(api, path.front().x + 0.5, path.front().y + 0.5);
        }
        else
        {
            BotStatus = status::idle;
        }
    }
    if (stuckCheck(api, 3))
    {

        BotStatus = status::retreat;
        stuckCheckStartTime = std::chrono::system_clock::now();
    }
}
void retreatStatus(IStudentAPI& api)
{
    std::cout << "retreat" << std::endl;
    if (isWindowInPath(api, path))
    {
        isCrossingWindow = 1;
    }
    else
        isCrossingWindow = 0;

    if (isCrossingWindow == 1)
    {
        if (isSurroundWindow(api) == 1 && isDelayedAfterWindow(api) == 1)
        {
            std::cout << "my skipping window!" << std::endl;
            api.SkipWindow();
        }
    }
    double randAngle = 1;
    auto currentTime = std::chrono::system_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - stuckCheckStartTime);
    if (diff.count() > 500)
    {
        std::cout << "deep rand!!" << std::endl;
        randAngle = 2;
        BotStatus = status::initial;
    }
    if (!path.empty())
    {
        std::cout << path.front().x << path.front().y << std::endl;
        Goto(api, path.front().x, path.front().y, randAngle);
        if (isSurround(api, path.front().x, path.front().y))
            path.pop();
    }
    else
    {
        BotStatus = status::idle;
    }
    if (!stuckCheck(api, 3))
    {
        BotStatus = status::move;
    }
}
void initialStatus(IStudentAPI& api)
{
    std::cout << "initial" << std::endl;
    climbingFrameCount = 21;
    int x = (api.GetSelfInfo()->x) / 1000;
    int y = (api.GetSelfInfo()->y) / 1000;
    path = bfs(Point(targetP.x, targetP.y), Point(x, y));
    if (path.empty())
    {
        std::cout << "empty!" << std::endl;
        blankReset();
        path = bfs(Point(targetP.x, targetP.y), Point(x, y));
	}
    if (path.empty())
    {
        BotStatus = status::idle;
        return;
    }
    IHaveArrived = false;
    BotStatus = status::move;
    printQueue(path);
}
void idleStatus(IStudentAPI& api)
{
    std::cout << "idling!" << std::endl;

    switch (decision)
    {
    case 1:
    {
        closestJuan(api);
        break;
    }
    case 2:
    {
        std::cout << "graduate!!" << std::endl;
        graduate(api);
        break;
    }
    case 3:
    {
        decision = 0;
        break;
    }
    case 4:
    {
        rouse(api);
        break;
    }
    case 5:
    {
        gotoTricker(api);
        break;
    }
    case 6:
    {
        gotoStudents(api);
        break;
    }
    default:
    {
        break;
    }
    }
}