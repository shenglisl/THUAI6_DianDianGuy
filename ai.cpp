#include <vector>
#include <thread>
#include <queue>
#include <array>
#include <chrono>
#include "AI.h"
#include "constants.h"

#define PI 3.1415
// 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新
extern const bool asynchronous = false;

// 选手需要依次将player0到player4的职业在这里定义

extern const std::array<THUAI6::StudentType, 4> studentType = {
    THUAI6::StudentType::Athlete,
    THUAI6::StudentType::Teacher,
    THUAI6::StudentType::StraightAStudent,
    THUAI6::StudentType::Sunshine };

extern const THUAI6::TrickerType trickerType = THUAI6::TrickerType::Assassin;
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
    move

};

static status BotStatus = status::initial;
static status LastStatus = status::reset;
// 存储路径的常量
const int MAXN = 50;
int a[MAXN][MAXN];      // 存储地图
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
    Node() {}
    Node(Point p, int dist) : p(p), dist(dist) {}
    bool operator<(const Node& other) const
    { // 定义优先级，距离更小的优先级更高
        return dist > other.dist;
    }
};
// 状态机函数
void playerBot(IStudentAPI&);
void moveStatus(IStudentAPI& api);
void retreatStatus(IStudentAPI& api);
void initialStatus(IStudentAPI& api);

// 可以在AI.cpp内部声明变量与函数

// 函数

// 循迹相关
double Distance(Point, Point);
std::queue<Point> bfs(Point, Point);
void Goto(IStudentAPI&, double, double, double); // randAngle = 1，则取波动范围为-0.5pi-0.5pi
void InitMapForMove(IAPI&);
// 状态检查类
bool isSurround(IStudentAPI&, int, int);
bool stuckCheck(IStudentAPI&, int); // 注意，n必须在2-10之间
// debug相关
void printPathType(IStudentAPI&, std::queue<Point>);
void printQueue(std::queue<Point> q);
void printPosition(IStudentAPI&);
void printPointVector(std::vector<Point>);

// 变量

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
//目标坐标
Point targetP = Point(12,3);

//特殊点坐标
std::vector<Point> hw;
std::vector<Point> door;
std::vector<Point> window;
std::vector<Point> gate;
std::vector<Point> chest;


void AI::play(IStudentAPI& api)
{
    // 公共操作
    if (this->playerID == 0)
    {
        // 玩家0执行操作
    }
    else if (this->playerID == 1)
    {
        // 玩家1执行操作
        playerBot(api);
    }
    else if (this->playerID == 2)
    {
        // 玩家2执行操作
    }
    else if (this->playerID == 3)
    {
        // 玩家3执行操作
    }
    // 当然可以写成if (this->playerID == 2||this->playerID == 3)之类的操作
    //  公共操作
}

void AI::play(ITrickerAPI& api)
{
    auto self = api.GetSelfInfo();
    api.PrintSelfInfo();
    api.MoveLeft(1000);
}



void InitMapForMove(IAPI& api)
{
    int i, j;
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 休眠 2 秒
    a[26][42] = 1;
    a[13][48] = 1;
    for (i = 0; i < 50; i++)
    {
        for (j = 0; j < 50; j++)
        {
            a[i][j] = (int)api.GetPlaceType(i, j) - 1;
            if (a[i][j] >= 10)
                a[i][j] = 9;
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

    printPointVector(hw);
    api.Wait();
}





// 搜索最短路径
std::queue<Point> bfs(Point start, Point end)
{
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
        if (p.x - 1 >= 0 && !vis[p.x - 1][p.y] && a[p.x - 1][p.y] == 0)
        { // 向上搜索
            vis[p.x - 1][p.y] = 1;
            dis[p.x - 1][p.y] = dist + 1;
            pre[p.x - 1][p.y][0] = p.x;
            pre[p.x - 1][p.y][1] = p.y;
            q.push(Node(Point(p.x - 1, p.y), dist + 1));
        }
        if (p.x + 1 < MAXN && !vis[p.x + 1][p.y] && a[p.x + 1][p.y] == 0)
        { // 向下搜索
            vis[p.x + 1][p.y] = 1;
            dis[p.x + 1][p.y] = dist + 1;
            pre[p.x + 1][p.y][0] = p.x;
            pre[p.x + 1][p.y][1] = p.y;
            q.push(Node(Point(p.x + 1, p.y), dist + 1));
        }
        if (p.y - 1 >= 0 && !vis[p.x][p.y - 1] && a[p.x][p.y - 1] == 0)
        { // 向左搜索
            vis[p.x][p.y - 1] = 1;
            dis[p.x][p.y - 1] = dist + 1;
            pre[p.x][p.y - 1][0] = p.x;
            pre[p.x][p.y - 1][1] = p.y;
            q.push(Node(Point(p.x, p.y - 1), dist + 1));
        }
        if (p.y + 1 < MAXN && !vis[p.x][p.y + 1] && a[p.x][p.y + 1] == 0)
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

bool isSurround(IStudentAPI& api, int x, int y)
{
    double distance;
    auto self = api.GetSelfInfo();
    auto sx = (self->x) / 1000;
    auto sy = (self->y) / 1000;
    distance = sqrt((sx - x) * (sx - x) + (sy - y) * (sy - y));
    if (distance <= 1)
        return true;
    return false;
}

void Goto(IStudentAPI& api, double destX, double destY, double randAngle = 0)
{
    std::printf("goto %d,%d\n", destX, destY);
    auto self = api.GetSelfInfo();
    int sx = self->x;
    int sy = self->y;
    std::printf("-------------%d,%d------------\n", sx, sy);
    auto delta_x = (double)(destX * 1000 - sx);
    auto delta_y = (double)(destY * 1000 - sy);
    std::printf("-------------%lf,%lf------------\n", delta_x, delta_y);
    double ang = 0;
    // 直接走
    ang = atan2(delta_y, delta_x);
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
        if (memoryX[0] == sx && memoryY[0] == sy)
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
double Distance(Point a, Point b)
{
    return (sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)));
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

void playerBot(IStudentAPI& api)
{
    std::ios::sync_with_stdio(false);
    auto self = api.GetSelfInfo();

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
        std::cout << "idling!" << std::endl;
        break;
    }
    api.Wait();
    }
}

void moveStatus(IStudentAPI& api)
{
    auto self = api.GetSelfInfo();
    std::cout << "move!" << std::endl;
    if (!path.empty())
    {
        // std::cout << path.front().x << path.front().y << std::endl;
        Goto(api, path.front().x + 0.5, path.front().y + 0.5);
        if (isSurround(api, path.front().x, path.front().y))
            path.pop();
    }
    else
    {
        BotStatus = status::idle;
    }
    if (stuckCheck(api, 3))
    {
        BotStatus = status::retreat;
        derectionBeforeRetreat = self->facingDirection;
    }
}

void retreatStatus(IStudentAPI& api)
{
    std::cout << "retreat" << std::endl;
    if (!path.empty())
    {
        std::cout << path.front().x << path.front().y << std::endl;
        Goto(api, path.front().x, path.front().y, 1);
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
    if (!hasInitMap)
    {
        InitMapForMove(api);
        hasInitMap = true;
    }
    int x = (api.GetSelfInfo()->x) / 1000;
    int y = (api.GetSelfInfo()->y) / 1000;
    path = bfs(Point(targetP.x, targetP.y), Point(x, y));
    IHaveArrived = false;
    BotStatus = status::move;
    printQueue(path);
}

