#include <vector>
#include <thread>
#include <queue>
#include <array>
#include "AI.h"
#include "constants.h"
// 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新
extern const bool asynchronous = false;

// 选手需要依次将player0到player4的职业在这里定义

extern const std::array<THUAI6::StudentType, 4> studentType = {
    THUAI6::StudentType::Athlete,
    THUAI6::StudentType::Teacher,
    THUAI6::StudentType::StraightAStudent,
    THUAI6::StudentType::Sunshine };

extern const THUAI6::TrickerType trickerType = THUAI6::TrickerType::Assassin;
//记录函数

void InitMap(IAPI& api);
bool issurround(IStudentAPI& api, int x, int y);
//
static THUAI6::PlaceType map[50][50];

static int steps;
static int hasinitmap;
static int hasBeenTo;
static bool hasInitMap = false;
static bool IHaveArrived = false;
static int lastX = 0, lastY = 0;
static int lastFrameCount = 0;
//定义非常多的状态（有限状态机）
enum class status
{
    initial,
    watch,
    sorround,
    idle,
    reset,
    index,
    move
};

static status BotStatus = status::initial;
static status LastStatus = status::reset;
//存储路径的常量
const int MAXN = 50;
int a[MAXN][MAXN];  // 存储地图
int vis[MAXN][MAXN];  // 标记数组，记录每个点是否被访问过
int dis[MAXN][MAXN];  // 记录每个点到起点的最短距离
int pre[MAXN][MAXN][2];  // 记录每个点的前驱节点
// 定义坐标结构体
struct Point {
    int x, y;
    Point() {}
    Point(int x, int y) : x(x), y(y) {}
};
// 定义队列元素结构体
struct Node {
    Point p;
    int dist;
    Node() {}
    Node(Point p, int dist) : p(p), dist(dist) {}
    bool operator<(const Node& other) const {  // 定义优先级，距离更小的优先级更高
        return dist > other.dist;
    }
};
void printQueue(std::queue<Point> q);
std::queue<Point> bfs(Point start, Point end);
void Goto(IStudentAPI& api, int destX, int destY);
bool isWalld(IStudentAPI& api);
// 可以在AI.cpp内部声明变量与函数
std::queue<Point> path;
int count = 1;

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
        std::ios::sync_with_stdio(false);
        auto self = api.GetSelfInfo();

        switch (BotStatus)
        {
            //有限状态机的core
        case status::initial:
        {
            if (!hasInitMap)
            {
                InitMap(api);
                hasInitMap = true;
            }
            int x = (api.GetSelfInfo()->x) / 1000;
            int y = (api.GetSelfInfo()->y) / 1000;
            path = bfs(Point(1, 3), Point(x, y));
            IHaveArrived = false;
            BotStatus = status::move;
            break;
        }
        case status::move:
        {
            std::cout << "move!" << std::endl;
            if (!path.empty())
            {
                std::cout << path.front().x << path.front().y << std::endl;
                Goto(api, path.front().x, path.front().y);
                if (issurround(api, path.front().x, path.front().y))
                    path.pop();
                if (isWalld(api))
                {
                    api.MoveRight(3000);
                    api.Wait();
                }
            }
            //Goto(api, );*/
            printQueue(path);
        }
        api.Wait();
        }
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
}

//我写的函数：

void InitMap(IAPI& api)
{
    int i, j;
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 休眠 2 秒
    for (i = 0; i < 50; i++)
    {
        for (j = 0; j < 50; j++)
        {
            a[i][j] = (int)api.GetPlaceType(i, j) - 1;
            if (a[i][j] >= 10)
                a[i][j] = 9;
            std::cout << a[i][j];
            a[26][42] = 1;
            a[13][48] = 1;
        }
        std::cout << std::endl;
    }
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
    while (!q.empty()) {
        Node cur = q.top();
        q.pop();
        Point p = cur.p;
        int dist = cur.dist;
        if (p.x == end.x && p.y == end.y)
        {  // 找到终点
            path.push(p);
            while (p.x != start.x || p.y != start.y)
            {  // 回溯路径
                Point pre_p = Point(pre[p.x][p.y][0], pre[p.x][p.y][1]);
                path.push(pre_p);
                p = pre_p;
            }
            break;
        }
        if (p.x - 1 >= 0 && !vis[p.x - 1][p.y] && a[p.x - 1][p.y] == 0)
        {  // 向上搜索
            vis[p.x - 1][p.y] = 1;
            dis[p.x - 1][p.y] = dist + 1;
            pre[p.x - 1][p.y][0] = p.x;
            pre[p.x - 1][p.y][1] = p.y;
            q.push(Node(Point(p.x - 1, p.y), dist + 1));
        }
        if (p.x + 1 < MAXN && !vis[p.x + 1][p.y] && a[p.x + 1][p.y] == 0)
        {  // 向下搜索
            vis[p.x + 1][p.y] = 1;
            dis[p.x + 1][p.y] = dist + 1;
            pre[p.x + 1][p.y][0] = p.x;
            pre[p.x + 1][p.y][1] = p.y;
            q.push(Node(Point(p.x + 1, p.y), dist + 1));
        }
        if (p.y - 1 >= 0 && !vis[p.x][p.y - 1] && a[p.x][p.y - 1] == 0)
        {  // 向左搜索
            vis[p.x][p.y - 1] = 1;
            dis[p.x][p.y - 1] = dist + 1;
            pre[p.x][p.y - 1][0] = p.x;
            pre[p.x][p.y - 1][1] = p.y;
            q.push(Node(Point(p.x, p.y - 1), dist + 1));
        }
        if (p.y + 1 < MAXN && !vis[p.x][p.y + 1] && a[p.x][p.y + 1] == 0)
        {  // 向右搜索
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
    //printing content of queue 
    while (!q.empty())
    {
        std::cout << "(" << q.front().x << "," << q.front().y << ")->";
        q.pop();
    }
    std::cout << std::endl;
}
bool issurround(IStudentAPI& api, int x, int y)
{
    double distance;
    auto self = api.GetSelfInfo();
    int sx = (self->x) / 1000;
    int sy = (self->y) / 1000;
    distance = sqrt((sx - x) * (sx - x) + (sy - y) * (sy - y));
    if (distance <= 1)
        return true;
    return false;
}
bool isWalld(IStudentAPI& api)
{
    auto self = api.GetSelfInfo();
    int sx = (self->x) / 1000;
    int sy = (self->y) / 1000;
    if (self->speed == 0)
        return true;
    return false;
}
void Goto(IStudentAPI& api, int destX, int destY)
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
    //直接走
    ang = atan2(delta_y, delta_x);
    api.Move(300, ang);
}
