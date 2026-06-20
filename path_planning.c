#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>  
#include <stdbool.h>
#include <time.h>
#include <stdlib.h>

/* ===================================================================
 * 第1部分：宏定义常量
 * =================================================================== */

/* ---- 地图基本尺寸 ---- */
#define MAP_ROWS             12                 /* 地图行数 */
#define MAP_COLS             16                 /* 地图列数 */
#define MAX_BOXES            5                  /* 最大箱子数量 */
#define MAX_IDS              10                 /* 最大ID数量 */
#define MAX_VISIT_STEPS      (2 * MAX_BOXES)    /* 最大访问步数 */
#define MAX_PATH_LEN         500                /* 最大路径长度 */
#define INF                  65535U             /* 无穷大 */
#define DIR_COUNT            4                  /* 方向数量 */

/* 地图元素标识 */
#define WALL                 1          /* 墙 */
#define FLOOR                0          /* 地板 */
#define PLAYER               2          /* 玩家 */
#define BOX                  3          /* 箱子 */
#define TARGET               6          /* 目标 */
#define BOOM                 7          /* 炸弹 */

/* ---- A* 相关常量 ---- */
#define MAX_OPENSET          50000      /* 哈希表最大容量(A*用) */
#define MAX_PERMUTATIONS     120        /* 最大排列数 */
#define MAX_PQ_SIZE          10000      /* 优先级队列大小 */
#define MAX_SEARCH_CNT       50000      /* A*最大搜索次数 */
#define SINGLE_STEP_MAX_PATH 500        /* 单步A*最大路径长度 */
#define TURN_WEIGHT          4          /* 转弯惩罚权重 */
#define ROTATION_PENALTY     3          /* 非0度角度观察惩罚(0度=站在元素左边看向右边) */
#define DIR_INVALID          255        /* 无效方向值 */

/* ---- 模式3炸弹破局相关常量 ---- */
#define MAX_BOOMS            4          /* 最大炸弹数量 */
#define MAX_BREAK_WALLS      100        /* 最大可炸墙数量 */
#define MAX_DETONATE_POINTS  40         /* 最大引爆候选点 */
#define MAX_PROBLEM_POINTS   10         /* 最大死锁问题点 */
#define MAX_PLANS_PER_BOOM   50         /* 每炸弹最大方案数 */
#define MAX_ENCLOSED_REGIONS 10         /* 最大封闭区域数 */
#define MAX_SIM_QUEUE        16384      /* 推箱子模拟队列最大容量 */

/* ---- 死锁类型标识 ---- */
#define DEADLOCK_NONE        0x00
#define DEADLOCK_BOX         0x01
#define DEADLOCK_ENCLOSED    0x04

/* ---- 死锁问题点类型标识 ---- */
#define PROBLEM_ENCLOSED            0x01    /* 封闭区域：玩家无法进入 */
#define PROBLEM_SEPARATED           0x02    /* 隔离区域：影响区不包含目标 */
#define PROBLEM_NEED_SIM            0x04    /* 需要仿真：单纯BFS无法确定 */
#define PROBLEM_TARGET_UNREACHABLE  0x08    /* 目标不可达：箱子无法推到目标 */

/* 压缩状态编码：px:4 py:4 bx:4 by:4 = 16bit */
#define ENCODE_STATE(px, py, bx, by) \
    ((uint16_t)(((px)&0xF)<<12 | ((py)&0xF)<<8 | ((bx)&0xF)<<4 | ((by)&0xF)))
#define DECODE_PX(s) ((uint8_t)(((s)>>12) & 0xF))
#define DECODE_PY(s) ((uint8_t)(((s)>>8)  & 0xF))
#define DECODE_BX(s) ((uint8_t)(((s)>>4)  & 0xF))
#define DECODE_BY(s) ((uint8_t)((s)       & 0xF))

/* BFS visited 索引计算（用于推箱模拟状态去重） */
#define BFS_VISITED_INDEX(px, py, bx, by) \
    (((uint16_t)(px) * 16 + (py)) * 192 + ((uint16_t)(bx) * 16 + (by)))

/* 验证缓存大小 */
#define VCACHE_SIZE 2048

/* 统一移动方向：上、下、左、右 */
static const int8_t DIRS[DIR_COUNT][2] = {{-1,0},{1,0},{0,-1},{0,1}};

/* ===================================================================
 * 第2部分：数据结构定义
 * =================================================================== */

/* ---------- 2.1 通用基础结构 ---------- */

/* 二维坐标 */
typedef struct { 
    uint8_t x;  /* 列坐标 */
    uint8_t y;  /* 行坐标 */
} Point;

/* A* 路径节点（simple_astar 使用） */
typedef struct {
    Point pos;          /* 位置 */
    int8_t last_dir;    /* 上一次移动方向 (-1无效) */
    int g_cost;         /* 实际代价 */
    int f_cost;         /* 估计总代价 f = g + h */
    int parent_idx;     /* 父节点索引 */
    bool closed;        /* 是否已关闭 */
} PathNode;

/* simple_astar 最小堆节点 */
typedef struct {
    int idx;            /* 对应 path_nodes 中的索引 */
} SimpleHeapNode;

/* 箱子-目标配对 */
typedef struct {
    uint8_t box_idx;        /* 箱子索引 */
    uint8_t target_idx;     /* 目标索引 */
    Point box_pos;          /* 箱子位置 */
    Point target_pos;       /* 目标位置 */
} BoxTargetPair;

/* 解决方案序列（模式1/2使用） */
typedef struct {
    BoxTargetPair pairs[MAX_BOXES];     /* 配对列表 */
    uint8_t count;                      /* 配对数量 */
    int total_cost;                     /* 总代价 */
    bool is_valid;                      /* 是否有效 */
    Point full_path[MAX_PATH_LEN];      /* 完整路径 */
    uint16_t full_path_len;             /* 路径长度 */
} SolutionSequence;

/* ---------- 2.2 推箱子 A* 相关结构 ---------- */

/* 推箱子 A* 状态 */
typedef struct {
    Point player;                       /* 玩家位置 */
    Point box;                          /* 箱子位置 */
    Point target;                       /* 目标位置 */
    uint16_t wall_bitmap[MAP_ROWS];     /* 墙位图 */
    uint32_t walls_hash;                /* wall_bitmap预计算哈希 */
    uint8_t last_dir;                   /* 上一方向 */
} State;

/* 哈希表项 */
typedef struct { 
    State state;        /* 状态 */
    uint16_t g;         /* 实际代价 */
    int32_t came_from;  /* 前驱状态哈希索引 */
    uint16_t epoch;     /* epoch标记（替代used，0=未使用） */
} HashEntry;

/* 优先级队列节点 */
typedef struct { 
    uint16_t f;         /* 估计总代价 */
    int32_t state_idx;  /* 哈希状态索引 */
} PQNode;

/* 推箱子 A* 结果 */
typedef struct {
    int32_t cost;                           /* 总代价 */
    Point final_player_pos;                 /* 最终玩家位置 */
    bool success;                           /* 是否成功 */
    Point path_points[SINGLE_STEP_MAX_PATH];/* 最短路径点 */
    uint16_t path_len;                      /* 路径长度 */
} AStarResult;

/* ---------- 2.3 START 模式结构 ---------- */

/* 接近结果（START 模式用） */
typedef struct {
    Point path[MAX_PATH_LEN];       /* 路径点数组 */
    uint16_t path_len;              /* 路径长度 */
    Point elem_pos;                 /* 元素位置 */
    uint8_t elem_type;              /* 元素类型（BOX/TARGET） */
    Point approach_pos;             /* 接近位置 */
    int16_t angle;                  /* 接近角度 */
    bool success;                   /* 是否成功 */
} ApproachResult;

/* 最短路径（START模式用） */
typedef struct {
    uint8_t x[MAX_PATH_LEN];        /* 路径 x 坐标（列） */
    uint8_t y[MAX_PATH_LEN];        /* 路径 y 坐标（行） */
    uint16_t len;                   /* 路径长度 */
    int16_t angle;                  /* 接近角度 */
    uint8_t type;                   /* 元素类型 */
} Path_Start;

/* ---------- 2.4 模式1（非ID配对）结果结构 ---------- */

typedef struct {
    uint8_t x[MAX_PATH_LEN];        /* 路径 x 坐标（列） */
    uint8_t y[MAX_PATH_LEN];        /* 路径 y 坐标（行） */
    uint16_t len;                   /* 路径长度 */
    uint8_t is_push[MAX_PATH_LEN];  /* 是否推动动作 */
} Path;

/* ---------- 2.5 模式2（ID配对）结构 ---------- */

/* 访问步骤 */
typedef struct {
    Point pos;                      /* 访问位置 */
    uint8_t type;                   /* 元素类型（BOX/TARGET） */
    uint8_t original_idx;           /* 原始索引 */
    uint8_t direction;              /* 方向 */
    int16_t angle;                  /* 角度 */
    Point path[MAX_PATH_LEN];       /* 路径点数组 */
    uint16_t path_len;              /* 路径长度 */
    int8_t scanned_id;              /* 扫描ID */
    uint16_t step_cost;             /* 步骤代价 */
} VisitStep;

/* 观察路径结构 */
typedef struct {
    uint8_t x[MAX_PATH_LEN];        /* 路径 x 坐标（列） */
    uint8_t y[MAX_PATH_LEN];        /* 路径 y 坐标（行） */
    uint16_t len;                   /* 路径长度 */
    int16_t angle[MAX_PATH_LEN];    /* 观察角度 */
    uint8_t type[MAX_PATH_LEN];     /* 元素类型 */
    bool is_look[MAX_PATH_LEN];     /* 是否需要扫描 */
} Path_Look;

/* ---------- 2.6 模式3炸弹破局数据结构 ---------- */

/* 压缩推箱子模拟状态（从4字节->2字节） */
typedef uint16_t SimState;

/* 死锁检测结果 */
typedef struct {
    uint8_t box_deadlock_indices[MAX_BOXES]; uint8_t box_deadlock_count;
    uint8_t enclosed_indices[MAX_BOXES * 2]; uint8_t enclosed_count;
    uint8_t enclosed_types[MAX_BOXES * 2]; bool has_deadlock;
} DeadlockResult;

/* 死锁问题点 */
typedef struct {
    uint8_t type; int8_t region_id;
    uint8_t box_indices[MAX_BOXES]; uint8_t box_count;
    uint8_t target_indices[MAX_BOXES]; uint8_t target_count;
    uint16_t influence_mask[MAP_ROWS]; /* 箱子的影响区域（位图） */
    char description[64];
} DeadlockProblemPoint;

/* 可炸墙 */
typedef struct {
    Point wall_pos; uint8_t deadlock_type; int benefit_score;
} BreakableWall;

/* 炸弹方案 */
typedef struct {
    Point detonate_pos; Point walls_covered[9]; uint8_t wall_count;
    uint8_t bomb_index; Point bomb_initial_pos;
    uint16_t bomb_push_distance; int total_benefit;
    bool resolves_deadlock; uint8_t resolved_mask;
    uint8_t affected_mask;
} DetonatePlan;

/* 炸弹推送路径（为推炸弹最短路径） */
typedef struct {
    Point path[MAX_PATH_LEN];           /* 最短路径点序列 */
    uint16_t path_len;                  /* 路径长度（含起点） */
    bool     path_valid;                /* 路径是否有效 */
} BombPushPath;

/* 一个炸弹执行步骤 */
typedef struct {
    uint8_t  step_order;                /* 执行序号（0,1,2...） */
    uint8_t  bomb_index;                /* 使用的炸弹索引 */
    Point    bomb_from;                 /* 炸弹起始位置 */
    Point    detonate_at;               /* 引爆点（炸弹位置/墙位置 */
    Point    walls_removed[9];          /* 该步摧毁的墙 */
    uint8_t  walls_removed_count;       /* 摧毁墙数量 */
    BombPushPath push_path;             /* 推炸弹最短路径 */
    Point    player_before;             /* 该步骤开始前玩家位置 */
    Point    player_after;              /* 该步骤结束后玩家位置 */
} BombExecutionStep;

/* 多炸弹执行计划 */
typedef struct {
    BombExecutionStep steps[MAX_BOOMS];
    uint8_t  step_count;
    uint16_t total_cost;
    bool     is_valid;
} BombExecutionPlan;

/* ===================================================================
 * 第3部分：函数声明
 * =================================================================== */

/* --- 3.1 地图解析 --- */
static void parse_map_input(uint8_t map[MAP_ROWS][MAP_COLS]);

/* --- 3.2 工具函数 --- */
static inline bool pos_equal(Point a, Point b);
static inline int get_smooth_idx(int x, int y, int dir);
static inline uint16_t manhattan_distance(Point a, Point b);
static bool is_wall_bit(const uint16_t walls[MAP_ROWS], Point p);
static bool is_corner_deadlock(Point box, const uint16_t walls[MAP_ROWS]);

/* --- 3.3 纯 A* 寻路（不推箱） --- */
static uint16_t simple_astar(Point start, Point end, uint16_t walls[MAP_ROWS], Point* out_path);

/* --- 3.4 BFS 距离/可达性 --- */
static void bfs_compute_distances(Point start, uint16_t walls[MAP_ROWS]);
static void bfs_compute_reachability(Point start, const uint16_t obstacles[MAP_ROWS]);

/* --- 3.5 推箱子 A* 函数 --- */
static uint32_t hash_state(const State *s);
static uint8_t state_equal(const State *a, const State *b);
static int32_t hash_lookup_insert(const State *s, uint16_t g, int32_t from, uint8_t insert);
static void pq_push(PQNode node);
static PQNode pq_pop(void);
static uint8_t heuristic(const State *s);
static uint8_t is_goal(const State *s);
static void get_successors(const State *cur, State *res, uint8_t *cnt);
static AStarResult solve_single_box_a_star(Point player, Point box, Point target,
                                           uint16_t dynamic_walls[MAP_ROWS]);
/* --- 3.6 模式1贪心配对与回溯验证 --- */
static void generate_greedy_pairing(SolutionSequence* sol);
static bool backtrack_validate(SolutionSequence* sol);
static void validate_solution(SolutionSequence* sol);

/* --- 3.7 START模式：最近接近点与路径提取 --- */
static ApproachResult* find_nearest_approach(uint8_t map[MAP_ROWS][MAP_COLS]);
static void extract_start_turn_points(ApproachResult* res);
static void extract_turn_points(Path* p);

/* --- 3.8 模式2/ID学习推理 --- */
static void id_learning(uint8_t map[MAP_ROWS][MAP_COLS]);
static void id_record(int id);
static bool next_permutation(int *arr, int n);
static void id_inference(void);
static void extract_look_turn_points(VisitStep* plan);
static SolutionSequence* build_solution_from_id_pairing(void);
static Path path_id_calculate(SolutionSequence* sol);

/* --- 3.9 模式3炸弹破局分析 --- */
static uint32_t hash_walls(const uint16_t walls[MAP_ROWS]);
static void compute_player_region_with_walls(const uint16_t walls[MAP_ROWS], bool ignore_bombs);
static void compute_player_region(void);
static uint16_t compute_box_influence(Point box, const uint16_t walls[MAP_ROWS],
                                      uint16_t influence_mask[MAP_ROWS]);
static bool influence_contains_target(const uint16_t influence[MAP_ROWS]);
static bool box_can_reach_any_target(Point box, const uint16_t walls[MAP_ROWS],
                                     const Point targets[], uint8_t target_count);
static bool simulate_box_deadlock(uint8_t box_idx, const uint16_t obstacles[MAP_ROWS],
                                   const Point targets[], uint8_t target_count,
                                   const uint16_t walls[MAP_ROWS],
                                   Point start_player, Point start_box);
static bool simulate_box_to_target(uint8_t box_idx, Point target,
                                    const uint16_t walls[MAP_ROWS],
                                    Point start_player, Point start_box);
static void detect_and_generate_problems(void);
static DeadlockResult detect_all_deadlocks(uint8_t map[MAP_ROWS][MAP_COLS]);
static uint8_t check_problems_resolved_incremental(const uint16_t modified_walls[MAP_ROWS], Point player_pos);
static uint8_t check_problems_resolved_cached(const uint16_t modified_walls[MAP_ROWS], Point player_pos);
static bool is_breakable_wall(Point pos);
static void precompute_bomb_reachability(void);
static bool is_valid_detonation_point(Point pos);
static inline bool can_explosion_cover_wall(Point detonate_pos, Point target_wall);
static void find_detonation_points_for_wall(Point target_wall, Point out_points[], uint8_t *out_count);
static bool verify_bomb_push_feasibility(const DetonatePlan *plan);
static uint16_t estimate_bomb_push_distance(Point bomb_pos, Point detonate_pos, Point player_pos);
static void find_walls_for_enclosed(const uint16_t region_mask[MAP_ROWS],
                                     BreakableWall out_walls[], uint8_t *out_count);
static void find_walls_for_separated(const uint16_t influence[MAP_ROWS],
                                      BreakableWall out_walls[], uint8_t *out_count);
static void generate_plans_for_walls(const BreakableWall breakable_walls[], uint8_t wall_count,
                                      DetonatePlan out_plans[], uint8_t *out_plan_count);
static void sort_plans_by_benefit(DetonatePlan plans[], uint8_t count);
static bool search_multi_bomb_combination(DetonatePlan all_plans[], uint8_t plan_count,
                                           DetonatePlan result_plans[], uint8_t *result_count,
                                           int *out_total_benefit);
static bool analyze_bomb_breakthrough(const DeadlockResult *deadlock,
                                       DetonatePlan out_plans[], uint8_t *out_plan_count,
                                       int *out_best_benefit);
static uint8_t bfs_bomb_distances(Point player, const uint16_t walls[MAP_ROWS],
                                   const bool done[], uint8_t plan_count,
                                   const DetonatePlan plans[], uint16_t dists[MAX_BOOMS]);
static void compute_one_bomb_push(Point bomb_pos, uint8_t bomb_index,
                                   Point detonate_pos, Point player_start,
                                   const uint16_t walls[MAP_ROWS],
                                   const bool active_bombs[MAX_BOOMS],
                                   BombPushPath *out_path);
static bool try_execute_step(const DetonatePlan *p, uint8_t step_idx,
                              const bool active_bombs[MAX_BOOMS],
                              Point *player, uint16_t walls[MAP_ROWS],
                              BombExecutionStep *out_st);
static void plan_bomb_execution_sequence(const DetonatePlan plans[], uint8_t plan_count,
                                          BombExecutionPlan *out_exec);

/* --- 3.10 对外接口 --- */
Path_Start path_start_calculation(uint8_t map[MAP_ROWS][MAP_COLS]);
Path path_calculation(uint8_t box_x[MAP_ROWS][MAP_COLS]);
Path_Look path_look_calculation(uint8_t box_x[MAP_ROWS][MAP_COLS]);
void id_input(int id);
Path path_id_calculation(void);
Path path_boom_calculation(uint8_t map[MAP_ROWS][MAP_COLS]);
void map_boom_out(uint8_t out_map[MAP_ROWS][MAP_COLS]);
void reset_planning_system(void);

/* ===================================================================
 * 第4部分：全局变量定义
 * =================================================================== */

/* ---------- 4.1 A* 推箱子全局缓冲 ---------- */
static HashEntry hash_table[MAX_OPENSET];           /* A*哈希表 */
static PQNode pq[MAX_PQ_SIZE];                      /* A*优先级队列 */
static uint32_t pq_size = 0;                        /* 优先级队列大小 */
static uint16_t hash_epoch = 1;                     /* 哈希表epoch（替代memset） */
static State g_succ_buf[8];                         /* 后继状态缓冲区 */
static PathNode path_nodes[MAP_ROWS * MAP_COLS * 5];/* A*路径节点池 */
static uint8_t astar_epoch = 1;                     /* simple_astar epoch */
static uint8_t astar_node_epoch[MAP_ROWS * MAP_COLS * 5]; /* simple_astar节点epoch */

/* ---------- 4.2 simple_astar 最小堆 ---------- */
static SimpleHeapNode s_heap[MAP_ROWS * MAP_COLS * 5];  /* 最小堆 */
static uint32_t s_heap_size = 0;                         /* 堆大小 */

/* ---------- 4.3 通用临时缓冲区 ---------- */
static uint16_t g_bfs_walls[MAP_ROWS];              /* BFS墙位图 */
static Point g_bfs_queue[MAP_ROWS * MAP_COLS];      /* BFS/通用队列 */
static Point g_temp_path[MAX_PATH_LEN];              /* 临时路径 */
static uint8_t g_temp_x[MAX_PATH_LEN];               /* 临时x坐标 */
static uint8_t g_temp_y[MAX_PATH_LEN];               /* 临时y坐标 */
static uint8_t g_temp_push[MAX_PATH_LEN];            /* 临时推动标识 */

/* ---------- 4.4 地图初始状态 ---------- */
static Point g_initial_boxes[MAX_BOXES];             /* 初始箱子位置 */
static Point g_initial_targets[MAX_BOXES];           /* 初始目标位置 */
static Point g_initial_player;                       /* 初始玩家位置 */
static uint8_t g_box_count = 0;                      /* 箱子数量 */
static uint8_t g_target_count = 0;                   /* 目标数量 */
static uint16_t g_static_walls[MAP_ROWS];            /* 静态墙位图 */
static uint16_t g_target_bitmap[MAP_ROWS];          /* 目标位图（O(1) 查询） */
static Point g_initial_bombs[MAX_BOOMS];             /* 初始炸弹位置 */
static uint8_t g_bomb_count = 0;                     /* 炸弹数量 */

/* ---------- 4.5 模式3炸弹破局全局缓冲 ---------- */
static uint8_t g_original_map[MAP_ROWS][MAP_COLS];   /* 原始地图（供模式3用） */
static SimState g_sim_queue[MAX_SIM_QUEUE];          /* 推箱子模拟队列（压缩态） */
static uint16_t g_obs_buf[MAP_ROWS];                 /* 障碍物缓冲区 */

/* 推箱子模拟BFS访问标记（epoch避免memset） */
static uint8_t g_bfs_epoch = 1;
static uint8_t g_bfs_visited[12 * 16 * 12 * 16];
static uint8_t g_dist_epoch = 1;
static uint8_t g_dist_epoch_tag[MAP_ROWS][MAP_COLS];

static uint8_t g_enclosed_region_count = 0;
static bool g_player_region[MAP_ROWS][MAP_COLS];     /* 玩家可达区域 */

/* 死锁问题点 */
static DeadlockProblemPoint g_problem_points[MAX_PROBLEM_POINTS];
static uint8_t g_problem_count = 0;
static DeadlockProblemPoint g_saved_problem_points[MAX_PROBLEM_POINTS];
static uint8_t g_saved_problem_count = 0;

/* 炸弹可达性缓存：每炸弹预计算的可达区域位图 */
static uint16_t g_bomb_reach_map[MAX_BOOMS][MAP_ROWS];

/* 问题验证时临时保存 */
static uint16_t g_saved_walls[MAP_ROWS];
static bool g_saved_player_region[MAP_ROWS][MAP_COLS];

/* 验证缓存：hash(walls) -> resolved_mask */
static uint32_t g_vcache_hash[VCACHE_SIZE];
static uint8_t  g_vcache_mask[VCACHE_SIZE];
static uint8_t  g_vcache_epoch_tag[VCACHE_SIZE];
static uint8_t  g_vcache_global_epoch = 0;

/* ---------- 4.6 回溯验证全局状态 ---------- */
static bool g_mode1_strict = false;                  /* 模式1严格模式（目标位阻挡对方） */
static bool g_solved[MAX_BOXES];                     /* 箱子是否已解决 */
static uint8_t g_remaining[MAX_BOXES];               /* 剩余箱子索引 */
static uint8_t g_remaining_cnt;                      /* 剩余箱子数量 */
static Point g_current_player_pos;                   /* 当前玩家位置 */
static uint16_t g_current_walls[MAP_ROWS];           /* 当前墙位图 */
static int g_total_cost;                             /* 总代价 */
static uint16_t g_fullpath_len;                      /* 完整路径长度 */
static Point g_fullpath[MAX_PATH_LEN];               /* 完整路径 */
static uint8_t g_solve_order[MAX_BOXES];             /* 求解顺序 */
static uint8_t g_order_idx;                          /* 顺序索引 */

/* ---------- 4.7 全局输出路径 ---------- */
static Path g_path_out = {0};                        /* 模式1输出 */
static Path_Start g_path_start_out = {0};            /* START模式输出 */
static Path_Look g_path_look_out = {0};              /* ID模式输出 */

/* ---------- 4.8 ID模式状态 ---------- */
static uint16_t g_dist_map[MAP_ROWS][MAP_COLS];      /* BFS距离图 */
static VisitStep g_visit_plan[MAX_VISIT_STEPS];       /* 访问计划 */
static uint8_t g_visit_count = 0;                    /* 访问计数 */
static uint8_t g_current_step = 0;                   /* 当前步骤 */
static int8_t g_id_pairing[MAX_IDS];                  /* ID配对表 */
static int8_t g_box_id_map[MAX_BOXES];                /* 箱子ID映射 */
static int8_t g_target_id_map[MAX_BOXES];             /* 目标ID映射 */
static SolutionSequence g_id_based_sol;               /* ID方案 */

/* ---------- 4.9 模式3最终地图 ---------- */
static uint16_t g_boom_final_walls[MAP_ROWS];        /* 爆炸后最终墙位图 */
static uint8_t  g_boom_used_mask;                    /* 已使用炸弹位掩码 */


/* ===================================================================
 * 第5部分：工具函数
 * =================================================================== */

/* ---------- 5.1 最小堆操作（simple_astar 使用） ---------- */

/**
 * @brief 向最小堆中插入一个节点索引
 * @param idx path_nodes 中的索引
 */
static inline void s_heap_push(int idx) {
    if (s_heap_size >= MAP_ROWS * MAP_COLS * 5) return;
    uint32_t pos = s_heap_size++;
    s_heap[pos].idx = idx;
    while (pos > 0) {
        uint32_t parent = (pos - 1) / 2;
        if (path_nodes[s_heap[parent].idx].f_cost <= path_nodes[s_heap[pos].idx].f_cost) break;
        int t = s_heap[parent].idx; s_heap[parent].idx = s_heap[pos].idx; s_heap[pos].idx = t;
        pos = parent;
    }
}

/**
 * @brief 弹出最小堆中f_cost最小的节点索引
 */
static inline int s_heap_pop(void) {
    int top_idx = s_heap[0].idx;
    s_heap[0].idx = s_heap[--s_heap_size].idx;
    uint32_t i = 0;
    while (1) {
        uint32_t l = i * 2 + 1, r = l + 1, min = i;
        if (l < s_heap_size && path_nodes[s_heap[l].idx].f_cost < path_nodes[s_heap[min].idx].f_cost) min = l;
        if (r < s_heap_size && path_nodes[s_heap[r].idx].f_cost < path_nodes[s_heap[min].idx].f_cost) min = r;
        if (min == i) break;
        int t = s_heap[i].idx; s_heap[i].idx = s_heap[min].idx; s_heap[min].idx = t;
        i = min;
    }
    return top_idx;
}

/* ---------- 5.2 坐标比较与几何工具 ---------- */

/**
 * @brief 判断两个坐标是否相等
 */
static inline bool pos_equal(Point a, Point b) {
    return a.x == b.x && a.y == b.y;
}

/**
 * @brief 获取平滑A*节点索引（含方向维度）
 */
static inline int get_smooth_idx(int x, int y, int dir) {
    return (x * MAP_COLS + y) * 5 + (dir + 1);
}

/**
 * @brief 计算曼哈顿距离
 */
static inline uint16_t manhattan_distance(Point a, Point b) {
    int16_t dx = (int)a.x - (int)b.x;
    int16_t dy = (int)a.y - (int)b.y;
    return (uint16_t)((dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy));
}

/* ---------- 5.3 墙位图操作 ---------- */

/**
 * @brief 判断某位置是否为墙（在位图中查询）
 */
static bool is_wall_bit(const uint16_t walls[MAP_ROWS], Point p) {
    if (p.x >= MAP_ROWS || p.y >= MAP_COLS) return true;
    return (walls[p.x] & (1 << p.y)) != 0;
}

/**
 * @brief 判断箱子是否在角落死锁（上下左右均为墙）
 */
static bool is_corner_deadlock(Point box, const uint16_t walls[MAP_ROWS]) {
    bool up    = is_wall_bit(walls, (Point){box.x - 1, box.y});
    bool down  = is_wall_bit(walls, (Point){box.x + 1, box.y});
    bool left  = is_wall_bit(walls, (Point){box.x, box.y - 1});
    bool right = is_wall_bit(walls, (Point){box.x, box.y + 1});
    return (up && left) || (up && right) || (down && left) || (down && right);
}

/* ---------- 5.4 哈希表操作（推箱子A*状态去重） ---------- */

/**
 * @brief 计算状态的哈希值（djb2算法）
 */
static uint32_t hash_state(const State *s) {
    uint32_t h = 5381;
    h = (h << 5 ^ h) ^ s->player.x;
    h = (h << 5 ^ h) ^ s->player.y;
    h = (h << 5 ^ h) ^ s->box.x;
    h = (h << 5 ^ h) ^ s->box.y;
    h = (h << 5 ^ h) ^ s->last_dir;
    h = (h << 5 ^ h) ^ s->walls_hash;
    return h % MAX_OPENSET;
}

/**
 * @brief 判断两个状态是否相等
 */
static uint8_t state_equal(const State *a, const State *b) {
    if (!pos_equal(a->player, b->player)) return 0;
    if (!pos_equal(a->box, b->box)) return 0;
    if (a->last_dir != b->last_dir) return 0;
    if (a->walls_hash != b->walls_hash) return 0;
    for (int i = 0; i < MAP_ROWS; i++)
        if (a->wall_bitmap[i] != b->wall_bitmap[i]) return 0;
    return 1;
}

/**
 * @brief 哈希表查询/插入（线性探测解决冲突）
 * @return >=0 索引 / -1 未找到 / -2 满
 */
static int32_t hash_lookup_insert(const State *s, uint16_t g, int32_t from, uint8_t insert) {
    uint32_t h = hash_state(s);
    uint32_t idx = h;
    uint32_t probe_cnt = 0;
    while (hash_table[idx].epoch == hash_epoch && probe_cnt < MAX_OPENSET) {
        probe_cnt++;
        if (state_equal(&hash_table[idx].state, s)) return (int32_t)idx;
        idx = (idx + 1) % MAX_OPENSET;
        if (idx == h) return -2;
    }
    if (!insert) return -1;
    if (hash_table[idx].epoch != hash_epoch) {
        hash_table[idx].epoch = hash_epoch;
        hash_table[idx].state = *s;
        hash_table[idx].g = g;
        hash_table[idx].came_from = from;
        return (int32_t)idx;
    }
    return -2;
}

/* ---------- 5.5 优先级队列操作（推箱子A*最小堆） ---------- */

/**
 * @brief 推箱子A*优先级队列插入
 */
static void pq_push(PQNode node) {
    if (pq_size >= MAX_PQ_SIZE) return;
    uint32_t pos = pq_size++;
    pq[pos] = node;
    while (pos > 0) {
        uint32_t parent = (pos - 1) / 2;
        if (pq[parent].f <= pq[pos].f) break;
        PQNode t = pq[parent]; pq[parent] = pq[pos]; pq[pos] = t;
        pos = parent;
    }
}

/**
 * @brief 推箱子A*优先级队列弹出
 */
static PQNode pq_pop(void) {
    PQNode top = pq[0];
    pq[0] = pq[--pq_size];
    uint32_t i = 0;
    while (1) {
        uint32_t l = i * 2 + 1, r = l + 1, min = i;
        if (l < pq_size && pq[l].f < pq[min].f) min = l;
        if (r < pq_size && pq[r].f < pq[min].f) min = r;
        if (min == i) break;
        PQNode t = pq[i]; pq[i] = pq[min]; pq[min] = t;
        i = min;
    }
    return top;
}

/* ---------- 5.6 推箱子A*启发式函数 ---------- */

/**
 * @brief 启发式函数（箱子的曼哈顿距离到目标）
 */
static uint8_t heuristic(const State *s) {
    return (uint8_t)(abs((int)s->box.x - (int)s->target.x) + abs((int)s->box.y - (int)s->target.y));
}

/**
 * @brief 判断是否达到目标状态（箱子到达目标位置）
 */
static uint8_t is_goal(const State *s) {
    return pos_equal(s->box, s->target);
}

/**
 * @brief 获取推箱子A*的后继状态
 * 
 * 玩家可以移动或推箱，推箱时需检查目标位是否合法（无墙/不死锁）
 * 
 * @param cur 当前状态
 * @param res 后继状态缓冲区
 * @param cnt 后继数量
 */
static void get_successors(const State *cur, State *res, uint8_t *cnt) {
    *cnt = 0;
    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        Point np = {cur->player.x + DIRS[d][0], cur->player.y + DIRS[d][1]};
        /* 移动受阻：墙/障碍物阻挡（目标位已达） */
        if (is_wall_bit(cur->wall_bitmap, np)) {
            if (!(g_target_bitmap[np.x] & (1 << np.y))) continue;
        }
        if (pos_equal(np, cur->box)) {
            /* 推箱：wall_bitmap 已包含目标位（模式1：箱子无法推动障碍物） */
            Point bp = {np.x + DIRS[d][0], np.y + DIRS[d][1]};
            if (is_wall_bit(cur->wall_bitmap, bp)) continue;
            if (!pos_equal(bp, cur->target) && is_corner_deadlock(bp, cur->wall_bitmap)) continue;
            State ns = *cur; ns.player = np; ns.box = bp; ns.last_dir = d;
            res[(*cnt)++] = ns;
        } else {
            State ns = *cur; ns.player = np; ns.last_dir = d;
            res[(*cnt)++] = ns;
        }
    }
}

/* ---------- 5.7 BFS 距离/可达性计算 ---------- */

/**
 * @brief 标准BFS计算到地图中各格的距离
 */
static void bfs_compute_distances(Point start, uint16_t walls[MAP_ROWS]) {
    for (int i = 0; i < MAP_ROWS; i++)
        for (int j = 0; j < MAP_COLS; j++)
            g_dist_map[i][j] = INF;
    int head = 0, tail = 0;
    g_dist_map[start.x][start.y] = 0;
    g_bfs_queue[tail++] = start;
    while (head < tail) {
        Point cur = g_bfs_queue[head++];
        uint16_t cd = g_dist_map[cur.x][cur.y];
        for (int d = 0; d < DIR_COUNT; d++) {
            Point nxt = {cur.x + DIRS[d][0], cur.y + DIRS[d][1]};
            if (nxt.x >= MAP_ROWS || nxt.y >= MAP_COLS) continue;
            if (walls[nxt.x] & (1 << nxt.y)) continue;
            if (g_dist_map[nxt.x][nxt.y] != INF) continue;
            g_dist_map[nxt.x][nxt.y] = cd + 1;
            g_bfs_queue[tail++] = nxt;
        }
    }
}

/**
 * @brief BFS可达性计算（使用epoch技术避免全局memset）
 */
static void bfs_compute_reachability(Point start, const uint16_t obstacles[MAP_ROWS]) {
    g_dist_epoch++;
    if (g_dist_epoch == 0) { memset(g_dist_epoch_tag, 0, sizeof(g_dist_epoch_tag)); g_dist_epoch = 1; }
    uint16_t head = 0, tail = 0;
    g_dist_epoch_tag[start.x][start.y] = g_dist_epoch;
    g_bfs_queue[tail++] = start;
    while (head < tail) {
        Point curr = g_bfs_queue[head++];
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            Point next = {(uint8_t)(curr.x + DIRS[d][0]), (uint8_t)(curr.y + DIRS[d][1])};
            if (next.x >= MAP_ROWS || next.y >= MAP_COLS) continue;
            if (obstacles[next.x] & (1 << next.y)) continue;
            if (g_dist_epoch_tag[next.x][next.y] == g_dist_epoch) continue;
            g_dist_epoch_tag[next.x][next.y] = g_dist_epoch;
            g_bfs_queue[tail++] = next;
        }
    }
}

/* ---------- 5.8 路径拐点提取（通用） ---------- */

/**
 * @brief 提取路径的拐点（模式1/3共用路径压缩）
 */
static void extract_turn_points(Path* p) {
    if (p->len == 0) return;
    uint8_t widx = 0;
    g_temp_x[0] = p->x[0]; g_temp_y[0] = p->y[0]; g_temp_push[0] = p->is_push[0];
    widx = 1;
    for (uint8_t i = 1; i < p->len - 1; i++) {
        int8_t dx1 = (int8_t)(p->x[i] - p->x[i-1]);
        int8_t dy1 = (int8_t)(p->y[i] - p->y[i-1]);
        int8_t dx2 = (int8_t)(p->x[i+1] - p->x[i]);
        int8_t dy2 = (int8_t)(p->y[i+1] - p->y[i]);
        if (dx1 != dx2 || dy1 != dy2) {
            g_temp_x[widx] = p->x[i]; g_temp_y[widx] = p->y[i];
            g_temp_push[widx] = p->is_push[i]; widx++;
        }
    }
    g_temp_x[widx] = p->x[p->len - 1]; g_temp_y[widx] = p->y[p->len - 1];
    g_temp_push[widx] = p->is_push[p->len - 1]; widx++;
    for (uint8_t i = 0; i < widx; i++) {
        p->x[i] = g_temp_x[i]; p->y[i] = g_temp_y[i]; p->is_push[i] = g_temp_push[i];
    }
    p->len = widx - 1;
}

/* ---------- 5.9 模式3专用工具函数 ---------- */

/**
 * @brief 墙位图哈希（验证缓存用）
 */
static uint32_t hash_walls(const uint16_t walls[MAP_ROWS]) {
    uint32_t h = 5381;
    for (uint8_t i = 0; i < MAP_ROWS; i++)
        h = ((h << 5) + h) + (uint32_t)walls[i];
    return h;
}

/**
 * @brief 判断可炸墙（非边界、至少有一面非墙隔壁）
 */
static bool is_breakable_wall(Point pos) {
    if (!is_wall_bit(g_static_walls, pos)) return false;
    if (pos.x == 0 || pos.x == MAP_ROWS - 1 || pos.y == 0 || pos.y == MAP_COLS - 1) return false;
    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        Point adj = {(uint8_t)(pos.x + DIRS[d][0]), (uint8_t)(pos.y + DIRS[d][1])};
        if (adj.x >= MAP_ROWS || adj.y >= MAP_COLS) continue;
        if (!is_wall_bit(g_static_walls, adj)) return true;
    }
    return false;
}

/**
 * @brief 炸弹点有效性验证
 */
static bool is_valid_detonation_point(Point pos) {
    if (pos.x >= MAP_ROWS || pos.y >= MAP_COLS) return false;
    if (!is_breakable_wall(pos)) return false;
    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        Point adj = {(uint8_t)(pos.x + DIRS[d][0]), (uint8_t)(pos.y + DIRS[d][1])};
        if (adj.x >= MAP_ROWS || adj.y >= MAP_COLS) continue;
        if (is_wall_bit(g_static_walls, adj)) continue;
        return true;
    }
    return false;
}

/**
 * @brief 判断爆炸3x3范围是否能覆盖目标墙
 */
static inline bool can_explosion_cover_wall(Point detonate_pos, Point target_wall) {
    int dx = (int)target_wall.x - (int)detonate_pos.x;
    int dy = (int)target_wall.y - (int)detonate_pos.y;
    return (dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1);
}

/**
 * @brief 估算推炸弹距离（BFS计算玩家+炸弹两段距离）
 * 
 * BFS 计算炸弹从当前位置到引爆点的最短路径
 * 障碍物：静态墙 + 其他箱子 + 其他炸弹
 * 引爆点须为空地（玩家需要推过去爆炸）
 * 
 * @return BFS最短距离；不可达返回 INF
 */
static uint16_t estimate_bomb_push_distance(Point bomb_pos, Point detonate_pos, Point player_pos) {
    if (bomb_pos.x == detonate_pos.x && bomb_pos.y == detonate_pos.y) return 0;

    /* 计算玩家BFS障碍物：墙 + 箱子 + 其他炸弹 */
    uint16_t player_obs[MAP_ROWS];
    memcpy(player_obs, g_static_walls, sizeof(player_obs));
    for (uint8_t i = 0; i < g_box_count; i++)
        player_obs[g_initial_boxes[i].x] |= (1 << g_initial_boxes[i].y);
    for (uint8_t i = 0; i < g_bomb_count; i++)
        player_obs[g_initial_bombs[i].x] |= (1 << g_initial_bombs[i].y);

    /* 玩家到炸弹相邻格最短距离 */
    uint16_t player_dist = INF;
    {
        g_dist_epoch++;
        if (g_dist_epoch == 0) { memset(g_dist_epoch_tag, 0, sizeof(g_dist_epoch_tag)); g_dist_epoch = 1; }
        uint16_t head = 0, tail = 0;
        if (!(player_obs[player_pos.x] & (1 << player_pos.y))) {
            g_dist_epoch_tag[player_pos.x][player_pos.y] = g_dist_epoch;
            g_dist_map[player_pos.x][player_pos.y] = 0;
            g_bfs_queue[tail++] = player_pos;
        }
        while (head < tail) {
            Point cur = g_bfs_queue[head++];
            uint16_t cd = g_dist_map[cur.x][cur.y];
            /* 检查是否到达炸弹相邻格 */
            for (uint8_t d = 0; d < DIR_COUNT; d++) {
                if (cur.x == (uint8_t)(bomb_pos.x + DIRS[d][0]) &&
                    cur.y == (uint8_t)(bomb_pos.y + DIRS[d][1])) {
                    player_dist = cd; head = tail; break;
                }
            }
            if (player_dist != INF) break;
            for (uint8_t d = 0; d < DIR_COUNT; d++) {
                Point nxt = {(uint8_t)(cur.x + DIRS[d][0]), (uint8_t)(cur.y + DIRS[d][1])};
                if (nxt.x >= MAP_ROWS || nxt.y >= MAP_COLS) continue;
                if (player_obs[nxt.x] & (1 << nxt.y)) continue;
                if (g_dist_epoch_tag[nxt.x][nxt.y] == g_dist_epoch) continue;
                g_dist_epoch_tag[nxt.x][nxt.y] = g_dist_epoch;
                g_dist_map[nxt.x][nxt.y] = (uint16_t)(cd + 1);
                g_bfs_queue[tail++] = nxt;
            }
        }
    }
    if (player_dist == INF) return INF;

    /* 计算炸弹BFS障碍物：墙 + 箱子 + 其他炸弹（排除自身） */
    uint16_t obs[MAP_ROWS];
    memcpy(obs, g_static_walls, sizeof(obs));
    for (uint8_t i = 0; i < g_box_count; i++)
        obs[g_initial_boxes[i].x] |= (1 << g_initial_boxes[i].y);
    for (uint8_t i = 0; i < g_bomb_count; i++) {
        if (g_initial_bombs[i].x == bomb_pos.x && g_initial_bombs[i].y == bomb_pos.y) continue;
        obs[g_initial_bombs[i].x] |= (1 << g_initial_bombs[i].y);
    }
    /* 引爆点须为空地（炸弹需推过去爆炸，临时去掉） */
    obs[detonate_pos.x] &= (uint16_t)~(1 << detonate_pos.y);

    /* BFS 炸弹到引爆点 */
    g_dist_epoch++;
    if (g_dist_epoch == 0) { memset(g_dist_epoch_tag, 0, sizeof(g_dist_epoch_tag)); g_dist_epoch = 1; }

    uint16_t head = 0, tail = 0;
    g_dist_epoch_tag[bomb_pos.x][bomb_pos.y] = g_dist_epoch;
    g_dist_map[bomb_pos.x][bomb_pos.y] = 0;
    g_bfs_queue[tail++] = bomb_pos;

    uint16_t bomb_dist = INF;
    while (head < tail) {
        Point cur = g_bfs_queue[head++];
        if (cur.x == detonate_pos.x && cur.y == detonate_pos.y)
            { bomb_dist = g_dist_map[cur.x][cur.y]; break; }

        uint16_t cd = g_dist_map[cur.x][cur.y];
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            Point nxt = {(uint8_t)(cur.x + DIRS[d][0]), (uint8_t)(cur.y + DIRS[d][1])};
            if (nxt.x >= MAP_ROWS || nxt.y >= MAP_COLS) continue;
            if (obs[nxt.x] & (1 << nxt.y)) continue;
            if (g_dist_epoch_tag[nxt.x][nxt.y] == g_dist_epoch) continue;
            g_dist_epoch_tag[nxt.x][nxt.y] = g_dist_epoch;
            g_dist_map[nxt.x][nxt.y] = (uint16_t)(cd + 1);
            g_bfs_queue[tail++] = nxt;
        }
    }
    if (bomb_dist == INF) return INF;
    return (uint16_t)(player_dist + bomb_dist);
}

/**
 * @brief 根据接近位置与元素的相对位置计算观察角度
 * 
 * dx = elem.x - approach.x, dy = elem.y - approach.y
 * 返回值: 0(右侧观察), 180(左侧), 90(下方), -90(上方)
 */
static int16_t compute_approach_angle(int8_t dx, int8_t dy) {
    if (dx == 0)      return (dy > 0) ? 0 : 180;
    else if (dy == 0) return (dx > 0) ? -90 : 90;
    return 0;
}

/* ===================================================================
 * 第6部分：模式算法实现
 * =================================================================== */

/* ---------- 6.1 通用算法（地图解析 & A*寻路） ---------- */

/**
 * @brief 解析地图数据，初始化全局状态
 * 
 * 从二维地图数组提取静态墙位图、箱子、目标、玩家、炸弹位置
 * 并保存原始地图（供模式3用）
 * 
 * @param map 12x16 二维地图数组
 */
static void parse_map_input(uint8_t map[MAP_ROWS][MAP_COLS]) {
    g_box_count = 0;
    g_target_count = 0;
    g_bomb_count = 0;
    memset(g_static_walls, 0, sizeof(g_static_walls));
    memset(g_target_bitmap, 0, sizeof(g_target_bitmap));
    memset(g_box_id_map, -1, sizeof(g_box_id_map));
    memset(g_target_id_map, -1, sizeof(g_target_id_map));
    memset(g_initial_bombs, 0, sizeof(g_initial_bombs));
    memcpy(g_original_map, map, sizeof(uint8_t) * MAP_ROWS * MAP_COLS);

    for (uint8_t i = 0; i < MAP_ROWS; i++) {
        for (uint8_t j = 0; j < MAP_COLS; j++) {
            uint8_t val = map[i][j];
            if (val == WALL) {
                g_static_walls[i] |= (1 << j);
            } else if (val == PLAYER) {
                g_initial_player.x = i; g_initial_player.y = j;
            } else if (val == BOX && g_box_count < MAX_BOXES) {
                g_initial_boxes[g_box_count].x = i;
                g_initial_boxes[g_box_count].y = j;
                g_box_count++;
            } else if (val == TARGET && g_target_count < MAX_BOXES) {
                g_initial_targets[g_target_count].x = i;
                g_initial_targets[g_target_count].y = j;
                g_target_bitmap[i] |= (1 << j);  /* 记录目标位图 */
                g_target_count++;
            } else if (val == BOOM && g_bomb_count < MAX_BOOMS) {
                g_initial_bombs[g_bomb_count].x = i;
                g_initial_bombs[g_bomb_count].y = j;
                g_bomb_count++;
            }
        }
    }
    g_vcache_global_epoch++;
    if (g_vcache_global_epoch == 0) { memset(g_vcache_epoch_tag, 0, sizeof(g_vcache_epoch_tag)); g_vcache_global_epoch = 1; }
}

/**
 * @brief 纯A*寻路（不推箱，含转向惩罚）
 * 
 * 使用最小堆优化，加权启发式 f = g + 1.2*h 加速收敛
 * 
 * @param start    起点
 * @param end      终点
 * @param walls    墙位图
 * @param out_path 最短路径输出
 * @return 路径长度，0=无路径
 */
static uint16_t simple_astar(Point start, Point end, uint16_t walls[MAP_ROWS], Point* out_path) {
    astar_epoch++;
    if (astar_epoch == 0) {
        memset(astar_node_epoch, 0, sizeof(astar_node_epoch));
        astar_epoch = 1;
    }
    s_heap_size = 0;

    int start_idx = get_smooth_idx(start.x, start.y, -1);
    int h_start = abs((int)start.x - (int)end.x) + abs((int)start.y - (int)end.y);
    astar_node_epoch[start_idx] = astar_epoch;
    path_nodes[start_idx].pos = start;
    path_nodes[start_idx].last_dir = -1;
    path_nodes[start_idx].g_cost = 0;
    path_nodes[start_idx].parent_idx = -1;
    path_nodes[start_idx].f_cost = (6 * h_start) / 5;
    s_heap_push(start_idx);

    int final_idx = -1;
    while (s_heap_size > 0) {
        int current_idx = s_heap_pop();
        if (astar_node_epoch[current_idx] != astar_epoch) continue;
        if (path_nodes[current_idx].closed) continue;
        path_nodes[current_idx].closed = true;
        if (path_nodes[current_idx].pos.x == end.x && 
            path_nodes[current_idx].pos.y == end.y) {
            final_idx = current_idx; break;
        }
        Point cur_pos = path_nodes[current_idx].pos;
        int8_t cur_dir = path_nodes[current_idx].last_dir;
        int cur_g = path_nodes[current_idx].g_cost;

        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            Point np = {cur_pos.x + DIRS[d][0], cur_pos.y + DIRS[d][1]};
            if (np.x >= MAP_ROWS || np.y >= MAP_COLS) continue;
            if (walls[np.x] & (1 << np.y)) continue;
            int nidx = get_smooth_idx(np.x, np.y, d);
            if (astar_node_epoch[nidx] == astar_epoch && path_nodes[nidx].closed) continue;
            int move_cost = 1 + ((cur_dir != -1 && cur_dir != (int8_t)d) ? TURN_WEIGHT : 0);
            int ng = cur_g + move_cost;
            int known = (astar_node_epoch[nidx] == astar_epoch);
            if (!known || ng < path_nodes[nidx].g_cost) {
                astar_node_epoch[nidx] = astar_epoch;
                path_nodes[nidx].pos = np;
                path_nodes[nidx].last_dir = (int8_t)d;
                path_nodes[nidx].g_cost = ng;
                int h = abs((int)np.x - (int)end.x) + abs((int)np.y - (int)end.y);
                path_nodes[nidx].f_cost = ng + (6 * h) / 5;
                path_nodes[nidx].parent_idx = current_idx;
                s_heap_push(nidx);
            }
        }
    }
    if (final_idx == -1) return 0;

    uint16_t len = 0;
    int idx = final_idx;
    while (idx != -1) {
        out_path[len++] = path_nodes[idx].pos;
        idx = path_nodes[idx].parent_idx;
    }
    for (int i = 0; i < len / 2; i++) {
        Point t = out_path[i];
        out_path[i] = out_path[len - 1 - i];
        out_path[len - 1 - i] = t;
    }
    return len;
}

/**
 * @brief 推箱子A*主函数
 * 
 * 将箱子从初始位置推到目标位置，返回玩家最短路径
 * 使用哈希去重 + 优先级队列（最小堆）优化
 * 
 * @param player        玩家初始位置
 * @param box           箱子初始位置
 * @param target        目标位置
 * @param dynamic_walls 动态墙位图（含其他箱子的障碍）
 * @return AStarResult 计算结果
 */
static AStarResult solve_single_box_a_star(Point player, Point box, Point target,
                                            uint16_t dynamic_walls[MAP_ROWS]) {
    AStarResult result = {0};
    result.success = false;
    result.cost = -1;
    result.final_player_pos = player;

    hash_epoch++;
    if (hash_epoch == 0) {
        memset(hash_table, 0, sizeof(hash_table));
        hash_epoch = 1;
    }
    memset(pq, 0, sizeof(pq));
    pq_size = 0;

    State start;
    memset(&start, 0, sizeof(State));
    start.player = player; start.box = box; start.target = target;
    start.last_dir = DIR_INVALID;
    memcpy(start.wall_bitmap, dynamic_walls, sizeof(uint16_t) * MAP_ROWS);
    start.walls_hash = hash_walls(dynamic_walls);

    int32_t start_idx = hash_lookup_insert(&start, 0, -1, 1);
    if (start_idx < 0) return result;
    pq_push((PQNode){.f = heuristic(&start), .state_idx = start_idx});

    int32_t goal_idx = -1;
    uint32_t search_cnt = 0;

    while (pq_size > 0 && search_cnt < MAX_SEARCH_CNT) {
        search_cnt++;
        PQNode node = pq_pop();
        HashEntry *entry = &hash_table[node.state_idx];
        if (is_goal(&entry->state)) { goal_idx = node.state_idx; break; }

        uint8_t cnt;
        get_successors(&entry->state, g_succ_buf, &cnt);
        for (int i = 0; i < cnt; i++) {
            uint16_t ng = entry->g + 1;
            if (entry->state.last_dir != DIR_INVALID &&
                g_succ_buf[i].last_dir != entry->state.last_dir) {
                ng += TURN_WEIGHT;
            }
            int32_t idx = hash_lookup_insert(&g_succ_buf[i], INF, -1, 0);
            if (idx == -1) {
                idx = hash_lookup_insert(&g_succ_buf[i], ng, node.state_idx, 1);
                if (idx >= 0)
                    pq_push((PQNode){.f = ng + heuristic(&g_succ_buf[i]), .state_idx = idx});
            } else if (idx >= 0 && hash_table[idx].g > ng) {
                hash_table[idx].g = ng;
                hash_table[idx].came_from = node.state_idx;
                pq_push((PQNode){.f = ng + heuristic(&g_succ_buf[i]), .state_idx = idx});
            }
        }
    }
    if (goal_idx == -1) {
        hash_epoch++; pq_size = 0; return result;
    }

    int len = 0;
    int32_t idx = goal_idx;
    while (idx != -1 && len < MAX_PATH_LEN) {
        g_temp_path[len].x = hash_table[idx].state.player.x;
        g_temp_path[len].y = hash_table[idx].state.player.y;
        len++; idx = hash_table[idx].came_from;
    }
    result.path_len = (uint16_t)len;
    for (int i = 0; i < len; i++)
        result.path_points[i] = g_temp_path[len - 1 - i];
    result.cost = len;
    result.final_player_pos = hash_table[goal_idx].state.player;
    result.success = true;
    hash_epoch++; pq_size = 0;
    return result;
}

/* ---------- 6.2 START模式：最近接近点与路径提取 ---------- */

/**
 * @brief 寻找最近的箱子或目标点，计算接近路径和角度
 * 
 * @param map 12x16 地图数组
 * @return 静态接近结果指针
 */
static ApproachResult* find_nearest_approach(uint8_t map[MAP_ROWS][MAP_COLS]) {
    static ApproachResult best_res;
    memset(&best_res, 0, sizeof(best_res));
    best_res.success = false;
    int global_min_cost = INF;

    Point player = {0};
    for (int i = 0; i < MAP_ROWS; i++)
        for (int j = 0; j < MAP_COLS; j++)
            if (map[i][j] == PLAYER) { player.x = (uint8_t)i; player.y = (uint8_t)j; break; }

    uint16_t sw[MAP_ROWS];
    memset(sw, 0, sizeof(sw));
    for (int i = 0; i < MAP_ROWS; i++)
        for (int j = 0; j < MAP_COLS; j++)
            if (map[i][j] == WALL || map[i][j] == BOOM || map[i][j] == TARGET) sw[i] |= (1 << j);

    for (int i = 0; i < MAP_ROWS; i++) {
        for (int j = 0; j < MAP_COLS; j++) {
            uint8_t cell = map[i][j];
            if (cell != BOX && cell != TARGET) continue;
            Point elem_pos = {(uint8_t)i, (uint8_t)j};
            uint16_t tw[MAP_ROWS];
            memcpy(tw, sw, sizeof(tw));
            tw[elem_pos.x] |= (1 << elem_pos.y);
            for (int d = 0; d < DIR_COUNT; d++) {
                Point app = {elem_pos.x + DIRS[d][0], elem_pos.y + DIRS[d][1]};
                if (app.x >= MAP_ROWS || app.y >= MAP_COLS) continue;
                if (tw[app.x] & (1 << app.y)) continue;
                uint16_t len = simple_astar(player, app, tw, g_temp_path);
                if (len > 0) {
                    int eff = (int)len + ((DIRS[d][1] != -1) ? ROTATION_PENALTY : 0);
                    if (eff < global_min_cost) {
                        global_min_cost = eff;
                    best_res.success = true;
                    best_res.elem_pos = elem_pos;
                    best_res.elem_type = cell;
                    best_res.approach_pos = app;
                    best_res.path_len = len;
                    memcpy(best_res.path, g_temp_path, sizeof(Point) * len);
                    best_res.angle = compute_approach_angle(
                        (int8_t)(elem_pos.x - app.x), (int8_t)(elem_pos.y - app.y));
                    }
                }
            }
        }
    }
    return &best_res;
}

/**
 * @brief 从接近结果中提取拐点（START模式路径压缩）
 */
static void extract_start_turn_points(ApproachResult* res) {
    if (res->path_len == 0) { g_path_start_out.len = 0; return; }
    uint8_t widx = 0;
    g_temp_x[0] = res->path[0].y; g_temp_y[0] = res->path[0].x; widx = 1;
    for (uint8_t i = 1; i < res->path_len - 1; i++) {
        int8_t dx1 = (int8_t)(res->path[i].y   - res->path[i-1].y);
        int8_t dy1 = (int8_t)(res->path[i].x   - res->path[i-1].x);
        int8_t dx2 = (int8_t)(res->path[i+1].y - res->path[i].y);
        int8_t dy2 = (int8_t)(res->path[i+1].x - res->path[i].x);
        if (dx1 != dx2 || dy1 != dy2) {
            g_temp_x[widx] = res->path[i].y; g_temp_y[widx] = res->path[i].x; widx++;
        }
    }
    g_temp_x[widx] = res->path[res->path_len - 1].y;
    g_temp_y[widx] = res->path[res->path_len - 1].x; widx++;
    for (uint8_t i = 0; i < widx; i++) {
        g_path_start_out.x[i] = g_temp_x[i]; g_path_start_out.y[i] = g_temp_y[i];
    }
    g_path_start_out.len = widx;
    g_path_start_out.type = res->elem_type;
    g_path_start_out.angle = res->angle;
}

/* ---------- 6.3 模式1贪心配对与回溯验证 ---------- */

/**
 * @brief 贪心算法生成箱子-目标配对
 *
 * 每次为当前箱子选择距离最近的未使用目标
 *
 * @param sol 解决方案序列
 */
static void generate_greedy_pairing(SolutionSequence* sol)
{
    sol->count = g_box_count;
    sol->is_valid = false;
    sol->total_cost = 0;
    sol->full_path_len = 0;

    bool target_used[MAX_BOXES] = {false};

    for (uint8_t b = 0; b < g_box_count; b++) {
        uint16_t best_dist = INF;
        int8_t best_target = -1;

        bfs_compute_distances(g_initial_boxes[b], g_static_walls);

        for (uint8_t t = 0; t < g_target_count; t++) {
            if (target_used[t]) continue;
            uint16_t d = g_dist_map[g_initial_targets[t].x][g_initial_targets[t].y];
            if (d < best_dist) {
                best_dist = d;
                best_target = (int8_t)t;
            }
        }

        if (best_target >= 0) {
            target_used[best_target] = true;
            sol->pairs[b].box_idx = b;
            sol->pairs[b].target_idx = (uint8_t)best_target;
            sol->pairs[b].box_pos = g_initial_boxes[b];
            sol->pairs[b].target_pos = g_initial_targets[best_target];
        }
    }
}

/**
 * @brief 回溯验证：尝试将剩余箱子推到各自目标
 * 
 * 对剩余箱子按距离排序，依次尝试推箱，失败则回退
 * 
 * @param sol 解决方案
 * @return true 验证成功
 */
static bool backtrack_validate(SolutionSequence* sol) {
    if (g_order_idx == sol->count) {
        sol->is_valid = true;
        sol->total_cost = g_total_cost;
        sol->full_path_len = g_fullpath_len;
        memcpy(sol->full_path, g_fullpath, sizeof(Point) * g_fullpath_len);
        return true;
    }

    g_remaining_cnt = 0;
    for (uint8_t i = 0; i < sol->count; i++)
        if (!g_solved[i]) g_remaining[g_remaining_cnt++] = i;

    /* 重置 BFS 临时障碍物位图（含其他已配对箱子） */
    {
        memcpy(g_bfs_walls, g_static_walls, sizeof(g_bfs_walls));
        for (uint8_t i = 0; i < g_remaining_cnt; i++) {
            Point p = sol->pairs[g_remaining[i]].box_pos;
            g_bfs_walls[p.x] |= (1 << p.y);
        }
        for (int i = 0; i < MAP_ROWS; i++)
            for (int j = 0; j < MAP_COLS; j++)
                g_dist_map[i][j] = INF;
        int head = 0, tail = 0;
        g_dist_map[g_current_player_pos.x][g_current_player_pos.y] = 0;
        g_bfs_queue[tail++] = g_current_player_pos;
        while (head < tail) {
            Point cur = g_bfs_queue[head++];
            uint16_t cd = g_dist_map[cur.x][cur.y];
            for (int d = 0; d < DIR_COUNT; d++) {
                Point nxt = {cur.x + DIRS[d][0], cur.y + DIRS[d][1]};
                if (nxt.x >= MAP_ROWS || nxt.y >= MAP_COLS) continue;
                if (g_bfs_walls[nxt.x] & (1 << nxt.y)) continue;
                if (g_dist_map[nxt.x][nxt.y] != INF) continue;
                g_dist_map[nxt.x][nxt.y] = cd + 1;
                g_bfs_queue[tail++] = nxt;
            }
        }
        for (uint8_t i = 0; i < g_remaining_cnt - 1; i++) {
            for (uint8_t j = 0; j < g_remaining_cnt - 1 - i; j++) {
                Point bp  = sol->pairs[g_remaining[j]].box_pos;
                Point bp2 = sol->pairs[g_remaining[j + 1]].box_pos;
                uint16_t d1 = INF, d2 = INF;
                for (int d = 0; d < DIR_COUNT; d++) {
                    Point ap1 = {bp.x + DIRS[d][0], bp.y + DIRS[d][1]};
                    if (ap1.x < MAP_ROWS && ap1.y < MAP_COLS && g_dist_map[ap1.x][ap1.y] < d1)
                        d1 = g_dist_map[ap1.x][ap1.y];
                    Point ap2 = {bp2.x + DIRS[d][0], bp2.y + DIRS[d][1]};
                    if (ap2.x < MAP_ROWS && ap2.y < MAP_COLS && g_dist_map[ap2.x][ap2.y] < d2)
                        d2 = g_dist_map[ap2.x][ap2.y];
                }
                if (d1 > d2) {
                    uint8_t t = g_remaining[j]; g_remaining[j] = g_remaining[j + 1]; g_remaining[j + 1] = t;
                }
            }
        }
    }

    for (uint8_t k = 0; k < g_remaining_cnt; k++) {
        uint8_t try_idx = g_remaining[k];
        Point cur_box = sol->pairs[try_idx].box_pos;
        Point cur_target = sol->pairs[try_idx].target_pos;

        memcpy(g_current_walls, g_static_walls, sizeof(g_current_walls));
        /* 重置 BFS 缓冲区与距离图
       模式1（非ID配对）：已配对箱子加入障碍物，其目标位阻塞后续操作
       模式2（ID配对）：已配对同ID箱子和同ID目标位均阻塞 */
        {
            int8_t my_id = g_box_id_map[sol->pairs[try_idx].box_idx];
            for (uint8_t m = 0; m < g_remaining_cnt; m++) {
                if (m == k) continue;
                uint8_t pi = g_remaining[m];
                Point obs = sol->pairs[pi].box_pos;
                g_current_walls[obs.x] |= (1 << obs.y);
                if (g_mode1_strict ||
                    (my_id != -1 && g_target_id_map[sol->pairs[pi].target_idx] == my_id)) {
                    Point tp = sol->pairs[pi].target_pos;
                    g_current_walls[tp.x] |= (1 << tp.y);
                }
            }
        }

        if (!pos_equal(cur_box, cur_target) && is_corner_deadlock(cur_box, g_current_walls))
            continue;

        AStarResult res = solve_single_box_a_star(g_current_player_pos, cur_box, cur_target, g_current_walls);
        if (!res.success) continue;

        /* 更新箱子和目标位图，支持目标位 O(1) 阻塞判断
       模式1（非ID配对）：禁止踏入同ID已配对目标位
       模式2（ID配对）：仅同ID目标位标记为禁止 */
        {
            int8_t my_id = g_box_id_map[sol->pairs[try_idx].box_idx];
            uint16_t forbid_targets[MAP_ROWS] = {0};
            for (uint8_t m = 0; m < sol->count; m++) {
                if (m == try_idx || g_solved[m]) continue;
                uint8_t ti = sol->pairs[m].target_idx;
                if (g_mode1_strict ||
                    (my_id != -1 && g_target_id_map[ti] == my_id))
                    forbid_targets[sol->pairs[m].target_pos.x] |= (1 << sol->pairs[m].target_pos.y);
            }

            Point sim_box = cur_box;
            bool box_path_valid = true;
            for (uint16_t i = 0; i < res.path_len && box_path_valid; i++) {
                if (pos_equal(res.path_points[i], sim_box)) {
                    if (i > 0) {
                        int8_t pdx = (int8_t)(res.path_points[i].x - res.path_points[i - 1].x);
                        int8_t pdy = (int8_t)(res.path_points[i].y - res.path_points[i - 1].y);
                        sim_box.x = (uint8_t)(sim_box.x + pdx);
                        sim_box.y = (uint8_t)(sim_box.y + pdy);
                        if (!pos_equal(sim_box, cur_target) &&
                            (forbid_targets[sim_box.x] & (1 << sim_box.y)))
                            box_path_valid = false;
                    }
                }
            }
            if (!box_path_valid) continue;
        }

        g_solved[try_idx] = true;
        g_solve_order[g_order_idx++] = try_idx;
        Point saved_player = g_current_player_pos;
        int saved_cost = g_total_cost;
        uint16_t saved_len = g_fullpath_len;

        g_current_player_pos = res.final_player_pos;
        g_total_cost += res.cost;

        uint16_t start_idx = 0;
        if (g_fullpath_len > 0 && pos_equal(g_fullpath[g_fullpath_len - 1], res.path_points[0]))
            start_idx = 1;
        uint16_t add_len = (uint16_t)(res.path_len - start_idx);
        if (g_fullpath_len + add_len <= MAX_PATH_LEN) {
            for (uint16_t i = start_idx; i < res.path_len; i++)
                g_fullpath[g_fullpath_len++] = res.path_points[i];
        }

        if (backtrack_validate(sol)) return true;

        g_solved[try_idx] = false;
        g_order_idx--;
        g_current_player_pos = saved_player;
        g_total_cost = saved_cost;
        g_fullpath_len = saved_len;
    }
    return false;
}

/**
 * @brief 验证解决方案的主入口函数
 * 
 * 初始化回溯状态，调用 backtrack_validate 递归验证
 * 成功后按求解顺序重新排序配对（共线排序）
 * 
 * @param sol 待验证的解决方案
 */
static void validate_solution(SolutionSequence* sol) {
    if (sol->count == 0 || g_box_count != g_target_count) {
        sol->is_valid = false; return;
    }
    memset(g_solved, 0, sizeof(g_solved));
    g_order_idx = 0;
    g_current_player_pos = g_initial_player;
    g_total_cost = 0;
    g_fullpath_len = 0;
    if (!backtrack_validate(sol)) { sol->is_valid = false; return; }
    BoxTargetPair opt[MAX_BOXES];
    for (uint8_t i = 0; i < sol->count; i++) opt[i] = sol->pairs[g_solve_order[i]];
    for (uint8_t i = 0; i < sol->count; i++) sol->pairs[i] = opt[i];
}

/* ---------- 6.4 模式2/ID学习与推理 ---------- */

/**
 * @brief ID学习阶段：生成访问计划
 * 
 * 通过BFS距离计算，规划依次访问所有箱子和目标的顺序路径
 * 先全局确定"最后一个"元素，再排除最后一个后生成访问顺序
 */
static void id_learning(uint8_t map[MAP_ROWS][MAP_COLS]) {
    parse_map_input(map);
    /* 计算所有未访问元素 */
    for (uint8_t b = 0; b < g_bomb_count; b++)
        g_static_walls[g_initial_bombs[b].x] |= (1 << g_initial_bombs[b].y);
    uint16_t walls[MAP_ROWS] = {0};
    for (int i = 0; i < MAP_ROWS; i++)
        for (int j = 0; j < MAP_COLS; j++)
            if (map[i][j] == WALL || map[i][j] == BOX || map[i][j] == BOOM)
                walls[i] |= (1 << j);

    if (g_box_count == 0 || g_target_count == 0) { g_visit_count = 0; return; }

    /* 找到距离最近的"最后一个"元素 */
    bool tvb[MAX_BOXES] = {false}, tvt[MAX_BOXES] = {false};
    Point tcp = g_initial_player;
    int last_box = -1, last_target = -1;
    int total = g_box_count + g_target_count;

    for (int k = 0; k < total; k++) {
        bfs_compute_distances(tcp, walls);
        uint16_t md = INF;
        int bei = -1; uint8_t bt = 0; Point ba = {0,0};
        for (int i = 0; i < g_box_count; i++) {
            if (tvb[i]) continue;
            for (int d = 0; d < DIR_COUNT; d++) {
                Point ap = {g_initial_boxes[i].x + DIRS[d][0], g_initial_boxes[i].y + DIRS[d][1]};
                if (ap.x >= MAP_ROWS || ap.y >= MAP_COLS) continue;
                if (walls[ap.x] & (1 << ap.y)) continue;
                uint16_t raw = g_dist_map[ap.x][ap.y];
                uint16_t eff = (raw < INF && DIRS[d][1] != -1) ? (uint16_t)(raw + ROTATION_PENALTY) : raw;
                if (eff < md) { md = eff; bei = i; bt = 0; ba = ap; }
            }
        }
        for (int i = 0; i < g_target_count; i++) {
            if (tvt[i]) continue;
            Point tp = g_initial_targets[i];
            for (int d = 0; d < DIR_COUNT; d++) {
                Point ap = {tp.x + DIRS[d][0], tp.y + DIRS[d][1]};
                if (ap.x >= MAP_ROWS || ap.y >= MAP_COLS) continue;
                if (walls[ap.x] & (1 << ap.y)) continue;
                uint16_t raw = g_dist_map[ap.x][ap.y];
                uint16_t eff = (raw < INF && DIRS[d][1] != -1) ? (uint16_t)(raw + ROTATION_PENALTY) : raw;
                if (eff < md) { md = eff; bei = i; bt = 1; ba = ap; }
            }
        }
        if (bei == -1) break;
        if (bt == 0) { last_box = bei; tvb[bei] = true; }
        else         { last_target = bei; tvt[bei] = true; }
        tcp = ba;
    }

    /* 排除最后一个元素后生成访问顺序 */
    bool vb[MAX_BOXES] = {false}, vt[MAX_BOXES] = {false};
    if (last_box >= 0)    vb[last_box] = true;
    if (last_target >= 0) vt[last_target] = true;

    Point cp = g_initial_player;
    g_visit_count = 0;
    int to_visit = 0;
    for (int i = 0; i < g_box_count; i++)    if (!vb[i]) to_visit++;
    for (int i = 0; i < g_target_count; i++) if (!vt[i]) to_visit++;

    while (g_visit_count < to_visit) {
        bfs_compute_distances(cp, walls);
        uint16_t md = INF;
        int bei = -1; uint8_t bt = 0; Point ba = {0,0};
        for (int i = 0; i < g_box_count; i++) {
            if (vb[i]) continue;
            for (int d = 0; d < DIR_COUNT; d++) {
                Point ap = {g_initial_boxes[i].x + DIRS[d][0], g_initial_boxes[i].y + DIRS[d][1]};
                if (ap.x >= MAP_ROWS || ap.y >= MAP_COLS) continue;
                if (walls[ap.x] & (1 << ap.y)) continue;
                uint16_t raw = g_dist_map[ap.x][ap.y];
                uint16_t eff = (raw < INF && DIRS[d][1] != -1) ? (uint16_t)(raw + ROTATION_PENALTY) : raw;
                if (eff < md) { md = eff; bei = i; bt = 0; ba = ap; }
            }
        }
        for (int i = 0; i < g_target_count; i++) {
            if (vt[i]) continue;
            Point tp = g_initial_targets[i];
            for (int d = 0; d < DIR_COUNT; d++) {
                Point ap = {tp.x + DIRS[d][0], tp.y + DIRS[d][1]};
                if (ap.x >= MAP_ROWS || ap.y >= MAP_COLS) continue;
                if (walls[ap.x] & (1 << ap.y)) continue;
                uint16_t raw = g_dist_map[ap.x][ap.y];
                uint16_t eff = (raw < INF && DIRS[d][1] != -1) ? (uint16_t)(raw + ROTATION_PENALTY) : raw;
                if (eff < md) { md = eff; bei = i; bt = 1; ba = ap; }
            }
        }
        if (bei == -1) break;

        VisitStep* step = &g_visit_plan[g_visit_count];
        step->pos = ba;
        step->original_idx = (uint8_t)bei;
        step->type = (bt == 0) ? BOX : TARGET;
        uint16_t tw[MAP_ROWS];
        memcpy(tw, walls, sizeof(tw));
        tw[ba.x] &= ~(1 << ba.y);
        step->path_len = simple_astar(cp, ba, tw, step->path);
        step->step_cost = step->path_len;
        Point ep = (step->type == BOX) ? g_initial_boxes[bei] : g_initial_targets[bei];
        int8_t dxep = (int8_t)(ep.x - ba.x);
        int8_t dyep = (int8_t)(ep.y - ba.y);
        step->angle = compute_approach_angle(dxep, dyep);
        if (dxep == -1)      step->direction = 0;
        else if (dxep == 1)  step->direction = 1;
        else if (dyep == -1) step->direction = 2;
        else if (dyep == 1)  step->direction = 3;
        else                 step->direction = DIR_INVALID;
        if (bt == 0) vb[bei] = true; else vt[bei] = true;
        g_visit_count++;
        cp = ba;
    }
}

/**
 * @brief 记录用户输入的ID并绑定到当前观察步骤的元素
 * 
 * 仅在观察阶段（g_current_step < g_visit_count）有效
 * 
 * @param id 用户扫描得到的ID数据
 */
static void id_record(int id) {
    if (g_current_step >= g_visit_count) return;
    VisitStep* step = &g_visit_plan[g_current_step];
    step->scanned_id = (int8_t)id;
    if (step->type == BOX) g_box_id_map[step->original_idx] = (int8_t)id;
    else                   g_target_id_map[step->original_idx] = (int8_t)id;
}

/**
 * @brief 下一字典序排列（用于ID同组全排列优化）
 */
static bool next_permutation(int *arr, int n) {
    int i = n - 2;
    while (i >= 0 && arr[i] >= arr[i + 1]) i--;
    if (i < 0) return false;
    int j = n - 1;
    while (arr[j] <= arr[i]) j--;
    int t = arr[i]; arr[i] = arr[j]; arr[j] = t;
    for (int a = i + 1, b = n - 1; a < b; a++, b--) { t = arr[a]; arr[a] = arr[b]; arr[b] = t; }
    return true;
}

/**
 * @brief ID推理：根据已知ID推断未知元素
 * 
 * 核心逻辑：统计概率、平衡分配、同ID组内优化（最小曼哈顿距离匹配）
 */
static void id_inference(void) {
    int bfreq[MAX_IDS] = {0}, tfreq[MAX_IDS] = {0};
    int ubi[MAX_BOXES], uti[MAX_BOXES], ubc = 0, utc = 0;
    for (int i = 0; i < g_box_count; i++) {
        if (g_box_id_map[i] != -1) bfreq[g_box_id_map[i]]++;
        else ubi[ubc++] = i;
    }
    for (int i = 0; i < g_target_count; i++) {
        if (g_target_id_map[i] != -1) tfreq[g_target_id_map[i]]++;
        else uti[utc++] = i;
    }
    for (int id = 0; id < MAX_IDS; id++) {
        if (bfreq[id] > tfreq[id]) {
            int need = bfreq[id] - tfreq[id];
            while (need > 0 && utc > 0) { g_target_id_map[uti[--utc]] = (int8_t)id; tfreq[id]++; need--; }
        } else if (bfreq[id] < tfreq[id]) {
            int need = tfreq[id] - bfreq[id];
            while (need > 0 && ubc > 0) { g_box_id_map[ubi[--ubc]] = (int8_t)id; bfreq[id]++; need--; }
        }
    }
    while (ubc > 0 && utc > 0) {
        int nid = 0;
        while (nid < MAX_IDS && (bfreq[nid] > 0 || tfreq[nid] > 0)) nid++;
        if (nid >= MAX_IDS) break;
        g_box_id_map[ubi[--ubc]] = (int8_t)nid;
        g_target_id_map[uti[--utc]] = (int8_t)nid;
        bfreq[nid] = 1; tfreq[nid] = 1;
    }
    for (int id = 0; id < MAX_IDS; id++) {
        if (bfreq[id] >= 2 && tfreq[id] >= 2 && bfreq[id] == tfreq[id]) {
            int k = bfreq[id];
            int bi[MAX_BOXES], ti[MAX_BOXES], bc = 0, tc = 0;
            for (int i = 0; i < g_box_count; i++)    if (g_box_id_map[i] == id) bi[bc++] = i;
            for (int i = 0; i < g_target_count; i++) if (g_target_id_map[i] == id) ti[tc++] = i;

            /* 预计算 BFS 距离矩阵 dist[i][j] = 箱子bi[i]到目标ti[j]的最短距离 */
            uint16_t dist[MAX_BOXES][MAX_BOXES];
            for (int i = 0; i < k; i++) {
                bfs_compute_distances(g_initial_boxes[bi[i]], g_static_walls);
                for (int j = 0; j < k; j++) {
                    dist[i][j] = g_dist_map[g_initial_targets[ti[j]].x]
                                              [g_initial_targets[ti[j]].y];
                }
            }

            int bperm[MAX_BOXES] = {0}, minc = 0x7FFFFFFF, perm[MAX_BOXES];
            for (int i = 0; i < k; i++) perm[i] = i;
            do {
                int cost = 0;
                for (int i = 0; i < k; i++)
                    cost += (int)dist[i][perm[i]];
                if (cost < minc) { minc = cost; for (int i = 0; i < k; i++) bperm[i] = perm[i]; }
            } while (next_permutation(perm, k));
            bool occ[MAX_IDS] = {false};
            for (int i = 0; i < MAX_IDS; i++) if (bfreq[i] > 0 || tfreq[i] > 0) occ[i] = true;
            for (int i = 1; i < k; i++) {
                int nid = 0;
                while (nid < MAX_IDS && occ[nid]) nid++;
                if (nid >= MAX_IDS) break;
                g_box_id_map[bi[i]] = (int8_t)nid;
                g_target_id_map[ti[bperm[i]]] = (int8_t)nid;
                bfreq[id]--; tfreq[id]--;
                bfreq[nid] = 1; tfreq[nid] = 1; occ[nid] = true;
            }
        }
    }
    memset(g_id_pairing, -1, sizeof(g_id_pairing));
    bool bex[MAX_IDS] = {false}, tex[MAX_IDS] = {false};
    for (int i = 0; i < g_box_count; i++)    if (g_box_id_map[i] != -1) bex[g_box_id_map[i]] = true;
    for (int i = 0; i < g_target_count; i++) if (g_target_id_map[i] != -1) tex[g_target_id_map[i]] = true;
    for (int i = 0; i < MAX_IDS; i++) if (bex[i] && tex[i]) g_id_pairing[i] = i;
}

/**
 * @brief 从ID观察计划中提取拐点路径
 */
static void extract_look_turn_points(VisitStep* plan) {
    int li = 0;
    if (g_visit_count > 0 && plan[0].path_len > 0) {
        Point sp = plan[0].path[0];
        g_path_look_out.x[li] = sp.y; g_path_look_out.y[li] = sp.x;
        g_path_look_out.angle[li] = 0; g_path_look_out.type[li] = 0; g_path_look_out.is_look[li] = 0;
        li++;
    }
    for (int i = 0; i < g_visit_count; i++) {
        if (plan[i].path_len == 0) continue;
        Point prev = (li > 0) ? (Point){g_path_look_out.y[li-1], g_path_look_out.x[li-1]} : plan[i].path[0];
        for (int j = 0; j < plan[i].path_len; j++) {
            Point cur = plan[i].path[j];
            if (j == plan[i].path_len - 1) {
                if (li >= MAX_PATH_LEN) break;
                g_path_look_out.x[li] = cur.y; g_path_look_out.y[li] = cur.x;
                g_path_look_out.angle[li] = plan[i].angle;
                g_path_look_out.type[li] = plan[i].type;
                g_path_look_out.is_look[li] = 1; li++;
            } else {
                Point nxt = plan[i].path[j+1];
                int8_t dx1 = (int8_t)(cur.x - prev.x), dy1 = (int8_t)(cur.y - prev.y);
                int8_t dx2 = (int8_t)(nxt.x - cur.x), dy2 = (int8_t)(nxt.y - cur.y);
                if ((dx1 != dx2 || dy1 != dy2) && !(li > 0 && g_path_look_out.y[li-1] == cur.x && g_path_look_out.x[li-1] == cur.y)) {
                    if (li >= MAX_PATH_LEN) break;
                    g_path_look_out.x[li] = cur.y; g_path_look_out.y[li] = cur.x;
                    g_path_look_out.angle[li] = 0; g_path_look_out.type[li] = 0; g_path_look_out.is_look[li] = 0; li++;
                }
            }
            prev = cur;
        }
    }
    g_path_look_out.len = (uint16_t)li;
}

/**
 * @brief 根据ID配对构建解决方案
 */
static SolutionSequence* build_solution_from_id_pairing(void) {
    memset(&g_id_based_sol, 0, sizeof(SolutionSequence));
    uint8_t pc = 0;
    for (int id = 0; id < MAX_IDS; id++) {
        if (g_id_pairing[id] == -1) continue;
        int bi = -1, ti = -1;
        for (int i = 0; i < g_box_count; i++)    if (g_box_id_map[i] == id) { bi = i; break; }
        for (int i = 0; i < g_target_count; i++) if (g_target_id_map[i] == id) { ti = i; break; }
        if (bi >= 0 && ti >= 0 && pc < MAX_BOXES) {
            g_id_based_sol.pairs[pc].box_idx = (uint8_t)bi;
            g_id_based_sol.pairs[pc].target_idx = (uint8_t)ti;
            g_id_based_sol.pairs[pc].box_pos = g_initial_boxes[bi];
            g_id_based_sol.pairs[pc].target_pos = g_initial_targets[ti];
            pc++;
        }
    }
    g_id_based_sol.count = pc;
    g_id_based_sol.is_valid = false;
    g_id_based_sol.total_cost = 0;
    g_id_based_sol.full_path_len = 0;
    return (pc > 0) ? &g_id_based_sol : NULL;
}

/**
 * @brief 根据ID解决方案生成最终路径
 */
static Path path_id_calculate(SolutionSequence* sol) {
    Path rp = {0};
    if (sol == NULL || sol->count == 0) return rp;
    validate_solution(sol);
    if (!sol->is_valid) return rp;
    uint16_t len = (sol->full_path_len > MAX_PATH_LEN) ? MAX_PATH_LEN : sol->full_path_len;
    rp.len = len;
    for (uint16_t i = 0; i < len; i++) {
        rp.x[i] = sol->full_path[i].y;
        rp.y[i] = sol->full_path[i].x;
        rp.is_push[i] = 0;
    }
    extract_turn_points(&rp);
    return rp;
}

/* ---------- 6.5 模式3炸弹破局分析 ---------- */

/**
 * @brief 计算玩家可达区域（箱子作为障碍物）
 */
static void compute_player_region_with_walls(const uint16_t walls[MAP_ROWS], bool ignore_bombs) {
    memset(g_obs_buf, 0, sizeof(g_obs_buf));
    for (uint8_t i = 0; i < MAP_ROWS; i++) for (uint8_t j = 0; j < MAP_COLS; j++) {
        uint8_t v = g_original_map[i][j];
        if (v == WALL) continue;
        if (v == BOX) g_obs_buf[i] |= (1 << j);
        if (v == BOOM && !ignore_bombs) g_obs_buf[i] |= (1 << j);
    }
    memset(g_player_region, 0, sizeof(g_player_region));
    for (uint8_t i = 0; i < MAP_ROWS; i++) for (uint8_t j = 0; j < MAP_COLS; j++)
        if (walls[i] & (1 << j)) g_player_region[i][j] = true;
    uint8_t head = 0, tail = 0;
    g_player_region[g_initial_player.x][g_initial_player.y] = true;
    g_bfs_queue[tail++] = g_initial_player;
    while (head < tail) {
        Point curr = g_bfs_queue[head++];
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            Point next = {(uint8_t)(curr.x + DIRS[d][0]), (uint8_t)(curr.y + DIRS[d][1])};
            if (next.x >= MAP_ROWS || next.y >= MAP_COLS) continue;
            if (g_player_region[next.x][next.y]) continue;
            if (walls[next.x] & (1 << next.y)) continue;
            if (g_obs_buf[next.x] & (1 << next.y)) continue;
            g_player_region[next.x][next.y] = true;
            g_bfs_queue[tail++] = next;
        }
    }
}

static void compute_player_region(void) {
    compute_player_region_with_walls(g_static_walls, false);
}

/**
 * @brief 计算箱子的影响区域（可达区域位图）
 */
static uint16_t compute_box_influence(Point box, const uint16_t walls[MAP_ROWS],
    uint16_t influence_mask[MAP_ROWS]) {
    memset(influence_mask, 0, MAP_ROWS * sizeof(uint16_t));
    g_dist_epoch++;
    if (g_dist_epoch == 0) { memset(g_dist_epoch_tag, 0, sizeof(g_dist_epoch_tag)); g_dist_epoch = 1; }
    uint16_t head = 0, tail = 0, count = 0;
    g_dist_epoch_tag[box.x][box.y] = g_dist_epoch;
    g_bfs_queue[tail++] = box;
    influence_mask[box.x] |= (1 << box.y); count++;
    while (head < tail) {
        Point curr = g_bfs_queue[head++];
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            Point next = {(uint8_t)(curr.x + DIRS[d][0]), (uint8_t)(curr.y + DIRS[d][1])};
            if (next.x >= MAP_ROWS || next.y >= MAP_COLS) continue;
            if (is_wall_bit(walls, next)) continue;
            { /* 跳过无效炸弹 (0xFF=已失效) */
                bool blocked_by_bomb = false;
                for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
                    if (g_initial_bombs[bi].x == 0xFF) continue; /* 跳过无效炸弹 */
                    if (g_initial_bombs[bi].x == next.x && g_initial_bombs[bi].y == next.y)
                        { blocked_by_bomb = true; break; }
                }
                if (blocked_by_bomb) continue;
            }
            if (g_dist_epoch_tag[next.x][next.y] == g_dist_epoch) continue;
            g_dist_epoch_tag[next.x][next.y] = g_dist_epoch;
            g_bfs_queue[tail++] = next;
            influence_mask[next.x] |= (1 << next.y); count++;
        }
    }
    return count;
}

/**
 * @brief 箱影响区是否包含任何目标
 */
static bool influence_contains_target(const uint16_t influence[MAP_ROWS]) {
    for (uint8_t t = 0; t < g_target_count; t++) {
        Point tp = g_initial_targets[t];
        if (influence[tp.x] & (1 << tp.y)) return true;
    }
    return false;
}

/**
 * @brief 某个推箱子是否能到达任一目标
 */
static bool box_can_reach_any_target(Point box, const uint16_t walls[MAP_ROWS],
    const Point targets[], uint8_t target_count) {
    for (uint8_t t = 0; t < target_count; t++)
        if (box.x == targets[t].x && box.y == targets[t].y) return true;
    g_dist_epoch++;
    if (g_dist_epoch == 0) { memset(g_dist_epoch_tag, 0, sizeof(g_dist_epoch_tag)); g_dist_epoch = 1; }
    uint16_t head = 0, tail = 0;
    g_dist_epoch_tag[box.x][box.y] = g_dist_epoch;
    g_bfs_queue[tail++] = box;
    while (head < tail) {
        Point curr = g_bfs_queue[head++];
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            Point next = {(uint8_t)(curr.x + DIRS[d][0]), (uint8_t)(curr.y + DIRS[d][1])};
            if (next.x >= MAP_ROWS || next.y >= MAP_COLS) continue;
            if (is_wall_bit(walls, next)) continue;
            { /* 跳过无效炸弹 (0xFF=已失效) */
                bool blocked_by_bomb = false;
                for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
                    if (g_initial_bombs[bi].x == 0xFF) continue;
                    if (g_initial_bombs[bi].x == next.x && g_initial_bombs[bi].y == next.y)
                        { blocked_by_bomb = true; break; }
                }
                if (blocked_by_bomb) continue;
            }
            if (g_dist_epoch_tag[next.x][next.y] == g_dist_epoch) continue;
            for (uint8_t t = 0; t < target_count; t++)
                if (next.x == targets[t].x && next.y == targets[t].y) return true;
            g_dist_epoch_tag[next.x][next.y] = g_dist_epoch;
            g_bfs_queue[tail++] = next;
        }
    }
    return false;
}

/**
 * @brief 推箱子BFS仿真模拟（验证箱子是否死锁）
 */
static bool simulate_box_deadlock(uint8_t box_idx, const uint16_t obstacles[MAP_ROWS],
    const Point targets[], uint8_t target_count, const uint16_t walls[MAP_ROWS],
    Point start_player, Point start_box) {
    for (uint8_t t = 0; t < target_count; t++)
        if (start_box.x == targets[t].x && start_box.y == targets[t].y) return false;
    g_bfs_epoch++; if (g_bfs_epoch == 0) { memset(g_bfs_visited, 0, sizeof(g_bfs_visited)); g_bfs_epoch = 1; }
    memcpy(g_obs_buf, obstacles, sizeof(g_obs_buf));
    /* 排除当前箱子，其他箱子作为障碍 */
    for (uint8_t i = 0; i < g_box_count; i++) {
        if (i != box_idx)
            g_obs_buf[g_initial_boxes[i].x] |= (1 << g_initial_boxes[i].y);
    }
    /* 炸弹也作为障碍，玩家不能穿过（去引爆） */
    for (uint8_t i = 0; i < g_bomb_count; i++) {
        if (g_initial_bombs[i].x != 0xFF) /* 已失效的炸弹 */
            g_obs_buf[g_initial_bombs[i].x] |= (1 << g_initial_bombs[i].y);
    }
    uint16_t head = 0, tail = 0;
    { uint32_t idx = BFS_VISITED_INDEX(start_player.x, start_player.y, start_box.x, start_box.y);
      g_bfs_visited[idx] = g_bfs_epoch; }
    g_sim_queue[tail++] = ENCODE_STATE(start_player.x, start_player.y, start_box.x, start_box.y);
    while (head < tail) {
        uint16_t s = g_sim_queue[head++];
        uint8_t spx = DECODE_PX(s), spy = DECODE_PY(s);
        uint8_t sbx = DECODE_BX(s), sby = DECODE_BY(s);
        for (uint8_t t = 0; t < target_count; t++)
            if (sbx == targets[t].x && sby == targets[t].y) return false;
        uint16_t saved_obs_row = g_obs_buf[sbx];
        g_obs_buf[sbx] |= (1 << sby);
        bfs_compute_reachability((Point){spx, spy}, g_obs_buf);
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            uint8_t dest_x = (uint8_t)(sbx + DIRS[d][0]), dest_y = (uint8_t)(sby + DIRS[d][1]);
            if (dest_x >= MAP_ROWS || dest_y >= MAP_COLS) continue;
            if (is_wall_bit(walls, (Point){dest_x, dest_y})) continue;
            /* 箱子不能推到另一个箱子或炸弹占的格子 */
            if (g_obs_buf[dest_x] & (1 << dest_y)) continue;
            { bool blocked_by_bomb = false;
              for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
                if (g_initial_bombs[bi].x == 0xFF) continue; /* 已失效 */
                if (g_initial_bombs[bi].x == dest_x && g_initial_bombs[bi].y == dest_y)
                    { blocked_by_bomb = true; break; }
              }
              if (blocked_by_bomb) continue; }
            uint8_t push_px = (uint8_t)(sbx - DIRS[d][0]), push_py = (uint8_t)(sby - DIRS[d][1]);
            if (push_px >= MAP_ROWS || push_py >= MAP_COLS) continue;
            if (is_wall_bit(walls, (Point){push_px, push_py})) continue;
            if (push_px == sbx && push_py == sby) continue;
            if (g_dist_epoch_tag[push_px][push_py] != g_dist_epoch) continue;
            uint32_t nidx = BFS_VISITED_INDEX(push_px, push_py, dest_x, dest_y);
            if (g_bfs_visited[nidx] != g_bfs_epoch) {
                g_bfs_visited[nidx] = g_bfs_epoch;
                if (tail >= MAX_SIM_QUEUE) { g_obs_buf[sbx] = saved_obs_row; return true; }
                g_sim_queue[tail++] = ENCODE_STATE(push_px, push_py, dest_x, dest_y);
            }
        }
        g_obs_buf[sbx] = saved_obs_row;
    }
    return true;
}

/**
 * @brief 推箱子BFS验证：固定箱子+固定目标，确认箱子能否推到目标
 * 
 * 与 simulate_box_deadlock 逻辑相同但目标是第一确定目标
 * @return true  = 箱子可推到目标
 * @return false = 箱子无法到达目标位置
 */
static bool simulate_box_to_target(uint8_t box_idx, Point target,
                                    const uint16_t walls[MAP_ROWS],
                                    Point start_player, Point start_box) {
    /* 跳过无效炸弹 */
    if (start_box.x == target.x && start_box.y == target.y) return true;

    g_bfs_epoch++;
    if (g_bfs_epoch == 0) { memset(g_bfs_visited, 0, sizeof(g_bfs_visited)); g_bfs_epoch = 1; }

    /* 障碍物 = 墙 + 其他箱子（固定） */
    memset(g_obs_buf, 0, sizeof(g_obs_buf));
    for (uint8_t i = 0; i < MAP_ROWS; i++)
        g_obs_buf[i] = walls[i];
    for (uint8_t i = 0; i < g_box_count; i++) {
        if (i != box_idx)
            g_obs_buf[g_initial_boxes[i].x] |= (1 << g_initial_boxes[i].y);
    }
    /* 炸弹也作为障碍，玩家不能穿过（去引爆） */
    for (uint8_t i = 0; i < g_bomb_count; i++) {
        if (g_initial_bombs[i].x != 0xFF)
            g_obs_buf[g_initial_bombs[i].x] |= (1 << g_initial_bombs[i].y);
    }

    uint16_t head = 0, tail = 0;
    {
        uint32_t idx = BFS_VISITED_INDEX(start_player.x, start_player.y,
                                          start_box.x, start_box.y);
        g_bfs_visited[idx] = g_bfs_epoch;
    }
    g_sim_queue[tail++] = ENCODE_STATE(start_player.x, start_player.y,
                                        start_box.x, start_box.y);

    while (head < tail) {
        uint16_t s = g_sim_queue[head++];
        uint8_t spx = DECODE_PX(s), spy = DECODE_PY(s);
        uint8_t sbx = DECODE_BX(s), sby = DECODE_BY(s);

        /* 箱子不能推到墙或箱子格子 */
        if (sbx == target.x && sby == target.y) return true;

        uint16_t saved_obs_row = g_obs_buf[sbx];
        g_obs_buf[sbx] |= (1 << sby);
        bfs_compute_reachability((Point){spx, spy}, g_obs_buf);

        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            uint8_t dest_x = (uint8_t)(sbx + DIRS[d][0]);
            uint8_t dest_y = (uint8_t)(sby + DIRS[d][1]);
            if (dest_x >= MAP_ROWS || dest_y >= MAP_COLS) continue;
            if (is_wall_bit(walls, (Point){dest_x, dest_y})) continue;
            if (g_obs_buf[dest_x] & (1 << dest_y)) continue;
            {
                bool blocked_by_bomb = false;
                for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
                    if (g_initial_bombs[bi].x == 0xFF) continue; /* 已失效 */
                    if (g_initial_bombs[bi].x == dest_x &&
                        g_initial_bombs[bi].y == dest_y)
                        { blocked_by_bomb = true; break; }
                }
                if (blocked_by_bomb) continue;
            }
            uint8_t push_px = (uint8_t)(sbx - DIRS[d][0]);
            uint8_t push_py = (uint8_t)(sby - DIRS[d][1]);
            if (push_px >= MAP_ROWS || push_py >= MAP_COLS) continue;
            if (is_wall_bit(walls, (Point){push_px, push_py})) continue;
            if (push_px == sbx && push_py == sby) continue;
            if (g_dist_epoch_tag[push_px][push_py] != g_dist_epoch) continue;

            uint32_t nidx = BFS_VISITED_INDEX(push_px, push_py, dest_x, dest_y);
            if (g_bfs_visited[nidx] != g_bfs_epoch) {
                g_bfs_visited[nidx] = g_bfs_epoch;
                if (tail >= MAX_SIM_QUEUE) {
                    g_obs_buf[sbx] = saved_obs_row; return false;
                }
                g_sim_queue[tail++] = ENCODE_STATE(push_px, push_py, dest_x, dest_y);
            }
        }
        g_obs_buf[sbx] = saved_obs_row;
    }
    return false;
}

/**
 * @brief 统一检测所有死锁问题并生成问题点
 * 
 * 对每个箱子影响区域检测三种问题类型：
 * - 封闭区域（玩家不可达）
 * - 隔离区域（影响区不含目标）
 * - 需仿真验证（BFS无法直接确认）
 */
static void detect_and_generate_problems(void) {
    g_problem_count = 0;
    g_enclosed_region_count = 0;

    /* 1. 计算玩家可达区域（箱子作为障碍物，排除炸弹路径时含 PROBLEM_ENCLOSED） */
    compute_player_region_with_walls(g_static_walls, false);

    /* 2. 对每个箱子计算影响区域 */
    bool box_handled[MAX_BOXES] = {0};
    for (uint8_t bi = 0; bi < g_box_count; bi++) {
        if (box_handled[bi]) continue;
        Point bp = g_initial_boxes[bi];
        uint16_t infl[MAP_ROWS];
        compute_box_influence(bp, g_static_walls, infl);
        bool has_target = influence_contains_target(infl);
        bool player_can_reach = false;
        for (uint8_t ri = 0; ri < MAP_ROWS && !player_can_reach; ri++) {
            uint16_t row = infl[ri];
            for (uint8_t rj = 0; rj < MAP_COLS; rj++)
                if ((row & (1 << rj)) && g_player_region[ri][rj]) { player_can_reach = true; break; }
        }

        DeadlockProblemPoint *pp = &g_problem_points[g_problem_count];
        memset(pp, 0, sizeof(DeadlockProblemPoint));

        if (!player_can_reach) {
            pp->type = PROBLEM_ENCLOSED;
            pp->region_id = (int8_t)g_enclosed_region_count;
            if (g_enclosed_region_count < MAX_ENCLOSED_REGIONS) {
                memcpy(pp->influence_mask, infl, MAP_ROWS * sizeof(uint16_t));
                g_enclosed_region_count++;
            }
            for (uint8_t bj = 0; bj < g_box_count; bj++) {
                Point bp2 = g_initial_boxes[bj];
                if (infl[bp2.x] & (1 << bp2.y)) { pp->box_indices[pp->box_count++] = bj; box_handled[bj] = true; }
            }
            for (uint8_t tj = 0; tj < g_target_count; tj++) {
                Point tp = g_initial_targets[tj];
                if (infl[tp.x] & (1 << tp.y)) pp->target_indices[pp->target_count++] = tj;
            }
            snprintf(pp->description, sizeof(pp->description), "封闭区域 #%d", pp->region_id);
        } else if (!has_target) {
            pp->type = PROBLEM_SEPARATED;
            pp->region_id = -1;
            pp->box_indices[0] = bi; pp->box_count = 1;
            memcpy(pp->influence_mask, infl, MAP_ROWS * sizeof(uint16_t));
            snprintf(pp->description, sizeof(pp->description), "箱子 #%d 隔离区域", bi);
            box_handled[bi] = true;
        } else {
            if (simulate_box_deadlock(bi, g_static_walls, g_initial_targets, g_target_count,
                g_static_walls, g_initial_player, bp)) {
                pp->type = PROBLEM_NEED_SIM;
                pp->region_id = -1;
                pp->box_indices[0] = bi; pp->box_count = 1;
                memcpy(pp->influence_mask, infl, MAP_ROWS * sizeof(uint16_t));
                snprintf(pp->description, sizeof(pp->description), "箱子#%d 需仿真验证", bi);
                box_handled[bi] = true;
            }
        }
        if (pp->type != 0) g_problem_count++;
        if (g_problem_count >= MAX_PROBLEM_POINTS) break;
    }

    /* 3. 检查每个箱子是否死锁 */
    if (g_problem_count < MAX_PROBLEM_POINTS) {
        for (uint8_t ti = 0; ti < g_target_count; ti++) {
            Point tp = g_initial_targets[ti];
            bool covered = false;
            for (uint8_t pi = 0; pi < g_problem_count && !covered; pi++)
                for (uint8_t b = 0; b < g_problem_points[pi].target_count; b++)
                    if (g_problem_points[pi].target_indices[b] == ti) { covered = true; break; }
            if (covered) continue;
            if (g_player_region[tp.x][tp.y]) continue;

            uint16_t catchment[MAP_ROWS];
            compute_box_influence(tp, g_static_walls, catchment);
            bool has_box = false;
            for (uint8_t bj = 0; bj < g_box_count && !has_box; bj++) {
                Point bp = g_initial_boxes[bj];
                if (catchment[bp.x] & (1 << bp.y)) has_box = true;
            }
            DeadlockProblemPoint *pp = &g_problem_points[g_problem_count];
            memset(pp, 0, sizeof(DeadlockProblemPoint));
            pp->type = PROBLEM_ENCLOSED;
            pp->region_id = (int8_t)g_enclosed_region_count;
            memcpy(pp->influence_mask, catchment, MAP_ROWS * sizeof(uint16_t));
            if (g_enclosed_region_count < MAX_ENCLOSED_REGIONS) {
                g_enclosed_region_count++;
            }
            for (uint8_t tj = 0; tj < g_target_count; tj++) {
                Point tp2 = g_initial_targets[tj];
                if (catchment[tp2.x] & (1 << tp2.y)) pp->target_indices[pp->target_count++] = tj;
            }
            if (has_box) {
                for (uint8_t bj = 0; bj < g_box_count; bj++) {
                    Point bp = g_initial_boxes[bj];
                    if (catchment[bp.x] & (1 << bp.y)) pp->box_indices[pp->box_count++] = bj;
                }
                snprintf(pp->description, sizeof(pp->description), "封闭区域 #%d(含箱子)", pp->region_id);
            } else {
                snprintf(pp->description, sizeof(pp->description), "封闭区域 #%d(无箱子)", pp->region_id);
            }
            g_problem_count++;
            if (g_problem_count >= MAX_PROBLEM_POINTS) break;
        }
    }

    /* 4. 对尚未解决任何死锁问题的区域进行处理 */
    if (g_problem_count < MAX_PROBLEM_POINTS && g_box_count > 0) {
        /* 收集问题点 */
        bool target_covered[MAX_BOXES] = {false};
        for (uint8_t pi = 0; pi < g_problem_count; pi++) {
            for (uint8_t t = 0; t < g_problem_points[pi].target_count; t++)
                target_covered[g_problem_points[pi].target_indices[t]] = true;
        }
        /* 如果仍未解决，标记为需仿真验证 */
        uint8_t uncov[MAX_BOXES], uncov_cnt = 0;
        uint16_t uncov_dist[MAX_BOXES];
        for (uint8_t ti = 0; ti < g_target_count; ti++) {
            if (target_covered[ti]) continue;
            /* 查找可炸墙候选位置 */
            Point tp = g_initial_targets[ti];
            bool has_push_dir = false;
            for (uint8_t d = 0; d < DIR_COUNT && !has_push_dir; d++) {
                /* 距离 d 已超出爆炸范围 */
                uint8_t box_from_x = (uint8_t)(tp.x - DIRS[d][0]);
                uint8_t box_from_y = (uint8_t)(tp.y - DIRS[d][1]);
                uint8_t push_x    = (uint8_t)(tp.x - 2 * DIRS[d][0]);
                uint8_t push_y    = (uint8_t)(tp.y - 2 * DIRS[d][1]);
                if (box_from_x >= MAP_ROWS || box_from_y >= MAP_COLS) continue;
                if (push_x    >= MAP_ROWS || push_y    >= MAP_COLS) continue;
                if (is_wall_bit(g_static_walls, (Point){box_from_x, box_from_y})) continue;
                if (is_wall_bit(g_static_walls, (Point){push_x, push_y})) continue;
                has_push_dir = true;
            }
            if (!has_push_dir) {
                /* 检查墙是否在封闭区域边界上 */
                DeadlockProblemPoint *pp = &g_problem_points[g_problem_count];
                memset(pp, 0, sizeof(DeadlockProblemPoint));
                pp->type = PROBLEM_TARGET_UNREACHABLE;
                pp->region_id = -1;
                pp->target_indices[0] = ti; pp->target_count = 1;
                uint16_t infl[MAP_ROWS];
                compute_box_influence(tp, g_static_walls, infl);
                memcpy(pp->influence_mask, infl, MAP_ROWS * sizeof(uint16_t));
                snprintf(pp->description, sizeof(pp->description),
                         "目标 #%d 不可达", ti);
                g_problem_count++;
                target_covered[ti] = true;
                if (g_problem_count >= MAX_PROBLEM_POINTS) break;
                continue;
            }
            /* 生成炸弹方案 */
            uint16_t min_d = INF;
            for (uint8_t bi = 0; bi < g_box_count; bi++) {
                uint16_t d = manhattan_distance(g_initial_boxes[bi], tp);
                if (d < min_d) min_d = d;
            }
            uncov[uncov_cnt] = ti;
            uncov_dist[uncov_cnt] = min_d;
            uncov_cnt++;
        }
        /* 评估方案收益 */
        for (uint8_t i = 1; i < uncov_cnt; i++) {
            uint8_t key_t = uncov[i];
            uint16_t key_d = uncov_dist[i];
            int8_t j = (int8_t)(i - 1);
            while (j >= 0 && uncov_dist[j] > key_d) {
                uncov[j + 1] = uncov[j];
                uncov_dist[j + 1] = uncov_dist[j];
                j--;
            }
            uncov[j + 1] = key_t;
            uncov_dist[j + 1] = key_d;
        }

        /* 重新BFS验证修改墙后是否仍死锁 */
        for (uint8_t ui = 0; ui < uncov_cnt; ui++) {
            if (g_problem_count >= MAX_PROBLEM_POINTS) break;
            uint8_t ti = uncov[ui];
            if (target_covered[ti]) continue;
            Point tp = g_initial_targets[ti];

            /* 需要仿真验证 */
            uint8_t box_order[MAX_BOXES];
            uint16_t box_dist[MAX_BOXES];
            for (uint8_t bi = 0; bi < g_box_count; bi++) {
                box_order[bi] = bi;
                box_dist[bi] = manhattan_distance(g_initial_boxes[bi], tp);
            }
            for (uint8_t a = 1; a < g_box_count; a++) {
                uint8_t kb = box_order[a]; uint16_t kd = box_dist[a];
                int8_t j = (int8_t)(a - 1);
                while (j >= 0 && box_dist[j] > kd) {
                    box_order[j + 1] = box_order[j];
                    box_dist[j + 1] = box_dist[j]; j--;
                }
                box_order[j + 1] = kb; box_dist[j + 1] = kd;
            }

            bool any_box_reaches = false;
            for (uint8_t bi = 0; bi < g_box_count; bi++) {
                uint8_t box_idx = box_order[bi];
                Point bp = g_initial_boxes[box_idx];
                if (simulate_box_to_target(box_idx, tp, g_static_walls,
                                            g_initial_player, bp)) {
                    any_box_reaches = true; break;
                }
            }

            if (!any_box_reaches) {
                DeadlockProblemPoint *pp = &g_problem_points[g_problem_count];
                memset(pp, 0, sizeof(DeadlockProblemPoint));
                pp->type = PROBLEM_TARGET_UNREACHABLE;
                pp->region_id = -1;
                pp->target_indices[0] = ti; pp->target_count = 1;
                uint16_t infl[MAP_ROWS];
                compute_box_influence(tp, g_static_walls, infl);
                memcpy(pp->influence_mask, infl, MAP_ROWS * sizeof(uint16_t));
                snprintf(pp->description, sizeof(pp->description),
                         "目标 #%d 不可达", ti);
                g_problem_count++;
            }
        }
    }

}

/**
 * @brief 统一死锁检测入口
 */
static DeadlockResult detect_all_deadlocks(uint8_t map[MAP_ROWS][MAP_COLS]) {
    parse_map_input(map);
    DeadlockResult result = {0};
    detect_and_generate_problems();
    for (uint8_t pi = 0; pi < g_problem_count; pi++)
        if (g_problem_points[pi].type & PROBLEM_ENCLOSED)
            for (uint8_t b = 0; b < g_problem_points[pi].box_count; b++)
                result.enclosed_indices[result.enclosed_count++] = g_problem_points[pi].box_indices[b];
    result.has_deadlock = (g_problem_count > 0);
    return result;
}

/**
 * @brief 增量问题验证：验证修改墙后的问题是否解决
 */
static uint8_t check_problems_resolved_incremental(const uint16_t modified_walls[MAP_ROWS], Point player_pos) {
    if (g_saved_problem_count == 0) return 0xFF;
    memcpy(g_saved_walls, g_static_walls, sizeof(g_static_walls));
    memcpy(g_saved_player_region, g_player_region, sizeof(g_player_region));
    memcpy(g_static_walls, modified_walls, sizeof(g_static_walls));
    compute_player_region_with_walls(modified_walls, false);
    uint8_t mask = 0;
    for (uint8_t pi = 0; pi < g_saved_problem_count; pi++) {
        const DeadlockProblemPoint *pp = &g_saved_problem_points[pi];
        bool resolved = true;
        if (pp->type & PROBLEM_ENCLOSED) {
            bool reachable = false;
            /* 在 modified_walls 下重新计算影响区（炸墙后墙已移除） */
            {
                Point seed = (pp->box_count > 0) ? g_initial_boxes[pp->box_indices[0]]
                          : g_initial_targets[pp->target_indices[0]];
                uint16_t new_infl[MAP_ROWS];
                compute_box_influence(seed, modified_walls, new_infl);
                for (uint8_t ri = 0; ri < MAP_ROWS && !reachable; ri++) {
                    uint16_t row = new_infl[ri] & ~modified_walls[ri];
                    for (uint8_t rj = 0; rj < MAP_COLS; rj++)
                        if ((row & (1 << rj)) && g_player_region[ri][rj]) { reachable = true; break; }
                }
            }
            if (!reachable) resolved = false;
            if (resolved && pp->box_count > 0) {
                for (uint8_t b = 0; b < pp->box_count && resolved; b++) {
                    uint8_t box_idx = pp->box_indices[b];
                    Point bp = g_initial_boxes[box_idx];
                    if (!box_can_reach_any_target(bp, modified_walls, g_initial_targets, g_target_count))
                        resolved = false;
                    else if (simulate_box_deadlock(box_idx, modified_walls, g_initial_targets,
                        g_target_count, modified_walls, player_pos, bp))
                        resolved = false;
                }
            }
        }
        if (pp->type & PROBLEM_SEPARATED) {
            uint8_t box_idx = pp->box_indices[0];
            uint16_t new_infl[MAP_ROWS];
            compute_box_influence(g_initial_boxes[box_idx], modified_walls, new_infl);
            if (!influence_contains_target(new_infl)) resolved = false;
        }
        if (pp->type & PROBLEM_NEED_SIM) {
            uint8_t box_idx = pp->box_indices[0];
            Point bp = g_initial_boxes[box_idx];
            if (!box_can_reach_any_target(bp, modified_walls, g_initial_targets, g_target_count))
                resolved = false;
            else if (simulate_box_deadlock(box_idx, modified_walls, g_initial_targets,
                g_target_count, modified_walls, player_pos, bp))
                resolved = false;
        }
        if (pp->type & PROBLEM_TARGET_UNREACHABLE) {
            /* 预计算炸弹可达区域 */
            uint8_t ti = pp->target_indices[0];
            Point tp = g_initial_targets[ti];
            bool any_box_reaches = false;
            for (uint8_t bi = 0; bi < g_box_count; bi++) {
                Point bp = g_initial_boxes[bi];
                if (simulate_box_to_target(bi, tp, modified_walls,
                                            player_pos, bp)) {
                    any_box_reaches = true; break;
                }
            }
            if (!any_box_reaches) resolved = false;
        }
        if (resolved) mask |= (1 << pi);
    }
    memcpy(g_static_walls, g_saved_walls, sizeof(g_static_walls));
    memcpy(g_player_region, g_saved_player_region, sizeof(g_player_region));
    return mask;
}

/**
 * @brief 缓存版本的问题验证
 */
static uint8_t check_problems_resolved_cached(const uint16_t modified_walls[MAP_ROWS], Point player_pos) {
    uint32_t h = hash_walls(modified_walls);
    h = (h ^ ((uint32_t)player_pos.x << 8 | player_pos.y));
    uint16_t idx = (uint16_t)(h % VCACHE_SIZE);
    if (g_vcache_epoch_tag[idx] == g_vcache_global_epoch && g_vcache_hash[idx] == h) return g_vcache_mask[idx];
    uint8_t mask = check_problems_resolved_incremental(modified_walls, player_pos);
    g_vcache_hash[idx] = h; g_vcache_mask[idx] = mask; g_vcache_epoch_tag[idx] = g_vcache_global_epoch;
    return mask;
}

/**
 * @brief 预计算各炸弹位置的可达区域
 *
 * 对每个炸弹执行一次BFS，生成可达位图实现 O(1) 查询
 * 障碍物：墙 + 其他箱子 + 其他炸弹（本炸弹位置可通过）
 */
static void precompute_bomb_reachability(void)
{
    memset(g_bomb_reach_map, 0, sizeof(g_bomb_reach_map));

    for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
        Point bomb_pos = g_initial_bombs[bi];
        memset(g_obs_buf, 0, sizeof(g_obs_buf));
        g_dist_epoch++;
        if (g_dist_epoch == 0) { memset(g_dist_epoch_tag, 0, sizeof(g_dist_epoch_tag)); g_dist_epoch = 1; }

        /* 初始化位图 */
        for (uint8_t i = 0; i < MAP_ROWS; i++) {
            for (uint8_t j = 0; j < MAP_COLS; j++) {
                uint8_t v = g_original_map[i][j];
                if (v == WALL) {
                    g_obs_buf[i] |= (1 << j);
                } else if ((v == BOX || v == BOOM) &&
                           !(i == bomb_pos.x && j == bomb_pos.y)) {
                    g_obs_buf[i] |= (1 << j);
                }
            }
        }

        /* BFS可达性计算 */
        uint8_t head = 0, tail = 0;
        g_dist_epoch_tag[bomb_pos.x][bomb_pos.y] = g_dist_epoch;
        g_bfs_queue[tail++] = bomb_pos;

        while (head < tail) {
            Point curr = g_bfs_queue[head++];
            g_bomb_reach_map[bi][curr.x] |= (1 << curr.y);

            for (uint8_t d = 0; d < DIR_COUNT; d++) {
                Point next = {
                    (uint8_t)(curr.x + DIRS[d][0]),
                    (uint8_t)(curr.y + DIRS[d][1])
                };
                if (next.x >= MAP_ROWS || next.y >= MAP_COLS) continue;
                if (g_dist_epoch_tag[next.x][next.y] == g_dist_epoch) continue;
                if (g_obs_buf[next.x] & (1 << next.y)) continue;
                g_dist_epoch_tag[next.x][next.y] = g_dist_epoch;
                g_bfs_queue[tail++] = next;
            }
        }
    }
}

/**
 * @brief 获取目标墙的爆炸候选位置
 */
static void find_detonation_points_for_wall(Point target_wall, Point out_points[], uint8_t *out_count) {
    *out_count = 0;
    for (int8_t dx = -1; dx <= 1; dx++) for (int8_t dy = -1; dy <= 1; dy++) {
        Point dp = {(uint8_t)(target_wall.x + dx), (uint8_t)(target_wall.y + dy)};
        if (dp.x >= MAP_ROWS || dp.y >= MAP_COLS) continue;
        if (!is_valid_detonation_point(dp)) continue;
        out_points[(*out_count)++] = dp;
        if (*out_count >= MAX_DETONATE_POINTS) return;
    }
}

/**
 * @brief 在封闭区域周围找可炸墙
 *
 * 遍历地图中所有可炸墙，检查其是否在封闭区域边界上
 *
 * @param region_mask 封闭区域掩码
 * @param out_walls   输出可炸墙列表
 * @param out_count   输出墙数量
 */
static void find_walls_for_enclosed(const uint16_t region_mask[MAP_ROWS],
    BreakableWall out_walls[], uint8_t *out_count)
{
    *out_count = 0;

    for (uint8_t i = 0; i < MAP_ROWS; i++) {
        for (uint8_t j = 0; j < MAP_COLS; j++) {
            Point p = {i, j};
            if (!is_breakable_wall(p)) continue;

            for (uint8_t d = 0; d < DIR_COUNT; d++) {
                uint8_t ni = (uint8_t)(i + DIRS[d][0]);
                uint8_t nj = (uint8_t)(j + DIRS[d][1]);
                if (ni >= MAP_ROWS || nj >= MAP_COLS) continue;
                if (g_original_map[ni][nj] == WALL) continue;
                if (region_mask[ni] & (1 << nj)) {
                    bool found = false;
                    for (uint8_t k = 0; k < *out_count; k++)
                        if (pos_equal(out_walls[k].wall_pos, p)) {
                            found = true; break;
                        }
                    if (!found && *out_count < MAX_BREAK_WALLS) {
                        out_walls[*out_count].wall_pos = p;
                        out_walls[*out_count].deadlock_type = DEADLOCK_ENCLOSED;
                        out_walls[*out_count].benefit_score = -10;
                        (*out_count)++;
                    }
                    break;
                }
            }
        }
    }
}

/**
 * @brief 在隔离区域边界找可炸墙
 */
static void find_walls_for_separated(const uint16_t influence[MAP_ROWS],
    BreakableWall out_walls[], uint8_t *out_count) {
    *out_count = 0;
    for (uint8_t i = 0; i < MAP_ROWS; i++) for (uint8_t j = 0; j < MAP_COLS; j++) {
        Point p = {i, j}; if (!is_breakable_wall(p)) continue;
        bool touches_influence = false, touches_outside = false;
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            uint8_t ni = (uint8_t)(i + DIRS[d][0]), nj = (uint8_t)(j + DIRS[d][1]);
            if (ni >= MAP_ROWS || nj >= MAP_COLS) continue;
            if (g_original_map[ni][nj] == WALL) continue;
            if (influence[ni] & (1 << nj)) touches_influence = true;
            else touches_outside = true;
        }
        if (touches_influence && touches_outside && *out_count < MAX_BREAK_WALLS) {
            out_walls[*out_count].wall_pos = p;
            out_walls[*out_count].deadlock_type = DEADLOCK_BOX;
            out_walls[*out_count].benefit_score = -5; (*out_count)++;
        }
    }
}

/**
 * @brief 为可炸墙生成炸弹方案
 */
static void generate_plans_for_walls(const BreakableWall breakable_walls[], uint8_t wall_count,
    DetonatePlan out_plans[], uint8_t *out_plan_count) {
    if (wall_count == 0 || g_bomb_count == 0) return;
    compute_player_region();
    for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
        Point bomb_pos = g_initial_bombs[bi];
        for (uint8_t wi = 0; wi < wall_count; wi++) {
            Point wall_pos = breakable_walls[wi].wall_pos;
            Point candidates[8]; uint8_t cand_count = 0;
            find_detonation_points_for_wall(wall_pos, candidates, &cand_count);
            for (uint8_t ci = 0; ci < cand_count; ci++) {
                Point dp = candidates[ci];
                bool bomb_can_reach = false;
                /* 利用预计算位图检查炸弹可达性 */
                for (uint8_t d = 0; d < DIR_COUNT; d++) {
                    uint8_t adj_x = (uint8_t)(dp.x + DIRS[d][0]);
                    uint8_t adj_y = (uint8_t)(dp.y + DIRS[d][1]);
                    if (adj_x < MAP_ROWS && adj_y < MAP_COLS &&
                        !is_wall_bit(g_static_walls, (Point){adj_x, adj_y}) &&
                        (g_bomb_reach_map[bi][adj_x] & (1 << adj_y))) {
                        bomb_can_reach = true; break;
                    }
                }
                if (!bomb_can_reach) continue;
                Point covered_walls[9]; uint8_t covered_count = 0;
                for (uint8_t wj = 0; wj < wall_count; wj++)
                    if (can_explosion_cover_wall(dp, breakable_walls[wj].wall_pos))
                        covered_walls[covered_count++] = breakable_walls[wj].wall_pos;
                if (covered_count == 0) continue;
                uint16_t sim_walls[MAP_ROWS];
                memcpy(sim_walls, g_static_walls, sizeof(g_static_walls));
                for (uint8_t k = 0; k < covered_count; k++) {
                    Point wp = covered_walls[k]; sim_walls[wp.x] &= (uint16_t)~(1 << wp.y);
                }
                /* 评估方案收益和代价 */
                uint8_t saved_bomb = g_original_map[bomb_pos.x][bomb_pos.y];
                g_original_map[bomb_pos.x][bomb_pos.y] = FLOOR;
                Point saved_ibomb = g_initial_bombs[bi];
                g_initial_bombs[bi].x = 0xFF;
                uint8_t resolved_mask = check_problems_resolved_cached(sim_walls, g_initial_player);
                bool resolves = (resolved_mask == (uint8_t)((1 << g_saved_problem_count) - 1));
                g_initial_bombs[bi] = saved_ibomb;
                g_original_map[bomb_pos.x][bomb_pos.y] = saved_bomb;
                uint16_t push_dist = estimate_bomb_push_distance(bomb_pos, dp, g_initial_player);
                int total_benefit = (int)push_dist;
                for (uint8_t k = 0; k < covered_count; k++)
                    for (uint8_t wj = 0; wj < wall_count; wj++)
                        if (pos_equal(covered_walls[k], breakable_walls[wj].wall_pos))
                            { total_benefit += breakable_walls[wj].benefit_score; break; }
                /* 规则#8：优先炸影响多个目标的墙 */
                {
                    int target_bonus = 0;
                    for (uint8_t pi = 0; pi < g_saved_problem_count; pi++)
                        if (resolved_mask & (1 << pi))
                            target_bonus += g_saved_problem_points[pi].target_count;
                    total_benefit -= target_bonus * 10;  /* 每个目标加10使目标方向靠前 */
                }
                if (!resolves) total_benefit += 100;
                bool duplicate = false; uint8_t dupe_idx = 0;
                for (uint8_t pi = 0; pi < *out_plan_count; pi++)
                    if (out_plans[pi].bomb_index == bi && pos_equal(out_plans[pi].detonate_pos, dp))
                        { dupe_idx = pi; duplicate = true; break; }
                if (duplicate) {
                    if (covered_count > out_plans[dupe_idx].wall_count) {
                        out_plans[dupe_idx].wall_count = covered_count;
                        memcpy(out_plans[dupe_idx].walls_covered, covered_walls, covered_count * sizeof(Point));
                        out_plans[dupe_idx].total_benefit = total_benefit;
                        out_plans[dupe_idx].resolves_deadlock = resolves;
                        out_plans[dupe_idx].resolved_mask = resolved_mask;
                    } continue;
                }
                DetonatePlan *plan = &out_plans[*out_plan_count];
                plan->detonate_pos = dp; plan->bomb_index = bi; plan->bomb_initial_pos = bomb_pos;
                plan->bomb_push_distance = push_dist; plan->wall_count = covered_count;
                memcpy(plan->walls_covered, covered_walls, covered_count * sizeof(Point));
                plan->total_benefit = total_benefit; plan->resolves_deadlock = resolves;
                plan->resolved_mask = resolved_mask; plan->affected_mask = resolved_mask;
                /* 炸弹推送方案排序择优 */
                {   bool can_push = false;
                    for (uint8_t d = 0; d < DIR_COUNT && !can_push; d++) {
                        uint8_t nx = (uint8_t)(bomb_pos.x + DIRS[d][0]);
                        uint8_t ny = (uint8_t)(bomb_pos.y + DIRS[d][1]);
                        uint8_t px = (uint8_t)(bomb_pos.x - DIRS[d][0]);
                        uint8_t py = (uint8_t)(bomb_pos.y - DIRS[d][1]);
                        if (nx >= MAP_ROWS || ny >= MAP_COLS || px >= MAP_ROWS || py >= MAP_COLS) continue;
                        /* 按收益排序，取最优方案 */
                        if (is_wall_bit(g_static_walls, (Point){px, py})) continue;
                        bool blocked = false;
                        for (uint8_t bj = 0; bj < g_box_count && !blocked; bj++) {
                            if (g_initial_boxes[bj].x == nx && g_initial_boxes[bj].y == ny) blocked = true;
                            if (g_initial_boxes[bj].x == px && g_initial_boxes[bj].y == py) blocked = true;
                        }
                        for (uint8_t bj = 0; bj < g_bomb_count && !blocked; bj++) {
                            if (bj == bi) continue;
                            if (g_initial_bombs[bj].x == nx && g_initial_bombs[bj].y == ny) blocked = true;
                            if (g_initial_bombs[bj].x == px && g_initial_bombs[bj].y == py) blocked = true;
                        }
                        if (!blocked) can_push = true;
                    }
                    if (!can_push) continue;
                }
                (*out_plan_count)++;
                if (*out_plan_count >= MAX_DETONATE_POINTS) return;
            }
        }
    }
}

/**
 * @brief 按方案收益排序（代价值小的优先）
 */
static void sort_plans_by_benefit(DetonatePlan plans[], uint8_t count)
{
    for (uint8_t i = 1; i < count; i++) {
        DetonatePlan key = plans[i];
        int8_t j = (int8_t)(i - 1);
        while (j >= 0 && plans[j].total_benefit > key.total_benefit) {
            plans[j + 1] = plans[j];
            j--;
        }
        plans[j + 1] = key;
    }
}

/**
 * @brief 递归枚举方案组合（剪枝 + 验证）
 */
static void try_multi_plans_recursive(
    uint8_t depth, uint8_t max_depth,
    const uint8_t bomb_idx[], const uint8_t bl[],
    DetonatePlan pb[][MAX_PLANS_PER_BOOM], const uint8_t fi[],
    uint8_t sel_plans[], int accum_benefit,
    const uint8_t all_mask, int *best_benefit,
    DetonatePlan result_plans[], uint8_t *result_count, uint8_t *found)
{
    if (*found) return;
    if (depth == max_depth) {
        uint16_t cw[MAP_ROWS];
        memcpy(cw, g_static_walls, sizeof(cw));
        for (uint8_t i = 0; i < max_depth; i++) {
            uint8_t ba = bl[bomb_idx[i]];
            DetonatePlan *p = &pb[ba][sel_plans[i]];
            for (uint8_t k = 0; k < p->wall_count; k++) {
                Point wp = p->walls_covered[k];
                cw[wp.x] &= (uint16_t)~(1 << wp.y);
            }
        }
        Point saved[MAX_BOOMS];
        for (uint8_t i = 0; i < max_depth; i++) {
            uint8_t ba = bl[bomb_idx[i]];
            saved[i] = g_initial_bombs[ba];
            g_initial_bombs[ba].x = 0xFF;
        }
        if (check_problems_resolved_cached(cw, g_initial_player) == all_mask) {
            if (accum_benefit < *best_benefit) {
                *best_benefit = accum_benefit;
                *result_count = max_depth;
                for (uint8_t i = 0; i < max_depth; i++)
                    result_plans[i] = pb[bl[bomb_idx[i]]][sel_plans[i]];
            }
            *found = 1;
        }
        for (uint8_t i = 0; i < max_depth; i++)
            g_initial_bombs[bl[bomb_idx[i]]] = saved[i];
        return;
    }
    uint8_t bi = bomb_idx[depth];
    uint8_t ba = bl[bi];
    for (uint8_t pi = 0; pi < fi[ba]; pi++) {
        int new_benefit = accum_benefit + pb[ba][pi].total_benefit;
        if (new_benefit >= *best_benefit) break;
        sel_plans[depth] = pi;
        try_multi_plans_recursive(depth + 1, max_depth, bomb_idx, bl, pb, fi,
            sel_plans, new_benefit, all_mask, best_benefit,
            result_plans, result_count, found);
    }
}

/**
 * @brief 对 m 个炸弹枚举组合，尝试找到 2/3/4 组组合
 */
static bool search_multi_combo(
    uint8_t target_m, const uint8_t N, const uint8_t bl[],
    DetonatePlan pb[][MAX_PLANS_PER_BOOM], const uint8_t fi[],
    const uint8_t all_mask, int *best_benefit,
    DetonatePlan result_plans[], uint8_t *result_count)
{
    uint8_t bomb_idx[MAX_BOOMS];
    for (uint8_t i = 0; i < target_m; i++) bomb_idx[i] = i;
    uint8_t sel_plans[MAX_BOOMS];
    uint8_t found = 0;
    while (!found) {
        try_multi_plans_recursive(0, target_m, bomb_idx, bl, pb, fi,
            sel_plans, 0, all_mask, best_benefit,
            result_plans, result_count, &found);
        if (found) return true;
        int8_t k = (int8_t)target_m - 1;
        while (k >= 0 && bomb_idx[k] == N - target_m + k) k--;
        if (k < 0) break;
        bomb_idx[k]++;
        for (uint8_t j = (uint8_t)(k + 1); j < target_m; j++)
            bomb_idx[j] = bomb_idx[j - 1] + 1;
    }
    return false;
}

/**
 * @brief 炸弹组合搜索入口（2/3/4炸弹组合，从少到多尝试）
 */
static bool search_multi_bomb_combination(DetonatePlan all_plans[], uint8_t plan_count,
    DetonatePlan result_plans[], uint8_t *result_count, int *out_total_benefit) {
    *result_count = 0;
    if (plan_count == 0 || g_bomb_count == 0 || g_saved_problem_count == 0) return false;
    const uint8_t all_mask = (uint8_t)((1 << g_saved_problem_count) - 1);

    for (uint8_t pi = 0; pi < plan_count; pi++)
        if (all_plans[pi].resolved_mask == all_mask) {
            uint16_t test_walls[MAP_ROWS];
            memcpy(test_walls, g_static_walls, sizeof(g_static_walls));
            for (uint8_t k = 0; k < all_plans[pi].wall_count; k++) {
                Point wp = all_plans[pi].walls_covered[k];
                test_walls[wp.x] &= (uint16_t)~(1 << wp.y);
            }
            if (check_problems_resolved_cached(test_walls, g_initial_player) == all_mask) {
                result_plans[0] = all_plans[pi]; *result_count = 1;
                *out_total_benefit = all_plans[pi].total_benefit; return true;
            }
        }

    uint8_t nplans[MAX_BOOMS]; memset(nplans, 0, sizeof(nplans));
    for (uint8_t pi = 0; pi < plan_count; pi++) nplans[all_plans[pi].bomb_index]++;

    DetonatePlan pb[MAX_BOOMS][MAX_PLANS_PER_BOOM];
    uint8_t fi[MAX_BOOMS]; memset(fi, 0, sizeof(fi));
    for (uint8_t pi = 0; pi < plan_count; pi++) {
        uint8_t bi = all_plans[pi].bomb_index;
        if (fi[bi] < MAX_PLANS_PER_BOOM) pb[bi][fi[bi]++] = all_plans[pi];
    }
    for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
        sort_plans_by_benefit(pb[bi], fi[bi]);
        uint8_t keep = 0;
        for (uint8_t pi = 0; pi < fi[bi]; pi++) {
            bool dup = false;
            for (uint8_t pj = 0; pj < keep; pj++) {
                if (pb[bi][pi].wall_count != pb[bi][pj].wall_count) continue;
                bool same = true;
                for (uint8_t wk = 0; wk < pb[bi][pi].wall_count; wk++)
                    if (!pos_equal(pb[bi][pi].walls_covered[wk], pb[bi][pj].walls_covered[wk]))
                        { same = false; break; }
                if (same) { dup = true; break; }
            }
            if (!dup && keep < MAX_PLANS_PER_BOOM) pb[bi][keep++] = pb[bi][pi];
        }
        #define BOOM_TOP_K 12
        if (keep > BOOM_TOP_K) keep = BOOM_TOP_K;
        fi[bi] = keep;
    }

    uint8_t bl[MAX_BOOMS]; uint8_t blc = 0;
    for (uint8_t bi = 0; bi < g_bomb_count; bi++)
        if (fi[bi] > 0) bl[blc++] = bi;
    if (blc < 2) return false;

    int best_benefit = 999999;
    const uint8_t N = blc;

    /* 尝试2/3/4炸弹组合破局 */
    if (search_multi_combo(2, N, bl, pb, fi, all_mask, &best_benefit, result_plans, result_count)) {
        *out_total_benefit = best_benefit; return true;
    }
    if (N >= 3 && search_multi_combo(3, N, bl, pb, fi, all_mask, &best_benefit, result_plans, result_count)) {
        *out_total_benefit = best_benefit; return true;
    }
    if (N >= 4 && search_multi_combo(4, N, bl, pb, fi, all_mask, &best_benefit, result_plans, result_count)) {
        *out_total_benefit = best_benefit; return true;
    }
    return false;
}

/**
 * @brief 验证炸弹推送可行性（用推箱A*确认能否推到引爆点）
 *
 * 计算"推"动作的起始位和"炸"位置
 * 推箱A*计算推箱路径和最终位置
 *
 * @param plan 炸弹方案
 * @return true 方案可行
 */
static bool verify_bomb_push_feasibility(const DetonatePlan *plan)
{
    uint16_t obs[MAP_ROWS];
    memcpy(obs, g_static_walls, sizeof(obs));

    /* 清除引爆点位置的墙障碍（爆炸后该格变为地板） */
    obs[plan->detonate_pos.x] &= (uint16_t)~(1 << plan->detonate_pos.y);

    /* 炸弹初始位置 */
    for (uint8_t i = 0; i < g_box_count; i++)
        obs[g_initial_boxes[i].x] |= (1 << g_initial_boxes[i].y);

    /* 检查炸弹是否有效 0xFF=已失效 */
    for (uint8_t i = 0; i < g_bomb_count; i++) {
        if (i == plan->bomb_index) continue;
        if (g_initial_bombs[i].x == 0xFF) continue;
        obs[g_initial_bombs[i].x] |= (1 << g_initial_bombs[i].y);
    }

    AStarResult ar = solve_single_box_a_star(
        g_initial_player, plan->bomb_initial_pos, plan->detonate_pos, obs);
    return ar.success;
}

/**
 * @brief 炸弹破局分析主函数
 */
static bool analyze_bomb_breakthrough(const DeadlockResult *deadlock, DetonatePlan out_plans[],
    uint8_t *out_plan_count, int *out_best_benefit) {
    if (g_bomb_count == 0 || !deadlock->has_deadlock) { *out_plan_count = 0; return false; }
    memcpy(g_saved_problem_points, g_problem_points, sizeof(DeadlockProblemPoint) * g_problem_count);
    g_saved_problem_count = g_problem_count;
    compute_player_region();
    precompute_bomb_reachability();

    DetonatePlan all_plans[MAX_DETONATE_POINTS];
    uint8_t all_plan_count = 0;

    for (uint8_t pi = 0; pi < g_problem_count; pi++) {
        BreakableWall walls[MAX_BREAK_WALLS];
        uint8_t wall_count = 0;
        if (g_problem_points[pi].type & PROBLEM_ENCLOSED)
            find_walls_for_enclosed(g_problem_points[pi].influence_mask, walls, &wall_count);
        if (g_problem_points[pi].type & PROBLEM_SEPARATED)
            find_walls_for_separated(g_problem_points[pi].influence_mask, walls, &wall_count);
        if (g_problem_points[pi].type & PROBLEM_NEED_SIM) {
            BreakableWall bw[MAX_BREAK_WALLS]; uint8_t bwc = 0;
            for (uint8_t bi = 0; bi < g_problem_points[pi].box_count; bi++) {
                Point box = g_initial_boxes[g_problem_points[pi].box_indices[bi]];
                for (uint8_t d = 0; d < DIR_COUNT; d++) {
                    Point wp = {(uint8_t)(box.x + DIRS[d][0]), (uint8_t)(box.y + DIRS[d][1])};
                    if (wp.x >= MAP_ROWS || wp.y >= MAP_COLS) continue;
                    if (!is_wall_bit(g_static_walls, wp)) continue;
                    if (wp.x == 0 || wp.x == MAP_ROWS-1 || wp.y == 0 || wp.y == MAP_COLS-1) continue;
                    bool found = false;
                    for (uint8_t k = 0; k < bwc; k++)
                        if (pos_equal(bw[k].wall_pos, wp)) { found = true; break; }
                    if (!found && bwc < MAX_BREAK_WALLS) {
                        bw[bwc].wall_pos = wp; bw[bwc].deadlock_type = DEADLOCK_BOX;
                        bw[bwc].benefit_score = -5; bwc++;
                    }
                }
            }
            for (uint8_t wi = 0; wi < bwc && wall_count < MAX_BREAK_WALLS; wi++) {
                bool found = false;
                for (uint8_t k = 0; k < wall_count; k++)
                    if (pos_equal(walls[k].wall_pos, bw[wi].wall_pos)) { found = true; break; }
                if (!found) walls[wall_count++] = bw[wi];
            }
        }
        if (g_problem_points[pi].type & PROBLEM_TARGET_UNREACHABLE) {
            /* 计算所有炸弹方案 + 评估收益 */
            find_walls_for_enclosed(g_problem_points[pi].influence_mask, walls, &wall_count);
            if (wall_count == 0) {
                /* 选择方案并验证是否解决死锁 */
                find_walls_for_separated(g_problem_points[pi].influence_mask, walls, &wall_count);
            }
            if (wall_count == 0) {
                /* 搜索墙在封闭区域周围 */
                uint8_t ti = g_problem_points[pi].target_indices[0];
                Point tp = g_initial_targets[ti];
                BreakableWall bw[MAX_BREAK_WALLS]; uint8_t bwc = 0;
                for (uint8_t d = 0; d < DIR_COUNT; d++) {
                    Point wp = {(uint8_t)(tp.x + DIRS[d][0]), (uint8_t)(tp.y + DIRS[d][1])};
                    if (wp.x >= MAP_ROWS || wp.y >= MAP_COLS) continue;
                    if (!is_breakable_wall(wp)) continue;
                    bool found2 = false;
                    for (uint8_t k = 0; k < bwc; k++)
                        if (pos_equal(bw[k].wall_pos, wp)) { found2 = true; break; }
                    if (!found2 && bwc < MAX_BREAK_WALLS) {
                        bw[bwc].wall_pos = wp; bw[bwc].deadlock_type = DEADLOCK_BOX;
                        bw[bwc].benefit_score = -5; bwc++;
                    }
                }
                for (uint8_t wi = 0; wi < bwc && wall_count < MAX_BREAK_WALLS; wi++) {
                    bool found2 = false;
                    for (uint8_t k = 0; k < wall_count; k++)
                        if (pos_equal(walls[k].wall_pos, bw[wi].wall_pos)) { found2 = true; break; }
                    if (!found2) walls[wall_count++] = bw[wi];
                }
            }
        }
        if (wall_count == 0) continue;
        generate_plans_for_walls(walls, wall_count, all_plans, &all_plan_count);
    }

    /* 从小到大尝试不同数量组合 */
    sort_plans_by_benefit(all_plans, all_plan_count);

    /* 缓存resolves_deadlock结果避免重复计算 */
    DetonatePlan best_plans[MAX_BOOMS];
    uint8_t best_count = 0;
    int best_overall_cost = 999999;

    for (uint8_t i = 0; i < all_plan_count; i++) {
        if (!all_plans[i].resolves_deadlock) continue;
        if (!verify_bomb_push_feasibility(&all_plans[i])) continue;
        int overall = (int)all_plans[i].bomb_push_distance;
        if (overall < best_overall_cost) {
            best_overall_cost = overall;
            best_plans[0] = all_plans[i]; best_count = 1;
            break;
        }
    }

    /* 验证方案：检查死锁 + 检查可达 + 第2轮验证 */
    if (best_count == 0 && g_bomb_count >= 2) {
        DetonatePlan combo_plans[MAX_BOOMS]; uint8_t combo_count = 0; int combo_benefit = 0;
        if (search_multi_bomb_combination(all_plans, all_plan_count, combo_plans, &combo_count, &combo_benefit)) {
            /* 若 combo 搜索成功则直接取第一个可行的 */
            if (combo_count == 1 && !verify_bomb_push_feasibility(&combo_plans[0])) {
                /* 递归组合搜索（剪枝加速） */
            } else if (combo_count >= 2) {
                /* 输出爆炸后的新地图数据 */
                bool all_pushable = true;
                for (uint8_t ci = 0; ci < combo_count && all_pushable; ci++) {
                    /* 格式化输出探索方案结果 */
                    Point saved_others[MAX_BOOMS];
                    for (uint8_t cj = 0; cj < combo_count; cj++) {
                        uint8_t bj = combo_plans[cj].bomb_index;
                        saved_others[cj] = g_initial_bombs[bj];
                        if (cj != ci) g_initial_bombs[bj].x = 0xFF;
                    }
                    if (!verify_bomb_push_feasibility(&combo_plans[ci]))
                        all_pushable = false;
                    /* 测试入口 */
                    for (uint8_t cj = 0; cj < combo_count; cj++)
                        g_initial_bombs[combo_plans[cj].bomb_index] = saved_others[cj];
                }
                if (all_pushable) {
                    best_count = combo_count;
                    best_overall_cost = combo_benefit;
                    for (uint8_t i = 0; i < combo_count; i++) best_plans[i] = combo_plans[i];
                }
            } else {
                best_count = combo_count;
                best_overall_cost = combo_benefit;
                for (uint8_t i = 0; i < combo_count; i++) best_plans[i] = combo_plans[i];
            }
        }
    }
    *out_plan_count = best_count;
    *out_best_benefit = best_overall_cost;
    memcpy(out_plans, best_plans, best_count * sizeof(DetonatePlan));
    return best_count > 0;
}

/* ---------- 6.6 模式3炸弹破局执行 ---------- */

/**
 * @brief BFS计算玩家到各炸弹的最短距离
 */
static uint8_t bfs_bomb_distances(Point player, const uint16_t walls[MAP_ROWS],
    const bool done[], uint8_t plan_count,
    const DetonatePlan plans[], uint16_t dists[MAX_BOOMS]) {
    for (uint8_t i = 0; i < plan_count; i++) dists[i] = INF;
    uint16_t obs[MAP_ROWS];
    memcpy(obs, walls, sizeof(uint16_t) * MAP_ROWS);
    for (uint8_t i = 0; i < g_box_count; i++)
        obs[g_initial_boxes[i].x] |= (1 << g_initial_boxes[i].y);
    g_dist_epoch++;
    if (g_dist_epoch == 0) { memset(g_dist_epoch_tag, 0, sizeof(g_dist_epoch_tag)); g_dist_epoch = 1; }
    uint16_t head = 0, tail = 0;
    g_dist_epoch_tag[player.x][player.y] = g_dist_epoch;
    g_bfs_queue[tail++] = player;
    for (uint8_t i = 0; i < MAP_ROWS; i++)
        for (uint8_t j = 0; j < MAP_COLS; j++)
            g_dist_map[i][j] = INF;
    g_dist_map[player.x][player.y] = 0;
    uint8_t found = 0;

    uint16_t bomb_mask[MAP_ROWS];
    memset(bomb_mask, 0, sizeof(bomb_mask));
    for (uint8_t pi = 0; pi < plan_count; pi++) {
        if (done[pi]) continue;
        Point bp = plans[pi].bomb_initial_pos;
        bomb_mask[bp.x] |= (1 << bp.y);
    }
    while (head < tail) {
        Point cur = g_bfs_queue[head++];
        uint16_t cd = g_dist_map[cur.x][cur.y];
        if (bomb_mask[cur.x] & (1 << cur.y)) {
            for (uint8_t pi = 0; pi < plan_count; pi++) {
                if (done[pi]) continue;
                if (pos_equal(cur, plans[pi].bomb_initial_pos)) {
                    if (dists[pi] == INF) { dists[pi] = cd; found++; }
                }
            }
        }
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            Point nx = {(uint8_t)(cur.x + DIRS[d][0]), (uint8_t)(cur.y + DIRS[d][1])};
            if (nx.x >= MAP_ROWS || nx.y >= MAP_COLS) continue;
            if (is_wall_bit(obs, nx)) continue;
            if (g_dist_epoch_tag[nx.x][nx.y] == g_dist_epoch) continue;
            g_dist_epoch_tag[nx.x][nx.y] = g_dist_epoch;
            g_dist_map[nx.x][nx.y] = (uint16_t)(cd + 1); g_bfs_queue[tail++] = nx;
        }
    }
    return found;
}

/**
 * @brief 计算单步：推炸弹到引爆点（墙位置），返回最短路径
 */
static void compute_one_bomb_push(Point bomb_pos, uint8_t bomb_index,
    Point detonate_pos, Point player_start,
    const uint16_t walls[MAP_ROWS], const bool active_bombs[MAX_BOOMS],
    BombPushPath *out_path) {
    memset(out_path, 0, sizeof(BombPushPath));
    if (pos_equal(bomb_pos, detonate_pos)) {
        out_path->path_valid = true; out_path->path_len = 1;
        out_path->path[0] = player_start; return;
    }
    uint16_t obs[MAP_ROWS];
    memcpy(obs, walls, sizeof(uint16_t) * MAP_ROWS);
    obs[detonate_pos.x] &= (uint16_t)~(1 << detonate_pos.y);
    for (uint8_t i = 0; i < g_box_count; i++) {
        Point bp = g_initial_boxes[i];
        if (!pos_equal(bp, bomb_pos)) obs[bp.x] |= (1 << bp.y);
    }
    for (uint8_t i = 0; i < g_bomb_count; i++) {
        if (i == bomb_index) continue;
        if (active_bombs && !active_bombs[i]) continue;
        obs[g_initial_bombs[i].x] |= (1 << g_initial_bombs[i].y);
    }
    AStarResult ar = solve_single_box_a_star(player_start, bomb_pos, detonate_pos, obs);
    if (!ar.success || ar.path_len == 0) return;
    uint16_t len = (ar.path_len > MAX_PATH_LEN) ? MAX_PATH_LEN : ar.path_len;
    out_path->path_len = len;
    memcpy(out_path->path, ar.path_points, len * sizeof(Point));
    out_path->path_valid = true;
}

/**
 * @brief 执行单步：推送炸弹+爆炸更新玩家位置和墙位置
 */
static bool try_execute_step(const DetonatePlan *p, uint8_t step_idx,
    const bool active_bombs[MAX_BOOMS], Point *player, uint16_t walls[MAP_ROWS],
    BombExecutionStep *out_st) {
    BombPushPath ppath;
    compute_one_bomb_push(p->bomb_initial_pos, p->bomb_index,
        p->detonate_pos, *player, walls, active_bombs, &ppath);
    if (!ppath.path_valid) return false;
    memset(out_st, 0, sizeof(BombExecutionStep));
    out_st->step_order = step_idx; out_st->bomb_index = p->bomb_index;
    out_st->bomb_from = p->bomb_initial_pos; out_st->detonate_at = p->detonate_pos;
    out_st->player_before = *player;
    memcpy(&out_st->push_path, &ppath, sizeof(BombPushPath));
    out_st->player_after = (ppath.path_len > 0) ? ppath.path[ppath.path_len - 1] : *player;
    *player = out_st->player_after;
    { Point dp = p->detonate_pos; uint8_t rc = 0;
    for (int8_t dx = -1; dx <= 1; dx++)
        for (int8_t dy = -1; dy <= 1; dy++) {
            uint8_t wx = (uint8_t)((int)dp.x + dx), wy = (uint8_t)((int)dp.y + dy);
            if (wx >= MAP_ROWS || wy >= MAP_COLS) continue;
            if (!(walls[wx] & (1 << wy))) continue;
            walls[wx] &= (uint16_t)~(1 << wy);
            if (rc < 9) { out_st->walls_removed[rc].x = wx; out_st->walls_removed[rc].y = wy; rc++; }
        }
    out_st->walls_removed_count = rc; }
    return true;
}

/**
 * @brief 贪心顺序推弹计划（BFS选择最短+拆墙逐步更新）
 * 
 * 炸弹推送到引爆点顺序规划
 */
static void plan_bomb_execution_sequence(const DetonatePlan plans[],
    uint8_t plan_count, BombExecutionPlan *out_exec) {
    memset(out_exec, 0, sizeof(BombExecutionPlan));
    if (plan_count == 0) return;
    uint16_t walls[MAP_ROWS];
    memcpy(walls, g_static_walls, sizeof(uint16_t) * MAP_ROWS);
    Point player = g_initial_player;
    bool done[MAX_BOOMS] = {false};
    uint8_t step_idx = 0;

    while (step_idx < plan_count) {
        if (step_idx + 1 == plan_count) {
            uint8_t pi = 0;
            while (pi < plan_count && done[pi]) pi++;
            if (pi >= plan_count) break;
            bool act_last[MAX_BOOMS];
            for (uint8_t j = 0; j < g_bomb_count; j++) act_last[j] = true;
            for (uint8_t j = 0; j < plan_count; j++)
                if (done[j]) act_last[plans[j].bomb_index] = false;
            if (try_execute_step(&plans[pi], step_idx, act_last, &player, walls, &out_exec->steps[step_idx]))
                { done[pi] = true; step_idx++; }
            break;
        }
        uint16_t dists[MAX_BOOMS];
        bfs_bomb_distances(player, walls, done, plan_count, plans, dists);
        uint8_t order[MAX_BOOMS], order_cnt = 0;
        for (uint8_t i = 0; i < plan_count; i++) {
            if (done[i] || dists[i] == INF) continue;
            uint8_t pos = order_cnt;
            while (pos > 0 && dists[order[pos - 1]] > dists[i]) { order[pos] = order[pos - 1]; pos--; }
            order[pos] = i; order_cnt++;
        }
        bool pushed = false;
        bool act[MAX_BOOMS];
        for (uint8_t j = 0; j < g_bomb_count; j++) act[j] = true;
        for (uint8_t j = 0; j < plan_count; j++)
            if (done[j]) act[plans[j].bomb_index] = false;
        for (uint8_t oi = 0; oi < order_cnt; oi++) {
            uint8_t pi = order[oi];
            if (try_execute_step(&plans[pi], step_idx, act, &player, walls, &out_exec->steps[step_idx])) {
                done[pi] = true; pushed = true; step_idx++; break;
            }
        }
        if (!pushed) break;
    }

    if (step_idx < plan_count && plan_count >= 2) {
        int order[MAX_BOOMS];
        for (int i = 0; i < (int)plan_count; i++) order[i] = i;
        uint16_t best_c = 0xFFFF; uint8_t best_n = 0;
        BombExecutionStep best_s[MAX_BOOMS];
        do {
            uint16_t w_try[MAP_ROWS]; memcpy(w_try, g_static_walls, sizeof(w_try));
            Point p_try = g_initial_player; bool d_try[MAX_BOOMS] = {false};
            BombExecutionStep t_s[MAX_BOOMS]; uint8_t t_n = 0; uint16_t t_c = 0;
            for (uint8_t si = 0; si < plan_count; si++) {
                uint8_t pi = (uint8_t)order[si];
                bool ab[MAX_BOOMS];
                for (uint8_t j = 0; j < g_bomb_count; j++) ab[j] = true;
                for (uint8_t j = 0; j < plan_count; j++)
                    if (d_try[j]) ab[plans[j].bomb_index] = false;
                if (!try_execute_step(&plans[pi], t_n, ab, &p_try, w_try, &t_s[t_n])) break;
                t_c = (uint16_t)(t_c + t_s[t_n].push_path.path_len);
                if (t_c >= best_c) break; /* 剪枝：累计代价已不小于当前最优 */
                d_try[pi] = true; t_n++;
            }
            if (t_n == plan_count && t_c < best_c) {
                best_c = t_c; best_n = t_n;
                memcpy(best_s, t_s, t_n * sizeof(BombExecutionStep));
            }
        } while (next_permutation(order, (int)plan_count));
        if (best_n == plan_count && best_c < 0xFFFF) {
            memcpy(out_exec->steps, best_s, best_n * sizeof(BombExecutionStep));
            out_exec->step_count = best_n; out_exec->total_cost = best_c;
            out_exec->is_valid = true; return;
        }
        /* 计算下一步：推炸弹 + 爆炸更新状态 */
        out_exec->is_valid = false;
        return;
    }
    if (step_idx == 0) { out_exec->is_valid = false; return; }
    out_exec->step_count = step_idx; out_exec->is_valid = true;
    uint16_t total = 0;
    for (uint8_t i = 0; i < step_idx; i++)
        total = (uint16_t)(total + out_exec->steps[i].push_path.path_len);
    out_exec->total_cost = total;
}

/* ===================================================================
 * 第7部分：对外接口
 * =================================================================== */

/**
 * @brief START模式对外接口：计算最近元素接近路径
 */
Path_Start path_start_calculation(uint8_t map[MAP_ROWS][MAP_COLS]) {
    memset(&g_path_start_out, 0, sizeof(Path_Start));
    ApproachResult* res = find_nearest_approach(map);
    if (res->success) extract_start_turn_points(res);
    return g_path_start_out;
}

/**
 * @brief 模式1（非ID配对）对外接口
 */
Path path_calculation(uint8_t box_x[MAP_ROWS][MAP_COLS]) {
    memset(&g_path_out, 0, sizeof(Path));
    parse_map_input(box_x);
    /* 执行贪心配对生成方案 */
    for (uint8_t b = 0; b < g_bomb_count; b++)
        g_static_walls[g_initial_bombs[b].x] |= (1 << g_initial_bombs[b].y);
    SolutionSequence sol;
    generate_greedy_pairing(&sol);
    g_mode1_strict = true;   /* 模式1严格模式：目标位也作为障碍物阻塞 */
    validate_solution(&sol);
    g_mode1_strict = false;
    if (sol.is_valid && sol.full_path_len > 0) {
        uint16_t len = (sol.full_path_len > MAX_PATH_LEN) ? MAX_PATH_LEN : sol.full_path_len;
        g_path_out.len = len;
        for (uint16_t i = 0; i < len; i++) {
            g_path_out.x[i] = sol.full_path[i].y;
            g_path_out.y[i] = sol.full_path[i].x;
            g_path_out.is_push[i] = 0;
        }
        extract_turn_points(&g_path_out);
    }
    return g_path_out;
}

/**
 * @brief 模式2/ID学习：观察阶段对外接口
 */
Path_Look path_look_calculation(uint8_t box_x[MAP_ROWS][MAP_COLS]) {
    memset(&g_path_look_out, 0, sizeof(Path_Look));
    id_learning(box_x);
    extract_look_turn_points(g_visit_plan);
    return g_path_look_out;
}

/**
 * @brief 记录用户输入的ID（ID模式回调函数）
 */
void id_input(int id) {
    if (g_current_step < g_visit_count) {
        id_record(id);
        g_current_step++;
    }
}

/**
 * @brief 模式2推理阶段对外接口
 */
Path path_id_calculation(void) {
    if (g_visit_count > 0) {
        g_initial_player = g_visit_plan[g_visit_count - 1].pos;
    }
    id_inference();
    SolutionSequence* id_sol = build_solution_from_id_pairing();
    return path_id_calculate(id_sol);
}

/**
 * @brief 模式3炸弹破局分析主对外接口
 */
Path path_boom_calculation(uint8_t map[MAP_ROWS][MAP_COLS]) {
    memset(&g_path_out, 0, sizeof(Path));
    memset(g_boom_final_walls, 0, sizeof(g_boom_final_walls));

    DeadlockResult deadlock = detect_all_deadlocks(map);
    if (!deadlock.has_deadlock || g_bomb_count == 0) return g_path_out;

    uint8_t plan_count = 0; int best_benefit = 0;
    DetonatePlan plans[MAX_BOOMS];
    if (!analyze_bomb_breakthrough(&deadlock, plans, &plan_count, &best_benefit) || plan_count == 0)
        return g_path_out;

    BombExecutionPlan exec_plan;
    plan_bomb_execution_sequence(plans, plan_count, &exec_plan);
    BombExecutionPlan *ep = &exec_plan;
    if (!ep->is_valid || ep->step_count == 0) return g_path_out;

    /* 检查并修复死锁 - 炸弹破局分析入口 */
    memcpy(g_boom_final_walls, g_static_walls, sizeof(uint16_t) * MAP_ROWS);
    g_boom_used_mask = 0;
    for (uint8_t si = 0; si < ep->step_count; si++) {
        BombExecutionStep *st = &ep->steps[si];
        g_boom_used_mask |= (uint8_t)(1 << st->bomb_index);
        for (uint8_t k = 0; k < st->walls_removed_count; k++) {
            Point wp = st->walls_removed[k];
            g_boom_final_walls[wp.x] &= (uint16_t)~(1 << wp.y);
        }
    }

    uint16_t total = 0;
    Point last_added = {0xFF, 0xFF};
    for (uint8_t si = 0; si < ep->step_count; si++) {
        BombExecutionStep *st = &ep->steps[si];
        BombPushPath *pp = &st->push_path;
        if (!pp->path_valid || pp->path_len == 0) continue;
        for (uint16_t i = 0; i < pp->path_len && total < MAX_PATH_LEN; i++) {
            Point cur = pp->path[i];
            if (total > 0 && pos_equal(cur, last_added)) continue;
            g_path_out.x[total] = cur.y; g_path_out.y[total] = cur.x;
            g_path_out.is_push[total] = 0; last_added = cur; total++;
        }
    }
    g_path_out.len = total;
    extract_turn_points(&g_path_out);
    return g_path_out;
}

/**
 * @brief 输出爆炸后的新地图（模式3用）
 *
 * 将当前地图状态复制到输出地图
 *
 * @param out_map 输出 12x16 地图数组
 */
void map_boom_out(uint8_t out_map[MAP_ROWS][MAP_COLS])
{
    memcpy(out_map, g_original_map, sizeof(uint8_t) * MAP_ROWS * MAP_COLS);

    for (uint8_t i = 0; i < MAP_ROWS; i++) {
        for (uint8_t j = 0; j < MAP_COLS; j++) {
            /* 目标位置 -> 目标 */
            if (out_map[i][j] == WALL && !(g_boom_final_walls[i] & (1 << j)))
                out_map[i][j] = FLOOR;

            /* 炸弹位置 -> 炸弹 */
            if (out_map[i][j] == BOOM) {
                for (uint8_t b = 0; b < g_bomb_count; b++) {
                    if (g_initial_bombs[b].x == i && g_initial_bombs[b].y == j) {
                        if (g_boom_used_mask & (1 << b))
                            out_map[i][j] = FLOOR;
                        break;
                    }
                }
            }
        }
    }
}

/**
 * @brief 重置路径规划系统，清理全局状态
 */
void reset_planning_system(void) {
    /* ---- 4.1 A* 推箱子全局缓冲 ---- */
    hash_epoch++;
    if (hash_epoch == 0) {
        memset(hash_table, 0, sizeof(hash_table));
        hash_epoch = 1;
    }
    memset(pq, 0, sizeof(pq)); pq_size = 0;
    memset(g_succ_buf, 0, sizeof(g_succ_buf));
    astar_epoch = 1; memset(astar_node_epoch, 0, sizeof(astar_node_epoch));

    /* ---- 4.2 simple_astar 最小堆 ---- */
    s_heap_size = 0; memset(s_heap, 0, sizeof(s_heap));

    /* ---- 4.3 通用临时缓冲区 ---- */
    memset(g_bfs_walls, 0, sizeof(g_bfs_walls));
    memset(g_bfs_queue, 0, sizeof(g_bfs_queue));
    memset(g_temp_path, 0, sizeof(g_temp_path));
    memset(g_temp_x,   0, sizeof(g_temp_x));
    memset(g_temp_y,   0, sizeof(g_temp_y));
    memset(g_temp_push,0, sizeof(g_temp_push));

    /* ---- 4.4 地图初始状态 ---- */
    memset(g_initial_boxes,  0, sizeof(g_initial_boxes));
    memset(g_initial_targets,0, sizeof(g_initial_targets));
    memset(&g_initial_player,0, sizeof(g_initial_player));
    g_box_count    = 0;
    g_target_count = 0;
    g_bomb_count   = 0;
    memset(g_static_walls,  0, sizeof(g_static_walls));
    memset(g_target_bitmap, 0, sizeof(g_target_bitmap));
    memset(g_initial_bombs, 0, sizeof(g_initial_bombs));
    for (uint8_t i = 0; i < MAX_BOOMS; i++) {
        g_initial_bombs[i].x = 0xFF;  /* 0xFF = 无效/已失效 */
        g_initial_bombs[i].y = 0xFF;
    }

    /* ---- 4.5 模式3全局状态 ---- */
    memset(g_original_map,        0, sizeof(g_original_map));
    memset(g_sim_queue,           0, sizeof(g_sim_queue));
    memset(g_obs_buf,             0, sizeof(g_obs_buf));
    g_bfs_epoch = 1;  memset(g_bfs_visited, 0, sizeof(g_bfs_visited));
    g_dist_epoch = 1; memset(g_dist_epoch_tag, 0, sizeof(g_dist_epoch_tag));
    g_enclosed_region_count = 0;
    memset(g_player_region, 0, sizeof(g_player_region));
    memset(g_problem_points,       0, sizeof(g_problem_points));
    g_problem_count = 0;
    memset(g_saved_problem_points, 0, sizeof(g_saved_problem_points));
    g_saved_problem_count = 0;
    memset(g_bomb_reach_map,       0, sizeof(g_bomb_reach_map));
    memset(g_saved_walls,          0, sizeof(g_saved_walls));
    memset(g_saved_player_region,  0, sizeof(g_saved_player_region));
    memset(g_vcache_hash, 0, sizeof(g_vcache_hash));
    memset(g_vcache_mask, 0, sizeof(g_vcache_mask));
    memset(g_vcache_epoch_tag, 0, sizeof(g_vcache_epoch_tag));
    g_vcache_global_epoch = 0;

    /* ---- 4.6 回溯验证状态 ---- */
    memset(g_solved,   0, sizeof(g_solved));
    g_mode1_strict = false;
    memset(g_remaining, 0, sizeof(g_remaining));
    g_remaining_cnt = 0;
    memset(&g_current_player_pos, 0, sizeof(g_current_player_pos));
    memset(g_current_walls, 0, sizeof(g_current_walls));
    g_total_cost   = 0;
    g_fullpath_len = 0;
    memset(g_fullpath,   0, sizeof(g_fullpath));
    memset(g_solve_order,0, sizeof(g_solve_order));
    g_order_idx = 0;

    /* ---- 4.7 全局输出路径 ---- */
    memset(&g_path_out,       0, sizeof(Path));
    memset(&g_path_start_out, 0, sizeof(Path_Start));
    memset(&g_path_look_out,  0, sizeof(Path_Look));

    /* ---- 4.8 ID模式状态 ---- */
    for (int i = 0; i < MAP_ROWS; i++)
        for (int j = 0; j < MAP_COLS; j++)
            g_dist_map[i][j] = INF;
    memset(g_visit_plan,  0, sizeof(g_visit_plan));
    g_visit_count  = 0;
    g_current_step = 0;
    memset(g_id_pairing,    -1, sizeof(g_id_pairing));
    memset(g_box_id_map,    -1, sizeof(g_box_id_map));
    memset(g_target_id_map, -1, sizeof(g_target_id_map));
    memset(&g_id_based_sol,  0, sizeof(SolutionSequence));

    /* ---- 4.9 模式3最终地图 ---- */
    memset(g_boom_final_walls, 0, sizeof(g_boom_final_walls));
    g_boom_used_mask = 0;
}

/* ===================================================================
 * 第8部分：主函数入口
 * =================================================================== */

/* 初始化地图 */
static uint8_t g_map_test[MAP_ROWS][MAP_COLS] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,0,1,0,0,0,1,0,0,1,0,0,0,0,6,1},
    {1,0,0,0,0,0,0,1,0,1,1,1,1,0,0,1},
    {1,1,0,0,0,0,0,1,0,1,0,0,0,0,0,1},
    {1,0,6,0,0,0,0,1,0,1,0,0,1,0,0,1},
    {1,0,1,1,0,1,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,1,0,0,0,0,0,0,0,1,1,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1},
    {1,0,0,3,0,1,0,0,0,0,2,3,7,0,0,1},
    {1,0,3,0,0,1,1,1,1,0,0,0,1,0,0,1},
    {1,0,0,0,0,1,6,0,0,0,0,0,0,0,0,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

/* 模式3 炸弹测试 */
static uint8_t g_map_test_bomb_complex[MAP_ROWS][MAP_COLS] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,0,1,0,0,0,1,0,0,1,0,0,0,0,6,1},
    {1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,1},
    {1,1,0,1,1,1,0,1,0,1,0,0,0,0,0,1},
    {1,0,6,1,0,1,0,1,0,1,0,0,1,0,0,1},
    {1,0,1,1,0,1,0,0,0,0,0,0,0,0,0,1},
    {1,0,2,1,0,0,0,0,0,0,0,1,1,0,0,1},
    {1,0,7,0,0,0,0,0,0,0,7,0,1,0,0,1},
    {1,0,0,3,0,1,0,0,0,0,0,3,7,0,0,1},
    {1,0,3,0,0,1,1,1,1,1,1,1,1,0,0,1},
    {1,0,0,0,0,1,6,0,0,0,0,0,0,0,0,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

/* ===================================================================
 * 第9部分：单元测试与调试入口
 * =================================================================== */

int main(void) {
    int mode;
    printf("===== 推箱子路径规划系统 =====\n");
    printf("请选择模式:\n");
    printf("  0 - START模式：计算最近元素接近路径\n");
    printf("  1 - 模式1（非ID贪心配对推箱子）\n");
    printf("  2 - 模式2（ID推理配对推箱子）\n");
    printf("  3 - 炸弹破局分析（含推炸弹路径）\n");
    printf("选择: ");
    scanf("%d", &mode);

    if (mode == 0) {
        /* ---- START模式 ---- */
        double start_time = clock();
        Path_Start result = path_start_calculation(g_map_test);
        double end_time = clock();
        printf("程序运行时间: %f ms\n", (end_time - start_time) / CLOCKS_PER_SEC * 1000.0);
        printf("提取的路径点 (path_start):\n");
        for (int i = 0; i < result.len; i++) {
            printf("(%d,%d) ", result.x[i], result.y[i]);
        }
        printf("\n长度: %d, 角度: %d, 类型: %d\n", result.len, result.angle, result.type);
    } else if (mode == 1) {
        /* ---- 科目一（非ID） ---- */
        double start_time = clock();
        Path path_car = path_calculation(g_map_test);
        double end_time = clock();
        printf("程序运行时间: %f ms\n", (end_time - start_time) / CLOCKS_PER_SEC * 1000.0);
        for (int i = 0; i <= path_car.len; i++) {
            printf("(%d,%d) ", path_car.x[i], path_car.y[i]);
        }
        printf("\n路径总长: %d\n", path_car.len);
    } else if (mode == 2) {
        /* ---- 科目二（ID） ---- */
        double start_time = clock();
        Path_Look path_look = path_look_calculation(g_map_test);
        double end_time = clock();
        printf("程序运行时间: %f ms\n", (end_time - start_time) / CLOCKS_PER_SEC * 1000.0);
        printf("(x,y),angel,is_look,type,%d\n",path_look.len);
        for (int i = 0; i < path_look.len; i++) {
            printf("(%d,%d),%d, %d,%d,%d\n",
                   path_look.x[i], path_look.y[i],i,
                   path_look.angle[i], path_look.is_look[i], path_look.type[i]);
        }
        int step = 0,id = -1;
        Path path_car = {0};
        for (int i = 0; i < path_look.len; i++) {
            if (path_look.is_look[i] == 1) {
                printf("%d:",path_look.type[i]);
                scanf("%d",&id);
                id_input(id); 
            }
            step++;
        }  
        if (step == path_look.len) {
            double st = clock();
            path_car = path_id_calculation();
            double et = clock();
            printf("程序运行时间: %f ms\n", (et - st) / CLOCKS_PER_SEC * 1000.0);
        }
        for (int i = 0; i <= path_car.len; i++) {
            printf("(%d,%d) ",path_car.x[i],path_car.y[i]);
        }
        printf("\n路径总长: %d\n", path_car.len);
    } else if (mode == 3) {
        /* ---- 模式3：炸弹破局分析 ---- */
        double start_time = clock();
        Path path_bomb = path_boom_calculation(g_map_test_bomb_complex);
        map_boom_out(g_map_test_bomb_complex);
        double end_time = clock();
        printf("程序运行时间: %.2f ms\n", (end_time - start_time) / CLOCKS_PER_SEC * 1000.0);

        if (path_bomb.len > 0) {
            for (int i = 0; i <= path_bomb.len; i++) {
                printf("(%d,%d) ", path_bomb.x[i], path_bomb.y[i]);
            }
            printf("\n路径长度: %d\n", path_bomb.len);
            for (int i = 0;i < 12;i++){
                for (int j = 0;j < 16;j++){
                    printf("%d ",g_map_test_bomb_complex[i][j]);
                }
                printf("\n");
            }
        } else {
            printf("无解\n");
        }
    }

    getchar();
    getchar();
    return 0;
}
