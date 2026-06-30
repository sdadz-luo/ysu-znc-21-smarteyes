#ifndef _PATH_PLANNING_H_
#define _PATH_PLANNING_H_

/* ===================================================================
 * 第1部分：宏定义常量
 * =================================================================== */

/* ---- 地图基本尺寸 ---- */
#define MAP_ROWS             12         /* 地图行数 */
#define MAP_COLS             16         /* 地图列数 */
#define MAX_BOXES            5          /* 最大箱子数量 */
#define MAX_IDS              10         /* 最大ID编号数 */
#define MAX_VISIT_STEPS      (2 * MAX_BOXES) /* 最大访问步骤数 */
#define MAX_PATH_LEN         500        /* 最大路径长度 */
#define INF                  65535U     /* 无穷大值 */
#define DIR_COUNT            4          /* 方向数量 */

/* 地图元素标识 */
#define WALL                 1          /* 墙 */
#define FLOOR                0          /* 地面 */
#define PLAYER               2          /* 玩家 */
#define BOX                  3          /* 箱子 */
#define TARGET               6          /* 目标 */
#define BOOM                 7          /* 炸弹 */

/* ---- A* 相关常量 ---- */
#define MAX_OPENSET          50000      /* 哈希表容量（推箱A*） */
#define MAX_PQ_SIZE          10000      /* 优先队列容量 */
#define MAX_SEARCH_CNT       50000      /* A*最大搜索步数 */
#define SINGLE_STEP_MAX_PATH 500        /* 单步A*最大路径长度 */
#define TURN_WEIGHT          4          /* 转向惩罚代价 */

#define ROTATION_PENALTY     4          /* 非0度角观察的旋转惩罚（0度=站在元素左侧看向右侧） */
#define DIR_INVALID          255        /* 无效方向标记 */
#define BOMB_INVALID         0xFF       /* 炸弹已消失/无效标记 */

/* ---- 模式3（炸弹破局）容量上限 ---- */
#define MAX_BOOMS            4          /* 最大炸弹数量 */
#define MAX_BREAK_WALLS      100        /* 最大可炸墙数量 */
#define MAX_DETONATE_POINTS  50         /* 最大爆炸点候选数量 */
#define MAX_PROBLEM_POINTS   10         /* 最大死锁问题点数量 */
#define MAX_PLANS_PER_BOOM   50         /* 每炸弹最大方案数 */
#define MAX_SIM_QUEUE        16384      /* 推箱模拟队列容量（压缩版） */
#define MAX_ENC_WALLS        50         /* 每个ENC最大围住墙数 */

/* ---- 死锁待解决点类型标记 ---- */
#define PROBLEM_ENCLOSED            0x01    /* 封闭区域：影响域∩玩家区=? */
#define PROBLEM_NEED_SIM            0x04    /* 待验证：连通性OK，需推箱BFS确认 */
#define PROBLEM_TARGET_UNREACHABLE  0x08    /* 目标不可达：无箱子能推到该目标 */

/* 压缩状态编码：px:4 py:4 bx:4 by:4 = 16bit */
#define ENCODE_STATE(px, py, bx, by) \
    ((uint16_t)(((px)&0xF)<<12 | ((py)&0xF)<<8 | ((bx)&0xF)<<4 | ((by)&0xF)))
#define DECODE_PX(s) ((uint8_t)(((s)>>12) & 0xF))
#define DECODE_PY(s) ((uint8_t)(((s)>>8)  & 0xF))
#define DECODE_BX(s) ((uint8_t)(((s)>>4)  & 0xF))
#define DECODE_BY(s) ((uint8_t)((s)       & 0xF))

/* BFS visited 索引（推箱模拟状态去重） */
#define BFS_VISITED_INDEX(px, py, bx, by) \
    ((uint32_t)(((uint16_t)(px) * MAP_COLS + (py)) * (MAP_ROWS * MAP_COLS) + ((uint16_t)(bx) * MAP_COLS + (by))))

/* 验证缓存容量 */
#define VCACHE_SIZE         2048
#define BOOM_TOP_K          12         /* 炸弹方案Top-K保留数 */

/* 统一移动方向：上、下、左、右 */
static const int8_t DIRS[DIR_COUNT][2] = {{-1,0},{1,0},{0,-1},{0,1}};

/* ===================================================================
 * 第2部分：数据结构定义
 * =================================================================== */

/* ---------- 2.1 通用基础结构 ---------- */

/* 二维坐标 */
typedef struct { 
    uint8_t x;  /* 行 */
    uint8_t y;  /* 列 */
} Point;

/* A* 路径节点（simple_astar 使用） */
typedef struct {
    Point pos;          /* 位置 */
    int8_t last_dir;    /* 上一步移动方向 (-1无效) */
    int g_cost;         /* 实际代价 */
    int f_cost;         /* 估计总代价 f = g + h */
    int parent_idx;     /* 父节点索引 */
    bool closed;        /* 是否已闭合 */
} PathNode;

/* simple_astar 最小堆节点 */
typedef struct {
    int idx;            /* 对应 path_nodes 中的索引 */
} SimpleHeapNode;

/* 箱子-目标配对 */
typedef struct {
    uint8_t box_idx;        /* 箱子索引 */
    uint8_t target_idx;     /* 目标点索引 */
    Point box_pos;          /* 箱子位置 */
    Point target_pos;       /* 目标点位置 */
} BoxTargetPair;

/* 完整解决方案（模式1/2共用） */
typedef struct {
    BoxTargetPair pairs[MAX_BOXES];     /* 配对列表 */
    uint8_t count;                      /* 配对数量 */
    int total_cost;                     /* 总代价 */
    bool is_valid;                      /* 是否有效 */
    Point full_path[MAX_PATH_LEN];      /* 完整路径 */
    uint16_t full_path_len;             /* 路径长度 */
} SolutionSequence;

/* ---------- 2.2 推箱 A* 相关结构 ---------- */

/* 推箱 A* 状态 */
typedef struct {
    Point player;                       /* 玩家位置 */
    Point box;                          /* 箱子位置 */
    Point target;                       /* 目标位置 */
    uint16_t wall_bitmap[MAP_ROWS];     /* 墙位图 */
    uint8_t last_dir;                   /* 上一步方向 */
} State;

/* 哈希表项 */
typedef struct { 
    State state;        /* 状态 */
    uint16_t g;         /* 实际代价 */
    uint16_t came_from; /* 前驱状态哈希索引（UINT16_MAX=无前驱） */
    uint8_t used;       /* 是否已使用 */
} HashEntry;

/* 优先队列节点 */
typedef struct { 
    uint16_t f;         /* 估计总代价 */
    int32_t state_idx;  /* 哈希状态索引 */
} PQNode;

/* 推箱 A* 结果 */
typedef struct {
    int32_t cost;                           /* 总代价 */
    Point final_player_pos;                 /* 最终玩家位置 */
    bool success;                           /* 是否成功 */
    Point path_points[SINGLE_STEP_MAX_PATH];/* 玩家路径 */
    uint16_t path_len;                      /* 路径长度 */
} AStarResult;

/* ---------- 2.3 START 模式结构 ---------- */

/* 接近结果（START 模式） */
typedef struct {
    Point path[MAX_PATH_LEN];       /* 路径 */
    uint16_t path_len;              /* 路径长度 */
    Point elem_pos;                 /* 元素位置 */
    uint8_t elem_type;              /* 元素类型（BOX/TARGET） */
    Point approach_pos;             /* 接近位置 */
    int16_t angle;                  /* 接近角度 */
    bool success;                   /* 是否成功 */
} ApproachResult;

/* 输出路径（START模式） */
typedef struct {
    uint8_t x[MAX_PATH_LEN];        /* 路径 x 坐标（列） */
    uint8_t y[MAX_PATH_LEN];        /* 路径 y 坐标（行） */
    uint16_t len;                   /* 路径长度 */
    int16_t angle;                  /* 接近角度 */
    uint8_t type;                   /* 元素类型 */
} Path_Start;

/* ---------- 2.4 模式1（非ID推箱）输出结构 ---------- */

typedef struct {
    uint8_t x[MAX_PATH_LEN];        /* 路径 x 坐标（列） */
    uint8_t y[MAX_PATH_LEN];        /* 路径 y 坐标（行） */
    uint16_t len;                   /* 路径长度 */
    uint8_t is_push[MAX_PATH_LEN];  /* 是否为推动动作 */
} Path;

/* ---------- 2.5 模式2（ID推箱）结构 ---------- */

/* 访问步骤 */
typedef struct {
    Point pos;                      /* 访问位置 */
    uint8_t type;                   /* 元素类型（BOX/TARGET） */
    uint8_t original_idx;           /* 原始索引 */
    uint8_t direction;              /* 方向 */
    int16_t angle;                  /* 角度 */
    Point path[MAX_PATH_LEN];       /* 路径 */
    uint16_t path_len;              /* 路径长度 */
} VisitStep;

/* 观察路径输出 */
typedef struct {
    uint8_t x[MAX_PATH_LEN];        /* 路径 x 坐标（列） */
    uint8_t y[MAX_PATH_LEN];        /* 路径 y 坐标（行） */
    uint16_t len;                   /* 路径长度 */
    int16_t angle[MAX_PATH_LEN];    /* 各点角度 */
    uint8_t type[MAX_PATH_LEN];     /* 各点类型 */
    bool is_look[MAX_PATH_LEN];     /* 是否需要扫码 */
} Path_Look;

/* ---------- 2.6 模式3（炸弹破局）数据结构 ---------- */

/* 压缩的推箱模拟状态（原4字节→2字节） */
typedef uint16_t SimState;

/* 死锁检测结果 */
typedef struct {
    uint8_t enclosed_indices[MAX_BOXES * 2]; uint8_t enclosed_count;
    bool has_deadlock;
} DeadlockResult;

/* 死锁待解决点 */
typedef struct {
    uint8_t type; 
    uint8_t box_indices[MAX_BOXES]; uint8_t box_count;
    uint8_t target_indices[MAX_BOXES]; uint8_t target_count;
    uint8_t wall_count;                            /* ENC: 围住墙数量 */
    Point enc_walls[MAX_ENC_WALLS];                /* ENC: 围住墙坐标 */
    uint16_t influence_mask[MAP_ROWS]; /* 影响域掩码 */
} DeadlockProblemPoint;

/* 可炸墙 */
typedef struct {
    Point wall_pos; int benefit_score;
} BreakableWall;

/* 爆炸方案 */
typedef struct {
    Point detonate_pos; Point walls_covered[9]; uint8_t wall_count;
    uint8_t bomb_index; Point bomb_initial_pos;
    uint16_t bomb_push_distance; int total_benefit;
    uint16_t resolved_mask;
} DetonatePlan;

/* 炸弹推送路径（纯玩家路径） */
typedef struct {
    Point path[MAX_PATH_LEN];           /* 玩家路径点序列 */
    uint16_t path_len;                  /* 路径长度（步数） */
    bool     path_valid;                /* 路径是否有效 */
} BombPushPath;

/* 一次炸弹执行步骤 */
typedef struct {
    uint8_t  step_order;                /* 执行序号（0,1,2...） */
    uint8_t  bomb_index;                /* 使用的炸弹编号 */
    Point    bomb_from;                 /* 炸弹起始位置 */
    Point    detonate_at;               /* 爆炸点=炸弹目标（墙位） */
    Point    walls_removed[9];          /* 该步摧毁的墙 */
    uint8_t  walls_removed_count;       /* 摧毁墙数量 */
    BombPushPath push_path;             /* 玩家推送路径 */
    Point    player_before;             /* 该步开始前玩家位置 */
    Point    player_after;              /* 该步结束后玩家位置 */
} BombExecutionStep;

/* 完整炸弹执行序列 */
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

/* --- 3.2 辅助工具 --- */
static inline bool pos_equal(Point a, Point b);
static inline int get_smooth_idx(int x, int y, int dir);
static inline uint16_t manhattan_distance(Point a, Point b);
static bool is_wall_bit(const uint16_t walls[MAP_ROWS], Point p);
static bool is_corner_deadlock(Point box, const uint16_t walls[MAP_ROWS]);
static bool is_soft_corner_deadlock(Point box, const uint16_t walls[MAP_ROWS],
    const bool solved[MAX_BOXES], uint8_t *out_blocker_idx);
static bool is_breakable_wall_on(Point pos, const uint16_t walls[MAP_ROWS]);
static bool is_valid_detonation_point_on(Point pos, const uint16_t walls[MAP_ROWS]);
static uint16_t estimate_bomb_push_distance_on(Point bomb_pos, Point detonate_pos,
    Point player_pos, const uint16_t walls[MAP_ROWS], const bool active_bombs[MAX_BOOMS]);
static int16_t compute_approach_angle(int8_t dx, int8_t dy);

/* --- 3.3 简单 A* 寻路（无推箱） --- */
static uint16_t simple_astar(Point start, Point end, uint16_t walls[MAP_ROWS], Point* out_path);
static inline void s_heap_push(int idx);
static inline int s_heap_pop(void);

/* --- 3.4 BFS 距离计算 --- */
static void bfs_compute_distances(Point start, const uint16_t walls[MAP_ROWS]);
static void bfs_compute_reachability(Point start, const uint16_t obstacles[MAP_ROWS]);

/* --- 3.5 推箱 A* 核心 --- */
static uint32_t hash_state(const State *s);
static bool state_equal(const State *a, const State *b);
static int32_t hash_lookup_insert(const State *s, uint16_t g, int32_t from, uint8_t insert);
static void pq_push(PQNode node);
static PQNode pq_pop(void);
static uint8_t heuristic(const State *s);
static bool is_goal(const State *s);
static void get_successors(const State *cur, State *res, uint8_t *cnt);
static AStarResult solve_single_box_a_star(Point player, Point box, Point target,
                                           uint16_t dynamic_walls[MAP_ROWS]);
/* --- 3.6 模式1：贪心配对与回溯验证 --- */
static void generate_greedy_pairing(SolutionSequence* sol);
static bool backtrack_validate(SolutionSequence* sol);
static void validate_solution(SolutionSequence* sol);

/* --- 3.7 START模式：最近元素接近与路径提取 --- */
static ApproachResult* find_nearest_approach(uint8_t map[MAP_ROWS][MAP_COLS]);
static void extract_start_turn_points(ApproachResult* res);
static void extract_turn_points(Path* p);

/* --- 3.8 模式2：ID学习与推理 --- */
static void id_learning(uint8_t map[MAP_ROWS][MAP_COLS]);
static void id_record(int id);
static bool next_permutation(int *arr, int n);
static void id_inference(void);
static void extract_look_turn_points(VisitStep* plan);
static SolutionSequence* build_solution_from_id_pairing(void);
static Path path_id_calculate(SolutionSequence* sol);

/* --- 3.9 模式3：炸弹破局分析 --- */

typedef struct { int cost; uint8_t idx; } ScoredPlan;

static uint32_t hash_walls(const uint16_t walls[MAP_ROWS]);
static void compute_player_region_with_walls(const uint16_t walls[MAP_ROWS], bool ignore_bombs);
static void compute_box_influence(Point box, const uint16_t walls[MAP_ROWS],
                                      uint16_t influence_mask[MAP_ROWS]);
static bool influence_contains_target(const uint16_t influence[MAP_ROWS]);
static bool box_can_reach_any_target(Point box, const uint16_t walls[MAP_ROWS],
                                     const Point targets[], uint8_t target_count);
static bool simulate_box_to_target(uint8_t box_idx, Point target,
                                    const uint16_t walls[MAP_ROWS],
                                    Point start_player, Point start_box);
static void detect_and_generate_problems(void);
static DeadlockResult detect_all_deadlocks(uint8_t map[MAP_ROWS][MAP_COLS]);
static uint16_t check_problems_resolved_incremental(const uint16_t modified_walls[MAP_ROWS], Point player_pos);
static uint16_t check_problems_resolved_cached(const uint16_t modified_walls[MAP_ROWS], Point player_pos);
static void compute_one_bomb_reachability(uint8_t bi, Point bomb_pos,
    const uint16_t walls[MAP_ROWS], const bool active[],
    uint16_t out_reach[MAP_ROWS]);
static void compute_bomb_reachability(const uint16_t walls[MAP_ROWS],
    const bool active_bombs[MAX_BOOMS],
    uint16_t out_reach_map[MAX_BOOMS][MAP_ROWS]);
static void find_detonation_points_for_wall_on(Point target_wall, const uint16_t walls[MAP_ROWS],
    Point out_points[], uint8_t *out_count);
static void find_walls_for_enclosed_on(const uint16_t region_mask[MAP_ROWS],
    const uint16_t walls[MAP_ROWS],
    BreakableWall out_walls[], uint8_t *out_count);
static void find_walls_for_separated_on(const uint16_t influence[MAP_ROWS],
    const uint16_t walls[MAP_ROWS],
    BreakableWall out_walls[], uint8_t *out_count);
static void generate_plans_for_walls_on(const BreakableWall breakable_walls[], uint8_t wall_count,
    const uint16_t walls[MAP_ROWS],
    const uint16_t bomb_reach_map[MAX_BOOMS][MAP_ROWS],
    Point player_pos, const bool active_bombs[MAX_BOOMS],
    DetonatePlan out_plans[], uint8_t *out_plan_count);
static bool verify_multi_bomb_sequence(DetonatePlan plans[], uint8_t count,
    const uint16_t walls[MAP_ROWS], Point player_pos,
    uint8_t out_order[]);
static void try_multi_plans_recursive(
    uint8_t depth, uint8_t max_depth,
    const uint8_t bomb_idx[], const uint8_t bl[],
    DetonatePlan pb[][MAX_PLANS_PER_BOOM], const uint8_t fi[],
    uint8_t sel_plans[], int accum_benefit,
    const uint16_t all_mask, int *best_benefit,
    DetonatePlan result_plans[], uint8_t *result_count, uint8_t *found);
static bool search_multi_combo(
    uint8_t target_m, const uint8_t N, const uint8_t bl[],
    DetonatePlan pb[][MAX_PLANS_PER_BOOM], const uint8_t fi[],
    const uint16_t all_mask, int *best_benefit,
    DetonatePlan result_plans[], uint8_t *result_count);
static void score_bomb_candidates(uint8_t cand_count, uint16_t resolved,
    const uint16_t cur_walls[MAP_ROWS],
    const uint16_t cur_reach[MAX_BOOMS][MAP_ROWS],
    const bool active_bombs[MAX_BOOMS],
    Point cur_player,
    ScoredPlan scored[], uint8_t *scored_count);
static bool iterative_bomb_breakthrough(const DeadlockResult *deadlock,
    DetonatePlan out_plans[], uint8_t *out_plan_count, int *out_best_benefit);
static void compute_player_bomb_walls(uint16_t bomb_walls[MAP_ROWS]);
static uint8_t find_enclosing_walls(const uint16_t influence[MAP_ROWS],
    const uint16_t bomb_walls[MAP_ROWS], Point out_walls[], uint8_t max_walls);
static inline bool can_explosion_cover_wall(Point detonate_pos, Point target_wall);
/* 为指定问题收集可炸墙（追加到walls列表，去重） */
static void collect_walls_for_problem(uint8_t problem_index, const uint16_t cur_walls[MAP_ROWS],
                                       BreakableWall walls[], uint8_t *wall_count,
                                       uint8_t max_walls, const bool active_bombs[MAX_BOOMS]);
/* 检查炸弹是否有至少一个方向可被推动 */
static bool can_push_bomb(uint8_t bomb_index, Point bomb_pos,
                           const uint16_t walls[MAP_ROWS], const bool active[]);

static void sort_plans_by_benefit(DetonatePlan plans[], uint8_t count);
static bool search_multi_bomb_combination(DetonatePlan all_plans[], uint8_t plan_count,
                                           DetonatePlan result_plans[], uint8_t *result_count,
                                           int *out_total_benefit);
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

/* --- 3.10 顶层外部接口 --- */
Path_Start path_start_calculation(uint8_t map[MAP_ROWS][MAP_COLS]);
Path path_calculation(uint8_t box_x[MAP_ROWS][MAP_COLS]);
Path_Look path_look_calculation(uint8_t box_x[MAP_ROWS][MAP_COLS]);
void id_input(int id);
Path path_id_calculation(void);
Path path_boom_calculation(uint8_t map[MAP_ROWS][MAP_COLS]);
void map_boom_out(uint8_t out_map[MAP_ROWS][MAP_COLS]);
void reset_planning_system(void);

#endif