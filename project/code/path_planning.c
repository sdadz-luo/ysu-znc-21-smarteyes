#include "zf_common_headfile.h"
#include "path_planning.h"

/* ===================================================================
 * 第4部分：全局变量定义
 * =================================================================== */

/* ---------- 4.1 A* 搜索全局缓冲区 ---------- */
__attribute__((section(".bss.SDRAM_CACHE")))static HashEntry hash_table[MAX_OPENSET];           /* A*哈希表 */
__attribute__((section(".bss.$DTCM"))) static PQNode pq[MAX_PQ_SIZE];                      /* A*优先队列 */
static uint32_t pq_size = 0;                        /* 优先队列大小 */
static State g_succ_buf[8];                         /* 后继状态缓冲区 */
static PathNode path_nodes[MAP_ROWS * MAP_COLS * 5];/* A*路径节点池 */

/* ---------- 4.2 simple_astar 最小堆 ---------- */
static SimpleHeapNode s_heap[MAP_ROWS * MAP_COLS * 5];  /* 最小堆 */
static uint32_t s_heap_size = 0;                         /* 堆大小 */

/* ---------- 4.3 共享临时缓冲区 ---------- */
static uint16_t g_bfs_walls[MAP_ROWS];              /* BFS墙位图 */
static uint16_t g_sim_walls[MAP_ROWS];              /* 方案模拟墙位图（复用） */
static Point g_bfs_queue[MAP_ROWS * MAP_COLS];      /* BFS/泛洪队列 */
static Point g_temp_path[MAX_PATH_LEN];              /* 临时路径 */
static uint8_t g_temp_x[MAX_PATH_LEN];               /* 临时x坐标 */
static uint8_t g_temp_y[MAX_PATH_LEN];               /* 临时y坐标 */
static uint8_t g_temp_push[MAX_PATH_LEN];            /* 临时推动标记 */

/* ---------- 4.4 地图初始状态 ---------- */
static Point g_initial_boxes[MAX_BOXES];             /* 初始箱子位置 */
static Point g_initial_targets[MAX_BOXES];           /* 初始目标位置 */
static Point g_initial_player;                       /* 初始玩家位置 */
static uint8_t g_box_count = 0;                      /* 箱子数量 */
static uint8_t g_target_count = 0;                   /* 目标数量 */
static uint16_t g_static_walls[MAP_ROWS];            /* 静态墙位图 */
static uint16_t g_target_bitmap[MAP_ROWS];          /* 目标点位图（O(1) 查询） */
static Point g_initial_bombs[MAX_BOOMS];             /* 初始炸弹位置 */
static uint8_t g_bomb_count = 0;                     /* 炸弹数量 */

/* ---------- 4.5 模式3（炸弹破局）全局缓冲区 ---------- */
static uint8_t g_original_map[MAP_ROWS][MAP_COLS];   /* 原始地图副本（模式3用） */
static SimState g_sim_queue[MAX_SIM_QUEUE];          /* 推箱模拟队列（压缩版） */
__attribute__((section(".bss.$DTCM"))) static uint16_t g_obs_buf[MAP_ROWS];                 /* 障碍物缓冲区 */

/* 推箱模拟BFS访问标记（epoch技术避免memset） */
static uint8_t g_bfs_epoch = 1;
__attribute__((section(".bss.$DTCM"))) static uint8_t g_bfs_visited[MAP_ROWS * MAP_COLS * MAP_ROWS * MAP_COLS];
static uint8_t g_dist_epoch = 1;
__attribute__((section(".bss.$DTCM"))) static uint8_t g_dist_epoch_tag[MAP_ROWS][MAP_COLS];

static uint8_t g_vis1[MAP_ROWS][MAP_COLS];           /* 临时访问标记1 */

static uint8_t g_player_region[MAP_ROWS][MAP_COLS];   /* 玩家可达区域 */
static uint8_t g_vis1_epoch = 1;
static uint8_t g_player_region_epoch = 1;
__attribute__((section(".data.$DTCM"))) static uint8_t g_hash_epoch = 1;                       /* 推箱A*哈希表epoch */

/* 死锁待解决点 */
static DeadlockProblemPoint g_problem_points[MAX_PROBLEM_POINTS];
static uint8_t g_problem_count = 0;
static DeadlockProblemPoint g_saved_problem_points[MAX_PROBLEM_POINTS];
static uint8_t g_saved_problem_count = 0;

/* 炸弹可达性缓存：每个炸弹预计算完整可达区域（位图） */
static uint16_t g_bomb_reach_map[MAX_BOOMS][MAP_ROWS];

/* 增量验证时临时保存 */
static uint16_t g_saved_walls[MAP_ROWS];
static uint8_t g_saved_player_region[MAP_ROWS][MAP_COLS];

/* 验证缓存：hash(walls)→resolved_mask */
static uint32_t g_vcache_hash[VCACHE_SIZE];
static uint16_t g_vcache_mask[VCACHE_SIZE];

/* ── 炸弹模式复用全局缓冲区（减少栈压）── */
static DetonatePlan g_bomb_pb[MAX_BOOMS][MAX_PLANS_PER_BOOM];     /* search_multi_bomb_combination */
static BombExecutionStep g_bomb_t_s[MAX_BOOMS];                   /* plan_bomb_execution_sequence */
static BombExecutionStep g_bomb_best_s[MAX_BOOMS];                /* plan_bomb_execution_sequence */
static DetonatePlan g_bomb_candidates[MAX_DETONATE_POINTS];       /* iterative_bomb_breakthrough */
static DetonatePlan g_bomb_all_plans[MAX_DETONATE_POINTS];        /* fallback */
static BreakableWall g_bomb_walls[MAX_BREAK_WALLS];               /* 通用墙收集 */

/* 阶段二：炸弹推送路径 & A* 结果复用 */
static BombPushPath g_bomb_ppath;                                 /* try_execute_step */
static AStarResult g_bomb_ar;                                     /* compute_one_bomb_push / verify */

/* 阶段三：回溯验证 & 地图保存复用 */
static AStarResult g_backtrack_ar;                                /* backtrack_validate (递归) */
static uint8_t g_bomb_saved_map[MAP_ROWS][MAP_COLS];              /* iterative_bomb_breakthrough 地图回退 */
static uint8_t g_bomb_visited[MAP_ROWS][MAP_COLS];                /* compute_player_bomb_walls */

/* 优化：UNR问题"箱子→目标"可达性缓存（墙只减不增，成功过的箱子不必重算A*） */
static int8_t g_unr_reach_cache[MAX_PROBLEM_POINTS];             /* -1=未探明, >=0=该箱子可达目标 */

/* 通用临时缓冲区（栈变量迁移→全局，减少栈占用） */
static uint16_t g_tmp_walls1[MAP_ROWS];         /* detect_and_generate_problems / check_resolved */
static uint16_t g_tmp_walls2[MAP_ROWS];         /* detect_and_generate_problems / check_resolved */
static uint16_t g_tmp_infl[MAP_ROWS];           /* 通用临时影响域 */
static uint16_t g_tmp_walls_cur[MAP_ROWS];      /* iterative_bomb_breakthrough cur_walls */
static uint16_t g_tmp_obs[MAP_ROWS];            /* compute_one_bomb_push obs */

/* ---------- 4.6 推箱验证全局状态 ---------- */
static bool g_mode1_strict = false;                  /* 模式1严格模式：靶位阻挡箱子 */
static bool g_solved[MAX_BOXES];                     /* 箱子是否已解决 */
static uint8_t g_remaining[MAX_BOXES];               /* 剩余箱子索引 */
static uint8_t g_remaining_cnt;                      /* 剩余箱子数量 */
static Point g_current_player_pos;                   /* 当前玩家位置 */
static uint16_t g_current_walls[MAP_ROWS];           /* 当前墙位图 */
static int g_total_cost;                             /* 总代价 */
static uint16_t g_fullpath_len;                      /* 完整路径长度 */
static Point g_fullpath[MAX_PATH_LEN];               /* 完整路径 */
static uint8_t g_solve_order[MAX_BOXES];             /* 解决顺序 */
static uint8_t g_order_idx;                          /* 顺序索引 */

/* ---------- 4.7 全局输出路径 ---------- */
static Path g_path_out = {0};                        /* 模式1输出 */
static Path_Start g_path_start_out = {0};            /* START模式输出 */
static Path_Look g_path_look_out = {0};              /* ID模式输出 */

/* ---------- 4.8 ID模式状态 ---------- */
__attribute__((section(".bss.$DTCM"))) static uint16_t g_dist_map[MAP_ROWS][MAP_COLS];      /* BFS距离图 */
static VisitStep g_visit_plan[MAX_VISIT_STEPS];       /* 访问计划 */
static uint8_t g_visit_count = 0;                    /* 访问计数 */
static uint8_t g_current_step = 0;                   /* 当前步骤 */
static int8_t g_id_pairing[MAX_IDS];                  /* ID配对表 */
static int8_t g_box_id_map[MAX_BOXES];                /* 箱子ID映射 */
static int8_t g_target_id_map[MAX_BOXES];             /* 目标ID映射 */
static SolutionSequence g_id_based_sol;               /* ID方案 */
static int8_t g_orig_box_id_map[MAX_BOXES];
static int8_t g_orig_target_id_map[MAX_BOXES];

/* ---------- 4.9 模式3最终地图 ---------- */
static uint16_t g_boom_final_walls[MAP_ROWS];        /* 引爆后最终墙位图 */
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
 * @brief 从最小堆中弹出f_cost最小的节点索引
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

/* ---------- 5.2 基本坐标与几何工具 ---------- */

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
static inline __attribute__((section("ITCM_NonCacheable"))) uint16_t manhattan_distance(Point a, Point b) {
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
 * @brief 判断箱子是否处于角落死锁（两个垂直方向都有墙）
 */
static __attribute__((section("ITCM_NonCacheable"))) bool is_corner_deadlock(Point box, const uint16_t walls[MAP_ROWS]) {
    bool up    = is_wall_bit(walls, (Point){box.x - 1, box.y});
    bool down  = is_wall_bit(walls, (Point){box.x + 1, box.y});
    bool left  = is_wall_bit(walls, (Point){box.x, box.y - 1});
    bool right = is_wall_bit(walls, (Point){box.x, box.y + 1});
    return (up && left) || (up && right) || (down && left) || (down && right);
}

/**
 * @brief 检测角落死锁是否为"软死锁"——被未推动箱子堵住（可先推挡路箱解除）
 *
 * 硬死锁 = 堵路来源均为静态墙 → 永久放弃
 * 软死锁 = 某方向堵路来源是另一个未推动的箱子 → 推走挡路箱后本箱不再死锁
 *
 * @param box        待检测箱子位置
 * @param walls      当前障碍物位图（含其他箱子的位置）
 * @param solved     哪些箱子已解决
 * @param out_blocker_idx  输出：挡路箱子的 box_idx
 * @return true=软死锁（out_blocker_idx 有效），false=硬死锁或无死锁
 */
static __attribute__((section("ITCM_NonCacheable"))) bool is_soft_corner_deadlock(Point box, const uint16_t walls[MAP_ROWS],
    const bool solved[MAX_BOXES], uint8_t *out_blocker_idx) {
    bool up    = is_wall_bit(walls, (Point){box.x - 1, box.y});
    bool down  = is_wall_bit(walls, (Point){box.x + 1, box.y});
    bool left  = is_wall_bit(walls, (Point){box.x, box.y - 1});
    bool right = is_wall_bit(walls, (Point){box.x, box.y + 1});

    if (!((up && left) || (up && right) || (down && left) || (down && right)))
        return false;

    Point neighbors[4] = {
        {box.x - 1, box.y}, {box.x + 1, box.y},
        {box.x, box.y - 1}, {box.x, box.y + 1}
    };
    bool blocked[4] = {up, down, left, right};

    for (int d = 0; d < 4; d++) {
        if (!blocked[d]) continue;
        uint8_t nx = neighbors[d].x, ny = neighbors[d].y;
        if (nx >= MAP_ROWS || ny >= MAP_COLS) continue;
        /* 静态墙 → 跳过，检查是否为未推箱子 */
        if (g_static_walls[nx] & (1 << ny)) continue;
        for (uint8_t bi = 0; bi < g_box_count; bi++) {
            if (solved[bi]) continue;
            if (g_initial_boxes[bi].x == nx && g_initial_boxes[bi].y == ny) {
                *out_blocker_idx = bi;
                return true;
            }
        }
    }
    return false;
}

/* ---------- 5.4 哈希表操作（推箱A*状态去重） ---------- */

/**
 * @brief 计算状态的哈希值（djb2算法）
 */
static uint32_t hash_state(const State *s) {
    uint32_t h = 5381;
    h = ((h << 5) + h) ^ (uint32_t)s->player.x;
    h = ((h << 5) + h) ^ (uint32_t)s->player.y;
    h = ((h << 5) + h) ^ (uint32_t)s->box.x;
    h = ((h << 5) + h) ^ (uint32_t)s->box.y;
    h = ((h << 5) + h) ^ (uint32_t)s->last_dir;
    return h & HASH_MASK;
}

/**
 * @brief 判断两个状态是否相等
 */
static bool state_equal(const State *a, const State *b) {
    if (!pos_equal(a->player, b->player)) return false;
    if (!pos_equal(a->box, b->box)) return false;
    if (a->last_dir != b->last_dir) return false;
    for (int i = 0; i < MAP_ROWS; i++)
        if (a->wall_bitmap[i] != b->wall_bitmap[i]) return false;
    return true;
}

/**
 * @brief 哈希表查找/插入（线性探测解决冲突）
 * @return >=0 索引 / -1 未找到 / -2 表满
 */
static __attribute__((section("ITCM_NonCacheable"))) int32_t hash_lookup_insert(const State *s, uint16_t g, int32_t from, uint8_t insert) {
    uint32_t h = hash_state(s);
    uint32_t idx = h;
    uint32_t probe_cnt = 0;
    while (hash_table[idx].used == g_hash_epoch && probe_cnt < MAX_OPENSET) {
        probe_cnt++;
        if (state_equal(&hash_table[idx].state, s)) return (int32_t)idx;
        idx = (idx + HASH_STEP) & HASH_MASK;
        if (idx == h) return -2;
    }
    if (!insert) return -1;
    if (hash_table[idx].used != g_hash_epoch) {
        hash_table[idx].used = g_hash_epoch;
        hash_table[idx].state = *s;
        hash_table[idx].g = g;
        hash_table[idx].came_from = (uint16_t)from;
        return (int32_t)idx;
    }
    return -2;
}

/* ---------- 5.5 优先队列操作（推箱A*最小堆） ---------- */

/**
 * @brief 推箱A*优先队列插入
 */
static __attribute__((section("ITCM_NonCacheable"))) void pq_push(PQNode node) {
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
 * @brief 推箱A*优先队列弹出
 */
static __attribute__((section("ITCM_NonCacheable"))) PQNode pq_pop(void) {
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

/* ---------- 5.6 推箱A*启发式与后继 ---------- */

/**
 * @brief 启发式函数（箱子的曼哈顿距离到目标）
 */
static __attribute__((section("ITCM_NonCacheable"))) uint8_t heuristic(const State *s) {
    return (uint8_t)(abs((int)s->box.x - (int)s->target.x) + abs((int)s->box.y - (int)s->target.y));
}

/**
 * @brief 判断是否达到目标状态（箱子到达目标位置）
 */
static bool is_goal(const State *s) {
    return pos_equal(s->box, s->target);
}

/**
 * @brief 生成推箱A*的后继状态
 * 
 * 玩家可移动或推箱子，推箱时检查目标位是否合法（非墙、非死角）。
 * 
 * @param cur 当前状态
 * @param res 后继状态输出数组
 * @param cnt 输出后继数量
 */
static __attribute__((section("ITCM_NonCacheable"))) void get_successors(const State *cur, State *res, uint8_t *cnt) {
    *cnt = 0;
    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        Point np = {cur->player.x + DIRS[d][0], cur->player.y + DIRS[d][1]};
        /* 玩家移动：真墙/真障碍阻挡，目标点可穿过 */
        if (is_wall_bit(cur->wall_bitmap, np)) {
            if (!(g_target_bitmap[np.x] & (1 << np.y))) continue;
        }
        if (pos_equal(np, cur->box)) {
            /* 推箱：wall_bitmap 已含非配对靶位（模式1），箱子无法推入障碍格 */
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

/* ---------- 5.7 BFS 距离与可达性计算 ---------- */

/**
 * @brief 从起点BFS计算到地图所有格子的距离
 */
static __attribute__((section("ITCM_NonCacheable"))) void bfs_compute_distances(Point start, const uint16_t walls[MAP_ROWS]) {
    memset(g_dist_map, 0xFF, sizeof(g_dist_map)); /* INF=0xFFFF */
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
            if (tail < MAP_ROWS * MAP_COLS) g_bfs_queue[tail++] = nxt;
        }
    }
}

/**
 * @brief BFS可达性计算（使用epoch技术避免全零memset）
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
            if (tail < MAP_ROWS * MAP_COLS) g_bfs_queue[tail++] = next;
        }
    }
}

/* ---------- 5.8 路径拐点提取（通用） ---------- */

/**
 * @brief 提取路径的拐点（模式1/3通用路径压缩）
 */
static void extract_turn_points(Path* p) {
    if (p->len == 0) return;
    uint16_t widx = 0;
    g_temp_x[0] = p->x[0]; g_temp_y[0] = p->y[0]; g_temp_push[0] = p->is_push[0];
    widx = 1;
    for (uint16_t i = 1; i < p->len - 1; i++) {
        int8_t dx1 = (int8_t)(p->x[i] - p->x[i-1]);
        int8_t dy1 = (int8_t)(p->y[i] - p->y[i-1]);
        int8_t dx2 = (int8_t)(p->x[i+1] - p->x[i]);
        int8_t dy2 = (int8_t)(p->y[i+1] - p->y[i]);
        if (dx1 != dx2 || dy1 != dy2 || p->is_push[i]) {
            if (widx < MAX_PATH_LEN) {
                g_temp_x[widx] = p->x[i]; g_temp_y[widx] = p->y[i];
                g_temp_push[widx] = p->is_push[i];
            }
            widx++;
        }
    }
    if (widx < MAX_PATH_LEN) {
        g_temp_x[widx] = p->x[p->len - 1]; g_temp_y[widx] = p->y[p->len - 1];
        g_temp_push[widx] = p->is_push[p->len - 1];
    }
    widx++;
    uint16_t copy_n = (widx < MAX_PATH_LEN) ? widx : MAX_PATH_LEN;
    for (uint16_t i = 0; i < copy_n; i++) {
        p->x[i] = g_temp_x[i]; p->y[i] = g_temp_y[i]; p->is_push[i] = g_temp_push[i];
    }
    p->len = (widx > 0) ? (uint16_t)(widx - 1) : 0;
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
 * @brief 判断可炸墙（自定义墙位图版）
 * @param pos   墙位置
 * @param walls 自定义墙位图
 */
static bool is_breakable_wall_on(Point pos, const uint16_t walls[MAP_ROWS]) {
    if (!is_wall_bit(walls, pos)) return false;
    if (pos.x == 0 || pos.x == MAP_ROWS - 1 || pos.y == 0 || pos.y == MAP_COLS - 1) return false;
    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        Point adj = {(uint8_t)(pos.x + DIRS[d][0]), (uint8_t)(pos.y + DIRS[d][1])};
        if (adj.x >= MAP_ROWS || adj.y >= MAP_COLS) continue;
        if (!is_wall_bit(walls, adj)) return true;
    }
    return false;
}

/**
 * @brief 爆炸点有效性检查（自定义墙位图版）
 */
static bool is_valid_detonation_point_on(Point pos, const uint16_t walls[MAP_ROWS]) {
    if (pos.x >= MAP_ROWS || pos.y >= MAP_COLS) return false;
    if (!is_breakable_wall_on(pos, walls)) return false;
    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        Point adj = {(uint8_t)(pos.x + DIRS[d][0]), (uint8_t)(pos.y + DIRS[d][1])};
        if (adj.x >= MAP_ROWS || adj.y >= MAP_COLS) continue;
        if (is_wall_bit(walls, adj)) continue;
        return true;
    }
    return false;
}

/**
 * @brief 判断爆炸3x3范围是否能覆盖目标墙
 */
/* 无分支版本：利用无符号比较实现 -1≤d≤1 即 d+1≤2 */
static inline bool can_explosion_cover_wall(Point detonate_pos, Point target_wall) {
    uint8_t dx = (uint8_t)((int)target_wall.x - (int)detonate_pos.x + 1);
    uint8_t dy = (uint8_t)((int)target_wall.y - (int)detonate_pos.y + 1);
    return (dx <= 2) && (dy <= 2);
}

/**
 * @brief 估算推炸弹距离（自定义墙位图/活跃炸弹版）
 */
static uint16_t estimate_bomb_push_distance_on(Point bomb_pos, Point detonate_pos,
    Point player_pos, const uint16_t walls[MAP_ROWS], const bool active_bombs[MAX_BOOMS]) {
    if (bomb_pos.x == detonate_pos.x && bomb_pos.y == detonate_pos.y) return 0;

    uint16_t player_obs[MAP_ROWS];
    memcpy(player_obs, walls, sizeof(player_obs));
    for (uint8_t i = 0; i < g_box_count; i++)
        player_obs[g_initial_boxes[i].x] |= (1 << g_initial_boxes[i].y);
    for (uint8_t i = 0; i < g_bomb_count; i++) {
        if (active_bombs && !active_bombs[i]) continue;
        if (g_initial_bombs[i].x == BOMB_INVALID) continue;
        player_obs[g_initial_bombs[i].x] |= (1 << g_initial_bombs[i].y);
    }

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

    uint16_t obs[MAP_ROWS];
    memcpy(obs, walls, sizeof(obs));
    for (uint8_t i = 0; i < g_box_count; i++)
        obs[g_initial_boxes[i].x] |= (1 << g_initial_boxes[i].y);
    for (uint8_t i = 0; i < g_bomb_count; i++) {
        if (g_initial_bombs[i].x == bomb_pos.x && g_initial_bombs[i].y == bomb_pos.y) continue;
        if (active_bombs && !active_bombs[i]) continue;
        if (g_initial_bombs[i].x == BOMB_INVALID) continue;
        obs[g_initial_bombs[i].x] |= (1 << g_initial_bombs[i].y);
    }
    obs[detonate_pos.x] &= (uint16_t)~(1 << detonate_pos.y);

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
 * @brief 估算推炸弹距离（自定义墙位图版）
 * 
 * BFS 计算炸弹从当前位置到引爆点的最短行走距离。
 * 障碍物：静态墙 + 所有箱子 + 其他炸弹。
 * 引爆点临时清除（炸弹需要推进去）。
 * 
 * @return BFS最短距离；不可达返回 INF
 */

/**
 * @brief 根据接近位置与元素的相对偏移计算观察角度
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
 * 第6部分：模式算法函数
 * =================================================================== */

/* ---------- 6.1 通用算法（地图解析 & A*寻路） ---------- */

/**
 * @brief 解析地图数据，初始化全局状态
 * 
 * 从二维地图数组中提取静态墙位图、箱子、目标、玩家、炸弹位置，
 * 并保存原始地图副本（模式3用）。
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
    memset(g_initial_boxes, 0, sizeof(g_initial_boxes));
    memset(g_initial_targets, 0, sizeof(g_initial_targets));
    memset(&g_initial_player, 0, sizeof(g_initial_player));
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
                g_target_bitmap[i] |= (1 << j);  /* 记录目标点位图 */
                g_target_count++;
            } else if (val == BOOM && g_bomb_count < MAX_BOOMS) {
                g_initial_bombs[g_bomb_count].x = i;
                g_initial_bombs[g_bomb_count].y = j;
                g_bomb_count++;
            }
        }
    }
}

/**
 * @brief 简单A*寻路（无推箱，带转向惩罚）
 * 
 * 使用最小堆优化，加权启发式 f = g + 1.2*h 加速搜索。
 * 
 * @param start    起点
 * @param end      终点
 * @param walls    墙位图
 * @param out_path 输出路径数组
 * @return 路径长度（0=无路径）
 */
static __attribute__((section("ITCM_NonCacheable"))) uint16_t simple_astar(Point start, Point end, uint16_t walls[MAP_ROWS], Point* out_path) {
    for (int i = 0; i < (MAP_ROWS * MAP_COLS * 5); i++) {
        path_nodes[i].g_cost = INF;
        path_nodes[i].closed = false;
        path_nodes[i].parent_idx = -1;
    }
    s_heap_size = 0;

    int start_idx = get_smooth_idx(start.x, start.y, -1);
    int h_start = abs((int)start.x - (int)end.x) + abs((int)start.y - (int)end.y);
    path_nodes[start_idx].pos = start;
    path_nodes[start_idx].last_dir = -1;
    path_nodes[start_idx].g_cost = 0;
    path_nodes[start_idx].f_cost = (6 * h_start) / 5;
    s_heap_push(start_idx);

    int final_idx = -1;
    while (s_heap_size > 0) {
        int current_idx = s_heap_pop();
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
            if (path_nodes[nidx].closed) continue;
            int move_cost = 1 + ((cur_dir != -1 && cur_dir != (int8_t)d) ? TURN_WEIGHT : 0);
            int ng = cur_g + move_cost;
            if (ng < path_nodes[nidx].g_cost) {
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
 * 将单个箱子从起始位置推到目标位置，搜索最优玩家路径。
 * 使用哈希表去重 + 优先队列最小堆优化。
 * 
 * @param player        玩家起始位置
 * @param box           箱子起始位置
 * @param target        目标位置
 * @param dynamic_walls 动态墙位图（含其他箱子等障碍）
 * @return AStarResult 搜索结果
 */
static __attribute__((section("ITCM_NonCacheable"))) AStarResult solve_single_box_a_star(Point player, Point box, Point target,
                                            uint16_t dynamic_walls[MAP_ROWS]) {
    AStarResult result = {0};
    result.success = false;
    result.cost = -1;
    result.final_player_pos = player;

    memset(pq, 0, sizeof(pq));
    pq_size = 0;

    /* 增量哈希：推进epoch，如回绕则全量清零 */
    g_hash_epoch++;
    if (g_hash_epoch == 0) {
        memset(hash_table, 0, sizeof(hash_table));
        g_hash_epoch = 1;
    }

    State start;
    memset(&start, 0, sizeof(State));
    start.player = player; start.box = box; start.target = target;
    start.last_dir = DIR_INVALID;
    memcpy(start.wall_bitmap, dynamic_walls, sizeof(uint16_t) * MAP_ROWS);

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
        pq_size = 0; return result;
    }

    int len = 0;
    int32_t idx = goal_idx;
    while (idx != (int32_t)UINT16_MAX && len < MAX_PATH_LEN) {
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
    pq_size = 0;
    return result;
}

/* ---------- 6.2 START模式：最近元素接近与路径提取 ---------- */

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
    bool player_found = false;
    for (int i = 0; i < MAP_ROWS && !player_found; i++)
        for (int j = 0; j < MAP_COLS; j++)
            if (map[i][j] == PLAYER) { player.x = (uint8_t)i; player.y = (uint8_t)j; player_found = true; break; }

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
    uint16_t widx = 0;
    g_temp_x[0] = res->path[0].y; g_temp_y[0] = res->path[0].x; widx = 1;
    for (uint16_t i = 1; i < res->path_len - 1; i++) {
        int8_t dx1 = (int8_t)(res->path[i].y   - res->path[i-1].y);
        int8_t dy1 = (int8_t)(res->path[i].x   - res->path[i-1].x);
        int8_t dx2 = (int8_t)(res->path[i+1].y - res->path[i].y);
        int8_t dy2 = (int8_t)(res->path[i+1].x - res->path[i].x);
        if (dx1 != dx2 || dy1 != dy2) {
            if (widx < MAX_PATH_LEN) {
                g_temp_x[widx] = res->path[i].y; g_temp_y[widx] = res->path[i].x;
            }
            widx++;
        }
    }
    if (widx < MAX_PATH_LEN) {
        g_temp_x[widx] = res->path[res->path_len - 1].y;
        g_temp_y[widx] = res->path[res->path_len - 1].x;
    }
    widx++;
    uint16_t copy_n = (widx < MAX_PATH_LEN) ? widx : MAX_PATH_LEN;
    for (uint16_t i = 0; i < copy_n; i++) {
        g_path_start_out.x[i] = g_temp_x[i]; g_path_start_out.y[i] = g_temp_y[i];
    }
    g_path_start_out.len = (widx < MAX_PATH_LEN) ? widx : MAX_PATH_LEN;
    g_path_start_out.type = res->elem_type;
    g_path_start_out.angle = res->angle;
}

/* ---------- 6.3 模式1：贪心配对与回溯验证 ---------- */

/**
 * @brief 贪心算法生成箱子-目标点配对
 *
 * 每次为当前箱子选择距离最近且未使用的目标点。
 *
 * @param sol 输出配对方案
 */
static void generate_greedy_pairing(SolutionSequence* sol) {
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
 * @brief 回溯验证推箱子方案的可行性
 * 
 * 按剩余箱子距玩家的接近度排序，逐个尝试推箱子。
 * 
 * @param sol 配对方案
 * @return true 方案可行
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

    /* BFS计算玩家可达区域，按接近度排序 */
    {
        memcpy(g_bfs_walls, g_static_walls, sizeof(g_bfs_walls));
        for (uint8_t i = 0; i < g_remaining_cnt; i++) {
            Point p = sol->pairs[g_remaining[i]].box_pos;
            g_bfs_walls[p.x] |= (1 << p.y);
        }
        memset(g_dist_map, 0xFF, sizeof(g_dist_map));
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
        /* 构建障碍物：其他箱子 + 会吸收当前箱子的非配对靶位
            模式1：所有靶位都吸收 → 阻挡全部非配对靶位
            模式2：仅同ID靶位吸收 → 仅阻挡同ID的非配对靶位 */
        {
            int8_t my_id = g_box_id_map[sol->pairs[try_idx].box_idx];
            for (uint8_t m = 0; m < g_remaining_cnt; m++) {
                if (m == k) continue;
                uint8_t pi = g_remaining[m];
                Point obs = sol->pairs[pi].box_pos;
                g_current_walls[obs.x] |= (1 << obs.y);
                if (g_mode1_strict ||
                    (my_id != -1 && g_target_id_map[sol->pairs[pi].target_idx] == my_id) ||
                    (my_id != -1
                     && g_orig_target_id_map[sol->pairs[pi].target_idx] != -1
                     && g_orig_target_id_map[sol->pairs[pi].target_idx] == g_orig_box_id_map[sol->pairs[try_idx].box_idx])) {
                    Point tp = sol->pairs[pi].target_pos;
                    g_current_walls[tp.x] |= (1 << tp.y);
                }
            }
        }

        if (!pos_equal(cur_box, cur_target) && is_corner_deadlock(cur_box, g_current_walls)) {
            uint8_t blocker_idx;
            if (is_soft_corner_deadlock(cur_box, g_current_walls, g_solved, &blocker_idx)) {
                /* ── 软死锁：挡路的是未推箱子 → 尝试推开一格让路 ── */
                /* 找到挡路箱在当前 sol 中的配对下标 */
                int8_t blocker_pair = -1;
                for (uint8_t m = 0; m < sol->count; m++)
                    if (sol->pairs[m].box_idx == blocker_idx) { blocker_pair = (int8_t)m; break; }
                if (blocker_pair < 0 || g_solved[blocker_pair]) {
                    continue;
                }

                Point bbox = sol->pairs[blocker_pair].box_pos;

                /* ── 保存状态 ── */
                Point saved_player2 = g_current_player_pos;
                int saved_cost2 = g_total_cost;
                uint16_t saved_len2 = g_fullpath_len;
                Point old_blocker_pos = bbox;
                Point current_blocker = bbox;

                /*
                 * 迭代推开挡路箱：每次推一格，直到本箱死锁解除或无路可走。
                 * 最多迭代 MAX_ITER_PUSH 次，防止无限循环。
                 */
                bool deadlock_broken = false;
                int push_count = 0;
                #define MAX_ITER_PUSH 10

                while (!deadlock_broken && push_count < MAX_ITER_PUSH) {
                    /* 在 current_blocker 周围找最佳可推方向 */
                    bool found = false;
                    Point best_dest = {0, 0};
                    uint16_t best_dist = INF;

                    for (int d = 0; d < DIR_COUNT; d++) {
                        int8_t dx = DIRS[d][0], dy = DIRS[d][1];
                        uint8_t nx = (uint8_t)(current_blocker.x + dx), ny = (uint8_t)(current_blocker.y + dy);
                        if (nx >= MAP_ROWS || ny >= MAP_COLS) continue;
                        if (g_static_walls[nx] & (1 << ny)) continue;
                        /* 不能推回旧位置（防止来回振荡） */
                        if (push_count > 0 && nx == old_blocker_pos.x && ny == old_blocker_pos.y) continue;
                        /* 目标格不能有其他箱子 */
                        bool occupied = false;
                        for (uint8_t m = 0; m < g_remaining_cnt && !occupied; m++) {
                            uint8_t pi = g_remaining[m];
                            if ((int8_t)pi == blocker_pair || g_solved[pi]) continue;
                            if (sol->pairs[pi].box_pos.x == nx && sol->pairs[pi].box_pos.y == ny)
                                occupied = true;
                        }
                        if (occupied) continue;
                        if (nx == cur_box.x && ny == cur_box.y) continue;
                        /* ★ 不能把挡路箱推到死锁位置 */
                        {
                            uint16_t check_walls[MAP_ROWS];
                            memcpy(check_walls, g_static_walls, sizeof(check_walls));
                            for (uint8_t m = 0; m < g_remaining_cnt; m++) {
                                uint8_t pi = g_remaining[m];
                                if ((int8_t)pi == blocker_pair || g_solved[pi]) continue;
                                Point obs = sol->pairs[pi].box_pos;
                                if (obs.x == current_blocker.x && obs.y == current_blocker.y) continue;
                                check_walls[obs.x] |= (1 << obs.y);
                            }
                            Point dest = {nx, ny};
                            if (!pos_equal(dest, sol->pairs[blocker_pair].target_pos) &&
                                is_corner_deadlock(dest, check_walls)) continue;
                        }
                        /* 玩家推箱站位 */
                        uint8_t px = (uint8_t)(current_blocker.x - dx), py = (uint8_t)(current_blocker.y - dy);
                        if (px >= MAP_ROWS || py >= MAP_COLS) continue;
                        if (g_static_walls[px] & (1 << py)) continue;
                        bool pbox = false;
                        if (px == cur_box.x && py == cur_box.y) pbox = true;
                        for (uint8_t m = 0; m < g_remaining_cnt && !pbox; m++) {
                            uint8_t pi = g_remaining[m];
                            if (g_solved[pi]) continue;
                            if (sol->pairs[pi].box_pos.x == px && sol->pairs[pi].box_pos.y == py)
                                pbox = true;
                        }
                        if (pbox) continue;
                        uint16_t dist = g_dist_map[px][py];
                        if (dist == INF) continue;
                        if (dist < best_dist) {
                            best_dist = dist;
                            best_dest.x = nx; best_dest.y = ny;
                            found = true;
                        }
                    }

                    if (!found) {
                        break;
                    }

                    /* 构建挡路箱障碍物（排除挡路箱自身） */
                    uint16_t blocker_walls[MAP_ROWS];
                    memcpy(blocker_walls, g_static_walls, sizeof(blocker_walls));
                    {
                        int8_t bid = g_box_id_map[blocker_idx];
                        for (uint8_t m = 0; m < g_remaining_cnt; m++) {
                            uint8_t pi = g_remaining[m];
                            if ((int8_t)pi == blocker_pair || g_solved[pi]) continue;
                            Point obs = sol->pairs[pi].box_pos;
                            blocker_walls[obs.x] |= (1 << obs.y);
                            if (g_mode1_strict ||
                                (bid != -1 && g_target_id_map[sol->pairs[pi].target_idx] == bid)) {
                                Point tp = sol->pairs[pi].target_pos;
                                blocker_walls[tp.x] |= (1 << tp.y);
                            }
                        }
                    }

                    AStarResult push_ar = solve_single_box_a_star(
                        g_current_player_pos, current_blocker, best_dest, blocker_walls);
                    if (!push_ar.success) {
                        break;
                    }

                    /* 执行这一步推动 */
                    sol->pairs[blocker_pair].box_pos = best_dest;
                    g_current_player_pos = push_ar.final_player_pos;
                    g_total_cost += push_ar.cost;
                    {
                        uint16_t si = 0;
                        if (g_fullpath_len > 0 &&
                            pos_equal(g_fullpath[g_fullpath_len-1], push_ar.path_points[0]))
                            si = 1;
                        uint16_t add = (uint16_t)(push_ar.path_len - si);
                        if (g_fullpath_len + add <= MAX_PATH_LEN)
                            for (uint16_t i = si; i < push_ar.path_len; i++)
                                g_fullpath[g_fullpath_len++] = push_ar.path_points[i];
                    }

                    current_blocker = best_dest;
                    push_count++;

                    /* 重建本箱障碍物，检查死锁是否解除 */
                    memcpy(g_current_walls, g_static_walls, sizeof(g_current_walls));
                    {
                        int8_t my_id2 = g_box_id_map[sol->pairs[try_idx].box_idx];
                        for (uint8_t m = 0; m < g_remaining_cnt; m++) {
                            uint8_t pi = g_remaining[m];
                            if (pi == try_idx || g_solved[pi]) continue;
                            Point obs = sol->pairs[pi].box_pos;
                            g_current_walls[obs.x] |= (1 << obs.y);
                            if (g_mode1_strict ||
                                (my_id2 != -1 && g_target_id_map[sol->pairs[pi].target_idx] == my_id2) ||
                                (my_id2 != -1
                                 && g_orig_target_id_map[sol->pairs[pi].target_idx] != -1
                                 && g_orig_target_id_map[sol->pairs[pi].target_idx] == g_orig_box_id_map[sol->pairs[try_idx].box_idx])) {
                                Point tp = sol->pairs[pi].target_pos;
                                g_current_walls[tp.x] |= (1 << tp.y);
                            }
                        }
                    }

                    deadlock_broken = (!pos_equal(cur_box, cur_target) &&
                        !is_corner_deadlock(cur_box, g_current_walls));
                }
                #undef MAX_ITER_PUSH

                if (deadlock_broken) {
                    g_backtrack_ar = solve_single_box_a_star(
                        g_current_player_pos, cur_box, cur_target, g_current_walls);
                    if (g_backtrack_ar.success) {
                        /* 事后验证本箱 */
                        {
                            int8_t mid = g_box_id_map[sol->pairs[try_idx].box_idx];
                            uint16_t forbid[MAP_ROWS] = {0};
                            for (uint8_t m = 0; m < sol->count; m++) {
                                if (m == try_idx || g_solved[m]) continue;
                                uint8_t ti = sol->pairs[m].target_idx;
                                if (g_mode1_strict ||
                                    (mid != -1 && g_target_id_map[ti] == mid) ||
                                    (mid != -1
                                     && g_orig_target_id_map[ti] != -1
                                     && g_orig_target_id_map[ti] == g_orig_box_id_map[sol->pairs[try_idx].box_idx]))
                                    forbid[sol->pairs[m].target_pos.x] |= (1 << sol->pairs[m].target_pos.y);
                            }
                            Point sim = cur_box; bool valid = true;
                            for (uint16_t i = 0; i < g_backtrack_ar.path_len && valid; i++) {
                                if (pos_equal(g_backtrack_ar.path_points[i], sim)) {
                                    if (i > 0) {
                                        int8_t dx = (int8_t)(g_backtrack_ar.path_points[i].x - g_backtrack_ar.path_points[i-1].x);
                                        int8_t dy = (int8_t)(g_backtrack_ar.path_points[i].y - g_backtrack_ar.path_points[i-1].y);
                                        sim.x = (uint8_t)(sim.x + dx); sim.y = (uint8_t)(sim.y + dy);
                                        if (!pos_equal(sim, cur_target) && (forbid[sim.x] & (1 << sim.y)))
                                            valid = false;
                                    }
                                }
                            }
                            if (valid) {
                                g_solved[try_idx] = true;
                                g_solve_order[g_order_idx++] = try_idx;
                                g_current_player_pos = g_backtrack_ar.final_player_pos;
                                g_total_cost += g_backtrack_ar.cost;
                                {
                                    uint16_t si = 0;
                                    if (g_fullpath_len > 0 &&
                                        pos_equal(g_fullpath[g_fullpath_len-1], g_backtrack_ar.path_points[0]))
                                        si = 1;
                                    uint16_t add = (uint16_t)(g_backtrack_ar.path_len - si);
                                    if (g_fullpath_len + add <= MAX_PATH_LEN)
                                        for (uint16_t i = si; i < g_backtrack_ar.path_len; i++)
                                            g_fullpath[g_fullpath_len++] = g_backtrack_ar.path_points[i];
                                }
                                if (backtrack_validate(sol)) return true;
                                /* 回溯本箱 */
                                g_solved[try_idx] = false;
                                g_order_idx--;
                                g_total_cost -= g_backtrack_ar.cost;
                                g_fullpath_len -= (uint16_t)(g_backtrack_ar.path_len -
                                    (g_fullpath_len > 0 && pos_equal(g_fullpath[g_fullpath_len-1], g_backtrack_ar.path_points[0]) ? 1 : 0));
                            }
                        }
                    }
                }

                /* 恢复挡路箱位置和状态 */
                sol->pairs[blocker_pair].box_pos = old_blocker_pos;
                g_current_player_pos = saved_player2;
                g_total_cost = saved_cost2;
                g_fullpath_len = saved_len2;
                continue;
            } else {
                continue;
            }
        }

        g_backtrack_ar = solve_single_box_a_star(g_current_player_pos, cur_box, cur_target, g_current_walls);
        if (!g_backtrack_ar.success) {
            continue;
        }

        /* 事后验证：箱子不穿越会吸收它的非配对靶位（位图 O(1) 查表）
            模式1：所有靶位都吸收 → 检查全部非配对靶位
            模式2：仅同ID靶位吸收 → 仅检查同ID的非配对靶位 */
        {
            int8_t my_id = g_box_id_map[sol->pairs[try_idx].box_idx];
            uint16_t forbid_targets[MAP_ROWS] = {0};
            for (uint8_t m = 0; m < sol->count; m++) {
                if (m == try_idx || g_solved[m]) continue;
                uint8_t ti = sol->pairs[m].target_idx;
                if (g_mode1_strict ||
                    (my_id != -1 && g_target_id_map[ti] == my_id) ||
                    (my_id != -1
                     && g_orig_target_id_map[ti] != -1
                     && g_orig_target_id_map[ti] == g_orig_box_id_map[sol->pairs[try_idx].box_idx]))
                    forbid_targets[sol->pairs[m].target_pos.x] |= (1 << sol->pairs[m].target_pos.y);
            }

            Point sim_box = cur_box;
            bool box_path_valid = true;
            for (uint16_t i = 0; i < g_backtrack_ar.path_len && box_path_valid; i++) {
                if (pos_equal(g_backtrack_ar.path_points[i], sim_box)) {
                    if (i > 0) {
                        int8_t pdx = (int8_t)(g_backtrack_ar.path_points[i].x - g_backtrack_ar.path_points[i - 1].x);
                        int8_t pdy = (int8_t)(g_backtrack_ar.path_points[i].y - g_backtrack_ar.path_points[i - 1].y);
                        sim_box.x = (uint8_t)(sim_box.x + pdx);
                        sim_box.y = (uint8_t)(sim_box.y + pdy);
                        if (!pos_equal(sim_box, cur_target) &&
                            (forbid_targets[sim_box.x] & (1 << sim_box.y)))
                            box_path_valid = false;
                    }
                }
            }
            if (!box_path_valid) {
                continue;
            }
        }

        g_solved[try_idx] = true;
        g_solve_order[g_order_idx++] = try_idx;
        Point saved_player = g_current_player_pos;
        int saved_cost = g_total_cost;
        uint16_t saved_len = g_fullpath_len;

        g_current_player_pos = g_backtrack_ar.final_player_pos;
        g_total_cost += g_backtrack_ar.cost;

        uint16_t start_idx = 0;
        if (g_fullpath_len > 0 && pos_equal(g_fullpath[g_fullpath_len - 1], g_backtrack_ar.path_points[0]))
            start_idx = 1;
        uint16_t add_len = (uint16_t)(g_backtrack_ar.path_len - start_idx);
        if (g_fullpath_len + add_len <= MAX_PATH_LEN) {
            for (uint16_t i = start_idx; i < g_backtrack_ar.path_len; i++)
                g_fullpath[g_fullpath_len++] = g_backtrack_ar.path_points[i];
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
 * @brief 验证推箱子方案的入口函数
 * 
 * 初始化回溯状态，调用 backtrack_validate 递归验证，
 * 成功后将配对按解决顺序重排（保持最优序）。
 * 
 * @param sol 待验证的配对方案
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

/* ---------- 6.4 模式2：ID学习与推理 ---------- */

/**
 * @brief ID学习阶段：生成访问计划
 * 
 * 基于BFS距离计算，规划访问所有箱子和目标点的顺序和路径。
 * 先全遍历确定"最后一个"元素，再生成排除最后一个的访问序列。
 */
static void id_learning(uint8_t map[MAP_ROWS][MAP_COLS]) {
    parse_map_input(map);
    /* 模式二中炸弹视为墙（不可通行） */
    for (uint8_t b = 0; b < g_bomb_count; b++)
        g_static_walls[g_initial_bombs[b].x] |= (1 << g_initial_bombs[b].y);
    uint16_t *walls = g_tmp_walls2;  /* 指向全局缓冲区省栈 */
    memset(walls, 0, sizeof(uint16_t) * MAP_ROWS);
    for (int i = 0; i < MAP_ROWS; i++)
        for (int j = 0; j < MAP_COLS; j++)
            if (map[i][j] == WALL || map[i][j] == BOX || map[i][j] == BOOM)
                walls[i] |= (1 << j);

    if (g_box_count == 0 || g_target_count == 0) { g_visit_count = 0; return; }

    /* 第一阶段：全遍历确定"最后一个" */
    bool tvb[MAX_BOXES] = {false}, tvt[MAX_BOXES] = {false};
    Point tcp = g_initial_player;
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
        if (bt == 0) { tvb[bei] = true; }
        else         { tvt[bei] = true; }
        tcp = ba;
    }

    /* 第二阶段：目标观测 n-1 个箱子和 n-1 个靶位。
     * 不预先排除任何元素，靠推箱开路尽量达成。推箱失败则接受部分结果。 */
    bool vb[MAX_BOXES] = {false}, vt[MAX_BOXES] = {false};
    int need_box = (g_box_count > 0) ? g_box_count - 1 : 0;
    int need_tgt = (g_target_count > 0) ? g_target_count - 1 : 0;

    Point cp = g_initial_player;
    g_visit_count = 0;
    g_current_step = 0;  /* ★ 重置步骤游标，防止上次运行的旧值导致 id_input 跳过 */
    int observed_box = 0, observed_tgt = 0;

    while (observed_box < need_box || observed_tgt < need_tgt) {
        bfs_compute_distances(cp, walls);
        uint16_t md = INF;
        int bei = -1; uint8_t bt = 0; Point ba = {0,0};
        if (observed_box < need_box) {
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
        } /* observed_box < need_box */
        if (observed_tgt < need_tgt) {
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
        } /* observed_tgt < need_tgt */
        if (bei == -1) {
            bool pushed = false;
            int rescue_attempts = 0;
            #define MAX_RESCUE 20

            while (!pushed && rescue_attempts < MAX_RESCUE) {
                rescue_attempts++;
                uint16_t walls_no_box[MAP_ROWS];
                memcpy(walls_no_box, g_static_walls, sizeof(walls_no_box));
                bfs_compute_distances(cp, walls_no_box);
                bool any_hidden = false;
                for (int i = 0; i < g_box_count && !any_hidden; i++) {
                    if (vb[i]) continue;
                    Point bp = g_initial_boxes[i];
                    for (int d = 0; d < DIR_COUNT && !any_hidden; d++) {
                        Point ap = {bp.x + DIRS[d][0], bp.y + DIRS[d][1]};
                        if (ap.x >= MAP_ROWS || ap.y >= MAP_COLS) continue;
                        if (g_static_walls[ap.x] & (1 << ap.y)) continue;
                        if (g_dist_map[ap.x][ap.y] < INF) any_hidden = true;
                    }
                }
                for (int i = 0; i < g_target_count && !any_hidden; i++) {
                    if (vt[i]) continue;
                    Point tp = g_initial_targets[i];
                    for (int d = 0; d < DIR_COUNT && !any_hidden; d++) {
                        Point ap = {tp.x + DIRS[d][0], tp.y + DIRS[d][1]};
                        if (ap.x >= MAP_ROWS || ap.y >= MAP_COLS) continue;
                        if (g_static_walls[ap.x] & (1 << ap.y)) continue;
                        if (g_dist_map[ap.x][ap.y] < INF) any_hidden = true;
                    }
                }
                if (!any_hidden) break;

                bfs_compute_distances(cp, walls);
                /* ★ 选最相关的箱子：离剩余元素最近的（用无障碍BFS距离） */
                int best_bi = -1;
                uint16_t best_relevance = INF;
                for (int bi = 0; bi < g_box_count; bi++) {
                    Point bp = g_initial_boxes[bi];
                    bool can_push = false;
                    for (int d = 0; d < DIR_COUNT && !can_push; d++) {
                        Point ap = {bp.x + DIRS[d][0], bp.y + DIRS[d][1]};
                        if (ap.x >= MAP_ROWS || ap.y >= MAP_COLS) continue;
                        if (g_dist_map[ap.x][ap.y] < INF) can_push = true;
                    }
                    if (!can_push) continue;
                    /* 用 any_hidden 阶段的 BFS 计算箱子到剩余元素的距离 */
                    uint16_t rel = INF;
                    for (int i = 0; i < g_box_count; i++) {
                        if (vb[i]) continue;
                        Point tbp = g_initial_boxes[i];
                        uint16_t d = manhattan_distance(bp, tbp);
                        if (d < rel) rel = d;
                    }
                    for (int i = 0; i < g_target_count; i++) {
                        if (vt[i]) continue;
                        Point tp = g_initial_targets[i];
                        uint16_t d = manhattan_distance(bp, tp);
                        if (d < rel) rel = d;
                    }
                    if (rel < best_relevance) { best_relevance = rel; best_bi = bi; }
                }
                if (best_bi < 0) { if (!pushed) break; continue; }
                {
                    int bi = best_bi;
                    Point bp = g_initial_boxes[bi];

                    /* ★ 推一格，检查是否打开通路；推后让外层重新扫描 */
                    {
                        bool found_dir = false;
                        uint8_t best_nx = 0, best_ny = 0, best_px = 0, best_py = 0;
                        uint16_t best_dist = INF;
                        /* 防振荡：记录最近推过的位置 */
                        static Point last_from[MAX_BOXES];
                        static bool last_init = false;
                        if (!last_init) { for (int li = 0; li < MAX_BOXES; li++) last_from[li] = (Point){0xFF,0xFF}; last_init = true; }

                        for (int d = 0; d < DIR_COUNT; d++) {
                            int8_t dx = DIRS[d][0], dy = DIRS[d][1];
                            uint8_t nx = (uint8_t)(bp.x + dx), ny = (uint8_t)(bp.y + dy);
                            if (nx >= MAP_ROWS || ny >= MAP_COLS) continue;
                            if (walls[nx] & (1 << ny)) continue;
                            /* 禁止推回上次推来的位置 */
                            if (pos_equal((Point){nx, ny}, last_from[bi])) continue;
                            uint8_t px = (uint8_t)(bp.x - dx), py = (uint8_t)(bp.y - dy);
                            if (px >= MAP_ROWS || py >= MAP_COLS) continue;
                            if (walls[px] & (1 << py)) continue;
                            if (g_dist_map[px][py] == INF) continue;
                            { uint16_t cw[MAP_ROWS]; memcpy(cw, walls, sizeof(cw));
                                cw[bp.x] &= (uint16_t)~(1 << bp.y);
                                if (is_corner_deadlock((Point){nx, ny}, cw)) continue; }
                            if (g_dist_map[px][py] < best_dist) {
                                best_dist = g_dist_map[px][py];
                                best_nx = nx; best_ny = ny;
                                best_px = px; best_py = py;
                                found_dir = true;
                            }
                        }
                        if (!found_dir) continue;

                        /* 记录推箱路径 */
                        {
                            Point saved_cp2 = cp;
                            VisitStep *ps = &g_visit_plan[g_visit_count];
                            ps->pos = bp;
                            ps->original_idx = (uint8_t)bi;
                            ps->type = 0;
                            Point push_stand = {best_px, best_py};
                            uint16_t walk_len = simple_astar(saved_cp2, push_stand, walls, ps->path);
                            if (walk_len == 0 && !pos_equal(saved_cp2, push_stand)) {
                                ps->path[0] = bp; ps->path_len = 1;
                            } else {
                                if (walk_len < MAX_PATH_LEN) {
                                    ps->path[walk_len] = bp;
                                    ps->path_len = (uint16_t)(walk_len + 1);
                                } else { ps->path_len = walk_len; }
                            }
                            ps->angle = 0; ps->direction = DIR_INVALID;
                            g_visit_count++;
                        }
                        /* 记录推前位置，防振荡 */
                        last_from[bi] = bp;
                        /* 执行推箱 */
                        walls[bp.x] &= (uint16_t)~(1 << bp.y);
                        walls[best_nx] |= (1 << best_ny);
                        g_initial_boxes[bi].x = best_nx;
                        g_initial_boxes[bi].y = best_ny;
                        cp = bp;

                        /* 检查是否打开通路 */
                        bfs_compute_distances(cp, walls);
                        bool helps = false;
                        for (int i = 0; i < g_box_count && !helps; i++) {
                            if (vb[i]) continue;
                            Point tbp = g_initial_boxes[i];
                            for (int dd = 0; dd < DIR_COUNT && !helps; dd++) {
                                Point ap = {tbp.x + DIRS[dd][0], tbp.y + DIRS[dd][1]};
                                if (ap.x >= MAP_ROWS || ap.y >= MAP_COLS) continue;
                                if (walls[ap.x] & (1 << ap.y)) continue;
                                if (g_dist_map[ap.x][ap.y] < INF) helps = true;
                            }
                        }
                        for (int i = 0; i < g_target_count && !helps; i++) {
                            if (vt[i]) continue;
                            Point tp = g_initial_targets[i];
                            for (int dd = 0; dd < DIR_COUNT && !helps; dd++) {
                                Point ap = {tp.x + DIRS[dd][0], tp.y + DIRS[dd][1]};
                                if (ap.x >= MAP_ROWS || ap.y >= MAP_COLS) continue;
                                if (walls[ap.x] & (1 << ap.y)) continue;
                                if (g_dist_map[ap.x][ap.y] < INF) helps = true;
                            }
                        }
                        if (helps) pushed = true;
                    }
                }
            }
            #undef MAX_RESCUE
            if (!pushed) break;
            continue;
        }

        VisitStep* step = &g_visit_plan[g_visit_count];
        step->pos = ba;
        step->original_idx = (uint8_t)bei;
        step->type = (bt == 0) ? BOX : TARGET;
        uint16_t tw[MAP_ROWS];
        memcpy(tw, walls, sizeof(tw));
        tw[ba.x] &= ~(1 << ba.y);
        step->path_len = simple_astar(cp, ba, tw, step->path);
        if (step->path_len == 0 && !pos_equal(cp, ba)) {
            step->path_len = 0;
        } else if (step->path_len == 0 && pos_equal(cp, ba)) {
            /* 已在接近位置 → 从前一个 visit 终点补全路径 */
            Point from = cp;
            if (g_visit_count > 0) {
                VisitStep *prev_step = &g_visit_plan[g_visit_count - 1];
                if (prev_step->path_len > 0)
                    from = prev_step->path[prev_step->path_len - 1];
            }
            uint16_t ftw[MAP_ROWS]; memcpy(ftw, walls, sizeof(ftw));
            ftw[ba.x] &= ~(1 << ba.y);
            step->path_len = simple_astar(from, ba, ftw, step->path);
            if (step->path_len == 0) {
                step->path[0] = ba; step->path_len = 1;
            }
        }
        Point ep = (step->type == BOX) ? g_initial_boxes[bei] : g_initial_targets[bei];
        int8_t dxep = (int8_t)(ep.x - ba.x);
        int8_t dyep = (int8_t)(ep.y - ba.y);
        step->angle = compute_approach_angle(dxep, dyep);
        if (dxep == -1)      step->direction = 0;
        else if (dxep == 1)  step->direction = 1;
        else if (dyep == -1) step->direction = 2;
        else if (dyep == 1)  step->direction = 3;
        else                 step->direction = DIR_INVALID;
        if (bt == 0) { vb[bei] = true; observed_box++; }
        else         { vt[bei] = true; observed_tgt++; }
        g_visit_count++;
        cp = ba;
    }
}

/**
 * @brief 记录用户输入的ID，绑定到当前观察步骤的元素
 * 
 * 仅在观察阶段（g_current_step < g_visit_count）有效。
 * 
 * @param id 用户扫描得到的ID编号
 */
static void id_record(int id) {
    if (g_current_step >= g_visit_count) return;
    VisitStep* step = &g_visit_plan[g_current_step];
    if (step->type == BOX) g_box_id_map[step->original_idx] = (int8_t)id;
    else                   g_target_id_map[step->original_idx] = (int8_t)id;
}

/**
 * @brief 下一个字典序排列（用于ID同配对的穷举）
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
 * @brief ID推理：根据已知ID推断未知配对
 * 
 * 核心逻辑：统计频次→平衡→分配新ID→多配对优化（曼哈顿距离最优匹配）。
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

            /* 预计算 BFS 真实距离矩阵 dist[i][j] = 箱子bi[i]到目标ti[j]的最短路径 */
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
                bool inf_found = false;
                for (int i = 0; i < k; i++) {
                    if (dist[i][perm[i]] == INF) { inf_found = true; break; }
                    cost += (int)dist[i][perm[i]];
                }
                if (!inf_found && cost < minc) { minc = cost; for (int i = 0; i < k; i++) bperm[i] = perm[i]; }
            } while (next_permutation(perm, k));
            bool occ[MAX_IDS] = {false};
            for (int i = 0; i < MAX_IDS; i++) if (bfreq[i] > 0 || tfreq[i] > 0) occ[i] = true;
            for (int i = 1; i < k; i++) {
                int nid = 0;
                while (nid < MAX_IDS && occ[nid]) nid++;
                if (nid >= MAX_IDS) break;
                g_box_id_map[bi[i]] = (int8_t)nid;
                g_target_id_map[ti[bperm[i]]] = (int8_t)nid;
                if (bfreq[id] > 0) bfreq[id]--;
                if (tfreq[id] > 0) tfreq[id]--;
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
        /* 若路径只有1点但与 prev 不连续 → 补全间隙（推箱后瞬移导致） */
        if (plan[i].path_len == 1 && !pos_equal(prev, plan[i].path[0])) {
            memcpy(g_tmp_walls1, g_static_walls, sizeof(g_tmp_walls1));
            uint16_t gap_len = simple_astar(prev, plan[i].path[0], g_tmp_walls1, g_temp_path);
            if (gap_len > 0) {
                Point gp = prev;
                for (uint16_t gi = 0; gi < gap_len; gi++) {
                    Point gc = g_temp_path[gi];
                    if (gi < gap_len - 1) {
                        Point gn = g_temp_path[gi + 1];
                        int8_t dx1 = (int8_t)(gc.x - gp.x), dy1 = (int8_t)(gc.y - gp.y);
                        int8_t dx2 = (int8_t)(gn.x - gc.x), dy2 = (int8_t)(gn.y - gc.y);
                        if ((dx1 != dx2 || dy1 != dy2) &&
                            !(li > 0 && g_path_look_out.y[li-1] == gc.x && g_path_look_out.x[li-1] == gc.y)) {
                            if (li >= MAX_PATH_LEN) break;
                            g_path_look_out.x[li] = gc.y; g_path_look_out.y[li] = gc.x;
                            g_path_look_out.angle[li] = 0; g_path_look_out.type[li] = 0;
                            g_path_look_out.is_look[li] = 0; li++;
                        }
                    }
                    gp = gc;
                }
            }
            /* 记录观测/推箱终点（同位置多观测：分别保留，不再覆盖） */
            if (li < MAX_PATH_LEN) {
                Point ep = plan[i].path[0];
                if (li > 0 && g_path_look_out.y[li-1] == ep.x && g_path_look_out.x[li-1] == ep.y) {
                    if (plan[i].type != 0 && !g_path_look_out.is_look[li-1]) { g_path_look_out.is_look[li-1] = 1; g_path_look_out.type[li-1] = plan[i].type; g_path_look_out.angle[li-1] = plan[i].angle; }
                    else if (plan[i].type != 0) {
                        g_path_look_out.x[li] = ep.y;
                        g_path_look_out.y[li] = ep.x;
                        g_path_look_out.angle[li] = plan[i].angle;
                        g_path_look_out.type[li] = plan[i].type;
                        g_path_look_out.is_look[li] = 1; li++;
                    }
                } else {
                    g_path_look_out.x[li] = ep.y;
                    g_path_look_out.y[li] = ep.x;
                    g_path_look_out.angle[li] = plan[i].angle;
                    g_path_look_out.type[li] = plan[i].type;
                    g_path_look_out.is_look[li] = (plan[i].type == 0) ? 0 : 1; li++;
                }
            }
            continue;
        }
        for (int j = 0; j < plan[i].path_len; j++) {
            Point cur = plan[i].path[j];
            if (j == plan[i].path_len - 1) {
                if (li >= MAX_PATH_LEN) break;
                /* 同位置多观测：分别保留，不再覆盖 */
                if (li > 0 && g_path_look_out.y[li-1] == cur.x && g_path_look_out.x[li-1] == cur.y) {
                    if (plan[i].type != 0 && !g_path_look_out.is_look[li-1]) { g_path_look_out.is_look[li-1] = 1; g_path_look_out.type[li-1] = plan[i].type; g_path_look_out.angle[li-1] = plan[i].angle; }
                    else if (plan[i].type != 0) {
                        g_path_look_out.x[li] = cur.y;
                        g_path_look_out.y[li] = cur.x;
                        g_path_look_out.angle[li] = plan[i].angle;
                        g_path_look_out.type[li] = plan[i].type;
                        g_path_look_out.is_look[li] = 1; li++;
                    }
                } else {
                    g_path_look_out.x[li] = cur.y; g_path_look_out.y[li] = cur.x;
                    g_path_look_out.angle[li] = plan[i].angle;
                    g_path_look_out.type[li] = plan[i].type;
                    g_path_look_out.is_look[li] = (plan[i].type == 0) ? 0 : 1; li++;
                }
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
    /* 后处理：压缩连续同向推箱点（type=0 且中间点可省略） */
    if (li >= 3) {
        int wi = 1; /* 写指针 */
        for (int ri = 2; ri < li; ri++) {
            /* 检查 ri-1 是否可压缩：type=0, is_look=0, 且三点共线 */
            if (g_path_look_out.is_look[ri-1] == 0 && g_path_look_out.type[ri-1] == 0) {
                int8_t dx1 = (int8_t)(g_path_look_out.x[ri-1] - g_path_look_out.x[ri-2]);
                int8_t dy1 = (int8_t)(g_path_look_out.y[ri-1] - g_path_look_out.y[ri-2]);
                int8_t dx2 = (int8_t)(g_path_look_out.x[ri] - g_path_look_out.x[ri-1]);
                int8_t dy2 = (int8_t)(g_path_look_out.y[ri] - g_path_look_out.y[ri-1]);
                if (dx1 == dx2 && dy1 == dy2) continue; /* 中间点可省略 */
            }
            g_path_look_out.x[wi] = g_path_look_out.x[ri-1];
            g_path_look_out.y[wi] = g_path_look_out.y[ri-1];
            g_path_look_out.angle[wi] = g_path_look_out.angle[ri-1];
            g_path_look_out.type[wi] = g_path_look_out.type[ri-1];
            g_path_look_out.is_look[wi] = g_path_look_out.is_look[ri-1];
            wi++;
        }
        /* 最后一个点 */
        g_path_look_out.x[wi] = g_path_look_out.x[li-1];
        g_path_look_out.y[wi] = g_path_look_out.y[li-1];
        g_path_look_out.angle[wi] = g_path_look_out.angle[li-1];
        g_path_look_out.type[wi] = g_path_look_out.type[li-1];
        g_path_look_out.is_look[wi] = g_path_look_out.is_look[li-1];
        li = wi + 1;
    }
    g_path_look_out.len = (uint16_t)li;
}

/**
 * @brief 从ID配对构造解决方案
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
 * @brief 从ID方案计算推箱子路径
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

/* ---------- 6.5 模式3：炸弹破局分析 ---------- */

/**
 * @brief 计算玩家可达区域（箱子和炸弹视为障碍）
 */
static __attribute__((section("ITCM_NonCacheable"))) void compute_player_region_with_walls(const uint16_t walls[MAP_ROWS], bool ignore_bombs) {
    memset(g_obs_buf, 0, sizeof(g_obs_buf));
    for (uint8_t i = 0; i < MAP_ROWS; i++) for (uint8_t j = 0; j < MAP_COLS; j++) {
        uint8_t v = g_original_map[i][j];
        if (v == WALL) continue;
        if (v == BOX) g_obs_buf[i] |= (1 << j);
        if (v == BOOM && !ignore_bombs) g_obs_buf[i] |= (1 << j);
    }
    g_player_region_epoch++;
    if (g_player_region_epoch == 0) {
        memset(g_player_region, 0, sizeof(g_player_region));
        g_player_region_epoch = 1;
    }
    for (uint8_t i = 0; i < MAP_ROWS; i++) for (uint8_t j = 0; j < MAP_COLS; j++)
        if (walls[i] & (1 << j)) g_player_region[i][j] = g_player_region_epoch;
    uint8_t head = 0, tail = 0;
    g_player_region[g_initial_player.x][g_initial_player.y] = g_player_region_epoch;
    g_bfs_queue[tail++] = g_initial_player;
    while (head < tail) {
        Point curr = g_bfs_queue[head++];
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            Point next = {(uint8_t)(curr.x + DIRS[d][0]), (uint8_t)(curr.y + DIRS[d][1])};
            if (next.x >= MAP_ROWS || next.y >= MAP_COLS) continue;
            if (g_player_region[next.x][next.y] == g_player_region_epoch) continue;
            if (walls[next.x] & (1 << next.y)) continue;
            if (g_obs_buf[next.x] & (1 << next.y)) continue;
            g_player_region[next.x][next.y] = g_player_region_epoch;
            g_bfs_queue[tail++] = next;
        }
    }
}

/**
 * @brief 计算"玩家可炸到的墙"——仅清除与玩家区域连通的可炸墙（方向性宽松墙）
 * 
 * 从玩家可达区域出发，BFS 扩展：可穿越地板 + 可炸墙。
 * 清除所有被穿越的可炸墙，得到 bomb_walls（用于后续 ENC 合并/目标验证）。
 * 
 * @param bomb_walls  输出：g_static_walls 基础上仅清除玩家可达的可炸墙
 */
static void compute_player_bomb_walls(uint16_t bomb_walls[MAP_ROWS]) {
    memcpy(bomb_walls, g_static_walls, sizeof(uint16_t) * MAP_ROWS);

    /* 用 g_player_region 作为起点集（玩家无需炸弹即可达的区域） */
    uint16_t head = 0, tail = 0;
    memset(g_bomb_visited, 0, sizeof(g_bomb_visited));

    /* 将所有玩家可达单元格入队（排除墙——g_player_region用epoch预标记了墙） */
    for (uint8_t i = 0; i < MAP_ROWS; i++)
        for (uint8_t j = 0; j < MAP_COLS; j++)
            if (g_player_region[i][j] == g_player_region_epoch && !is_wall_bit(g_static_walls, (Point){i, j})) {
                g_bomb_visited[i][j] = 1;
                g_bfs_queue[tail++] = (Point){i, j};
            }

    /* BFS：可穿越地板（非墙非箱非炸弹）和可炸墙 */
    while (head < tail) {
        Point curr = g_bfs_queue[head++];
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            Point next = {(uint8_t)(curr.x + DIRS[d][0]), (uint8_t)(curr.y + DIRS[d][1])};
            if (next.x >= MAP_ROWS || next.y >= MAP_COLS) continue;
            if (g_bomb_visited[next.x][next.y]) continue;

            bool is_breakable = is_breakable_wall_on(next, g_static_walls);
            bool is_floor = !is_wall_bit(g_static_walls, next)
                        && g_original_map[next.x][next.y] != BOX
                        && g_original_map[next.x][next.y] != BOOM;

            if (is_breakable) {
                /* 可炸墙：穿越并清除 */
                bomb_walls[next.x] &= (uint16_t)~(1 << next.y);
                g_bomb_visited[next.x][next.y] = 1;
                g_bfs_queue[tail++] = next;
            } else if (is_floor) {
                /* 地板：直接穿越 */
                g_bomb_visited[next.x][next.y] = 1;
                g_bfs_queue[tail++] = next;
            }
            /* 否则：不可穿越（静态墙/箱子/炸弹），跳过 */
        }
    }
}

/**
 * @brief 找围住影响域的墙：分隔ENC内外的边界墙
 * 
 * 条件：1) 可炸墙 2) 玩家可达 3) 紧邻影响域 4) 至少一侧在影响域外
 * 
 * @param influence   影响域位图
 * @param bomb_walls  方向性宽松墙位图（已清除的可炸墙在此图中 bit=0）
 * @param out_walls   输出墙坐标数组
 * @param max_walls   输出数组容量
 * @return 找到的墙数量
 */
static uint8_t find_enclosing_walls(const uint16_t influence[MAP_ROWS],
    const uint16_t bomb_walls[MAP_ROWS], Point out_walls[], uint8_t max_walls) {
    uint8_t count = 0;

    /* 预计算"真·外侧"：从玩家区域出发，把影响域当墙，能到达的格子才是外侧 */
    uint16_t exterior[MAP_ROWS];
    memset(exterior, 0, sizeof(exterior));
    {
        uint8_t head = 0, tail = 0;
        for (uint8_t r = 0; r < MAP_ROWS; r++)
            for (uint8_t c = 0; c < MAP_COLS; c++)
                if (g_player_region[r][c] == g_player_region_epoch &&
                    !is_wall_bit(g_static_walls, (Point){r,c}) &&
                    !(influence[r] & (1 << c))) {
                    exterior[r] |= (1 << c);
                    g_bfs_queue[tail++] = (Point){r, c};
                }
        while (head < tail) {
            Point cur = g_bfs_queue[head++];
            for (uint8_t d = 0; d < DIR_COUNT; d++) {
                Point nb = {(uint8_t)(cur.x + DIRS[d][0]), (uint8_t)(cur.y + DIRS[d][1])};
                if (nb.x >= MAP_ROWS || nb.y >= MAP_COLS) continue;
                if (exterior[nb.x] & (1 << nb.y)) continue;
                if (is_wall_bit(g_static_walls, nb)) continue;
                if (influence[nb.x] & (1 << nb.y)) continue;  /* 影响域当墙 */
                exterior[nb.x] |= (1 << nb.y);
                g_bfs_queue[tail++] = nb;
            }
        }
    }

    /* 遍历影响域每个格子，检查其4邻域 */
    for (uint8_t r = 1; r < MAP_ROWS - 1; r++) {
        uint16_t row = influence[r];
        if (row == 0) continue;
        for (uint8_t c = 1; c < MAP_COLS - 1; c++) {
            if (!(row & (1 << c))) continue;
            for (uint8_t d = 0; d < DIR_COUNT && count < max_walls; d++) {
                Point nb = {(uint8_t)(r + DIRS[d][0]), (uint8_t)(c + DIRS[d][1])};
                if (nb.x >= MAP_ROWS || nb.y >= MAP_COLS) continue;
                if (!is_wall_bit(g_static_walls, nb)) continue;
                if (is_wall_bit(bomb_walls, nb)) continue;
                if (!is_breakable_wall_on(nb, g_static_walls)) continue;
                /* 必须至少一侧在真·外侧（不穿影响域可达玩家） */
                bool has_outside = false;
                for (uint8_t wd = 0; wd < DIR_COUNT && !has_outside; wd++) {
                    Point wn = {(uint8_t)(nb.x + DIRS[wd][0]), (uint8_t)(nb.y + DIRS[wd][1])};
                    if (wn.x >= MAP_ROWS || wn.y >= MAP_COLS) continue;
                    if (is_wall_bit(g_static_walls, wn)) continue;
                    if (exterior[wn.x] & (1 << wn.y)) has_outside = true;
                }
                if (!has_outside) continue;
                bool dup = false;
                for (uint8_t k = 0; k < count; k++)
                    if (pos_equal(out_walls[k], nb)) { dup = true; break; }
                if (!dup) out_walls[count++] = nb;
            }
        }
    }
    return count;
}

/**
 * @brief 计算箱子的影响域（可达区域位图）
 */
static void compute_box_influence(Point box, const uint16_t walls[MAP_ROWS],
    uint16_t influence_mask[MAP_ROWS]) {
    memset(influence_mask, 0, MAP_ROWS * sizeof(uint16_t));
    g_dist_epoch++;
    if (g_dist_epoch == 0) { memset(g_dist_epoch_tag, 0, sizeof(g_dist_epoch_tag)); g_dist_epoch = 1; }
    uint16_t head = 0, tail = 0;
    g_dist_epoch_tag[box.x][box.y] = g_dist_epoch;
    g_bfs_queue[tail++] = box;
    influence_mask[box.x] |= (1 << box.y);
    while (head < tail) {
        Point curr = g_bfs_queue[head++];
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            Point next = {(uint8_t)(curr.x + DIRS[d][0]), (uint8_t)(curr.y + DIRS[d][1])};
            if (next.x >= MAP_ROWS || next.y >= MAP_COLS) continue;
            if (is_wall_bit(walls, next)) continue;
            { /* 炸弹也是障碍——箱子不能穿过炸弹(BOMB_INVALID=已消失) */
                bool blocked_by_bomb = false;
                for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
                    if (g_initial_bombs[bi].x == BOMB_INVALID) continue; /* 已使用的炸弹 */
                    if (g_initial_bombs[bi].x == next.x && g_initial_bombs[bi].y == next.y)
                        { blocked_by_bomb = true; break; }
                }
                if (blocked_by_bomb) continue;
            }
            if (g_dist_epoch_tag[next.x][next.y] == g_dist_epoch) continue;
            g_dist_epoch_tag[next.x][next.y] = g_dist_epoch;
            g_bfs_queue[tail++] = next;
            influence_mask[next.x] |= (1 << next.y);
        }
    }
}

/**
 * @brief 快速预检箱子是否能到达任意目标点
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
            { /* 炸弹也是障碍——箱子不能穿过炸弹(BOMB_INVALID=已消失) */
                bool blocked_by_bomb = false;
                for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
                    if (g_initial_bombs[bi].x == BOMB_INVALID) continue;
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
 * @brief 推箱BFS验证：固定箱子+固定目标，检查箱子能否被推到该目标
 * @return true  = 箱子可推到目标
 * @return false = 箱子无法推到目标
 */
static bool simulate_box_to_target(uint8_t box_idx, Point target,
                                    const uint16_t walls[MAP_ROWS],
                                    Point start_player, Point start_box) {
    /* 已在目标上 */
    if (start_box.x == target.x && start_box.y == target.y) return true;

    g_bfs_epoch++;
    if (g_bfs_epoch == 0) { memset(g_bfs_visited, 0, sizeof(g_bfs_visited)); g_bfs_epoch = 1; }

    /* 障碍物 = 墙 + 其他箱子（永久） */
    memset(g_obs_buf, 0, sizeof(g_obs_buf));
    for (uint8_t i = 0; i < MAP_ROWS; i++)
        g_obs_buf[i] = walls[i];
    for (uint8_t i = 0; i < g_box_count; i++) {
        if (i != box_idx)
            g_obs_buf[g_initial_boxes[i].x] |= (1 << g_initial_boxes[i].y);
    }
    /* 炸弹也是障碍——玩家不能穿过炸弹去推箱子 */
    for (uint8_t i = 0; i < g_bomb_count; i++) {
        if (g_initial_bombs[i].x != BOMB_INVALID)
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

        /* 命中目标 → 可达 */
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
                    if (g_initial_bombs[bi].x == BOMB_INVALID) continue; /* 已消失 */
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
 * @brief 统一死锁检测与待解决点生成
 * 
 * 对每个箱子做影响域分析：
 * - 封闭区域（玩家不可达）→ ENC
 * - 角落死锁（需推箱验证）→ SIM
 * 
 * 补充检查：目标不可达（无箱子能推入）→ UNR
 */
static void detect_and_generate_problems(void) {
    g_problem_count = 0;

    /* 1. 计算玩家可达区域（炸弹视为障碍，确保炸弹堵路时触发 PROBLEM_ENCLOSED） */
    compute_player_region_with_walls(g_static_walls, false);

    /* 1.5 预计算方向性宽松墙（玩家可达的可炸墙），供后续找围住墙使用 */
    compute_player_bomb_walls(g_tmp_walls1);  /* g_tmp_walls1 ← bomb_walls */

    /* 2. 对每个箱子做影响域分析 */
    bool box_handled[MAX_BOXES] = {0};
    for (uint8_t bi = 0; bi < g_box_count; bi++) {
        if (box_handled[bi]) continue;
        Point bp = g_initial_boxes[bi];
        uint16_t infl[MAP_ROWS];
        compute_box_influence(bp, g_static_walls, infl);
        bool player_can_reach = false;
        for (uint8_t ri = 0; ri < MAP_ROWS && !player_can_reach; ri++) {
            uint16_t row = infl[ri];
            for (uint8_t rj = 0; rj < MAP_COLS; rj++)
                if ((row & (1 << rj)) && g_player_region[ri][rj] == g_player_region_epoch) { player_can_reach = true; break; }
        }

        DeadlockProblemPoint *pp = &g_problem_points[g_problem_count];
        memset(pp, 0, sizeof(DeadlockProblemPoint));

        /* ── ENC 检测：玩家不可达（影响域与玩家区无交集）── */
        if (!player_can_reach) {
            pp->type = PROBLEM_ENCLOSED;
            memcpy(pp->influence_mask, infl, MAP_ROWS * sizeof(uint16_t));
            for (uint8_t bj = 0; bj < g_box_count; bj++) {
                Point bp2 = g_initial_boxes[bj];
                if (infl[bp2.x] & (1 << bp2.y)) { pp->box_indices[pp->box_count++] = bj; box_handled[bj] = true; }
            }
            for (uint8_t tj = 0; tj < g_target_count; tj++) {
                Point tp = g_initial_targets[tj];
                if (infl[tp.x] & (1 << tp.y)) pp->target_indices[pp->target_count++] = tj;
            }
            /* 找到围住该区域的墙（方向性：仅玩家可达的可炸墙） */
            pp->wall_count = find_enclosing_walls(infl, g_tmp_walls1, pp->enc_walls, MAX_ENC_WALLS);
            g_problem_count++;
        }

        /* ── SIM 检测：角落死锁（独立于 ENC，同一箱子可同时有 ENC+SIM）── */
        if (g_problem_count < MAX_PROBLEM_POINTS) {
            bool up_blk = false, down_blk = false, left_blk = false, right_blk = false;
            Point nb;
            nb = (Point){(uint8_t)(bp.x - 1), bp.y}; up_blk = (nb.x < MAP_ROWS && (g_static_walls[nb.x] & (1 << nb.y) || g_original_map[nb.x][nb.y] == BOX || g_original_map[nb.x][nb.y] == BOOM));
            nb = (Point){(uint8_t)(bp.x + 1), bp.y}; down_blk = (nb.x < MAP_ROWS && (g_static_walls[nb.x] & (1 << nb.y) || g_original_map[nb.x][nb.y] == BOX || g_original_map[nb.x][nb.y] == BOOM));
            nb = (Point){bp.x, (uint8_t)(bp.y - 1)}; left_blk = (nb.y < MAP_COLS && (g_static_walls[nb.x] & (1 << nb.y) || g_original_map[nb.x][nb.y] == BOX || g_original_map[nb.x][nb.y] == BOOM));
            nb = (Point){bp.x, (uint8_t)(bp.y + 1)}; right_blk = (nb.y < MAP_COLS && (g_static_walls[nb.x] & (1 << nb.y) || g_original_map[nb.x][nb.y] == BOX || g_original_map[nb.x][nb.y] == BOOM));
            bool corner_dead = (up_blk || down_blk) && (left_blk || right_blk);
            if (corner_dead) {
                DeadlockProblemPoint *sim_pp = &g_problem_points[g_problem_count];
                memset(sim_pp, 0, sizeof(DeadlockProblemPoint));
                sim_pp->type = PROBLEM_NEED_SIM;
                sim_pp->box_indices[0] = bi; sim_pp->box_count = 1;
                memcpy(sim_pp->influence_mask, infl, MAP_ROWS * sizeof(uint16_t));
                box_handled[bi] = true;
                g_problem_count++;
            }
        }
    }


    /* 3. 补充检查：目标在封闭区域但无箱子 */
    if (g_problem_count < MAX_PROBLEM_POINTS) {
        for (uint8_t ti = 0; ti < g_target_count; ti++) {
            Point tp = g_initial_targets[ti];
            bool covered = false;
            for (uint8_t pi = 0; pi < g_problem_count && !covered; pi++)
                for (uint8_t b = 0; b < g_problem_points[pi].target_count; b++)
                    if (g_problem_points[pi].target_indices[b] == ti) { covered = true; break; }
            if (covered) continue;
            if (g_player_region[tp.x][tp.y] == g_player_region_epoch)
                continue;

            /* ★ 构建"墙+箱子"位图：目标ENC检测中箱子视为障碍 */
            memcpy(g_tmp_walls2, g_static_walls, sizeof(g_tmp_walls2));
            for (uint8_t bj = 0; bj < g_box_count; bj++) {
                Point bp = g_initial_boxes[bj];
                g_tmp_walls2[bp.x] |= (1 << bp.y);
            }

            compute_box_influence(tp, g_tmp_walls2, g_tmp_infl);  /* g_tmp_infl ← catchment */
            bool has_box = false;
            for (uint8_t bj = 0; bj < g_box_count && !has_box; bj++) {
                Point bp = g_initial_boxes[bj];
                if (g_tmp_infl[bp.x] & (1 << bp.y)) has_box = true;
            }
            DeadlockProblemPoint *pp = &g_problem_points[g_problem_count];
            memset(pp, 0, sizeof(DeadlockProblemPoint));
            pp->type = PROBLEM_ENCLOSED;
            memcpy(pp->influence_mask, g_tmp_infl, MAP_ROWS * sizeof(uint16_t));
            for (uint8_t tj = 0; tj < g_target_count; tj++) {
                Point tp2 = g_initial_targets[tj];
                if (g_tmp_infl[tp2.x] & (1 << tp2.y)) pp->target_indices[pp->target_count++] = tj;
            }
            if (has_box) {
                for (uint8_t bj = 0; bj < g_box_count; bj++) {
                    Point bp = g_initial_boxes[bj];
                    if (g_tmp_infl[bp.x] & (1 << bp.y)) pp->box_indices[pp->box_count++] = bj;
                }
            }
            pp->wall_count = find_enclosing_walls(g_tmp_infl, g_tmp_walls1, pp->enc_walls, MAX_ENC_WALLS);
            g_problem_count++;
            if (g_problem_count >= MAX_PROBLEM_POINTS) break;
        }
    }

    /* ── 合并被同一堵可炸墙分隔的 ENC 问题 ── */
    for (uint8_t pi = 0; pi < g_problem_count; pi++) {
        if (!(g_problem_points[pi].type & PROBLEM_ENCLOSED)) continue;
        for (uint8_t pj = (uint8_t)(pi + 1); pj < g_problem_count; pj++) {
            if (!(g_problem_points[pj].type & PROBLEM_ENCLOSED)) continue;
            /* 用两个ENC的围住墙并集做松弛墙，检查两区域是否连通 */
            bool adjacent = false;
            {
                /* 构建并集墙：pi墙 + pj墙，去重 */
                uint16_t merge_walls[MAP_ROWS];
                memcpy(merge_walls, g_static_walls, sizeof(merge_walls));
                for (uint8_t w = 0; w < g_problem_points[pi].wall_count; w++) {
                    Point wp = g_problem_points[pi].enc_walls[w];
                    merge_walls[wp.x] &= (uint16_t)~(1 << wp.y);
                }
                for (uint8_t w = 0; w < g_problem_points[pj].wall_count; w++) {
                    Point wp = g_problem_points[pj].enc_walls[w];
                    merge_walls[wp.x] &= (uint16_t)~(1 << wp.y);
                }
                uint16_t merged_infl[MAP_ROWS];
                Point seed = (g_problem_points[pi].box_count > 0)
                    ? g_initial_boxes[g_problem_points[pi].box_indices[0]]
                    : g_initial_targets[g_problem_points[pi].target_indices[0]];
                compute_box_influence(seed, merge_walls, merged_infl);
                Point pj_seed = (g_problem_points[pj].box_count > 0)
                    ? g_initial_boxes[g_problem_points[pj].box_indices[0]]
                    : g_initial_targets[g_problem_points[pj].target_indices[0]];
                if (merged_infl[pj_seed.x] & (1 << pj_seed.y))
                    adjacent = true;
            }
            if (!adjacent) continue;
            /* 合并 pj → pi */
            for (uint8_t b = 0; b < g_problem_points[pj].box_count; b++) {
                uint8_t bi = g_problem_points[pj].box_indices[b];
                bool dup = false;
                for (uint8_t k = 0; k < g_problem_points[pi].box_count; k++)
                    if (g_problem_points[pi].box_indices[k] == bi) { dup = true; break; }
                if (!dup && g_problem_points[pi].box_count < MAX_BOXES)
                    g_problem_points[pi].box_indices[g_problem_points[pi].box_count++] = bi;
            }
            for (uint8_t t = 0; t < g_problem_points[pj].target_count; t++) {
                uint8_t ti = g_problem_points[pj].target_indices[t];
                bool dup = false;
                for (uint8_t k = 0; k < g_problem_points[pi].target_count; k++)
                    if (g_problem_points[pi].target_indices[k] == ti) { dup = true; break; }
                if (!dup && g_problem_points[pi].target_count < MAX_BOXES)
                    g_problem_points[pi].target_indices[g_problem_points[pi].target_count++] = ti;
            }
            for (uint8_t r = 0; r < MAP_ROWS; r++)
                g_problem_points[pi].influence_mask[r] |= g_problem_points[pj].influence_mask[r];
            /* 合并墙集 */
            for (uint8_t w = 0; w < g_problem_points[pj].wall_count; w++) {
                Point wp = g_problem_points[pj].enc_walls[w];
                bool dup = false;
                for (uint8_t k = 0; k < g_problem_points[pi].wall_count; k++)
                    if (pos_equal(g_problem_points[pi].enc_walls[k], wp)) { dup = true; break; }
                if (!dup && g_problem_points[pi].wall_count < MAX_ENC_WALLS)
                    g_problem_points[pi].enc_walls[g_problem_points[pi].wall_count++] = wp;
            }
            /* 移除 pj */
            g_problem_points[pj] = g_problem_points[g_problem_count - 1];
            g_problem_count--;
            pj--;
        }
    }


    /* 4. 目标反向可达性检查：未被任何死锁覆盖的目标，验证是否有箱子能推进来 */
    /* 标记SIM死锁箱子（角落卡死无法推动），验证目标时跳过它们 */
    bool box_is_sim[MAX_BOXES] = {false};
    for (uint8_t pi = 0; pi < g_problem_count; pi++)
        if (g_problem_points[pi].type & PROBLEM_NEED_SIM)
            for (uint8_t b = 0; b < g_problem_points[pi].box_count; b++)
                box_is_sim[g_problem_points[pi].box_indices[b]] = true;

    if (g_problem_count < MAX_PROBLEM_POINTS && g_box_count > 0) {
        /* 收集已覆盖的目标索引 */
        bool target_covered[MAX_BOXES] = {false};
        for (uint8_t pi = 0; pi < g_problem_count; pi++) {
            for (uint8_t t = 0; t < g_problem_points[pi].target_count; t++)
                target_covered[g_problem_points[pi].target_indices[t]] = true;
        }
        /* 对未被覆盖的目标，按到最近箱子的曼哈顿距离排序（近的优先验证） */
        uint8_t uncov[MAX_BOXES], uncov_cnt = 0;
        uint16_t uncov_dist[MAX_BOXES];
        for (uint8_t ti = 0; ti < g_target_count; ti++) {
            if (target_covered[ti]) continue;
            /* 几何预筛：目标是否至少有一个合法推入方向？ */
            Point tp = g_initial_targets[ti];
            bool has_push_dir = false;
            for (uint8_t d = 0; d < DIR_COUNT && !has_push_dir; d++) {
                /* 推入方向：箱子从 d 方向被推进来，玩家站在对面 */
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
                /* 几何上完全无法推入 → 直接标记为死锁目标 */
                DeadlockProblemPoint *pp = &g_problem_points[g_problem_count];
                memset(pp, 0, sizeof(DeadlockProblemPoint));
                pp->type = PROBLEM_TARGET_UNREACHABLE;
                pp->target_indices[0] = ti; pp->target_count = 1;
                uint16_t infl[MAP_ROWS];
                compute_box_influence(tp, g_static_walls, infl);
                memcpy(pp->influence_mask, infl, MAP_ROWS * sizeof(uint16_t));
                g_problem_count++;
                target_covered[ti] = true;
                if (g_problem_count >= MAX_PROBLEM_POINTS) break;
                continue;
            }
            /* 计算最近箱子的距离 */
            uint16_t min_d = INF;
            for (uint8_t bi = 0; bi < g_box_count; bi++) {
                uint16_t d = manhattan_distance(g_initial_boxes[bi], tp);
                if (d < min_d) min_d = d;
            }
            uncov[uncov_cnt] = ti;
            uncov_dist[uncov_cnt] = min_d;
            uncov_cnt++;
        }
        /* 按距离排序（近的优先） */
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

        /* 推箱BFS验证：对每个未被覆盖的目标，检查是否有箱子能推进来 */
        for (uint8_t ui = 0; ui < uncov_cnt; ui++) {
            if (g_problem_count >= MAX_PROBLEM_POINTS) break;
            uint8_t ti = uncov[ui];
            if (target_covered[ti]) continue;
            Point tp = g_initial_targets[ti];

            /* 按距离排序箱子（近的优先尝试） */
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
                if (box_is_sim[box_idx]) continue;  /* 死锁箱子跳过 */
                Point bp = g_initial_boxes[box_idx];
                /* 快速预检：普通 BFS 无视推箱约束，若直线路径都不通则绝无可能推到 */
                bfs_compute_distances(bp, g_static_walls);
                if (g_dist_map[tp.x][tp.y] == INF) continue;
                if (simulate_box_to_target(box_idx, tp, g_static_walls,
                                            g_initial_player, bp)) {
                    any_box_reaches = true; break;
                }
            }

            if (!any_box_reaches) {
                DeadlockProblemPoint *pp = &g_problem_points[g_problem_count];
                memset(pp, 0, sizeof(DeadlockProblemPoint));
                pp->type = PROBLEM_TARGET_UNREACHABLE;
                pp->target_indices[0] = ti; pp->target_count = 1;
                uint16_t infl[MAP_ROWS];
                compute_box_influence(tp, g_static_walls, infl);
                memcpy(pp->influence_mask, infl, MAP_ROWS * sizeof(uint16_t));
                g_problem_count++;
            }
        }

        /* 补充：对 ENC 内的目标用本ENC围住墙做宽松验证（仅清除本ENC的墙） */
        for (uint8_t pi = 0; pi < g_problem_count; pi++) {
            if (!(g_problem_points[pi].type & PROBLEM_ENCLOSED)) continue;
            if (g_problem_count >= MAX_PROBLEM_POINTS) break;
            /* 构建本ENC的宽松墙：仅清除本ENC的围住墙 */
            memcpy(g_tmp_walls1, g_static_walls, sizeof(g_tmp_walls1));
            for (uint8_t w = 0; w < g_problem_points[pi].wall_count; w++) {
                Point wp = g_problem_points[pi].enc_walls[w];
                g_tmp_walls1[wp.x] &= (uint16_t)~(1 << wp.y);
            }
            for (uint8_t t = 0; t < g_problem_points[pi].target_count; t++) {
                uint8_t ti = g_problem_points[pi].target_indices[t];
                Point tp = g_initial_targets[ti];

                /* 宽松墙下是否可达：检查全图所有箱子（炸墙后外部箱子也能推进来） */
                bool any_reaches = false;
                for (uint8_t bj = 0; bj < g_box_count; bj++) {
                    if (box_is_sim[bj]) continue;  /* 死锁箱子跳过 */
                    Point bp = g_initial_boxes[bj];
                    bfs_compute_distances(bp, g_tmp_walls1);
                    if (g_dist_map[tp.x][tp.y] == INF) continue;
                    if (simulate_box_to_target(bj, tp, g_tmp_walls1,
                                                g_initial_player, bp)) {
                        any_reaches = true; break;
                    }
                }

                if (!any_reaches) {
                    /* ENC 目标即使炸墙也无法推入 → 独立 UNR */
                    DeadlockProblemPoint *pp = &g_problem_points[g_problem_count];
                    memset(pp, 0, sizeof(DeadlockProblemPoint));
                    pp->type = PROBLEM_TARGET_UNREACHABLE;
                    pp->target_indices[0] = ti; pp->target_count = 1;
                    compute_box_influence(tp, g_tmp_walls1, g_tmp_infl);
                    memcpy(pp->influence_mask, g_tmp_infl, MAP_ROWS * sizeof(uint16_t));
                    g_problem_count++;
                    if (g_problem_count >= MAX_PROBLEM_POINTS) break;
                }
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
 * @brief 增量死锁验证（验证修改墙后问题是否解决）
 */
static uint16_t check_problems_resolved_incremental(const uint16_t modified_walls[MAP_ROWS], Point player_pos) {
    if (g_saved_problem_count == 0) return 0xFFFF;
    memcpy(g_saved_walls, g_static_walls, sizeof(g_static_walls));
    memcpy(g_saved_player_region, g_player_region, sizeof(g_player_region));
    uint8_t saved_epoch = g_player_region_epoch;
    memcpy(g_static_walls, modified_walls, sizeof(g_static_walls));
    compute_player_region_with_walls(modified_walls, false);
    uint16_t mask = 0;
    for (uint8_t pi = 0; pi < g_saved_problem_count; pi++) {
        const DeadlockProblemPoint *pp = &g_saved_problem_points[pi];
        bool resolved = true;
        if (pp->type & PROBLEM_ENCLOSED) {
            bool reachable = false;
            /* 用 modified_walls+箱子 重新计算影响域（与检测阶段一致，箱子=障碍） */
            {
                memcpy(g_tmp_walls1, modified_walls, sizeof(g_tmp_walls1));
                for (uint8_t bj = 0; bj < g_box_count; bj++) {
                    Point bp = g_initial_boxes[bj];
                    g_tmp_walls1[bp.x] |= (1 << bp.y);
                }
                Point seed = (pp->box_count > 0) ? g_initial_boxes[pp->box_indices[0]]
                            : g_initial_targets[pp->target_indices[0]];
                compute_box_influence(seed, g_tmp_walls1, g_tmp_infl);
                for (uint8_t ri = 0; ri < MAP_ROWS && !reachable; ri++) {
                    uint16_t row = g_tmp_infl[ri] & ~g_tmp_walls1[ri];
                    for (uint8_t rj = 0; rj < MAP_COLS; rj++)
                        if ((row & (1 << rj)) && g_player_region[ri][rj] == g_player_region_epoch) { reachable = true; break; }
                }
            }
            if (!reachable) resolved = false;
            if (resolved && pp->box_count > 0) {
                for (uint8_t b = 0; b < pp->box_count && resolved; b++) {
                    uint8_t box_idx = pp->box_indices[b];
                    Point bp = g_initial_boxes[box_idx];
                    if (!box_can_reach_any_target(bp, modified_walls, g_initial_targets, g_target_count))
                        resolved = false;
                }
            }
        }
        if (pp->type & PROBLEM_NEED_SIM) {
            uint8_t box_idx = pp->box_indices[0];
            Point bp = g_initial_boxes[box_idx];
            /* 角落死锁解除：检查炸墙后是否仍被水平+垂直双向堵死 */
            bool h_blocked = false, v_blocked = false;
            for (uint8_t d = 0; d < DIR_COUNT; d++) {
                uint8_t nx = (uint8_t)(bp.x + DIRS[d][0]), ny = (uint8_t)(bp.y + DIRS[d][1]);
                bool blocked = false;
                if (nx >= MAP_ROWS || ny >= MAP_COLS) blocked = true;
                else if (modified_walls[nx] & (1 << ny)) blocked = true;
                else {
                    for (uint8_t bj = 0; bj < g_box_count && !blocked; bj++)
                        if (g_initial_boxes[bj].x == nx && g_initial_boxes[bj].y == ny) blocked = true;
                    /* ★ 活炸弹也会阻挡箱子（非卡死炸弹，卡死的已在modified_walls中） */
                    for (uint8_t bj = 0; bj < g_bomb_count && !blocked; bj++)
                        if (g_initial_bombs[bj].x == nx && g_initial_bombs[bj].y == ny) blocked = true;
                }
                if (blocked) { if (d < 2) v_blocked = true; else h_blocked = true; }
            }
            if (h_blocked && v_blocked) resolved = false;
        }
        if (pp->type & PROBLEM_TARGET_UNREACHABLE) {
            /* 目标不可达：验证是否有箱子能推到该目标 */
            /* ★ 优化：墙只减不增，上次成功的箱子本次仍可达 → 跳过A*扫描 */
            uint8_t ti = pp->target_indices[0];
            Point tp = g_initial_targets[ti];
            bool any_box_reaches = false;
            int8_t cached = g_unr_reach_cache[pi];
            if (cached >= 0 && (uint8_t)cached < g_box_count) {
                /* 快速路径：只检查上次成功的箱子 */
                Point bp = g_initial_boxes[(uint8_t)cached];
                if (simulate_box_to_target((uint8_t)cached, tp, modified_walls,
                                            player_pos, bp)) {
                    any_box_reaches = true;
                } else {
                    /* 缓存失效（箱子被移走等极端情况），回退全扫描 */
                    cached = -1;
                }
            }
            if (!any_box_reaches) {
                for (uint8_t bi = 0; bi < g_box_count; bi++) {
                    Point bp = g_initial_boxes[bi];
                    if (simulate_box_to_target(bi, tp, modified_walls,
                                                player_pos, bp)) {
                        any_box_reaches = true;
                        g_unr_reach_cache[pi] = (int8_t)bi;  /* 记住成功的箱子 */
                        break;
                    }
                }
            }
            if (!any_box_reaches) resolved = false;
        }
        if (resolved) mask |= (1 << pi);
    }
    memcpy(g_static_walls, g_saved_walls, sizeof(g_static_walls));
    memcpy(g_player_region, g_saved_player_region, sizeof(g_player_region));
    g_player_region_epoch = saved_epoch;
    return mask;
}

/**
 * @brief 带缓存的增量验证
 */
static uint16_t check_problems_resolved_cached(const uint16_t modified_walls[MAP_ROWS], Point player_pos) {
    uint32_t h = hash_walls(modified_walls);
    uint16_t idx = (uint16_t)(h % VCACHE_SIZE);
    if (g_vcache_hash[idx] == h) return g_vcache_mask[idx];
    uint16_t mask = check_problems_resolved_incremental(modified_walls, player_pos);
    g_vcache_hash[idx] = h; g_vcache_mask[idx] = mask;
    return mask;
}

/**
 * @brief 在自定义墙位图上计算指定炸弹的可达区域
 *
 * @param bi          炸弹索引
 * @param bomb_pos    炸弹位置
 * @param walls       自定义墙位图
 * @param active      活跃炸弹标记（true=存在，false=已消失）
 * @param out_reach   输出可达区域位图
 */
static void compute_one_bomb_reachability(uint8_t bi, Point bomb_pos,
    const uint16_t walls[MAP_ROWS], const bool active[],
    uint16_t out_reach[MAP_ROWS]) {
    g_vis1_epoch++;
    if (g_vis1_epoch == 0) {
        memset(g_vis1, 0, sizeof(g_vis1));
        g_vis1_epoch = 1;
    }
    memset(g_obs_buf, 0, sizeof(g_obs_buf));

    /* 构建障碍物：墙 + 箱子 + 其他活跃炸弹 */
    for (uint8_t i = 0; i < MAP_ROWS; i++) {
        g_obs_buf[i] = walls[i];
    }
    for (uint8_t i = 0; i < g_box_count; i++) {
        Point bp = g_initial_boxes[i];
        g_obs_buf[bp.x] |= (1 << bp.y);
    }
    for (uint8_t i = 0; i < g_bomb_count; i++) {
        if (i == bi) continue;
        if (!active[i]) continue;
        Point bp = g_initial_bombs[i];
        if (bp.x == BOMB_INVALID) continue;
        g_obs_buf[bp.x] |= (1 << bp.y);
    }

    /* 推动感知BFS：炸弹只能被玩家推动，需要目标格和玩家格都为空 */
    uint8_t head = 0, tail = 0;
    g_vis1[bomb_pos.x][bomb_pos.y] = g_vis1_epoch;
    g_bfs_queue[tail++] = bomb_pos;

    while (head < tail) {
        Point curr = g_bfs_queue[head++];
        out_reach[curr.x] |= (1 << curr.y);
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            /* 炸弹被推向 DIRS[d] 方向：目标格 */
            Point next = {
                (uint8_t)(curr.x + DIRS[d][0]),
                (uint8_t)(curr.y + DIRS[d][1])
            };
            /* 玩家站在炸弹反方向推动 */
            Point player_stand = {
                (uint8_t)(curr.x - DIRS[d][0]),
                (uint8_t)(curr.y - DIRS[d][1])
            };
            if (next.x >= MAP_ROWS || next.y >= MAP_COLS) continue;
            if (player_stand.x >= MAP_ROWS || player_stand.y >= MAP_COLS) continue;
            if (g_vis1[next.x][next.y] == g_vis1_epoch) continue;
            /* 目标格不能有障碍物 */
            if (g_obs_buf[next.x] & (1 << next.y)) continue;
            /* 玩家站位不能有障碍物（玩家必须能站在那里推） */
            if (g_obs_buf[player_stand.x] & (1 << player_stand.y)) continue;
            /* 玩家站位不能是炸弹自身（防止死循环，炸弹不能推自己） */
            if (player_stand.x == curr.x && player_stand.y == curr.y) continue;
            g_vis1[next.x][next.y] = g_vis1_epoch;
            g_bfs_queue[tail++] = next;
        }
    }
}

/**
 * @brief 在自定义墙位图上预计算所有活跃炸弹的可达区域
 *
 * @param walls         自定义墙位图
 * @param active_bombs  活跃炸弹标记数组（长度 MAX_BOOMS）
 * @param out_reach_map 输出可达区域（每个炸弹一行位图）
 */
static void compute_bomb_reachability(const uint16_t walls[MAP_ROWS],
    const bool active_bombs[MAX_BOOMS],
    uint16_t out_reach_map[MAX_BOOMS][MAP_ROWS]) {
    memset(out_reach_map, 0, sizeof(uint16_t) * MAX_BOOMS * MAP_ROWS);
    for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
        if (!active_bombs[bi]) continue;
        Point bomb_pos = g_initial_bombs[bi];
        if (bomb_pos.x == BOMB_INVALID) continue;
        compute_one_bomb_reachability(bi, bomb_pos, walls, active_bombs, out_reach_map[bi]);
    }
}

/**
 * @brief 获取目标墙的爆炸点候选位置（自定义墙位图版）
 */
static void find_detonation_points_for_wall_on(Point target_wall, const uint16_t walls[MAP_ROWS],
    Point out_points[], uint8_t *out_count) {
    *out_count = 0;
    for (int8_t dx = -1; dx <= 1; dx++) for (int8_t dy = -1; dy <= 1; dy++) {
        Point dp = {(uint8_t)(target_wall.x + dx), (uint8_t)(target_wall.y + dy)};
        if (dp.x >= MAP_ROWS || dp.y >= MAP_COLS) continue;
        if (!is_valid_detonation_point_on(dp, walls)) continue;
        out_points[(*out_count)++] = dp;
        if (*out_count >= MAX_DETONATE_POINTS) return;
    }
}

/**
 * @brief 在封闭区域周围查找可炸墙（自定义墙位图版）
 */
static void find_walls_for_enclosed_on(const uint16_t region_mask[MAP_ROWS],
    const uint16_t walls[MAP_ROWS],
    BreakableWall out_walls[], uint8_t *out_count) {
    for (uint8_t i = 0; i < MAP_ROWS; i++) {
        for (uint8_t j = 0; j < MAP_COLS; j++) {
            Point p = {i, j};
            if (!is_breakable_wall_on(p, walls)) continue;
            bool touches_influence = false, touches_player = false;
            for (uint8_t d = 0; d < DIR_COUNT; d++) {
                uint8_t ni = (uint8_t)(i + DIRS[d][0]);
                uint8_t nj = (uint8_t)(j + DIRS[d][1]);
                if (ni >= MAP_ROWS || nj >= MAP_COLS) continue;
                if (is_wall_bit(walls, (Point){ni, nj})) continue;
                if (region_mask[ni] & (1 << nj)) touches_influence = true;
                if (g_player_region[ni][nj] == g_player_region_epoch) touches_player = true;
            }
            /* ★ 纳入所有邻接玩家区的可炸墙；同时邻接影响域的高优先 */
            if (touches_player) {
                bool found = false;
                for (uint8_t k = 0; k < *out_count; k++)
                    if (pos_equal(out_walls[k].wall_pos, p)) { found = true; break; }
                if (!found && *out_count < MAX_BREAK_WALLS) {
                    out_walls[*out_count].wall_pos = p;
                    out_walls[*out_count].benefit_score = touches_influence ? -50 : -5;
                    (*out_count)++;
                }
            }
        }
    }
}

/**
 * @brief 在分隔死锁区域查找可炸墙（自定义墙位图版）
 */
static void find_walls_for_separated_on(const uint16_t influence[MAP_ROWS],
    const uint16_t walls[MAP_ROWS],
    BreakableWall out_walls[], uint8_t *out_count) {
    uint8_t start = *out_count;
    for (uint8_t i = 0; i < MAP_ROWS; i++) for (uint8_t j = 0; j < MAP_COLS; j++) {
        Point p = {i, j}; if (!is_breakable_wall_on(p, walls)) continue;
        bool touches_influence = false, touches_player = false;
        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            uint8_t ni = (uint8_t)(i + DIRS[d][0]), nj = (uint8_t)(j + DIRS[d][1]);
            if (ni >= MAP_ROWS || nj >= MAP_COLS) continue;
            if (is_wall_bit(walls, (Point){ni, nj})) continue;
            if (influence[ni] & (1 << nj)) touches_influence = true;
            if (g_player_region[ni][nj] == g_player_region_epoch) touches_player = true;
        }
        if (touches_influence && touches_player && *out_count < MAX_BREAK_WALLS) {
            /* 跳过已在列表中的墙 */
            bool dup = false;
            for (uint8_t k = 0; k < start; k++)
                if (pos_equal(out_walls[k].wall_pos, p)) { dup = true; break; }
            for (uint8_t k = start; k < *out_count; k++)
                if (pos_equal(out_walls[k].wall_pos, p)) { dup = true; break; }
            if (!dup) {
                out_walls[*out_count].wall_pos = p;
                out_walls[*out_count].benefit_score = -5; (*out_count)++;
            }
        }
    }
}

/**
 * @brief 为指定死锁问题收集候选可炸墙（追加到列表，自动去重）
 */
static void collect_walls_for_problem(uint8_t pi, const uint16_t cur_walls[MAP_ROWS],
                                       BreakableWall walls[], uint8_t *wall_count,
                                        uint8_t max_walls, const bool active_bombs[MAX_BOOMS]) {
    const DeadlockProblemPoint *pp = &g_saved_problem_points[pi];
    BreakableWall local[MAX_BREAK_WALLS];
    uint8_t local_cnt = 0;

    if (pp->type & PROBLEM_ENCLOSED)
        find_walls_for_enclosed_on(pp->influence_mask, cur_walls, local, &local_cnt);
    if (pp->type & PROBLEM_NEED_SIM) {
        /* 构建含活炸弹的墙位图：活炸弹也会阻挡箱子造成角落死锁 */
        uint16_t walls_with_bombs[MAP_ROWS];
        memcpy(walls_with_bombs, cur_walls, sizeof(uint16_t) * MAP_ROWS);
        for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
            if (!active_bombs[bi]) continue;
            Point bp = g_initial_bombs[bi];
            if (bp.x == BOMB_INVALID) continue;
            walls_with_bombs[bp.x] |= (1 << bp.y);
        }
        BreakableWall bw[MAX_BREAK_WALLS]; uint8_t bwc = 0;
        for (uint8_t bi = 0; bi < pp->box_count; bi++) {
            Point box = g_initial_boxes[pp->box_indices[bi]];
            bool corner = is_corner_deadlock(box, walls_with_bombs);
            for (uint8_t d = 0; d < DIR_COUNT; d++) {
                Point wp = {(uint8_t)(box.x + DIRS[d][0]), (uint8_t)(box.y + DIRS[d][1])};
                if (wp.x >= MAP_ROWS || wp.y >= MAP_COLS) continue;
                if (!is_wall_bit(cur_walls, wp)) continue;
                if (wp.x == 0 || wp.x == MAP_ROWS-1 || wp.y == 0 || wp.y == MAP_COLS-1) continue;
                bool found = false;
                for (uint8_t k = 0; k < bwc; k++) if (pos_equal(bw[k].wall_pos, wp)) { found = true; break; }
                if (!found && bwc < MAX_BREAK_WALLS) {
                    bw[bwc].wall_pos = wp; bw[bwc].benefit_score = corner ? -50 : -5; bwc++;
                }
            }
        }
        if (bwc < 5)
            find_walls_for_separated_on(pp->influence_mask, cur_walls, bw, &bwc);
        find_walls_for_enclosed_on(pp->influence_mask, cur_walls, bw, &bwc);
        for (uint8_t wi = 0; wi < bwc; wi++) {
            bool found = false;
            for (uint8_t k = 0; k < local_cnt; k++) if (pos_equal(local[k].wall_pos, bw[wi].wall_pos)) { found = true; break; }
            if (!found && local_cnt < MAX_BREAK_WALLS) local[local_cnt++] = bw[wi];
        }
    }
    if (pp->type & PROBLEM_TARGET_UNREACHABLE) {
        find_walls_for_enclosed_on(pp->influence_mask, cur_walls, local, &local_cnt);
        if (local_cnt == 0)
            find_walls_for_separated_on(pp->influence_mask, cur_walls, local, &local_cnt);
        if (local_cnt == 0) {
            uint8_t ti = pp->target_indices[0];
            Point tp = g_initial_targets[ti];
            for (uint8_t d = 0; d < DIR_COUNT; d++) {
                Point wp = {(uint8_t)(tp.x + DIRS[d][0]), (uint8_t)(tp.y + DIRS[d][1])};
                if (wp.x >= MAP_ROWS || wp.y >= MAP_COLS) continue;
                if (!is_breakable_wall_on(wp, cur_walls)) continue;
                bool dup = false;
                for (uint8_t k = 0; k < local_cnt; k++) if (pos_equal(local[k].wall_pos, wp)) { dup = true; break; }
                if (!dup && local_cnt < MAX_BREAK_WALLS) { local[local_cnt].wall_pos = wp; local[local_cnt].benefit_score = -5; local_cnt++; }
            }
        }
    }

    /* 追加到主列表（去重） */
    for (uint8_t wi = 0; wi < local_cnt && *wall_count < max_walls; wi++) {
        bool dup = false;
        for (uint8_t k = 0; k < *wall_count; k++)
            if (pos_equal(walls[k].wall_pos, local[wi].wall_pos)) { dup = true; break; }
        if (!dup) walls[(*wall_count)++] = local[wi];
    }
}

/**
 * @brief 检查炸弹是否有至少一个方向可被玩家推动
 */
static bool can_push_bomb(uint8_t bomb_index, Point bomb_pos,
                            const uint16_t walls[MAP_ROWS], const bool active[]) {
    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        uint8_t nx = (uint8_t)(bomb_pos.x + DIRS[d][0]);
        uint8_t ny = (uint8_t)(bomb_pos.y + DIRS[d][1]);
        uint8_t px = (uint8_t)(bomb_pos.x - DIRS[d][0]);
        uint8_t py = (uint8_t)(bomb_pos.y - DIRS[d][1]);
        if (nx >= MAP_ROWS || ny >= MAP_COLS || px >= MAP_ROWS || py >= MAP_COLS) continue;
        if (is_wall_bit(walls, (Point){px, py})) continue;
        bool blocked = false;
        for (uint8_t bj = 0; bj < g_box_count && !blocked; bj++)
            if ((g_initial_boxes[bj].x == nx && g_initial_boxes[bj].y == ny) ||
                (g_initial_boxes[bj].x == px && g_initial_boxes[bj].y == py)) blocked = true;
        for (uint8_t bj = 0; bj < g_bomb_count && !blocked; bj++) {
            if (bj == bomb_index) continue;
            if (!active[bj]) continue;
            if (g_initial_bombs[bj].x == BOMB_INVALID) continue;
            if ((g_initial_bombs[bj].x == nx && g_initial_bombs[bj].y == ny) ||
                (g_initial_bombs[bj].x == px && g_initial_bombs[bj].y == py)) blocked = true;
        }
        if (!blocked) return true;
    }
    return false;
}

/**
 * @brief 为可炸墙生成爆炸方案（自定义墙位图/可达图/玩家版）
 *
 * @param breakable_walls  可炸墙列表
 * @param wall_count       可炸墙数量
 * @param walls            自定义墙位图
 * @param bomb_reach_map   自定义炸弹可达图
 * @param player_pos       玩家当前位置
 * @param active_bombs     活跃炸弹标记
 * @param out_plans        输出方案
 * @param out_plan_count   输出方案数
 */
static void generate_plans_for_walls_on(const BreakableWall breakable_walls[], uint8_t wall_count,
    const uint16_t walls[MAP_ROWS],
    const uint16_t bomb_reach_map[MAX_BOOMS][MAP_ROWS],
    Point player_pos, const bool active_bombs[MAX_BOOMS],
    DetonatePlan out_plans[], uint8_t *out_plan_count) {
    if (wall_count == 0 || g_bomb_count == 0) return;
    memcpy(g_sim_walls, walls, sizeof(g_sim_walls));  /* 一次性初始化 */
    for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
        if (!active_bombs[bi]) continue;
        Point bomb_pos = g_initial_bombs[bi];
        if (bomb_pos.x == BOMB_INVALID) continue;
        for (uint8_t wi = 0; wi < wall_count; wi++) {
            Point wall_pos = breakable_walls[wi].wall_pos;
            Point candidates[8]; uint8_t cand_count = 0;
            find_detonation_points_for_wall_on(wall_pos, walls, candidates, &cand_count);
            for (uint8_t ci = 0; ci < cand_count; ci++) {
                Point dp = candidates[ci];
                bool bomb_can_reach = false;
                for (uint8_t d = 0; d < DIR_COUNT && !bomb_can_reach; d++) {
                    uint8_t adj_x = (uint8_t)(dp.x + DIRS[d][0]);
                    uint8_t adj_y = (uint8_t)(dp.y + DIRS[d][1]);
                    if (adj_x >= MAP_ROWS || adj_y >= MAP_COLS) continue;
                    if (is_wall_bit(walls, (Point){adj_x, adj_y})) continue;
                    if (bomb_reach_map[bi][adj_x] & (1 << adj_y))
                        bomb_can_reach = true;
                }
                if (!bomb_can_reach) continue;
                Point covered_walls[9]; uint8_t covered_count = 0;
                for (uint8_t wj = 0; wj < wall_count; wj++)
                    if (can_explosion_cover_wall(dp, breakable_walls[wj].wall_pos))
                        covered_walls[covered_count++] = breakable_walls[wj].wall_pos;
                if (covered_count == 0) continue;

                /* 局部清除覆盖墙（sim_walls 在外层已初始化为 walls 副本） */
                for (uint8_t k = 0; k < covered_count; k++) {
                    Point wp = covered_walls[k];
                    g_sim_walls[wp.x] &= (uint16_t)~(1 << wp.y);
                }
                uint16_t resolved_mask = 0;
                bool resolves = false;
                /* ★ 不再预判 walls_may_help：即使爆炸不直接邻接影响域，
                 * 炸墙后扩大玩家区/改变地形，可能帮助后续迭代。
                 * 统一走增量验证判断实际效果。 */
                {
                    resolved_mask = check_problems_resolved_cached(g_sim_walls, player_pos);
                    resolves = (resolved_mask == (uint16_t)((1 << g_saved_problem_count) - 1));
                }
                /* 恢复 sim_walls */
                for (uint8_t k = 0; k < covered_count; k++) {
                    Point wp = covered_walls[k];
                    g_sim_walls[wp.x] = walls[wp.x];  /* 还原整行 */
                }
                uint16_t push_dist = estimate_bomb_push_distance_on(bomb_pos, dp, player_pos, walls, active_bombs);
                int total_benefit = (int)push_dist;
                for (uint8_t k = 0; k < covered_count; k++)
                    for (uint8_t wj = 0; wj < wall_count; wj++)
                        if (pos_equal(covered_walls[k], breakable_walls[wj].wall_pos))
                            { total_benefit += breakable_walls[wj].benefit_score; break; }
                {
                    int target_bonus = 0;
                    for (uint8_t pi = 0; pi < g_saved_problem_count; pi++)
                        if (resolved_mask & (1 << pi))
                            target_bonus += g_saved_problem_points[pi].target_count;
                    total_benefit -= target_bonus * 10;
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
                        out_plans[dupe_idx].resolved_mask = resolved_mask;
                    } continue;
                }
                if (*out_plan_count >= MAX_DETONATE_POINTS) return;
                if (!can_push_bomb(bi, bomb_pos, walls, active_bombs)) continue;
                DetonatePlan *plan = &out_plans[*out_plan_count];
                plan->detonate_pos = dp; plan->bomb_index = bi; plan->bomb_initial_pos = bomb_pos;
                plan->bomb_push_distance = push_dist; plan->wall_count = covered_count;
                memcpy(plan->walls_covered, covered_walls, covered_count * sizeof(Point));
                plan->total_benefit = total_benefit;
                plan->resolved_mask = resolved_mask;
                (*out_plan_count)++;
                if (*out_plan_count >= MAX_DETONATE_POINTS) return;
            }
        }
    }
}

/**
 * @brief 按收益排序方案（插入排序，小值优先）
 */
static void sort_plans_by_benefit(DetonatePlan plans[], uint8_t count) {
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
 * @brief 验证多炸弹组合是否存在一个可执行的顺序
 *
 * 对方案组合尝试所有排列顺序，在每个排列中依次验证：
 * 1. 当前炸弹在更新后的墙位图上是否可达引爆点邻格
 * 2. 能否被推弹+引爆（用推箱A*验证）
 * 每步成功后更新墙位图（移除爆炸范围墙 + 炸弹消失）。
 *
 * @param plans      炸弹方案组合（每个 plan 对应一个炸弹的爆炸方案）
 * @param count      方案数量
 * @param walls      初始墙位图（调用者传入的当前墙位图）
 * @param player_pos 初始玩家位置
 * @param out_order  输出成功排列顺序（plans 索引的排列）
 * @return true      存在可行执行顺序
 */
static bool verify_multi_bomb_sequence(DetonatePlan plans[], uint8_t count,
    const uint16_t walls[MAP_ROWS], Point player_pos,
    uint8_t out_order[]) {
    if (count == 0) return true;
    if (count == 1) {
        bool active[MAX_BOOMS];
        for (uint8_t i = 0; i < MAX_BOOMS; i++) active[i] = true;
        uint16_t walls_copy[MAP_ROWS];
        memcpy(walls_copy, walls, sizeof(uint16_t) * MAP_ROWS);
        BombExecutionStep step;
        if (try_execute_step(&plans[0], 0, active, &player_pos, walls_copy, &step)) {
            out_order[0] = 0;
            return true;
        }
        return false;
    }

    /* 多炸弹：尝试所有排列 */
    int perm[MAX_BOOMS];
    for (uint8_t i = 0; i < count; i++) perm[i] = (int)i;

    uint16_t best_cost = 0xFFFF;
    uint8_t best_perm[MAX_BOOMS];
    bool found = false;

    do {
        uint16_t cur_walls[MAP_ROWS];
        memcpy(cur_walls, walls, sizeof(uint16_t) * MAP_ROWS);
        Point cur_player = player_pos;
        bool done_plans[MAX_BOOMS] = {false};
        uint16_t total_cost = 0;
        bool ok = true;

        for (uint8_t si = 0; si < count; si++) {
            uint8_t pi = (uint8_t)perm[si];
            bool active[MAX_BOOMS];
            for (uint8_t i = 0; i < g_bomb_count; i++) active[i] = true;
            for (uint8_t i = 0; i < count; i++)
                if (done_plans[i]) active[plans[i].bomb_index] = false;

            /* 在 cur_walls 上重算 bomb 可达性（用 plan 自带位置，不受全局 BOMB_INVALID 影响） */
            uint16_t reach_map[MAX_BOOMS][MAP_ROWS];
            memset(reach_map, 0, sizeof(reach_map));
            for (uint8_t i = 0; i < count; i++) {
                uint8_t bi = plans[i].bomb_index;
                if (done_plans[i]) continue;
                Point bp = plans[i].bomb_initial_pos;
                compute_one_bomb_reachability(bi, bp, cur_walls, active, reach_map[bi]);
            }

            /* 检查 bomb 在更新后墙上是否可达引爆点邻格 */
            Point dp = plans[pi].detonate_pos;
            bool can_reach = false;
            for (uint8_t d = 0; d < DIR_COUNT && !can_reach; d++) {
                uint8_t adj_x = (uint8_t)(dp.x + DIRS[d][0]);
                uint8_t adj_y = (uint8_t)(dp.y + DIRS[d][1]);
                if (adj_x >= MAP_ROWS || adj_y >= MAP_COLS) continue;
                if (is_wall_bit(cur_walls, (Point){adj_x, adj_y})) continue;
                if (reach_map[plans[pi].bomb_index][adj_x] & (1 << adj_y))
                    can_reach = true;
            }
            if (!can_reach) { ok = false; break; }

            /* 尝试执行推弹+引爆 */
            BombExecutionStep step;
            if (!try_execute_step(&plans[pi], si, active, &cur_player, cur_walls, &step)) {
                ok = false; break;
            }
            total_cost = (uint16_t)(total_cost + step.push_path.path_len);
            if (total_cost >= best_cost) { ok = false; break; }
            done_plans[pi] = true;
        }

        if (ok && total_cost < best_cost) {
            best_cost = total_cost;
            memcpy(best_perm, perm, count * sizeof(uint8_t));
            found = true;
        }
    } while (next_permutation(perm, (int)count));

    if (found) {
        memcpy(out_order, best_perm, count * sizeof(uint8_t));
        return true;
    }
    return false;
}

/**
 * @brief 递归枚举方案组合（剪枝 + 验证 + 顺序可执行性）
 */
static void try_multi_plans_recursive(
    uint8_t depth, uint8_t max_depth,
    const uint8_t bomb_idx[], const uint8_t bl[],
    DetonatePlan pb[][MAX_PLANS_PER_BOOM], const uint8_t fi[],
    uint8_t sel_plans[], int accum_benefit,
    const uint16_t all_mask, int *best_benefit,
    DetonatePlan result_plans[], uint8_t *result_count, uint8_t *found) {
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

        /* 快速预筛：组合中被炸墙是否至少触及每个问题点的影响域？ */
        bool touches_all = true;
        for (uint8_t pi = 0; pi < g_saved_problem_count && touches_all; pi++) {
            bool touches_this = false;
            for (uint8_t i = 0; i < max_depth && !touches_this; i++) {
                DetonatePlan *p = &pb[bl[bomb_idx[i]]][sel_plans[i]];
                for (uint8_t k = 0; k < p->wall_count && !touches_this; k++) {
                    Point wp = p->walls_covered[k];
                    for (uint8_t d = 0; d < DIR_COUNT; d++) {
                        uint8_t nx = (uint8_t)(wp.x + DIRS[d][0]);
                        uint8_t ny = (uint8_t)(wp.y + DIRS[d][1]);
                        if (nx >= MAP_ROWS || ny >= MAP_COLS) continue;
                        if (g_saved_problem_points[pi].influence_mask[nx] & (1 << ny))
                            { touches_this = true; break; }
                    }
                }
            }
            if (!touches_this) touches_all = false;
        }
        if (!touches_all) return;

        Point saved[MAX_BOOMS];
        for (uint8_t i = 0; i < max_depth; i++) {
            uint8_t ba = bl[bomb_idx[i]];
            saved[i] = g_initial_bombs[ba];
            g_initial_bombs[ba].x = BOMB_INVALID;
        }
        if (check_problems_resolved_cached(cw, g_initial_player) == all_mask) {
            /* 组装临时 plans 数组做顺序验证 */
            DetonatePlan combo_plans[MAX_BOOMS];
            for (uint8_t i = 0; i < max_depth; i++)
                combo_plans[i] = pb[bl[bomb_idx[i]]][sel_plans[i]];
            uint8_t seq_order[MAX_BOOMS];
            /* 顺序可执行性验证 — 确保各炸弹能依次推弹+引爆 */
            if (verify_multi_bomb_sequence(combo_plans, max_depth,
                    g_static_walls, g_initial_player, seq_order)) {
                if (accum_benefit < *best_benefit) {
                    *best_benefit = accum_benefit;
                    *result_count = max_depth;
                    for (uint8_t i = 0; i < max_depth; i++)
                        result_plans[i] = combo_plans[i];
                }
                *found = 1;
            }
        }
        for (uint8_t i = 0; i < max_depth; i++)
            g_initial_bombs[bl[bomb_idx[i]]] = saved[i];
        return;
    }
    uint8_t bi = bomb_idx[depth];
    uint8_t ba = bl[bi];
    for (uint8_t pi = 0; pi < fi[ba]; pi++) {
        /* 跳过不解决任何问题且不影响任何影响域的方案 */
        if (pb[ba][pi].resolved_mask == 0 && pb[ba][pi].wall_count == 0) continue;
        int new_benefit = accum_benefit + pb[ba][pi].total_benefit;
        if (new_benefit >= *best_benefit) break;
        sel_plans[depth] = pi;
        try_multi_plans_recursive(depth + 1, max_depth, bomb_idx, bl, pb, fi,
            sel_plans, new_benefit, all_mask, best_benefit,
            result_plans, result_count, found);
    }
}

/**
 * @brief 搜索 m 个炸弹的组合（表驱动，统一 2/3/4 弹搜索）
 */
static bool search_multi_combo(
    uint8_t target_m, const uint8_t N, const uint8_t bl[],
    DetonatePlan pb[][MAX_PLANS_PER_BOOM], const uint8_t fi[],
    const uint16_t all_mask, int *best_benefit,
    DetonatePlan result_plans[], uint8_t *result_count) {
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
 * @brief 多炸弹组合搜索（2/3/4炸弹组合，倾向少用炸弹）
 */
static bool search_multi_bomb_combination(DetonatePlan all_plans[], uint8_t plan_count,
    DetonatePlan result_plans[], uint8_t *result_count, int *out_total_benefit) {
    *result_count = 0;
    if (plan_count == 0 || g_bomb_count == 0 || g_saved_problem_count == 0) return false;
    const uint16_t all_mask = (uint16_t)((1 << g_saved_problem_count) - 1);

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

    uint8_t fi[MAX_BOOMS]; memset(fi, 0, sizeof(fi));
    for (uint8_t pi = 0; pi < plan_count; pi++) {
        uint8_t bi = all_plans[pi].bomb_index;
        if (fi[bi] < MAX_PLANS_PER_BOOM) g_bomb_pb[bi][fi[bi]++] = all_plans[pi];
    }
    for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
        sort_plans_by_benefit(g_bomb_pb[bi], fi[bi]);
        uint8_t keep = 0;
        for (uint8_t pi = 0; pi < fi[bi]; pi++) {
            bool dup = false;
            for (uint8_t pj = 0; pj < keep; pj++) {
                if (g_bomb_pb[bi][pi].wall_count != g_bomb_pb[bi][pj].wall_count) continue;
                bool same = true;
                for (uint8_t wk = 0; wk < g_bomb_pb[bi][pi].wall_count; wk++)
                    if (!pos_equal(g_bomb_pb[bi][pi].walls_covered[wk], g_bomb_pb[bi][pj].walls_covered[wk]))
                        { same = false; break; }
                if (same) { dup = true; break; }
            }
            if (!dup && keep < MAX_PLANS_PER_BOOM) g_bomb_pb[bi][keep++] = g_bomb_pb[bi][pi];
        }
        if (keep > BOOM_TOP_K) keep = BOOM_TOP_K;
        fi[bi] = keep;
    }

    uint8_t bl[MAX_BOOMS]; uint8_t blc = 0;
    for (uint8_t bi = 0; bi < g_bomb_count; bi++)
        if (fi[bi] > 0) bl[blc++] = bi;
    if (blc < 2) return false;

    /* 贪心预求解：快速找一个可行解作为上界，大幅加速后续组合搜索剪枝 */
    int best_benefit = 999999;
    {
        uint8_t gmask = 0; int gcost = 0;
        bool gused[MAX_BOOMS] = {false};
        for (uint8_t i = 0; i < plan_count && gmask != all_mask; i++) {
            uint8_t bi = all_plans[i].bomb_index;
            if (gused[bi]) continue;
            uint8_t nb = all_plans[i].resolved_mask & ~gmask;
            if (nb == 0) continue;
            gmask |= nb; gused[bi] = true;
            gcost += all_plans[i].total_benefit;
        }
        if (gmask == all_mask && gcost < best_benefit)
            best_benefit = gcost + 1; /* +1 保证严格更优时才会替换 */
    }
    const uint8_t N = blc;

    /* 多炸弹组合（2/3/4弹，倾向少用炸弹） */
    if (search_multi_combo(2, N, bl, g_bomb_pb, fi, all_mask, &best_benefit, result_plans, result_count)) {
        *out_total_benefit = best_benefit; return true;
    }
    if (N >= 3 && search_multi_combo(3, N, bl, g_bomb_pb, fi, all_mask, &best_benefit, result_plans, result_count)) {
        *out_total_benefit = best_benefit; return true;
    }
    if (N >= 4 && search_multi_combo(4, N, bl, g_bomb_pb, fi, all_mask, &best_benefit, result_plans, result_count)) {
        *out_total_benefit = best_benefit; return true;
    }
    return false;
}

/**
 * @brief 迭代贪心炸弹破局搜索
 *
 * 每轮在更新后的墙位图上重新计算炸弹可达性和可炸墙，
 * 用 BFS 真实距离（玩家→炸弹 + 炸弹→引爆点）评估最优方案，
 * 逐弹引爆并更新墙位图。
 *
 * @param deadlock      初始死锁检测结果
 * @param out_plans     输出炸弹方案列表
 * @param out_plan_count 输出方案数量
 * @param out_best_benefit 输出最优收益
 * @return true         找到可行方案
 */

/**
 * @brief 对炸弹引爆候选方案进行评分排序
 *
 * 综合考量：行走距离 + 推弹距离 - 邻接死锁墙奖励 + 距UNR目标距离
 *
 * @param cand_count     候选方案数量
 * @param resolved       已解决问题位掩码
 * @param cur_walls      当前墙位图
 * @param cur_reach      炸弹可达性位图
 * @param active_bombs   活跃炸弹标记
 * @param cur_player     当前玩家位置
 * @param[out] scored    评分结果（按代价升序）
 * @param[out] scored_count 候选数量
 */
static void score_bomb_candidates(uint8_t cand_count, uint16_t resolved,
    const uint16_t cur_walls[MAP_ROWS],
    const uint16_t cur_reach[MAX_BOOMS][MAP_ROWS],
    const bool active_bombs[MAX_BOOMS],
    Point cur_player,
    ScoredPlan scored[], uint8_t *scored_count) {
    *scored_count = 0;

    /* ENC/UNR 未解时，引爆点离目标越近越好 */
    bool has_enc_unr = false;
    for (uint8_t pi = 0; pi < g_saved_problem_count && !has_enc_unr; pi++)
        if (!(resolved & (1 << pi)) &&
            (g_saved_problem_points[pi].type & (PROBLEM_ENCLOSED | PROBLEM_TARGET_UNREACHABLE)))
            has_enc_unr = true;

    /* 预计算玩家到所有可到达格的BFS距离（替代逐候选A*） */
    {
        memset(g_obs_buf, 0, sizeof(g_obs_buf));
        for (uint8_t r = 0; r < MAP_ROWS; r++) g_obs_buf[r] = cur_walls[r];
        for (uint8_t b = 0; b < g_box_count; b++)
            g_obs_buf[g_initial_boxes[b].x] |= (1 << g_initial_boxes[b].y);
        for (uint8_t b = 0; b < g_bomb_count; b++) {
            if (!active_bombs[b]) continue;
            if (g_initial_bombs[b].x == BOMB_INVALID) continue;
            g_obs_buf[g_initial_bombs[b].x] |= (1 << g_initial_bombs[b].y);
        }
        bfs_compute_distances(cur_player, g_obs_buf);
    }

    for (uint8_t i = 0; i < cand_count; i++) {
        DetonatePlan *cp = &g_bomb_candidates[i];
        if (!active_bombs[cp->bomb_index]) continue;
        Point bomb_approach = cp->bomb_initial_pos;
        Point approach_pos = {0, 0};
        bool found_approach = false;
        for (uint8_t d = 0; d < DIR_COUNT && !found_approach; d++) {
            uint8_t nx = (uint8_t)(bomb_approach.x + DIRS[d][0]);
            uint8_t ny = (uint8_t)(bomb_approach.y + DIRS[d][1]);
            if (nx >= MAP_ROWS || ny >= MAP_COLS) continue;
            /* 炸弹可被推入墙格引爆：邻格=引爆点时可跳过墙检查 */
            bool is_detonate_cell = (nx == cp->detonate_pos.x && ny == cp->detonate_pos.y);
            if (!is_detonate_cell && is_wall_bit(cur_walls, (Point){nx, ny})) continue;
            bool blocked = false;
            for (uint8_t bj = 0; bj < g_box_count && !blocked; bj++)
                if (g_initial_boxes[bj].x == nx && g_initial_boxes[bj].y == ny) blocked = true;
            for (uint8_t bj = 0; bj < g_bomb_count && !blocked; bj++) {
                if (!active_bombs[bj]) continue;
                if (bj == cp->bomb_index) continue;
                if (g_initial_bombs[bj].x == BOMB_INVALID) continue;
                if (g_initial_bombs[bj].x == nx && g_initial_bombs[bj].y == ny) blocked = true;
            }
            if (blocked) continue;
            if (is_detonate_cell || (cur_reach[cp->bomb_index][nx] & (1 << ny))) {
                approach_pos.x = nx; approach_pos.y = ny;
                found_approach = true;
            }
        }
        if (!found_approach) continue;

        uint16_t walk_dist = g_dist_map[approach_pos.x][approach_pos.y];
        /* 引爆点是墙时玩家无法走到该格，用玩家到炸弹邻侧距离替代 */
        if (is_wall_bit(cur_walls, approach_pos)) {
            /* 计算玩家到炸弹另一侧(推弹位)的距离 */
            uint8_t px = (uint8_t)(bomb_approach.x - (approach_pos.x - bomb_approach.x));
            uint8_t py = (uint8_t)(bomb_approach.y - (approach_pos.y - bomb_approach.y));
            walk_dist = (px < MAP_ROWS && py < MAP_COLS) ? g_dist_map[px][py] : INF;
        }
        if (walk_dist == 0 || walk_dist == INF) continue;

        /* 向最近UNR目标推进的距离 */
        uint16_t min_dist = INF;
        for (uint8_t pi = 0; pi < g_saved_problem_count; pi++) {
            if (resolved & (1 << pi)) continue;
            if (!(g_saved_problem_points[pi].type & PROBLEM_TARGET_UNREACHABLE)) continue;
            uint8_t ti = g_saved_problem_points[pi].target_indices[0];
            uint16_t d = manhattan_distance(cp->detonate_pos, g_initial_targets[ti]);
            if (d < min_dist) min_dist = d;
        }

        /* 统计摧毁墙中有多少邻接未解决死锁的影响域 */
        int coverage_bonus = 0;
        for (int8_t ddx = -1; ddx <= 1; ddx++)
            for (int8_t ddy = -1; ddy <= 1; ddy++) {
                uint8_t wwx = (uint8_t)((int)cp->detonate_pos.x + ddx);
                uint8_t wwy = (uint8_t)((int)cp->detonate_pos.y + ddy);
                if (wwx >= MAP_ROWS || wwy >= MAP_COLS) continue;
                if (wwx == 0 || wwx == MAP_ROWS-1 || wwy == 0 || wwy == MAP_COLS-1) continue;
                if (!is_wall_bit(cur_walls, (Point){wwx, wwy})) continue;
                for (uint8_t d = 0; d < DIR_COUNT; d++) {
                    uint8_t nbx = (uint8_t)(wwx + DIRS[d][0]);
                    uint8_t nby = (uint8_t)(wwy + DIRS[d][1]);
                    if (nbx >= MAP_ROWS || nby >= MAP_COLS) continue;
                    for (uint8_t pi = 0; pi < g_saved_problem_count; pi++) {
                        if (resolved & (1 << pi)) continue;
                        if (g_saved_problem_points[pi].influence_mask[nbx] & (1 << nby)) {
                            coverage_bonus += 10;
                            goto next_wall;
                        }
                    }
                }
                next_wall:;
            }

        int total = (int)walk_dist + (int)cp->bomb_push_distance * 3
                  - coverage_bonus
                  + (has_enc_unr ? (int)min_dist * 5 : 0);
        for (uint8_t pi = 0; pi < g_saved_problem_count; pi++)
            if (!(resolved & (1 << pi)) && (cp->resolved_mask & (1 << pi)))
                total -= 500;

        if (*scored_count < MAX_DETONATE_POINTS) {
            scored[*scored_count].cost = total;
            scored[*scored_count].idx = i;
            (*scored_count)++;
        }
    }

    /* 按代价升序排序 */
    for (uint8_t si = 0; si + 1 < *scored_count; si++) {
        uint8_t best_s = si;
        for (uint8_t sj = (uint8_t)(si + 1); sj < *scored_count; sj++)
            if (scored[sj].cost < scored[best_s].cost) best_s = sj;
        if (best_s != si) { ScoredPlan tmp = scored[si]; scored[si] = scored[best_s]; scored[best_s] = tmp; }
    }
}

static bool iterative_bomb_breakthrough(const DeadlockResult *deadlock,
    DetonatePlan out_plans[], uint8_t *out_plan_count, int *out_best_benefit) {
    *out_plan_count = 0;
    if (g_bomb_count == 0 || !deadlock->has_deadlock) return false;

    /* 清空验证缓存（迭代过程中墙位图变化，缓存可能误命中） */
    memset(g_vcache_hash, 0, sizeof(g_vcache_hash));
    memset(g_vcache_mask, 0, sizeof(g_vcache_mask));
    memset(g_unr_reach_cache, -1, sizeof(g_unr_reach_cache));   /* UNR箱子缓存重置 */

    memcpy(g_saved_problem_points, g_problem_points, sizeof(DeadlockProblemPoint) * g_problem_count);
    g_saved_problem_count = g_problem_count;

    uint8_t chosen_count = 0;
    int total_benefit = 0;
    const uint16_t all_mask = (uint16_t)((1 << g_saved_problem_count) - 1);

    /* ================================================================
     * 主路径：迭代贪心逐弹选择 + 逐轮重评估 + 首轮回溯
     * ================================================================ */

    /* 保存初始状态（用于失败后回溯重试） */
    Point  saved_initial_bombs[MAX_BOOMS];
    memcpy(saved_initial_bombs, g_initial_bombs, sizeof(g_initial_bombs));
    memcpy(g_bomb_saved_map, g_original_map, sizeof(g_original_map));

    int8_t top_alt_per_bomb[MAX_BOOMS];     /* 每颗炸弹的最佳方案索引，-1=无 */
    uint8_t top_alt_count = 0;               /* 有方案的炸弹数 */
    int8_t normal_bomb = -1;                  /* retry=0 首轮实际使用的炸弹 */
    for (uint8_t i = 0; i < MAX_BOOMS; i++) top_alt_per_bomb[i] = -1;

    /* ── 回溯重试循环 ── */
    for (uint8_t retry = 0; ; retry++) {
        if (retry > 0) {
            /* 恢复初始状态 */
            memcpy(g_initial_bombs, saved_initial_bombs, sizeof(g_initial_bombs));
            memcpy(g_original_map, g_bomb_saved_map, sizeof(g_original_map));
            memset(g_vcache_hash, 0, sizeof(g_vcache_hash));
            memset(g_vcache_mask, 0, sizeof(g_vcache_mask));
        }

        /* 当前墙位图副本（逐步更新）—— 指向全局缓冲区省栈 */
        uint16_t *cur_walls = g_tmp_walls_cur;
        memcpy(g_tmp_walls_cur, g_static_walls, sizeof(uint16_t) * MAP_ROWS);
        Point cur_player = g_initial_player;
        bool active_bombs[MAX_BOOMS];
        for (uint8_t i = 0; i < MAX_BOOMS; i++) active_bombs[i] = true;

        /* 卡死炸弹复活标记：区分"被卡死"与"已引爆" */
        bool bomb_stuck[MAX_BOOMS] = {false};
        Point stuck_bomb_pos[MAX_BOOMS];  /* 保存卡死炸弹原始位置 */

        /* ★ 卡死的炸弹视为不可炸墙：从活跃列表移除，加入墙位图 */
        for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
            if (!active_bombs[bi]) continue;
            Point bp = g_initial_bombs[bi];
            if (bp.x == BOMB_INVALID) continue;
            if (!can_push_bomb(bi, bp, cur_walls, active_bombs)) {
                stuck_bomb_pos[bi] = bp;
                bomb_stuck[bi] = true;
                cur_walls[bp.x] |= (1 << bp.y);
                active_bombs[bi] = false;
                g_initial_bombs[bi].x = BOMB_INVALID;
            }
        }

        chosen_count = 0;
        total_benefit = 0;
        bool forced_round0 = (retry > 0 && (retry - 1) < top_alt_count);

        for (uint8_t iteration = 0; iteration < g_bomb_count; iteration++) {
            uint16_t resolved = check_problems_resolved_incremental(cur_walls, cur_player);

            if (resolved == all_mask) break;  /* 全部解决 */

        /* ★ 用当前墙位图重算玩家区（炸墙后玩家区扩大，才能找到更深层的可炸墙） */
        compute_player_region_with_walls(cur_walls, false);

        /* 在 cur_walls 上重算炸弹可达性 */
        compute_bomb_reachability(cur_walls, active_bombs, g_bomb_reach_map);

        /*
         * ★ 关键修复：在 cur_walls（已更新墙位图）上重新计算每个未解决问题的影响域。
         * 炸墙后影响域会扩大，旧掩码仅覆盖旧边界 → 找不到新边界上的可炸墙。
         */
        for (uint8_t pi = 0; pi < g_saved_problem_count; pi++) {
            if (resolved & (1 << pi)) continue;
            Point seed;
            if (g_saved_problem_points[pi].box_count > 0)
                seed = g_initial_boxes[g_saved_problem_points[pi].box_indices[0]];
            else if (g_saved_problem_points[pi].target_count > 0)
                seed = g_initial_targets[g_saved_problem_points[pi].target_indices[0]];
            else continue;
            compute_box_influence(seed, cur_walls, g_saved_problem_points[pi].influence_mask);
        }

        /* 为当前活跃炸弹生成候选方案 */
        uint8_t cand_count = 0;

        /* 收集所有未解决问题的可炸墙（不提前退出，确保每颗炸弹都有机会） */
        uint8_t all_walls_count = 0;
        for (uint8_t priority = 0; priority < 3; priority++) {
            uint8_t target_type = (priority == 0) ? PROBLEM_ENCLOSED :
                                  (priority == 1) ? PROBLEM_NEED_SIM : PROBLEM_TARGET_UNREACHABLE;

        for (uint8_t pi = 0; pi < g_saved_problem_count; pi++) {
            if (!(g_saved_problem_points[pi].type & target_type)) continue;

            collect_walls_for_problem(pi, cur_walls, g_bomb_walls, &all_walls_count, MAX_BREAK_WALLS, active_bombs);
            /* 遗墙继承：已解ENC的墙邻接开放区的，供未解问题使用 */
            for (uint8_t pj = 0; pj < g_saved_problem_count; pj++) {
                if (!(g_saved_problem_points[pj].type & PROBLEM_ENCLOSED)) continue;
                for (uint8_t w = 0; w < g_saved_problem_points[pj].wall_count && all_walls_count < MAX_BREAK_WALLS; w++) {
                    Point wp = g_saved_problem_points[pj].enc_walls[w];
                    if (!is_breakable_wall_on(wp, cur_walls)) continue;
                    bool touches_open = false;
                    for (uint8_t d = 0; d < DIR_COUNT && !touches_open; d++) {
                        Point nb = {(uint8_t)(wp.x + DIRS[d][0]), (uint8_t)(wp.y + DIRS[d][1])};
                        if (nb.x >= MAP_ROWS || nb.y >= MAP_COLS) continue;
                        if (!is_wall_bit(cur_walls, nb)) touches_open = true;
                    }
                    if (!touches_open) continue;
                    bool dup = false;
                    for (uint8_t k = 0; k < all_walls_count; k++)
                        if (pos_equal(g_bomb_walls[k].wall_pos, wp)) { dup = true; break; }
                    if (!dup) {
                        g_bomb_walls[all_walls_count].wall_pos = wp;
                        g_bomb_walls[all_walls_count].benefit_score = -10;
                        all_walls_count++;
                    }
                }
            }
        }
        }  /* priority loop */

        if (all_walls_count > 0) {
            generate_plans_for_walls_on(g_bomb_walls, all_walls_count,
                cur_walls, g_bomb_reach_map, cur_player, active_bombs,
                g_bomb_candidates, &cand_count);
        }
        if (cand_count == 0) {
            /* 无候选方案 → 尝试多炸弹组合回退 */
            uint8_t all_plan_count = 0;
            /* 重新从原始问题收集，用 cur_walls 生成 */
            for (uint8_t pi = 0; pi < g_saved_problem_count; pi++) {
                if (resolved & (1 << pi)) continue;
                uint8_t wall_count = 0;
                collect_walls_for_problem(pi, cur_walls, g_bomb_walls, &wall_count, MAX_BREAK_WALLS, active_bombs);
                if (wall_count > 0)
                    generate_plans_for_walls_on(g_bomb_walls, wall_count, cur_walls, g_bomb_reach_map, cur_player, active_bombs, g_bomb_all_plans, &all_plan_count);
            }

            if (all_plan_count > 0) {
                DetonatePlan combo_plans[MAX_BOOMS];
                uint8_t combo_count = 0;
                int combo_benefit = 0;
                sort_plans_by_benefit(g_bomb_all_plans, all_plan_count);
                if (search_multi_bomb_combination(g_bomb_all_plans, all_plan_count, combo_plans, &combo_count, &combo_benefit)) {
                    for (uint8_t ci = 0; ci < combo_count && chosen_count < MAX_BOOMS; ci++) {
                        out_plans[chosen_count++] = combo_plans[ci];
                        active_bombs[combo_plans[ci].bomb_index] = false;
                        {
                            uint8_t bi = combo_plans[ci].bomb_index;
                            Point bp = g_initial_bombs[bi];
                            g_initial_bombs[bi].x = BOMB_INVALID;
                            g_original_map[bp.x][bp.y] = FLOOR;
                        }
                        total_benefit += combo_plans[ci].total_benefit;
                    }
                    break;
                }
            }
            break; /* 无解 */
        }

        /* ★ 综合评价：行走距离 + 推弹距离 - 解决死锁的奖励 */
        ScoredPlan scored[MAX_DETONATE_POINTS];
        uint8_t scored_count = 0;
        score_bomb_candidates(cand_count, resolved, cur_walls, g_bomb_reach_map,
            active_bombs, cur_player, scored, &scored_count);

        /* retry=0 且首轮：保存每颗炸弹的最佳方案（用于回溯尝试不同炸弹） */
        if (retry == 0 && iteration == 0) {
            top_alt_count = 0;
            for (uint8_t si = 0; si < scored_count; si++) {
                DetonatePlan *cp = &g_bomb_candidates[scored[si].idx];
                uint8_t bi = cp->bomb_index;
                if (top_alt_per_bomb[bi] >= 0) continue;
                BombPushPath tp;
                compute_one_bomb_push(cp->bomb_initial_pos, cp->bomb_index,
                    cp->detonate_pos, cur_player, cur_walls, active_bombs, &tp);
                if (tp.path_valid) {
                    top_alt_per_bomb[bi] = (int8_t)scored[si].idx;
                }
            }
        }

        /* 第二阶段：按排序依次尝试执行，直到成功 */
        int best_idx = -1;
        if (forced_round0 && iteration == 0) {
            uint8_t cnt = 0;
            for (uint8_t bi = 0; bi < MAX_BOOMS; bi++) {
                if (top_alt_per_bomb[bi] < 0) continue;
                if ((int8_t)bi == normal_bomb) continue;
                if (cnt == retry - 1) { best_idx = (int)top_alt_per_bomb[bi]; break; }
                cnt++;
            }
        } else {
            for (uint8_t si = 0; si < scored_count; si++) {
                DetonatePlan *cp = &g_bomb_candidates[scored[si].idx];
                BombPushPath test_path;
                compute_one_bomb_push(cp->bomb_initial_pos, cp->bomb_index,
                    cp->detonate_pos, cur_player, cur_walls, active_bombs, &test_path);
                if (test_path.path_valid) {
                    best_idx = (int)scored[si].idx;
                    break;
                }
            }
        }

        if (best_idx < 0) break;

        /* 执行最佳方案 */
        DetonatePlan *chosen = &g_bomb_candidates[best_idx];
        if (retry == 0 && iteration == 0) normal_bomb = (int8_t)chosen->bomb_index;
        /* retry=0 首轮执行后：统计备选炸弹数（不含已用的 normal_bomb） */
        if (retry == 0 && iteration == 0) {
            top_alt_count = 0;
            for (uint8_t bi = 0; bi < MAX_BOOMS; bi++)
                if (top_alt_per_bomb[bi] >= 0 && (int8_t)bi != normal_bomb)
                    top_alt_count++;
        }

        BombExecutionStep step;
        bool exec_ok = try_execute_step(chosen, chosen_count, active_bombs, &cur_player, cur_walls, &step);
        if (!exec_ok) break;

        out_plans[chosen_count++] = *chosen;
        active_bombs[chosen->bomb_index] = false;
        {
            uint8_t bi = chosen->bomb_index;
            Point bp = g_initial_bombs[bi];
            g_initial_bombs[bi].x = BOMB_INVALID;
            g_original_map[bp.x][bp.y] = FLOOR;
        }
        total_benefit += chosen->total_benefit;

        /* ★ 死炸弹复活检测：炸墙后卡死的炸弹可能重获推动空间 */
        for (uint8_t bi = 0; bi < g_bomb_count; bi++) {
            if (!bomb_stuck[bi]) continue;
            Point bp = stuck_bomb_pos[bi];
            /* 临时移除cur_walls中的炸弹位，检测是否可推动 */
            cur_walls[bp.x] &= (uint16_t)~(1 << bp.y);
            if (can_push_bomb(bi, bp, cur_walls, active_bombs)) {
                /* 复活！恢复为活跃炸弹，参与后续迭代 */
                g_initial_bombs[bi] = bp;
                g_original_map[bp.x][bp.y] = BOOM;
                active_bombs[bi] = true;
                bomb_stuck[bi] = false;
            } else {
                /* 仍卡死，重新加入cur_walls */
                cur_walls[bp.x] |= (1 << bp.y);
            }
        }
    }

    /* ★ 最终验证：所有死锁问题已解决？ */
    {
        uint16_t final_resolved = check_problems_resolved_incremental(cur_walls, cur_player);
        if (final_resolved == all_mask) {
            *out_plan_count = chosen_count;
            *out_best_benefit = total_benefit;
            return chosen_count > 0;
        }
    }

    /* 失败 → 尝试回溯：用首轮备选方案重试 */
    if (retry >= top_alt_count) break;
    /* 继续下一轮重试（retry++） */
    }

    *out_plan_count = 0;
    return false;
}

/* ---------- 6.5.1 模式3子功能：顺序推弹路径规划 ---------- */

/**
 * @brief BFS计算玩家到各炸弹的行走距离
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
    memset(g_dist_map, 0xFF, sizeof(g_dist_map));
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
 * @brief 单次推弹：将炸弹推到爆炸点（墙位），返回玩家路径
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
    /* 始终走A*计算推弹路径（含玩家走到推弹位+推动），不再短路邻接情况 */
    memcpy(g_tmp_obs, walls, sizeof(uint16_t) * MAP_ROWS);
    g_tmp_obs[detonate_pos.x] &= (uint16_t)~(1 << detonate_pos.y);
    for (uint8_t i = 0; i < g_box_count; i++) {
        Point bp = g_initial_boxes[i];
        if (!pos_equal(bp, bomb_pos)) g_tmp_obs[bp.x] |= (1 << bp.y);
    }
    for (uint8_t i = 0; i < g_bomb_count; i++) {
        if (i == bomb_index) continue;
        if (active_bombs && !active_bombs[i]) continue;
        if (g_initial_bombs[i].x == BOMB_INVALID) continue;
        g_tmp_obs[g_initial_bombs[i].x] |= (1 << g_initial_bombs[i].y);
    }
    g_bomb_ar = solve_single_box_a_star(player_start, bomb_pos, detonate_pos, g_tmp_obs);
    if (!g_bomb_ar.success || g_bomb_ar.path_len == 0) return;
    uint16_t len = (g_bomb_ar.path_len > MAX_PATH_LEN) ? MAX_PATH_LEN : g_bomb_ar.path_len;
    out_path->path_len = len;
    memcpy(out_path->path, g_bomb_ar.path_points, len * sizeof(Point));
    out_path->path_valid = true;

    /* 检测推弹步：仅标记最后一次推弹（引爆步） */
    {
        Point cur_bomb = bomb_pos;
        int16_t last_push = -1;
        for (uint16_t i = 0; i < len; i++) {
            if (pos_equal(out_path->path[i], cur_bomb)) {
                last_push = (int16_t)i;
                if (i > 0) {
                    int8_t dx = (int8_t)(out_path->path[i].x - out_path->path[i-1].x);
                    int8_t dy = (int8_t)(out_path->path[i].y - out_path->path[i-1].y);
                    cur_bomb.x = (uint8_t)(cur_bomb.x + dx);
                    cur_bomb.y = (uint8_t)(cur_bomb.y + dy);
                }
            }
        }
        if (last_push >= 0)
            out_path->is_push[last_push] = true;
    }
}

/**
 * @brief 执行单步推弹+引爆，更新玩家位置和墙体
 */
static bool try_execute_step(const DetonatePlan *p, uint8_t step_idx,
    const bool active_bombs[MAX_BOOMS], Point *player, uint16_t walls[MAP_ROWS],
    BombExecutionStep *out_st) {
    compute_one_bomb_push(p->bomb_initial_pos, p->bomb_index,
        p->detonate_pos, *player, walls, active_bombs, &g_bomb_ppath);
    if (!g_bomb_ppath.path_valid) return false;
    memset(out_st, 0, sizeof(BombExecutionStep));
    out_st->step_order = step_idx; out_st->bomb_index = p->bomb_index;
    out_st->bomb_from = p->bomb_initial_pos; out_st->detonate_at = p->detonate_pos;
    out_st->player_before = *player;
    memcpy(&out_st->push_path, &g_bomb_ppath, sizeof(BombPushPath));
    out_st->player_after = (g_bomb_ppath.path_len > 0) ? g_bomb_ppath.path[g_bomb_ppath.path_len - 1] : *player;
    *player = out_st->player_after;
    { Point dp = p->detonate_pos; uint8_t rc = 0;
    for (int8_t dx = -1; dx <= 1; dx++)
        for (int8_t dy = -1; dy <= 1; dy++) {
            uint8_t wx = (uint8_t)((int)dp.x + dx), wy = (uint8_t)((int)dp.y + dy);
            if (wx >= MAP_ROWS || wy >= MAP_COLS) continue;
            /* ★ 最外层边界墙不可破坏 */
            if (wx == 0 || wx == MAP_ROWS - 1 || wy == 0 || wy == MAP_COLS - 1) continue;
            if (!(walls[wx] & (1 << wy))) continue;
            walls[wx] &= (uint16_t)~(1 << wy);
            if (rc < 9) { out_st->walls_removed[rc].x = wx; out_st->walls_removed[rc].y = wy; rc++; }
        }
    out_st->walls_removed_count = rc; }
    return true;
}

/**
 * @brief 贪心顺序推弹规划（BFS选最近+地图逐步更新）
 * 
 * 若贪心不能完成，回退到全排列搜索。
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
        do {
            uint16_t w_try[MAP_ROWS]; memcpy(w_try, g_static_walls, sizeof(w_try));
            Point p_try = g_initial_player; bool d_try[MAX_BOOMS] = {false};
            uint8_t t_n = 0; uint16_t t_c = 0;
            for (uint8_t si = 0; si < plan_count; si++) {
                uint8_t pi = (uint8_t)order[si];
                bool ab[MAX_BOOMS];
                for (uint8_t j = 0; j < g_bomb_count; j++) ab[j] = true;
                for (uint8_t j = 0; j < plan_count; j++)
                    if (d_try[j]) ab[plans[j].bomb_index] = false;
                if (!try_execute_step(&plans[pi], t_n, ab, &p_try, w_try, &g_bomb_t_s[t_n])) break;
                t_c = (uint16_t)(t_c + g_bomb_t_s[t_n].push_path.path_len);
                if (t_c >= best_c) break; /* 剪枝：累积代价已不小于当前最优 */
                d_try[pi] = true; t_n++;
            }
            if (t_n == plan_count && t_c < best_c) {
                best_c = t_c; best_n = t_n;
                memcpy(g_bomb_best_s, g_bomb_t_s, t_n * sizeof(BombExecutionStep));
            }
        } while (next_permutation(order, (int)plan_count));
        if (best_n == plan_count && best_c < 0xFFFF) {
            memcpy(out_exec->steps, g_bomb_best_s, best_n * sizeof(BombExecutionStep));
            out_exec->step_count = best_n; out_exec->total_cost = best_c;
            out_exec->is_valid = true; return;
        }
        /* 全排列也失败 → 标记为无效，不返回部分结果 */
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
 * 第7部分：顶层规划接口函数
 * =================================================================== */

/**
 * @brief START模式顶层接口：计算最近元素接近路径
 */
Path_Start path_start_calculation(uint8_t map[MAP_ROWS][MAP_COLS]) {
    memset(&g_path_start_out, 0, sizeof(Path_Start));
    ApproachResult* res = find_nearest_approach(map);
    if (res->success) extract_start_turn_points(res);
    return g_path_start_out;
}

/**
 * @brief 模式1（非ID推箱子）顶层接口
 */
Path path_calculation(uint8_t box_x[MAP_ROWS][MAP_COLS]) {
    memset(&g_path_out, 0, sizeof(Path));
    parse_map_input(box_x);

    /* 模式一中炸弹视为墙（不可通行） */
    for (uint8_t b = 0; b < g_bomb_count; b++)
        g_static_walls[g_initial_bombs[b].x] |= (1 << g_initial_bombs[b].y);
    SolutionSequence sol;
    generate_greedy_pairing(&sol);

    g_mode1_strict = true;   /* 模式1：非配对靶位阻挡箱子 */
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
 * @brief 模式2（ID推箱子）观察阶段顶层接口
 */
Path_Look path_look_calculation(uint8_t box_x[MAP_ROWS][MAP_COLS]) {
    memset(&g_path_look_out, 0, sizeof(Path_Look));
    id_learning(box_x);
    extract_look_turn_points(g_visit_plan);
    return g_path_look_out;
}

/**
 * @brief 记录用户输入的ID（ID模式交互回调）
 */
void id_input(int id) {
    /* 跳过推箱步（type=0），找到下一个实际观测步 */
    while (g_current_step < g_visit_count && g_visit_plan[g_current_step].type == 0)
        g_current_step++;
    if (g_current_step < g_visit_count) {
        id_record(id);
        g_current_step++;
    }
}

/**
 * @brief 模式2推理阶段顶层接口
 */
Path path_id_calculation(void) {
    if (g_visit_count > 0) {
        g_initial_player = g_visit_plan[g_visit_count - 1].pos;
    }
    /* 备份原始ID映射，backtrack_validate中用原始ID判断同源阻挡 */
    memcpy(g_orig_box_id_map, g_box_id_map, sizeof(g_box_id_map));
    memcpy(g_orig_target_id_map, g_target_id_map, sizeof(g_target_id_map));
    id_inference();
    SolutionSequence* id_sol = build_solution_from_id_pairing();
    return path_id_calculate(id_sol);
}

/**
 * @brief 模式3（炸弹破局分析）顶层接口
 */
Path path_boom_calculation(uint8_t map[MAP_ROWS][MAP_COLS]) {
    memset(&g_path_out, 0, sizeof(Path));
    memset(g_boom_final_walls, 0, sizeof(g_boom_final_walls));

    DeadlockResult deadlock = detect_all_deadlocks(map);
    if (!deadlock.has_deadlock || g_bomb_count == 0)
        return g_path_out;

    uint8_t plan_count = 0; int best_benefit = 0;
    DetonatePlan plans[MAX_BOOMS];
    if (!iterative_bomb_breakthrough(&deadlock, plans, &plan_count, &best_benefit) || plan_count == 0)
        return g_path_out;

    /* ★ 重置全局状态：iterative_bomb_breakthrough 修改了 g_initial_bombs/g_original_map */
    parse_map_input(map);

    BombExecutionPlan exec_plan;
    plan_bomb_execution_sequence(plans, plan_count, &exec_plan);
    BombExecutionPlan *ep = &exec_plan;
    if (!ep->is_valid || ep->step_count == 0) return g_path_out;

    /* ── 计算最终墙位图：原始墙 - 所有步骤摧毁的墙 ── */
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

    /* ── 后处理is_push：仅当下一步穿过刚炸的墙时保留引爆步标志 ── */
    for (uint8_t si = 0; si + 1 < ep->step_count; si++) {
        BombExecutionStep *st = &ep->steps[si];
        BombPushPath *next_pp = &ep->steps[si + 1].push_path;
        bool crosses = false;
        for (uint16_t i = 0; i < next_pp->path_len && !crosses; i++) {
            Point pt = next_pp->path[i];
            for (uint8_t w = 0; w < st->walls_removed_count; w++)
                if (pos_equal(pt, st->walls_removed[w])) { crosses = true; break; }
        }
        if (!crosses) {
            /* 下一步不穿过刚炸的墙 → 清除该步骤的is_push，无需等待 */
            memset(st->push_path.is_push, 0, sizeof(st->push_path.is_push));
        }
    }
    /* 最后一颗弹：无人走它炸开的墙，清除is_push */
    if (ep->step_count > 0)
        memset(ep->steps[ep->step_count - 1].push_path.is_push, 0,
               sizeof(ep->steps[0].push_path.is_push));

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
            g_path_out.is_push[total] = (uint8_t)pp->is_push[i];
            last_added = cur; total++;
        }
    }
    g_path_out.len = total;
    extract_turn_points(&g_path_out);
    return g_path_out;
}

/**
 * @brief 输出炸弹引爆后的新地图
 *
 * 将原始地图中已被炸毁的墙清除，已使用的炸弹移除。
 *
 * @param out_map 输出 12x16 地图数组
 */
void map_boom_out(uint8_t out_map[MAP_ROWS][MAP_COLS]) {
    memcpy(out_map, g_original_map, sizeof(uint8_t) * MAP_ROWS * MAP_COLS);

    for (uint8_t i = 0; i < MAP_ROWS; i++) {
        for (uint8_t j = 0; j < MAP_COLS; j++) {
            /* 被炸毁的墙 -> 空地 */
            if (out_map[i][j] == WALL && !(g_boom_final_walls[i] & (1 << j)))
                out_map[i][j] = FLOOR;

            /* 已使用的炸弹 -> 空地 */
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
 * @brief 重置整个规划系统的所有全局状态
 */
void reset_planning_system(void) {
    /* ---- 4.1 A* 搜索全局缓冲区 ---- */
    memset(hash_table, 0, sizeof(hash_table)); g_hash_epoch = 1;
    memset(pq, 0, sizeof(pq)); pq_size = 0;
    memset(g_succ_buf, 0, sizeof(g_succ_buf));
    for (int i = 0; i < (MAP_ROWS * MAP_COLS * 5); i++) {
        path_nodes[i].g_cost = INF; path_nodes[i].closed = false;
        path_nodes[i].parent_idx = -1; path_nodes[i].last_dir = -1;
    }

    /* ---- 4.2 simple_astar 最小堆 ---- */
    s_heap_size = 0; memset(s_heap, 0, sizeof(s_heap));

    /* ---- 4.3 共享临时缓冲区 ---- */
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
        g_initial_bombs[i].x = BOMB_INVALID;
        g_initial_bombs[i].y = BOMB_INVALID;
    }

    /* ---- 4.5 模式3（炸弹破局）全局缓冲区 ---- */
    memset(g_original_map,        0, sizeof(g_original_map));
    memset(g_sim_queue,           0, sizeof(g_sim_queue));
    memset(g_obs_buf,             0, sizeof(g_obs_buf));
    g_bfs_epoch = 1;  memset(g_bfs_visited, 0, sizeof(g_bfs_visited));
    g_dist_epoch = 1; memset(g_dist_epoch_tag, 0, sizeof(g_dist_epoch_tag));
    g_vis1_epoch = 1; memset(g_vis1, 0, sizeof(g_vis1));
    g_player_region_epoch = 1; memset(g_player_region, 0, sizeof(g_player_region));
    memset(g_problem_points,       0, sizeof(g_problem_points));
    g_problem_count = 0;
    memset(g_saved_problem_points, 0, sizeof(g_saved_problem_points));
    g_saved_problem_count = 0;
    memset(g_bomb_reach_map,       0, sizeof(g_bomb_reach_map));
    memset(g_saved_walls,          0, sizeof(g_saved_walls));
    memset(g_saved_player_region,  0, sizeof(g_saved_player_region));
    memset(g_vcache_hash, 0, sizeof(g_vcache_hash));
    memset(g_vcache_mask, 0, sizeof(g_vcache_mask));

    /* ── 炸弹模式复用缓冲区（阶段一～三）── */
    memset(g_bomb_pb,        0, sizeof(g_bomb_pb));
    memset(g_bomb_t_s,       0, sizeof(g_bomb_t_s));
    memset(g_bomb_best_s,    0, sizeof(g_bomb_best_s));
    memset(g_bomb_candidates,0, sizeof(g_bomb_candidates));
    memset(g_bomb_all_plans, 0, sizeof(g_bomb_all_plans));
    memset(g_bomb_walls,     0, sizeof(g_bomb_walls));
    memset(&g_bomb_ppath,    0, sizeof(g_bomb_ppath));
    memset(&g_bomb_ar,       0, sizeof(g_bomb_ar));
    memset(&g_backtrack_ar,  0, sizeof(g_backtrack_ar));
    memset(g_bomb_saved_map, 0, sizeof(g_bomb_saved_map));
    memset(g_bomb_visited,   0, sizeof(g_bomb_visited));

    /* ── UNR 箱子可达性缓存 ── */
    memset(g_unr_reach_cache, -1, sizeof(g_unr_reach_cache));
    memset(g_tmp_walls1,    0, sizeof(g_tmp_walls1));
    memset(g_tmp_walls2,    0, sizeof(g_tmp_walls2));
    memset(g_tmp_infl,      0, sizeof(g_tmp_infl));
    memset(g_tmp_walls_cur, 0, sizeof(g_tmp_walls_cur));
    memset(g_tmp_obs,       0, sizeof(g_tmp_obs));

    /* ---- 4.6 推箱验证全局状态 ---- */
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