#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#include <windows.h>
#define SLEEP(ms) Sleep(ms)
#else
#include <unistd.h>
#define SLEEP(ms) usleep((ms)*1000)
#endif

// Constants for grid and agent limits
#define MAX_AGENTS 26
#define MAX_GRID 50
#define MAX_PATH 200
// Simple 2D point
typedef struct {
    int x, y;
} Point;
// A path made of multiple points with its length
typedef struct {
    Point path[MAX_PATH];
    int length;
} Path;
// Represents the MAPF instance
typedef struct {
    char map[MAX_GRID][MAX_GRID];  // Grid map
    int width, height;             // Grid dimensions
    int num_agents;                // Number of agents
    Point starts[MAX_AGENTS];     // Start locations
    Point goals[MAX_AGENTS];      // Goal locations
    Path paths[MAX_AGENTS];       // Paths for agents
    int makespan;                 // Total time taken
} Instance;
// Global instance and supporting arrays
Instance inst;
int reserved[MAX_GRID][MAX_GRID][MAX_PATH]; // Space-time reservation grid
int done_agents[MAX_AGENTS];               // Track if agent is done
int finished_time[MAX_AGENTS];             // When each agent finished
char agent_names[MAX_AGENTS];              // Agent labels (e.g., A, B, C)
// Movement directions: up, down, left, right, wait
int dx[] = {-1, 1, 0, 0, 0};
int dy[] = {0, 0, -1, 1, 0};
// Check if (x,y) is valid and not an obstacle
int is_valid(int x, int y) {
    return x >= 0 && x < inst.height && y >= 0 && y < inst.width && inst.map[x][y] != '#';
}
// Reset reservation grid based on current paths
void reset_reserved() {
    memset(reserved, 0, sizeof(reserved));
    for (int a = 0; a < inst.num_agents; a++) {
        for (int t = 0; t < inst.paths[a].length; t++) {
            Point p = inst.paths[a].path[t];
            reserved[p.x][p.y][t] = 1;
        }
    }
}

// Globals for BFS
static Point bfs_parent[MAX_GRID][MAX_GRID][MAX_PATH];
static int bfs_visited[MAX_GRID][MAX_GRID][MAX_PATH];

// BFS pathfinding avoiding conflicts
int bfs(int agent, Point start, Point goal, int start_time, int forbid_x, int forbid_y) {
    typedef struct { Point p; int time; } Node;
    static Node queue[MAX_GRID * MAX_GRID * MAX_PATH];
    int front = 0, back = 0;

    memset(bfs_visited, 0, sizeof(bfs_visited));
    queue[back++] = (Node){start, start_time};
    bfs_visited[start.x][start.y][start_time] = 1;
    bfs_parent[start.x][start.y][start_time] = (Point){-1, -1};

    while (front < back) {
        Node cur = queue[front++];
        // Goal reached
        if (cur.p.x == goal.x && cur.p.y == goal.y) {
            int t = cur.time;
            inst.paths[agent].length = t + 1 - start_time;
            for (int i = t; i >= start_time; i--) {
                inst.paths[agent].path[i - start_time] = cur.p;
                cur.p = bfs_parent[cur.p.x][cur.p.y][i];
            }
            return 1;
        }
        // Try all directions
        for (int d = 0; d < 5; d++) {
            int nx = cur.p.x + dx[d], ny = cur.p.y + dy[d];
            int nt = cur.time + 1;
            if (nt >= MAX_PATH) continue; // Prevent out-of-bounds
            if (!is_valid(nx, ny) || (nx == forbid_x && ny == forbid_y)) continue;
            if (reserved[nx][ny][nt]) continue;
            // Prevent edge swap conflict
            if (d != 4 && cur.time > 0) {
                for (int a = 0; a < inst.num_agents; a++) {
                    if (a == agent) continue;
                    if (cur.time < inst.paths[a].length) {
                        Point other_prev = inst.paths[a].path[cur.time - 1];
                        Point other_now = inst.paths[a].path[cur.time];
                        if (other_prev.x == nx && other_prev.y == ny &&
                            other_now.x == cur.p.x && other_now.y == cur.p.y) {
                            goto skip;
                        }
                    }
                }
            }
            if (!bfs_visited[nx][ny][nt]) {
                bfs_visited[nx][ny][nt] = 1;
                bfs_parent[nx][ny][nt] = cur.p;
                if (back < MAX_GRID * MAX_GRID * MAX_PATH) // Prevent overflow
                    queue[back++] = (Node){{nx, ny}, nt};
            }
        skip:;
        }
    }

    return 0; // No path found
}

// Reserve the path cells for the given agent
void reserve_path(int agent) {
    for (int t = 0; t < inst.paths[agent].length; t++) {
        Point p = inst.paths[agent].path[t];
        reserved[p.x][p.y][t] = 1;
    }
}

// Pad agent path to the same length (wait at goal)
void pad_and_reserve_agent(int agent, int target_len) {
    Point last = inst.paths[agent].path[inst.paths[agent].length - 1];
    for (int t = inst.paths[agent].length; t < target_len; t++) {
        inst.paths[agent].path[t] = last;
        reserved[last.x][last.y][t] = 1;
    }
    inst.paths[agent].length = target_len;
}
// Detect and resolve vertex conflicts
int resolve_conflicts() {
    for (int t = 0; t < MAX_PATH; t++) {
        for (int a1 = 0; a1 < inst.num_agents; a1++) {
            for (int a2 = a1 + 1; a2 < inst.num_agents; a2++) {
                if (t < inst.paths[a1].length && t < inst.paths[a2].length) {
                    Point p1 = inst.paths[a1].path[t];
                    Point p2 = inst.paths[a2].path[t];
                    if (p1.x == p2.x && p1.y == p2.y) {
                        // Replan both agents involved in the conflict
                        reset_reserved();
                        bfs(a1, inst.starts[a1], inst.goals[a1], 0, -1, -1);
                        reserve_path(a1);
                        bfs(a2, inst.starts[a2], inst.goals[a2], 0, -1, -1);
                        reserve_path(a2);
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}
// Get the longest path length
int calculate_makespan() {
    int max = 0;
    for (int a = 0; a < inst.num_agents; a++) {
        if (inst.paths[a].length > max) max = inst.paths[a].length;
    }
    return max;
}
// Print the grid with agents and goals at time t
void print_grid(int t) {
    printf("Time: %d\n", t);
    for (int i = 0; i < inst.height; i++) {
        for (int j = 0; j < inst.width; j++) {
            char ch = inst.map[i][j];
            int printed = 0;
            for (int a = 0; a < inst.num_agents; a++) {
                if (t < inst.paths[a].length &&
                    inst.paths[a].path[t].x == i &&
                    inst.paths[a].path[t].y == j) {
                    putchar(agent_names[a]);
                    printed = 1;
                    break;
                }
            }
            if (!printed) {
                for (int a = 0; a < inst.num_agents; a++) {
                    if (inst.goals[a].x == i && inst.goals[a].y == j) {
                        putchar('+');
                        printed = 1;
                        break;
                    }
                }
            }
            if (!printed) putchar(ch == '#' ? '#' : '.');
        }
        putchar('\n');
    }
    putchar('\n');
}
//STMS Algorithm Execution
void run_stms() {
    memset(done_agents, 0, sizeof(done_agents));
    for (int i = 0; i < MAX_AGENTS; i++) finished_time[i] = -1;
    int stable = 0;
    int attempts = 0;
    const int MAX_ATTEMPTS = 1000; // Limit attempts to avoid infinite loops

    while (!stable && attempts < MAX_ATTEMPTS) {
        attempts++;
        reset_reserved();

        // Plan and reserve for each agent, padding and reserving goal after arrival
        int max_len = 0;
        for (int i = 0; i < inst.num_agents; i++) {
            bfs(i, inst.starts[i], inst.goals[i], 0, -1, -1);
            reserve_path(i);
            if (inst.paths[i].length > max_len) max_len = inst.paths[i].length;
        }
        // Now pad and reserve goal for all agents up to max_len
        for (int i = 0; i < inst.num_agents; i++) {
            pad_and_reserve_agent(i, max_len);
        }

        // Mark finished agents and print when they finish (fix: only when they arrive and stay)
        for (int a = 0; a < inst.num_agents; a++) {
            if (!done_agents[a]) {
                int t;
                int found = 0;
                for (t = 0; t < inst.paths[a].length; t++) {
                    Point p = inst.paths[a].path[t];
                    if (p.x == inst.goals[a].x && p.y == inst.goals[a].y) {
                        // Check if agent stays at goal for all remaining timesteps
                        int stays = 1;
                        for (int k = t+1; k < inst.paths[a].length; k++) {
                            Point q = inst.paths[a].path[k];
                            if (q.x != inst.goals[a].x || q.y != inst.goals[a].y) {
                                stays = 0;
                                break;
                            }
                        }
                        // Only print and mark as finished if this is the *first* time agent arrives and stays,
                        // and the agent was NOT at the goal at t-1 (or t==0)
                        if (stays && (t == 0 || inst.paths[a].path[t-1].x != inst.goals[a].x || inst.paths[a].path[t-1].y != inst.goals[a].y)) {
                            done_agents[a] = 1;
                            finished_time[a] = t;
                            found = 1;
                            break;
                        }
                    }
                }
                // Print finish message outside the loop, only once
                if (found) {
                    printf("Agent %c finished at time %d\n", agent_names[a], finished_time[a]);
                }
            }
        }

        // Check for conflicts (vertex and edge)
        int conflict_found = 0;
        for (int t = 0; t < max_len && !conflict_found; t++) {
            // Store previous coordinates for unfinished agents
            Point prev_coords[MAX_AGENTS];
            if (t > 0) {
                for (int a = 0; a < inst.num_agents; a++) {
                    prev_coords[a] = inst.paths[a].path[t-1];
                }
            }
            for (int a1 = 0; a1 < inst.num_agents && !conflict_found; a1++) {
                for (int a2 = a1 + 1; a2 < inst.num_agents && !conflict_found; a2++) {
                    Point p1 = inst.paths[a1].path[t];
                    Point p2 = inst.paths[a2].path[t];
                    // Vertex conflict
                    if (p1.x == p2.x && p1.y == p2.y) {
                        printf("WARNING: Agents %c and %c occupy the same space (%d,%d) at time %d\n", agent_names[a1], agent_names[a2], p1.x, p1.y, t);
                        // Print previous coordinates of all unfinished agents
                        printf("Previous coordinates of unfinished agents:\n");
                        for (int a = 0; a < inst.num_agents; a++) {
                            if (!done_agents[a] && t > 0) {
                                printf("Agent %c: (%d,%d)\n", agent_names[a], prev_coords[a].x, prev_coords[a].y);
                            }
                        }
                        conflict_found = 1;
                    }
                    // Edge conflict
                    if (t > 0 && !conflict_found) {
                        Point p1_prev = inst.paths[a1].path[t-1];
                        Point p2_prev = inst.paths[a2].path[t-1];
                        if (p1.x == p2_prev.x && p1.y == p2_prev.y &&
                            p2.x == p1_prev.x && p2.y == p1_prev.y) {
                            printf("WARNING: Agents %c and %c swap positions between (%d,%d) and (%d,%d) at time %d\n",
                                agent_names[a1], agent_names[a2], p1_prev.x, p1_prev.y, p2_prev.x, p2_prev.y, t);
                            printf("Previous coordinates of unfinished agents:\n");
                            for (int a = 0; a < inst.num_agents; a++) {
                                if (!done_agents[a] && t > 0) {
                                    printf("Agent %c: (%d,%d)\n", agent_names[a], prev_coords[a].x, prev_coords[a].y);
                                }
                            }
                            conflict_found = 1;
                        }
                    }
                }
            }
            // Check for unfinished agent occupying goal of a finished agent
            for (int a = 0; a < inst.num_agents && !conflict_found; a++) {
                if (!done_agents[a]) {
                    Point p = inst.paths[a].path[t];
                    for (int f = 0; f < inst.num_agents; f++) {
                        if (done_agents[f] && p.x == inst.goals[f].x && p.y == inst.goals[f].y) {
                            printf("WARNING: Agent %c occupies the goal of finished agent %c at (%d,%d) at time %d\n",
                                agent_names[a], agent_names[f], p.x, p.y, t);
                            printf("Previous coordinates of unfinished agents:\n");
                            for (int u = 0; u < inst.num_agents; u++) {
                                if (!done_agents[u] && t > 0) {
                                    printf("Agent %c: (%d,%d)\n", agent_names[u], prev_coords[u].x, prev_coords[u].y);
                                }
                            }
                            conflict_found = 1;
                            break;
                        }
                    }
                }
            }
        }

        if (!conflict_found) {
            stable = 1;
            inst.makespan = max_len;
            // Final pad to makespan (should already be done)
        }
        // else: loop again, replanning all agents
    }
    if (attempts == MAX_ATTEMPTS) {
        printf("Failed to find conflict-free paths after %d attempts.\n", MAX_ATTEMPTS);
        inst.makespan = calculate_makespan();
        for (int i = 0; i < inst.num_agents; i++) {
            pad_and_reserve_agent(i, inst.makespan);
        }
    }
}
// Visualize the paths and conflicts
void visualize() {
    for (int t = 0; t < inst.makespan; t++) {
        print_grid(t);
        // Print when an agent finishes at this timestep
        for (int a = 0; a < inst.num_agents; a++) {
            if (finished_time[a] == t) {
                printf("Agent %c finished at time %d\n", agent_names[a], t);
            }
        }
        // --- Add warnings during visualization ---
        // Store previous coordinates for unfinished agents
        Point prev_coords[MAX_AGENTS];
        if (t > 0) {
            for (int a = 0; a < inst.num_agents; a++) {
                prev_coords[a] = inst.paths[a].path[t-1];
            }
        }
        // Check for two agents occupying the same cell
        for (int a1 = 0; a1 < inst.num_agents; a1++) {
            for (int a2 = a1 + 1; a2 < inst.num_agents; a2++) {
                Point p1 = inst.paths[a1].path[t];
                Point p2 = inst.paths[a2].path[t];
                if (p1.x == p2.x && p1.y == p2.y) {
                    printf("WARNING: Agents %c and %c occupy the same space (%d,%d) at time %d\n", agent_names[a1], agent_names[a2], p1.x, p1.y, t);
                    printf("Previous coordinates of unfinished agents:\n");
                    for (int a = 0; a < inst.num_agents; a++) {
                        if (!done_agents[a] && t > 0) {
                            printf("Agent %c: (%d,%d)\n", agent_names[a], prev_coords[a].x, prev_coords[a].y);
                        }
                    }
                }
            }
        }
        // Check for unfinished agent occupying goal of a finished agent
        for (int a = 0; a < inst.num_agents; a++) {
            if (!done_agents[a]) {
                Point p = inst.paths[a].path[t];
                for (int f = 0; f < inst.num_agents; f++) {
                    if (done_agents[f] && p.x == inst.goals[f].x && p.y == inst.goals[f].y) {
                        printf("WARNING: Agent %c occupies the goal of finished agent %c at (%d,%d) at time %d\n",
                            agent_names[a], agent_names[f], p.x, p.y, t);
                        printf("Previous coordinates of unfinished agents:\n");
                        for (int u = 0; u < inst.num_agents; u++) {
                            if (!done_agents[u] && t > 0) {
                                printf("Agent %c: (%d,%d)\n", agent_names[u], prev_coords[u].x, prev_coords[u].y);
                            }
                        }
                    }
                }
            }
        }
        SLEEP(500); // 500 ms
    }
}
// Load the instance data
void load_instance() {
    char *grid[] = {
        "...........",
        ".#.#.#.#.#.",
        "...........",
        ".#.#.#.#.#.",
        "...........",
        ".#.#.#.#.#.",
        "...........",
        ".#.#.#.#.#.",
        "...........",
        ".#.#.#.#.#.",
        "..........."
        
    };

    inst.height = 11;
    inst.width = strlen(grid[0]);

    for (int i = 0; i < inst.height; i++)
        strcpy(inst.map[i], grid[i]);

    inst.num_agents = 5;
    // Set agent names (customize as needed)
    agent_names[0] = 'A';
    agent_names[1] = 'B';
    agent_names[2] = 'C';
    agent_names[3] = 'D';
    agent_names[4] = 'E';

    inst.starts[0] = (Point){5, 1}; inst.goals[0] = (Point){5, 10};
    inst.starts[1] = (Point){3, 3}; inst.goals[1] = (Point){10, 7};
    inst.starts[2] = (Point){7, 5}; inst.goals[2] = (Point){5, 0};
    inst.starts[3] = (Point){1, 7}; inst.goals[3] = (Point){10, 0};
    inst.starts[4] = (Point){9, 9}; inst.goals[4] = (Point){10, 3};
    inst.makespan = 0; // Initialize makespan
}
// Main function to run the STMS algorithm
int main() {
    srand(time(NULL));
    load_instance();
    run_stms();
    visualize();
    return 0;
}
