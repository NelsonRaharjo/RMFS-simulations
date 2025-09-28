#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

// Constants
#define MAX_AGENTS 26
#define MAX_TIME 100
#define MAX_GRID 50

// Position struct
typedef struct {
    int x, y;
} Pos;

// Agent struct
typedef struct {
    char id;              // Identifier (A-Z)
    Pos start, goal;      // Start and goal positions
    Pos path[MAX_TIME];   // Path taken
    int path_len;         // Path length
    bool active;          // If the agent is active
} Agent;

// Queue node for BFS
typedef struct QueueNode {
    Pos pos;
    int time;
} QueueNode;

// Global variables
int width = 10, height = 10, agent_count = 0;
char map[MAX_GRID][MAX_GRID];           // Map grid
Agent agents[MAX_AGENTS];               // All agents

// Space-time occupancy: [Time][Row][Col]
char occupancy[MAX_TIME][MAX_GRID][MAX_GRID];

// Directions: Up, Right, Down, Left, Wait
int dx[] = {-1, 0, 1, 0, 0};
int dy[] = {0, 1, 0, -1, 0};

// Preallocated large arrays to avoid stack overflow
static Pos parent[MAX_TIME][MAX_GRID][MAX_GRID];
static bool visited[MAX_TIME][MAX_GRID][MAX_GRID];
static QueueNode queue[MAX_GRID * MAX_GRID * MAX_TIME];

// Checks if position is within bounds and not a wall
bool is_valid(int x, int y) {
    return x >= 0 && y >= 0 && x < height && y < width && map[x][y] != '#';
}

// Checks if a cell is free at time `t`
bool is_free(int t, int x, int y) {
    if (t >= MAX_TIME) return false;
    // Prevent moving into any cell occupied by any agent at any time
    for (int i = 0; i < agent_count; i++) {
        if (agents[i].path_len > 0) {
            int goal_time = agents[i].path_len - 1;
            int gx = agents[i].goal.x, gy = agents[i].goal.y;
            // After agent reaches its goal, it stays at its goal cell forever
            if (t >= goal_time && x == gx && y == gy)
                return false;
            if (t < agents[i].path_len && x == agents[i].path[t].x && y == agents[i].path[t].y)
                return false; // Agent occupies the cell at this time
        }
    }
    return map[x][y] != '#';
}

// Prevent two agents from swapping positions in the same time step
bool is_swap_conflict(int t, int from_x, int from_y, int to_x, int to_y) {
    if (t <= 0 || t >= MAX_TIME) return false;
    char prev = occupancy[t-1][to_x][to_y];
    // Only check if the previous cell was occupied by an agent 
    if (prev != '.' && prev != '#' && prev != occupancy[t][to_x][to_y]) {
        // Check if that agent is moving to our current cell at this timestep
        // That is, at time t, is the agent that was at (to_x, to_y) now at (from_x, from_y)?
        if (occupancy[t][from_x][from_y] == prev) {
            return true;
        }
    }
    return false;
}

// Mark occupancy of agent path in space-time grid
void set_occupancy(Agent *a) {
    for (int t = 0; t < a->path_len; t++) {
        Pos p = a->path[t];
        occupancy[t][p.x][p.y] = a->id;
    }
    // Block goal cell after arrival with agent's ID (not '#')
    Pos g = a->path[a->path_len - 1];
    for (int t = a->path_len; t < MAX_TIME; t++) {
        occupancy[t][g.x][g.y] = a->id;
    }
}
// BFS with time-expanded search to find space-time paths for an agent
bool bfs(Agent *a) {
    // Clear visited and parent arrays
    for (int t = 0; t < MAX_TIME; t++)
        for (int i = 0; i < MAX_GRID; i++)
            for (int j = 0; j < MAX_GRID; j++) {
                visited[t][i][j] = false;
                parent[t][i][j] = (Pos){-1, -1};
            }

    int front = 0, rear = 0;
    queue[rear++] = (QueueNode){a->start, 0};
    visited[0][a->start.x][a->start.y] = true;
    parent[0][a->start.x][a->start.y] = (Pos){-1, -1};

    while (front < rear) {
        QueueNode curr = queue[front++];
        if (curr.pos.x == a->goal.x && curr.pos.y == a->goal.y) {
            // Reconstruct path
            a->path_len = curr.time + 1;
            for (int t = curr.time; t >= 0; t--) {
                a->path[t] = curr.pos;
                curr.pos = parent[t][curr.pos.x][curr.pos.y];
            }
            return true;
        }
        // Try all 5 directions including wait
        for (int d = 0; d < 5; d++) {
            int nx = curr.pos.x + dx[d], ny = curr.pos.y + dy[d];
            int nt = curr.time + 1;
            if (!is_valid(nx, ny)) continue;
            if (!is_free(nt, nx, ny)) continue;
            // Prevent swapping (edge) conflict
            if (is_swap_conflict(nt, curr.pos.x, curr.pos.y, nx, ny)) continue;
            if (visited[nt][nx][ny]) continue;
            visited[nt][nx][ny] = true;
            parent[nt][nx][ny] = curr.pos;
            if (rear < MAX_GRID * MAX_GRID * MAX_TIME) // Prevent queue overflow
                queue[rear++] = (QueueNode){(Pos){nx, ny}, nt};
        }
    }
    return false;
}
// Print current state of map and agent positions
void print_map(int t) {
    char visual[MAX_GRID][MAX_GRID];
    // Only copy the relevant portion of the map
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            visual[i][j] = map[i][j];

    // Mark all goals with '+'
    for (int i = 0; i < agent_count; i++) {
        visual[agents[i].goal.x][agents[i].goal.y] = '+';
    }

    // Show agent positions (including at goal after finished)
    for (int i = 0; i < agent_count; i++) {
        if (t < agents[i].path_len) {
            visual[agents[i].path[t].x][agents[i].path[t].y] = agents[i].id;
        } else if (agents[i].path_len > 0) {
            visual[agents[i].goal.x][agents[i].goal.y] = agents[i].id;
        }
    }

    printf("\nTime %d:\n", t);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++)
            printf("%c", visual[i][j]);
        printf("\n");
    }
}
// Print agent positions at a specific time
void print_agents_positions(int t) {
    printf("Agents' positions at time %d:\n", t);
    for (int i = 0; i < agent_count; i++) {
        int ax, ay;
        if (t < agents[i].path_len) {
            ax = agents[i].path[t].x;
            ay = agents[i].path[t].y;
        } else {
            ax = agents[i].goal.x;
            ay = agents[i].goal.y;
        }
        printf("Agent %c: (%d, %d)\n", agents[i].id, ax, ay);
    }
}
// Run the full simulation and report warnings if agents collide
void simulate() {
    int max_time = 0;
    bool goal_announced[MAX_AGENTS] = {0};
    // Determine the latest time step any agent finishes
    for (int i = 0; i < agent_count; i++)
        if (agents[i].path_len > max_time)
            max_time = agents[i].path_len;
    // Simulate each timestep
    for (int t = 0; t < max_time; t++) {
        // Notify when agent reaches its goal
        for (int i = 0; i < agent_count; i++) {
            if (!goal_announced[i]) {
                int ax, ay;
                if (t < agents[i].path_len) {
                    ax = agents[i].path[t].x;
                    ay = agents[i].path[t].y;
                } else {
                    ax = agents[i].goal.x;
                    ay = agents[i].goal.y;
                }
                if (ax == agents[i].goal.x && ay == agents[i].goal.y) {
                    printf("Agent %c reached its goal at time %d.\n", agents[i].id, t);
                    goal_announced[i] = true;
                }
            }
        }
        // Collision detection
        for (int i = 0; i < agent_count; i++) {
            int ax, ay;
            if (t < agents[i].path_len) {
                ax = agents[i].path[t].x;
                ay = agents[i].path[t].y;
            } else {
                ax = agents[i].goal.x;
                ay = agents[i].goal.y;
            }
            for (int j = i + 1; j < agent_count; j++) {
                int bx, by;
                if (t < agents[j].path_len) {
                    bx = agents[j].path[t].x;
                    by = agents[j].path[t].y;
                } else {
                    bx = agents[j].goal.x;
                    by = agents[j].goal.y;
                }
                if (ax == bx && ay == by) {
                    printf("WARNING: Agents %c and %c occupy the same cell (%d, %d) at time %d!\n", agents[i].id, agents[j].id, ax, ay, t);
                    // Print positions at the timestep before the collision
                    if (t > 0) {
                        printf("Positions before collision (time %d):\n", t-1);
                        print_agents_positions(t-1);
                    } else {
                        printf("No previous timestep before collision.\n");
                    }
                }
            }
        }
        print_map(t);
    }
}

int main() {
    // Define map
    char *raw[] = {
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
    height = 11;
    width = strlen(raw[0]);

    for (int i = 0; i < height; i++)
        strcpy(map[i], raw[i]);

    // Init occupancy: '.' for free, '#' for wall
    for (int t = 0; t < MAX_TIME; t++)
        for (int i = 0; i < height; i++)
            for (int j = 0; j < width; j++)
                occupancy[t][i][j] = (map[i][j] == '#') ? '#' : '.';

    // Setup agents (initialize all fields)
    agent_count = 5;
    agents[0] = (Agent){ .id='A', .start={1,1}, .goal={10,9}, .path={{0}}, .path_len=0, .active=true };
    agents[1] = (Agent){ .id='B', .start={1,3}, .goal={10,7}, .path={{0}}, .path_len=0, .active=true };
    agents[2] = (Agent){ .id='C', .start={1,5}, .goal={10,5}, .path={{0}}, .path_len=0, .active=true };
    agents[3] = (Agent){ .id='D', .start={1,7}, .goal={10,3}, .path={{0}}, .path_len=0, .active=true };
    agents[4] = (Agent){ .id='E', .start={1,9}, .goal={10,1}, .path={{0}}, .path_len=0, .active=true };

    // ST-SPF: Plan each agent sequentially
    for (int i = 0; i < agent_count; i++) {
        bool success = bfs(&agents[i]);
        if (!success) {
            printf("Agent %c: no path found!\n", agents[i].id);
            return 1;
        }
        set_occupancy(&agents[i]);
    }

    simulate();
    return 0;
}