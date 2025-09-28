//when a warning occur, will recallculate simulation with the finnised agent as a wall
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define MAX_AGENTS 26
#define MAX_TIME 100
#define MAX_GRID 50

typedef struct {
    int x, y;
} Pos;

typedef struct {
    char id;
    Pos start, goal;
    Pos path[MAX_TIME];
    int path_len;
    bool active;
} Agent;

typedef struct QueueNode {
    Pos pos;
    int time;
} QueueNode;

int width = 10, height = 10, agent_count = 0;
char map[MAX_GRID][MAX_GRID];
Agent agents[MAX_AGENTS];

// Space-time occupancy: [time][row][col]
char occupancy[MAX_TIME][MAX_GRID][MAX_GRID];

// Directions (up, right, down, left, wait)
int dx[] = {-1, 0, 1, 0, 0};
int dy[] = {0, 1, 0, -1, 0};

// Move these arrays to global scope to avoid stack overflow
static Pos parent[MAX_TIME][MAX_GRID][MAX_GRID];
static bool visited[MAX_TIME][MAX_GRID][MAX_GRID];
static QueueNode queue[MAX_GRID * MAX_GRID * MAX_TIME];

bool is_valid(int x, int y) {
    return x >= 0 && y >= 0 && x < height && y < width && map[x][y] != '#';
}

bool is_free(int t, int x, int y) {
    if (t >= MAX_TIME) return false;
    // Prevent moving into any cell occupied by any agent at any time (including after they finish)
    for (int i = 0; i < agent_count; i++) {
        if (agents[i].path_len > 0) {
            int goal_time = agents[i].path_len - 1;
            int gx = agents[i].goal.x, gy = agents[i].goal.y;
            // After agent reaches its goal, it stays at its goal cell forever
            if (t >= goal_time && x == gx && y == gy)
                return false;
            // Otherwise, block the cell the agent occupies at this time
            if (t < agents[i].path_len && x == agents[i].path[t].x && y == agents[i].path[t].y)
                return false;
        }
    }
    // Only allow if cell is not a wall
    return map[x][y] != '#';
}

// Improved swap conflict check: prevent two agents from swapping positions at the same timestep
bool is_swap_conflict(int t, int from_x, int from_y, int to_x, int to_y) {
    if (t <= 0 || t >= MAX_TIME) return false;
    char prev = occupancy[t-1][to_x][to_y];
    // Only check if the previous cell was occupied by an agent (not wall or empty)
    if (prev != '.' && prev != '#' && prev != occupancy[t][to_x][to_y]) {
        // Check if that agent is moving to our current cell at this timestep
        // That is, at time t, is the agent that was at (to_x, to_y) now at (from_x, from_y)?
        if (occupancy[t][from_x][from_y] == prev) {
            return true;
        }
    }
    return false;
}

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

void simulate() {
    int max_time = 0;
    for (int i = 0; i < agent_count; i++)
        if (agents[i].path_len > max_time)
            max_time = agents[i].path_len;

    for (int t = 0; t < max_time; t++) {
        // Check for agents occupying the same cell
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
        ".......",
        ".#.#.#.",
        ".......",
        ".#.#.#.",
        ".......",
        ".#.#.#.",
        "......."
    };
    height = 7;
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
    agents[0] = (Agent){ .id='q', .start={1,1}, .goal={6,6}, .path={{0}}, .path_len=0, .active=true };
    agents[1] = (Agent){ .id='B', .start={5,1}, .goal={0,6}, .path={{0}}, .path_len=0, .active=true };
    agents[2] = (Agent){ .id='C', .start={1,5}, .goal={6,0}, .path={{0}}, .path_len=0, .active=true };
    agents[3] = (Agent){ .id='D', .start={5,5}, .goal={0,0}, .path={{0}}, .path_len=0, .active=true };
    agents[4] = (Agent){ .id='E', .start={3,3}, .goal={0,3}, .path={{0}}, .path_len=0, .active=true };
    // agents[2] = (Agent){ .id='C', .start={5,0}, .goal={5,9}, .path={{0}}, .path_len=0, .active=true };
    // inst.starts[0] = (Point){1, 1}; inst.goals[0] = (Point){6, 6};
    // inst.starts[1] = (Point){5, 1}; inst.goals[1] = (Point){0, 6};
    // inst.starts[2] = (Point){1, 5}; inst.goals[2] = (Point){6, 0};
    // inst.starts[3] = (Point){5, 5}; inst.goals[3] = (Point){0, 0};
    // inst.starts[4] = (Point){3, 3}; inst.goals[4] = (Point){0, 3};
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