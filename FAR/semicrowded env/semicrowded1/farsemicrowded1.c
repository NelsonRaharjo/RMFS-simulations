#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <limits.h>
#include <unistd.h>
// Maximum number of agents and map size
#define MAX_AGENTS 26
#define MAX_MAP 32
// Struct to represent coordinates
typedef struct {
    int x, y;
} Point;
// Struct to represent an agent with ID, start/goal/position, and statu
typedef struct {
    char id;
    Point start, goal, pos;
    bool done;
} Agent;
// Node for A* search tree
typedef struct Node {
    Point pt;
    int cost, priority;
    struct Node *parent;
} Node;

// Global variables
char map[MAX_MAP][MAX_MAP];      // Static map layout
char display[MAX_MAP][MAX_MAP];  // Visual map with agents
int width = 10, height = 10;
Agent agents[MAX_AGENTS];        // Agent list
int agent_count = 0;
int density[MAX_MAP][MAX_MAP] = {0}; // Tracks congestion for deadlock resolution
// Directions (up, down, left, right)
int dx[4] = {0, 0, -1, 1};
int dy[4] = {-1, 1, 0, 0};
// Check if (x, y) is within bounds and not a wall
bool is_valid(int x, int y) {
    return x >= 0 && y >= 0 && x < width && y < height && map[y][x] != '#';
}
// Check if (x, y) is occupied by any agent except the one at `exclude` index
bool is_occupied(int x, int y, int exclude) {
    for (int i = 0; i < agent_count; ++i)
        if (i != exclude && agents[i].pos.x == x && agents[i].pos.y == y)
            return true;
    return false;
}
// Add penalty if movement contradicts flow direction
int flow_penalty(Point from, Point to) {
    int penalty = 0;
    if (from.y % 2 == 0 && to.x < from.x) penalty += 2;
    if (from.y % 2 == 1 && to.x > from.x) penalty += 2;
    if (from.x % 2 == 0 && to.y < from.y) penalty += 2;
    if (from.x % 2 == 1 && to.y > from.y) penalty += 2;
    return penalty;
}
// Manhattan distance heuristic for A*
int heuristic(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}
// Create a new node for A* search
Node *new_node(int x, int y, int cost, int priority, Node *parent) {
    Node *n = malloc(sizeof(Node));
    n->pt = (Point){x, y};
    n->cost = cost;
    n->priority = priority;
    n->parent = parent;
    return n;
}
// A* search algorithm for an agent
Node *a_star(Agent *a) {
    Node *open[1024];
    bool closed[MAX_MAP][MAX_MAP] = { false };
    int open_len = 0;

    Node *start = new_node(a->pos.x, a->pos.y, 0, heuristic(a->pos, a->goal), NULL);
    open[open_len++] = start;

    while (open_len) {
        // Get node with lowest priority (best path estimate)
        int min_i = 0;
        for (int i = 1; i < open_len; i++)
            if (open[i]->priority < open[min_i]->priority)
                min_i = i;

        Node *curr = open[min_i];
        open[min_i] = open[--open_len];
        // Reached goal
        if (curr->pt.x == a->goal.x && curr->pt.y == a->goal.y)
            return curr;

        if (closed[curr->pt.y][curr->pt.x]) {
            free(curr);
            continue;
        }
        closed[curr->pt.y][curr->pt.x] = true;
        // Explore neighbors
        for (int d = 0; d < 4; ++d) {
            int nx = curr->pt.x + dx[d], ny = curr->pt.y + dy[d];
            if (!is_valid(nx, ny)) continue;
            if (is_occupied(nx, ny, a - agents) && !(nx == a->goal.x && ny == a->goal.y)) continue;

            Point next = {nx, ny};
            int cost = curr->cost + 1;
            int priority = cost + heuristic(next, a->goal) + flow_penalty(curr->pt, next);
            Node *child = new_node(nx, ny, cost, priority, curr);
            open[open_len++] = child;
        }
    }

    return NULL;// No path found
}
// Display map and agents at current timestep
void visualize_with_timestep(int timestep) {
    memcpy(display, map, sizeof(map));
    for (int i = 0; i < agent_count; ++i)
        display[agents[i].goal.y][agents[i].goal.x] = '+';
    for (int i = 0; i < agent_count; ++i)
        display[agents[i].pos.y][agents[i].pos.x] = agents[i].id;
    printf("\nTimestep: %d\n", timestep);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x)
            putchar(display[y][x]);
        putchar('\n');
    }
    usleep(300000); // Delay for animation
}

// Recursively detect cycles in blocking chain
int find_blocking_chain(int idx, bool visited[]) {
    if (visited[idx]) return idx;
    visited[idx] = true;

    Agent *a = &agents[idx];
    Node *path = a_star(a);
    if (!path) return -1;

    Node *step = path;
    while (step->parent && step->parent->parent) step = step->parent;
    int tx = step->pt.x, ty = step->pt.y;

    for (int i = 0; i < agent_count; i++) {
        if (i != idx && agents[i].pos.x == tx && agents[i].pos.y == ty && !agents[i].done)
            return find_blocking_chain(i, visited);
    }
    return -1;
}
// Deadlock resolution by moving to least crowded valid cell
void resolve_deadlock(int agent_idx) {
    Agent *a = &agents[agent_idx];
    int best_dir = -1, best_score = INT_MIN;

    for (int d = 0; d < 4; ++d) {
        int nx = a->pos.x + dx[d], ny = a->pos.y + dy[d];
        if (!is_valid(nx, ny)) continue;
        if (is_occupied(nx, ny, agent_idx)) continue;

        int score = -density[ny][nx]; // move to lower congestion
        if (score > best_score) {
            best_score = score;
            best_dir = d;
        }
    }

    if (best_dir != -1) {
        int x = a->pos.x, y = a->pos.y;
        density[y][x]--;
        a->pos.x += dx[best_dir];
        a->pos.y += dy[best_dir];
        density[a->pos.y][a->pos.x]++;
    }
}
// Simulate all agents step-by-step until all reach goals
void simulate() {
    bool changed;
    int timestep = 0;
    visualize_with_timestep(timestep); // Initial state
    do {
        changed = false;

        for (int i = 0; i < agent_count; ++i) {
            if (agents[i].done) continue;

            Node *path = a_star(&agents[i]);
            if (!path) continue;

            Node *step = path;
            while (step->parent && step->parent->parent) step = step->parent;

            // Warning checks
            for (int j = 0; j < agent_count; ++j) {
                if (i != j && agents[j].pos.x == step->pt.x && agents[j].pos.y == step->pt.y && !agents[j].done) {
                    printf("Warning: Agent %c is trying to move to a position occupied by Agent %c at timestep %d\n",
                        agents[i].id, agents[j].id, timestep + 1);
                }
            }
            for (int j = 0; j < agent_count; ++j) {
                if (i != j && agents[j].pos.x == step->pt.x && agents[j].pos.y == step->pt.y && agents[j].done) {
                    printf("Warning: Agent %c is moving over the goal position of finished Agent %c at timestep %d\n",
                        agents[i].id, agents[j].id, timestep + 1);
                }
            }
            // No movement needed
            if (step->pt.x == agents[i].pos.x && step->pt.y == agents[i].pos.y)
                continue;
            // Detect potential deadlock
            int target_idx = -1;
            for (int j = 0; j < agent_count; ++j)
                if (i != j && agents[j].pos.x == step->pt.x && agents[j].pos.y == step->pt.y && !agents[j].done)
                    target_idx = j;

            if (target_idx != -1) {
                bool visited[MAX_AGENTS] = {false};
                int cycle = find_blocking_chain(i, visited);
                if (cycle != -1) {
                    resolve_deadlock(cycle);
                    while (path) {
                        Node *prev = path;
                        path = path->parent;
                        free(prev);
                    }
                    continue;
                }
            }
            // Perform move
            density[agents[i].pos.y][agents[i].pos.x]--;
            agents[i].pos = step->pt;
            density[agents[i].pos.y][agents[i].pos.x]++;
            if (agents[i].pos.x == agents[i].goal.x && agents[i].pos.y == agents[i].goal.y) {
                agents[i].done = true;
                printf("Agent %c finished at timestep %d\n", agents[i].id, timestep + 1);
            }
            changed = true;
            // Free path memory
            while (path) {
                Node *prev = path;
                path = path->parent;
                free(prev);
            }
        }

        timestep++;
        visualize_with_timestep(timestep);
    } while (changed); // Repeat until no agent moves
}
// Map initialization
void setup_map() {
    const char *rows[] = {
        ".......",
        ".#.#.#.",
        ".......",
        ".#.#.#.",
        ".......",
        ".#.#.#.",
        "......."
    };
    height = 7;
    width = strlen(rows[0]);
    for (int y = 0; y < height; y++)
        strcpy(map[y], rows[y]);
}
// Initialize agents with positions and goals
void setup_agents() {
    agents[0] = (Agent){'A', {1, 1}, {6, 6}, {1, 1}, false};
    agents[1] = (Agent){'B', {1, 5}, {6, 0}, {1, 5}, false};
    agents[2] = (Agent){'C', {3, 1}, {0, 3}, {3, 1}, false};
    agents[3] = (Agent){'D', {3, 3}, {3, 0}, {3, 3}, false};
    agents[4] = (Agent){'E', {3, 5}, {6, 3}, {3, 5}, false};
    agents[5] = (Agent){'F', {5, 1}, {0, 6}, {5, 1}, false};
    agents[6] = (Agent){'G', {5, 5}, {0, 0}, {5, 5}, false};
    
    agent_count = 7;
    // Update congestion density
    for (int i = 0; i < agent_count; ++i)
        density[agents[i].pos.y][agents[i].pos.x]++;
}

int main() {
    setup_map();
    setup_agents();
    visualize_with_timestep(0);
    simulate();
    printf("All agents reached their goals.\n");
    return 0;
}
