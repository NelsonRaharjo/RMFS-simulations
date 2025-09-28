#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <limits.h>
#include <unistd.h>

#define MAX_AGENTS 26
#define MAX_MAP 32

typedef struct {
    int x, y;
} Point;

typedef struct {
    char id;
    Point start, goal, pos;
    bool done;
} Agent;

typedef struct Node {
    Point pt;
    int cost, priority;
    struct Node *parent;
} Node;

char map[MAX_MAP][MAX_MAP];
char display[MAX_MAP][MAX_MAP];
int width = 10, height = 10;
Agent agents[MAX_AGENTS];
int agent_count = 0;
int density[MAX_MAP][MAX_MAP] = {0}; // For deadlock resolution

int dx[4] = {0, 0, -1, 1};
int dy[4] = {-1, 1, 0, 0};

bool is_valid(int x, int y) {
    return x >= 0 && y >= 0 && x < width && y < height && map[y][x] != '#';
}

bool is_occupied(int x, int y, int exclude) {
    for (int i = 0; i < agent_count; ++i)
        if (i != exclude && agents[i].pos.x == x && agents[i].pos.y == y)
            return true;
    return false;
}

int flow_penalty(Point from, Point to) {
    int penalty = 0;
    if (from.y % 2 == 0 && to.x < from.x) penalty += 2;
    if (from.y % 2 == 1 && to.x > from.x) penalty += 2;
    if (from.x % 2 == 0 && to.y < from.y) penalty += 2;
    if (from.x % 2 == 1 && to.y > from.y) penalty += 2;
    return penalty;
}

int heuristic(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

Node *new_node(int x, int y, int cost, int priority, Node *parent) {
    Node *n = malloc(sizeof(Node));
    n->pt = (Point){x, y};
    n->cost = cost;
    n->priority = priority;
    n->parent = parent;
    return n;
}

Node *a_star(Agent *a) {
    Node *open[1024];
    bool closed[MAX_MAP][MAX_MAP] = { false };
    int open_len = 0;

    Node *start = new_node(a->pos.x, a->pos.y, 0, heuristic(a->pos, a->goal), NULL);
    open[open_len++] = start;

    while (open_len) {
        int min_i = 0;
        for (int i = 1; i < open_len; i++)
            if (open[i]->priority < open[min_i]->priority)
                min_i = i;

        Node *curr = open[min_i];
        open[min_i] = open[--open_len];

        if (curr->pt.x == a->goal.x && curr->pt.y == a->goal.y)
            return curr;

        if (closed[curr->pt.y][curr->pt.x]) {
            free(curr);
            continue;
        }
        closed[curr->pt.y][curr->pt.x] = true;

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

    return NULL;
}

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
    usleep(300000);
}

// Deadlock detection: returns agent index cycle or -1
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

void simulate() {
    bool changed;
    int timestep = 0;
    visualize_with_timestep(timestep); // Show initial state at timestep 0
    do {
        changed = false;

        for (int i = 0; i < agent_count; ++i) {
            if (agents[i].done) continue;

            Node *path = a_star(&agents[i]);
            if (!path) continue;

            Node *step = path;
            while (step->parent && step->parent->parent) step = step->parent;

            // Warning: agent in the same position as another agent
            for (int j = 0; j < agent_count; ++j) {
                if (i != j && agents[j].pos.x == step->pt.x && agents[j].pos.y == step->pt.y && !agents[j].done) {
                    printf("Warning: Agent %c is trying to move to a position occupied by Agent %c at timestep %d\n",
                        agents[i].id, agents[j].id, timestep + 1);
                }
            }

            // Warning: agent moves over a finished agent's space
            for (int j = 0; j < agent_count; ++j) {
                if (i != j && agents[j].pos.x == step->pt.x && agents[j].pos.y == step->pt.y && agents[j].done) {
                    printf("Warning: Agent %c is moving over the goal position of finished Agent %c at timestep %d\n",
                        agents[i].id, agents[j].id, timestep + 1);
                }
            }

            if (step->pt.x == agents[i].pos.x && step->pt.y == agents[i].pos.y)
                continue;

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

            density[agents[i].pos.y][agents[i].pos.x]--;
            agents[i].pos = step->pt;
            density[agents[i].pos.y][agents[i].pos.x]++;
            if (agents[i].pos.x == agents[i].goal.x && agents[i].pos.y == agents[i].goal.y) {
                agents[i].done = true;
                printf("Agent %c finished at timestep %d\n", agents[i].id, timestep + 1);
            }
            changed = true;

            while (path) {
                Node *prev = path;
                path = path->parent;
                free(prev);
            }
        }

        timestep++;
        visualize_with_timestep(timestep);
    } while (changed);
}

void setup_map() {
    const char *rows[] = {
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
    width = strlen(rows[0]);
    for (int y = 0; y < height; y++)
        strcpy(map[y], rows[y]);
}

void setup_agents() {
    agents[0] = (Agent){'A', {1, 1}, {5, 10}, {1, 1}, false};
    agents[1] = (Agent){'B', {3, 3}, {8, 10}, {3, 3}, false};
    agents[2] = (Agent){'C', {1, 5}, {5, 0}, {1, 5}, false};
    agents[3] = (Agent){'D', {3, 7}, {6, 10}, {3, 7}, false};
    agents[4] = (Agent){'E', {5, 1}, {3, 10}, {5, 1}, false};
    agents[5] = (Agent){'F', {7, 5}, {0, 5}, {7, 5}, false};
    agents[6] = (Agent){'G', {9, 9}, {3, 0}, {9, 9}, false};


agent_count = 7;

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