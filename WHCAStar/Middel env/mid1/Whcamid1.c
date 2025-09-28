//Final Implementation
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h> // For sleep()

#define GRID_WIDTH 7
#define GRID_HEIGHT 7
#define MAX_AGENTS 10
#define MAX_PATH 100
#define WINDOW 1000
#define STEP_DELAY 1000000 // Microseconds (0.5 sec)

typedef struct {
    int x, y;
} Position;

typedef struct {
    Position pos[MAX_PATH];
    int length;
} Path;

typedef struct {
    Position start, goal;
    Path path;
    char name; // Agent name (A, B, C, ...)
    bool finished;
} Agent;

typedef struct Node {
    Position pos;
    int g, h, f, time;
    struct Node *parent;
} Node;

int dx[5] = {0, 1, 0, -1, 0}; // cardinal + wait
int dy[5] = {1, 0, -1, 0, 0};

char map[GRID_HEIGHT][GRID_WIDTH] = {
    ".......",
    ".#.#.#.",
    ".......",
    ".#.#.#.",
    ".......",
    ".#.#.#.",
    "......."
};


bool grid[GRID_HEIGHT][GRID_WIDTH]; // true = free, false = obstacle
bool reservation_table[GRID_HEIGHT][GRID_WIDTH][MAX_PATH]; // reserved by time

int manhattan(Position a, Position b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

bool is_valid(int x, int y) {
    return x >= 0 && y >= 0 && x < GRID_WIDTH && y < GRID_HEIGHT && grid[y][x];
}

bool is_reserved(int x, int y, int time) {
    if (time >= MAX_PATH) return true;
    return reservation_table[y][x][time];
}

void reserve_path(Path *path) {
    for (int t = 0; t < path->length && t < MAX_PATH; t++) {
        int x = path->pos[t].x;
        int y = path->pos[t].y;
        reservation_table[y][x][t] = true;

        // Prevent swap collisions: reserve the previous position at the next time step
        if (t > 0 && (t < MAX_PATH)) {
            int prev_x = path->pos[t - 1].x;
            int prev_y = path->pos[t - 1].y;
            reservation_table[prev_y][prev_x][t] = true;
        }
    }
}

Node* create_node(int x, int y, int g, int h, int time, Node* parent) {
    Node* n = malloc(sizeof(Node));
    n->pos = (Position){x, y};
    n->g = g;
    n->h = h;
    n->f = g + h;
    n->time = time;
    n->parent = parent;
    return n;
}

void free_all_nodes(Node* list[], int count) {
    for (int i = 0; i < count; i++) {
        free(list[i]);
    }
}

// WHCA* A* planner for a single agent with reservations
bool whca_star(Agent* agent, int window) {
    Node* open[MAX_PATH * GRID_WIDTH * GRID_HEIGHT];
    bool closed[GRID_HEIGHT][GRID_WIDTH][WINDOW] = {false};
    int open_size = 0;

    Position start = agent->start;
    Position goal = agent->goal;

    Node* start_node = create_node(start.x, start.y, 0,
        manhattan(start, goal), 0, NULL);
    open[open_size++] = start_node;
    Node* goal_node = NULL;

    while (open_size > 0) {
        // Find lowest f
        int best = 0;
        for (int i = 1; i < open_size; i++) {
            if (open[i]->f < open[best]->f) best = i;
        }
        Node* current = open[best];
        open[best] = open[--open_size];

        // Check goal
        if (current->pos.x == goal.x && current->pos.y == goal.y) {
            goal_node = current;
            break;
        }

        // Time window limit
        if (current->time >= window) {
            free(current);
            continue;
        }

        // Skip if already closed
        if (closed[current->pos.y][current->pos.x][current->time]) {
            free(current);
            continue;
        }
        closed[current->pos.y][current->pos.x][current->time] = true;

        // Expand neighbors including wait
        for (int dir = 0; dir < 5; dir++) {
            int nx = current->pos.x + dx[dir];
            int ny = current->pos.y + dy[dir];
            int nt = current->time + 1;

            if (dir < 4 && !is_valid(nx, ny)) continue;
            if (is_reserved(nx, ny, nt)) continue;
            if (nt < WINDOW && closed[ny][nx][nt]) continue;

            if (open_size < MAX_PATH * GRID_WIDTH * GRID_HEIGHT) {
                Node* neighbor = create_node(nx, ny,
                    current->g + 1,
                    manhattan((Position){nx, ny}, goal),
                    nt, current);
                open[open_size++] = neighbor;
            }
        }
    }

    if (!goal_node) {
        free_all_nodes(open, open_size);
        return false;
    }

    // Reconstruct path
    Path* path = &agent->path;
    path->length = 0;
    Node* n = goal_node;
    while (n && path->length < MAX_PATH) {
        path->pos[path->length++] = n->pos;
        n = n->parent;
    }

    // Reverse
    for (int i = 0; i < path->length / 2; i++) {
        Position tmp = path->pos[i];
        path->pos[i] = path->pos[path->length - i - 1];
        path->pos[path->length - i - 1] = tmp;
    }

    reserve_path(path);

    // Free remaining nodes
    free_all_nodes(open, open_size);
    return true;
}

void print_grid(Agent agents[], int agent_count, int time) {
    char display[GRID_HEIGHT][GRID_WIDTH];
    memcpy(display, map, sizeof(map));

    // Mark goals
    for (int a = 0; a < agent_count; a++) {
        display[agents[a].goal.y][agents[a].goal.x] = 'G';
    }

    // Mark agents at current time and check for collisions
    bool occupied[GRID_HEIGHT][GRID_WIDTH] = {false};
    int occ_agent[GRID_HEIGHT][GRID_WIDTH];
    for (int y = 0; y < GRID_HEIGHT; y++)
        for (int x = 0; x < GRID_WIDTH; x++)
            occ_agent[y][x] = -1;

    for (int a = 0; a < agent_count; a++) {
        if (time < agents[a].path.length) {
            int x = agents[a].path.pos[time].x;
            int y = agents[a].path.pos[time].y;
            char agent_char = agents[a].name;
            // Collision with another agent
            if (occupied[y][x]) {
                int other = occ_agent[y][x];
                printf("WARNING: Agent %c and Agent %c occupy (%d,%d) at time %d!\n",
                    agent_char, agents[other].name, x, y, time);
                printf("  Agent %c previous: (%d,%d)\n", agent_char,
                    (time > 0) ? agents[a].path.pos[time-1].x : agents[a].start.x,
                    (time > 0) ? agents[a].path.pos[time-1].y : agents[a].start.y);
                printf("  Agent %c previous: (%d,%d)\n", agents[other].name,
                    (time > 0) ? agents[other].path.pos[time-1].x : agents[other].start.x,
                    (time > 0) ? agents[other].path.pos[time-1].y : agents[other].start.y);
            }
            // Occupying goal of a finished agent
            for (int b = 0; b < agent_count; b++) {
                if (b != a && agents[b].finished &&
                    x == agents[b].goal.x && y == agents[b].goal.y) {
                    printf("WARNING: Agent %c occupies the goal of finished Agent %c at (%d,%d) at time %d!\n",
                        agent_char, agents[b].name, x, y, time);
                    printf("  Agent %c previous: (%d,%d)\n", agent_char,
                        (time > 0) ? agents[a].path.pos[time-1].x : agents[a].start.x,
                        (time > 0) ? agents[a].path.pos[time-1].y : agents[a].start.y);
                    printf("  Agent %c previous: (%d,%d)\n", agents[b].name,
                        (time > 0 && time-1 < agents[b].path.length) ? agents[b].path.pos[time-1].x : agents[b].goal.x,
                        (time > 0 && time-1 < agents[b].path.length) ? agents[b].path.pos[time-1].y : agents[b].goal.y);
                }
            }
            occupied[y][x] = true;
            occ_agent[y][x] = a;
            display[y][x] = agent_char;
        }
    }

    // Clear screen
    // printf("\033[H\033[J");
    printf("\nTime Step: %d\n", time);
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            printf("%c ", display[y][x]);
        }
        printf("\n");
    }    
}

void setup_grid() {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            grid[y][x] = (map[y][x] != '#');
        }
    }
}

int main() {
    memset(reservation_table, false, sizeof(reservation_table));
    setup_grid();

    // Customize agent names and positions here
    Agent agents[MAX_AGENTS] = {
        {{1,1}, {6,6}, {{0}}, 'A', false},
        {{1,5}, {6,0}, {{0}}, 'B', false},
        {{5,1}, {0,6}, {{0}}, 'C', false},
        {{5,5}, {0,0}, {{0}}, 'D', false},
        {{3,3}, {3,0}, {{0}}, 'E', false}
    };
    int agent_count = 5;

    for (int i = 0; i < agent_count; i++) {
        if (!whca_star(&agents[i], WINDOW)) {
            printf("Agent %c could not find a path within window.\n", agents[i].name);
        }
        agents[i].finished = false;
    }

    int max_steps = 0;
    for (int i = 0; i < agent_count; i++) {
        if (agents[i].path.length > max_steps) {
            max_steps = agents[i].path.length;
        }
    }

    for (int t = 0; t < max_steps; t++) {
        // Check for agent finish and notify
        for (int i = 0; i < agent_count; i++) {
            if (!agents[i].finished &&
                t < agents[i].path.length &&
                agents[i].path.pos[t].x == agents[i].goal.x &&
                agents[i].path.pos[t].y == agents[i].goal.y) {
                printf("Agent %c finished at time %d at (%d,%d)\n",
                    agents[i].name, t, agents[i].goal.x, agents[i].goal.y);
                agents[i].finished = true;
            }
        }
        print_grid(agents, agent_count, t);
        usleep(STEP_DELAY);
    }

    printf("Simulation complete.\n");
    return 0;
}
