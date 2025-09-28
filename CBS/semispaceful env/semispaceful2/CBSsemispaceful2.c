#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#define MAX_AGENTS 26
#define MAX_ROWS 20
#define MAX_COLS 20
#define MAX_STEPS 100


// Direction vectors: up, down, left, right, wait
int dRow[] = {-1, 1, 0, 0, 0};
int dCol[] = {0, 0, -1, 1, 0};

// Position struct for row/col on grid
typedef struct {
    int row, col;
} Position;

// A* Node with pathfinding info
typedef struct Node {
    Position pos;
    int g_cost, h_cost, f_cost;
    struct Node* parent;
    int agent_id;
    int step;
} Node;

// Grid structure
typedef struct {
    char grid[MAX_ROWS][MAX_COLS];
    int rows, cols;
} Grid;

// Each agent has a start and goal, and a path to store its movement
typedef struct {
    Position start, goal;
    Position path[MAX_STEPS];
    int path_length;
} Agent;

Grid grid;
Agent agents[MAX_AGENTS];
int num_agents = 0;

// Constraints prevent agents from being at certain positions at specific times
int constraints[MAX_AGENTS][MAX_STEPS][MAX_ROWS][MAX_COLS];

// Prints the grid with agent starts and goals
void print_grid() {
    for (int r = 0; r < grid.rows; r++) {
        for (int c = 0; c < grid.cols; c++) {
            char cell = grid.grid[r][c];
            for (int i = 0; i < num_agents; i++) {
                if (agents[i].start.row == r && agents[i].start.col == c)
                    cell = 'A' + i;// Show agent's starting position
                if (agents[i].goal.row == r && agents[i].goal.col == c && cell == '.')
                    cell = '+'; // Show goal with '+'
            }
            printf("%c ", cell);
        }
        printf("\n");
    }
}

// Heuristic function: Manhattan distance between two positions
int manhattan_distance(Position a, Position b) {
    return abs(a.row - b.row) + abs(a.col - b.col);
}

// Checks if position is within bounds and not a wall
int is_valid_position(Position p) {
    return (p.row >= 0 && p.row < grid.rows && p.col >= 0 && p.col < grid.cols && grid.grid[p.row][p.col] != '#');
}

// Check for conflicts between two agents: vertex or edge (swap)
int has_conflict(Position a_pos, int a_step, Position b_pos, int b_step, Position a_prev, Position b_prev) {
    if (a_step != b_step) return 0;
    if (a_pos.row == b_pos.row && a_pos.col == b_pos.col) return 1; // vertex conflict
    if (a_prev.row == b_pos.row && a_prev.col == b_pos.col &&
        b_prev.row == a_pos.row && b_prev.col == a_pos.col) return 1; // swap conflict
    return 0;
}

// Performs A* pathfinding with temporal constraints
Node* a_star_search(int agent_id, int local_constraints[MAX_STEPS][MAX_ROWS][MAX_COLS]) {
    Node* open_list[MAX_ROWS * MAX_COLS * MAX_STEPS];
    int open_size = 0;
    Node* closed_list[MAX_ROWS * MAX_COLS * MAX_STEPS];
    int closed_size = 0;

    // Initialize start node
    Node* start_node = (Node*)malloc(sizeof(Node));
    start_node->pos = agents[agent_id].start;
    start_node->g_cost = 0;
    start_node->h_cost = manhattan_distance(start_node->pos, agents[agent_id].goal);
    start_node->f_cost = start_node->g_cost + start_node->h_cost;
    start_node->parent = NULL;
    start_node->step = 0;

    open_list[open_size++] = start_node;

    while (open_size > 0) {
        // Find node with lowest f_cost
        int best_index = 0;
        for (int i = 1; i < open_size; i++) {
            if (open_list[i]->f_cost < open_list[best_index]->f_cost)
                best_index = i;
        }
        Node* current = open_list[best_index];
        for (int i = best_index; i < open_size - 1; i++)
            open_list[i] = open_list[i + 1];
        open_size--;

        closed_list[closed_size++] = current;

        // If goal reached, reconstruct and store path
        if (current->pos.row == agents[agent_id].goal.row &&
            current->pos.col == agents[agent_id].goal.col) {
            Node* path_node = current;
            int len = 0;
            while (path_node != NULL) {
                len++;
                path_node = path_node->parent;
            }
            agents[agent_id].path_length = len;
            path_node = current;
            for (int i = len - 1; i >= 0; i--) {
                agents[agent_id].path[i] = path_node->pos;
                path_node = path_node->parent;
            }
            // Extend path with goal to prevent reoccupying
            for (int i = agents[agent_id].path_length; i < MAX_STEPS; i++) {
                agents[agent_id].path[i] = agents[agent_id].goal;
            }
            agents[agent_id].path_length = MAX_STEPS;
            return current;
        }

        // Try all directions (including wait)
        for (int i = 0; i < 5; i++) {
            Position next_pos = { current->pos.row + dRow[i], current->pos.col + dCol[i] };
        
            if (!is_valid_position(next_pos)) continue;

            int step = current->step + 1;
            if (step >= MAX_STEPS) continue;
            if (local_constraints[step][next_pos.row][next_pos.col]) continue;

            // Check if already visited
            int in_closed = 0;
            for (int j = 0; j < closed_size; j++) {
                if (closed_list[j]->pos.row == next_pos.row &&
                    closed_list[j]->pos.col == next_pos.col &&
                    closed_list[j]->step == step) {
                    in_closed = 1;
                    break;
                }
            }
            if (in_closed) continue;

            // Create neighbor node
            Node* neighbor = (Node*)malloc(sizeof(Node));
            neighbor->pos = next_pos;
            neighbor->g_cost = current->g_cost + 1;
            neighbor->h_cost = manhattan_distance(next_pos, agents[agent_id].goal);
            neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
            neighbor->parent = current;
            neighbor->step = step;

            open_list[open_size++] = neighbor;
        }
    }

    return NULL;// Path not found
}

// Visualize grid at specific timestep with warnings
void visualize_timestep(int step) {
    static int goal_reported[MAX_AGENTS] = {0}; 
    static int goal_time[MAX_AGENTS] = {0};
    printf("Timestep %d:\n", step);
    for (int r = 0; r < grid.rows; r++) {
        for (int c = 0; c < grid.cols; c++) {
            char cell = grid.grid[r][c];
            for (int i = 0; i < num_agents; i++) {
                if (step < agents[i].path_length &&
                    agents[i].path[step].row == r &&
                    agents[i].path[step].col == c)
                    cell = 'A' + i;
            }
            for (int i = 0; i < num_agents; i++) {
                if (agents[i].goal.row == r && agents[i].goal.col == c && cell == '.')
                    cell = '+';
            }
            printf("%c ", cell);
        }
        printf("\n");
    }
    // Warn if another agent occupies a goal cell of a finished agent
    for (int finished = 0; finished < num_agents; finished++) {
        if (goal_reported[finished]) {
            for (int moving = 0; moving < num_agents; moving++) {
                if (moving == finished) continue;
                if (step >= goal_time[finished] && step < agents[moving].path_length &&
                    agents[moving].path[step].row == agents[finished].goal.row &&
                    agents[moving].path[step].col == agents[finished].goal.col) {
                    printf("WARNING: Agent %c moves onto Agent %c's finished goal at timestep %d\n",
                        'A' + moving, 'A' + finished, step);
                }
            }
        }
    }
    // Report agents reaching goal
    for (int i = 0; i < num_agents; i++) {
        if (!goal_reported[i] &&
            step < agents[i].path_length &&
            agents[i].path[step].row == agents[i].goal.row &&
            agents[i].path[step].col == agents[i].goal.col) {
            printf("Agent %c reached its goal at timestep %d\n", 'A' + i, step);
            goal_reported[i] = 1;
            goal_time[i] = step;
        }
    }
    // Warn about conflicts at same position
    for (int i = 0; i < num_agents; i++) {
        if (step >= agents[i].path_length) continue;
        for (int j = i + 1; j < num_agents; j++) {
            if (step >= agents[j].path_length) continue;
            if (agents[i].path[step].row == agents[j].path[step].row &&
                agents[i].path[step].col == agents[j].path[step].col) {
                printf("WARNING: Agent %c and Agent %c occupy the same cell (%d, %d) at timestep %d\n",
                    'A' + i, 'A' + j, agents[i].path[step].row, agents[i].path[step].col, step);
            }
        }
    }

    printf("\n");
}
// Find the timestep when the last agent reaches its goal
int get_last_goal_timestep() {
    int last = 0;
    for (int i = 0; i < num_agents; i++) {
        int t = 0;
        for (; t < MAX_STEPS; t++) {
            if (agents[i].path[t].row == agents[i].goal.row &&
                agents[i].path[t].col == agents[i].goal.col) {
                break;
            }
        }
        if (t > last) last = t;
    }
    return last;
}
// Prevent other agents from occupying an agent's goal after it reaches it
void add_goal_occupation_constraints() {
    int goal_time[MAX_AGENTS];
    for (int i = 0; i < num_agents; i++) {
        goal_time[i] = -1;
        for (int t = 0; t < agents[i].path_length; t++) {
            if (agents[i].path[t].row == agents[i].goal.row &&
                agents[i].path[t].col == agents[i].goal.col) {
                goal_time[i] = t;
                break;
            }
        }
    }
    for (int i = 0; i < num_agents; i++) {
        if (goal_time[i] == -1) continue;
        for (int j = 0; j < num_agents; j++) {
            if (i == j) continue;
            for (int t = goal_time[i]; t < MAX_STEPS; t++) {
                constraints[j][t][agents[i].goal.row][agents[i].goal.col] = 1;
            }
        }
    }
}

// Conflict-Based Search (CBS) implementation
void cbs() {
    // Initial paths
    for (int i = 0; i < num_agents; i++) {
        if (!a_star_search(i, constraints[i])) {
            printf("Agent %c cannot find initial path\n", 'A' + i);
            exit(1);
        }
        printf("Agent %c path found\n", 'A' + i);
        for (int j = 0; j < agents[i].path_length; j++)
            printf("(%d, %d) -> ", agents[i].path[j].row, agents[i].path[j].col);
        printf("\n");
    }

    add_goal_occupation_constraints();
    // Replan with constraints applied
    for (int i = 0; i < num_agents; i++) {
        if (!a_star_search(i, constraints[i])) {
            printf("Agent %c cannot find path after adding goal occupation constraints\n", 'A' + i);
            exit(1);
        }
    }

    int replan_limit = 10000;
    int replan_count = 0;
    // Conflict detection and resolution loop
    int step = 1;
    while (step < MAX_STEPS) {
        int conflict_found = 0;
        for (int i = 0; i < num_agents && !conflict_found; i++) {
            for (int j = i + 1; j < num_agents && !conflict_found; j++) {
                if (step >= agents[i].path_length || step >= agents[j].path_length)
                    continue;

                Position a_prev = agents[i].path[step - 1];
                Position b_prev = agents[j].path[step - 1];
                Position a_curr = agents[i].path[step];
                Position b_curr = agents[j].path[step];

                if (has_conflict(a_curr, step, b_curr, step, a_prev, b_prev)) {
                    printf("Conflict detected between agent %c and agent %c at step %d\n", 'A' + i, 'A' + j, step);

                    int first = (agents[i].path_length <= agents[j].path_length) ? i : j;
                    int second = (first == i) ? j : i;

                    // Add vertex constraint for the replanned agent at the conflicting cell
                    int row = (first == i ? a_curr.row : b_curr.row);
                    int col = (first == i ? a_curr.col : b_curr.col);
                    constraints[first][step][row][col] = 1;

                    // Add swap (edge) constraint for the replanned agent if it's a swap conflict
                    int swap = 0;
                    int swap_row = 0, swap_col = 0;
                    if (a_prev.row == b_curr.row && a_prev.col == b_curr.col &&
                        b_prev.row == a_curr.row && b_prev.col == a_curr.col) {
                        swap = 1;
                        swap_row = (first == i ? b_curr.row : a_curr.row);
                        swap_col = (first == i ? b_curr.col : a_curr.col);
                        constraints[first][step - 1][swap_row][swap_col] = 1;
                    }

                    if (!a_star_search(first, constraints[first])) {
                        // Remove the constraints for the first agent before trying the second
                        constraints[first][step][row][col] = 0;
                        if (swap) constraints[first][step - 1][swap_row][swap_col] = 0;

                        // Now try the second agent
                        row = (second == i ? a_curr.row : b_curr.row);
                        col = (second == i ? a_curr.col : b_curr.col);
                        constraints[second][step][row][col] = 1;
                        if (swap) constraints[second][step - 1][(second == i ? b_curr.row : a_curr.row)][(second == i ? b_curr.col : a_curr.col)] = 1;

                        if (!a_star_search(second, constraints[second])) {
                            printf("Both agent %c and agent %c failed to replan at step %d.\n", 'A' + i, 'A' + j, step);
                            exit(1);
                        }
                    }

                    // After any replan, re-apply goal occupation constraints and replan all agents
                    add_goal_occupation_constraints();
                    for (int k = 0; k < num_agents; k++) {
                        if (!a_star_search(k, constraints[k])) {
                            printf("Agent %c cannot find path after adding goal occupation constraints\n", 'A' + k);
                            exit(1);
                        }
                    }

                    if (++replan_count > replan_limit) {
                        printf("Too many replans, problem may be unsolvable with current constraints.\n");
                        exit(1);
                    }

                    conflict_found = 1;
                    break;
                }
            }
        }
        if (conflict_found) {
            step = 1; // Restart checking from the beginning
        } else {
            step++;
        }
    }

    int last_goal_step = get_last_goal_timestep();

    for (int step = 0; step <= last_goal_step; step++) {
        visualize_timestep(step);
        // _sleep(1000); // Sleep for 1 second
    }
}
// Main function with a sample map and agents
int main() {
    grid.rows = 9;
    grid.cols = 9;
    strcpy(grid.grid[0], ".........");
    strcpy(grid.grid[1], ".#.#.#.#.");
    strcpy(grid.grid[2], ".........");
    strcpy(grid.grid[3], ".#.#.#.#.");
    strcpy(grid.grid[4], ".........");
    strcpy(grid.grid[5], ".#.#.#.#.");
    strcpy(grid.grid[6], ".........");
    strcpy(grid.grid[7], ".#.#.#.#.");
    strcpy(grid.grid[8], ".........");
    // Set up agents with their start and goal positions
    num_agents = 5;
    agents[0].start = (Position){1, 1};agents[0].goal = (Position){8, 5};
    agents[1].start = (Position){7, 1};agents[1].goal = (Position){0, 4};
    agents[2].start = (Position){1, 7};agents[2].goal = (Position){8, 4};
    agents[3].start = (Position){7, 7};agents[3].goal = (Position){8, 3};
    agents[4].start = (Position){7, 3};agents[4].goal = (Position){8, 7};  

    print_grid(); // Display initial map
    cbs();  // Run CBS

    return 0;
}
