/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

// As a baseline implement multi goal A*
// Every single (x, y, t) is considered a goal.

// Questions
// 1. How do we construct the graph?

void multi_backwards_djikstra(
    PriorityQueue &Q,
    GraphManagerVector &gm,
    int start,
    std::vector<int> &heuristic
)
{
    std::vector<int> successors(8, -1);
    
    // Insert goal as start state, with 0 for cost-to-go and 0 for heuristic (cause we djikstra-ing)
    for (int goal : gm.l_goals)
    {
        Q.insert(goal, 0, 0);
        heuristic[goal] = 0;
    }   

    Coord2d current_coord(0, 0);
    Coord2d successor_coord(0, 0);
    int current_state_index, current_g, current_cost;
    int n_states_explored = 0;

    // Explore entire map
    while (!Q.empty())
    {
        const State current_state = Q.top();
        current_state_index = current_state.index;
        current_g = current_state.cost_to_go;
        heuristic[current_state_index] = current_g;

        if (gm.closed_queue[current_state.index])
        {
            Q.pop();
            continue;
        }

        Q.pop();
        n_states_explored++;

        // Expanded node is optimal and won't be visited again.
        gm.add_to_closed(current_state_index);

        // Getting successors.
        gm.index_to_coord(current_state_index, current_coord);
        gm.get_successors(current_coord, successors);
        current_cost = gm.get_c(current_coord);
        
        int successor_cost_to_goal;
        for (int successor_index : successors)
        {   
            if (successor_index == -1)
                continue;
            
            // For the backward Djikstra, we've established that the cost-to-go is whatever cost + current node's cost.
            successor_cost_to_goal = current_g + current_cost;
            
            // Insert if cost is smaller. The former portion of the code handles CLOSED.
            if (heuristic[successor_index] > successor_cost_to_goal)
            {
                heuristic[successor_index] = successor_cost_to_goal;
                Q.insert(successor_index, successor_cost_to_goal, 0);
            }
        }
    }
    std::cout << "Explored " << n_states_explored << " states." << std::endl;
}

long long multi_goal_astar(
    PriorityQueue &Q,
    GraphManager &gm,
    long long start,
    std::unordered_map<long long, int> &optimal_action_to_state,
    std::vector<int> &heuristic,
    bool accept_suboptimal=false)
{   
    // [DEBUG] log.txt logs expanded states
    // std::ofstream outFile("log.txt");
    
    // Initializing data structure to maintain potential successors
    std::vector<long long> successors(9, -1);

    // Inserting the robot start index into the queue with 0 cost and heuristic.
    Q.insert(start, 0, 0);

    // Initializing variables used later.
    Coord3d goal_coord(0, 0, 0);
    
    // current_coord = top of priority queue (lowest f value)
    Coord3d current_coord(0, 0, 0);
    Coord2d current_coord_2d(0, 0);

    // successor_coord = potential successors that we're expanding
    Coord3d successor_coord(0, 0, 0);
    Coord2d successor_coord_2d(0, 0);
    
    // Used for logging and debugging.
    Coord3d debug_coord_3d(0,0,0);

    // Current state index, in 3D
    long long current_state_index;
    
    // TODO: actual_goal_index might not be the best way to keep track of the actual goal.
    long long actual_goal_index = -1;

    int successor_index_2d, current_g;
    
    while (!Q.empty())
    {
        const State current_state = Q.top();
        current_state_index = current_state.index;
        current_g = current_state.cost_to_go;

        // We discard this state as the optimal version of this state has already been expanded.
        if (gm.closed_queue.find(current_state_index) != gm.closed_queue.end())
        {
            Q.pop();
            continue;
        }

        // [DEBUG] Logging states that A* is currently expanding.
        gm.index_to_coord(current_state_index, debug_coord_3d);
        // outFile << "(" << debug_coord_3d.x << "," << debug_coord_3d.y << "," << debug_coord_3d.t << ")" << std::endl;
                
        // Checking if current state is the dummy goal for multi-goal A*.
        if (current_state.index == gm.goal_index)
            return actual_goal_index;

        // Updating the optimal action to this current state.
        optimal_action_to_state[current_state.index] = current_state.action;

        Q.pop();

        // While we're still doing 3D A* search, as long as we're at the goal coordinate and we're there before, 
        // the target, we'll terminate.
        if (accept_suboptimal)
        {
            Coord2d current_coord_2d(debug_coord_3d.x, debug_coord_3d.y);
            auto iter = std::find(gm.l_goals_2d.begin(), gm.l_goals_2d.end(), gm.coord_to_index(current_coord_2d));
            if (iter != gm.l_goals_2d.end())
            {
                size_t index = std::distance(gm.l_goals_2d.begin(), iter);
                long long goal_index = gm.l_goals[index];
                gm.index_to_coord(goal_index, goal_coord);
                if (goal_coord.t > debug_coord_3d.t)
                {
                    Q.insert(gm.goal_index, current_g, 0);
                    actual_goal_index = current_state_index;
                }
            }
        }

        // If the current state matches (x, y, time) exactly, we insert the dummy state into the queue.
        else
        {
            if (std::find(gm.l_goals.begin(), gm.l_goals.end(), current_state.index) != gm.l_goals.end())
            {
                Q.insert(gm.goal_index, current_g, 0);
                actual_goal_index = current_state_index;
            }
        }

        
        // Expanded node is optimal and won't be visited again.
        gm.add_to_closed(current_state_index);

        // Converting (long long) state_index to Coord3d and Coord2d
        gm.index_to_coord(current_state_index, current_coord);
        gm.get_successors(current_coord, successors);

        int successor_cost_to_go;
        for (int action_index = 0; action_index < 9; action_index++)
        {   
            long long successor_index = successors[action_index];
            
            // Successor_index = -1 means there's no valid successor for that action.
            if (successor_index == -1)
                continue;

            // Converting successor into Coord3d / Coord2d (for lookup tables).
            gm.index_to_coord(successor_index, successor_coord);
            successor_coord_2d.x = successor_coord.x;
            successor_coord_2d.y = successor_coord.y;
            successor_index_2d = gm.coord_to_index(successor_coord_2d);
            successor_cost_to_go = current_g + gm.get_c(successor_coord_2d);
            
            Q.insert(successor_index, successor_cost_to_go, heuristic[successor_index_2d], action_index);            
        }
    }
    return -1;
}

int single_goal_astar(
    PriorityQueue &Q,
    GraphManager &gm,
    int start,
    int goal,
    std::unordered_map<long long, int> &optimal_action_to_state,
    std::vector<int> &heuristic)
{   
    // [DEBUG] log.txt logs expanded states
    // std::ofstream outFile("log.txt");
    
    // Initializing data structure to maintain potential successors
    std::vector<int> successors(9, -1);

    // Inserting the robot start index into the queue with 0 cost and heuristic.
    Q.insert(start, 0, 0);

    Coord2d goal_coord(0, 0);
    Coord2d current_coord(0, 0); // current_coord = top of priority queue (lowest f value)
    Coord2d successor_coord(0, 0); // successor_coord = potential successors that we're expanding
    int current_state_index; // Current state index, in 3D
    
    // TODO: actual_goal_index might not be the best way to keep track of the actual goal.
    int actual_goal_index = -1;

    int successor_index, current_g;
    
    while (!Q.empty())
    {
        const State current_state = Q.top();
        current_state_index = current_state.index;
        current_g = current_state.cost_to_go;

        // We discard this state as the optimal version of this state has already been expanded.
        if (gm.closed_queue.find(current_state_index) != gm.closed_queue.end())
        {
            Q.pop();
            continue;
        }

        // [DEBUG] Logging states that A* is currently expanding.
        gm.index_to_coord(current_state_index, current_coord);
        // outFile << "(" << current_coord.x << "," << current_coord.y << ")" << std::endl;

        // Updating the optimal action to this current state.
        optimal_action_to_state[current_state.index] = current_state.action;

        // Checking if current state is the dummy goal for multi-goal A*.
        if (current_state.index == goal)
            return goal;

        Q.pop();
        
        // Expanded node is optimal and won't be visited again.
        gm.add_to_closed(current_state_index);

        // Converting (int) state_index to Coord3d and Coord2d
        gm.index_to_coord(current_state_index, current_coord);
        gm.get_successors(current_coord, successors);

        int successor_cost_to_go;
        for (int action_index = 0; action_index < 8; action_index++)
        {   
            int successor_index = successors[action_index];
            
            // Successor_index = -1 means there's no valid successor for that action.
            if (successor_index == -1)
                continue;

            // Converting successor into Coord3d / Coord2d (for lookup tables).
            gm.index_to_coord(successor_index, successor_coord);
            successor_cost_to_go = current_g + gm.get_c(successor_coord);
            
            Q.insert(successor_index, successor_cost_to_go, heuristic[successor_index], action_index);            
        }
    }
    return -1;
}

void backtrack(
    GraphManager &gm,
    std::vector<int> &plan,
    std::unordered_map<long long, int> &optimal_action_to_state,
    int* dX_full,
    int* dY_full,
    long long goal_state,
    long long start_state)
{
    // Initialize backtracking from goal
    long long current_state = goal_state;
    // [Debug] for checking time
    Coord3d goal_coords(0, 0, 0);

    Coord3d current_state_coords(0, 0, 0);
    int current_action, current_time = goal_coords.t;
    // std::ofstream log_backtrack("backtracking.txt");
    std::cout << "Plan size before backtracking:" << plan.size() << std::endl;
    
    while (current_state != start_state)
    {
        gm.index_to_coord(current_state, current_state_coords);
        
        // log_backtrack << "(" 
        //     << current_state_coords.x << "," 
        //     << current_state_coords.y << "," 
        //     << current_state_coords.t << ")" << std::endl;
        
        current_action = optimal_action_to_state.at(current_state);
        
        plan.push_back(current_action);
        
        current_state_coords.x -= dX_full[current_action];
        current_state_coords.y -= dY_full[current_action];
        current_state_coords.t -= 1;

        if (current_state_coords.t < 0)
        {
            std::cout << "FATAL ERROR IN BACKTRACKING" << std::endl;
            break;
        }

        current_state = gm.coord_to_index(current_state_coords);
    }
    std::cout << "Plan size after backtracking:" << plan.size() << std::endl;
}

void backtrack_2d(
    GraphManager &gm,
    std::vector<int> &plan,
    std::unordered_map<long long, int> &optimal_action_to_state,
    int* dX_full,
    int* dY_full,
    int goal_state,
    int start_state)
{
    // Initialize backtracking from goal
    int current_state = goal_state;

    Coord2d current_state_coords(0, 0);

    int current_action;
    // std::ofstream log_backtrack("backtracking.txt");
    std::cout << "Plan size before backtracking:" << plan.size() << std::endl;
    
    while (current_state != start_state)
    {
        gm.index_to_coord(current_state, current_state_coords);
        
        // log_backtrack << "(" 
        //     << current_state_coords.x << "," 
        //     << current_state_coords.y << ")" << std::endl;
        
        current_action = optimal_action_to_state.at(current_state);
        
        plan.push_back(current_action);
        
        current_state_coords.x -= dX_full[current_action];
        current_state_coords.y -= dY_full[current_action];
        current_state = gm.coord_to_index(current_state_coords);
    }
    std::cout << "Plan size after backtracking:" << plan.size() << std::endl;
}


int get_next_action(std::vector<int> &plan, int curr_time, int &previous_time)
{
    // Stay stationary = 8
    int current_action = 8;
    
    // We've reached the end of our plan, i.e. the goal state. Stay stationary.
    if (plan.empty())
        return current_action;
    else
    {
        current_action = plan.back();
        // If we're asked to wait but we've taken too long planning, we don't wait.
        while (curr_time >= previous_time + 1 && current_action == 8)
        {
            plan.pop_back();
            previous_time++;
            return get_next_action(plan, curr_time, previous_time);
        }
        plan.pop_back();
    }
    return current_action;
}

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    // Hello friendly TAs, tune here:
    // Refer to my table for parameters used.
    double EPSILON = 0.0;
    bool ACCEPT_SUBOPTIMAL = false;
    
    // Globals
    static bool is_cached = false;
    static std::vector<int> heuristic(x_size * y_size, std::numeric_limits<int>::max());
    static bool plan_found = false;
    static long long goal_state;
    static std::unordered_map<long long, int> optimal_action_to_state;
    static int prev_time;
    static std::vector<int> plan;
    
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // 9-connected grid (includes waiting)
    int dX_full[NUMOFDIRS + 1] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY_full[NUMOFDIRS + 1] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

    auto start_t = std::chrono::high_resolution_clock::now();
    auto stop_t = std::chrono::high_resolution_clock::now();
    auto duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);

    bool single_goal = false;
    int single_goal_index;

    Coord2d robot_pose_2d(robotposeX, robotposeY);
    int robot_pose_2d_index;

    // The graphmanagervector uses a vector for lookup
    // AHH looks troubling but it doesn't affect anything since the heuristic is always 0.
    // Sorry for bad coding practices.
    PriorityQueue Q_dijkstra(EPSILON);
    GraphManagerVector gm_dijkstra(x_size, y_size, target_steps, map, collision_thresh, NUMOFDIRS);

    if (!is_cached)
    {
        // First call - we initialize our prev_time to curr_time so we know how much time has elapsed.
        prev_time = curr_time;

        // Initializing a lookup table for heuristic to be used with A*
        heuristic.resize(x_size * y_size);
        std::fill(heuristic.begin(), heuristic.end(), std::numeric_limits<int>::max());
        
        gm_dijkstra.init_actions(dX, dY, NUMOFDIRS);
        gm_dijkstra.populate_l_goals(target_traj, robot_pose_2d);
        robot_pose_2d_index = gm_dijkstra.coord_to_index(robot_pose_2d);
        multi_backwards_djikstra(
            Q_dijkstra,
            gm_dijkstra,
            gm_dijkstra.coord_to_index(robot_pose_2d),
            heuristic);
        
        stop_t = std::chrono::high_resolution_clock::now();
        duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        std::cout << "Time taken by Djikstra: " << duration_t.count() << std::endl;
        
        // If single goal, we'll generate a plan using Dijkstra only.
        if (gm_dijkstra.l_goals.size() == 1)
        {
            single_goal = true;
            single_goal_index = *gm_dijkstra.l_goals.begin();
        }
            

        // Updating this so Dijkstra won't be called again.
        is_cached = true;
    }

    // [DEBUG] writing heuristic to file
    // std::ofstream outfile("heuristic");
    // for (size_t i = 0; i < heuristic.size(); ++i) {
    //     outfile << heuristic[i];
    //     if (i != heuristic.size() - 1) {
    //         outfile << ","; // Separate each value with a comma
    //     }
    // }
    // outfile.close();

    
    if (!plan_found)
    {
        PriorityQueue Q(EPSILON);
        GraphManager gm(x_size, y_size, target_steps, map, collision_thresh, NUMOFDIRS+1);

        // No need for A* if it's a single goal
        if (single_goal)
        {
            start_t = std::chrono::high_resolution_clock::now();
            gm.init_actions(dX, dY, NUMOFDIRS);
            // Plan towards a goal state using multi_goal_astar
            goal_state = single_goal_astar(
                Q,
                gm,
                gm.coord_to_index(robot_pose_2d),
                single_goal_index,
                optimal_action_to_state,
                heuristic);
            plan_found = true;
            stop_t = std::chrono::high_resolution_clock::now();
            duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
            std::cout << "Time taken by single goal A*: " << duration_t.count() << std::endl;

            // Populates plan here.
            int robot_index = gm.coord_to_index(robot_pose_2d); // pass to function as start_state
            backtrack_2d(gm, plan, optimal_action_to_state, dX_full, dY_full, goal_state, robot_index);
        }
        else
        {
            start_t = std::chrono::high_resolution_clock::now();
            Coord3d robot_pose_3d(robotposeX, robotposeY, curr_time);    
            gm.init_actions(dX_full, dY_full, NUMOFDIRS + 1);
            gm.populate_l_goals(target_traj, robot_pose_3d);
            
            // Plan towards a goal state using multi_goal_astar
            goal_state = multi_goal_astar(Q,
                                        gm,
                                        gm.coord_to_index(robot_pose_3d),
                                        optimal_action_to_state,
                                        heuristic,
                                        ACCEPT_SUBOPTIMAL);
            plan_found = true;
            stop_t = std::chrono::high_resolution_clock::now();
            duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
            std::cout << "Time taken by A*: " << duration_t.count() << std::endl;

            // Populates plan here.
            long long robot_index_3d = gm.coord_to_index(robot_pose_3d); // pass to function as start_state
            backtrack(gm, plan, optimal_action_to_state, dX_full, dY_full, goal_state, robot_index_3d);
        }

    }

    // TODO: into a function here as well.
    int current_action = get_next_action(plan, curr_time, prev_time);
    action_ptr[0] = robotposeX + dX_full[current_action];
    action_ptr[1] = robotposeY + dY_full[current_action];
    
    // Updating the time so at next function call we know which timestep we were at before.
    prev_time++;
    return;
}