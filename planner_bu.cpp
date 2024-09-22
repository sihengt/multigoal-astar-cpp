/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <math.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <limits>

#include "GraphManagerVector.h"
#include "LookupPriorityQueueVector.h"

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
    LookupPriorityQueueVector &Q,
    GraphManagerVector &gm,
    int start,
    std::vector<int> &heuristic
    // std::vector<int> &time_of_closest_goal_to_state,
    // std::vector<int> &distance_of_closest_goal_to_state,
    // std::vector<long long> &closest_goal_to_state
)
{
    std::vector<int> successors(8, -1);
    
    // Insert goal as start state, with 0 for cost-to-go and 0 for heuristic (cause we djikstra-ing)
    for (int goal : gm.l_goals)
        Q.insert(goal, 0, 0);

    Coord2d current_coord(0, 0);
    Coord2d successor_coord(0, 0);
    int current_state_index, current_g, current_cost;
    while (!Q.empty())
    {
        const State current_state = Q.top();
        current_state_index = current_state.index;
        current_g = current_state.cost_to_go;
        heuristic[current_state_index] = current_g;
        
        // If current_state is the start_state, we've found a solution.
        if (current_state.index == start)
            break;
        Q.pop();

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
            
            // Successor is not in OPEN, so we insert it into the queue.
            // This is the first time the successor has been encountered, so we will update our lookup tables too.
            if (Q.lookup_table[successor_index].node_ == nullptr)
            {
                Q.insert(successor_index, successor_cost_to_goal, 0);
                // gm.populate_lookup_tables(
                //     successor_index,
                //     time_of_closest_goal_to_state,
                //     distance_of_closest_goal_to_state,
                //     closest_goal_to_state
                // );
            }
                
                
            // Successor is in OPEN, we need to check if it's current cost to go is > previous state's cost to go + cost
            else
            {
                auto succ_state_ptr = Q.lookup_table[successor_index];
                if ((*succ_state_ptr).cost_to_go > successor_cost_to_goal)
                {
                    Q.check_cost_and_update(successor_index, successor_cost_to_goal);
                }
            }
                
        }
    }
}

long long multi_goal_astar(
    LookupPriorityQueue &Q,
    GraphManager &gm,
    int start,
    std::unordered_map<long long, int> &optimal_action_to_state,
    std::vector<int> &heuristic
)
{   
    std::ofstream outFile("log.txt");
    std::vector<long long> successors(9, -1);

    // Insert start state, with 0 for cost-to-go and 0 for heuristic
    Q.insert(start, 0, 0);

    Coord3d current_coord(0, 0, 0);
    Coord3d successor_coord(0, 0, 0);
    Coord2d current_coord_2d(0, 0);
    Coord2d successor_coord_2d(0, 0);
    Coord3d debug_coord_3d(0,0,0);

    long long current_state_index;
    long long actual_goal_index;
    int current_state_index_2d, successor_index_2d, current_g, current_cost;
    while (!Q.empty())
    {
        auto start_t = std::chrono::high_resolution_clock::now();
        const State current_state = Q.top();
        auto stop_t = std::chrono::high_resolution_clock::now();
        auto duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        std::cout << "Top took " << duration_t.count() << " microseconds." << std::endl;

        current_state_index = current_state.index;
        current_g = current_state.cost_to_go;
        
        gm.index_to_coord(current_state_index, debug_coord_3d);
        // std::cout << "Exploring: (" << debug_coord_3d.x << "," << debug_coord_3d.y << "," << debug_coord_3d.t << ")" << std::endl;
        outFile << "(" << debug_coord_3d.x << "," << debug_coord_3d.y << "," << debug_coord_3d.t << ")" << std::endl;

        // Checking if current state is the dummy goal for multi-goal A*.
        if (current_state.index == gm.goal_index)
            return actual_goal_index;
        start_t = std::chrono::high_resolution_clock::now();
        Q.pop();
        stop_t = std::chrono::high_resolution_clock::now();
        duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        std::cout << "Pop took " << duration_t.count() << " microseconds." << std::endl;

        // Check if current_state_index is in the list of goals. If so, 
        // 1. Add dummy goal state into Q
        // 2. save actual_goal_index = current_state_index;
        // 3. Continue
        
        start_t = std::chrono::high_resolution_clock::now();
        
        if (std::find(gm.l_goals.begin(), gm.l_goals.end(), current_state.index) != gm.l_goals.end())
        {
            Q.insert(gm.goal_index, current_g, 0);
            actual_goal_index = current_state_index;
            continue;
        }
        stop_t = std::chrono::high_resolution_clock::now();
        duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        std::cout << "Checking goals took " << duration_t.count() << " microseconds." << std::endl;


        start_t = std::chrono::high_resolution_clock::now();
    
        // Expanded node is optimal and won't be visited again.
        gm.add_to_closed(current_state_index);

        stop_t = std::chrono::high_resolution_clock::now();
        duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        std::cout << "Adding to closed took " << duration_t.count() << " microseconds." << std::endl;


        start_t = std::chrono::high_resolution_clock::now();

        // Getting successors.
        gm.index_to_coord(current_state_index, current_coord);
        
        current_coord_2d.x = current_coord.x;
        current_coord_2d.y = current_coord.y;
        current_state_index_2d = gm.coord_to_index(current_coord_2d);
        
        gm.get_successors(current_coord, successors);

        stop_t = std::chrono::high_resolution_clock::now();
        duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        std::cout << "Getting successors took " << duration_t.count() << " microseconds." << std::endl;

        int successor_cost_to_go;
        // for (int successor_index : successors)
        for (int action_index = 0; action_index < 9; action_index++)
        {   
            long long successor_index = successors[action_index];
            // successor_index = -1 means there's no successor for that direction.
            if (successor_index == -1)
                continue;

            // For a forward
            gm.index_to_coord(successor_index, successor_coord);
            successor_coord_2d.x = successor_coord.x;
            successor_coord_2d.y = successor_coord.y;
            successor_index_2d = gm.coord_to_index(successor_coord_2d);
            successor_cost_to_go = current_g + gm.get_c(successor_coord_2d);
            
            // Successor is not in OPEN, so we insert it into the queue.

            if (Q.lookup_table.find(successor_index) == Q.lookup_table.end())
            {
                bool successor_at_goal = false;
                long long time_heuristic = 0;
                long long full_heuristic;
                int shortest_t = std::numeric_limits<int>::max();
                int shortest_distance = std::numeric_limits<int>::max();
                Coord3d goal_coord(0,0,0);
                // TODO: iterating through the goals every single time to get the time heuristic is slow, but we 
                // cannot possibly precompute this.
                // we have successor_coord (3d), successor_index (3d), goal_indices (3d) no goal_coord, 
                // 1) which we need for the time, 
                // 2) which we need for the x-y coordinates.

                // Summary of this long loop
                // 1. Find if successor is a goal. If so, the heuristic is the time difference, which will be multiplied
                // with cost to go for staying there.
                // 2. Find the shortest goal to the successor. Get the shortest time, and the shortest distance.
                // 3. THREE lookup tables: 
                // std::vector<int> time_of_closest_goal_to_state
                // std::vector<int> distance_of_closest_goal_to_state
                // std::vector<long long> closest_goal_to_state
                start_t = std::chrono::high_resolution_clock::now();
                for (long long goal_index : gm.l_goals)
                {
                    gm.index_to_coord(goal_index, goal_coord);
                    if (goal_coord.x == successor_coord.x && goal_coord.y == successor_coord.y)
                    {
                        if (goal_coord.t < successor_coord.t)
                        {
                            time_heuristic = std::numeric_limits<int>::max();
                            break;
                        }
                        time_heuristic = std::abs(successor_coord.t - goal_coord.t);
                        successor_at_goal = true;
                        break;
                    }

                    // TODO: should we store a lookuptable of the closest goal to all possible states?
                    // std::vector<long long> closest_goal_to_state(max_x, max_y);
                    // This would become O(1), and we can get the shortest time easily.
                    // std::vector<int> closest_goal_to_state_time(max_x, max_y); if we don't need other goal info.
                    int cheby_dist = std::max(goal_coord.x - successor_coord.x, goal_coord.y - successor_coord.y);
                    if (cheby_dist < shortest_distance)
                    {
                        shortest_t = goal_coord.t;
                        shortest_distance = cheby_dist;
                    }
                }
                stop_t = std::chrono::high_resolution_clock::now();
                duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
                std::cout << "Finding time heuristic took " << duration_t.count() << " microseconds." << std::endl;


                if (successor_at_goal)
                {
                    full_heuristic = time_heuristic * gm.get_c(successor_coord_2d);
                } else {
                    full_heuristic = static_cast<long long>(heuristic[successor_index_2d]) + \
                        std::abs(shortest_t - successor_coord.t);
                }

                // Q.insert(successor_index, successor_cost_to_go, heuristic[successor_index_2d]);
                start_t = std::chrono::high_resolution_clock::now();
                Q.insert(successor_index, successor_cost_to_go, full_heuristic);
                stop_t = std::chrono::high_resolution_clock::now();
                duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
                std::cout << "Q insert took " << duration_t.count() << " microseconds." << std::endl;

                start_t = std::chrono::high_resolution_clock::now();
                optimal_action_to_state[successor_index] = action_index;
                stop_t = std::chrono::high_resolution_clock::now();
                duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
                std::cout << "Updating optimal action took " << duration_t.count() << " microseconds." << std::endl;
            }
                
                
            // Successor is in OPEN, we need to check if it's current cost to go is > previous state's cost to go + cost
            else
            {
                start_t = std::chrono::high_resolution_clock::now();

                auto succ_state_ptr = Q.lookup_table[successor_index];
                if ((*succ_state_ptr).cost_to_go > successor_cost_to_go)
                {
                    Q.check_cost_and_update(successor_index, successor_cost_to_go);
                    optimal_action_to_state[successor_index] = action_index;
                }
                
                stop_t = std::chrono::high_resolution_clock::now();
                duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
                std::cout << "Updating queue took " << duration_t.count() << " microseconds." << std::endl;
            } 
        }
    }
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
    static bool is_cached = false;
    static std::vector<int> heuristic(x_size * y_size, std::numeric_limits<int>::max());
    static std::vector<long long> closest_goal_to_state(x_size * y_size * target_steps, -1);
    static std::vector<int> time_of_closest_goal_to_state(x_size * y_size, -1);
    static std::vector<int> distance_of_closest_goal_to_state(x_size * y_size, -1);

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // 9-connected grid (includes waiting)
    int dX_full[NUMOFDIRS + 1] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY_full[NUMOFDIRS + 1] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

    auto start_t = std::chrono::high_resolution_clock::now();
    auto stop_t = std::chrono::high_resolution_clock::now();
    auto duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);

    if (!is_cached)
    {        
        LookupPriorityQueueVector Q_dijkstra(x_size, y_size);
        GraphManagerVector gm_dijkstra(x_size, y_size, target_steps, map, collision_thresh, NUMOFDIRS);
        Coord2d robot_pose_2d(robotposeX, robotposeY);

        heuristic.resize(x_size * y_size);
        std::fill(heuristic.begin(), heuristic.end(), std::numeric_limits<int>::max());
        
        gm_dijkstra.init_actions(dX, dY, NUMOFDIRS);
        gm_dijkstra.populate_l_goals(target_traj, robot_pose_2d);
        multi_backwards_djikstra(
            Q_dijkstra,
            gm_dijkstra,
            gm_dijkstra.coord_to_index(robot_pose_2d),
            heuristic);
            // time_of_closest_goal_to_state,
            // distance_of_closest_goal_to_state,
            // closest_goal_to_state);
        
        stop_t = std::chrono::high_resolution_clock::now();
        duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        std::cout << "Time taken by Djikstra: " << duration_t.count() << std::endl;
        is_cached = true;
    }

    // Reinitializing new start robot pose.
    Coord3d robot_pose_3d(robotposeX, robotposeY, curr_time);
    LookupPriorityQueue Q(x_size, y_size, target_steps);
    GraphManager gm(x_size, y_size, target_steps, map, collision_thresh, NUMOFDIRS);

    // These actions are the actions that brought you to this state, NOT the actions to take at the state.
    // You can backtrack from the goal state all the way back to the start state to get the best action to take,
    // kind of like MPC.
    std::unordered_map<long long, int> optimal_action_to_state;
    gm.init_actions(dX_full, dY_full, NUMOFDIRS + 1);
    gm.populate_l_goals(target_traj, robot_pose_3d);
    start_t = std::chrono::high_resolution_clock::now();
    long long goal_state = multi_goal_astar(Q,
                                            gm,
                                            gm.coord_to_index(robot_pose_3d),
                                            optimal_action_to_state,
                                            heuristic);
    if (goal_state < 0 || goal_state > static_cast<long long>(x_size) * static_cast<long long>(y_size) * static_cast<long long>(target_steps))
    {
        std::cout << "INVALID GOAL" << std::endl;
    }
    long long current_state = goal_state;
    long long robot_index_3d = gm.coord_to_index(robot_pose_3d);
    Coord3d debug_goal_coords(0,0,0);
    gm.index_to_coord(current_state, debug_goal_coords);
    std::cout << "goal=(" << debug_goal_coords.x << "," << debug_goal_coords.y << "," << debug_goal_coords.t << ")" << std::endl;
    Coord3d current_state_coords(0, 0, 0);
    int current_action;
    while (current_state != robot_index_3d)
    {
        gm.index_to_coord(current_state, current_state_coords);
        current_action = optimal_action_to_state.at(current_state);
        current_state_coords.x -= dX_full[current_action];
        current_state_coords.y -= dY_full[current_action];
        current_state_coords.t -= 1;
        if (current_state_coords.t < 0)
        {
            std::cout << "FATAL ERROR in BACKTRACKING" << std::endl;
            break;
        }
        current_state = gm.coord_to_index(current_state_coords);
    }
    stop_t = std::chrono::high_resolution_clock::now();
    duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
    std::cout << "Time taken by A*: " << duration_t.count() << std::endl;

    action_ptr[0] = robotposeX + dX_full[current_action];
    action_ptr[1] = robotposeY + dY_full[current_action];
    return;
}