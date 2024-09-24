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
        Q.insert(goal, 0, 0);

    Coord2d current_coord(0, 0);
    Coord2d successor_coord(0, 0);
    int current_state_index, current_g, current_cost;
    int n_states_explored = 0;
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

        // If current_state is the start_state, we've found a solution. We can continue expanding until all points
        // in a radius around the current_state is covered.
        // if (current_state.index == start)
        // {
        //     break;
        // }
            
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
            
            // Insert if heuristic is greater. The former portion of the code will only expand states that are not 
            // in closed list.
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
    std::unordered_map<int, long long> &closest_state_lookup
)
{   
    // [DEBUG] log.txt logs expanded states
    std::ofstream outFile("log.txt");
    std::vector<long long> successors(9, -1);

    Q.insert(start, 0, 0);
    Coord3d start_coord(0, 0, 0);
    Coord2d start_coord_2d(0, 0);
    gm.index_to_coord(start, start_coord_2d);
    Coord3d goal_coord(0, 0, 0);
    
    // current = top of priority queue (lowest cost)
    Coord3d current_coord(0, 0, 0);
    Coord2d current_coord_2d(0, 0);
    // successor = potential successors that we're expanding
    Coord3d successor_coord(0, 0, 0);
    Coord2d successor_coord_2d(0, 0);
    Coord3d debug_coord_3d(0,0,0);

    long long current_state_index;
    long long actual_goal_index = -1;
    int current_state_index_2d, successor_index_2d, current_g, current_cost;
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
        outFile << "(" << debug_coord_3d.x << "," << debug_coord_3d.y << "," << debug_coord_3d.t << ")" << std::endl;
        
        // Updating the optimal action to this current state.
        optimal_action_to_state[current_state.index] = current_state.action;
        
        // Checking if current state is the dummy goal for multi-goal A*.
        if (current_state.index == gm.goal_index)
            return actual_goal_index;

        Q.pop();

        // If the current state matches (x, y) exactly, we insert the dummy state into the queue
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

        // // If the current state matches (x, y, time) exactly, we insert the dummy state into the queue.
        // if (std::find(gm.l_goals.begin(), gm.l_goals.end(), current_state.index) != gm.l_goals.end())
        // {
        //     Q.insert(gm.goal_index, current_g, 0);
        //     actual_goal_index = current_state_index;
        // }
        
        // Expanded node is optimal and won't be visited again.
        gm.add_to_closed(current_state_index);

        // Converting (long long) state_index to Coord3d and Coord2d
        gm.index_to_coord(current_state_index, current_coord);
        current_coord_2d.x = current_coord.x;
        current_coord_2d.y = current_coord.y;
        current_state_index_2d = gm.coord_to_index(current_coord_2d);
        gm.get_successors(current_coord, successors);

        int successor_cost_to_go;
        // for (int successor_index : successors)
        for (int action_index = 0; action_index < 9; action_index++)
        {   
            long long successor_index = successors[action_index];
            
            // Successor_index = -1 means there's no successor for that action. Skip.
            if (successor_index == -1)
                continue;

            // Converting successor into Coord3d / Coord2d (for lookup tables).
            gm.index_to_coord(successor_index, successor_coord);
            successor_coord_2d.x = successor_coord.x;
            successor_coord_2d.y = successor_coord.y;
            successor_index_2d = gm.coord_to_index(successor_coord_2d);
            successor_cost_to_go = current_g + gm.get_c(successor_coord_2d);
            
            // Q.heuristic_lookup gets updated with our time heuristic after the state has been
            // encountered. Time heuristic only depends on 2D state (same for each state).

            // In this case we have to compute the heuristic since we've never done it.
            // bool successor_at_goal = false;
            // int time_diff_succ_to_goal = 0;
            // long long full_heuristic;
            // int shortest_t = std::numeric_limits<int>::infinity();
            // int shortest_distance = std::numeric_limits<int>::infinity();
            // Coord3d goal_coord(0,0,0);
            // long long shortest_goal_index;
            
            // // We've encountered this 2D state before - we can just access the closest 3D goal
            // // state from cache instead of iterating through everything.
            // if (closest_state_lookup.find(successor_index_2d) != closest_state_lookup.end())
            // {
            //     // Retrieve goal that is closest to the current state (by time index).
            //     long long goal_index = closest_state_lookup[successor_index_2d];
            //     gm.index_to_coord(goal_index, goal_coord);
                
            //     // If successor is at goal position (x, y, but not t)
            //     if (goal_coord.x == successor_coord.x && goal_coord.y == successor_coord.y)
            //     {
            //         successor_at_goal = true;
            //         // We'll never be able to reach this state anymore
            //         if (goal_coord.t < successor_coord.t)
            //             time_diff_succ_to_goal = std::numeric_limits<int>::infinity();
            //         time_diff_succ_to_goal = std::abs(successor_coord.t - goal_coord.t);
                    
            //     }
            //     // Successor is not at goal position. We take the goal_coord to have the shortest time.
            //     else
            //     {
            //         shortest_t = goal_coord.t;
            //     }
            // }
            
            // // We haven't encountered this 2D state before. We need to find the closest 3D goal
            // // state and add it to our cache ( slow :( )
            // else 
            // {
            // for (long long goal_index : gm.l_goals)
            // {
            //     gm.index_to_coord(goal_index, goal_coord);
            //     // Special case - successor's 2D coordinates are at the goal coordinates.
            //     if (goal_coord.x == successor_coord.x && goal_coord.y == successor_coord.y)
            //     {
            //         shortest_t = goal_coord.t;
            //         shortest_goal_index = goal_index;
            //         successor_at_goal = true;
            //         break;
            //     }

            //     int cheby_dist = std::max(
            //         std::abs(goal_coord.x - successor_coord.x),
            //         std::abs(goal_coord.y - successor_coord.y)
            //     );

            //     // [HEURISTIC b] we could take the goal point that is lowest in time to our 
            //     // current successor.
            //     int time_diff = goal_coord.t - successor_coord.t;
            //     if (time_diff < shortest_distance)
            //     {
            //         shortest_t = goal_coord.t;
            //         shortest_distance = cheby_dist;
            //         shortest_goal_index = goal_index;
            //     }

            //     // [HEURISTIC c] we could take the goal point that is lowest in time to our
            //         // current successor, and also reachable (cheby distance < time difference)
                
            //     // [HEURISTIC a] this takes the goal point that is closest in Cheby distance to 
            //     // our current successor
            //     // if (cheby_dist < shortest_distance)
            //     // {
            //     //     shortest_t = goal_coord.t;
            //     //     shortest_distance = cheby_dist;
            //     //     shortest_goal_index = goal_index;
            //     // }
            // }
            // // [CACHE] adds to the cache.
            // closest_state_lookup[successor_index_2d] = shortest_goal_index;
            // }
            
            // [Heuristic 2] we just use the shortest time index as a guiding signal. This assumes that 
            // each time step just incurs cost 1.
            // Issue: time heuristic overshadows the distance heuristic
            // full_heuristic = heuristic[successor_index_2d] + std::abs(shortest_t - successor_coord.t) / gm.target_steps * heuristic[gm.coord_to_index(start_coord_2d)];
            // full_heuristic = heuristic[successor_index_2d] + std::abs(shortest_t - successor_coord.t);

            // [Heuristic 1] Two part time heuristic.
            // I give up - as long as we reach the goal it's ok.
            // if (successor_at_goal)
            // {
            //     // full_heuristic = time_diff_succ_to_goal * gm.get_c(successor_coord_2d);
            //     full_heuristic = 0;
            // } 
            // else 
            // {
            //     full_heuristic = static_cast<long long>(heuristic[successor_index_2d]) + \
            //         std::abs(shortest_t - successor_coord.t);
            // }

            // full_heuristic = static_cast<long long>(heuristic[successor_index_2d]) + 
            //     std::abs(shortest_t - successor_coord.t);

            Q.insert(successor_index, successor_cost_to_go, heuristic[successor_index_2d], action_index);
            
            // [Heuristic 3]
            // Just use Dijkstra (will never find a solution in time)
            // Q.insert(successor_index, successor_cost_to_go, heuristic[successor_index_2d], action_index);                                
        }
    }
    return -1;
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
    static std::unordered_map<int, long long> lazy_lookup;

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
        PriorityQueue Q_dijkstra;
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
        
        stop_t = std::chrono::high_resolution_clock::now();
        duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        std::cout << "Time taken by Djikstra: " << duration_t.count() << std::endl;
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

    // Reinitializing new start robot pose.
    Coord3d robot_pose_3d(robotposeX, robotposeY, curr_time);
    PriorityQueue Q;
    GraphManager gm(x_size, y_size, target_steps, map, collision_thresh, NUMOFDIRS+1);

    // std::cout << "Robot_pose_3d_index = " << gm.coord_to_index(robot_pose_3d) << std::endl;
    // std::cout << "robotposeX = " << robotposeX << " robotposeY = " << robotposeY << " curr_time " << curr_time << std::endl;

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
                                            heuristic,
                                            lazy_lookup);
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