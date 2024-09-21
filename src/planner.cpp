/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <math.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <iostream>
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
    LookupPriorityQueue &Q,
    GraphManager &gm,
    int start,
    std::vector<int> &heuristic
)
{
    // auto start_t = std::chrono::high_resolution_clock::now();
    std::vector<int> successors(8, -1);
    
    // Insert goal as start state, with 0 for cost-to-go and 0 for heuristic (cause we djikstra-ing)
    for (int goal : gm.l_goals)
        Q.insert(goal, 0, 0);

    // auto stop_t = std::chrono::high_resolution_clock::now();
    // auto duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
    // std::cout << "Time taken by goal insertion: " << duration_t.count() << std::endl;

    Coord2d current_coord(0, 0);
    Coord2d successor_coord(0, 0);
    int current_state_index, current_g, current_cost;
    while (!Q.empty())
    {
        // start_t = std::chrono::high_resolution_clock::now();
        const State current_state = Q.top();
        current_state_index = current_state.index;
        current_g = current_state.cost_to_go;
        heuristic[current_state_index] = current_g;
        
        // If current_state is the start_state, we've found a solution.
        if (current_state.index == start)
        {   
            // gm.index_to_coord(current_state.index, current_coord);
            // std::cout << "Currently at: (" << current_coord.x << "," << current_coord.y << ")" << std::endl;
            break;
        }
        Q.pop();

        // Expanded node is optimal and won't be visited again.
        gm.add_to_closed(current_state_index);

        // Getting successors.
        gm.index_to_coord(current_state_index, current_coord);
        gm.get_successors(current_coord, successors);
        current_cost = gm.get_c(current_coord);
        
        // stop_t = std::chrono::high_resolution_clock::now();
        // duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        // std::cout << "Time taken to get successors: " << duration_t.count() << std::endl;

        int successor_cost_to_goal;
        for (int successor_index : successors)
        {   
            if (successor_index == -1)
                continue;
            
            // gm.index_to_coord(successor_index, successor_coord);

            // For the backward Djikstra, we've established that the cost-to-go is whatever cost + current node's cost.
            successor_cost_to_goal = current_g + current_cost;
            
            // Successor is not in OPEN, so we insert it into the queue.
            if (Q.lookup_table[successor_index].node_ == nullptr)
            {
                Q.insert(successor_index, successor_cost_to_goal, 0);
            }
                
            // Successor is in OPEN, we need to check if it's current cost to go is > previous state's cost to go + cost
            else
            {
                Q.check_cost_and_update(successor_index, successor_cost_to_goal);
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
    static std::vector<int> heuristic;
    
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    Coord2d robot_pose_2d(robotposeX, robotposeY);

    // Initialization of OPEN (implemented as fibonacci heap priority queue)
    LookupPriorityQueue Q(x_size, y_size);

    GraphManager gm(x_size, y_size, target_steps, map, collision_thresh, NUMOFDIRS);
    gm.init_actions(dX, dY, NUMOFDIRS);


    if (!is_cached)
    {
        heuristic.resize(x_size * y_size);
        std::fill(heuristic.begin(), heuristic.end(), std::numeric_limits<int>::max());
        auto start_t = std::chrono::high_resolution_clock::now();
        gm.populate_l_goals(target_traj, robot_pose_2d);
        multi_backwards_djikstra(
            Q,
            gm,
            gm.coord_to_index(robot_pose_2d),
            heuristic);
        auto stop_t = std::chrono::high_resolution_clock::now();
        auto duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        std::cout << "Time taken by Djikstra: " << duration_t.count() << std::endl;
        
    }


    //     prev_action = policy[current_index];
    //     Coord2d current_state = graph_manager.index_to_coord_2d(current_index);
    //     Coord2d previous_state(current_state.x - prev_action.action_x, current_state.y - prev_action.action_y);
    //     current_index = graph_manager.coord_to_index_2d(previous_state);
    //     if (current_index == start_index)
    //     {
    //         break;
    //     }

    // auto stop_t = std::chrono::high_resolution_clock::now();
    // auto duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
    // std::cout << "Time taken by Djikstra: " << duration_t.count() << std::endl;


    // Some sanity checks:
    // For each of the goal, see if we can reach back to start
    // for (auto goal : gm.l_goals)
    // {
    //     int current_t = 0;
    //     int current_state = goal;
    //     while(true)
    //     {
    //         Action action_to_take(dijkstra_policy[current_state]);
    //         Coord2d prev_state = 

    //         if (current_t >= MAX_TIMESTEP)
    //         {
    //             std::cout << "FAILED TO FIND PATH TO START FOR INDEX: " << goal << std::endl;
    //             break;
    //         }
    //         current_t++;
    //     }
    // }

    // std::vector<Node> successors;
    
    // Insert start state
    // Q.insert(gm.coord_to_index_2d(robot_pose_2d), 0, gm.compute_heuristic(robot_pose_2d));
    // Q.insert(graph_manager.coord_to_index_3d(robot_pose), 0, graph_manager.compute_heuristic(robot_pose));

    // Planning loop
    // while (true) // s_{goal} is not expanded, OPEN != 0.
    // {
    //     if (Q.empty())
    //         break;
    //     const State current_state = Q.top();
    //     Q.pop();

    //     // Coord3d debug_coords = graph_manager.index_to_coord_3d(current_state.index);
    //     // std::cout << "Currently at (" << debug_coords.x << "," << debug_coords.y << "," << debug_coords.t << ")" << std::endl;
    //     Coord2d debug_coords = graph_manager.index_to_coord_2d(current_state.index);
    //     // std::cout << "Currently at (" << debug_coords.x << "," << debug_coords.y << ")" << std::endl;
    //     graph_manager.add_to_closed(current_state.index);
        
    //     if (debug_coords.x == goalposeX && debug_coords.y == goalposeY)
    //     {
    //         // std::cout << "GOAL REACHED!" << std::endl;
    //         break;
    //     }

    //     // TODO: so clunky, I want to re-think the data structures here.
    //     Coord2d coord = graph_manager.index_to_coord_2d(current_state.index);
    //     successors = graph_manager.get_successors(coord);
        
    //     for (Node successor_node : successors)
    //     {
    //         double succ_cost_to_go, succ_heuristic;
            
    //         // Every other normal state
    //         // Coord3d successor_coords = graph_manager.index_to_coord_3d(successor_index_3d);
    //         Coord2d successor_coords = graph_manager.index_to_coord_2d(successor_node.index);
    //         succ_cost_to_go = current_state.cost_to_go + graph_manager.get_c(successor_coords);
    //         succ_heuristic = graph_manager.compute_heuristic(successor_coords);

            
    //         // Successor is not in OPEN, so we insert it into the queue.
    //         if (!Q.index_in_lookup_table(successor_node.index))
    //         {
    //             Q.insert(successor_node.index, succ_cost_to_go, succ_heuristic);
    //             policy.insert({successor_node.index, successor_node.action_to_get_here});
    //         }
                
            
    //         // Successor is in OPEN, we need to check if it's current cost to go is > previous state's cost to go + cost
    //         else
    //         {
    //             auto succ_state_ptr = Q.get_state_handle(successor_node.index);
    //             // We found a lower cost to the successor
    //             if ((*succ_state_ptr).cost_to_go > succ_cost_to_go)
    //             {
    //                 Q.update(successor_node.index, succ_cost_to_go);
    //                 policy[successor_node.index] = successor_node.action_to_get_here;
    //             }
    //         }
    //     }
    // }

    // int start_index = graph_manager.coord_to_index_2d(robot_pose_2d);
    // int current_index = goalIndex;
    // Action prev_action;
    // while (true)
    // {
    //     prev_action = policy[current_index];
    //     Coord2d current_state = graph_manager.index_to_coord_2d(current_index);
    //     Coord2d previous_state(current_state.x - prev_action.action_x, current_state.y - prev_action.action_y);
    //     current_index = graph_manager.coord_to_index_2d(previous_state);
    //     if (current_index == start_index)
    //     {
    //         break;
    //     }
    // }

    // action_ptr[0] = robotposeX + prev_action.action_x;
    // action_ptr[1] = robotposeY + prev_action.action_y;
    return;
}