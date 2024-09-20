/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <math.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <iostream>

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

int backwards_djikstra(LookupPriorityQueue &Q, GraphManager &gm, int* map, int start, int goal)
{
    // Ensuring priority queue and graph manager's closed set are empty
    Q.reset();
    gm.reset_closed();
    
    // Add start index as goal
    gm.l_goals.push_back(start);

    std::vector<Node> successors;
    
    // Insert goal as start state, with 0 for cost-to-go and 0 for heuristic (cause we djikstra-ing)
    Q.insert(goal, 0, 0);
    while (true)
    {
        // If priority queue is empty, algorithm ends, and an optimal solution has not been found.
        if (Q.empty())
            return -1;

        const State current_state = Q.top();
        
        // If current_state is the start_state, we've found a solution.
        if (current_state.index == start)
            return current_state.cost_to_go;
        Q.pop();

        // Expanded node is optimal and won't be visited again.
        gm.add_to_closed(current_state.index);

        // Getting successors.
        Coord2d coord = gm.index_to_coord_2d(current_state.index);
        successors = gm.get_successors(coord);
        
        for (Node& successor_node : successors)
        {
            double successor_cost_to_goal, succ_heuristic;
            
            // For the backward Djikstra, we've established that the cost-to-go is whatever cost + current node's cost.
            successor_cost_to_goal = current_state.cost_to_go + gm.get_c(coord);
            
            // Successor is not in OPEN, so we insert it into the queue.
            if (!Q.index_in_lookup_table(successor_node.index))
                Q.insert(successor_node.index, successor_cost_to_goal, 0);
                
            // Successor is in OPEN, we need to check if it's current cost to go is > previous state's cost to go + cost
            else
            {
                auto succ_state_ptr = Q.get_state_handle(successor_node.index);
                // We found a lower cost to the successor
                if ((*succ_state_ptr).cost_to_go > successor_cost_to_goal)
                    Q.update(successor_node.index, successor_cost_to_goal);
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
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // Initialization of OPEN (implemented as fibonacci heap priority queue)
    LookupPriorityQueue Q;
    
    GraphManager graph_manager(x_size, y_size, target_steps, map, collision_thresh, NUMOFDIRS);
    graph_manager.init_actions(dX, dY, NUMOFDIRS);
    // graph_manager.populate_l_goals(target_traj);
    
    // Setting up goal
    int goalposeX = target_traj[target_steps - 1];
    int goalposeY = target_traj[target_steps - 1 + target_steps];
    Coord2d goalcoord = Coord2d(goalposeX, goalposeY);
    int goalIndex = graph_manager.coord_to_index_2d(goalcoord);
    graph_manager.l_goals.push_back(goalIndex);

    Coord2d robot_pose_2d(robotposeX, robotposeY);
    // Coord3d robot_pose(robotposeX, robotposeY, curr_time);
    std::cout << "hello" << std::endl;
    backwards_djikstra(Q, graph_manager, map, graph_manager.coord_to_index_2d(robot_pose_2d), goalIndex);

    std::vector<Node> successors;
    
    // Insert start state
    Q.insert(graph_manager.coord_to_index_2d(robot_pose_2d), 0, graph_manager.compute_heuristic(robot_pose_2d));
    // Q.insert(graph_manager.coord_to_index_3d(robot_pose), 0, graph_manager.compute_heuristic(robot_pose));

    std::unordered_map<int, std::pair<int, int>> policy;

    // Planning loop
    while (true) // s_{goal} is not expanded, OPEN != 0.
    {
        if (Q.empty())
            break;
        const State current_state = Q.top();
        Q.pop();

        // Coord3d debug_coords = graph_manager.index_to_coord_3d(current_state.index);
        // std::cout << "Currently at (" << debug_coords.x << "," << debug_coords.y << "," << debug_coords.t << ")" << std::endl;
        Coord2d debug_coords = graph_manager.index_to_coord_2d(current_state.index);
        // std::cout << "Currently at (" << debug_coords.x << "," << debug_coords.y << ")" << std::endl;
        graph_manager.add_to_closed(current_state.index);
        
        if (debug_coords.x == goalposeX && debug_coords.y == goalposeY)
        {
            // std::cout << "GOAL REACHED!" << std::endl;
            break;
        }

        // TODO: so clunky, I want to re-think the data structures here.
        Coord2d coord = graph_manager.index_to_coord_2d(current_state.index);
        successors = graph_manager.get_successors(coord);
        
        for (Node successor_node : successors)
        {
            double succ_cost_to_go, succ_heuristic;
            
            // Every other normal state
            // Coord3d successor_coords = graph_manager.index_to_coord_3d(successor_index_3d);
            Coord2d successor_coords = graph_manager.index_to_coord_2d(successor_node.index);
            succ_cost_to_go = current_state.cost_to_go + graph_manager.get_c(successor_coords);
            succ_heuristic = graph_manager.compute_heuristic(successor_coords);

            
            // Successor is not in OPEN, so we insert it into the queue.
            if (!Q.index_in_lookup_table(successor_node.index))
            {
                Q.insert(successor_node.index, succ_cost_to_go, succ_heuristic);
                policy.insert({successor_node.index, successor_node.action_to_get_here});
            }
                
            
            // Successor is in OPEN, we need to check if it's current cost to go is > previous state's cost to go + cost
            else
            {
                auto succ_state_ptr = Q.get_state_handle(successor_node.index);
                // We found a lower cost to the successor
                if ((*succ_state_ptr).cost_to_go > succ_cost_to_go)
                {
                    Q.update(successor_node.index, succ_cost_to_go);
                    policy[successor_node.index] = successor_node.action_to_get_here;
                }
            }
        }
    }

    int start_index = graph_manager.coord_to_index_2d(robot_pose_2d);
    int current_index = goalIndex;
    std::pair<int, int> prev_action;
    while (true)
    {
        prev_action = policy[current_index];
        Coord2d current_state = graph_manager.index_to_coord_2d(current_index);
        Coord2d previous_state(current_state.x - prev_action.first, current_state.y - prev_action.second);
        current_index = graph_manager.coord_to_index_2d(previous_state);
        if (current_index == start_index)
        {
            break;
        }
    }

    action_ptr[0] = robotposeX + prev_action.first;
    action_ptr[1] = robotposeY + prev_action.second;
    return;
}