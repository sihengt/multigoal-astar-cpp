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
    graph_manager.populate_l_goals(target_traj);

    Coord2d robot_pose_2d(robotposeX, robotposeY);
    Coord3d robot_pose(robotposeX, robotposeY, curr_time);

    std::vector<int> successors;
    successors = graph_manager.get_successors(robot_pose);
    
    // Insert start state
    Q.insert(graph_manager.coord_to_index_3d(robot_pose), 0, graph_manager.compute_heuristic(robot_pose));

    // Planning loop
    while (true) // s_{goal} is not expanded, OPEN != 0.
    {
        if (Q.empty())
            break;
        const State& current_state = Q.top();
        Coord3d debug_coords = graph_manager.index_to_coord_3d(current_state.index);
        std::cout << "Currently at (" << debug_coords.x << "," << debug_coords.y << "," << debug_coords.t << ")" << std::endl;
        graph_manager.add_to_closed(current_state.index);
        
        // TODO: so clunky, I want to re-think the data structures here.
        Coord3d coord = graph_manager.index_to_coord_3d(current_state.index);
        successors = graph_manager.get_successors(coord);
        
        for (int successor_index_3d : successors)
        {
            double succ_cost_to_go, succ_heuristic;
            // Dummy goal state
            if (successor_index_3d == -1)
            {
                succ_cost_to_go = current_state.cost_to_go;
                succ_heuristic = 0;
            }
            
            // Every other normal state
            else
            {
                Coord3d successor_coords = graph_manager.index_to_coord_3d(successor_index_3d);
                succ_cost_to_go = current_state.cost_to_go + graph_manager.get_c(successor_coords);
                succ_heuristic = graph_manager.compute_heuristic(successor_coords);
            }
            
            // Successor is not in OPEN, so we insert it into the queue.
            if (!Q.index_in_lookup_table(successor_index_3d))
                Q.insert(successor_index_3d, succ_cost_to_go, succ_heuristic);
            
            // Successor is in OPEN, we need to check if it's current cost to go is > previous state's cost to go + cost
            else
            {
                auto succ_state_ptr = Q.get_state_handle(successor_index_3d);
                // We found a lower cost to the successor
                if ((*succ_state_ptr).cost_to_go > succ_cost_to_go)
                    Q.update(successor_index_3d, succ_cost_to_go);
            }
        }
    }
    
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    int goalposeX = target_traj[target_steps - 1];
    int goalposeY = target_traj[target_steps - 1 + target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    double olddisttotarget = (double)sqrt(
        ((robotposeX - goalposeX) * (robotposeX - goalposeX) + (robotposeY - goalposeY) * (robotposeY - goalposeY))
    );
    double disttotarget;
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                if(disttotarget < olddisttotarget)
                {
                    olddisttotarget = disttotarget;
                    bestX = dX[dir];
                    bestY = dY[dir];
                }
            }
        }
    }
    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
    return;
}