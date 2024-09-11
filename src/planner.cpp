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

// Define the custom State class
struct State {
    int x, y;                  // Coordinates or unique identifiers
    double g_cost, h_cost;     // A* costs
    double f_cost;             // Total cost (g_cost + h_cost)
    using FibonacciHeapType = boost::heap::fibonacci_heap<State*, boost::heap::compare<StateComparator>>;
    using HandleType = FibonacciHeapType::handle_type; // Correct handle type
    HandleType heap_handle; // Heap handle

    // Constructor
    State(int x, int y, double g, double h)
        : x(x), y(y), g_cost(g), h_cost(h) {
        f_cost = g_cost + h_cost;
    }
};

struct StateComparator {
    bool operator()(const State* lhs, const State* rhs) const {
        // Use f_cost for comparison; lesser f_cost has higher priority (min-heap behavior)
        return lhs->f_cost > rhs->f_cost; // '>' because Boost's heap is a max-heap by default
    }
};


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
    
    // Fibonacci heap 
    boost::heap::fibonacci_heap<State*, boost::heap::compare<StateComparator>> fib_heap;

    State* start = new State(0, 0, 0, 10);
    State* middle = new State(1, 1, 5, 5);
    State* end = new State(2, 2, 10, 0);

    start->heap_handle = fib_heap.push(start);
    middle->heap_handle = fib_heap.push(middle);
    end->heap_handle = fib_heap.push(end);

    // Extract the minimum (lowest f_cost)
    State* min_state = fib_heap.top();
    fib_heap.pop();
    
    std::cout << "Extracted state at: (" << min_state->x << ", " << min_state->y << ")" 
              << " with f_cost: " << min_state->f_cost << std::endl;

    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    int goalposeX = target_traj[target_steps-1];
    int goalposeY = target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
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