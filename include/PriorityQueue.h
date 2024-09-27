#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H
#include <queue>
#include <unordered_map>
#include <iostream>

struct State
{
    long long index;
    int cost_to_go;
    double heuristic;
    static double epsilon;
    int action;
    
    double getCost() const
    {
        return cost_to_go + epsilon * heuristic;
    }
    bool operator<(State const & rhs) const
    {
        return getCost() > rhs.getCost();
    }
    State(const long index, const int cost_to_go, const double heuristic) :
        index(index),
        cost_to_go(cost_to_go),
        heuristic(heuristic)
    {}
    State(const long index, const int cost_to_go, const double heuristic, const int action) :
        index(index),
        cost_to_go(cost_to_go),
        heuristic(heuristic),
        action(action)
    {}
};

class PriorityQueue
/**
 * @brief Optimized version of a priority queue that manages state with a lookup table.
 * 
 */
{    
    public: 
        std::priority_queue<State> pq;
        std::unordered_map<long long, int> heuristic_lookup;
        
        PriorityQueue();
        PriorityQueue(double epsilon);
        void insert(long long index, double cost_to_go, double heuristic);
        void insert(long long index, double cost_to_go, double heuristic, int action);
        bool empty();
        const State& top();
        void pop();
};

#endif // PRIORITY_QUEUE_H