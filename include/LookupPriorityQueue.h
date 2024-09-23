#ifndef LOOKUP_PRIORITY_QUEUE_H
#define LOOKUP_PRIORITY_QUEUE_H
#include "boost/heap/fibonacci_heap.hpp"
#include <unordered_map>
#include <iostream>

struct State
{
    long index;
    int cost_to_go; // g, can be changed
    double heuristic; // h, won't be changed.
    double getCost() const
    {
        return cost_to_go + 10.0*heuristic;
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
};

class LookupPriorityQueue
/**
 * @brief Optimized version of a priority queue that manages state with a lookup table.
 * 
 */
{    
    public: 
        using HeapType = boost::heap::fibonacci_heap<State>;
        using HandleType = HeapType::handle_type;
        
        LookupPriorityQueue(int max_x, int max_y);
        LookupPriorityQueue(int max_x, int max_y, int max_t);
        void insert(long index, double cost_to_go, double heuristic);
        void check_cost_and_update(long index, int cost_to_go);
        bool empty();
        const State& top();
        void pop();

        HeapType pq; // actual priority queue. value corresponds to g()
        std::unordered_map<int, HandleType> lookup_table;
    private:
        int max_x;
        int max_y;
};

#endif // LOOKUP_PRIORITY_QUEUE_H