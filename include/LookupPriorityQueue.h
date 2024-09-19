#ifndef LOOKUP_PRIORITY_QUEUE_H
#define LOOKUP_PRIORITY_QUEUE_H
#include "boost/heap/fibonacci_heap.hpp"
#include <unordered_map>
#include <iostream>
typedef typename boost::heap::fibonacci_heap<float> state_t;
typedef typename state_t::handle_type handle_t;

struct State
{
    int index; // For debugging
    double cost_to_go; // g, can be changed
    double heuristic; // h, won't be changed.
    double getCost() const
    {
        return cost_to_go + heuristic;
    }
    bool operator<(State const & rhs) const
    {
        return getCost() > rhs.getCost();
    }
    State(const int index, const double cost_to_go, const double heuristic) :
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
        
        LookupPriorityQueue();
        void insert(int index, double cost_to_go, double heuristic);
        void update(int index, double cost_to_go);
        bool empty();
        const State& top();
        void pop();

        // look for key in lookup_table
        bool index_in_lookup_table(int index);
        HandleType get_state_handle(int index);
        
    private:
        HeapType pq; // actual priority queue. value corresponds to g()
        std::unordered_map<int, HandleType> lookup_table; // will be dynamically updated when nodes are in PQ.
};

#endif // LOOKUP_PRIORITY_QUEUE_H