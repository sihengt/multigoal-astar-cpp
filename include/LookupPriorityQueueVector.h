#ifndef LOOKUP_PRIORITY_QUEUE_VECTOR_H
#define LOOKUP_PRIORITY_QUEUE_VECTOR_H
#include "boost/heap/fibonacci_heap.hpp"
#include <unordered_map>
#include <iostream>

class LookupPriorityQueueVector
/**
 * @brief Optimized version of a priority queue that manages state with a lookup table.
 * 
 */
{    
    public: 
        using HeapType = boost::heap::fibonacci_heap<State>;
        using HandleType = HeapType::handle_type;
        
        LookupPriorityQueueVector(int max_x, int max_y);
        void insert(int index, double cost_to_go, double heuristic);
        void check_cost_and_update(int index, int cost_to_go);
        bool empty();
        const State& top();
        void pop();

        HeapType pq; // actual priority queue. value corresponds to g()
        std::vector<HandleType> lookup_table;
    private:
        int max_x;
        int max_y;
};

#endif // LOOKUP_PRIORITY_QUEUE_VECTOR_H