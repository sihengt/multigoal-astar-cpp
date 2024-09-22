#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H
#include <queue>
#include <unordered_map>
#include <iostream>
#include <LookupPriorityQueue.h>

class PriorityQueue
/**
 * @brief Optimized version of a priority queue that manages state with a lookup table.
 * 
 */
{    
    public: 
        // using HeapType = boost::heap::fibonacci_heap<State>;
        // using HandleType = HeapType::handle_type;
        std::priority_queue<State> pq;
        std::unordered_map<long long, int> heuristic_lookup;
        
        PriorityQueue();
        void insert(long long index, double cost_to_go, double heuristic);
        bool empty();
        const State& top();
        void pop();
        bool find(long long index);
};

#endif // PRIORITY_QUEUE_H