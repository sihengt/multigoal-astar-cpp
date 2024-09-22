#include "PriorityQueue.h"

PriorityQueue::PriorityQueue() {}

void PriorityQueue::insert(long long index, double cost_to_go, double heuristic)
{
    pq.emplace(index, cost_to_go, heuristic);
}

const State& PriorityQueue::top()
{
    return pq.top(); // TODO: I don't think I need to create a new State everytime right?
}

void PriorityQueue::pop()
{
    pq.pop();
}

bool PriorityQueue::empty()
{
    return pq.empty();
}

bool PriorityQueue::find(long long index)
{
    return pq.empty();
}
