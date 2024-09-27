#include "PriorityQueue.h"

double State::epsilon = 0.0;

PriorityQueue::PriorityQueue() {}

PriorityQueue::PriorityQueue(double epsilon)
{
    State::epsilon = epsilon;
}

void PriorityQueue::insert(long long index, double cost_to_go, double heuristic)
{
    pq.emplace(index, cost_to_go, heuristic);
}

void PriorityQueue::insert(long long index, double cost_to_go, double heuristic, int action)
{
    pq.emplace(index, cost_to_go, heuristic, action);
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
