#include "LookupPriorityQueueVector.h"

LookupPriorityQueueVector::LookupPriorityQueueVector(int max_x, int max_y) : 
    max_x(max_x),
    max_y(max_y),
    lookup_table(max_x * max_y)
{}

void LookupPriorityQueueVector::insert(int index, double cost_to_go, double heuristic)
{
    State s(index, cost_to_go, heuristic);
    lookup_table[index] = pq.push(s);
    // .insert({index, pq.push(s)});
}

void LookupPriorityQueueVector::check_cost_and_update(int index, int new_cost)
{
    auto succ_state_ptr = lookup_table[index];
    State s_new = (*succ_state_ptr); // Does this create a copy?
    s_new.cost_to_go = new_cost;
    pq.update(lookup_table[index], s_new);
}

const State& LookupPriorityQueueVector::top()
{
    return pq.top(); // TODO: I don't think I need to create a new State everytime right?
}

void LookupPriorityQueueVector::pop()
{
    // Getting reference to remove from the lookup table
    const State& s = pq.top();
    lookup_table[s.index] = HandleType();
    pq.pop();
}

bool LookupPriorityQueueVector::empty()
{
    return pq.empty();
}
