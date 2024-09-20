#include "LookupPriorityQueue.h"

using HeapType = boost::heap::fibonacci_heap<State>;
using HandleType = HeapType::handle_type;

// Helper overload for printing.
std::ostream& operator << (std::ostream& os, const State& state)
{
    return os << "Index: " << state.index << " | " << "Cost: " << state.getCost() << std::endl;
}

LookupPriorityQueue::LookupPriorityQueue()
{}

void LookupPriorityQueue::insert(int index, double cost_to_go, double heuristic)
{
    State s(index, cost_to_go, heuristic);
    lookup_table.insert({index, pq.push(s)});
}

void LookupPriorityQueue::update(int index, double cost_to_go)
{
    State s_new = *(lookup_table[index]); // Does this create a copy?
    s_new.cost_to_go = cost_to_go;
    lookup_table.erase(index);
    lookup_table.insert({index, pq.push(s_new)});
}

const State LookupPriorityQueue::top()
{
    return pq.top(); // TODO: I don't think I need to create a new State everytime right?
}

void LookupPriorityQueue::pop()
{
    // Getting reference to remove from the lookup table
    const State& s = pq.top();
    lookup_table.erase(s.index);
    pq.pop();
}

bool LookupPriorityQueue::index_in_lookup_table(int index)
{
    if (lookup_table.find(index) != lookup_table.end())
        return true;
    return false;
}

bool LookupPriorityQueue::empty()
{
    return pq.empty();
}

HandleType LookupPriorityQueue::get_state_handle(int index)
{
    return lookup_table[index];
}