#include "boost/heap/fibonacci_heap.hpp"
#include <iostream>

struct State
{
    double x, y;
    double cost;
    // So we can get least cost out first.
    bool operator<(State const & rhs) const
    {
        return cost > rhs.cost;
    }
    State(double x, double y, double cost) :
        x(x),
        y(y),
        cost(cost)
    {}
};

// Helper overload for printing.
std::ostream& operator << (std::ostream& os, const State& state)
{
    return os << "Coordinates: (" << state.x << "," << state.y << ") | " << "Cost: " << state.cost << std::endl;
}


int main()
{
    typedef typename boost::heap::fibonacci_heap<State> state_t;
    typedef typename state_t::handle_type handle_t;
    
    state_t pq;

    handle_t t1 = pq.push(State(1,2,3));
    handle_t t2 = pq.push(State(3,4,5));
    handle_t t3 = pq.push(State(5,6,7));

    pq.update(t2, State(3,4,1));

    std::cout << "Prioritiy Queue: popped elements" << std::endl;
    std::cout << pq.top();
    pq.pop(); 
    std::cout << pq.top();
    pq.pop();
    std::cout << pq.top();
    pq.pop();
    std::cout << std::endl;
}
