#ifndef GRAPH_MANAGER_HPP
#define GRAPH_MANAGER_HPP
#include <unordered_set>
#include <iostream>

struct Coord3d
{
    int x;
    int y;
    int t;
    Coord3d(int x, int y, int t) : x(x), y(y), t(t)
    {}
};

struct Coord2d
{
    int x;
    int y;
    Coord2d(int x, int y) : x(x), y(y)
    {}
};

class GraphManager
{
    std::unordered_set<int> closed_queue;
    public:
        int coord_to_index_3d(Coord3d &coord);
        Coord3d index_to_coord_3d(int index);

        int coord_to_index_2d(Coord2d &coord);
        Coord2d index_to_coord_2d(int index);

        void add_to_closed(int index);
        std::vector<int> get_successors(Coord2d& robot_pose);
        void init_actions(int* dX, int* dY, int num_dirs);
        
        GraphManager(int max_x, int max_y, int* map, int collision_thresh, int num_dirs);

    private:
        const int max_x; // max x dimensions of the map
        const int max_y; // max y dimensions of the map
        const int num_dirs;   // 
        int* map;
        int collision_thresh;
        std::vector<std::pair<int, int>> actions; // assumes 2D 
};

#endif // GRAPH_MANAGER_HPP