#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "spot.h"

#include <nav_msgs/OccupancyGrid.h>

namespace AStar {

    class Algorithm {
        public:
        /// The deault constructor
        Algorithm() {}
        bool Astar(Spot* start, Spot* goal, Spot** map, int rows, int cols);
        double heuristics(int a, int b);
        Spot* findLowestf(std::vector<Spot*>& Set);
        std::vector<Spot*> quicksort(std::vector<Spot*> Set, int left, int right);
        void addNeighbors(Spot* spot, Spot** map, int height, int width);
    };
}
#endif // ALGORITHM_H
