#include "algorithm.h"

#define DISTANCE(a,b) (sqrt(pow(a,2) + pow(b,2)))

using namespace AStar;

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting spots by increasing f value - the total estimated cost
  bool operator()(const Spot* lhs, const Spot* rhs) const {
    return lhs->getf() > rhs->getf();
  }
};

bool Algorithm::Astar(Spot* start, Spot* goal, Spot** map, int rows, int cols){
    // Initialize openSet and closedSet
    typedef boost::heap::binomial_heap<Spot*,boost::heap::compare<CompareNodes>> priorityQueue;
    priorityQueue Open;
    std::vector<Spot*> closedSet;

    // //show map, start and goal
    // show_map_in_terminal(rows, cols, start, goal, map);
    
    // Initialize neighbors
    for(int j = 0; j < rows; j++){
        for(int i = 0; i < cols; i++){
            addNeighbors(&(map[j][i]), map, rows, cols);
        }
    }

    // start and destination are never obstacles
    if (start->getwall() || goal->getwall()){
        std::cout << "\033[31m" << "ERROR: One of start or goal is obstacle !" << "\033[0m" << std::endl;
        return false;
    }

    // Add start into openSet
    Open.push(start);
    start->update_open(true);

    // start looping

    while(!Open.empty()){
        // find lowest f_cost index in openSet
        Spot* current = Open.top();
        // std::cout << "current:" << std::endl;
        // std::cout << "(" << current -> geti() << ", " << current -> getj() << ")  wall: " << current->getwall() 
        // << "  g: " << current->getg() << "  h: " << current->geth() << "  f: " << current->getf() << std::endl;

        // If current is the goal, we are done!
        if(current == goal){
            return true;
        }

        // Else, remove current and add into closedSet
        Open.pop();
        current->update_open(false);
        current->update_closed(true);
        
        // Add new spot into openSet
        const std::vector<Spot*> neighbors = current->getneighbors();
        
        for(int i = 0; i < neighbors.size(); i++){
            Spot* neighbor = neighbors[i];
            if(!neighbor->getclosed() && !neighbor->getwall()){
                double temp = current->getg() + DISTANCE(current->geti()-neighbor->geti(),current->getj()-neighbor->getj());
                bool newpath = false;
                if(neighbor->getopen()){
                    if(temp < neighbor->getg()){
                        neighbor->update_g(temp);
                        newpath = true;
                    }
                }
                else{
                    neighbor->update_g(temp);
                    newpath = true;
                    Open.push(neighbor);
                    neighbor->update_open(true);
                }
                if(newpath){
                    neighbor->update_h(heuristics(neighbor->geti()-goal->geti(),neighbor->getj()-goal->getj()));
                    neighbor->update_f();
                    neighbor->update_prev(current);
                }
            }
        }
        // sleep(1);
    }
    return false;
}

inline double Algorithm::heuristics(int a, int b){
    return DISTANCE(a,b);
}

std::vector<Spot*> Algorithm::quicksort(std::vector<Spot*> Set, int left, int right){
    if(left > right){ return Set;}
    int i = left;
    int j = right;
    Spot* base = Set[left];
    while(i<j){
        while(i < j && Set[j]->getf() < base->getf()){ j--;}
        while(i < j && Set[j]->getf() >= base->getf()){ i++;}
        if(i < j){
            Spot* temp = Set[i];
            Set[i] = Set[j];
            Set[j] = temp;
        }
    }
    Spot* temp = Set[left];
    Set[left] = Set[j];
    Set[j] = temp;
    Set = quicksort(Set, left, i-1);
    Set = quicksort(Set, i+1, right);
    return Set;
}

void Algorithm::addNeighbors(Spot* spot, Spot** map, int rows, int cols){
    int i = spot->geti();
    int j = spot->getj();
    std::vector<Spot*> neighbors;
    if (i < cols-1){
        neighbors.push_back(&(map[j][i+1]));
    }
    if (i > 0){
        neighbors.push_back(&(map[j][i-1]));
    }
    if (j < rows-1){
        neighbors.push_back(&(map[j+1][i]));
    }
    if (j > 0){
        neighbors.push_back(&(map[j-1][i]));
    }
    // diagonals
    if (i < cols-1 && j < rows-1){
        neighbors.push_back(&(map[j+1][i+1]));
    }
    if (i > 0 && j > 0){
        neighbors.push_back(&(map[j-1][i-1]));
    }
    if (i < cols-1 && j > 0){
        neighbors.push_back(&(map[j-1][i+1]));
    }
    if (i > 0 && j < rows-1){
        neighbors.push_back(&(map[j+1][i-1]));
    }
    spot->update_neighbors(neighbors);
}

void show_map_in_terminal(int rows, int cols, Spot* start, Spot* goal, Spot** map){
    for(int j = rows-1; j >= -1; j--){
        if(j < 10 && j >= 0) std::cout << " ";
        std::cout << " " << j;
        if(j > -1){
            for(int i = 0; i < cols; i++){
                if(start->geti() == i && start->getj() == j) std::cout << "\033[32m";
                else if(goal->geti() == i && goal->getj() == j) std::cout << "\033[33m";
                if(map[j][i].getwall()){
                    std::cout << "  1";
                }
                else{
                    std::cout << "  0";
                }
                std::cout << "\033[0m";
            }
        }
        else if(j == -1){
            for(int i = 0; i < cols; i++){
                if(i < 10) std::cout << "  " << i;
                else std::cout << " " << i;
            }
        }
        std::cout << std::endl;
    }
}