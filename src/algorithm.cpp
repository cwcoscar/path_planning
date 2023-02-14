#include "algorithm.h"

using namespace AStar;

bool Algorithm::Astar(Spot* start, Spot* goal, Spot** map, int rows, int cols){
    // Initialize openSet and closedSet
    std::vector<Spot*> openSet;
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
    openSet.push_back(start);
    start->update_open(true);

    // start looping

    while(openSet.size() != 0){
        // find lowest f_cost index in openSet
        Spot* current = findLowestf(openSet);
        // std::cout << "current:" << std::endl;
        // std::cout << "(" << current -> geti() << ", " << current -> getj() << ")  wall: " << current->getwall() 
        // << "  g: " << current->getg() << "  h: " << current->geth() << "  f: " << current->getf() << std::endl;

        // If current is the goal, we are done!
        if(current == goal){
            return true;
        }

        // Else, remove current and add into closedSet
        openSet.pop_back();
        current->update_open(false);
        current->update_closed(true);
        
        // Add new spot into openSet
        const std::vector<Spot*> neighbors = current->getneighbors();
        
        for(int i = 0; i < neighbors.size(); i++){
            Spot* neighbor = neighbors[i];
            if(!neighbor->getclosed() && !neighbor->getwall()){
                double temp = current->getg() + heuristics(current,neighbor);
                // double temp = current->getg() + 1;
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
                    openSet.push_back(neighbor);
                    neighbor->update_open(true);
                }
                if(newpath){
                    neighbor->update_h(heuristics(neighbor,goal));
                    neighbor->update_f();
                    neighbor->update_prev(current);
                }
            }
        }
        // sleep(1);
    }
    return false;
}

double Algorithm::heuristics(Spot* neighbor, Spot* goal){
    // double cost = abs(goal->geti() - neighbor->geti()) + abs(goal->getj() - neighbor->getj());
    double a = pow(goal->geti()-neighbor->geti(),2);
    double b = pow(goal->getj()-neighbor->getj(),2);
    double cost = sqrt(a + b);
    return cost;
}

Spot* Algorithm::findLowestf(std::vector<Spot*>& Set){
    std::vector<Spot*> openSet = Set;
    // for(int i = 0; i < openSet.size(); i++){
    //     std::cout << std::fixed << std::setprecision(1);
    //     std::cout << openSet[i]->getf() << " ";
    // }
    // std::cout << std::endl;
    // openSet = quicksort(openSet, 0, openSet.size()-1);
    // for(int i = 0; i < openSet.size(); i++){
    //     std::cout << openSet[i]->getf() << " ";
    // }
    int index = 0;
    double min = INT_MAX;
    for(int i = 0; i < openSet.size(); i++){
        if(min > openSet[i]->getf()){
            index = i;
            min = openSet[i]->getf();
        }
    }
    Spot* temp = openSet[index];
    openSet[index] = openSet[openSet.size()-1];
    openSet[openSet.size()-1] = temp;
    // for(int i = 0; i < openSet.size(); i++){
    //     std::cout << openSet[i]->getf() << " ";
    // }
    Set = openSet;
    return Set[Set.size()-1];
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