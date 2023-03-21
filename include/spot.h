#ifndef SPOT_H
#define SPOT_H

#include <cmath>
#include <climits>
#include <vector>


namespace AStar {

    class Spot{
        private:
            int i_ = 0;
            int j_ = 0;
            double x_ = 0;
            double y_ = 0;
            bool wall_ = false;
            double cost_g_ = 0;
            double cost_h_ = 0;
            double cost_f_ = INT_MAX;
            Spot* previous_ = nullptr;
            std::vector<Spot*> neighbors_;
            bool inopenset_ = false;
            bool inclosedset_ = false;

        public:
            Spot():Spot(0,0,false,1){};
            Spot(int i, int j, bool obstacle, double edgeLength){
                i_ = i;
                j_ = j;
                x_ = edgeLength/2 + (double)(i) * edgeLength;
                y_ = edgeLength/2 + (double)(j) * edgeLength;
                wall_ = obstacle;
            }
            Spot(int i, int j, double x, double y, bool obstacle, double edgeLength){
                i_ = i;
                j_ = j;
                x_ = x;
                y_ = y;
                wall_ = obstacle;
            }
            int geti(){ return i_;}
            int getj(){ return j_;}
            double getx(){ return x_;}
            double gety(){ return y_;}
            bool getwall(){ return wall_;}
            double getg(){ return cost_g_;}
            double geth(){ return cost_h_;}
            double getf() const{ return cost_f_;}
            double getopen(){ return inopenset_;}
            double getclosed(){ return inclosedset_;}
            Spot* getprev(){ return previous_;}
            const std::vector<Spot*> getneighbors(){ return neighbors_;}

            void update_open(bool new_open){inopenset_ = new_open;}
            void update_closed(bool new_closed){inclosedset_ = new_closed;}
            void update_g(double new_g){cost_g_ = new_g;}
            void update_h(double new_h){cost_h_ = new_h;}
            void update_f(){cost_f_ = cost_g_ + cost_h_;}
            void update_prev(Spot* previous){previous_ = previous;}
            void update_neighbors(std::vector<Spot*> neighbors){neighbors_ = neighbors;}

    };
}
#endif //SPOT_H