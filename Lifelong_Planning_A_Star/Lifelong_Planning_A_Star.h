#ifndef DYNAMIC_A_STAR_H
#define DYNAMIC_A_STAR_H

#include <vector>
#include<iostream>
#include <map>
#include <stdlib.h>
#include "Lifelong_Planning_A_Star_Node.h"
#include "Min_Heap_Map_Opt.h"


namespace LIFELONG_PLANNING_A_STAR_NAMESPACE {

using namespace std;


class VALUE{
public:
    float primary;
    float secondary;
public:
    bool operator < (const VALUE& _a);
};



class LIFELONG_PLANNING_A_STAR{
public:
    enum NODE_TYPE {Space,Obstacle,Current,Goal}; // 定义枚举类型NODE_TYPE

public:
    LIFELONG_PLANNING_A_STAR(const vector<vector<int>>& _array);

    void initialize();

    void compute_shortest_path();

    void update_vertex(LIFELONG_PLANNING_A_STAR_NODE* const _u);

    //void set_cur_location(const LOCATION& _x);

    bool go_one_step();

    LIFELONG_PLANNING_A_STAR_NODE* calculate_next_step(LIFELONG_PLANNING_A_STAR_NODE* const _cur);

    void update_map(const vector<vector<int>>& _array);

    void load_shortest_path();

    ~LIFELONG_PLANNING_A_STAR();

private:
    vector<LIFELONG_PLANNING_A_STAR_NODE*> get_surround_node_list(const LIFELONG_PLANNING_A_STAR_NODE* const _x);

    bool location_check(const LOCATION& _x);

    float calculate_distance(const LIFELONG_PLANNING_A_STAR_NODE* const _a,const LIFELONG_PLANNING_A_STAR_NODE* const _b);

    float Cost(LIFELONG_PLANNING_A_STAR_NODE* const _x,LIFELONG_PLANNING_A_STAR_NODE* const  _y);

    VALUE calculate_key(const LIFELONG_PLANNING_A_STAR_NODE* const _u);

    float calculate_h(const LIFELONG_PLANNING_A_STAR_NODE* const _u);

public:
    vector<LOCATION> path_list;
    float obstacle_cost;
    //LIFELONG_PLANNING_A_STAR_NODE* X_c;
    LIFELONG_PLANNING_A_STAR_NODE* X_g;
    LIFELONG_PLANNING_A_STAR_NODE* X_s;


private:
    vector<vector<LIFELONG_PLANNING_A_STAR_NODE*>> node_table;
    map<pair<LIFELONG_PLANNING_A_STAR_NODE*,LIFELONG_PLANNING_A_STAR_NODE*>,float> cost_map;
    vector<vector<NODE_TYPE>> grid_map;
    Min_Heap_Map_Opt<VALUE,LIFELONG_PLANNING_A_STAR_NODE> open_list;

    bool goal_flag;
    bool start_flag;
    bool init_plan_flag;

    int row_num;
    int col_num;

private:
    const float big_cost=100000;
    const float epsion=float(0.0001);
};

}

#endif
