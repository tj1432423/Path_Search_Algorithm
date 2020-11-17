#ifndef DYNAMIC_A_STAR_H
#define DYNAMIC_A_STAR_H

#include <vector>
#include<iostream>
#include <map>
#include <stdlib.h>
#include "D_Star_Lite_Node.h"
#include "Min_Heap_Map_Opt.h"

namespace D_STAR_LITE_NAMESPACE {

using namespace std;


class VALUE{
public:
    float primary;
    float secondary;
public:
    bool operator < (const VALUE& _a);
};



class D_STAR_LITE{
public:
    enum NODE_TYPE {Space,Obstacle,Current,Goal}; // 定义枚举类型NODE_TYPE

public:
    D_STAR_LITE(const vector<vector<int>>& _array);

    void initialize();

    void compute_shortest_path();

    void update_vertex(D_STAR_LITE_NODE* const _u);

    void set_cur_location(const LOCATION& _x);

    bool go_one_step();

    D_STAR_LITE_NODE* calculate_next_step();

    void update_map(const vector<vector<int>>& _array);

    ~D_STAR_LITE();

private:
    vector<D_STAR_LITE_NODE*> get_surround_node_list(const D_STAR_LITE_NODE* const _x);

    bool location_check(const LOCATION& _x);

    float calculate_distance(const D_STAR_LITE_NODE* const _a,const D_STAR_LITE_NODE* const _b);

    float Cost(D_STAR_LITE_NODE* const _x,D_STAR_LITE_NODE* const  _y);

    VALUE calculate_key(const D_STAR_LITE_NODE* const _u);

    float calculate_h(const D_STAR_LITE_NODE* const _u);

    float calculate_dis(const D_STAR_LITE_NODE* const _x,const D_STAR_LITE_NODE* const _y);

public:
    vector<LOCATION> path_list;
    float obstacle_cost;
    D_STAR_LITE_NODE* X_c; //current position
    D_STAR_LITE_NODE* X_g;
    //D_STAR_LITE_NODE* X_s;

    D_STAR_LITE_NODE* X_l; // last position

    float Km;

    //bool map_update_flag;

private:
    vector<vector<D_STAR_LITE_NODE*>> node_table;
    map<pair<D_STAR_LITE_NODE*,D_STAR_LITE_NODE*>,float> cost_map;
    vector<vector<NODE_TYPE>> grid_map;
    Min_Heap_Map_Opt<VALUE,D_STAR_LITE_NODE> open_list;

    bool goal_flag;
    bool cur_flag;
    bool init_plan_flag;

    int row_num;
    int col_num;

private:
    const float big_cost=100000;
    const float epsion=float(0.0001);
};

}

#endif
