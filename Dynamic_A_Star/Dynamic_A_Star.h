#ifndef DYNAMIC_A_STAR_H
#define DYNAMIC_A_STAR_H

#include <vector>
#include<iostream>
#include <map>
#include <stdlib.h>
#include "Dynamic_A_Star_Node.h"
#include "Min_Heap_Map_Opt.h"

namespace DYNAMIC_A_STAR_NAMESPACE {

using namespace std;

class DYNAMIC_A_STAR{
public:
    enum NODE_TYPE {Space,Obstacle,Current,Goal}; // 定义枚举类型NODE_TYPE

public:
    DYNAMIC_A_STAR(const vector<vector<int>>& _array);

    void set_cur_location(const LOCATION& _x);

    vector<LOCATION> init_plan();

    vector<LOCATION> repair_replan();

    void prepair_repair();

    void update_map(const vector<vector<int>>& _array);

    ~DYNAMIC_A_STAR();

private:

    void modify_cost(DYNAMIC_A_STAR_NODE* const _x,DYNAMIC_A_STAR_NODE* const _y,VALUE _val);

    VALUE process_state();

    void insert(DYNAMIC_A_STAR_NODE* const _x,VALUE _h_new);

    bool location_check(const LOCATION& _x);

    vector<DYNAMIC_A_STAR_NODE*> get_surround_node_list(const DYNAMIC_A_STAR_NODE* const _x);

    VALUE calculate_distance(const DYNAMIC_A_STAR_NODE* const _a,const DYNAMIC_A_STAR_NODE* const _b);

    VALUE Cost(DYNAMIC_A_STAR_NODE* const _x,DYNAMIC_A_STAR_NODE* const  _y);

    VALUE Detect(const DYNAMIC_A_STAR_NODE* const _x,const DYNAMIC_A_STAR_NODE* const _y);

    vector<LOCATION> get_path();

    VALUE min_value(VALUE& _a,VALUE& _b);

public:
     vector<vector<LOCATION>> path_set;
     DYNAMIC_A_STAR_NODE* X_c;
     DYNAMIC_A_STAR_NODE* X_g;

private:
    vector<vector<DYNAMIC_A_STAR_NODE*>> node_table;
    map<pair<DYNAMIC_A_STAR_NODE*,DYNAMIC_A_STAR_NODE*>,VALUE> cost_map;
    vector<vector<NODE_TYPE>> grid_map;
    Min_Heap_Map_Opt<VALUE,DYNAMIC_A_STAR_NODE> open_list;

    VALUE obstacle_cost;

    bool goal_flag;
    bool cur_flag;
    bool init_plan_flag;

    int row_num;
    int col_num;

private:
    const float big_cost=100000;
};

}

#endif // DYNAMIC_A_STAR_H
