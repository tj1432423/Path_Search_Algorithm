#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H



//#include <vector>
//#include <unordered_map>

//#include "PNode.h"

//#include "PNode_Map.h"
//#include <pair>
#include <map>
#include "../A_Star/Min_Heap_Map_Opt.h"
#include "Hybrid_A_Star_Node.h"

class Vehicle_Parameters{
public:
    float max_front_wheel_angle;
    float wheel_base;
    float back_to_rear;
    float front_to_rear;
    float width;
    float min_turn_radius;
};

class Search_Parameters{
public:
    float heading_resolution;
    float front_wheel_angle_resolution;
    float move_resolution;
    float collision_check_resolution;
    bool debug_info_switch;
};


class Hybrid_A_Star{
public:
    void Set_Vehicle_Parameters(float _max_front_wheel_angle,float _wheel_base,float _back_to_rear,float _front_to_rear,float _width);

    void Set_Search_Parameters(float _heading_resolution,float _front_wheel_angle_resolution,float _move_resolution,float _collision_check_resolution,bool _debug_info_switch);

    bool Load_Map(const vector<vector<int>>& _array,const vector<float>& _start_point,const vector<float>& _end_point,int _Max_Search_Time);

    bool Get_The_Shortest_Path(int Max_Search_Time);

    vector<vector<float>> rs_path;
    vector<vector<float>> a_star_path;

    vector<vector<float>> a_star_distance_table;


private:
    vector<int> Point_Index_Calculate(const vector<float>& _point); // change [x,y,phi] into [index_x,index_y,index_phi],front_wheel_angle is not include.

    void Set_The_Hybrid_A_Star_Node(Hybrid_A_Star_Node* _targer_hybrid_a_star_node,vector<float> _point,Drive_Direction _direction,Hybrid_A_Star_Node* _node_before,float _G);

    bool search(Hybrid_A_Star_Node* _current_node);

    vector<float> Vehicle_Kinematic(const vector<float>& _current_pos,float _front_wheel_angle,float _move_distance,Drive_Direction _direction);

    bool Collision_Check(const vector<float>& _pos);

    float Move_Cost_Calculate(Hybrid_A_Star_Node* _current_node,float _front_wheel_angle,Drive_Direction _direction);

    Vehicle_Parameters vehicle_parameters;
    Search_Parameters search_parameters;


    vector<float> start_point;  // x,y,phi,front_wheel_angle
    vector<float> end_point;  // x,y,phi,front_wheel_angle

    vector<vector<int>> obstacles_map;

    Min_Heap_Map_Opt<float,Hybrid_A_Star_Node> open_list;  //min heap of Hybrid_A_Star_Node
    map<pair<vector<int>,Drive_Direction>,Hybrid_A_Star_Node*> mp;   // input the vector {index_x,index_y,index_phi}, can get the address of Hybrid_A_Star_Node
};



#endif // HYBRID_A_STAR_H
