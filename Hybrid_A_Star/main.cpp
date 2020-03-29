#include "Hybrid_A_Star.h"
#include <iostream>

#define pi 3.1415926

using namespace std;

int main()
{
    Hybrid_A_Star hybrid_a_star;
    float _max_front_wheel_angle=40.0/180.0*pi;
    float _wheel_base=2.65;
    float _back_to_rear=1.1;
    float _front_to_rear=3.1;
    float _width=2.8;
    hybrid_a_star.Set_Vehicle_Parameters(_max_front_wheel_angle,_wheel_base,_back_to_rear,_front_to_rear,_width);

    float _heading_resolution=5.0/180.0*pi;
    float _front_wheel_angle_resolution=5.0/180.0*pi;
    float _move_resolution=1.6;
    float _collision_check_resolution=1.5;
    bool _debug_info_switch=true;
    hybrid_a_star.Set_Search_Parameters(_heading_resolution,_front_wheel_angle_resolution,_move_resolution,_collision_check_resolution,_debug_info_switch);

    /********** Creat Map ****************/
    //算法的输入为vector<vector<int>>类型的数组，且必须是m*n的矩形形式
    //约定：0是可走的，1表示障碍物不可走，2表示起点，3表示终点
//    vector<vector<int>> _array={
//    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//    { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
//    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
//    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
//    { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
//    { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
//    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
//    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
//    { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
//    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }};
    vector<int> tmp(30,0);
    vector<vector<int>> _array(30,tmp);
    for(size_t i=0;i<_array.size();i++){
        for(size_t j=0;j<_array[0].size();j++){
            if(i>10 && i<=20 && j<=20) _array[i][j]=1;
        }
    }
    for(size_t i=0;i<_array.size();i++){
        for(size_t j=0;j<_array[0].size();j++){
            cout<<_array[i][j]<<" ";
        }
        cout<<endl;
    }
    cout<<"------------------------"<<endl;
    vector<float> _start_point(4,0.0);
    _start_point[0]=1;
    _start_point[1]=2;
    _start_point[2]=0;
    vector<float> _end_point(4,0.0);
    _end_point[0]=1;
    _end_point[1]=26;
    //_end_point[2]=float(0);
    _end_point[2]=float(pi);
    int _Max_Search_Time=500;
    hybrid_a_star.Load_Map(_array,_start_point,_end_point,_Max_Search_Time);
    cout<<"---load map sucessfull !!!---"<<endl;
//    for(size_t i=0;i<hybrid_a_star.a_star_distance_table.size();i++){
//        for(size_t j=0;j<hybrid_a_star.a_star_distance_table[i].size();j++){
//            cout<<hybrid_a_star.a_star_distance_table[i][j]<<" ";
//        }
//        cout<<endl;
//    }

    hybrid_a_star.Get_The_Shortest_Path(_Max_Search_Time);
    //cout<<"------------------------"<<endl;
    for(size_t i=0;i<hybrid_a_star.a_star_path.size();i++){
        cout<<" -> [ "<<hybrid_a_star.a_star_path[i][0]<<" , "<<hybrid_a_star.a_star_path[i][1]<<" , "<<hybrid_a_star.a_star_path[i][2]<<" ]"<<endl;
        //cout<<" -> [ "<<hybrid_a_star.a_star_path[0];
    }
    cout<<"------------"<<endl;
    for(size_t i=0;i<hybrid_a_star.rs_path.size();i++){
        cout<<" -> [ "<<hybrid_a_star.rs_path[i][0]<<" , "<<hybrid_a_star.rs_path[i][1]<<" , "<<hybrid_a_star.rs_path[i][2]<<" ]"<<endl;
        //cout<<" -> [ "<<hybrid_a_star.a_star_path[0];
    }
    cout<<"------------"<<endl;
    cout<<" -> [ "<<_end_point[0]<<" , "<<_end_point[1]<<" , "<<_end_point[2]<<" ]"<<endl;
    //cout<<" The search is over !!!!!!!!!!!!";

    return 0;
}
