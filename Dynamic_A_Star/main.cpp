#include "Dynamic_A_Star.h"
#include "Plot.cpp"

using namespace std;
using namespace DYNAMIC_A_STAR_NAMESPACE;


int main(int argc, char** argv)
{
    /********** Creat Map ****************/
    //算法的输入为vector<vector<int>>类型的数组，且必须是m*n的矩形形式
    //约定：0是可走的，1表示障碍物不可走，2表示起点，3表示终点
    vector<vector<int>> array={
    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }};

    array[0][0]=2;  //set the start point
    array[0][9]=3;  //set the endpoint


    vector<vector<int>> array2={
    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }};
    //array2[0][0]=2;  //set the start point
    array2[0][9]=3;  //set the endpoint

    DYNAMIC_A_STAR dynamic_a_star(array);
    vector<LOCATION> move_list;
    move_list.push_back(dynamic_a_star.X_c->Location);


    vector<LOCATION> path=dynamic_a_star.init_plan();
    if(path.empty()){ return 0;}
    while(dynamic_a_star.X_c != dynamic_a_star.X_g){
        dynamic_a_star.prepair_repair();
        path=dynamic_a_star.repair_replan();
        if(path.empty()){ return 0;}
        dynamic_a_star.set_cur_location(path[1]);
        move_list.push_back(dynamic_a_star.X_c->Location);
        dynamic_a_star.update_map(array2);
    }

    for (size_t ii=0;ii<move_list.size();ii++) {
        cout<<" ["<<move_list[ii].first<<" , "<<move_list[ii].second<<"] -> ";
    }

    /********** PLOT ****************/
    vector<vector<float>> plot_path;
    for (size_t ii=0;ii<move_list.size();ii++) {
        plot_path.push_back({float(move_list[ii].second),float(move_list[ii].first)});
        cout<<" ["<<move_list[ii].first<<" , "<<move_list[ii].second<<"] -> ";
    }


    PLOT::Set_data(plot_path,array);
    PLOT::Plot_data(argc,argv);


    //a_star.clear();
    return 0;
}

//enum NODETYPE {Reachable,UnReachable,StartPoint,EndPoint}; // 定义枚举类型NodeType
