#include "Lifelong_Planning_A_Star.h"
#include "Plot.cpp"

using namespace std;
using namespace LIFELONG_PLANNING_A_STAR_NAMESPACE;


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
    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 }};

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
    array2[0][0]=2;  //set the start point
    array2[0][9]=3;  //set the endpoint

    LIFELONG_PLANNING_A_STAR LIFELONG_PLANNING_A_STAR(array);
    LIFELONG_PLANNING_A_STAR.initialize();
    LIFELONG_PLANNING_A_STAR.compute_shortest_path();
    LIFELONG_PLANNING_A_STAR.update_map(array2);
    LIFELONG_PLANNING_A_STAR.compute_shortest_path();


    /********** PLOT ****************/
    vector<vector<float>> plot_path;
    for (size_t ii=0;ii<LIFELONG_PLANNING_A_STAR.path_list.size();ii++) {
        plot_path.push_back({float(LIFELONG_PLANNING_A_STAR.path_list[ii].second),float(LIFELONG_PLANNING_A_STAR.path_list[ii].first)});
        cout<<" ["<<LIFELONG_PLANNING_A_STAR.path_list[ii].first<<" , "<<LIFELONG_PLANNING_A_STAR.path_list[ii].second<<"] -> ";
    }

    PLOT::Set_data(plot_path,array2);
    PLOT::Plot_data(argc,argv);

    return 0;
}

