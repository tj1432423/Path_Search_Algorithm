#include "D_Star_Lite.h"
#include "Plot.cpp"

using namespace std;
using namespace D_STAR_LITE_NAMESPACE;


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

    D_STAR_LITE d_star_lite(array);

    bool map_update_flag=false;
    while(d_star_lite.X_c != d_star_lite.X_g){
        if(!map_update_flag){
            d_star_lite.update_map(array2);
            //d_star_lite.map_update_flag=true;
            map_update_flag=true;
        }

        if(!d_star_lite.go_one_step()){
            break;
        }
    }

    /********** PLOT ****************/
    vector<vector<float>> plot_path;
    for (size_t ii=0;ii<d_star_lite.path_list.size();ii++) {
        plot_path.push_back({float(d_star_lite.path_list[ii].second),float(d_star_lite.path_list[ii].first)});
        cout<<" ["<<d_star_lite.path_list[ii].first<<" , "<<d_star_lite.path_list[ii].second<<"] -> ";
    }

    PLOT::Set_data(plot_path,array);
    PLOT::Plot_data(argc,argv);

    return 0;
}
