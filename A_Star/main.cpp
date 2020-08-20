#include "A_Star.h"
#include "Plot.cpp"

using namespace std;

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
    { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }};

    array[0][0]=2;  //set the start point
    array[0][9]=3;  //set the endpoint

    A_Star a_star;
    int Max_Search_Time=100;
    int Errorflag=a_star.Get_Shortest_Path(array,Max_Search_Time,false);

    cout<<" After "<<a_star.Count<<" times search,";
    cout<<" The Shortest_Path_Long is "<<a_star.Shortest_Path_Long<<" ! "<<endl;
    cout<<" The Path : "<<endl;

    if(Errorflag==0)    //如果Errorflag==0，则说明搜到了最优路径，保存在 a_star.Shortest_Path 中
    {
        for(size_t i=0;i<a_star.Shortest_Path.size();i++){
            cout<<" ["<<a_star.Shortest_Path[i][0]<<" , "<<a_star.Shortest_Path[i][1]<<"] -> ";
        }
    }
    cout<<endl;

    /********** PLOT ****************/
    PLOT::Set_data(a_star.Shortest_Path,array);
    PLOT::Plot_data(argc,argv);


    //a_star.clear();
    return 0;
}

//enum NODETYPE {Reachable,UnReachable,StartPoint,EndPoint}; // 定义枚举类型NodeType
