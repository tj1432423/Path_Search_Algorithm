#include "A_Star.h"

using namespace std;

int main()
{
    /********** Creat Map ****************/
    //算法的输入为vector<vector<int>>类型的数组，且必须是m*n的矩形形式
    //约定：0是可走的，1表示障碍物不可走，2表示起点，3表示终点，4表示路径
    vector<vector<int>> array={
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 1, 3, 0, 0, 0 },
    { 0, 0, 2, 0, 0, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }};

    A_Star a_star;
    int Max_Search_Time=200;
    int Errorflag=a_star.Get_Shortest_Path(array,Max_Search_Time);

    if(Errorflag==0)    //如果Errorflag==0，则说明搜到了最优路径，保存在 a_star.Shortest_Path 中
    {
        for(size_t i=0;i<a_star.Shortest_Path.size();i++){
            cout<<" ["<<a_star.Shortest_Path[i][0]<<" , "<<a_star.Shortest_Path[i][1]<<"] -> ";
        }
    }
    return 0;
}

//enum NODETYPE {Reachable,UnReachable,StartPoint,EndPoint}; // 定义枚举类型NodeType
