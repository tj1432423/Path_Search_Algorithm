#ifndef NODE_H
#define NODE_H


#include <vector>

using namespace std;

//约定：0是可走的，1表示障碍物不可走，2表示起点，3表示终点，4表示路径
enum NODETYPE {Reachable,UnReachable,StartPoint,EndPoint}; // 定义枚举类型NodeType


class Node{
public:
    float F;   //F = G + H;
    float G;   //G：从起点移动到指定方格的移动代价，沿着到达该方格而生成的路径
    float H;   //H：从指定的方格移动到终点的估算成本
    float x, y;   //节点的坐标
    bool open_flag;  //在开放列表中为1，不在为0
    bool close_flag;  //在关闭列表中为1，不在为0
};


#endif
