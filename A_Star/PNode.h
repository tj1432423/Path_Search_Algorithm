#ifndef PNODE_H
#define PNODE_H


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

typedef class A_Star_Node: public Node {
public:
    NODETYPE nodetype;    //节点的类型
    vector<A_Star_Node*> adjacent_node;                   //用于记录真实世界相邻节点
    A_Star_Node* path_before;            //用于最终找到的路径

} PNode;

#endif
