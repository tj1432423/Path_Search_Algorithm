#include"../A_Star/A_Star_Node.h"

//class ssNode{
//public:
//    NODETYPE nodetype;    //节点的类型
//    float F;   //F = G + H;
//    float G;   //G：从起点移动到指定方格的移动代价，沿着到达该方格而生成的路径
//    float H;   //H：从指定的方格移动到终点的估算成本
//    float x, y;   //节点的坐标
//    bool open_flag;  //在开放列表中为1，不在为0
//    bool close_flag;  //在关闭列表中为1，不在为0
//};


enum Drive_Direction {Foward,Backward}; // 定义枚举类型Drive_Direction

class Hybrid_A_Star_Node: public Node {
public:
    float phi;  //节点的航向角
    int index_x;
    int index_y;
    int index_phi;
    float steer_angle;
    Drive_Direction direction;
    Hybrid_A_Star_Node* path_before;            //用于最终找到的路径
};
