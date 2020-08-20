#ifndef A_STAR_NODE_H
#define A_STAR_NODE_H


#include "Node.h"

using namespace std;


class A_Star_Node: public Node {
public:
    NODETYPE nodetype;    //节点的类型
    vector<A_Star_Node*> adjacent_node;                   //用于记录真实世界相邻节点
    A_Star_Node* path_before;            //用于最终找到的路径

};

#endif
