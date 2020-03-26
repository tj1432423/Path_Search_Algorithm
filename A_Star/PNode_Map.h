#ifndef PNODE_MAP_H
#define PNODE_MAP_H

#include <vector>
#include <iostream>
#include "PNode.h"

class PNode_Map{
public:
    void Build_Map(const vector<vector<int>>& array);
    void Clear_Map();
    PNode* Start_node;
    PNode* End_node;

private:
    vector<vector<PNode*>> pnode;
};


#endif
