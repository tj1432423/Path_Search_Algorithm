#ifndef PNODE_MIN_HEAP_OPT_H
#define PNODE_MIN_HEAP_OPT_H

#include <unordered_map>
//#include <stdio.h>

#include "PNode.h"


class LNode_Min_Heap_Opt{
public:
    void Heap_push(const pair<float,PNode *>& new_node_pair);

    void Heap_pop();

    void Heap_delect(PNode *obj_node);

    size_t Heap_size();

    pair<float,PNode *> Heap_top();



private:
    void siftdown(size_t index);
    void siftup(size_t index);

    vector<pair<float,PNode *>> Min_Heap;
    unordered_map<PNode *,size_t> Mp;
};




#endif
