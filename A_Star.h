#ifndef A_STAR_H
#define A_STAR_H



#include <vector>
#include <unordered_map>

#include "Math_KDK.h"
#include "PNode.h"
#include "PNode_Min_Heap_Opt.h"
#include "PNode_Map.h"



class A_Star{
public:
    int Get_Shortest_Path(const vector<vector<int>>& array,int Max_Search_Time);
    vector<vector<float>> Shortest_Path;

private:
    PNode_Map map;
    LNode_Min_Heap_Opt open_List;
    float H_Calculat(PNode *cur,PNode *end);
    int Search();
};
#endif // A_STAR_H
