#ifndef D_STAR_LITE_NODE_H
#define D_STAR_LITE_NODE_H

#include <vector>
#include <cmath>

namespace LIFELONG_PLANNING_A_STAR_NAMESPACE {

using namespace std;

typedef  pair<int,int> LOCATION;

class LIFELONG_PLANNING_A_STAR_NODE{
public:
    enum NODE_SEARCH_TYPE {New,Open,Close}; // 定义枚举类型NODE_SEARCH_TYPE
public:
    float rhs;
    float g;
    float h;
    LOCATION Location;
    NODE_SEARCH_TYPE nodetype;
};

}

#endif
