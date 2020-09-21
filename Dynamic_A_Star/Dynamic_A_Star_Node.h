#ifndef DYNAMIC_A_STAR_NODE_H
#define DYNAMIC_A_STAR_NODE_H

#include <vector>

namespace DYNAMIC_A_STAR_NAMESPACE {

using namespace std;

typedef  pair<int,int> LOCATION;

class VALUE{
public:
    VALUE(float _val=0):val(_val){
        big_cost=100000;
        epsion=float(0.001);
    }
    float val;
public:
    VALUE operator + (const VALUE& _a){
        VALUE res(min(val+_a.val,big_cost));
        return res;
    }

    bool operator == (const VALUE& _a){
        if(val>(big_cost-epsion) && _a.val>(big_cost-epsion)) return true;
        if( (val-_a.val)<epsion && (val-_a.val)>(-epsion) ) return true;
        return false;
    }

    bool operator != (const VALUE& _a){
        return !(*this==_a );
    }

    bool operator < (const VALUE& _a){
        if(val>(big_cost-epsion)) return false;
        return val<_a.val;
    }
    bool operator > (const VALUE& _a){
        if(_a.val>(big_cost-epsion)) return false;
        return val>_a.val;
    }

    bool operator >= (const VALUE& _a){
        return (*this>_a || *this==_a);
    }

    bool operator <= (const VALUE& _a){
        return (*this<_a || *this==_a);
    }

private:
    float big_cost;
    float epsion;
};




class DYNAMIC_A_STAR_NODE{
public:
    enum NODE_SEARCH_TYPE {New,Open,Close}; // 定义枚举类型NODE_SEARCH_TYPE
public:
    VALUE H;    //H：从指定的方格移动到终点的成本
    VALUE K;    //K：从指定的方格移动到终点的成本历史小值，OpenList排序依据
    LOCATION Location;
    DYNAMIC_A_STAR_NODE* next;
    NODE_SEARCH_TYPE nodetype;
};

}

#endif
