#include "D_Star_Lite.h"
#include <cmath>

using namespace D_STAR_LITE_NAMESPACE;


bool VALUE::operator < (const VALUE& _a){
    if(primary < _a.primary) return true;
    else if (primary > _a.primary) return false;
    return secondary < _a.secondary;
}

D_STAR_LITE::D_STAR_LITE(const vector<vector<int>>& _array){
    if(_array.empty()){
        cout<<"Loat Map fail !!!"<<endl;
        exit(0);
    }
    row_num=int(_array.size());
    col_num=int(_array[0].size());

    vector<D_STAR_LITE_NODE*> tmp(col_num);
    vector<vector<D_STAR_LITE_NODE*>> ttmp(row_num,tmp);
    node_table=ttmp;

    obstacle_cost=big_cost;

    for(size_t ii=0;ii<_array.size();ii++){
        grid_map.resize(grid_map.size()+1);
        for(size_t jj=0;jj<_array[0].size();jj++){
            grid_map[ii].push_back(NODE_TYPE(_array[ii][jj]));  // load and save the map !!! force type change !!!
            node_table[ii][jj]= new D_STAR_LITE_NODE();
            node_table[ii][jj]->g=obstacle_cost;
            if(grid_map[ii][jj]==Goal) node_table[ii][jj]->rhs=0;
            else node_table[ii][jj]->rhs=obstacle_cost;
            node_table[ii][jj]->Location=make_pair(ii,jj);
            node_table[ii][jj]->nodetype=D_STAR_LITE_NODE::New;
        }
    }

    for(size_t ii=0;ii<_array.size();ii++){
        for(size_t jj=0;jj<_array[0].size();jj++){
            vector<D_STAR_LITE_NODE*> surround_node_list=get_surround_node_list(node_table[ii][jj]);
            for(size_t kk=0;kk<surround_node_list.size();kk++){
                if(grid_map[ii][jj]==Obstacle || grid_map[ surround_node_list[kk]->Location.first ] [ surround_node_list[kk]->Location.second ]==Obstacle){
                    cost_map[ make_pair(node_table[ii][jj],surround_node_list[kk] ) ]  =  obstacle_cost;
                }
                else {
                    cost_map[ make_pair(node_table[ii][jj],surround_node_list[kk] ) ]  =  calculate_distance(node_table[ii][jj],surround_node_list[kk]);
                }
            }
            if(grid_map[ii][jj]==Goal){
                X_g=node_table[ii][jj];
                goal_flag=true;
            }
            else if(grid_map[ii][jj]==Current){
                //X_s=node_table[ii][jj];

                X_c=node_table[ii][jj];
                cur_flag=true;
                path_list.push_back(X_c->Location);
            }

        }
    }

    if(!goal_flag){
        cout<<"Missing the goal !!!"<<endl;
        exit(0);
    }
}

void D_STAR_LITE::initialize(){
    if(!cur_flag){
        cout<<"Missing the current position !!!"<<endl;
        exit(0);
    }
    open_list.Heap_push(make_pair(calculate_key(X_g),X_g));
    X_g->nodetype=D_STAR_LITE_NODE::Open;
    init_plan_flag = true;
}

void D_STAR_LITE::compute_shortest_path(){
    if(!init_plan_flag){
        cout<<"Fetal wrong !!! Please run the function initialize() first !!!"<<endl;
        exit(0);
    }
    while(open_list.Heap_size() && (open_list.Heap_top().first < calculate_key(X_c) || abs(X_c->rhs - X_c->g) > epsion)){
        pair<VALUE,D_STAR_LITE_NODE*> tmp=open_list.Heap_pop();
        D_STAR_LITE_NODE* u=tmp.second;
        u->nodetype=D_STAR_LITE_NODE::Close;
        if(u->g > u->rhs){
            u->g = u->rhs;
        }
        else {
            u->g = big_cost;
            update_vertex(u);
        }
        vector<D_STAR_LITE_NODE*> tmp_surround_node_list=get_surround_node_list(u);
        for(size_t kk=0;kk<tmp_surround_node_list.size();kk++){
            update_vertex(tmp_surround_node_list[kk]);
        }
    }
}

void D_STAR_LITE::update_vertex(D_STAR_LITE_NODE* const _u){
    float min_rhs=big_cost;
    if(_u != X_g){
        vector<D_STAR_LITE_NODE*> tmp_surround_node_list=get_surround_node_list(_u);
        for(size_t kk=0;kk<tmp_surround_node_list.size();kk++){
            float tmp_rhs=Cost(_u,tmp_surround_node_list[kk])+tmp_surround_node_list[kk]->g;
            min_rhs=tmp_rhs < min_rhs ? tmp_rhs : min_rhs ;
        }
        _u->rhs=min_rhs;
    }
    if(_u->nodetype == D_STAR_LITE_NODE::Open){
        open_list.Heap_delect(_u);
        _u->nodetype=D_STAR_LITE_NODE::Close;
    }
    if(abs(_u->g - _u->rhs) > epsion){
        if(_u->nodetype != D_STAR_LITE_NODE::Open){
            open_list.Heap_push(make_pair(calculate_key(_u),_u));
            _u->nodetype=D_STAR_LITE_NODE::Open;
        }
        else {
            open_list.Heap_modify(_u,calculate_key(_u));
            //_u->nodetype=D_STAR_LITE_NODE::Open;
        }
    }
}




D_STAR_LITE::~D_STAR_LITE(){
    if(node_table.empty()) return;
    for (size_t ii=0;ii<node_table.size();ii++) {
        for (size_t jj=0;jj<node_table.size();jj++) {
            delete node_table[ii][jj];
            node_table[ii][jj]=nullptr;
        }
    }
}

void D_STAR_LITE::set_cur_location(const LOCATION& _x){
    if(!location_check(_x)){
        cout<<"Wrong cur location !!! "<<endl;
        exit(0);
    }
    X_c=node_table[_x.first][_x.second];
    cur_flag=true;
}

bool D_STAR_LITE::go_one_step(){
    D_STAR_LITE_NODE* next_node=calculate_next_step();
    if(next_node->g >= obstacle_cost-1.0){
        cout<<"There is no know path !!!"<<endl;
        return false;
    }
    if(grid_map[next_node->Location.first][next_node->Location.second] == Obstacle){
        //X_s=X_c;
        vector<D_STAR_LITE_NODE*> tmp_surround_node_list=get_surround_node_list(next_node);
        for(size_t ii=0;ii<tmp_surround_node_list.size();ii++){
            cost_map[make_pair(next_node,tmp_surround_node_list[ii])]=obstacle_cost;
            cost_map[make_pair(tmp_surround_node_list[ii],next_node)]=obstacle_cost;
        }
        for(size_t ii=0;ii<tmp_surround_node_list.size();ii++){
            update_vertex(tmp_surround_node_list[ii]);
        }
        update_vertex(next_node);

        Min_Heap_Map_Opt<VALUE,D_STAR_LITE_NODE> tmp_open_list=open_list;
        open_list.Heap_clear();
        while(tmp_open_list.Heap_size()){
            D_STAR_LITE_NODE* tmp=tmp_open_list.Heap_pop().second;
            open_list.Heap_push(make_pair(calculate_key(tmp),tmp));
        }
        tmp_open_list.Heap_clear();
        compute_shortest_path();
        return go_one_step();
    }
    X_c=next_node;
    cout<<"Robot has arrive [ "<<X_c->Location.first<<" , "<<X_c->Location.second<<" ]."<<endl;
    path_list.push_back(X_c->Location);
    return true;
}

D_STAR_LITE_NODE* D_STAR_LITE::calculate_next_step(){
    vector<D_STAR_LITE_NODE*> tmp_surrond_node=get_surround_node_list(X_c);
    float min_value=Cost(X_c,tmp_surrond_node[0])+tmp_surrond_node[0]->g;
    D_STAR_LITE_NODE* tmp_start=tmp_surrond_node[0];
    for(size_t ii=1;ii<tmp_surrond_node.size();ii++){
        float tmp_min_val=Cost(X_c,tmp_surrond_node[ii])+tmp_surrond_node[ii]->g;
        if(tmp_min_val < min_value){
            min_value = tmp_min_val;
            tmp_start = tmp_surrond_node[ii];
        }
    }
    return tmp_start;
}

void D_STAR_LITE::update_map(const vector<vector<int>>& _array){
    for(size_t ii=0;ii<_array.size();ii++){
        for(size_t jj=0;jj<_array[0].size();jj++){
            grid_map[ii][jj]=NODE_TYPE(_array[ii][jj]);  // load and save the map !!! force typr change !!!
        }
    }
}


bool D_STAR_LITE::location_check(const LOCATION& _x){
    if(_x.first>=0 && _x.first<row_num && _x.second>=0 && _x.second<col_num) return true;
    else return false;
}

float D_STAR_LITE::calculate_distance(const D_STAR_LITE_NODE* const _a,const D_STAR_LITE_NODE* const _b){
    double dis;
    dis=pow(_a->Location.first-_b->Location.first,2)+pow(_a->Location.second-_b->Location.second,2);
    dis=pow(dis,0.5);
    return float(dis);
}


float D_STAR_LITE::Cost(D_STAR_LITE_NODE* const _x,D_STAR_LITE_NODE* const  _y){
    return cost_map[make_pair(_x,_y)];
}

VALUE D_STAR_LITE::calculate_key(const D_STAR_LITE_NODE* const _u){
    VALUE k;
    k.primary=min(_u->g,_u->rhs)+calculate_h(_u);
    k.secondary=min(_u->g,_u->rhs);
    return k;
}

float D_STAR_LITE::calculate_h(const D_STAR_LITE_NODE* const _u){
    double dis_pow_2=pow((double(_u->Location.first-X_c->Location.first)),2)+pow((double(_u->Location.second-X_c->Location.second)),2);
    return float(0.99*pow(dis_pow_2,0.5));
    //return 0;
}

vector<D_STAR_LITE_NODE*> D_STAR_LITE::get_surround_node_list(const D_STAR_LITE_NODE* const _x){
    vector<D_STAR_LITE_NODE*> list;
    for (int ii=_x->Location.first-1;ii<=_x->Location.first+1;ii++) {
        for (int jj=_x->Location.second-1;jj<=_x->Location.second+1;jj++) {
            if(ii==_x->Location.first && jj==_x->Location.second) continue;
            if(location_check(make_pair(ii,jj))){
                list.push_back(node_table[ii][jj]);
            }
        }
    }
    return list;
}
