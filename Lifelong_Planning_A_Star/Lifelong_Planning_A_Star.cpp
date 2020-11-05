#include "Lifelong_Planning_A_Star.h"
#include <cmath>

using namespace LIFELONG_PLANNING_A_STAR_NAMESPACE;


bool VALUE::operator < (const VALUE& _a){
    if(primary < _a.primary) return true;
    else if (primary > _a.primary) return false;
    return secondary < _a.secondary;
}

LIFELONG_PLANNING_A_STAR::LIFELONG_PLANNING_A_STAR(const vector<vector<int>>& _array){
    if(_array.empty()){
        cout<<"Loat Map fail !!!"<<endl;
        exit(0);
    }
    row_num=int(_array.size());
    col_num=int(_array[0].size());

    vector<LIFELONG_PLANNING_A_STAR_NODE*> tmp(col_num);
    vector<vector<LIFELONG_PLANNING_A_STAR_NODE*>> ttmp(row_num,tmp);
    node_table=ttmp;

    obstacle_cost=big_cost;

    for(size_t ii=0;ii<_array.size();ii++){
        grid_map.resize(grid_map.size()+1);
        for(size_t jj=0;jj<_array[0].size();jj++){
            grid_map[ii].push_back(NODE_TYPE(_array[ii][jj]));  // load and save the map !!! force type change !!!
            node_table[ii][jj]= new LIFELONG_PLANNING_A_STAR_NODE();
            node_table[ii][jj]->g=obstacle_cost;
            if(grid_map[ii][jj]==Current){
                node_table[ii][jj]->rhs=0;
            }
            else{
                node_table[ii][jj]->rhs=obstacle_cost;
            }
            node_table[ii][jj]->Location=make_pair(ii,jj);
            node_table[ii][jj]->nodetype=LIFELONG_PLANNING_A_STAR_NODE::New;
        }
    }

    for(size_t ii=0;ii<_array.size();ii++){
        for(size_t jj=0;jj<_array[0].size();jj++){
            vector<LIFELONG_PLANNING_A_STAR_NODE*> surround_node_list=get_surround_node_list(node_table[ii][jj]);
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
                X_s=node_table[ii][jj];
                start_flag=true;

                //X_c=node_table[ii][jj];
                //path_list.push_back(X_c->Location);
            }
        }
    }
    if(!goal_flag){
        cout<<"Missing the goal !!!"<<endl;
        exit(0);
    }
    for(size_t ii=0;ii<_array.size();ii++){
        for(size_t jj=0;jj<_array[0].size();jj++){
            node_table[ii][jj]->h=calculate_h(node_table[ii][jj]);
        }
    }
}

void LIFELONG_PLANNING_A_STAR::initialize(){
    if(!start_flag){
        cout<<"Missing the start position !!!"<<endl;
        exit(0);
    }
    open_list.Heap_push(make_pair(calculate_key(X_s),X_s));
    X_s->nodetype=LIFELONG_PLANNING_A_STAR_NODE::Open;
    init_plan_flag = true;
}

void LIFELONG_PLANNING_A_STAR::compute_shortest_path(){
    if(!init_plan_flag){
        cout<<"Fetal wrong !!! Please run the function initialize() first !!!"<<endl;
        exit(0);
    }
    while(open_list.Heap_size() && (open_list.Heap_top().first < calculate_key(X_g) || abs(X_g->rhs - X_g->g) > epsion)){
        pair<VALUE,LIFELONG_PLANNING_A_STAR_NODE*> tmp=open_list.Heap_pop();
        LIFELONG_PLANNING_A_STAR_NODE* u=tmp.second;
        u->nodetype=LIFELONG_PLANNING_A_STAR_NODE::Close;

        if(u->g > u->rhs){
            u->g = u->rhs;
        }
        else {
            u->g = big_cost;
            update_vertex(u);
        }
        vector<LIFELONG_PLANNING_A_STAR_NODE*> tmp_surround_node_list=get_surround_node_list(u);
        for(size_t kk=0;kk<tmp_surround_node_list.size();kk++){
            update_vertex(tmp_surround_node_list[kk]);
        }
    }
    //cout<<"X_g.g = "<<X_g->g<<endl;
    load_shortest_path();

}

void LIFELONG_PLANNING_A_STAR::load_shortest_path(){
    path_list.clear();
    if(X_g->g > (obstacle_cost-1)){    // If the g_val of goal is inf, there is no useable path to the goal.  --KDK
        cout<<"No useable path to the goal !!!"<<endl;
        return;
    }
    path_list.push_back(X_g->Location);

    while(path_list.back() != X_s->Location){
        path_list.push_back(calculate_next_step(  node_table[path_list.back().first][path_list.back().second]  )->Location);
        cout<<"[ "<<path_list.back().first<<" , "<<path_list.back().second<<" ] ->  ";
    }
    cout<<"the path is end !!!"<<endl;
}


void LIFELONG_PLANNING_A_STAR::update_vertex(LIFELONG_PLANNING_A_STAR_NODE* const _u){
    float min_rhs=big_cost;
    if(_u != X_s){
        vector<LIFELONG_PLANNING_A_STAR_NODE*> tmp_surround_node_list=get_surround_node_list(_u);
        for(size_t kk=0;kk<tmp_surround_node_list.size();kk++){
            float tmp_rhs=Cost(tmp_surround_node_list[kk],_u)+tmp_surround_node_list[kk]->g;
            min_rhs=tmp_rhs < min_rhs ? tmp_rhs : min_rhs ;
        }
        _u->rhs=min_rhs;
    }
    if(_u->nodetype == LIFELONG_PLANNING_A_STAR_NODE::Open){
        open_list.Heap_delect(_u);
        _u->nodetype=LIFELONG_PLANNING_A_STAR_NODE::Close;
    }
    if(abs(_u->g - _u->rhs) > epsion){
        if(_u->nodetype != LIFELONG_PLANNING_A_STAR_NODE::Open){
            open_list.Heap_push(make_pair(calculate_key(_u),_u));
            _u->nodetype=LIFELONG_PLANNING_A_STAR_NODE::Open;
        }
        else {
            open_list.Heap_modify(_u,calculate_key(_u));
            //_u->nodetype=LIFELONG_PLANNING_A_STAR_NODE::Open;
        }
    }
}


LIFELONG_PLANNING_A_STAR::~LIFELONG_PLANNING_A_STAR(){
    if(node_table.empty()) return;
    for (size_t ii=0;ii<node_table.size();ii++) {
        for (size_t jj=0;jj<node_table.size();jj++) {
            delete node_table[ii][jj];
            node_table[ii][jj]=nullptr;
        }
    }
}

LIFELONG_PLANNING_A_STAR_NODE* LIFELONG_PLANNING_A_STAR::calculate_next_step(LIFELONG_PLANNING_A_STAR_NODE* const _cur){
    vector<LIFELONG_PLANNING_A_STAR_NODE*> tmp_surrond_node=get_surround_node_list(_cur);
    float min_value=Cost(_cur,tmp_surrond_node[0])+tmp_surrond_node[0]->g;
    LIFELONG_PLANNING_A_STAR_NODE* tmp_start=tmp_surrond_node[0];
    for(size_t ii=1;ii<tmp_surrond_node.size();ii++){
        float tmp_min_val=Cost(_cur,tmp_surrond_node[ii])+tmp_surrond_node[ii]->g;
        if(tmp_min_val < min_value){
            min_value = tmp_min_val;
            tmp_start = tmp_surrond_node[ii];
        }
    }
    return tmp_start;
}


void LIFELONG_PLANNING_A_STAR::update_map(const vector<vector<int>>& _array){
    for(size_t ii=0;ii<_array.size();ii++){
        for(size_t jj=0;jj<_array[0].size();jj++){
            NODE_TYPE tmp_node_type=NODE_TYPE(_array[ii][jj]);
            if(grid_map[ii][jj] != tmp_node_type){
                if(grid_map[ii][jj]==Obstacle){
                    grid_map[ii][jj]=tmp_node_type;  // load and save the map !!! force typr change !!!
                    vector<LIFELONG_PLANNING_A_STAR_NODE*> tmp_surround_node_list=get_surround_node_list(node_table[ii][jj]);
                    for(size_t kk=0;kk<tmp_surround_node_list.size();kk++){
                        if(grid_map[tmp_surround_node_list[kk]->Location.first][tmp_surround_node_list[kk]->Location.second]==Obstacle){
                            cost_map[make_pair(node_table[ii][jj],tmp_surround_node_list[kk])]=obstacle_cost;
                            cost_map[make_pair(tmp_surround_node_list[kk],node_table[ii][jj])]=obstacle_cost;
                        }
                        else {
                            cost_map[make_pair(node_table[ii][jj],tmp_surround_node_list[kk])]=calculate_distance(node_table[ii][jj],tmp_surround_node_list[kk]);
                            cost_map[make_pair(tmp_surround_node_list[kk],node_table[ii][jj])]=calculate_distance(tmp_surround_node_list[kk],node_table[ii][jj]);
                        }
                    }
                }
                else if (tmp_node_type==Obstacle) {
                    grid_map[ii][jj]=tmp_node_type;  // load and save the map !!! force typr change !!!
                    vector<LIFELONG_PLANNING_A_STAR_NODE*> tmp_surround_node_list=get_surround_node_list(node_table[ii][jj]);
                    for(size_t kk=0;kk<tmp_surround_node_list.size();kk++){
                        cost_map[make_pair(node_table[ii][jj],tmp_surround_node_list[kk])]=obstacle_cost;
                        cost_map[make_pair(tmp_surround_node_list[kk],node_table[ii][jj])]=obstacle_cost;
                    }
                    for(size_t kk=0;kk<tmp_surround_node_list.size();kk++){
                        update_vertex(tmp_surround_node_list[kk]);
                    }
                    update_vertex(node_table[ii][jj]);
                }
            }
        }
    }
}


bool LIFELONG_PLANNING_A_STAR::location_check(const LOCATION& _x){
    if(_x.first>=0 && _x.first<row_num && _x.second>=0 && _x.second<col_num) return true;
    else return false;
}

float LIFELONG_PLANNING_A_STAR::calculate_distance(const LIFELONG_PLANNING_A_STAR_NODE* const _a,const LIFELONG_PLANNING_A_STAR_NODE* const _b){
    double dis;
    dis=pow(_a->Location.first-_b->Location.first,2)+pow(_a->Location.second-_b->Location.second,2);
    dis=pow(dis,0.5);
    return float(dis);
}


float LIFELONG_PLANNING_A_STAR::Cost(LIFELONG_PLANNING_A_STAR_NODE* const _x,LIFELONG_PLANNING_A_STAR_NODE* const  _y){
    return cost_map[make_pair(_x,_y)];
}

VALUE LIFELONG_PLANNING_A_STAR::calculate_key(const LIFELONG_PLANNING_A_STAR_NODE* const _u){
    VALUE k;
    k.primary=min(_u->g,_u->rhs)+_u->h;
    k.secondary=min(_u->g,_u->rhs);
    return k;
}

float LIFELONG_PLANNING_A_STAR::calculate_h(const LIFELONG_PLANNING_A_STAR_NODE* const _u){
    //return calculate_distance(_u,X_g);
    return float(0.99)*calculate_distance(_u,X_g);
    //return 0;
}

vector<LIFELONG_PLANNING_A_STAR_NODE*> LIFELONG_PLANNING_A_STAR::get_surround_node_list(const LIFELONG_PLANNING_A_STAR_NODE* const _x){
    vector<LIFELONG_PLANNING_A_STAR_NODE*> list;
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
