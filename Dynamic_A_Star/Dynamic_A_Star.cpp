#include "Dynamic_A_Star.h"

using namespace DYNAMIC_A_STAR_NAMESPACE;

DYNAMIC_A_STAR::DYNAMIC_A_STAR(const vector<vector<int>>& _array){
    if(_array.empty()){
        cout<<"Loat Map fail !!!"<<endl;
        exit(0);
    }
    row_num=_array.size();
    col_num=_array[0].size();

    vector<DYNAMIC_A_STAR_NODE*> tmp(col_num);
    vector<vector<DYNAMIC_A_STAR_NODE*>> ttmp(row_num,tmp);
    node_table=ttmp;

    obstacle_cost.val=big_cost;

    for(size_t ii=0;ii<_array.size();ii++){
        grid_map.resize(grid_map.size()+1);
        for(size_t jj=0;jj<_array[0].size();jj++){
            grid_map[ii].push_back(NODE_TYPE(_array[ii][jj]));  // load and save the map !!! force type change !!!
            node_table[ii][jj]= new DYNAMIC_A_STAR_NODE();
            node_table[ii][jj]->H.val=0;
            node_table[ii][jj]->K.val=0;
            node_table[ii][jj]->next=nullptr;
            node_table[ii][jj]->Location=make_pair(ii,jj);
            node_table[ii][jj]->nodetype=DYNAMIC_A_STAR_NODE::New;
        }
    }

    for(size_t ii=0;ii<_array.size();ii++){
        for(size_t jj=0;jj<_array[0].size();jj++){
            vector<DYNAMIC_A_STAR_NODE*> surround_node_list=get_surround_node_list(node_table[ii][jj]);
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
                X_c=node_table[ii][jj];
                cur_flag=true;
            }

        }
    }
    if(!goal_flag){
        cout<<"Missing the goal !!!"<<endl;
        exit(0);
    }
}

void DYNAMIC_A_STAR::set_cur_location(const LOCATION& _x){
    if(!location_check(_x)){
        cout<<"Wrong cur location !!! "<<endl;
        exit(0);
    }
    X_c=node_table[_x.first][_x.second];
    cur_flag=true;
}

vector<LOCATION> DYNAMIC_A_STAR::init_plan(){
    if(!cur_flag){cout<<"fetal wrong !!! please set the current location first !!!"<<endl;exit(0);}
    VALUE unvalid(-1);
    VALUE k_min;
    open_list.Heap_push(make_pair(X_g->K,X_g));
    X_g->nodetype=DYNAMIC_A_STAR_NODE::Open;
    do{
        k_min = process_state();
    }while (k_min!=unvalid && X_c->nodetype!=DYNAMIC_A_STAR_NODE::Close);
    init_plan_flag=true;
    return get_path();
}

vector<LOCATION> DYNAMIC_A_STAR::repair_replan(){
    if(!cur_flag){cout<<"fetal wrong !!! please set the current location first !!!"<<endl;exit(0);}
    if(!init_plan_flag){cout<<"fetal wrong !!! please run the init_plan() first !!!"<<endl;exit(0);}
    VALUE unvalid(-1);
    VALUE k_min;
    do{
        k_min = process_state();
    }while (k_min < X_c->H && k_min!=unvalid);
    return get_path();
}

void DYNAMIC_A_STAR::prepair_repair(){
    if(!cur_flag){cout<<"fetal wrong !!! please set the current location first !!!"<<endl;exit(0);}
    if(!init_plan_flag){cout<<"fetal wrong !!! please run the init_plan() first !!!"<<endl;exit(0);}
    vector<DYNAMIC_A_STAR_NODE*> surround_node_list_1=get_surround_node_list(X_c);
    for(size_t ii=0;ii<surround_node_list_1.size();ii++){
        DYNAMIC_A_STAR_NODE* X=node_table[surround_node_list_1[ii]->Location.first][surround_node_list_1[ii]->Location.second];
        vector<DYNAMIC_A_STAR_NODE*> surround_node_list_2=get_surround_node_list(X);
        for(size_t jj=0;jj<surround_node_list_2.size();jj++){
            DYNAMIC_A_STAR_NODE* Y=node_table[surround_node_list_2[jj]->Location.first][surround_node_list_2[jj]->Location.second];
            if(Detect(Y,X) != Cost(Y,X)){
                //VALUE aa=Detect(Y,X);
                //VALUE bb=Cost(Y,X);
                modify_cost(Y,X,Detect(Y,X));
            }
        }
        for(size_t jj=0;jj<surround_node_list_2.size();jj++){
            DYNAMIC_A_STAR_NODE* Y=node_table[surround_node_list_2[jj]->Location.first][surround_node_list_2[jj]->Location.second];
            if(Detect(X,Y) != Cost(X,Y)){
                modify_cost(X,Y,Detect(X,Y));
            }
        }
    }
}

void DYNAMIC_A_STAR::update_map(const vector<vector<int>>& _array){
    for(size_t ii=0;ii<_array.size();ii++){
        for(size_t jj=0;jj<_array[0].size();jj++){
            grid_map[ii][jj]=NODE_TYPE(_array[ii][jj]);  // load and save the map !!! force typr change !!!
        }
    }
}

DYNAMIC_A_STAR::~DYNAMIC_A_STAR(){
    if(node_table.empty()) return;
    for (size_t ii=0;ii<node_table.size();ii++) {
        for (size_t jj=0;jj<node_table.size();jj++) {
            delete node_table[ii][jj];
            node_table[ii][jj]=nullptr;
        }
    }
}


void DYNAMIC_A_STAR::modify_cost(DYNAMIC_A_STAR_NODE* const _x,DYNAMIC_A_STAR_NODE* const _y,VALUE _val){
    cost_map[make_pair(_x,_y)]=_val;
    if(_x->nodetype==DYNAMIC_A_STAR_NODE::Close){
        insert(_x,_x->H);
    }
    //return open_list.Heap_top().first;
}

VALUE DYNAMIC_A_STAR::process_state(){
    VALUE unvalid(-1);
    VALUE k_old;
    if(open_list.Heap_size()==0) return unvalid;
    DYNAMIC_A_STAR_NODE* X=open_list.Heap_top().second;
    k_old=open_list.Heap_top().first;
    open_list.Heap_pop();
    X->nodetype=DYNAMIC_A_STAR_NODE::Close;
    if(k_old<X->H){
        vector<DYNAMIC_A_STAR_NODE*> surround_node_list=get_surround_node_list(X);
        for(size_t ii=0;ii<surround_node_list.size();ii++){
            DYNAMIC_A_STAR_NODE* Y=surround_node_list[ii];
            if(Y->nodetype != DYNAMIC_A_STAR_NODE::New
                        && Y->H <= k_old
                                && X->H > (Y->H + Cost(Y,X))
                    ){
                X->next=Y;
                X->H=Y->H+Cost(Y,X);
            }
        }
    }
    if(k_old==X->H){
        vector<DYNAMIC_A_STAR_NODE*> surround_node_list=get_surround_node_list(X);
        for(size_t ii=0;ii<surround_node_list.size();ii++){
            DYNAMIC_A_STAR_NODE* Y=surround_node_list[ii];
            if(Y->nodetype==DYNAMIC_A_STAR_NODE::New
                    || (Y->next==X && Y->H != (X->H + Cost(X,Y)))
                            || (Y->next!=X && Y->H > (X->H + Cost(X,Y)))
                    ){
                Y->next=X;
                insert(Y,X->H+Cost(X,Y));
            }
        }
    }
    else {
        vector<DYNAMIC_A_STAR_NODE*> surround_node_list=get_surround_node_list(X);
        for(size_t ii=0;ii<surround_node_list.size();ii++){
            DYNAMIC_A_STAR_NODE* Y=surround_node_list[ii];
            if(Y->nodetype==DYNAMIC_A_STAR_NODE::New
                    || (Y->next==X && Y->H != (X->H+Cost(X,Y)) ) ){
                Y->next=X;
                insert(Y,X->H+Cost(X,Y));
            }
            else {
                if(Y->next != X && Y->H > (X->H + Cost(X,Y))){
                    insert(X,X->H);
                }
                else {
                    if(Y->next != X && X->H > (Y->H +Cost(X,Y)) && Y->nodetype==DYNAMIC_A_STAR_NODE::Close && Y->H > k_old){
                        insert(Y,Y->H);
                    }
                }
            }
        }
    }
    if(open_list.Heap_size()==0) return unvalid;
    return open_list.Heap_top().first;
}


void DYNAMIC_A_STAR::insert(DYNAMIC_A_STAR_NODE* const _x,VALUE _h_new){
    if(_x->nodetype==DYNAMIC_A_STAR_NODE::New){
        _x->K=_h_new;
        open_list.Heap_push(make_pair(_x->K,_x));
    }
    else if(_x->nodetype==DYNAMIC_A_STAR_NODE::Open){
        _x->K=min_value(_x->K,_h_new);
        open_list.Heap_modify(_x,_x->K);
    }
    else {  //Close
        _x->K=min_value(_x->H,_h_new);
        open_list.Heap_push(make_pair(_x->K,_x));
    }
    _x->H=_h_new;
    _x->nodetype=DYNAMIC_A_STAR_NODE::Open;
}




bool DYNAMIC_A_STAR::location_check(const LOCATION& _x){
    if(_x.first>=0 && _x.first<row_num && _x.second>=0 && _x.second<col_num) return true;
    else return false;
}

vector<DYNAMIC_A_STAR_NODE*> DYNAMIC_A_STAR::get_surround_node_list(const DYNAMIC_A_STAR_NODE* const _x){
    vector<DYNAMIC_A_STAR_NODE*> list;
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


VALUE DYNAMIC_A_STAR::calculate_distance(const DYNAMIC_A_STAR_NODE* const _a,const DYNAMIC_A_STAR_NODE* const _b){
    VALUE dis;
    dis.val=float(_a->Location.first-_b->Location.first)*float(_a->Location.first-_b->Location.first)+float(_a->Location.second-_b->Location.second)*float(_a->Location.second-_b->Location.second);
    dis.val=pow(dis.val,0.5);
    return dis;
}

VALUE DYNAMIC_A_STAR::Cost(DYNAMIC_A_STAR_NODE* const _x,DYNAMIC_A_STAR_NODE* const  _y){
    return cost_map[make_pair(_x,_y)];
}

VALUE DYNAMIC_A_STAR::Detect(const DYNAMIC_A_STAR_NODE* const _x,const DYNAMIC_A_STAR_NODE* const _y){
    if(grid_map[_x->Location.first][_x->Location.second]==Obstacle || grid_map[_y->Location.first][_y->Location.second]==Obstacle){
        return obstacle_cost;
    }
    else {
        return calculate_distance(_x,_y);
    }
}

vector<LOCATION> DYNAMIC_A_STAR::get_path(){
    vector<LOCATION> path;
    DYNAMIC_A_STAR_NODE* cur=X_c;
    while(cur != X_g){
        path.push_back(cur->Location);
        cur=cur->next;
    }
    path.push_back(X_g->Location);
    if(path_set.empty() || path_set.back()!=path){
        path_set.push_back(path);
    }
    return path;
}


VALUE DYNAMIC_A_STAR::min_value(VALUE& _a,VALUE& _b){
    return _a<_b ? _a :_b;
}
