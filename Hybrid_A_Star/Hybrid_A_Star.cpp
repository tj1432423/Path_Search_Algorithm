#include "Hybrid_A_Star.h"
#include "../A_Star/A_Star.h"
#include "../RS_Lib/rs.h"
#include <math.h>
#include <limits.h>

#define pi 3.1415926
#define BIG_COST float(99999999.9)

void Hybrid_A_Star::Set_Vehicle_Parameters(float _max_front_wheel_angle,float _wheel_base,float _back_to_rear,float _front_to_rear,float _width){
    vehicle_parameters.wheel_base=_wheel_base;
    vehicle_parameters.width=_width;
    vehicle_parameters.back_to_rear=_back_to_rear;
    vehicle_parameters.front_to_rear=_front_to_rear;
    vehicle_parameters.max_front_wheel_angle=fabs(_max_front_wheel_angle);
    vehicle_parameters.min_turn_radius=vehicle_parameters.wheel_base/tan(vehicle_parameters.max_front_wheel_angle);
}

void Hybrid_A_Star::Set_Search_Parameters(float _heading_resolution,float _front_wheel_angle_resolution,float _move_resolution,float _collision_check_resolution,bool _debug_info_switch){
    search_parameters.move_resolution=_move_resolution;
    search_parameters.heading_resolution=_heading_resolution;
    search_parameters.front_wheel_angle_resolution=_front_wheel_angle_resolution;
    search_parameters.debug_info_switch=_debug_info_switch;
    search_parameters.collision_check_resolution=_collision_check_resolution;
}


bool Hybrid_A_Star::Load_Map(const vector<vector<int>>& _array,const vector<float>& _start_point,const vector<float>& _end_point,int _Max_Search_Time){
    //为简单，干脆把把下面数组转为链表结构的数组
    //约定：0是可走的，1表示障碍物不可走，2表示起点，3表示终点   //本函数输入的_array不包含起点和终点
//    vector<vector<int>> array={
//        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//        { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
//        { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
//        { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
//        { 0, 0, 0, 0, 0, 1, 3, 0, 0, 0 },
//        { 0, 0, 2, 0, 0, 1, 0, 0, 0, 0 },
//        { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
//        { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
//        { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
//        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
    obstacles_map=_array;
    start_point= _start_point;  //[x,y,phi,front_wheel_angle]
    end_point=_end_point;       //[x,y,phi,front_wheel_angle]
    vector<vector<int>> array;
    array=_array;
    vector<int> index_end_point;
    vector<int> index_start_point;
    index_end_point=Point_Index_Calculate(end_point);
    index_start_point=Point_Index_Calculate(start_point);

    if(index_start_point[0]==index_end_point[0] && index_start_point[1]==index_end_point[1]) return false;
    if(array[size_t(index_start_point[0])][size_t(index_start_point[1])] !=0 ) return false;
    if(array[size_t(index_end_point[0])][size_t(index_end_point[1])] !=0 ) return false;
    A_Star a_star;
    vector<float> tmp(_array.size(),0.0);
    vector<vector<float>> ttmp(_array[0].size(),tmp);
    a_star_distance_table=ttmp;  //only initial
    array[size_t(index_end_point[0])][size_t(index_end_point[1])]=3;   //set the end_point;

    for(size_t i=0;i<array.size();i++){
        for(size_t j=0;j<array[i].size();j++){

            if(array[i][j]==1){
                a_star_distance_table[i][j]=BIG_COST;
                continue;
            }
            if(array[i][j]==3){
                a_star_distance_table[i][j]=0.0;
                continue;
            }
            array[i][j]=2;

            int Errorflag=a_star.Get_Shortest_Path(array,_Max_Search_Time,false);
            if(Errorflag){
                a_star_distance_table[i][j]=BIG_COST;
            }
            else{
                a_star_distance_table[i][j]=a_star.Shortest_Path_Long;
            }
            array[i][j]=0;
//            if(Errorflag) {
//                cout<<" Fatal Wrong , The A_Star algorithm can not get all of the shortest paths !!! "<<endl;
//                return false;
//            }
            a_star.clear();         //very important!!!! ----KDK 20200329
        }
    }
    return true;
}





bool Hybrid_A_Star::Get_The_Shortest_Path(int Max_Search_Time){
    /*************** Set the start node ****************/
    Hybrid_A_Star_Node* start_hybrid_a_star_node=new Hybrid_A_Star_Node();
    Set_The_Hybrid_A_Star_Node(start_hybrid_a_star_node,start_point,Foward,nullptr,float(0.0));
    if (start_hybrid_a_star_node->F>(BIG_COST-float(1.0))){     //we can not use >= between float in computer
        start_hybrid_a_star_node->close_flag=true;
        start_hybrid_a_star_node->open_flag=false;
        return false;
    }
    else{
        start_hybrid_a_star_node->close_flag=false;
        start_hybrid_a_star_node->open_flag=true;
        open_list.Heap_push(make_pair(start_hybrid_a_star_node->F,start_hybrid_a_star_node));
    }
    vector<int> tmp={start_hybrid_a_star_node->index_x,start_hybrid_a_star_node->index_y,start_hybrid_a_star_node->index_phi};
    mp[make_pair(tmp,start_hybrid_a_star_node->direction)]=start_hybrid_a_star_node;
     /*************** Main Search begin ****************/
    int count=0;
    while(open_list.Heap_size()){
        Hybrid_A_Star_Node* current_node=open_list.Heap_top().second;
        open_list.Heap_pop();
        current_node->open_flag=false;
        current_node->close_flag=true;

        cout<<"the top open list node:[ "<<current_node->x<<","<<current_node->y<<" ]"<<endl;

        /************************RS shengcheng***************************/
        double q0[3]={current_node->x,current_node->y,current_node->phi};
        double q1[3]={end_point[0],end_point[1],end_point[2]};
        ReedsSheppStateSpace   reedssheppstatespace(vehicle_parameters.min_turn_radius);
        vector<vector<double>> tmp_rs_finalpath;
        tmp_rs_finalpath=reedssheppstatespace.xingshensample(q0,q1,search_parameters.move_resolution);
        /******************** RS colliction verify***********************/
        vector<vector<float>> rs_finalpath;
        bool rs_path_ok=true;
        for(size_t i=0;i<tmp_rs_finalpath.size();i++){
            rs_finalpath.push_back({float(tmp_rs_finalpath[i][0]),float(tmp_rs_finalpath[i][1]),float(tmp_rs_finalpath[i][2])});
            //cout<<"-->"<<"[ "<<rs_finalpath.back()[0]<<","<<rs_finalpath.back()[1]<<","<<rs_finalpath.back()[2]<<" ]";
            if(! Collision_Check(rs_finalpath.back())){
                rs_path_ok=false;
                break;
            }
        }
        if(rs_path_ok){
               cout<<"the rs path is ok!!"<<endl;
                rs_path=rs_finalpath;
                Hybrid_A_Star_Node* tmp_get_res_ptr;
                tmp_get_res_ptr=current_node;
                while(tmp_get_res_ptr){
                    a_star_path.push_back({tmp_get_res_ptr->x,tmp_get_res_ptr->y,tmp_get_res_ptr->phi});
                    tmp_get_res_ptr=tmp_get_res_ptr->path_before;
                }
                reverse(a_star_path.begin(),a_star_path.end());
                return true;
        }
        /******************** A_Star search***********************/
        bool search_flag=search(current_node);
        if(!search_flag) break;
        count++;
        if(count>Max_Search_Time){
            cout<<"Time run out, can not get the path !!!"<<endl;
            return false;
        }
    }
    cout<<"The open list is empty, can not get the path !!!"<<endl;
    return false;
}

bool Hybrid_A_Star::search(Hybrid_A_Star_Node* _current_node){                      //on going!!!!!!!!!!!!!!!!
    vector<float> current_pos(3,0.0);
    current_pos[0]=_current_node->x;
    current_pos[1]=_current_node->y;
    current_pos[2]=_current_node->phi;
    for(int i=0;i<2;i++){
        Drive_Direction direction;
        if (i==0) direction=Foward;
        else direction=Backward;
        float max_front_wheel_angle=vehicle_parameters.max_front_wheel_angle;
        float min_front_wheel_angle=-vehicle_parameters.max_front_wheel_angle;
        float front_wheel_angle_resolution=search_parameters.front_wheel_angle_resolution;
        for(float front_wheel_angle=min_front_wheel_angle;front_wheel_angle<max_front_wheel_angle;front_wheel_angle=(front_wheel_angle+front_wheel_angle_resolution)){

            vector<float> target_pos;

            target_pos=Vehicle_Kinematic(current_pos,front_wheel_angle,search_parameters.move_resolution,direction);

            cout<<front_wheel_angle<<"-----------[ "<<target_pos[0]<<","<<target_pos[1]<<","<<target_pos[2]<<","<<target_pos[3]<<" ]"<<endl;
            if(Collision_Check(target_pos)){          //trajectory has no collision!
                vector<int> index_target_pos=Point_Index_Calculate(target_pos);
                float tmp_Cost=Move_Cost_Calculate(_current_node,front_wheel_angle,direction);
                float tmp_G=tmp_Cost+_current_node->G;
                if(mp.count(make_pair(index_target_pos,direction))){
                    Hybrid_A_Star_Node* tmp_node=mp[make_pair(index_target_pos,direction)];
                    if(tmp_node->close_flag==true && tmp_node->open_flag==false){    //in the close_list
                        continue;
                    }
                    else if(tmp_node->close_flag==false && tmp_node->open_flag==true){    //in the open_list
                        if(tmp_G<tmp_node->G){    //should update this node in the open_list!
                                Set_The_Hybrid_A_Star_Node(tmp_node,target_pos,direction,_current_node,tmp_G);
                                open_list.Heap_modify(tmp_node,tmp_node->F);

                        }
                        else{
                            continue;
                        }
                    }
                    else{
                        cout<<"fatel wrong!!! a node is in or not in the both open_list and close_list !!!"<<endl;
                        return false;
                    }
                }
                else{           //the new node, should be added!!
                    Hybrid_A_Star_Node* tmp_node=new Hybrid_A_Star_Node();
                    Set_The_Hybrid_A_Star_Node(tmp_node,target_pos,direction,_current_node,tmp_G);
                    tmp_node->open_flag=true;
                    tmp_node->close_flag=false;
                    mp[make_pair(index_target_pos,direction)]=tmp_node;
                    open_list.Heap_push(make_pair(tmp_node->F,tmp_node));
                }
            }
            cout<<"22222222222222"<<endl;
        }
    }
    return true;
}

bool Hybrid_A_Star::Collision_Check(const vector<float>& target_pos){       //need modify!!!!!!!!!!!!!!!
    vector<int> index_target_pos=Point_Index_Calculate(target_pos);
    if(index_target_pos[0]>=0 && size_t(index_target_pos[0])<obstacles_map.size() && index_target_pos[1]>=0 && size_t(index_target_pos[1])<obstacles_map[0].size()){
        if(obstacles_map[size_t(index_target_pos[0])][size_t(index_target_pos[1])]==0){
            return true;
        }
    }
    return false;
}

float Hybrid_A_Star::Move_Cost_Calculate(Hybrid_A_Star_Node* _current_node,float _front_wheel_angle,Drive_Direction _direction){
    float Cost=0;
    float weight_1=float(1.0);
    float weight_2=float(2.0);
    Cost+=weight_1*fabs(_front_wheel_angle-_current_node->steer_angle);
    Cost+=weight_2*fabs(_direction-_current_node->direction);
    Cost+=search_parameters.move_resolution;
    return Cost;
}


vector<float> Hybrid_A_Star::Vehicle_Kinematic(const vector<float>& _current_pos,float _front_wheel_angle,float _move_distance,Drive_Direction _direction){
    vector<float> target_pos(3,0.0);
    vector<float> tmp_target_pos(3,0.0);
    float min_num=search_parameters.front_wheel_angle_resolution/float(2.0);
    if(_front_wheel_angle<min_num && _front_wheel_angle>-min_num){
        tmp_target_pos[0]=_move_distance;
        tmp_target_pos[1]=0.0;
        tmp_target_pos[2]=0.0;
    }
    else{
        float R=fabs(vehicle_parameters.wheel_base/tan(_front_wheel_angle));
        float Alpha=_move_distance/R;
        tmp_target_pos[0]=R*sin(Alpha);
        tmp_target_pos[1]=_front_wheel_angle > 0 ? R*(1-cos(Alpha)) : R*(-1+cos(Alpha));
        tmp_target_pos[2]=_front_wheel_angle > 0 ? Alpha : -Alpha;
    }
    if(_direction==Backward){
        tmp_target_pos[0]=-tmp_target_pos[0];
        tmp_target_pos[2]=-tmp_target_pos[2];
    }
    vector<float> deta={_current_pos[0],_current_pos[1],_current_pos[2]};
    target_pos[0]=tmp_target_pos[0]*cos(deta[2])-tmp_target_pos[1]*sin(deta[2])+deta[0];        //二维坐标旋转公式,代待验证  --KDK 20200328
    target_pos[1]=tmp_target_pos[0]*sin(deta[2])+tmp_target_pos[1]*cos(deta[2])+deta[1];        //二维坐标旋转公式,代待验证  --KDK 20200328
    target_pos[2]=deta[2]+tmp_target_pos[2];
    target_pos.push_back(_front_wheel_angle);
    return target_pos;
}


void Hybrid_A_Star::Set_The_Hybrid_A_Star_Node(Hybrid_A_Star_Node* _targer_hybrid_a_star_node,vector<float> _point,Drive_Direction _direction,Hybrid_A_Star_Node* _node_before,float _G){
    // the struct of _point-->[x,y,phi,front_wheel_angle]
    _targer_hybrid_a_star_node->x=_point[0];
    _targer_hybrid_a_star_node->y=_point[1];
    _targer_hybrid_a_star_node->phi=_point[2];
    _targer_hybrid_a_star_node->steer_angle=_point[3];
    vector<int> index_start_point=Point_Index_Calculate(_point);
    _targer_hybrid_a_star_node->index_x=index_start_point[0];
    _targer_hybrid_a_star_node->index_y=index_start_point[1];
    _targer_hybrid_a_star_node->index_phi=index_start_point[2];
    _targer_hybrid_a_star_node->direction=_direction;     //we assume that the direction of vehicle if foward at the begin.
    _targer_hybrid_a_star_node->path_before=_node_before;
    _targer_hybrid_a_star_node->G=_G;
    if(_targer_hybrid_a_star_node->index_x>=0 && _targer_hybrid_a_star_node->index_x<int(a_star_distance_table.size()) \
            && _targer_hybrid_a_star_node->index_y>=0 && _targer_hybrid_a_star_node->index_y<int(a_star_distance_table[0].size())){
        _targer_hybrid_a_star_node->H=a_star_distance_table[size_t(_targer_hybrid_a_star_node->index_x)][size_t(_targer_hybrid_a_star_node->index_y)];
    }
    else{
        _targer_hybrid_a_star_node->H=BIG_COST;
    }
    _targer_hybrid_a_star_node->F=_targer_hybrid_a_star_node->G+_targer_hybrid_a_star_node->H;
}

vector<int> Hybrid_A_Star::Point_Index_Calculate(const vector<float>& _point){
    vector<int> Index(3,0);
    Index[1]=round(_point[0]);
    Index[0]=round(_point[1]);
    Index[2]=round(_point[2]/search_parameters.heading_resolution);
    return Index;
}













