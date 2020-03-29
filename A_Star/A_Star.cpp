#include "A_Star.h"

//#include <queue>
//#include <stdio.h>
//#include <math.h>
//#include <vector>
//#include <cassert>
//#include <typeinfo>
//#include <vector>

#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <vector>


using namespace std;

int A_Star::Get_Shortest_Path(const vector<vector<int>>& array,int Max_Search_Time,bool Debug_Info_Switch){
    //为简单，干脆把把下面数组转为链表结构的数组
    //约定：0是可走的，1表示障碍物不可走，2表示起点，3表示终点
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
    debug_info_switch=Debug_Info_Switch;
    int Error_flag=0;
    bool Input_OK=false;
    Input_OK =Input_Verify(array);
    if(!Input_OK) return Error_flag;
    map.Build_Map(array);

    //PNode *Start_node;
//    Start_node=map.Start_node;
    //PNode *End_node;
//    End_node=map.End_node;

    map.Start_node->G=0;
    map.Start_node->H=H_Calculat(map.Start_node,map.End_node);
    map.Start_node->F=map.Start_node->G+map.Start_node->H;
    map.Start_node->open_flag=1;
    open_List.Heap_push(make_pair(map.Start_node->F,map.Start_node));
//   cout<<"hello world!  "<<Open_List->next->G<<" , "<<Open_List->next->H<<endl;

/********** main search ****************/
    Shortest_Path_Long=-1;
    Count=0;
    int Search_end_flag;
    while(1)
    {
        //cout<<"Count=  "<<Count<<endl;
        Search_end_flag=Search();
        if (Search_end_flag==1)
         {
            if(debug_info_switch)  cout<<" Having find the useable path !!! "<<endl;
            Error_flag=0;
            break;
         }
        if (Count>Max_Search_Time)
         {
            cout<<" Path finding timeout !!! "<<endl;
            Error_flag=1;
            break;
         }
        if (Search_end_flag==2)
         {
            if(debug_info_switch)  cout<<" Can not find the useable path !!! "<<endl;
            Error_flag=1;
            break;
         }

        Count++;
 //       cout<<"opt_node->[G,H]  "<<opt_node->G<<" , "<<opt_node->H<<endl;
    }
    if (Error_flag==0)
    {
        PNode *tmp=map.End_node;
        while(tmp)
        {
            //cout<<" ["<<tmp->x<<" , "<<tmp->y<<"] -> ";
            Shortest_Path.push_back({tmp->x,tmp->y});
            tmp=tmp->path_before;
            Shortest_Path_Long++;
        }
        reverse(Shortest_Path.begin(),Shortest_Path.end());  // " use of undlclared identifier 'reverse' ..."  ----it is why?  need #include <boost/math/constants/constants.hpp>
        //for(size_t i=0;i<Shortest_Path.size()/2;i++){
        //    swap(Shortest_Path[i],Shortest_Path[Shortest_Path.size()-1-i]);
        //}

    }

    //map.Clear_Map();

    return Error_flag;
}

bool A_Star::Input_Verify(const vector<vector<int>>& array){
    int Start_Point_Count=0,End_Point_Count=0;
    for(size_t i=0;i<array.size();i++){
        if(array[i].size() != array[0].size()){
            cout<<" Fatal Wrong , The Input Must be a Mutrix !!! "<<endl;
            return false;
        }
        for(size_t j=0;j<array[i].size();j++){
            if (array[i][j]<0 || array[i][j]>3){
                cout<<" Fatal Wrong , The Value in the Input Mutrix must be 0, 1, 2, 3  !!! "<<endl;
                return false;
            }
            if (array[i][j]==2) Start_Point_Count++;
            if (array[i][j]==3) End_Point_Count++;
        }
    }
    if (Start_Point_Count==1 && End_Point_Count==1) return true;
    else{
        if (Start_Point_Count!=1) cout<<" Fatal Wrong , Cannot find the start point or more than one start point !!! "<<endl;
        if (End_Point_Count!=1) cout<<" Fatal Wrong , Cannot find the end point or more than one end point !!! "<<endl;
        return false;
    }
}


float A_Star::H_Calculat(PNode *cur,PNode *end)
{
    float res=float_abs(cur->x-end->x)+float_abs(cur->y-end->y);
    return res;
}

float A_Star::float_abs(float x){
    return x > 0 ? x:-x;
}


int A_Star::Search()
{   
     if (open_List.Heap_size()==0){
         if(debug_info_switch)  cout<<" The Open_List_Min_Heap is empty !!! "<<endl;
         return 2;
     }
     PNode *Opt_node;
     Opt_node=open_List.Heap_top().second;
     Opt_node->open_flag=0;
     Opt_node->close_flag=1;
     open_List.Heap_pop();
    for(size_t i=0;i<Opt_node->adjacent_node.size();i++)
    {
        PNode* Tmp_adjacent_next_node=Opt_node->adjacent_node[i];
        if (Tmp_adjacent_next_node->nodetype==EndPoint)       // End point has been found!!!
        {
            Tmp_adjacent_next_node->path_before=Opt_node;
            return 1;
        }


        if (Tmp_adjacent_next_node->open_flag==0 && Tmp_adjacent_next_node->close_flag==0 && Tmp_adjacent_next_node->nodetype !=UnReachable)
        {
            Tmp_adjacent_next_node->path_before=Opt_node;
            Tmp_adjacent_next_node->G=Opt_node->G+1;
            Tmp_adjacent_next_node->H=H_Calculat(Tmp_adjacent_next_node,map.End_node);
            Tmp_adjacent_next_node->F=Tmp_adjacent_next_node->G+Tmp_adjacent_next_node->H;
            Tmp_adjacent_next_node->open_flag=1;    //The open_flag should be modified! --20200312
            open_List.Heap_push(make_pair(Tmp_adjacent_next_node->F,Tmp_adjacent_next_node));
        }

        else if(Tmp_adjacent_next_node->open_flag==1 && Tmp_adjacent_next_node->close_flag==0)
        {
            float tmp_G=Opt_node->G+1;
            float tmp_H=H_Calculat(Tmp_adjacent_next_node,map.End_node);
            float tmp_F=tmp_G+tmp_H;
            if (tmp_F<Tmp_adjacent_next_node->F){
                //update thr f which is in open list!!!!
                //update thr f which is in open list!!!!
                //update thr f which is in open list!!!!
                Tmp_adjacent_next_node->G=tmp_G;
                Tmp_adjacent_next_node->H=tmp_H;
                Tmp_adjacent_next_node->F=tmp_F;
                Tmp_adjacent_next_node->path_before=Opt_node;
                //open_List.Heap_delect(Tmp_adjacent_next_node);
                //open_List.Heap_push(make_pair(Tmp_adjacent_next_node->F,Tmp_adjacent_next_node));
                open_List.Heap_modify(Tmp_adjacent_next_node,Tmp_adjacent_next_node->F);

            }
            else {
                //no action!
            }

        }
        else if(Tmp_adjacent_next_node->open_flag==0 && Tmp_adjacent_next_node->close_flag==1)
        {
            // no action!
        }
        else if((Tmp_adjacent_next_node->open_flag==0 && Tmp_adjacent_next_node->close_flag==0 && Tmp_adjacent_next_node->nodetype ==UnReachable))
        {
            Tmp_adjacent_next_node->close_flag=1;
        }
        else {
            cout<<"Fatel Error!!!"<<endl;
             return 2;
        }

    }

    return 0;
}

void A_Star::clear(){
    Shortest_Path.clear();
    open_List.Heap_clear();
    map.Clear_Map();
}
