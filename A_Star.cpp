#include "A_Star.h"

//#include <queue>
//#include <stdio.h>
#include <math.h>
#include <iostream>

#include <vector>


using namespace std;

int A_Star::Get_Shortest_Path(const vector<vector<int>>& array,int Max_Search_Time){
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
    int Search_end_flag;
    int Error_flag;
    int Count=0;
    while(1)
    {
        //cout<<"Count=  "<<Count<<endl;
        Search_end_flag=Search();
        if (Search_end_flag==1)
         {
            cout<<" Having find the useable path !!! "<<endl;
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
            cout<<" Can not find the useable path !!! "<<endl;
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

        }
        //reverse(Shortest_Path.begin(),Shortest_Path.end());  // " use of undlclared identifier 'reverse' ..."  ----it is why?
        for(size_t i=0;i<Shortest_Path.size()/2;i++){
            swap(Shortest_Path[i],Shortest_Path[Shortest_Path.size()-1-i]);
        }

    }

    map.Clear_Map();

    return Error_flag;
}




float A_Star::H_Calculat(PNode *cur,PNode *end)
{
    Math_KDK math_kdk;
    float res=math_kdk.abs(cur->x-end->x)+math_kdk.abs(cur->y-end->y);
    return res;
}



int A_Star::Search()
{   
     if (open_List.Heap_size()==0){
         cout<<" The Open_List_Min_Heap is empty !!! "<<endl;
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
                open_List.Heap_delect(Tmp_adjacent_next_node);
                open_List.Heap_push(make_pair(Tmp_adjacent_next_node->F,Tmp_adjacent_next_node));

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
