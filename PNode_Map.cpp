#include "PNode_Map.h"

void PNode_Map::Build_Map(const vector<vector<int>>& array){
    //为简单，干脆把把下面数组转为链表结构的数组
    //约定：0是可走的，1表示障碍物不可走，2表示起点，3表示终点，4表示路径
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

    vector<PNode*> tmp(array[0].size());
    vector<vector<PNode*>> tmp_node(array.size(),tmp);
    pnode=tmp_node;
    size_t row_num=pnode.size(),col_num=pnode[0].size();

    for (size_t i=0;i<row_num;i++)
    {
       for (size_t j=0;j<col_num;j++)
       {
           pnode[i][j]=new PNode;
       }
    }


    size_t i_start=0,j_start=0,i_end=0,j_end=0;

    for (size_t i=0;i<row_num;i++)
    {
       for (size_t j=0;j<col_num;j++)
       {
           /********** Write node type****************/
           if (array[i][j]==0) {
                pnode[i][j]->nodetype=Reachable;
           }
           else if (array[i][j]==1) {
                pnode[i][j]->nodetype=UnReachable;
           }
           else if (array[i][j]==2) {
                pnode[i][j]->nodetype=StartPoint;
                i_start=i;j_start=j;
           }
           else {
                pnode[i][j]->nodetype=EndPoint;
                i_end=i;j_end=j;
           }

           /********** Write node position****************/
           pnode[i][j]->x=j;
           pnode[i][j]->y=i;
           /********** Write adjacent_node position****************/
           size_t max_row=row_num-1;
           size_t max_col=col_num-1;
           if ((i>0) && (i<max_row) && (j>0) && (j<max_col)){
                pnode[i][j]->adjacent_node.push_back(pnode[i-1][j]);
                pnode[i][j]->adjacent_node.push_back(pnode[i+1][j]);
                pnode[i][j]->adjacent_node.push_back(pnode[i][j-1]);
                pnode[i][j]->adjacent_node.push_back(pnode[i][j+1]);
           }
           else if ((i==0) & (j!=0) & (j!=max_col) ){
                pnode[i][j]->adjacent_node.push_back(pnode[i][j-1]);
                pnode[i][j]->adjacent_node.push_back(pnode[i][j+1]);
                pnode[i][j]->adjacent_node.push_back(pnode[i+1][j]);
           }
           else if ((i==max_row) & (j!=0) & (j!=max_col) ){
                pnode[i][j]->adjacent_node.push_back(pnode[i][j-1]);
                pnode[i][j]->adjacent_node.push_back(pnode[i][j+1]);
                pnode[i][j]->adjacent_node.push_back(pnode[i-1][j]);
           }
           else if ((j==0) & (i!=0) & (i!=max_row) ){
                pnode[i][j]->adjacent_node.push_back(pnode[i+1][j]);
                pnode[i][j]->adjacent_node.push_back(pnode[i-1][j]);
                pnode[i][j]->adjacent_node.push_back(pnode[i][j+1]);
           }
           else if ((j==max_col) & (i!=0) & (i!=max_row) ){
                pnode[i][j]->adjacent_node.push_back(pnode[i+1][j]);
                pnode[i][j]->adjacent_node.push_back(pnode[i-1][j]);
                pnode[i][j]->adjacent_node.push_back(pnode[i][j-1]);
           }
           else if ((i==0) & (j==0)){
                pnode[i][j]->adjacent_node.push_back(pnode[i+1][j]);
                pnode[i][j]->adjacent_node.push_back(pnode[i][j+1]);
           }
           else if ((i==0) & (j==max_col)){
                pnode[i][j]->adjacent_node.push_back(pnode[i+1][j]);
                pnode[i][j]->adjacent_node.push_back(pnode[i][j-1]);
           }
           else if ((i==max_row) & (j==0)){
                pnode[i][j]->adjacent_node.push_back(pnode[i-1][j]);
                pnode[i][j]->adjacent_node.push_back(pnode[i][j+1]);
           }
           else if ((i==max_row) & (j==max_col)){
                pnode[i][j]->adjacent_node.push_back(pnode[i-1][j]);
                pnode[i][j]->adjacent_node.push_back(pnode[i][j-1]);
           }
           else{
                cout<<"Fatal Wrong!!"<<endl;
                return;
           }
           /********** Write default data****************/
           //lnode[i][j]->next=nullptr;
           pnode[i][j]->path_before=nullptr;
           pnode[i][j]->open_flag=0;
           pnode[i][j]->close_flag=0;
           pnode[i][j]->F=0;
           pnode[i][j]->G=0;
           pnode[i][j]->H=0;
       }
    }

/*
    cout<<"hello world!  "<<lnode[2][3]->x<<" , "<<lnode[2][3]->y<<endl;
    cout<<"hello world!  "<<lnode[0][3]->x<<" , "<<lnode[0][3]->y<<endl;
    cout<<"hello world!  "<<lnode[1][2]->x<<" , "<<lnode[1][2]->y<<endl;
    cout<<"hello world!  "<<lnode[1][4]->x<<" , "<<lnode[1][4]->y<<endl;

        cout<<"hello world!  "<<lnode[1][3]->adjacent_next[0]->x<<" , "<<lnode[1][3]->adjacent_next[0]->y<<endl;
       cout<<"hello world!  "<<lnode[1][3]->adjacent_next[1]->x<<" , "<<lnode[1][3]->adjacent_next[1]->y<<endl;
        cout<<"hello world!  "<<lnode[1][3]->adjacent_next[2]->x<<" , "<<lnode[1][3]->adjacent_next[2]->y<<endl;
        cout<<"hello world!  "<<lnode[1][3]->adjacent_next[3]->x<<" , "<<lnode[1][3]->adjacent_next[3]->y<<endl;
 */
    Start_node=pnode[i_start][j_start];
    End_node=pnode[i_end][j_end];
    cout<<" The Map has been created succesfull !!! "<<endl;
}

void PNode_Map::Clear_Map(){
    size_t row_num=pnode.size(),col_num=pnode[0].size();
    for (size_t i=0;i<row_num;i++)
    {
       for (size_t j=0;j<col_num;j++)
       {
           delete pnode[i][j];
       }
    }
    pnode.clear();
    Start_node=nullptr;
    End_node=nullptr;
    cout<<" The Map has been cleared succesfull !!! "<<endl;
}

