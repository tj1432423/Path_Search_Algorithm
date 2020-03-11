#ifndef PNODE_MIN_HEAP_OPT_H
#define PNODE_MIN_HEAP_OPT_H

#include <unordered_map>
#include <iostream>
#include <vector>
using namespace std;

template<class T_VALUE,class T_NODE>

class Min_Heap_Map_Opt{
public:
    void Heap_push(const pair<T_VALUE,T_NODE *>& new_node_pair){
        Min_Heap.push_back(new_node_pair);
        Mp[new_node_pair.second]=Min_Heap.size()-1;
        siftup(Min_Heap.size()-1);
    }

    void Heap_pop(){
        if (Min_Heap.empty()){
            cout<<" The Open_List_Min_Heap is empty !!! "<<endl;
            return;
        }
        if (Min_Heap.size()==1){
            Mp.erase(Min_Heap.front().second);
            Min_Heap.pop_back();
            return;
        }
        swap(Min_Heap[0],Min_Heap[Min_Heap.size()-1]);
        Mp[Min_Heap[0].second]=0;
        Mp.erase(Min_Heap.back().second);
        Min_Heap.pop_back();
        siftdown(0);
    }

    void Heap_delect(T_NODE* obj_Node){
        if(Mp.count(obj_Node)==0){
            cout<<" Cannot find this node in Open_List_Min_Heap!!! "<<endl;
            return;
        }
        size_t index=Mp[obj_Node];
        swap(Min_Heap[index],Min_Heap[Min_Heap.size()-1]);
        Mp[Min_Heap[index].second]=index;
        Mp.erase(Min_Heap.back().second);
        Min_Heap.pop_back();
        if (index!=0 && Min_Heap[index]<Min_Heap[(index-1)/2]){     // 若节点小于父亲节点上浮
            siftup(index);
        }
        else{                                                        // 若节点不于父亲节点下沉
            siftdown(index);
        }
    }

    size_t Heap_size(){
        return Min_Heap.size();
    }

    pair<T_VALUE,T_NODE *> Heap_top(){
        if (Min_Heap.empty()){
            cout<<" The Open_List_Min_Heap is empty !!! "<<endl;
            return make_pair(0,nullptr);
        }
        return Min_Heap[0];
    }



private:
    void siftdown(size_t index){     //将索引为Index的节点下沉
        if(index>=Min_Heap.size()){
            cout<<" The Index is out of the range of Open_List_Min_Heap!!! ----siftdown"<<endl;
            return;
        }
        size_t size=Min_Heap.size();
        while(2*index+1<size){   //索引为Index的节点存在孩子时
            size_t c_left,c_right,c;
            c_left=2*index+1;   //左孩子节点索引
            c_right=2*index+2;  //又孩子节点索引
            c=(c_right<size && Min_Heap[c_left]>Min_Heap[c_right]) ? c_right : c_left;  // c是值最小的孩子节点
            if (Min_Heap[index]>Min_Heap[c]) {
                swap(Min_Heap[index],Min_Heap[c]);  // 若孩子节点小于父亲节点,交换位置
                Mp[Min_Heap[index].second]=index;
                Mp[Min_Heap[c].second]=c;
            }
            else break;
            /**若此时节点不是最小值,则有可能也大于<以该节点为根的下一个子堆>的孩子节点，所以需要将该节点也进行一次下沉操作**/
            index=c;
        }
    }
    void siftup(size_t index){     //将索引为Index的节点上浮，
        if(index>=Min_Heap.size()){
            cout<<" The Index is out of the range of Open_List_Min_Heap!!!---- siftup"<<endl;
            return;
        }
        while((int(index)-1)/2>=0){   //索引为Index的节点不是根节点时
            size_t c=(index-1)/2;
            if(Min_Heap[index]<Min_Heap[c]){
                swap(Min_Heap[index],Min_Heap[c]);  // 若孩子节点小于父亲节点,交换位置
                Mp[Min_Heap[index].second]=index;
                Mp[Min_Heap[c].second]=c;
            }
            else break;
            /**若此时节点小于其父节点,则有可能也小于<以该节点为叶的下一个子堆>的父节点，所以需要将该节点也进行一次上浮操作**/
            index=c;
        }
    }

    vector<pair<T_VALUE,T_NODE *>> Min_Heap;
    unordered_map<T_NODE *,size_t> Mp;
};


#endif