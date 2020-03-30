#include "reeds_shepp.h"
#include "dubins.h"
#include <vector>
#include <iostream>
#define pi 3.1415926
using namespace std;

int main(){
    ReedsSheppStateSpace reedssheppstatespace(3.0);
    double q0[3]={0,0,0};
    double q1[3]={5,5,pi};
    double length;
    vector<vector<double>> point;

    reedssheppstatespace.sample(q0,q1,3,length,point);
    cout<<"The reeds-shepp path is :"<<endl;
    cout<<"[ "<<q0[0]<<" , "<<q0[1]<<" , "<<q0[2]<<" ]"<<endl;
    for(size_t i=0;i<point.size();i++){
        cout<<"[ "<<point[i][0]<<" , "<<point[i][1]<<" , "<<point[i][2]<<" ]"<<endl;
    }
    cout<<"[ "<<q1[0]<<" , "<<q1[1]<<" , "<<q1[2]<<" ]"<<endl;

    cout<<"----------------------------"<<endl;

    point.clear();
    DubinsStateSpace dubinssheppstatespace(3.0);
    dubinssheppstatespace.sample(q0,q1,3,length,point);
    cout<<"The dubins path is :"<<endl;
    cout<<"[ "<<q0[0]<<" , "<<q0[1]<<" , "<<q0[2]<<" ]"<<endl;
    for(size_t i=0;i<point.size();i++){
        cout<<"[ "<<point[i][0]<<" , "<<point[i][1]<<" , "<<point[i][2]<<" ]"<<endl;
    }
    cout<<"[ "<<q1[0]<<" , "<<q1[1]<<" , "<<q1[2]<<" ]"<<endl;

    return 0;
}
