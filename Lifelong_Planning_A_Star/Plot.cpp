#include <stdlib.h>
#include <time.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>
//#include <GL/aux.h>
#include <math.h>
#include "unistd.h"
#include <vector>
#include <mutex>

#define PLOT_SCALE 0.5
#define PLOT_POINT_SIZE 30.0

using namespace std;

namespace PLOT {

    vector<vector<GLfloat>> plot_data;
    vector<vector<GLfloat>> plot_data_white;
    vector<vector<GLfloat>> plot_data_red;
    vector<vector<GLfloat>> plot_data_green;
    //mutex data_mutex;

void init(void){
    //背景色
    glClearColor(0.0, 0.0, 0.0, 1.0);
    //将控制点坐标映射为曲线坐标
    //参数1：GL_MAP1_VERTEX_3，3维点坐标
    //参数2和3：控制参数t或u的取值范围[0, 1]
    //参数4：曲线内插值点间的步长3————3维坐标
    //参数5：曲线间的补偿为顶点数4个————总步长为12
    //参数6：控制点二维数组首元素地址

    //注意: 若是在这里设置了相关参数，后续对ctrlpoints内容更改曲线不变
    //glMap1f(GL_MAP1_VERTEX_3, 0.0, 1.0, 3, 4, &ctrlpoints[0][0]);

    //打开开关——允许3维坐标控制点到参数点转换开关
    glEnable(GL_MAP1_VERTEX_3);
    glShadeModel(GL_FLAT);

    //代码开关2：去掉本注释，可启用反走样
    /*
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);  //允许直线反走样
    glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);  // Antialias the lines
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    */
}

//3D空间中绘制2D效果，采用正交投影
void reshape(GLsizei w, GLsizei h)
{
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (w <= h)
        glOrtho(-5.0, 5.0, -5.0*(GLfloat)h / (GLfloat)w, 5.0*(GLfloat)h / (GLfloat)w, -5.0, 5.0);
    else
        glOrtho(-5.0*(GLfloat)w / (GLfloat)h, 5.0*(GLfloat)w / (GLfloat)h, -5.0, 5.0, -5.0, 5.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}


void keyboard(unsigned char key, int x, int y)
{
    if(key=='p'){
        //
    }
    else if (key==27) {   //ESC键
        exit(0);
    }
    return;

//        switch (key)
//        {
//        case 'p':{
//            cout<<"PPPPPPPP"<<endl;
//            pause_mutex.lock();
//        };
//        case 'g':{
//            cout<<"GGGGGGGG"<<endl;
//            pause_mutex.unlock();
//        };
//        case 27:{   //ESC键
//            cout<<"KKKKKKK"<<endl;
//            exit(0);
//            break;
//        };
//        default:
//            break;
//        }
}





void display(void)
{
    //data_mutex.lock();

    glClear(GL_COLOR_BUFFER_BIT);
    /* 显示控制点 */
    glLoadIdentity();

    glTranslatef(-3.0f, 3.0f, 0.0f); //Axis Ofset Seting !!!!!! --Daike Kang
    glPointSize(PLOT_POINT_SIZE);

    glBegin(GL_POINTS);
    glColor3f(1.0, 1.0, 1.0);  //Color is here !!!!!!!!!!!! --Daike Kang
    for (size_t i = 0; i < plot_data_white.size(); i++){
        glVertex3fv(&plot_data_white[i][0]);
     }
    glEnd();

    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);  //Color is here !!!!!!!!!!!! --Daike Kang
    for (size_t i = 0; i < plot_data_red.size(); i++){
        glVertex3fv(&plot_data_red[i][0]);
     }
    glEnd();

    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 0.0);  //Color is here !!!!!!!!!!!! --Daike Kang
    for (size_t i = 0; i < plot_data_green.size(); i++){
        glVertex3fv(&plot_data_green[i][0]);
     }
    glEnd();



    //glColor3f(1.0f, 0.0f, 0.0f);

    glRasterPos2f(0.0f, 0.0f);


    glutSwapBuffers();
    //data_mutex.unlock();
}



void Set_data(vector<vector<float>> _path,vector<vector<int>> _map){
    //*************Set data*************//

    //plot_data_white.clear();
    //plot_data_red.clear();

    float plot_scale=PLOT_SCALE;
    for (int i=0;i<_map.size();i++) {
        for (int j=0;j<_map[0].size();j++){
            if(_map[i][j]==0){
                GLfloat tmp_x=j;
                GLfloat tmp_y=-i;
                plot_data_white.push_back({tmp_x*plot_scale,tmp_y*plot_scale,0});
            }
            else {
                GLfloat tmp_x=j;
                GLfloat tmp_y=-i;
                plot_data_red.push_back({tmp_x*plot_scale,tmp_y*plot_scale,0});
            }
        }
    }

    //plot_data_green.clear();

    for (size_t i=0;i<_path.size();i++) {
        GLfloat tmp_x=_path[i][0];
        GLfloat tmp_y=-_path[i][1];
        plot_data_green.push_back({tmp_x*plot_scale,tmp_y*plot_scale,0});
    }

}



void Plot_data(int &argc, char** argv){



    //data_mutex.unlock();

    //*************************//
    srand((unsigned int)time(0));
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);//使用双缓存模式和深度缓存
    glutInitWindowSize(800, 800);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Plot");
    init();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(display);//设置空闲时调用的函数
    glutMainLoop();


}


//vector<vector<GLfloat>> get_linear_point(float _x1,float _y1,float _x2,float _y2,float _plot_scale,float _sample_scale){
//    vector<vector<GLfloat>> res;
//    float num=pow(pow(_x1-_x2,2)+pow(_y1-_y2,2),0.5)/_sample_scale;
//    float deta_x=(_x2-_x1)/num;
//    float deta_y=(_y2-_y1)/num;
//    res.push_back({_x1,_y1,0.0});
//    for(float i=0;i<=num ;i+=1.0){
//        float tmp_x=res.back()[0]+deta_x;
//        float tmp_y=res.back()[1]+deta_y;
//        res.push_back({tmp_x,tmp_y,0.0});
//    }
//    for(size_t i=0;i<res.size();i++){
//        vector<GLfloat> tmp=res[i];
//        res[i][0]=-_plot_scale*tmp[1];
//        res[i][1]=_plot_scale*tmp[0];
//    }
//    return res;
//}



}
