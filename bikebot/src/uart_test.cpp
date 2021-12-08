#include <Python.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ros/ros.h> // 包含ROS的头文件
//RT
#include <limits.h>
#include <sched.h>
#include <sys/mman.h>
#include "bikebot_timer.h"
#include <sys/timerfd.h>
#include <signal.h>
// 必须的头文件
#include <pthread.h>
#include <mutex>
#include <thread>

using namespace std;

typedef struct{
	double varphi;
	double psi;
	double dvarphi;
	double dpsi;
	double T0;
    double y;
    double dx;
    double dy;
    double phi;
} Input;
Input inp;

double optm_result;
int optm_begin = 1;
int shut_down=0;

//usepy_thread
void* usepy_thread(void* args)
{
    //调用python
    Py_Initialize();
    if (!Py_IsInitialized())
    {
        printf("初始化失败！");
        return 0;
    }
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("import os");
    // PyRun_SimpleString("print(os.getcwd())");
    PyRun_SimpleString("sys.path.append('/home/hxy/catkin_ws/src/bikebot/node/')");//这一步很重要，修改Python路径
    PyObject* pModule = NULL;//声明变量
    PyObject* pFunc = NULL;// 声明变量
    pModule = PyImport_ImportModule("optimize");//这里是要调用的文件名hash_hmac.py
    if (pModule == NULL)
    {
        cout << "没找到" << endl;
    }
    pFunc = PyObject_GetAttrString(pModule, "calculate_Tau");//这里是要调用的函数名
    cout<<"optimizer begin"<<endl;
    //初始化定时器
    float _period = 0.02;
    auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
    int seconds = (int)_period;
    int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));
    Timer t;
    itimerspec timerSpec;
    timerSpec.it_interval.tv_sec = seconds;
    timerSpec.it_value.tv_sec = seconds;
    timerSpec.it_value.tv_nsec = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;
    timerfd_settime(timerFd, 0, &timerSpec, nullptr);
    unsigned long long missed = 0;
    float _lastRuntime = 0;
    float _lastPeriodTime = 0;
    float _maxPeriod = 0;
    float _maxRuntime = 0;

    while(!shut_down)
    {
        //计时
        _lastPeriodTime = (float)t.getSeconds();
        t.start();

        /********************** running begin **********************/
        if(optm_begin==1){
            double req_varphi = inp.varphi;
            double req_dvarphi = inp.dvarphi;

            PyObject* pParams = Py_BuildValue("(dd)",req_varphi,req_dvarphi);   
            PyObject* pRet = PyObject_CallObject(pFunc, pParams);//调用函数
            PyArg_Parse(pRet, "d", &optm_result);//转换返回类型
            cout << "resresresresresresresresresresres:" << optm_result << endl;//输出结果
        } 
        /********************** running end **********************/

        _lastRuntime = (float)t.getSeconds();
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        cout<<"lastRuntime:"<<_lastRuntime<<endl;
        // cout<<"lastPeriodTime:"<<_lastPeriodTime<<endl;
        
        /// 延时
        // int m = read(timerFd, &missed, sizeof(missed));
        // (void)m;

    }
    Py_Finalize();
    return NULL;
}

int main(int argc, char **argv){
    

    pthread_t tids[5];
    int ret;

    struct sched_param param;
    pthread_attr_t attr;

    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret) {
            printf("init pthread attributes failed\n");
    }
    /* Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
    }
    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret) {
            printf("pthread setschedpolicy failed\n");
    }
    param.sched_priority = 49;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
            printf("pthread setschedparam failed\n");
    }
    // /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
    }
    pthread_attr_getschedparam(&attr, &param);
    cout<<"can_thread prior:"<<param.sched_priority<<endl;

    ret = pthread_create(&tids[0], &attr, usepy_thread, NULL);
    if (ret != 0){
        cout << "pthread_create0 error: error_code=" << ret << endl;
    }
    while(!shut_down){

    }
    return 0;
}