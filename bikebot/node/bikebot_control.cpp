#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <Python.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <time.h>
#include <sys/time.h>
#include "bikebot_control.h"
#include "leg_controller.h"
#include "bicycle_controller.h"

// 必须的头文件
#include <pthread.h>
#include <mutex>
#include <thread>
// 使用ros
#include <ros/ros.h> // 包含ROS的头文件
#include <serial/serial.h>
//RT
#include <limits.h>
#include <sched.h>
#include <sys/mman.h>
#include "bikebot_timer.h"
#include <sys/timerfd.h>
#include <signal.h>


using namespace std;

int socket0;
int socket1;

serial::Serial ser0;
serial::Serial ser1;

//全局变量
char ch[64] = {0};
Angle angle[2];
Angle angleV[2];
Position p0[2];
Position v0[2];
Position pdes[2];
Position vdes[2];
Position nextpos[2];
Angle nextangle[2];
Angle L_angle;
Angle R_angle;
int Iter = 400;//设置2000ms

//共享全局变量
//leg_data
CANMessage cbmsg[6];
//bike_data
Bike_cb bike_cb;
//imu_data
double varphi, dvarphi, psi, pitch, dpitch, dpsi;
double acc[3];

//全局变量
Leg_state leg_state;
Bike_state bike_state;
CANMessage L_msgs[3];
CANMessage R_msgs[3];
Leg_command leg_cmd;
Bike_command bike_cmd;
Posdiff poserror;
float bike_timestamp;
float leg_timestamp;

//if can start
int bike_begin = 0;
int can0_recieved = 0;
int can1_recieved = 0;
int safety_det_begin = 0;
static int shut_down;

//leg and bike communicate
int eic_able = 0;
int impact_happen = 0;
int stance_end = 0;
int stance_begin = 0;
int leg_down = 0;
int leg_up = 0;
int include_u = 0;
int eic_unable = 0;
float optm_tau = 0;

//optimize
double optm_result;
int optm_begin = 0;

static void sighand(int sig)
{
	shut_down = 1;
}

// Can0_thread
void* Can0_thread(void* args)
{
    cout << "Can0_thread" << endl;
    int nbytes;
    struct can_frame frame;
    while(!shut_down)
    {
        nbytes = read(socket0, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
                perror("can raw socket0 read");
                continue;
        }
        /* paranoid check ... */
        if (nbytes < sizeof(struct can_frame)) {
                fprintf(stderr, "read: incomplete CAN0 frame\n");
                continue;
        }

        //读取指定ID的数据
        if( frame.data[0] == 1){
            for(int i=0;i<6;i++){
                cbmsg[0].data[i] = frame.data[i];
            }
        }
        else if( frame.data[0] == 2){
            for(int i=0;i<6;i++){
                cbmsg[1].data[i] = frame.data[i];
            }
        }
        else{
            if( frame.data[0] == 3){
                for(int i=0;i<6;i++){
                    cbmsg[2].data[i] = frame.data[i];
                }		
            }
        }

        can0_recieved = 1;

    }

    return NULL;
}
// Can1_thread
void* Can1_thread(void* args)
{
    cout << "Can1_thread" << endl;
    int nbytes;
    struct can_frame frame;
    while(!shut_down)
    {
        nbytes = read(socket1, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
                perror("can raw socket1 read");
                continue;
        }
        /* paranoid check ... */
        if (nbytes < sizeof(struct can_frame)) {
                fprintf(stderr, "read: incomplete CAN1 frame\n");
                continue;
        }

        //读取指定ID的数据
        if( frame.data[0] == 1+bias){
            for(int i=0;i<6;i++){
                cbmsg[3].data[i] = frame.data[i];
            }
        }
        else if( frame.data[0] == 2+bias){
            for(int i=0;i<6;i++){
                cbmsg[4].data[i] = frame.data[i];
            }		
        }
        else{
            if( frame.data[0] == 3+bias){
                for(int i=0;i<6;i++){
                    cbmsg[5].data[i] = frame.data[i];
                }	
            }
        }

        //can_msg recieved
        can1_recieved = 1;

    }

    return NULL;
}

//imu_thread
void* imu_thread(void* args)
{
    cout << "imu_thread" << endl;
    //定义线程局部变量
    unsigned char buf[1]; //定义字符串长度
    unsigned char Rxbuf[12];
    unsigned char Rxcnt = 0;

    struct SGyro
    {
        short w[3];
        short T;
    };
    struct SAngle
    {
        short Angle[3];
        short T;
    };
    struct SAcc
    {
        short a[3];
        short T;
    };
    struct SGyro stcGyro;
    struct SAngle stcAngle;
    struct SAcc stcAcc;

    try{ 
        //设置串口属性，并打开串口 
        ser0.setPort("/dev/ttyTHS2"); 
        ser0.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser0.setTimeout(to); 
        ser0.open(); 
    } 
    catch (serial::IOException& e){ 
        printf("Unable to open port ");  
    } 
 
    if(ser0.isOpen()){ 
        printf("Serial Port initialized\r\n"); 
    } 
    else{ 
        printf("Serial Port initialize error"); 
    } 

    while(!shut_down)
    {
        //读串口数据
        Rxbuf[Rxcnt++] = buf[0];
        if(Rxbuf[0] != 0x55){ //数据头不对
            Rxcnt = 0;
            try{
                ser0.read(buf, sizeof(buf));
            }
            catch (exception& e){
                cout << "imu_sp1" << endl;
            } 
            continue;
        }
        if(Rxcnt < 11){ //数据不满11个
            try{
                ser0.read(buf, sizeof(buf)); 
            }
            catch (exception& e){
                cout << "imu_sp2" << endl;
            } 
            continue;
        }
        else{
            switch(Rxbuf[1])
            {
                case 0x51:
                    // g
                    memcpy(&stcAcc,&Rxbuf[2],8);
                    acc[0] = (double)stcAcc.a[0]/32768*4;
                    acc[1] = (double)stcAcc.a[1]/32768*4;
                    acc[2] = (double)stcAcc.a[2]/32768*4;
                    break;

                case 0x52:
                    // rad/s
                    memcpy(&stcGyro,&Rxbuf[2],8);
                    dvarphi = (double)stcGyro.w[0]*8.725/32768;
                    dpitch = (double)stcGyro.w[1]*8.725/32768;
                    dpsi = (double)stcGyro.w[2]*8.725/32768;
                    break;
                
                case 0x53:
                    // rad
                    memcpy(&stcAngle,&Rxbuf[2],8);
                    varphi = (double)stcAngle.Angle[0]/32768*PI;
                    pitch = (double)stcAngle.Angle[1]/32768*PI;
                    psi = (double)stcAngle.Angle[2]/32768*PI;
                    break;
            }
            Rxcnt = 0;//清空缓存区

            try{
                ser0.read(buf, sizeof(buf));
            }
            catch (exception& e){
                cout << "imu_sp" << endl;
            }     
        }
    }
    return NULL;
}

//bikemsg_thread
void* bikemsg_thread(void* args)
{
    cout << "bikemsg_thread" << endl;
    //定义线程局部变量
    unsigned char Rxbuf[14];
    unsigned char Rxcnt = 0;
    unsigned char buf[1];

    try{ 
        //设置串口属性，并打开串口 
        ser1.setPort("/dev/bike"); 
        ser1.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser1.setTimeout(to); 
        ser1.open(); 
    } 
    catch (serial::IOException& e){ 
        printf("Unable to open port ");  
    } 
 
    if(ser1.isOpen()){ 
        printf("Serial Port initialized\r\n"); 
    } 
    else{ 
        printf("Serial Port initialize error"); 
    } 

    while(!shut_down)
    {
        //读串口数据
        Rxbuf[Rxcnt++] = buf[0];
        if(Rxbuf[0] != 0x55){ //数据头不对
            Rxcnt = 0;
            try{
                ser1.read(buf, sizeof(buf));
            }
            catch (exception& e){
                cout << "bikemsg_sp1" << endl;
            } 
            continue;
        }
        if(Rxcnt < 14){ //数据不满11个
            try{
                ser1.read(buf, sizeof(buf)); 
            }
            catch (exception& e){
                cout << "bikemsg_sp2" << endl;
            } 
            continue;
        }
        else{
            // for(int i(0);i<14;i++) printf("%02x",Rxbuf[i]);
            // printf("\r\n");
            if(Rxbuf[13]==0x50){
                bike_begin = 1;
            }
            else{
                //修改全局变量
                memcpy(&bike_cb,&Rxbuf[1],12);

                Rxcnt = 0;//清空缓存区

                try{
                    ser1.read(buf, sizeof(buf)); 
                }
                catch (exception& e){
                    cout << "bikemsg_sp" << endl;
                }
            }
  
        }
    }
    return NULL;
}

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
    float _period = 0.01;
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
            double req_varphi = varphi;
            double req_dvarphi = dvarphi;

            PyObject* pParams = Py_BuildValue("(dd)",req_varphi,req_dvarphi);   
            PyObject* pRet = PyObject_CallObject(pFunc, pParams);//调用函数
            PyArg_Parse(pRet, "d", &optm_result);//转换返回类型
            // cout << "resresresresresresresresresresres:" << optm_result << endl;//输出结果
        } 
        /********************** running end **********************/

        _lastRuntime = (float)t.getSeconds();
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        // cout<<"lastRuntime:"<<_lastRuntime<<endl;
        // cout<<"lastPeriodTime:"<<_lastPeriodTime<<endl;
        
        /// 延时
        int m = read(timerFd, &missed, sizeof(missed));
        (void)m;

    }
    Py_Finalize();
    return NULL;
}

//safety_thread
void* safety_thread(void* args)
{
    cout << "safety_thread" << endl;

    while(safety_det_begin == 0);

    //初始化定时器
    float _period = 0.05;
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

    int print_flag = 1;

    while(!shut_down)
    {
        //计时
        _lastPeriodTime = (float)t.getSeconds();
        t.start();

        //angle_limit
        if(leg_state.cbdata[0].p < -90.0*PI/180.0 || leg_state.cbdata[0].p > 30.0*PI/180.0) {
            reset_motors();
            if(print_flag==1) {
                cout<<"angle0 error"<<endl;
                print_flag = 0;
            }  
            // shut_down = 1;
        }
        if(leg_state.cbdata[1].p < -30.0*PI/180.0 || leg_state.cbdata[1].p > 180.0*PI/180.0) {
            reset_motors();
            if(print_flag==1) {
                cout<<"angle1 error"<<endl;
                print_flag = 0;
            } 
            // shut_down = 1;
        }
        if(leg_state.cbdata[2].p > 30.0*PI/180.0) {
            reset_motors();
            if(print_flag==1) {
                cout<<"angle2 error"<<endl;
                print_flag = 0;
            } 
            // shut_down = 1;
        }
        if(leg_state.cbdata[3].p < -30.0*PI/180.0 || leg_state.cbdata[3].p > 90.0*PI/180.0) {
            reset_motors();
            if(print_flag==1) {
                cout<<"angle3 error"<<endl;
                print_flag = 0;
            } 
            // shut_down = 1;
        }
        if(leg_state.cbdata[4].p < -180.0*PI/180.0 || leg_state.cbdata[4].p > 30.0*PI/180.0) {
            reset_motors();
            if(print_flag==1) {
                cout<<"angle4 error"<<endl;
                print_flag = 0;
            } 
            // shut_down = 1;
        }
        if(leg_state.cbdata[5].p < -30.0*PI/180.0) {
            reset_motors();
            if(print_flag==1) {
                cout<<"angle5 error"<<endl;
                print_flag = 0;
            } 
            // shut_down = 1;
        }
        //torque_limit
        for(int i(0);i<6;i++){
            if(leg_state.cbdata[i].t < -24.0 || leg_state.cbdata[i].t > 24.0) {
                reset_motors();
                if(print_flag==1) {
                    cout<<"error: torque"<<i<<endl;
                    print_flag = 0;
                } 
                // shut_down = 1;
            }
        }


        _lastRuntime = (float)t.getSeconds();
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        // cout<<"maxPeriod:"<<_maxPeriod<<endl;
        // cout<<"maxRuntime:"<<_maxRuntime<<endl;

        //延时
        int m = read(timerFd, &missed, sizeof(missed));
        (void)m;
    }

    return NULL;
}

//bikecontrol_thread
void* bikecontrol_thread(void* args)
{

    cout<<"bike controller start!"<<endl;

    //初始化控制器
    bikestate_update();
    bicycle_controller b_controller(&bike_state,ch);

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
        bike_timestamp = _lastPeriodTime;
        t.start();

        /********************** running begin **********************/
        //更新机器人状态
        bikestate_update();
        //bike控制 50HZ 
        float leg_torque = 0;
        if(stance_end == 1){
            eic_able = 1;
            stance_end = 0;
            cout<<"eic_able"<<endl;
        } 
        if(stance_begin == 1){
            include_u = 1;
            stance_begin = 0;
            
            // cout<<"include_u:"<<leg_torque<<endl;
        }
        if(leg_down == 1){
            eic_unable = 1;
            leg_down = 0;
            leg_torque = optm_tau;
            cout<<"eic_unable"<<endl;
        }
        b_controller.get_action(&bike_cmd,eic_able,include_u,eic_unable,leg_torque);
        eic_able = 0;
        include_u = 0;
        eic_unable = 0;
        leg_torque = 0;
        //驱动bike执行器
        drive_bike(bike_cmd);
        
        /********************** running end **********************/

        _lastRuntime = (float)t.getSeconds();
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        // cout<<"maxPeriod:"<<_maxPeriod<<endl;
        // cout<<"maxRuntime:"<<_maxRuntime<<endl;
        
        /// 延时
        int m = read(timerFd, &missed, sizeof(missed));
        (void)m;

    }
    return NULL;
}

//legcontrol_thread
void* legcontrol_thread(void* args)
{

    cout<<"leg controller start!"<<endl;

    //初始化控制器
    leg_controller l_controller(&leg_state);

    //初始化定时器
    float _period = 0.001;
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
        leg_timestamp = _lastPeriodTime;
        t.start();

        /********************** running begin **********************/
        //更新数据
        legstate_update();

        //leg控制  1000HZ
        if(l_controller.get_action(&leg_cmd,1)==1){
            leg_up = 1;
        }
        if(l_controller.get_stance_ifend()==1){
            stance_end = 1;
        }
        if(l_controller.get_stance_ifbegin()==1){
            stance_begin = 1;
            
        }
        if(l_controller.get_swdown_ifbegin()==1){
            leg_down = 1;
            optm_tau = leg_state.optmT;
        }

        poserror = l_controller.get_error();
        //驱动leg执行器
        motor_control(leg_cmd);
        
        //打印数据
        // for(int j=0;j<6;j++){
        //     for(int i=0;i<6;i++){
        //         printf("%x ",cbmsg[j].data[i]);
        //     }
        //     printf("\n");
        // }
        // for(int i=0;i<6;i++){
        //     cb_Inf(leg_state.cbdata+i);
        // }
        // printf("\r\n");

        /********************** running end **********************/

        _lastRuntime = (float)t.getSeconds();
        // cout<<"lastPeriodTime:"<<_lastPeriodTime<<endl;
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        // cout<<"maxPeriod:"<<_maxPeriod<<endl;
        // cout<<"maxRuntime:"<<_maxRuntime<<endl;

        // cout<<leg_state.varphi/ PI * 180.0<<endl;
        
        /// 延时
        int m = read(timerFd, &missed, sizeof(missed));
        (void)m;

    }
    return NULL;
}

//record_thread
void* record_thread(void* args)
{
    cout << "record_thread" << endl;

    //生成数据编号
    char result[100] = {0};
    sprintf(result, "/home/hxy/0318/dataFile%s.txt", ch);
    ofstream dataFile;
    dataFile.open(result, ofstream::app);

    //初始化定时器
    float _period = 0.002;
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

        /*** record the data ***/
        // 朝TXT文档中写入数据
        dataFile <<leg_cmd.torque[0] << ", " << leg_cmd.torque[1] << ", " << leg_cmd.torque[2] << ", " 
                << leg_cmd.torque[3]<< ", " << leg_cmd.torque[4] << ", " << leg_cmd.torque[5] << ", " 
                << leg_state.cbdata[0].p << ", " << leg_state.cbdata[1].p << ", " << leg_state.cbdata[2].p << ", " 
                << leg_state.cbdata[3].p << ", " << leg_state.cbdata[4].p << ", " << leg_state.cbdata[5].p << ", " 
                << leg_state.cbdata[0].v << ", " << leg_state.cbdata[1].v << ", " << leg_state.cbdata[2].v << ", " 
                << leg_state.cbdata[3].v << ", " << leg_state.cbdata[4].v << ", " << leg_state.cbdata[5].v << ", "
                << leg_state.cbdata[0].t << ", " << leg_state.cbdata[1].t << ", " << leg_state.cbdata[2].t << ", " 
                << leg_state.cbdata[3].t << ", " << leg_state.cbdata[4].t << ", " << leg_state.cbdata[5].t << ", " 
                << leg_state.varphi << ", "<< poserror.error[0] << ", " << poserror.error[1] << ", " << poserror.error[2] << ", " 
                << poserror.error[3] << ", " << poserror.error[4] << ", " << poserror.error[5] << ", " 
                << leg_state.dvarphi << ", "<< leg_state.accx << ", "<< optm_result << ", " << bike_timestamp << ", " << leg_timestamp 
                << std::endl;


        _lastRuntime = (float)t.getSeconds();
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        // cout<<"maxPeriod:"<<_maxPeriod<<endl;
        // cout<<"maxRuntime:"<<_maxRuntime<<endl;

        //延时
        int m = read(timerFd, &missed, sizeof(missed));
        (void)m;
    }
    dataFile.close();
    return NULL;
}

int main(int argc, char **argv)
{
    cout << "main_thread" << endl;

    //初始化can0，can1
    CAN_init();
    
    //开启多线程
    thread_setup();

    //启动电机
    setup_motors();

    //判断leg硬件是否就绪
    // while(can0_recieved == 0 || can1_recieved == 0);

    sleep(1);
    legstate_update();
    safety_det_begin = 1;

    //腿初始化
    setpoint(0.16,0.08,-0.12);
    setpoint1(0.04,0.13,-0.34);
    cout<<"leg init finished!"<<endl;

    legstate_update();
    for(int i=0;i<6;i++){
        cb_Inf(leg_state.cbdata+i);
    }
    printf("\r\n");

    //判断bike硬件是否就绪
    while(bike_begin == 0);
    time_t tt = time(NULL);
    strftime(ch, sizeof(ch) - 1, "%H%M", localtime(&tt));

    setpoint(0.16,0.08,-0.12);
    bike_cmd.speed = 0;
    drive_bike(bike_cmd);
    control_threadcreate();    
    optm_begin = 1;

    signal(SIGINT, sighand);

    while(!shut_down){

    }

    reset_motors();
    cout<<"down!"<<endl;

    /* Join the thread and wait until it is done */
    // pthread_t thread;
    // int ret = pthread_join(thread, NULL);
    // if (ret)  printf("join pthread failed: %m\n");

    return 0;

}

void thread_setup(void){
    //定义线程的 id 变量，多个变量使用数组
    pthread_t tids[6];
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
    //参数依次是：创建的线程id，线程参数，调用的函数，传入的函数参数
    ret = pthread_create(&tids[0], &attr, Can0_thread, NULL);
    if (ret != 0){
        cout << "pthread_create0 error: error_code=" << ret << endl;
    }

    ret = pthread_create(&tids[1], &attr, Can1_thread, NULL);
    if (ret != 0){
        cout << "pthread_create1 error: error_code=" << ret << endl;
    }

    ret = pthread_create(&tids[2], NULL, imu_thread, NULL);
    if (ret != 0){
        cout << "pthread_create2 error: error_code=" << ret << endl;
    }

    ret = pthread_create(&tids[3], NULL, bikemsg_thread, NULL);
    if (ret != 0){
        cout << "pthread_create3 error: error_code=" << ret << endl;
    }

    ret = pthread_create(&tids[4], NULL, safety_thread, NULL);
    if (ret != 0){
        cout << "pthread_create4 error: error_code=" << ret << endl;
    }

    param.sched_priority = 47;
    ret = pthread_attr_setschedparam(&attr, &param);
    pthread_attr_getschedparam(&attr, &param);
    cout<<"usepy_thread prior:"<<param.sched_priority<<endl;

    ret = pthread_create(&tids[5], &attr, usepy_thread, NULL);
    if (ret != 0){
        cout << "pthread_create5 error: error_code=" << ret << endl;
    }

}

void control_threadcreate(void){
    struct sched_param param;
    pthread_attr_t attr;
    pthread_t tids[3];

    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            printf("mlockall failed: %m\n");
    }
    /* Initialize pthread attributes (default values) */
    int ret = pthread_attr_init(&attr);
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
    cout<<"bikec_thread prior:"<<param.sched_priority<<endl;
    ret = pthread_create(&tids[0], &attr, bikecontrol_thread, NULL);
    if (ret != 0){
        cout << "ctr_pthread_create0 error: error_code=" << ret << endl;
    }

    param.sched_priority = 99;
    ret = pthread_attr_setschedparam(&attr, &param);
    pthread_attr_getschedparam(&attr, &param);
    cout<<"legc_thread prior:"<<param.sched_priority<<endl;
    ret = pthread_create(&tids[1], &attr, legcontrol_thread, NULL);
    if (ret != 0){
        cout << "ctr_pthread_create1 error: error_code=" << ret << endl;
    }

    param.sched_priority = 48;
    ret = pthread_attr_setschedparam(&attr, &param);
    pthread_attr_getschedparam(&attr, &param);
    cout<<"record_thread prior:"<<param.sched_priority<<endl;
    ret = pthread_create(&tids[2], &attr, record_thread, NULL);
    if (ret != 0){
        cout << "record_pthread error: error_code=" << ret << endl;
    }
}

void legstate_update(){
    cb_trans(cbmsg,leg_state.cbdata);
    leg_state.varphi = varphi;
    leg_state.dvarphi = dvarphi;
    leg_state.accx = acc[0];
    leg_state.optmT = optm_result;
}

void bikestate_update(){

    bike_state.bike_msg = bike_cb;
    bike_state.varphi = varphi;
    bike_state.dvarphi = dvarphi;
    bike_state.psi = psi;
    bike_state.pitch = pitch;
    bike_state.dpitch = dpitch;
    bike_state.dpsi = dpsi;
    bike_state.accx = acc[0];
    bike_state.accy = acc[1];
    bike_state.accz = acc[2];
}

void CAN_init(){
    struct sockaddr_can addr0;
	struct ifreq ifr0;
    struct sockaddr_can addr1;
	struct ifreq ifr1;

    if ((socket0 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
		perror("Error while opening socket0");
	}
    const char *ifname0 = "can0";
	strcpy(ifr0.ifr_name, ifname0);
	ioctl(socket0, SIOCGIFINDEX, &ifr0);
    addr0.can_family  = AF_CAN;
	addr0.can_ifindex = ifr0.ifr_ifindex;
    if (bind(socket0, (struct sockaddr *)&addr0, sizeof(addr0)) == -1) {
		perror("Error in socket0 bind");
	}

    if ((socket1 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
		perror("Error while opening socket1");
	}
    const char *ifname1 = "can1";
	strcpy(ifr1.ifr_name, ifname1);
	ioctl(socket1, SIOCGIFINDEX, &ifr1);
    addr1.can_family  = AF_CAN;
	addr1.can_ifindex = ifr1.ifr_ifindex;
    if (bind(socket1, (struct sockaddr *)&addr1, sizeof(addr1)) == -1) {
		perror("Error in socket1 bind");
	}
}

void can0_tx(uint8_t tdata[],uint8_t id){
	struct can_frame frame;
    frame.can_id  = id;
    frame.can_dlc = 8;
    memcpy(frame.data,tdata,8);
    write(socket0, &frame, sizeof(struct can_frame));
}

void can1_tx(uint8_t tdata[],uint8_t id){
	struct can_frame frame;
    frame.can_id  = id;
    frame.can_dlc = 8;
    memcpy(frame.data,tdata,8);
    write(socket1, &frame, sizeof(struct can_frame));
}

void reset_motors(){
    for(int i=1;i<=3;i++){
		can0_tx(reset,i);
		can1_tx(reset,i+bias);
        //sleep
        Sleep_us(300);
	}
}

void setup_motors(){
    for(int i=1;i<=3;i++){
        can0_tx(set_foc,i);
        can1_tx(set_foc,i+bias);
        //sleep
        Sleep_us(300);

	}
}

void motor_control(Leg_command leg_cmd){
    int i;
    for(i=0;i<2;i++){
        cmd_transfer(i+1,&L_msgs[i],0,0,0,0,leg_cmd.torque[i]);
        can0_tx(L_msgs[i].data,i+1);
        cmd_transfer(i+4,&R_msgs[i],0,0,0,0,leg_cmd.torque[i+3]);
        can1_tx(R_msgs[i].data,i+1+bias);
        //延时
        Sleep_us(300);
	}
    i = 2;
    cmd_transfer(i+1,&L_msgs[i],0,0,0,0,leg_cmd.torque[i]);
    can0_tx(L_msgs[i].data,i+1);
    cmd_transfer(i+4,&R_msgs[i],0,0,0,0,leg_cmd.torque[i+3]);
    can1_tx(R_msgs[i].data,i+1+bias);
}

void drive_bike(Bike_command msg){
    //写串口数据
    unsigned char Txbuf[9];
    Txbuf[0] = 0x55;
    memcpy(&Txbuf[1],&msg,sizeof(msg));
    try{
        ser1.write(Txbuf, sizeof(Txbuf));
    }
    catch (exception& e){
        cout << "bikedrive_sp" << endl;
    }
}

void setpoint(float x,float y,float z){
    /***  进行点位控制  ***/
    CBData cbdata[6];
    //获取当前关节位置和速度
    cb_trans(cbmsg,cbdata);
    angle[0].q[0] = cbdata[0].p;
    angle[0].q[1] = cbdata[1].p;
    angle[0].q[2] = cbdata[2].p;
    angleV[0].q[0] = cbdata[0].v;
    angleV[0].q[1] = cbdata[1].v;
    angleV[0].q[2] = cbdata[2].v;
    angle[1].q[0] = cbdata[3].p;
    angle[1].q[1] = cbdata[4].p;
    angle[1].q[2] = cbdata[5].p;
    angleV[1].q[0] = cbdata[3].v;
    angleV[1].q[1] = cbdata[4].v;
    angleV[1].q[2] = cbdata[5].v;

    //求正运动学，计算当前末端位置；求雅可比，计算当前末端速度
    Kinematics(&angle[0],&p0[0],0);
    Kinematics(&angle[1],&p0[1],1);
    v0[0].x = 0;
    v0[0].y = 0;
    v0[0].z = 0;
    v0[1].x = 0;
    v0[1].y = 0;
    v0[1].z = 0;
//		pos_Inf(&p0[0]);
//		pos_Inf(&p0[1]);
    
    //输入期望位置和速度
    pdes[0].x = x;
    pdes[0].y = y;
    pdes[0].z = z;
    vdes[0].x = 0;
    vdes[0].y = 0;
    vdes[0].z = 0;   
    pdes[1].x = x;
    pdes[1].y = -y;
    pdes[1].z = z;
    vdes[1].x = 0;
    vdes[1].y = 0;
    vdes[1].z = 0;
    
    //三次样条插补，逆运动学，pd控制
    init_chabu(&pdes[0],&vdes[0],&p0[0],&v0[0],Iter,0);
    init_chabu(&pdes[1],&vdes[1],&p0[1],&v0[1],Iter,1);
    for(int j=0;j<Iter;j++){	
        chabu(&nextpos[0],j+1,0);
        Inv_kinematics(&L_angle,&nextpos[0],0);
        chabu(&nextpos[1],j+1,1);
        Inv_kinematics(&R_angle,&nextpos[1],1);
        for(int i=0;i<3;i++){
            cmd_transfer(i+1,&L_msgs[i],L_angle.q[i],0,8,0.2,0);
            can0_tx(L_msgs[i].data,i+1);
            cmd_transfer(i+4,&R_msgs[i],R_angle.q[i],0,8,0.2,0);
            can1_tx(R_msgs[i].data,i+1+bias);
            Sleep_us(300);
        }
        // cout<<L_angle.q[0]*180/PI<<","<<L_angle.q[1]*180/PI<<","<<L_angle.q[2]*180/PI<<","
        //     <<R_angle.q[0]*180/PI<<","<<R_angle.q[1]*180/PI<<","<<R_angle.q[2]*180/PI<<endl;
    }
}

void setpoint1(float x,float y,float z){
    /***  进行点位控制  ***/
    CBData cbdata[6];
    //获取当前关节位置和速度
    cb_trans(cbmsg,cbdata);
    angle[0].q[0] = cbdata[0].p;
    angle[0].q[1] = cbdata[1].p;
    angle[0].q[2] = cbdata[2].p;
    angleV[0].q[0] = cbdata[0].v;
    angleV[0].q[1] = cbdata[1].v;
    angleV[0].q[2] = cbdata[2].v;
    angle[1].q[0] = cbdata[3].p;
    angle[1].q[1] = cbdata[4].p;
    angle[1].q[2] = cbdata[5].p;
    angleV[1].q[0] = cbdata[3].v;
    angleV[1].q[1] = cbdata[4].v;
    angleV[1].q[2] = cbdata[5].v;

    //求正运动学，计算当前末端位置；求雅可比，计算当前末端速度
    Kinematics(&angle[0],&p0[0],0);
    Kinematics(&angle[1],&p0[1],1);
    v0[0].x = 0;
    v0[0].y = 0;
    v0[0].z = 0;
    v0[1].x = 0;
    v0[1].y = 0;
    v0[1].z = 0;
//		pos_Inf(&p0[0]);
//		pos_Inf(&p0[1]);
    
    //输入期望位置和速度
    pdes[0].x = x;
    pdes[0].y = y;
    pdes[0].z = z;
    vdes[0].x = 0;
    vdes[0].y = 0;
    vdes[0].z = 0;   
    pdes[1].x = x;
    pdes[1].y = -y;
    pdes[1].z = z;
    vdes[1].x = 0;
    vdes[1].y = 0;
    vdes[1].z = 0;
    
    //三次样条插补，逆运动学，pd控制
    init_chabu(&pdes[0],&vdes[0],&p0[0],&v0[0],Iter,0);
    init_chabu(&pdes[1],&vdes[1],&p0[1],&v0[1],Iter,1);
    for(int j=0;j<Iter;j++){	
        chabu(&nextpos[0],j+1,0);
        Inv_kinematics(&L_angle,&nextpos[0],0);
        chabu(&nextpos[1],j+1,1);
        Inv_kinematics(&R_angle,&nextpos[1],1);
        for(int i=0;i<3;i++){
            cmd_transfer(i+1,&L_msgs[i],L_angle.q[i],0,20,0.2,0);
            can0_tx(L_msgs[i].data,i+1);
            cmd_transfer(i+4,&R_msgs[i],R_angle.q[i],0,20,0.2,0);
            can1_tx(R_msgs[i].data,i+1+bias);
            Sleep_us(300);
        }
        // cout<<L_angle.q[0]*180/PI<<","<<L_angle.q[1]*180/PI<<","<<L_angle.q[2]*180/PI<<","
        //     <<R_angle.q[0]*180/PI<<","<<R_angle.q[1]*180/PI<<","<<R_angle.q[2]*180/PI<<endl;
    }
}

void Sleep_us(int us)
{
    struct timeval delay;
    delay.tv_sec = 0;
    delay.tv_usec = us;
    select(0, NULL, NULL, NULL, &delay);
}