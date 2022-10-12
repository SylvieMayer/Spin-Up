#include "system.hpp"
#include "sylib/env.hpp"
#include <iostream>
#include <stdint.h>

namespace sylib {
    void delay_until(std::uint32_t* const prev_time, const std::uint32_t delta){
        #ifdef SYLIB_ENV_PROS    
        pros::Task::delay_until(prev_time, delta);
        #elif defined(SYLIB_ENV_VEXCODE)
        std::uint32_t time = *prev_time + delta;
        vex::this_thread::sleep_until(time);  
        *prev_time = time;
        #endif
    }
    void delay(uint32_t delay){
        #ifdef SYLIB_ENV_PROS    
        pros::delay(delay);
        #elif defined(SYLIB_ENV_VEXCODE)
        vex::this_thread::sleep_for(delay);  
        #endif
    }
    uint32_t millis(){
        #ifdef SYLIB_ENV_PROS    
        return pros::millis();
        #elif defined(SYLIB_ENV_VEXCODE)
        return vex::timer::system();  
        #endif
    }
    uint64_t micros(){
        return vexSystemHighResTimeGet();
    }

    #ifndef SYLIB_PROS_ENV
    bool Mutex::take(){lock(); return 1;}
    bool Mutex::give(){unlock(); return 1;}
    #endif
    bool Mutex::try_lock_for(){return 0;}
    bool Mutex::try_lock_until(){return 0;}

    Mutex sylib_port_mutexes[V5_MAX_DEVICE_PORTS]; 

    SylibDaemon::SylibDaemon(): managerTask(sylib::Task(managerTaskFunction)) {} // 
    SylibDaemon::~SylibDaemon(){}
    SylibDaemon& SylibDaemon::getInstance(){
        mutex_lock _lock(mutex);
        static SylibDaemon instance;
	    return instance;
    }
    int SylibDaemon::createSubTask(sylib::Device  * objectPointerToSchedule){
        mutex_lock _lock(mutex);
        subTasksCreated++;
        livingSubTasks.push_back(objectPointerToSchedule);
        return subTasksCreated;
        return 1;
    }
    void SylibDaemon::removeSubTask(sylib::Device  *objectPointerToSchedule){
        mutex_lock _lock(mutex);
        livingSubTasks.erase(std::remove(livingSubTasks.begin(), livingSubTasks.begin(), objectPointerToSchedule));
    }
    void SylibDaemon::removeSubTaskByID(int idToKill){
        mutex_lock _lock(mutex);
        livingSubTasks.erase(std::remove_if(livingSubTasks.begin(), livingSubTasks.end(), [&](sylib::Device  * x){return (x->getSubTaskID() == idToKill);}));
    }
    int SylibDaemon::managerTaskFunction(){
	    printf("\nstarted sylib daemon %d\n", sylib::millis());
        
        /*
        
        THIS SECTION TAKES CARE OF DESYNCING SYLIB DAEMON WITH VEX BACKGROUND PROCESSING
        
        */

        // A 1ms loop will actually take around 1040 or 960 microseconds, always alternating. 
        // Over 3ms, the total length of time in micros should be either around 3040 or 960
        // Daemon needs to start on a cycle to be directly opposite of vexBackgroundProcessing()
        // vexBackgroundProcessing always runs after a short cycle, meaning the sylib daemon needs to start after a long cycle
        // Values offset by 20 to give room for error, the groupings are very tight so it shouldnt matter
        
        constexpr std::uint64_t SHORT_MICROS_CYCLE_LENGTH = 960 + 20;
        constexpr std::uint64_t LONG_MICROS_CYCLE_LENGTH = 1040 - 20;
        constexpr std::uint64_t AVERAGE_MICROS_CYCLE_LENGTH = 1000;
        constexpr std::uint64_t DIFFERENCE_BETWEEN_AVERAGE_AND_LONG = LONG_MICROS_CYCLE_LENGTH - AVERAGE_MICROS_CYCLE_LENGTH;

        uint32_t systemTime = sylib::millis();
        uint32_t detectorPreviousTime = sylib::millis();
        uint64_t systemTimeMicros = sylib::micros();
        uint64_t prevMicros = systemTimeMicros;

        do{
            systemTimeMicros = sylib::micros();
            detectorPreviousTime = systemTime;
            prevMicros = systemTimeMicros;
            sylib::delay_until(&systemTime, 3);
        }while((sylib::micros() - prevMicros) > (((systemTime-detectorPreviousTime)*AVERAGE_MICROS_CYCLE_LENGTH) - DIFFERENCE_BETWEEN_AVERAGE_AND_LONG));

        /*
        
        NOW WE'RE TIMED CORRECTLY, STARTING DAEMON
        
        */

        while(1){
            {
                mutex_lock _lock{mutex};
                frameCount++;
                for(auto & subTask : livingSubTasks){
                    if(!subTask->getSubTaskPaused() && ((frameCount + subTask->getUpdateOffset()) % subTask->getUpdateFrequency() == 0)){
                        subTask->update();
                    }
                }
            } 
            sylib::delay_until(&systemTime, 2);
        }
        return 1;
    }
    int SylibDaemon::frameCount = 0;
    sylib::Mutex SylibDaemon::mutex = sylib::Mutex();
    std::vector<sylib::Device  *> SylibDaemon::livingSubTasks = std::vector<sylib::Device  *>();
    int SylibDaemon::subTasksCreated = 0;

    void Device::startSubTask(){
        mutex_lock _lock{mutex};
        static SylibDaemon& taskMgr = SylibDaemon::getInstance();
        subTaskID = taskMgr.createSubTask(this);
    }
    void Device::killSubTask(){
        mutex_lock _lock{mutex};
        static SylibDaemon& taskMgr = SylibDaemon::getInstance();
        taskMgr.removeSubTask(this);
    }
    Device::Device (int interval, int offset) : updateFrequency(interval), updateOffset(offset){
        startSubTask();
    }
    Device::~Device (){
        mutex.lock();
        killSubTask();
    }
    int Device::getSubTaskID(){mutex_lock _lock(mutex);return subTaskID;}
    bool Device::getSubTaskPaused(){mutex_lock _lock(mutex); return subTaskPaused;}
    int Device::getUpdateFrequency(){mutex_lock _lock(mutex); return updateFrequency;}
    int Device::getUpdateOffset(){mutex_lock _lock(mutex); return updateOffset;}
    void Device::update(){
        printf("%dms - sylib subtask %d updated\n", sylib::millis(), subTaskID);
    }
}
