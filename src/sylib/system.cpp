#include "system.hpp"
#include "vex/v5_api.h"
#include <iostream>
#include <stdint.h>

namespace sylib {
    void delay_until(std::uint32_t* const prev_time, const std::uint32_t delta){
        #ifdef SYLIB_ENV_PROS    
        pros::Task::delay_until(prev_time, delta);
        #elif defined(SYLIB_ENV_VEXCODE)
        std::uint32_t time = *prev_time + delta;
        vex::this_thread::sleep_until(time);  
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

    SylibDaemon::SylibDaemon(): managerTask(sylib::Task(managerTaskFunction)) {} // 
    SylibDaemon::~SylibDaemon(){}
    SylibDaemon& SylibDaemon::getInstance(){
        mutex_lock _lock(mutex);
	    return instance;
    }
    int SylibDaemon::createSubTask(sylib::UpdatingObject * objectPointerToSchedule){
        mutex_lock _lock(mutex);
        subTasksCreated++;
        livingSubTasks.push_back(objectPointerToSchedule);
        return subTasksCreated;
        return 1;
    }
    void SylibDaemon::removeSubTask(sylib::UpdatingObject *objectPointerToSchedule){
        mutex_lock _lock(mutex);
        livingSubTasks.erase(std::remove(livingSubTasks.begin(), livingSubTasks.begin(), objectPointerToSchedule));
    }
    void SylibDaemon::removeSubTaskByID(int idToKill){
        mutex_lock _lock(mutex);
        livingSubTasks.erase(std::remove_if(livingSubTasks.begin(), livingSubTasks.end(), [&](sylib::UpdatingObject * x){return (x->getSubTaskID() == idToKill);}));
    }
    int SylibDaemon::managerTaskFunction(){
	    printf("started sylib daemon %d\n", sylib::millis());
        uint32_t systemTime;
        uint64_t prevMicros = sylib::micros();
        sylib::delay(10);
        while((sylib::micros() - prevMicros) < 1000){
            systemTime = sylib::millis();
            sylib::delay_until(&systemTime, 1);
        }
        while(1){
            systemTime = sylib::millis();
            printf("%dms - sylib daemon updated\n", sylib::millis());
            {
                mutex_lock _lock{mutex};
                frameCount++;
                for(auto & subTask : livingSubTasks){
                    if(!subTask->getSubTaskPaused() && ((frameCount + subTask->getUpdateOffset()) % subTask->getUpdateFrequency() == 0)){
                        printf("%dms - subtask updater called\n", sylib::millis());
                        subTask->update();
                    }
                }
            }
            sylib::delay_until(&systemTime, 2- systemTime % 2);
        }
        return 1;
    }
    uint64_t SylibDaemon::frameCount = 0;
    sylib::Mutex SylibDaemon::mutex = sylib::Mutex();
    SylibDaemon SylibDaemon::instance = SylibDaemon();
    std::vector<sylib::UpdatingObject *> SylibDaemon::livingSubTasks = std::vector<sylib::UpdatingObject *>();
    int SylibDaemon::subTasksCreated = 0;
    SylibDaemon& UpdatingObject::taskMgr = SylibDaemon::getInstance();


    void UpdatingObject::startSubTask(){
        mutex_lock _lock{mutex};
        subTaskID = taskMgr.createSubTask(this);
    }
    void UpdatingObject::killSubTask(){
        mutex_lock _lock{mutex};
        taskMgr.removeSubTask(this);
    }
    UpdatingObject::UpdatingObject(int interval, int offset){
        startSubTask();
    }
    UpdatingObject::~UpdatingObject(){
        mutex.lock();
        killSubTask();
    }
    int UpdatingObject::getSubTaskID(){mutex_lock _lock(mutex);return subTaskID;}
    bool UpdatingObject::getSubTaskPaused(){mutex_lock _lock(mutex); return subTaskPaused;}
    int UpdatingObject::getUpdateFrequency(){mutex_lock _lock(mutex); return updateFrequency;}
    int UpdatingObject::getUpdateOffset(){mutex_lock _lock(mutex); return updateOffset;}
    void UpdatingObject::update(){
        std::cout << "Sylib subtask " << subTaskID << "updated at " << sylib::millis() << "ms" << std::endl;
    }
}