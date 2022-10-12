#pragma once

#include "env.hpp"
#include "system.hpp"
#include <mutex>
#include <stdint.h>


namespace sylib {
    #ifdef SYLIB_ENV_PROS
    typedef pros::Mutex PlatformMutex;
    typedef pros::Task PlatformTask;
    #elif defined(SYLIB_ENV_VEXCODE)
    #define V5_MAX_DEVICE_PORTS 32
    typedef vex::mutex PlatformMutex;
    typedef vex::task PlatformTask;
    #endif

    void delay_until(std::uint32_t* const prev_time, const std::uint32_t delta); 
    void delay(std::uint32_t delay);
    uint32_t millis();
    uint64_t micros();
    class Mutex : public PlatformMutex{
        private: 
            bool try_lock_for();
            bool try_lock_until();
        public:
            bool take();
            bool give();
    };
    class Task : public PlatformTask{
        public:
            template <class F>
            Task(F&& func) : PlatformTask(func) {}
    };
    extern Mutex sylib_port_mutexes[V5_MAX_DEVICE_PORTS]; 
    using mutex_lock = const std::lock_guard<sylib::Mutex>;
    class Device ;
    class SylibDaemon{
        private:
            SylibDaemon();
	        ~SylibDaemon();
	        SylibDaemon(const SylibDaemon&);
	        const SylibDaemon& operator=(const SylibDaemon&);
            sylib::Task managerTask; // Change to something else if vexcode
            static int subTasksCreated;
            static std::vector<sylib::Device  *> livingSubTasks;
            static int frameCount;
        public:
            static sylib::Mutex mutex;
            static SylibDaemon& getInstance();
            static int managerTaskFunction();
            static int createSubTask(sylib::Device  * objectPointerToSchedule);
            static void removeSubTask(sylib::Device  * objectPointerToSchedule);
            static void removeSubTaskByID(int idToKill);
            static uint64_t getFrameCount();
    };

    class Device  {
        private:
            bool subTaskPaused = false;
            int updateFrequency;
            int updateOffset;
            int subTaskID;
            void startSubTask();
            bool idSet = false;
        public:
            sylib::Mutex mutex;
            Device (int interval = 2, int offset = 0);
            ~Device ();
            void resumeSubTask();
            void pauseSubTask();
            void killSubTask();
            bool isSubTaskRunning();
            int getUpdateFrequency();
            int getUpdateOffset();
            int setUpdateFrequency();
            int getSubTaskID();
            virtual void update();
            bool getSubTaskPaused();
            bool getFrameEnabled();
    };
}