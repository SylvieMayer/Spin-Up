#pragma once

#include "env.hpp"
#include "system.hpp"
#include <cstdint>
#include <mutex>
#include <optional>
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
    extern Mutex sylib_controller_mutexes[2];
    using mutex_lock = const std::lock_guard<sylib::Mutex>;
    class Device ;
    class SylibDaemon{
        private:
            // static std::vector<Device*>& createLivingSubTasks();
            SylibDaemon();
	        ~SylibDaemon();
	        SylibDaemon(const SylibDaemon&);
	        const SylibDaemon& operator=(const SylibDaemon&);
            std::optional<sylib::Task> managerTask;
            static int subTasksCreated;
            static std::vector<sylib::Device*>& getLivingSubtasks();
            static int frameCount;
        public:
            static int indicator;
            static sylib::Mutex mutex;
            static SylibDaemon& getInstance();
            static void startSylibDaemon();
            static SylibDaemon& getInstanceUnsafe();
            static int managerTaskFunction();
            static int createSubTask(sylib::Device  * objectPointerToSchedule);
            static void removeSubTask(sylib::Device  * objectPointerToSchedule);
            static int createSubTaskUnsafe(sylib::Device  * objectPointerToSchedule);
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
            void setUpdateFrequency(int interval);
            void setUpdateOffset(int offset);
            int getSubTaskID();
            virtual void update();
            bool getSubTaskPaused();
            bool getFrameEnabled();
    };
}