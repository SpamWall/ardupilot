/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  main loop scheduler for APM
 *  Author: Craig Walters March 2018
 *
 */
#pragma once

//#include <AP_Param/AP_Param.h>
//#include <AP_HAL/Util.h>
//#include <AP_Math/AP_Math.h>
//#include "PerfInfo.h"       // loop perf monitoring

//#define AP_SCHEDULER_NAME_INITIALIZER(_name) .name = #_name,

/*
  useful macros for creating scheduler task table
 */
#define SCHED_TASK_CLASS(classname, classptr, func, _rate_hz, _max_time_micros) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_SCHEDULER_NAME_INITIALIZER(func)\
    .rate_hz = _rate_hz,\
    .max_time_micros = _max_time_micros\
}
#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(Tracker, &tracker, func, _interval_ticks, _max_time_micros)


/*
  A task scheduler for APM main loops

  Sketches should call scheduler.init() on startup, then call
  scheduler.tick() at regular intervals (typically every 10ms).

  To run tasks use scheduler.run(), passing the amount of time that
  the scheduler is allowed to use before it must return
 */

//#include <AP_HAL/AP_HAL.h>
//#include <AP_Vehicle/AP_Vehicle.h>

namespace ArduTracker {

    class Scheduler {
    public:
        FUNCTOR_TYPEDEF(scheduler_fastloop_fn_t, void);

        Scheduler(scheduler_fastloop_fn_t fastloop_fn = nullptr);

        /* Do not allow copies */
        Scheduler(const AP_Scheduler &other) = delete;
        Scheduler &operator=(const AP_Scheduler &) = delete;

//        FUNCTOR_TYPEDEF(task_fn_t, void);

        struct Task {
            task_fn_t function;
            const char *name;
            float rate_hz;
            uint16_t max_time_micros;
        };

        void init(const Task *tasks, uint8_t num_tasks, uint32_t log_performance_bit);

        // called by the sketch's main loop - which should be the only thing that function does
        void loop();

        // call to update any logging the scheduler might do; call at 1Hz
        void update_logging();

        // write out PERF message to dataflash
        void Log_Write_Performance();

        // call when one tick has passed
        void tick(void);

        // return current tick counter
        uint16_t ticks() const { return _tick_counter; }

        // run the tasks. Call this once per 'tick'.
        // time_available is the amount of time available to run task in microseconds
        void run(uint32_t time_available);

        // return the number of microseconds available for the current task
        uint16_t time_available_usec(void);

        // return debug parameter
        uint8_t debug_flags(void) { return _debug; }

        // return load average, as a number between 0 and 1. 1 means
        // 100% load. Calculated from how much spare time we have at the
        // end of a run()
        float load_average();

        // get the active main loop rate
        uint16_t get_loop_rate_hz(void) {
            if (_active_loop_rate_hz == 0) {
                _active_loop_rate_hz = _loop_rate_hz;
            }
            return _active_loop_rate_hz;
        }

        // get the time-allowed-per-loop in microseconds
        uint32_t get_loop_period_us() {
            if (_loop_period_us == 0) {
                _loop_period_us = 1000000UL / _loop_rate_hz;
            }
            return _loop_period_us;
        }

        // get the time-allowed-per-loop in seconds
        float get_loop_period_s() {
            if (is_zero(_loop_period_s)) {
                _loop_period_s = 1.0 / _loop_rate_hz;
            }
            return _loop_period_s;
        }

        float get_filtered_loop_time(void) const {
            return perf_info.get_filtered_time();
        }

        // get the time in seconds that the last loop took
        float get_last_loop_time_s(void) const {
            return _last_loop_time_s;
        }

        static const struct AP_Param::GroupInfo var_info[];

        // current running task, or -1 if none. Used to debug stuck tasks
        static int8_t current_task;

        // loop performance monitoring:
//        AP::PerfInfo perf_info;

    private:
        scheduler_fastloop_fn_t _fastloop_fn;          // function that is called before anything in the scheduler table:
        AP_Int8 _debug;                                // used to enable scheduler debugging
        AP_Int16 _loop_rate_hz;                        // overall scheduling rate in Hz
        AP_Int16 _active_loop_rate_hz;                 // loop rate in Hz as set at startup
        uint16_t _loop_period_us;                      // calculated loop period in usec
        float _loop_period_s;                          // calculated loop period in seconds
        const struct Task *_tasks;                     // progmem list of tasks to run
        uint8_t _num_tasks;                            // number of tasks in _tasks list
        uint16_t _tick_counter;                        // number of 'ticks' that have passed (number of times that tick() has been called
        uint16_t *_last_run;                           // tick counter at the time we last ran each task
        uint32_t _task_time_allowed;                   // number of microseconds allowed for the current task
        uint32_t _task_time_started;                   // the time in microseconds when the task started
        uint32_t _spare_micros;                        // number of spare microseconds accumulated
        uint8_t _spare_ticks;                          // number of ticks that _spare_micros is counted over
        uint32_t _loop_timer_start_us;                 // start of loop timing
        float _last_loop_time_s;                       // time of last loop in seconds
        AP_HAL::Util::perf_counter_t *_perf_counters;  // performance counters
        uint32_t _log_performance_bit;                 // bitmask bit which indicates if we should log PERF message to dataflash
    };
}

