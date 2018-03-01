#include <Arduino.h>

#include <Scheduler.h>

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 20ms units) and the maximum time they are expected to take (in
  microseconds)
 */
const Scheduler::Task Tracker::scheduler_tasks[] = {
        SCHED_TASK(flash_led,       10,    100)
};

void setup() {

}

void loop() {
    scheduler.tick();

    scheduler.run(19900UL);
}