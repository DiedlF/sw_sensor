#ifndef SRC_WATCHDOG_HANDLER_H_
#define SRC_WATCHDOG_HANDLER_H_

extern RestrictedTask watchdog_handler;
extern Semaphore watchdog_activator;
extern bool user_initiated_reset;

#endif /* SRC_WATCHDOG_HANDLER_H_ */
