#pragma once

void rpm_pid_init(void);
void rpm_pid_task(void *arg);

void rpm_pid_set_target(float left_rpm, float right_rpm);
void rpm_pid_stop(void);

float rpm_pid_get_target_left(void);
float rpm_pid_get_target_right(void);