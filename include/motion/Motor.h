#pragma once
#ifndef _MOTOR_H_
#define _MOTOR_H_

class Motor
{
  public:
    int model = 0;
    
  public:
    int position = 0;
    int speed = 0;
    int load = 0;
    char voltage = 0x00;
    char temperature = 0x00;
    char torque = 0x00;

  public:
    int position_max = 0;
    int position_min = 0;
    int load_limit = 0;
    bool flag_moving = 0;
    char flag_safety_lock = 0x00;
};

#endif
