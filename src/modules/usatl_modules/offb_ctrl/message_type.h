//
// Created by sun on 2020/5/16.
//

#ifndef PX4_MESSAGE_TYPE_H
#define PX4_MESSAGE_TYPE_H

enum MESSAGE_TYPE : unsigned char{
    COMMAND = 0x01;
    CURRENT_STATE = 0x02;
    SETPOINT = 0x03;
    COMMAND_BACK = 0x03;
};

enum BACK_INfO : unsigned  char{

};

#endif //PX4_MESSAGE_TYPE_H
