#pragma once
#ifndef _MOTOR_COM_H_
#define _MOTOR_COM_H_

#define COM_INST    com[0]
#define COM_FUNC    com[1]
#define COM_ID      com[2]
#define COM_DATA    com[3]
#define COM_DATA2   com[4]

#define MOTOR_INST_PING             'P'
#define MOTOR_INST_READ             'R'
#define MOTOR_INST_WRITE            'W'
#define MOTOR_INST_REG_WRITE        'G'
#define MOTOR_INST_ACTION           'A'
#define MOTOR_INST_RESET            'E'
#define MOTOR_INST_SYNC_WRITE       'S'
#define MOTOR_INST_INFO             '-'
#define MOTOR_INST_WARN             '~'
#define MOTOR_INST_FATAL            '!'

#define MOTOR_FUNC_POSITION         'P'
#define MOTOR_FUNC_SPEED            'S'
#define MOTOR_FUNC_LOAD             'L'
#define MOTOR_FUNC_VOLTAGE          'V'
#define MOTOR_FUNC_TEMPERATURE      'T'
#define MOTOR_FUNC_TORQUE           'Q'

#endif
