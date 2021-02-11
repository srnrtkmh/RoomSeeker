//================================================================================================//
//                                                                                                //
// FILE : command.h                                                                               //
// MEMO : This library enables you to read command from upper controller                          //
//                                                                                                //
// Update Log                                                                                     //
//   2021/01/03 : Start this project                                                              //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

#ifndef COMMAND_H

#include <inttypes.h>

void read_wheel_vel_cmd(char *str, uint8_t ptr, long *omega_cmd_x10);
void read_work_vel_cmd(char *str, uint8_t ptr, long *work_vel_cmd_x10);
void read_vol_cmd(char *str, uint8_t ptr, int16_t *vol_cmd);
void read_pr1_cmd(char *str, uint8_t ptr);
void read_pr2_cmd(char *str, uint8_t ptr);
void read_pr3_cmd(char *str, uint8_t ptr);

#endif // COMMAND_H
