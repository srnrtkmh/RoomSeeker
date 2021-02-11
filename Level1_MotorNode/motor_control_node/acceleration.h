//================================================================================================//
//                                                                                                //
// FILE : acceleration.h                                                                          //
// MEMO : This library provides acceleration object to control robot                              //
//                                                                                                //
// Update Log                                                                                     //
//   2020/08/11 : Start this project                                                              //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

#ifndef ACCELERATION_H

#include <inttypes.h>

class acceleration {
  public:
    long init(long acc_init, long acc_cmd, long acc_time_ms, long sampling_time_ms);
    long get_cmd(long n);
    void clear(void);
    long n;
    long acc_num;
  private:
    long acc_table[100];
};

#endif // ACCELERATION_H
