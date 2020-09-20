//================================================================================================//
//                                                                                                //
// FILE : print.h                                                                                 //
// MEMO : serial print functions to transmit debug and log data                                   //
//                                                                                                //
// Update Log                                                                                     //
//   2020/08/11 : Start this project                                                              //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

#ifndef PRINT_H
#define PRINT_H

//================================================================================================//
// void print_label(void) --- print data label                                                    //
//================================================================================================//
void print_label(void) {
  // For normal
  Serial.print("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3], omega_res_x10_lpf[0], omega_res_x10_lpf[1], omega_res_x10_lpf[2], omega_res_x10_lpf[3], omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3], vout[0], vout[1], vout[2], vout[3], vout_ff[0], vout_ff[1], vout_ff[2], vout_ff[3], vout_fb[0], vout_fb[1], vout_fb[2], vout_fb[3]\n");

  // For debug
  //  Serial.print("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], diff_cnt[0], diff_cnt[1], diff_cnt[2], diff_cnt[3], omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3], omega_res_x10_lpf[0], omega_res_x10_lpf[1], omega_res_x10_lpf[2], omega_res_x10_lpf[3], omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3], vout[0], vout[1], vout[2], vout[3]\n");
}

//================================================================================================//
// void print_log(void) --- print counted value                                                   //
//================================================================================================//
void print_log(void) {
  int i;
  Serial.print(n);
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(cnt_now[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(omega_res_x10[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(omega_res_x10_lpf[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(omega_cmd_x10[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(vout[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(vout_ff[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(vout_fb[i]);
  }
  Serial.print(",");
  Serial.print(acc1.n);
  Serial.print(",");
  Serial.print(acc1.acc_num);
  Serial.print(",");
  Serial.print(acc_stat);
  Serial.print(",");
  Serial.print(test_wait_cnt);
  Serial.print("\n");
}

void print_us(void) {
  Serial.print(head_angle);
  Serial.print(", ");
  Serial.print(head_time_1);
  Serial.print(", ");
  Serial.print(head_time_2);
  Serial.print(", ");
  Serial.print(head_time_2 - head_time_1);
  Serial.print("\n");
}

#endif
