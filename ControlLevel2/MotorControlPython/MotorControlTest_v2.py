#!/usr/bin/env python
# coding: cp932
#==================================================================================================#
#                                                                                                  #
# FILE : MotorControlTest_v2.py                                                                    #
# Memo : Motor control test program in constant velocity command                                   #
#                                                                                                  #
# Updated : 2020/09/20 Started this project based on "MotorControlTest_v1.py"                      #
#                                                                                                  #
#                                                                       (C) 2020 Kyohei Umemoto    #
# How to execute :                                                                                 #
#     python MotorControlTest_v2.py [Serial device name]                                           #
#         or                                                                                       #
#     nohup python MotorControlTest_v2.py [Serial device name] > [tty????.txt] &                   #
#                                                                                                  #
# Argument Discription :                                                                           #
#     Serial device name : Example "/dev/ttyACM0", "/dev/ttyUSB1"                                  #
#                                                                                                  #
# How to end                                                                                       #
#     kill -INT [PID]                                                                              #
#     * PID is written [Devicename]_PID.txt                                                        #
#                                                                                                  #
# I/O assign :                                                                                     #
# 01 : 3.3V                                       02 : 5.0V                                        #
# 03 : NC                                         04 : 5.0V                                        #
# 05 : NC                                         06 : GND                                         #
# 07 : NC                                         08 : NC                                          #
# 09 : NC                                         10 : NC                                          #
# 11 : NC                                         12 : NC                                          #
# 13 : NC                                         14 : NC                                          #
# 15 : NC                                         16 : NC                                          #
# 17 : NC                                         18 : NC                                          #
# 19 : NC                                         20 : NC                                          #
# 21 : NC                                         22 : NC                                          #
# 23 : NC                                         24 : NC                                          #
# 25 : GND                                        26 : NC                                          #
#                                                                                                  #
#==================================================================================================#

#==================================================================================================#
# Import Module                                                                                    #
#==================================================================================================#
import sys            # sys(exit()の使用, 引数取得のために必要)
import os             # OS(ファイルの各種処理のために必要)
import locale         # local(!)
import datetime       # datetimeモジュール
import time           # 時刻取得用モジュール
import threading      # マルチスレッドプログラミング用モジュール
from math import *    # 算術演算用モジュール
import serial         # Arduinoとの通信で使用するシリアル通信用モジュール
import signal         # シグナル受信に使用するモジュール
import shutil         # ディレクトリが空でなくても削除するために使うモジュール

#==================================================================================================#
# Global variables                                                                                 #
#==================================================================================================#
# SignalTraceInterface用変数
data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
before_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

serialDeviceName = "/dev/ttyACM0" # シリアルデバイスの設定
serialChNum = 32                  # I/Oチャンネル数の設定
baudrate = 230400                 # ボーレート設定
parentPath = ""                   # 親ディレクトリの設定
dataPath = ""                     # データ用ディレクトリの設定
labelFileName = ""                # ラベル設定ファイル名
data_file = []                    # CSV出力用ファイルオブジェクト
strCsvName = ""                   # ファイル名用変数
file_header = ""                  # CSVファイルのヘッダー
preffix = "MotorControlTest_"     # ファイル名プレフィックス
suffix = ""                       # ファイル名サフィックス
dataPath = "./"                   # データ用ディレクトリの設定
parentPath = "../"                # 親ディレクトリの設定

before_nokori = ""
one_hour_cnt = 0
dt = 100000                       # サンプリングタイムの設定(デフォルトは100ms、引数にて設定するので実質無効)
plus_cnt = 0                      # サンプルタイムと定周期実行時間の差がプラス方向にオーバーしている回数を数えるカウンタ
minus_cnt = 0                     # サンプルタイムと定周期実行時間の差がマイナス方向にオーバーしている回数を数えるカウンタ

dateCurrentMonth = 0              # 月毎にログファイルを切り替えるための変数
dateBeforeMonth = 0               # 月毎にログファイルを切り替えるための変数

current_sample_number1 = 999      # 最初の1レコード目はサンプルナンバー"0"が来るはずなので初期今回値は"999"
current_sample_number2 = 999      # 最初の1レコード目はサンプルナンバー"0"が来るはずなので初期今回値は"999"
before_sample_number1 = 999       # どうせ書き換えられてしまうので初期値は適当
before_sample_number2 = 999       # どうせ書き換えられてしまうので初期値は適当

# シグナル受信処理用変数
pid = 0
pid_file = []

#==================================================================================================#
# Signal Receive Function                                                                          #
#==================================================================================================#
def receive_signal(signum, stack):
  print "Received signal :", signum
  currentDate = datetime.datetime.today()  # today()メソッドで現在日付・時刻のdatetime型データの変数を取得
  log_file.write(str(currentDate) + " : Received signal " + str(signum) + ". End this process/\n")
  con.write("t")
  time.sleep(0.01)
  con.write("t")
  time.sleep(0.01)
  con.write("t")
  data_file.close()
  log_file.close()
  sys.exit(0)

#==================================================================================================#
# Keyboard input function                                                                          #
#==================================================================================================#
def key_input():
  print("key_input started.")
  while True:
    test = raw_input()
    print(test)

#==================================================================================================#
# Main Function                                                                                    #
#==================================================================================================#
if __name__ == '__main__':
  # 引数処理
  param = sys.argv
  if len(param) >= 2:
    serialDeviceName = param[1]    # シリアルデバイスへのパス
  else:
    print "Invalid argument. Valid command : \"" + param[0] + " [Serial device name]\""
    sys.exit(1)
  
  # 初期設定
  currentDate = datetime.datetime.today() # today()メソッドで現在日付・時刻のdatetime型データの変数を取得
  
  # シグナルハンドル関係の設定
  signal.signal(signal.SIGINT, receive_signal)                # SIGINTを受け取ったら指定関数を呼び出すように設定
  
  # キーボード入力用スレッド作成
  th_key = threading.Thread(target=key_input)
  th_key.setDaemon(True)
  th_key.start()
  
  # ログファイルを開く
  log_file_name = unicode("./" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".log", encoding='shift-jis')
  log_file = open(log_file_name, 'w')      # ログファイルを新規作成
  log_file.write(str(currentDate) + " : Started to logging." + "\n")
  
  # データファイルを開く
  strCsvName = unicode("./" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".csv", encoding='shift-jis')
  data_file = open(strCsvName, 'w')                # ファイルを作成
  log_file.write(str(currentDate) + " : New file opened successfully. File name = " + strCsvName + "\n")
  
  # シリアル通信ポートをセットアップ
  try:
    con = serial.Serial(serialDeviceName, baudrate, timeout=0.2, rtscts=False, dsrdtr=False)  # ポートのオープン(timeoutの設定は適当、rtsctsとdsrdtrはTrueにしておかないとArduinoMicroとは通信できない)
    time.sleep(1.0)
  except serial.SerialException as e:
    print "SerialException : " + str(e)
    sys.exit(1)
  tmp_str = con.read(con.inWaiting())    # 読み取りバッファを空にしておく
  log_file.write(str(currentDate) + " : Serial port opened successfully. Port name = " + con.portstr + "\n")
  
  # 最初に1回だけスタートコマンドを送信
  tmp_str = con.read(con.inWaiting())    # 読み取りバッファを空にしておく(前に記述した1回だけだと空にならない時がある(?))
  time.sleep(1.0)                         # スタートコマンドのコールバックまで1秒間待ち
  con.write("s")                          # スタートコマンドを送信(1回目)
  log_file.write(str(currentDate) + " : Start command transmitted (1st try).\n")      # スタートコマンドを送った旨ログ
  time.sleep(1.0)                         # スタートコマンドのコールバックまで1秒間待ち
  tmp_str = con.read()                    # スタートコマンドのコールバックを確認
  if len(tmp_str) >= 10:
    con.write("s")                        # スタートコマンドを送信(2回目)
    log_file.write(str(currentDate) + " : Start command transmitted (2nd try).\n")    # スタートコマンドを送った旨ログ
    time.sleep(1.0)                       # スタートコマンドのコールバックまで1秒間待ち
    tmp_str = con.read()                  # スタートコマンドのコールバックを確認
    if len(tmp_str) >= 10:
      con.write("s")                      # スタートコマンドを送信(3回目)
      log_file.write(str(currentDate) + " : Start command transmitted (3rd try).\n")  # スタートコマンドを送った旨ログ
      time.sleep(1.0)                     # スタートコマンドのコールバックまで1秒間待ち
      tmp_str = con.read()                # スタートコマンドのコールバックを確認
      if len(tmp_str) >= 10:
        print "Serial Device did not transmit call back : " + tmp_str
        con.write("t")                    # 念のためストップコマンドを送っておく
        sys.exit(1)
  log_file.write(str(currentDate) + " : Start command echo back received.\n")
  
  # 制御変数初期化
  ctrl_cnt = 0
  ctrl_state = 0
  ctrl_cmd = "wrk,000,+1000,+0000,+0000,end"
  
  # 周期処理開始
  try:
    while True:
      # 制御処理
      if ctrl_cnt >= 100:
        ctrl_cnt = 0
        if ctrl_state == 0:
          ctrl_cmd = "wrk,000,+1000,+0000,+0000,end"
          ctrl_state = 1
        elif ctrl_state == 1:
          ctrl_cmd = "wrk,000,+0000,+1000,+0000,end"
          ctrl_state = 2
        elif ctrl_state == 2:
          ctrl_cmd = "wrk,000,-1000,+0000,+0000,end"
          ctrl_state = 3
        elif ctrl_state == 3:
          ctrl_cmd = "wrk,000,+0000,-1000,+0000,end"
          ctrl_state = 4
        elif ctrl_state == 4:
          ctrl_cmd = "wrk,000,+1000,+1000,+1000,end"
          ctrl_state = 5
        elif ctrl_state == 5:
          ctrl_cmd = "wrk,000,-1000,+1000,-1000,end"
          ctrl_state = 6
        elif ctrl_state == 6:
          ctrl_cmd = "wrk,000,-1000,-1000,+1000,end"
          ctrl_state = 7
        elif ctrl_state == 7:
          ctrl_cmd = "wrk,000,+1000,-1000,-1000,end"
          ctrl_state = 0
      ctrl_cnt = ctrl_cnt + 1
      
      con.write(ctrl_cmd)
      
      # 現在の日時を取得
      currentDate = datetime.datetime.today()
      
      # 読み取りデータをファイルに書き込み
      tmp_str = con.read(con.inWaiting())
      tmp_list = []                                     # 配列の中身をクリアしておく(必要かどうかは明確ではない)
      tmp_list = (before_nokori + tmp_str).split('\n')  # 前回サンプル残りと今回サンプルをつなぎ合わせて改行で分割
      if len(tmp_list) >= 2:
        for i in range(0, len(tmp_list) - 1):
          # data_file.write(dateCurrentSample.strftime("%Y/%m/%d %H:%M:%S") + ".%03d" % (dateCurrentSample.microsecond // 1000) + ',' + tmp_list[i] + '\n')
          data_file.write(currentDate.strftime("%Y/%m/%d %H:%M:%S") + ".%03d" % (currentDate.microsecond // 1000) + ',' + tmp_list[i] + '\n')
      before_nokori = tmp_list[len(tmp_list) - 1]
       
      log_file.flush()          # ログファイルへの書き込みを反映
      beforeDate = currentDate  # 前回処理時刻を更新
      time.sleep(0.01)           # 一定時間スリープ
  
  except KeyboardInterrupt:
    con.write("t")
    time.sleep(0.01)
    con.write("t")
    time.sleep(0.01)
    con.write("t")
    data_file.close()
    currentDate = datetime.datetime.today()                    # 現在の日時を取得
    log_file.write(str(currentDate) + " : Exit by the keyboard input" + "\n")  # キーボード入力で閉じたことをログに記載
    log_file.close()
    sys.exit(0)
