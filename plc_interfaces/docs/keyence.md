# memo
上位リンク通信のコマンド(see manual "8-4 コマンド一覧")

## general description
 msg(read)  = cmd + " " + device_type + device_num + data_format + separator
  ex ->     = WR  + " " + R           + 1000       + .U          + " " + 0/1    + separator
 msg(write) = cmd + " " + device_type + device_num + data_format + " " + data   + separator
  ex ->     = RD  + " " + R           + 1000       + .U          + separator

## cmd
- ?K: 機種の問い合わせ
- RD: データ読み込み
- WR: データ書き込み

## device_type + device_num
depend on device

## data_format
- "": 指定なし
- .U: 16bit unsigned
- .S: 16bit signed
- .D: 32bit unsigned
- .L: 32bit signed
- .H: 16bit hex

## separator
- '\r'  区切り符号CR

## port
- 8501 (default)
