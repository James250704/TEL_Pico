# TEL_Pico

使用 Raspberry Pi Pico 與 Arduino 控制 TEL 機器人。

## 檔案與資料夾說明

專案內主要資料夾與用途：

- `Brushless_motor` : 無刷馬達控制程式（PICO 與通訊方式尚未最終決定）
- `CLI_move` : 使用 CLI 控制移動平台
- `DBR4_Chanel_test` : DBR4 收訊頻道測試
- `Lifting_platform` : Arduino 升降平台程式
- `pipeline` : Arduino 輸球管道步進馬達控制
- `Reciver` : 收訊程式與發送訊號
- `Remote_move` : 移動平台控制程式
- `Servo_motor` : 伺服馬達控制程式

## 接線圖

設計與接線圖（Canva）：
https://www.canva.com/design/DAGzxWia0qw/PkkhCPlSIxHo7IvcaDqVeg/edit

## 遙控器按鈕對應

```
0      1      2      3     4    5   6   7   8   9

右水平  右垂直  左垂直  左水平  SA  SD  SB  SC  SE  SI
```

- 右水平 / 右垂直：移動平台控制
- 左水平：發射平台水平控制
- SB（左三段按鈕）：升降平台
- SI（右後滾輪）：移動平台原地旋轉
- SC (右三段按鈕)：無刷馬達速度調整(50, 70, 90)
（其餘按鈕功能可依專案需要延伸或補充說明）

## 未實作 / 待辦

- [ ] AI 辨識（尚未實作）
- [ ] 鏡頭尚未安裝（兩個皆未安裝）
- [ ] 皮帶尚未更改
- [ ] （尚未安裝）推拉式電磁鐵
