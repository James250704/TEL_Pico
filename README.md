# TEL_Pico

使用 Pico 控制 TEL 機器人

## 文件說明

-   Brushless_motor :   無刷馬達控制程式 (尚未決定 PICO 與通訊)
-   CLI_move :          使用 CLI 控制移動平台
-   DBR4_Chanel_test :  DBR4 收訊頻道測試
-   Lifting_platform :  Arduino 升降平台程式
-   Reciver :           收訊程式與發送訊號
-   Remote_move :       移動平台控制程式
-   Servo_motor :       伺服馬達控制程式 (俯仰、水平)

## 接線圖

https://www.canva.com/design/DAGzxWia0qw/PkkhCPlSIxHo7IvcaDqVeg/edit


## 遙控器按鈕對應

```
0     1      2     3      4   5   6   7   8   9

右水平 右垂直  左垂直 左水平  SA  SD  SB  SC  SE  SI
```

-   右水平 右垂直 : 移動平台控制
-   左垂直 : 
-   左水平 : 發射平台水平
-   SA (左前按鈕) :       
-   SD (右前按鈕) :       
-   SB (左三段按鈕) : 升降平台 
-   SC (右三段控制) : 
-   SE (左扳機按鈕) :
-   SI (右後滾輪) : 移動平台原地旋轉


## 還未有設定之功能

-   球上升步進馬達控制
-   (還未安裝) 推拉式電磁鐵
-   tt馬達控制

# 尚未實作之內容
- [ ] AI辨識還未實作
- [ ] 鏡頭還未安裝 (兩個都是)
- [ ] 皮帶尚未更改