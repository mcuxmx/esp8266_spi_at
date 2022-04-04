此工程实现MCU通过SPI的AT指令集访问ESP8266

### **硬件连接**

```
         MCU                                        ESP8266
|----------------------|                    |----------------------|
|                 GPIO |------------------->|RST                   |
|                      |                    |                      |
|                   CS |------------------->|HSPI_CS(IO15)         |
|                  CLK |------------------->|HSPI_CLK(IO14)        |
|                 MISO |<-------------------|HSPI_MISO(IO12)       |
|                 MOSI |------------------->|HSPI_MOSI(IO13)       |
|                      |                    |                      |
|                 GPIO |<-------------------|WR(IO0)               |
|                 GPIO |<-------------------|RD(IO2)               |
|                      |                    |                      |
|----------------------|                    |----------------------|
```
### **ESP8266引脚定义**


|  PIN |  NAME           | I/O      | Description |
| -----| --------------- | -------- | -------- |
| 15   | RST             | IN       | ESP8266复位脚，低电平复位     |
| 6    | HSPI_CS(IO15)   | IN       | ESP8266 HSPI片选             |
| 3    | HSPI_CLK(IO14)  | IN       | ESP8266 HSPI时钟             |
| 4    | HSPI_MISO(IO12) | OUT      | ESP8266 HSPI数据输出         |
| 5    | HSPI_MOSI(IO13) | IN       | ESP8266 HSPI数据输入         |
| 8    | WR(IO0)         | OUT      | ESP8266 高电平时主机可写数据，低电平不可写 |
| 7    | RD(IO2)         | OUT      | ESP8266 高电表示SPI有数据需要主机读取 |

### **通信协议**
硬件上MCU与ESP8266的HSPI通信使用HSPI的双线通信，具体参考[《ESP8266 SPI透传协议(双线).pdf》](ESP8266%20SPI透传协议%28双线%29.pdf)

MCU与ESP8266 SPI一帧传输固定32字节，格式如下：

|  Bytes |  1    | 2         | 3           | 4           | 5-32           |
| -----  | ----- | --------- | ----------- | ----------- | -------------- |
| Filed  | cmd   | block num | block index | size        | payload        |
其中：

cmd:表示当前传输的命令，目前仅传输AT指令cmd固定为1

block num:该次数据分成的数据块数

block index:当前传输的块序号，范围从1到block num

size:当前数据帧中有效数据的长度，若block index小于block num，则该值应固定为28，否则应小于等于28

payload:数据内容