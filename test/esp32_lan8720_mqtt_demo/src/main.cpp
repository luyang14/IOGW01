#include <Arduino.h>
/*
    This sketch shows how to configure different external or internal clock sources for the Ethernet PHY
*/

#include <ETH.h>
#include <PubSubClient.h>
#include <CAN_config.h>
#include <ESP32CAN.h>
#include "OneButton.h"//引用库函数

/* 
   * ETH_CLOCK_GPIO0_IN   - default: external clock from crystal oscillator
   * ETH_CLOCK_GPIO0_OUT  - 50MHz clock from internal APLL output on GPIO0 - possibly an inverter is needed for LAN8720
   * ETH_CLOCK_GPIO16_OUT - 50MHz clock from internal APLL output on GPIO16 - possibly an inverter is needed for LAN8720
   * ETH_CLOCK_GPIO17_OUT - 50MHz clock from internal APLL inverted output on GPIO17 - tested with LAN8720
*/
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN  //  ETH_CLOCK_GPIO17_OUT

// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN  16

// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE        ETH_PHY_LAN8720

// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR        1

// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN     23

// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN    18

PubSubClient client;

HardwareSerial LTE4G(1);
HardwareSerial RS485(2);

#define SYS_LED  2
#define LTE_POW  33
#define SYS_KEY  36

CAN_device_t CAN_cfg;

static bool eth_connected = false;
OneButton button(SYS_KEY,true);//
        //实例化一个OneButton对象
        //里面可以传三个参数:
        //pin : 按钮的pin角
        //activeLow : true:按下为低电平; false : 按下为高电平;不设置时默认值为：true
        //pullupActive : 如果有上拉电阻就激活上拉电阻

//按键事件回调函数
//单击
void attachClick()
{
  	Serial.println("click-单击");
}

//双击
void attachDoubleClick()
{
  	Serial.println("doubleclick-双击");
}

//长铵开始
void attachLongPressStart()
{
    Serial.println("longPressStart-长按开始");
}
//长按过程
void attachDuringLongPress()
{
  if (button.isLongPressed())
  {
    	Serial.println("duringLongPress-长按期间");
  }
}

//长按结束
void attachLongPressStop()
{
    Serial.println("longPressStop-长按结束");
}

//按下多次
void attachMultiClick()
{
  Serial.printf("getNumberClicks-总共按了：%d次。\r\n",button.getNumberClicks());
  switch(button.getNumberClicks()){
    	case 3:{Serial.printf("switch语句判断出打印3次。\r\n");break;}
    	case 4:{Serial.printf("switch语句判断出打印4次。\r\n");break;}
    	case 5:{Serial.printf("switch语句判断出打印5次。\r\n");break;}
    	case 6:{Serial.printf("switch语句判断出打印6次。\r\n");break;}
    	default:{Serial.printf("switch语句判断出打印其它次数:[%d]。\r\n",button.getNumberClicks());break;}
  }
}

//回调函数绑定子程序
void button_event_init(){

  button.reset();//清除一下按钮状态机的状态
   /**
   * set # millisec after safe click is assumed.
   */
  //void setDebounceTicks(const int ticks);
  button.setDebounceTicks(80);//设置消抖时长为80毫秒,默认值为：50毫秒

  /**
   * set # millisec after single click is assumed.
   */
  //void setClickTicks(const int ticks);
  button.setClickTicks(500);//设置单击时长为500毫秒,默认值为：400毫秒

  /**
   * set # millisec after press is assumed.
   */
  //void setPressTicks(const int ticks);
  button.setPressTicks(1000);//设置长按时长为1000毫秒,默认值为：800毫秒
  
  button.attachClick(attachClick);//初始化单击回调函数
  button.attachDoubleClick(attachDoubleClick);//初始化双击回调函数
  button.attachLongPressStart(attachLongPressStart);//初始化长按开始回调函数
  button.attachDuringLongPress(attachDuringLongPress);//初始化长按期间回调函数
  button.attachLongPressStop(attachLongPressStop);//初始化长按结束回调函数
  button.attachMultiClick(attachMultiClick);//初始化按了多次(3次或以上)回调函数
}

//按钮检测状态子程序
void button_attach_loop(){
    //不断检测按钮按下状态
    button.tick();
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void testClient(const char * host, uint16_t port) {
  Serial.print("\nconnecting to ");
  Serial.println(host);

  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    return;
  }
  client.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
  while (client.connected() && !client.available());
  //long i;
  while (client.available()) {
   // i=i+1;
    Serial.write(client.read());
   // if(i==100){i=0; delay(1);}
  }

  Serial.println("closing connection\n");
  client.stop();
}
#define mqtt_topic_max_size   100
#define mqtt_max_packet_size  256
#define mqtt_topic            "test"
#define gateway_name          "IOTTS"
#define will_Topic            "will_test"
#define mqtt_user             "xxxx"
#define mqtt_pass             "xxxx"

void connectMQTT() {

  Serial.println("MQTT connection...");
  char topic[mqtt_topic_max_size];
  strcpy(topic, mqtt_topic);
  strcat(topic, gateway_name);
  strcat(topic, will_Topic);
  client.setBufferSize(mqtt_max_packet_size);
#if 1
  if (client.connect(gateway_name, mqtt_user, mqtt_pass)) { // AWS doesn't support will topic for the moment
#else
  if (client.connect(gateway_name, mqtt_user, mqtt_pass, topic, will_QoS, will_Retain, will_Message)) {
#endif

    Serial.println("Connected to broker");
    //failure_number_mqtt = 0;
    // Once connected, publish an announcement...
    //pub(will_Topic, Gateway_AnnouncementMsg, will_Retain);
    // publish version
    client.publish(mqtt_topic,"haha");
    //Subscribing to topic
    char topic2[mqtt_topic_max_size];
    strcpy(topic2, mqtt_topic);
    strcat(topic2, gateway_name);
    Serial.println(topic2);
    if (client.subscribe(topic2)) {
    }
  } 
}
// Callback function, when the gateway receive an MQTT value on the topics subscribed this function is called
void callback(char* topic, byte* payload, unsigned int length) {
  // In order to republish this payload, a copy must be made
  // as the orignal payload buffer will be overwritten whilst
  // constructing the PUBLISH packet.
  Serial.printf("Hey I got a callback %s\n", topic);
}
static void* eClient = nullptr;

void setup() {
    Serial.begin(115200);
    LTE4G.begin(115200,SERIAL_8N1,35,32);
    RS485.begin(115200,SERIAL_8N1,34,17);
    pinMode(SYS_LED,OUTPUT);
    pinMode(LTE_POW,OUTPUT);
    pinMode(SYS_KEY,INPUT);
    button_event_init();//按钮事件初始化
    pinMode(4,OUTPUT);
    pinMode(5,OUTPUT);
    //pinMode(17,OUTPUT);
    digitalWrite(LTE_POW,HIGH);
    WiFi.onEvent(WiFiEvent);
    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
    eClient = new WiFiClient; 
    client.setClient(*(Client*)eClient);
    client.setServer("xxx.com",1883);
    client.setCallback(callback);

    CAN_cfg.speed=CAN_SPEED_125KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_5;
    CAN_cfg.rx_pin_id = GPIO_NUM_4;
    CAN_cfg.rx_queue = xQueueCreate(10,sizeof(CAN_frame_t));
    //start CAN Module
    ESP32Can.CANInit();
}

uint32_t sys_led_times = 0;
uint32_t sys_key_times = 0;
uint8_t key_press_flag = 0;

void loop() {
    button_attach_loop();
    /*
    if((SYS_KEY == 0)&&(key_press_flag == 0)){
        if(millis() - sys_led_times > 10){
          sys_led_times = millis(); 
          if((SYS_KEY == 0)&&(key_press_flag == 0)){
            key_press_flag = 1;
            Serial.println("key pressed");
          }
        }
    }
    if(SYS_KEY == 1)key_press_flag = 0;
    */
    CAN_frame_t rx_frame;
    //receive next CAN frame from queue
    if(xQueueReceive(CAN_cfg.rx_queue,&rx_frame, 3*portTICK_PERIOD_MS)==pdTRUE){

      //do stuff!
      if(rx_frame.FIR.B.FF==CAN_frame_std)
        printf("New standard frame");
      else
        printf("New extended frame");

      if(rx_frame.FIR.B.RTR==CAN_RTR)
        printf(" RTR from 0x%08x, DLC %d\r\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);
      else{
        printf(" from 0x%08x, DLC %d\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);
        for(int i = 0; i < 8; i++){
          printf("%c\t", (char)rx_frame.data.u8[i]);
        }
        printf("\n");
      }
    }
  if(millis() - sys_led_times > 500){
    sys_led_times = millis();
    digitalWrite(SYS_LED,!digitalRead(SYS_LED));
    //digitalWrite(4,!digitalRead(4));
    //digitalWrite(5,!digitalRead(5));
    //digitalWrite(17,!digitalRead(17));
    //LTE4G.println("AT");
    //RS485.println("RTRT");
    rx_frame.FIR.B.FF = CAN_frame_std;
    rx_frame.MsgID = 1;
    rx_frame.FIR.B.DLC = 8;
    rx_frame.data.u8[0] = 'h';
    rx_frame.data.u8[1] = 'e';
    rx_frame.data.u8[2] = 'l';
    rx_frame.data.u8[3] = 'l';
    rx_frame.data.u8[4] = 'o';
    rx_frame.data.u8[5] = 'c';
    rx_frame.data.u8[6] = 'a';
    rx_frame.data.u8[7] = 'n';

    
    ESP32Can.CANWriteFrame(&rx_frame);
  }
  if(RS485.available()){
    while (RS485.available())
    {
      Serial.println(RS485.readString());
    }
    
  }
  if(LTE4G.available()){
    while (LTE4G.available())
    {
      Serial.println(LTE4G.readString());
    }
    
  }
  if (eth_connected) {
    //testClient("xxxx.com", 80);
    if (client.connected()) {
      //digitalWrite(LED_INFO, LED_INFO_ON);
      //failure_number_ntwk = 0;

      client.loop();
    }else{
      connectMQTT();
    }
  }
}
