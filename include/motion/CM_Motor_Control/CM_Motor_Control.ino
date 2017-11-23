#define DELAY_TIME  20
#define PIN_BATTERY 0
#define RED_LED_485EXP     18
#define GREEN_LED_485EXP   19
#define BLUE_LED_485EXP    20
#define BUTTON1_485EXP 16
#define BUTTON2_485EXP 17
#define LED_CYCLE_BLINK_DIFF 12400//Max : 65535
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#define MOTOR_SIZE 32
#define CHAR_SIZE  32
//#define FREQ_STATUS_SCAN 1
#define TOLERENCE_POSITION 5
#define TOLERENCE_SPEED 5
#define TOLERENCE_LOAD 5
#define TOLERENCE_VOLTAGE 10
#define TOLERENCE_TEMPERATURE 10

////////Motor
#define WARN_RATIO  0.7
#define FATAL_RATIO 0.9
#define VOLTAGE_MIN 10
#define VOLTAGE_MAX 15
#define TEMPERATUE_LIM 70
#define LOAD_LIM 1023

#define p_model1 0//(0X00) Model Number(L) 低位元資料 R 12 (0X0C)
#define p_model2 1//(0X01) Model Number(H) 高位元資料 R 0 (0X00) 
#define p_version 2//(0X02) Version of Firmware 版本資料 R - 

#define p_id 3//(0X03) ID ID 編號 RW 1 (0X01) 
#define p_rate 4//(0X04) Baud Rate 鮑率資料 RW 1 (0X01) 
#define p_rdt 5//(0X05) Return Delay Time 回傳延遲時間 RW 0 (0X0) 
#define p_cwall 6//(0X06) CW Angle Limit(L) 順時針角度限制低位元資料 RW 0 (0X00) 
#define p_cwalh 7//(0X07) CW Angle Limit(H) 順時針角度限制高位元資料 RW 0 (0X00) 
#define p_ccwall 8//(0X08) CCW Angle Limit(L) 逆時針角度限制低位元資料 RW 255 (0XFF)
#define p_ccwalh 9//(0X09) CCW Angle Limit(H) 逆時針角度限制高位元資料 RW 3 (0X03) 
#define p_hlt 11//(0X0B)the Highest Limit Temperature 可容許的最大溫度 RW 70 (0X46)
#define p_llv 12//(0X0C)the Lowest Limit Voltage 可容許的最低電壓 RW 60 (0X3C)
#define p_hlv 13//(0X0D)the Highest LimitVoltage 可容許的最高電壓 RW 140 (0XBE)
#define p_mtl 14//(0X0E) Max Torque(L) 最大扭拒低位元資料 RW 255 (0XFF)
#define p_mth 15//(0X0F) Max Torque(H) 最大扭拒高位元資料 RW 3 (0X03) 
#define p_srl 16//(0X10) Status Return Level 狀態回報等級 RW 2 (0X02) 
#define p_al 17//(0X11) Alarm LED LED 指示燈 RW 36(0x24)
#define p_ass 18//(0X12) Alarm Shutdown Shutdown for Alarm RW 36(0x24)

#define p_te 24//(0X18) Torque Enable 可開啟、關閉馬達的扭力 RW 0 (0X00) 
#define p_led 25//(0X19) LED LED On/Off RW 0 (0X00) 
#define p_cwcm 26//(0X1A) CW Compliance Margin 轉距變數，進階使用者選項 RW 1 (0X01) 
#define p_ccwcm 27//(0X1B)CCW Compliance Margin 轉距變數，進階使用者選項 RW 1 (0X01) 
#define p_cwcs 28//(0X1C) CW Compliance Slope 轉距斜率變數，進階使用者選項 RW 32 (0X20)
#define p_ccwcs 29//(0X1D) CCW Compliance Slope 轉距斜率變數，進階使用者選項 RW 32 (0X20)
#define p_posl 30//(0X1E) Goal Position(L) 角度位置低位元 RW - 
#define p_posh 31//(0X1F) Goal Position(H) 角度位置高位元 RW - 
#define p_speedl 32//(0X20) Moving Speed(L) 轉動速度低位元 RW - 
#define p_speedh 33//(0X21) Moving Speed(H) 轉動速度高位元 RW - 
#define p_tll 34//(0X22) Torque Limit(L) 扭力限制低位元 RW ADD14 
#define p_tlh 35//(0X23) Torque Limit(H) 扭力限制高位元 RW ADD15 

#define p_preposl 36//(0X24) Present Position(L) 當前角度位置低位元 R - 
#define p_preposh 37//(0X25) Present Position(H) 當前角度位置高位元 R - 
#define p_prespeedl 38//(0X26) Present Speed(L) 當前轉動速度低位元 R - 
#define p_prespeedh 39//(0X27) Present Speed(H) 當前轉動速度高位元 R - 
#define p_preloadl 40//(0X28) Present Load(L) 目前的馬達負荷量低位元 R - 
#define p_preloadh 41//(0X29) Present Load(H) 目前的馬達負荷量高位元 R - 
#define p_prev 42//(0X2A) Present Voltage 當前電壓 R - 
#define p_pret 43//(0X2B) Present Temperature 當前溫度 R - 
#define p_rm 44//(0X2C) Registered Means if Instruction is registered R 0 (0X00) 
#define p_m 46//(0X2E) Moving 轉動旗標 R 0 (0X00) 
#define p_lle 47//(0X2F) Lock Locking EEPROM RW 0 (0X00) 
#define p_punchl 48//(0X30) Punch(L) Lowest byte of Punch RW 32 (0X20)
#define p_punchh 49//(0X31) Punch(H) Highest byte of Punch RW 0 (0X00) 

Dynamixel Dxl(DXL_BUS_SERIAL3);
int get_model(byte& id)                             {delay(DELAY_TIME);return Dxl.readWord(id, p_model1);};
byte get_id(byte& id)                               {return Dxl.readByte(id, p_id);};
void set_id(byte& id, byte data)                    {Dxl.writeByte(id, p_id, data);};
byte get_baudrate(byte& id)                         {return Dxl.readByte(id, p_rate);};
void set_baudrate(byte& id, byte data)              {Dxl.writeByte(id, p_rate, data);};
byte get_return_delay_time(byte& id)                {return Dxl.readByte(id, p_rdt);};
void set_return_delay_time(byte& id, byte data)     {Dxl.writeByte(id, p_rdt, data);};
int get_cw_angle_limit(byte& id)                    {return Dxl.readWord(id, p_cwall);};
void set_cw_angle_limit(byte& id, int data)         {Dxl.writeWord(id, p_cwall, data);};
int get_ccw_angle_limit(byte& id)                   {return Dxl.readWord(id, p_ccwall);};
void set_ccw_angle_limit(byte& id, int data)        {Dxl.writeWord(id, p_ccwall, data);};
byte get_temperature_limit(byte& id)                {return Dxl.readByte(id, p_hlt);};
void set_temperature_limit(byte& id, byte data)     {Dxl.writeByte(id, p_hlt, data);};//Be aware of using this function ! Motor will stop for 20 minutes while temperature exceeding the limit.
byte get_voltage_min(byte& id)                      {return Dxl.readByte(id, p_llv);};
void set_voltage_min(byte& id, byte data)           {Dxl.writeByte(id, p_llv, data);};
byte get_voltage_max(byte& id)                      {return Dxl.readByte(id, p_hlv);};
void set_voltage_max(byte& id, byte data)           {Dxl.writeByte(id, p_hlv, data);};
int get_torque_limit(byte& id)                      {return Dxl.readWord(id, p_mtl);};
void set_torque_limit(byte& id, int data)           {Dxl.writeWord(id, p_mtl, data);};
byte get_status(byte& id)                           {return Dxl.readByte(id, p_srl);};
byte get_alarm_LED(byte& id)                        {return Dxl.readByte(id, p_al);};
void set_alarm_LED(byte& id, byte data)             {Dxl.writeByte(id, p_al, data);};

int get_position(byte& id)                          {return Dxl.readWord(id, p_preposl)%1024;};
int get_speed(byte& id)                             {return Dxl.readWord(id, p_prespeedl)%1024;};
int get_load(byte& id)                              {return Dxl.readWord(id, p_preloadl)%1024;};
byte get_voltage(byte& id)                          {return Dxl.readByte(id, p_prev);};
byte get_temperature(byte& id)                      {return Dxl.readByte(id, p_pret);};

void set_position(byte& id, int data)               {Dxl.writeWord(id, p_posl, data);};
void set_speed(byte& id, int data)                  {Dxl.writeWord(id, p_speedl, data);};
void set_load_limit(byte& id, int data)             {Dxl.writeWord(id, p_tll, data);};
int get_load_limit(byte& id)			    {return Dxl.readWord(id, p_tll);};
void set_torque_enable(byte& id, byte data)         {Dxl.writeByte(id, p_te, data);};
byte get_torque_enable(byte& id)                    {return Dxl.readByte(id, p_te);};
void set_LED_enable(byte& id, byte data)            {Dxl.writeByte(id, p_led, data);};
byte get_LED_enable(byte& id)                       {return Dxl.readByte(id, p_led);};
byte is_moving(byte& id)                            {return Dxl.readByte(id, p_m);};

class Motor
{
  public:
    bool flag_motor;
    int model;
    
  public:
    int position;
    int speed;
    int load;
    byte voltage;
    byte temperature;
    byte torque;

  public:
    int position_max;
    int position_min;
    int load_limit;
    bool flag_moving;
    byte flag_safety_lock;
    
  public:
    Motor(){};
    ~Motor(){};
};
#define LOCK_POSITION       0x01
#define LOCK_SPEED          0x02
#define LOCK_LOAD           0x04
#define LOCK_VOLTAGE        0x08
#define LOCK_TEMPERATURE    0x10
#define LOCK_TORQUE         0x20
#define LOCK_XX2            0x40
#define LOCK_XX1            0x80
Motor motor[MOTOR_SIZE];
Motor motor_old[MOTOR_SIZE];
byte motor_list_id[MOTOR_SIZE];
byte motor_size = 0;

void Motor_Init();
void Motor_Ping();
void Motor_Scan();
void Motor_Status_Scan(byte& id);
void Motor_Scan(byte& id);
void Motor_Position_Set(byte& id, int data);
void Motor_Speed_Set(byte& id, int data);
void Motor_Load_Limit_Set(byte& id, byte data);
void Motor_Torque_Enable_Set(byte& id, byte data);
void Motor_Position_Detect(byte& id);
void Motor_Speed_Detect(byte& id);
void Motor_Load_Detect(byte& id);
void Motor_Voltage_Detect(byte& id);
void Motor_Temperature_Detect(byte& id);
void Motor_Torque_Detect(byte& id);
void Motor_Moving_Detect(byte& id);
////////
////////Communication
char com[CHAR_SIZE];
byte com_size = 0;

void USB_Interrupt(byte* buffer, byte nCount);
void USB_Publisher(byte* buffer, byte nCount);
void USB_Publisher(char* buffer, byte nCount);
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


void pub_info_set(){COM_INST = MOTOR_INST_INFO;};
void pub_warn_set(){COM_INST = MOTOR_INST_WARN;};
void pub_fatal_set(){COM_INST = MOTOR_INST_FATAL;};
void pub_position_set(byte& id)
{
  if(!motor[id].flag_motor) return;
  COM_ID = id;
  COM_FUNC = MOTOR_FUNC_POSITION;
  COM_DATA = byte(motor[id].position/256);
  COM_DATA2= byte(motor[id].position%256);
  com_size = 5;
}
void pub_speed_set(byte& id)
{
  if(!motor[id].flag_motor) return;
  COM_ID = id;
  COM_FUNC = MOTOR_FUNC_SPEED;
  COM_DATA = byte(motor[id].speed/256);
  COM_DATA2= byte(motor[id].speed%256);
  com_size = 5;
}
void pub_load_set(byte& id)
{
  if(!motor[id].flag_motor) return;
  COM_ID = id;
  COM_FUNC = MOTOR_FUNC_LOAD;
  COM_DATA = byte(motor[id].load/256);
  COM_DATA2= byte(motor[id].load%256);
  com_size = 5;
}
void pub_voltage_set(byte& id)
{
  if(!motor[id].flag_motor) return;
  COM_ID = id;
  COM_FUNC = MOTOR_FUNC_VOLTAGE;
  COM_DATA = 0x00;
  COM_DATA2= motor[id].voltage;
  com_size = 5;
}
void pub_temperature_set(byte& id)
{
  if(!motor[id].flag_motor) return;
  COM_ID = id;
  COM_FUNC = MOTOR_FUNC_TEMPERATURE;
  COM_DATA = 0x00;
  COM_DATA2= motor[id].temperature;
  com_size = 5;
}
void pub_torque_set(byte& id)
{
  if(!motor[id].flag_motor) return;
  COM_ID = id;
  COM_FUNC = MOTOR_FUNC_TORQUE;
  COM_DATA = 0x00;
  COM_DATA2= motor[id].torque;
  com_size = 5;
}


bool read_position(byte* buffer, byte& buffer_size)
{
  if(buffer[1] != MOTOR_FUNC_POSITION) return false;
  if(!motor[buffer[2]].flag_motor) return false;
  Motor_Position_Detect(buffer[2]);
  pub_position_set(buffer[2]);
  USB_Publisher(com, com_size);
  return true;
}
bool write_position(byte* buffer, byte& buffer_size)
{
  if(buffer[1] != MOTOR_FUNC_POSITION) return false;
  if(buffer[2] >= motor_size) return false;
  Motor_Position_Set(buffer[2], int(buffer[3])*256 + int(buffer[4]));
  return true;
}
bool read_speed(byte* buffer, byte& buffer_size)
{
  if(buffer[1] != MOTOR_FUNC_SPEED) return false;
  if(!motor[buffer[2]].flag_motor) return false;
  Motor_Speed_Detect(buffer[2]);
  pub_speed_set(buffer[2]);
  USB_Publisher(com, com_size);
  return true;
}
bool write_speed(byte* buffer, byte& buffer_size)
{
  if(buffer[1] != MOTOR_FUNC_SPEED) return false;
  if(buffer[2] >= motor_size) return false;
  Motor_Speed_Set(buffer[2], int(buffer[3])*256 + int(buffer[4]));
  return true;
}
bool read_load(byte* buffer, byte& buffer_size)
{
  if(buffer[1] != MOTOR_FUNC_LOAD) return false;
  if(!motor[buffer[2]].flag_motor) return false;
  Motor_Load_Detect(buffer[2]);
  pub_load_set(buffer[2]);
  USB_Publisher(com, com_size);
  return true;
}
bool write_load_limit(byte* buffer, byte& buffer_size)
{
  if(buffer[1] != MOTOR_FUNC_LOAD) return false;
  if(buffer[2] >= motor_size) return false;
  Motor_Load_Limit_Set(buffer[2], int(buffer[3])*256 + int(buffer[4]));
  return true;
}
bool read_voltage(byte* buffer, byte& buffer_size)
{
  if(buffer[1] != MOTOR_FUNC_VOLTAGE) return false;
  if(!motor[buffer[2]].flag_motor) return false;
  Motor_Voltage_Detect(buffer[2]);
  pub_voltage_set(buffer[2]);
  USB_Publisher(com, com_size);
  return true;
}
bool read_temperature(byte* buffer, byte& buffer_size)
{
  if(buffer[1] != MOTOR_FUNC_TEMPERATURE) return false;
  if(!motor[buffer[2]].flag_motor) return false;
  Motor_Temperature_Detect(buffer[2]);
  pub_temperature_set(buffer[2]);
  USB_Publisher(com, com_size);
  return true;
}
bool read_torque(byte* buffer, byte& buffer_size)
{
  if(buffer[1] != MOTOR_FUNC_TORQUE) return false;
  if(!motor[buffer[2]].flag_motor) return false;
  Motor_Torque_Detect(buffer[2]);
  pub_torque_set(buffer[2]);
  USB_Publisher(com, com_size);
  return true;
}
bool write_torque_enable(byte* buffer, byte& buffer_size)
{
  if(buffer[1] != MOTOR_FUNC_TORQUE) return false;
  if(buffer[2] >= motor_size) return false;
  if(int(buffer[3])*256 + int(buffer[4]) > 0)
    Motor_Torque_Enable_Set(buffer[2], 0x01);
  else
    Motor_Torque_Enable_Set(buffer[2], 0x00);
  return true;
}
void ping(byte* buffer, byte& buffer_size){if(buffer_size < 1) return; if(buffer[0] == MOTOR_INST_PING) {Motor_Ping();  Motor_Scan();}};
//void action(byte* buffer, byte& buffer_size){if(buffer_size < 1) return; if(buffer[0] == MOTOR_INST_ACTION){};};
void read(byte* buffer, byte& buffer_size)
{
  if(buffer_size < 5) return;
  if(buffer[0] != MOTOR_INST_READ) 
    return;
  pub_info_set();
  if(read_position(buffer, buffer_size))    return;
  if(read_speed(buffer, buffer_size))       return;
  if(read_load(buffer, buffer_size))        return;
  if(read_voltage(buffer, buffer_size))     return;
  if(read_temperature(buffer, buffer_size)) return;
  if(read_torque(buffer, buffer_size))      return;
}
void write(byte* buffer, byte& buffer_size)
{
  if(buffer_size < 5) return;
  if(buffer[0] != MOTOR_INST_WRITE)         return;
  if(write_position(buffer, buffer_size))   return;
  if(write_speed(buffer, buffer_size))      return;
  if(write_load_limit(buffer, buffer_size)) return;
  if(write_torque_enable(buffer, buffer_size))  return;
}
//void reg_write(byte* buffer, byte& buffer_size);

////////General

int loop_count = 0;
byte scan_idx = 0;
int blink_value = 0;
int blink_diff = LED_CYCLE_BLINK_DIFF;
bool flag_scan = true;


void LED_Cycle_Blink_Breath() {if((65535-blink_value) < blink_diff) {blink_value = 65535; blink_diff *= -1;} else if((0-blink_value) > blink_diff) {blink_value = 0; blink_diff *= -1;} else blink_value += blink_diff; analogWrite(BOARD_LED_PIN, blink_value);};
float Battery_Read() {return float(analogRead(PIN_BATTERY))/10.0/910.0*11.1;}; // Reading values from our battery} //910 is the result of (4095/(R2(R2+R1)))/10
//void delay_func(){LED_Cycle_Blink_Breath(); delay(DELAY_TIME);};
void Safety_Load_Limit_Lock(byte& id){if(motor[id].load_limit <= 0) return; motor[id].load_limit = (motor[id].load_limit - 1 > 0)? motor[id].load_limit - 1 : 0; set_load_limit(id, motor[id].load_limit);};
void Safety_Load_Limit_Release(byte& id){if(motor[id].load_limit >= 1023) return; motor[id].load_limit = (motor[id].load_limit + 1 < 1024)? motor[id].load_limit + 1 : 1023; set_load_limit(id, motor[id].load_limit);};
void Safety_Load_Limit(byte& id){if(motor[id].flag_safety_lock > 0) Safety_Load_Limit_Lock(id);};
void Safety_Load_Limit(){for(byte i = 0; i < motor_size; i++){Safety_Load_Limit(motor_list_id[i]);}};

////////
////////Motor Definition & Setting
#define MOTOR_LEFT_THIGH_ID             1
#define MOTOR_LEFT_THIGH_ANGLE_MIN      273//512-1024/300*90    273
#define MOTOR_LEFT_THIGH_ANGLE_MAX      870//512+1024/300*90    870
#define MOTOR_LEFT_THIGH_LOAD_LIMIT   1023
#define MOTOR_LEFT_CALF_ID              2
#define MOTOR_LEFT_CALF_ANGLE_MIN       307//512-1024/300*0     307
#define MOTOR_LEFT_CALF_ANGLE_MAX       1023//wait for test     1023
#define MOTOR_LEFT_CALF_LOAD_LIMIT    1023
#define MOTOR_LEFT_ANKLE_ID             3
#define MOTOR_LEFT_ANKLE_ANGLE_MIN      376//512-1024/300*90    376
#define MOTOR_LEFT_ANKLE_ANGLE_MAX      821//wait for test      821
#define MOTOR_LEFT_ANKLE_LOAD_LIMIT   1023
#define MOTOR_RIGHT_THIGH_ID            4
#define MOTOR_RIGHT_THIGH_ANGLE_MIN     154//512-1024/300*90    154
#define MOTOR_RIGHT_THIGH_ANGLE_MAX     751//512+1024/300*90    751
#define MOTOR_RIGHT_THIGH_LOAD_LIMIT  1023
#define MOTOR_RIGHT_CALF_ID             5
#define MOTOR_RIGHT_CALF_ANGLE_MIN      0//wait for test        0
#define MOTOR_RIGHT_CALF_ANGLE_MAX      717//                   717
#define MOTOR_RIGHT_CALF_LOAD_LIMIT   1023
#define MOTOR_RIGHT_ANKLE_ID            6
#define MOTOR_RIGHT_ANKLE_ANGLE_MIN     203//wait for test      203
#define MOTOR_RIGHT_ANKLE_ANGLE_MAX     648//512+1024/300*90    648
#define MOTOR_RIGHT_ANKLE_LOAD_LIMIT  1023
#define MOTOR_MOUTH_ID                  7
#define MOTOR_MOUTH_ANGLE_MIN           427//512-1024/300*90    427
#define MOTOR_MOUTH_ANGLE_MAX           819//512+1024/300*90    819
#define MOTOR_MOUTH_LOAD_LIMIT        1023
#define MOTOR_HEAD_ID                   8
#define MOTOR_HEAD_ANGLE_MIN            205//512-1024/300*90
#define MOTOR_HEAD_ANGLE_MAX            819//512+1024/300*90
#define MOTOR_HEAD_LOAD_LIMIT         1023
#define MOTOR_NECK_VERTICAL_ID          9
#define MOTOR_NECK_VERTICAL_ANGLE_MIN   427//512-1024/300*90    427
#define MOTOR_NECK_VERTICAL_ANGLE_MAX   736//512+1024/300*90    736
#define MOTOR_NECK_VERTICAL_LOAD_LIMIT    1023
#define MOTOR_NECK_HORIZON_ID           10
#define MOTOR_NECK_HORIZON_ANGLE_MIN    393//512-1024/300*90    393
#define MOTOR_NECK_HORIZON_ANGLE_MAX    633//512+1024/300*90    633
#define MOTOR_NECK_HORIZON_LOAD_LIMIT 1023
#define LENGTH_THIGH    7.6
#define LENGTH_CALF     4.8
#define LENGTH_ANGLE    8.9

void Raptor_Motor_Init();
////////

void setup() 
{
  SerialUSB.println("## CM_Motor_Control ##");
  delay(2000);//Opening for power and other device on
  pinMode(BOARD_LED_PIN, PWM);
  pinMode(RED_LED_485EXP, OUTPUT); 
  pinMode(GREEN_LED_485EXP, OUTPUT); 
  pinMode(BLUE_LED_485EXP, OUTPUT); 
  digitalWrite(RED_LED_485EXP, 0);
  digitalWrite(GREEN_LED_485EXP, 0);
  digitalWrite(BLUE_LED_485EXP, 0);
  pinMode(BUTTON1_485EXP, INPUT); 
  pinMode(BUTTON2_485EXP, INPUT); 
  pinMode(PIN_BATTERY, INPUT_ANALOG);
  SerialUSB.attachInterrupt(USB_Interrupt);
  SerialUSB.println("Initializing ...");
  Motor_Init();
  Raptor_Motor_Init();
//  SerialUSB.println("Scaning motor ...");
//  Motor_Scan();
  blink_diff = 65535/motor_size*2;
  SerialUSB.println("Program Running ...");
  delay(5000);//Delay for Light on Motors
  for(byte i = 0; i < motor_size; i++)
    set_LED_enable(motor_list_id[i], 0x00);
  digitalWrite(RED_LED_485EXP, 1);
  digitalWrite(GREEN_LED_485EXP, 1);
  digitalWrite(BLUE_LED_485EXP, 1);
}

void loop() 
{
  //if(loop_count % FREQ_STATUS_SCAN == 0)
  {
    if(flag_scan)
    {
      if(motor_size > 0)
      {
        Motor_Scan(motor_list_id[scan_idx]);
        Motor_Status_Scan(motor_list_id[scan_idx]);
        Safety_Load_Limit(motor_list_id[scan_idx]);
        scan_idx = (scan_idx + 1 < motor_size)? scan_idx + 1 : 0;
      }
    }
  }
  LED_Cycle_Blink_Breath();
  loop_count += 1;
}

void Raptor_Motor_Init()
{
  motor[MOTOR_LEFT_THIGH_ID].position_max = MOTOR_LEFT_THIGH_ANGLE_MAX;
  motor[MOTOR_LEFT_THIGH_ID].position_min = MOTOR_LEFT_THIGH_ANGLE_MIN;
  motor[MOTOR_LEFT_THIGH_ID].load_limit = MOTOR_LEFT_THIGH_LOAD_LIMIT;
  motor[MOTOR_LEFT_CALF_ID].position_max = MOTOR_LEFT_CALF_ANGLE_MAX;
  motor[MOTOR_LEFT_CALF_ID].position_min = MOTOR_LEFT_CALF_ANGLE_MIN;
  motor[MOTOR_LEFT_CALF_ID].load_limit = MOTOR_LEFT_CALF_LOAD_LIMIT;
  motor[MOTOR_LEFT_ANKLE_ID].position_max = MOTOR_LEFT_ANKLE_ANGLE_MAX;
  motor[MOTOR_LEFT_ANKLE_ID].position_min = MOTOR_LEFT_ANKLE_ANGLE_MIN;
  motor[MOTOR_LEFT_ANKLE_ID].load_limit = MOTOR_LEFT_ANKLE_LOAD_LIMIT;
  motor[MOTOR_RIGHT_THIGH_ID].position_max = MOTOR_RIGHT_THIGH_ANGLE_MAX;
  motor[MOTOR_RIGHT_THIGH_ID].position_min = MOTOR_RIGHT_THIGH_ANGLE_MIN;
  motor[MOTOR_RIGHT_THIGH_ID].load_limit = MOTOR_RIGHT_THIGH_LOAD_LIMIT;
  motor[MOTOR_RIGHT_CALF_ID].position_max = MOTOR_RIGHT_CALF_ANGLE_MAX;
  motor[MOTOR_RIGHT_CALF_ID].position_min = MOTOR_RIGHT_CALF_ANGLE_MIN;
  motor[MOTOR_RIGHT_CALF_ID].load_limit = MOTOR_RIGHT_CALF_LOAD_LIMIT;
  motor[MOTOR_RIGHT_ANKLE_ID].position_max = MOTOR_RIGHT_ANKLE_ANGLE_MAX;
  motor[MOTOR_RIGHT_ANKLE_ID].position_min = MOTOR_RIGHT_ANKLE_ANGLE_MIN;
  motor[MOTOR_RIGHT_ANKLE_ID].load_limit = MOTOR_RIGHT_ANKLE_LOAD_LIMIT;
  motor[MOTOR_MOUTH_ID].position_max = MOTOR_MOUTH_ANGLE_MAX;
  motor[MOTOR_MOUTH_ID].position_min = MOTOR_MOUTH_ANGLE_MIN;
  motor[MOTOR_MOUTH_ID].load_limit = MOTOR_MOUTH_LOAD_LIMIT;
  motor[MOTOR_HEAD_ID].position_max = MOTOR_HEAD_ANGLE_MAX;
  motor[MOTOR_HEAD_ID].position_min = MOTOR_HEAD_ANGLE_MIN;
  motor[MOTOR_HEAD_ID].load_limit = MOTOR_HEAD_LOAD_LIMIT;
  motor[MOTOR_NECK_VERTICAL_ID].position_max = MOTOR_NECK_VERTICAL_ANGLE_MAX;
  motor[MOTOR_NECK_VERTICAL_ID].position_min = MOTOR_NECK_VERTICAL_ANGLE_MIN;
  motor[MOTOR_NECK_VERTICAL_ID].load_limit = MOTOR_NECK_VERTICAL_LOAD_LIMIT;
  motor[MOTOR_NECK_HORIZON_ID].position_max = MOTOR_NECK_HORIZON_ANGLE_MAX;
  motor[MOTOR_NECK_HORIZON_ID].position_min = MOTOR_NECK_HORIZON_ANGLE_MIN;
  motor[MOTOR_NECK_HORIZON_ID].load_limit = MOTOR_NECK_HORIZON_LOAD_LIMIT;
}

void Motor_Init()
{
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  motor_size = 0;
  for(byte i = 0; i < MOTOR_SIZE; i++)
  {
    int model = get_model(i);
    if(model != 65535)
    {
        Dxl.jointMode(i); //jointMode() is to use position mode
        set_LED_enable(i, 0x01);
        set_torque_enable(i, 0x01);
        set_load_limit(i, 512);
        set_speed(i, 128);
        motor_list_id[motor_size] = i;
        motor[i].flag_motor = true;
        motor[i].model = model;
        motor[i].position_max = get_ccw_angle_limit(i);
        motor[i].position_min = get_cw_angle_limit(i);
        motor[i].load_limit = get_load_limit(i);
        motor[i].flag_moving = false;
        motor[i].flag_safety_lock = 0x00;
        motor_size += 1;
    }
    else
    {
        motor[i].flag_motor = false;
    }
  }
}

void Motor_Ping()
{
  motor_size = 0;
  for(byte i = 0; i < MOTOR_SIZE; i++)
  {
    int model = get_model(i);
    if(model != 65535)
    {
        motor_list_id[motor_size] = i;
        motor[i].flag_motor = true;
        motor[i].model = model;
        motor_size += 1;
    }
    else
    {
        motor[i].flag_motor = false;
    }
  }
}

void Motor_Scan()
{
  for(byte i = 0; i < motor_size; i++)
  {
    Motor_Scan(motor_list_id[i]);
  }
}

void Motor_Status_Scan(byte& id)
{
  Motor_Voltage_Detect(id);
  Motor_Temperature_Detect(id);
//  Motor_Torque_Detect(id);
}

void Motor_Scan(byte& id)
{
//  Motor_Moving_Detect(id);
  Motor_Load_Detect(id);
//  if(motor[id].flag_moving == 0x00) return;
  Motor_Position_Detect(id);
  Motor_Speed_Detect(id);
}

void Motor_Position_Set(byte& id, int data)
{
  set_position(id, data);
}

void Motor_Speed_Set(byte& id, int data)
{
  set_speed(id, data);
}

void Motor_Load_Limit_Set(byte& id, int data)
{
  set_load_limit(id, data);
}

void Motor_Torque_Enable_Set(byte& id, byte data)
{
  set_torque_enable(id, data);
}

void Motor_Position_Detect(byte& id)
{
  motor[id].position         = get_position(id);
  if(abs(motor[id].position - motor_old[id].position) > TOLERENCE_POSITION)
  {
    pub_position_set(id);
    USB_Publisher(com, com_size);
    motor_old[id].position = motor[id].position;
  }
  return;
}

void Motor_Speed_Detect(byte& id)
{
  motor[id].speed            = get_speed(id);
  if(abs(motor[id].speed - motor_old[id].speed) > TOLERENCE_SPEED)
  {
    pub_speed_set(id);
    USB_Publisher(com, com_size);
    motor_old[id].speed = motor[id].speed;
  }
  return;
}

void Motor_Load_Detect(byte& id)
{
  motor[id].load             = get_load(id);
  if(motor[id].load == 65535) return;
  motor[id].load = (motor[id].load > 1023)? motor[id].load - 1024 : motor[id].load;//CCW : negative; CW positive
  if((float)(abs(motor[id].load))        > (float)(LOAD_LIM)*(FATAL_RATIO))
  {
    if(motor[id].flag_safety_lock == 0)
    {
      set_LED_enable(id, 0x01);
      pub_fatal_set();
      pub_load_set(id);
      USB_Publisher(com, com_size);
    }
    motor[id].flag_safety_lock = motor[id].flag_safety_lock | LOCK_LOAD;
  }
  else if((float)(abs(motor[id].load))   > (float)(LOAD_LIM)*(WARN_RATIO))
  {
    if(motor[id].flag_safety_lock == 0)
    {
      set_LED_enable(id, 0x01);
      pub_warn_set();
      pub_load_set(id);
      USB_Publisher(com, com_size);
    }
    motor[id].flag_safety_lock = motor[id].flag_safety_lock | LOCK_LOAD;
  }
  else
  {
    if(motor[id].flag_safety_lock > 0)
    {
      motor[id].flag_safety_lock = motor[id].flag_safety_lock & (0xFF-LOCK_LOAD);
      if(motor[id].flag_safety_lock == 0)
        set_LED_enable(id, 0x00);
    }
    if(abs(abs(motor[id].load) - abs(motor_old[id].load)) > TOLERENCE_LOAD)
    {
      pub_load_set(id);
      USB_Publisher(com, com_size);
      motor_old[id].load = motor[id].load;
    }
  }
  return;
}

void Motor_Voltage_Detect(byte& id)
{
  motor[id].voltage          = get_voltage(id);
  if(motor[id].voltage == 255) return;
  if((float)(motor[id].voltage/10 - VOLTAGE_MIN)      < (float)(VOLTAGE_MAX - VOLTAGE_MIN)*(1-FATAL_RATIO))
  {
    if(motor[id].flag_safety_lock == 0)
    {
      set_LED_enable(id, 0x01);
      pub_fatal_set();
      pub_voltage_set(id);
      USB_Publisher(com, com_size);
    }
    motor[id].flag_safety_lock = motor[id].flag_safety_lock | LOCK_VOLTAGE;
  }
  else if((float)(motor[id].voltage/10 - VOLTAGE_MIN) < (float)(VOLTAGE_MAX - VOLTAGE_MIN)*(1-WARN_RATIO))
  {
    if(motor[id].flag_safety_lock == 0)
    {
      set_LED_enable(id, 0x01);
      pub_warn_set();
      pub_voltage_set(id);
      USB_Publisher(com, com_size);
    }
    motor[id].flag_safety_lock = motor[id].flag_safety_lock | LOCK_VOLTAGE;
  }
  else if((float)(motor[id].voltage/10 - VOLTAGE_MIN) > (float)(VOLTAGE_MAX - VOLTAGE_MIN)*(FATAL_RATIO))
  {
    if(motor[id].flag_safety_lock == 0)
    {
      set_LED_enable(id, 0x01);
      pub_fatal_set();
      pub_voltage_set(id);
      USB_Publisher(com, com_size);
    }
    motor[id].flag_safety_lock = motor[id].flag_safety_lock | LOCK_VOLTAGE;
  }
  else if((float)(motor[id].voltage/10 - VOLTAGE_MIN) > (float)(VOLTAGE_MAX - VOLTAGE_MIN)*(WARN_RATIO))
  {
    if(motor[id].flag_safety_lock == 0)
    {
      set_LED_enable(id, 0x01);
      pub_warn_set();
      pub_voltage_set(id);
      USB_Publisher(com, com_size);
    }
    motor[id].flag_safety_lock = motor[id].flag_safety_lock | LOCK_VOLTAGE;
  }
  else
  {
    if(motor[id].flag_safety_lock > 0)
    {
      motor[id].flag_safety_lock = motor[id].flag_safety_lock & (0xFF-LOCK_VOLTAGE);
      if(motor[id].flag_safety_lock == 0)
        set_LED_enable(id, 0x00);
    }
    if(abs(motor[id].voltage - motor_old[id].voltage) > TOLERENCE_VOLTAGE)
    {
      pub_voltage_set(id);
      USB_Publisher(com, com_size);
      motor_old[id].voltage = motor[id].voltage;
    }
  }
  return;
}

void Motor_Temperature_Detect(byte& id)
{
  motor[id].temperature      = get_temperature(id);
  if(motor[id].temperature == 255) return;
  if((float)(motor[id].temperature) > (float)(TEMPERATUE_LIM)*(FATAL_RATIO))
  {
    if(motor[id].flag_safety_lock == 0)
    {
      set_LED_enable(id, 0x01);
      pub_fatal_set();
      pub_temperature_set(id);
      USB_Publisher(com, com_size);
    }
    motor[id].flag_safety_lock = motor[id].flag_safety_lock | LOCK_TEMPERATURE;
  }
  else if((float)(motor[id].temperature) > (float)(TEMPERATUE_LIM)*(WARN_RATIO))
  {
    if(motor[id].flag_safety_lock == 0)
    {
      set_LED_enable(id, 0x01);
      pub_warn_set();
      pub_temperature_set(id);
      USB_Publisher(com, com_size);
    }
    motor[id].flag_safety_lock = motor[id].flag_safety_lock | LOCK_TEMPERATURE;
  }
  else
  {
    if(motor[id].flag_safety_lock > 0)
    {
      motor[id].flag_safety_lock = motor[id].flag_safety_lock & (0xFF-LOCK_TEMPERATURE);
      if(motor[id].flag_safety_lock == 0)
        set_LED_enable(id, 0x00);
    }
    if(abs(motor[id].temperature - motor_old[id].temperature) > TOLERENCE_TEMPERATURE)
    {
      pub_temperature_set(id);
      USB_Publisher(com, com_size);
      motor_old[id].temperature = motor[id].temperature;
    }
  }
  return;
}

void Motor_Torque_Detect(byte& id)
{
  motor[id].torque             = get_torque_enable(id);
  if(motor[id].torque != motor_old[id].torque)
  {
    pub_torque_set(id);
    USB_Publisher(com, com_size);
    motor_old[id].torque = motor[id].torque;
  }
  return;
}

void Motor_Moving_Detect(byte& id)
{
  motor[id].flag_moving      = is_moving(id);
  return;
}

void USB_Interrupt(byte* buffer, byte nCount)
{
  digitalWrite(BLUE_LED_485EXP, 0); 
  ping(buffer, nCount);
  read(buffer, nCount);
  write(buffer, nCount);
  digitalWrite(BLUE_LED_485EXP, 1); 
}

void USB_Publisher(byte* buffer, byte nCount)
{
  digitalWrite(GREEN_LED_485EXP, 0); 
  for(byte i = 0; i < nCount; i++)  //printf_SerialUSB_Buffer[N]_receive_Data 
    SerialUSB.print((char)buffer[i]);
  SerialUSB.print("\n");
  digitalWrite(GREEN_LED_485EXP, 1);
}

void USB_Publisher(char* buffer, byte nCount)
{
  digitalWrite(GREEN_LED_485EXP, 0); 
  for(byte i = 0; i < nCount; i++)  //printf_SerialUSB_Buffer[N]_receive_Data 
    SerialUSB.print(buffer[i]);
  SerialUSB.print("\n");
  digitalWrite(GREEN_LED_485EXP, 1);
}
