#include <DynamixelSDK.h>
#include <IMU.h>
#include <stdarg.h>
#include <Dynamixel2Arduino.h>



#define DXL_USB_VER           20240114

#define CMD_PORT              Serial      // USB
#define DBG_PORT              Serial2     // UART1
#define DXL_PORT              Serial3
#define DXL_BAUD              1000000


#define DXL_LED_RX            BDPIN_LED_USER_1
#define DXL_LED_TX            BDPIN_LED_USER_2

#define DXL_POWER_DISABLE()   digitalWrite(BDPIN_DXL_PWR_EN, LOW);
#define DXL_POWER_ENABLE()    digitalWrite(BDPIN_DXL_PWR_EN, HIGH);

#define DXL_TX_BUFFER_LENGTH  1024


uint8_t tx_buffer[DXL_TX_BUFFER_LENGTH];
uint8_t rx_buffer[DXL_TX_BUFFER_LENGTH];


static int rx_led_count = 0;
static int tx_led_count = 0;

static int rx_led_update_time;
static int tx_led_update_time;

static uint32_t update_time[8];


static uint32_t rx_data_cnt = 0;
static uint32_t tx_data_cnt = 0;

static uint32_t rx_bandwidth = 0;
static uint32_t tx_bandwidth = 0;

uint32_t usb_baud;

// Dynamixel2Arduino
#define DXL_MODEL_NUM 1220	//XC330-T288-T
#define DXL_PROTOCOL_VER_2_0 2.0
#define DXL_SLAVE_ID 40
class DXLPortHandler2 : public DXLPortHandler{
   int available_length = 0;
   int read_length = 0;
   uint8_t *buffer;
  public:
  using DXLPortHandler::DXLPortHandler;
  // // override
    void begin() override{}
    void end() override{}
   int available(void) override{
    return (available_length - read_length);
   }
   int read(void) override{
      if(available() <= 0){
        return 0;
      }
      int buf = buffer[read_length++];                                                                        
      return buf;
   }
   size_t write(uint8_t data) override{
    return CMD_PORT.write(data);
   }
   size_t write(uint8_t *buf, size_t len) override{
    return CMD_PORT.write(buf, len);
   }
  void update(uint8_t *data, int length){
    read_length = 0;
    buffer = data;
    available_length = length;
  }
};
DXLPortHandler2 slave_port;
DYNAMIXEL::Slave dxl(slave_port, DXL_MODEL_NUM);
// ControlTable
typedef struct {
    uint16_t address;
    uint16_t length;
    float data;
} ControlitemF;
typedef struct {
    uint16_t address;
    uint16_t length;
    int16_t data;
} ControlitemI16;
typedef struct {
    uint16_t address;
    uint16_t length;
    uint8_t data;
} ControlitemU1;

typedef struct {
  ControlitemU1 calib = {64, 1, 0};
  ControlitemI16 gyro_x = {76, 2, 0};
  ControlitemI16 gyro_y = {78, 2, 0};
  ControlitemI16 gyro_z = {80, 2, 0};
  ControlitemI16 acc_x = {82, 2, 0};
  ControlitemI16 acc_y = {84, 2, 0};
  ControlitemI16 acc_z = {86, 2, 0};
  ControlitemF quat_x = {88, 4, 0};
  ControlitemF quat_y = {92, 4, 0};
  ControlitemF quat_z = {96, 4, 0};
  ControlitemF quat_w = {100, 4, 0};
} ControlTable;

ControlTable control_table;

// IMU
cIMU    IMU;
#define IMU_UPDATE_MS 10


void setup()
{
  CMD_PORT.begin(115200);
  DBG_PORT.begin(57600);
  DXL_PORT.begin(DXL_BAUD);
  slave_port.setOpenState(true);

  pinMode( BDPIN_DXL_PWR_EN, OUTPUT );
  pinMode( DXL_LED_RX, OUTPUT );
  pinMode( DXL_LED_TX, OUTPUT );

  digitalWrite(DXL_LED_TX, HIGH);
  digitalWrite(DXL_LED_RX, HIGH);

  drv_dxl_tx_enable(FALSE);

  // IMU
  IMU.begin();
  calibrationGyro();

  //Dynamixel2Arduino
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER_2_0);
  dxl.setFirmwareVersion(1);
  dxl.setID(DXL_SLAVE_ID);
  dxl.addControlItem(control_table.calib.address, (uint8_t*)&control_table.calib.data, control_table.calib.length);
  dxl.addControlItem(control_table.gyro_x.address, (uint8_t*)&control_table.gyro_x.data, control_table.gyro_x.length);
  dxl.addControlItem(control_table.gyro_y.address, (uint8_t*)&control_table.gyro_y.data, control_table.gyro_y.length);
  dxl.addControlItem(control_table.gyro_z.address, (uint8_t*)&control_table.gyro_z.data, control_table.gyro_z.length);
  dxl.addControlItem(control_table.acc_x.address, (uint8_t*)&control_table.acc_x.data, control_table.acc_x.length);
  dxl.addControlItem(control_table.acc_y.address, (uint8_t*)&control_table.acc_y.data, control_table.acc_y.length);
  dxl.addControlItem(control_table.acc_z.address, (uint8_t*)&control_table.acc_z.data, control_table.acc_z.length);
  dxl.addControlItem(control_table.quat_x.address, (uint8_t*)&control_table.quat_x.data, control_table.quat_x.length);
  dxl.addControlItem(control_table.quat_y.address, (uint8_t*)&control_table.quat_y.data, control_table.quat_y.length);
  dxl.addControlItem(control_table.quat_z.address, (uint8_t*)&control_table.quat_z.data, control_table.quat_z.length);
  dxl.addControlItem(control_table.quat_w.address, (uint8_t*)&control_table.quat_w.data, control_table.quat_w.length);
  // Add interrupt callback functions to process read packet.
  // dxl.setReadCallbackFunc(read_callback_func);

  DXL_POWER_ENABLE();
}

void loop() {
  // Update IMU
  if( IMU.update() > 0 ){
    // calibration
    if(control_table.calib.data == 1){
      control_table.calib.data = 0;
      calibrationGyro();
    }
    // update imu data
    control_table.gyro_x.data = IMU.gyroRaw[0];
    control_table.gyro_y.data = IMU.gyroRaw[1];
    control_table.gyro_z.data = IMU.gyroRaw[2];
    control_table.acc_x.data = IMU.accRaw[0];
    control_table.acc_y.data = IMU.accRaw[1];
    control_table.acc_z.data = IMU.accRaw[2];
    control_table.quat_x.data = IMU.quat[0];
    control_table.quat_y.data = IMU.quat[1];
    control_table.quat_z.data = IMU.quat[2];
    control_table.quat_w.data = IMU.quat[3];
  }

  update_dxl();
  update_led();

  if( CMD_PORT.getBaudRate() != DXL_PORT.getBaudRate() )
  {
    DXL_PORT.begin(CMD_PORT.getBaudRate());
  }

  if( (millis()-update_time[1]) > 1000 )
  {
    update_time[1] = millis();

    tx_bandwidth = tx_data_cnt;
    rx_bandwidth = rx_data_cnt;

    tx_data_cnt = 0;
    rx_data_cnt = 0;
  }
}


void update_dxl()
{
  int length;
  int i;


  //-- USB -> DXL
  length = CMD_PORT.available();
  if( length > 0 )
  {
    drv_dxl_tx_enable(TRUE);
    for(i=0; i<length; i++ ) {
      rx_buffer[i] = CMD_PORT.read();
      DXL_PORT.write(rx_buffer[i]);
      DXL_PORT.flush();
    }
    drv_dxl_tx_enable(FALSE);

    tx_led_count = 3;

    tx_data_cnt += length;
    slave_port.update(rx_buffer, length);
    dxl.processPacket();
  }

  //-- DXL -> USB
  length = DXL_PORT.available();
  if( length > 0 )
  {
    if( length > DXL_TX_BUFFER_LENGTH )
    {
      length = DXL_TX_BUFFER_LENGTH;
    }
    for(i=0; i<length; i++ )
    {
      tx_buffer[i] = DXL_PORT.read();
    }
    CMD_PORT.write(tx_buffer, length);

    rx_led_count = 3;
    rx_data_cnt += length;
  }
}


void update_led()
{
  if( (millis()-tx_led_update_time) > 50 )
  {
    tx_led_update_time = millis();

    if( tx_led_count )
    {
      digitalWrite(DXL_LED_TX, !digitalRead(DXL_LED_TX));
      tx_led_count--;
    }
    else
    {
      digitalWrite(DXL_LED_TX, HIGH);
    }
  }

  if( (millis()-rx_led_update_time) > 50 )
  {
    rx_led_update_time = millis();

    if( rx_led_count )
    {
      digitalWrite(DXL_LED_RX, !digitalRead(DXL_LED_RX));
      rx_led_count--;
    }
    else
    {
      digitalWrite(DXL_LED_RX, HIGH);
    }
  }
}

void calibrationGyro(){
  IMU.SEN.gyro_cali_start();
  while( IMU.SEN.gyro_cali_get_done() == false ) {
    IMU.update();
  }
}