uint16[] id_list

# 固定情報
string[] model
uint16[] model_number
uint16[] protocol_type
uint16[] firmware_version

# 状況
float64[] realtime_tick_s          
DynamixelExtraMovingstatus moving_status      
bool[] moving

#　設定
float64[] return_delay_time_us
DynamixelExtraDrivemode drive_mode
uint16[] shadow_id
float64[] homing_offset_deg
float64[] moving_threshold_deg_s
DynamixelExtraRestoreconfig restore_configuration 
float64[] pwm_slope_percent
DynamixelExtraShutdown shutdown

# 機能
DynamixelExtraLed led                    
float64[] bus_watchdog_ms          
bool[] reboot             
uint16[] change_id
