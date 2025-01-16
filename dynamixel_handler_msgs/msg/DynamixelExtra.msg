uint16[] id_list

# 書き込めない
string[] model
uint16[] firmware_version

# 書き込める
string[] protocol_version
DynamixelExtraDrivemode drive_mode
DynamixelExtraShutdown shutdown
DynamixelExtraRestureconfig restore_configuration 
float64[] homing_offset_deg
float64[] return_delay_time_us
float64[] bus_watchdog_ms          

#　機能
DynamixelExtraLed led                    
uint8[] shadow_id
float64[] moving_threshold_deg_s
uint8[] status_return_level    

# 状況
DynamixelExtraMovingstatus moving_status      
float64[] realtime_tick_us          
uint8[] moving                 
uint8[] registered_instruction 

# 追加での機能
bool[] reboot