import cantools
import can
import time
import random
from typing import Dict, Any

def load_dbc(dbc_file: str) -> cantools.database.Database:
    """Load the DBC file."""
    try:
        return cantools.database.load_file(dbc_file)
    except Exception as e:
        print(f"Error loading DBC file: {e}")
        raise

def get_cycle_times(db: cantools.database.Database) -> Dict[int, float]:
    """Extract cycle times from DBC file attributes."""
    cycle_times = {}
    for message in db.messages:
        cycle_time = message.cycle_time / 1000.0 if message.cycle_time else 0.1  # Default to 100ms if not specified
        cycle_times[message.frame_id] = cycle_time
    return cycle_times

def assign_signal_values(db: cantools.database.Database) -> Dict[str, Dict[str, Any]]:
    """Define signal values for each message. Modify this to set desired values."""
    signal_values = {
        'Vehicle_Mileage2': {
            'Vehicle_Mileage2_MsgCntr': 0,  # Counter (0-15)
            'Vehicle_TRIP1': 1000,  # cm
            'Vehicle_Remote_Mileage1': 50.0  # km
        },
        'Vehicle_Mileage1': {
            'Vehicle_Mileage1_MsgCntr': 0,
            'Vchicle_AD_Mileage1': 100.0,
            'Vehicle_ODO1': 200.0
        },
        'BMS_2B1h': {
            'BMS_Battery_Capacity_Kwh': 50.0,
            'BMS_Battery_Capacity_Ah': 100,
            'BMS_HVBatLowestTemCellNum': 1,
            'BMS_HVBatLowestTem': 25,
            'BMS_HVBatHighestTemCellNum': 2,
            'BMS_HVBatHighestTem': 30
        },
        'BMS_2A1h': {
            'BMS_HVBatCellVolDiff': 10,
            'BMS_HVBatHighestVolCellNum': 1,
            'BMS_HVBatLowestVolCellNum': 2,
            'BMS_HVBatHighestCellVol': 4200,
            'BMS_HVBatLowestCellVol': 4100
        },
        'BMS_A0h': {
            'BMS_HVDisplaySOH': 95,
            'BMS_Sys_Flt': 0,
            'BMS_Charge_StsCc': 0,
            'BMS_Charge_StsCc2': 0,
            'BMS_Sys_Sts': 0,
            'BMS_HVBatSOC': 80.0,
            'BMS_HVBatCrnt': 10.0,
            'BMS_HVBatVol': 400.0
        },
        'VCU_Vehicle_Error_Status': {
            'VCU_CAN2_Fault': 0,
            'VCU_CAN1_Fault': 0,
            'VCU_CAN0_Fault': 0,
            'VCU_PwrCtrl_Fault': 0,
            'VCU_EPBActinWhenEBSMF': 0,
            'VCU_EEPROM_Fault': 0,
            'VCU_EBSActinWhenEPBMF': 0,
            'Error_Code': 0
        },
        'DBS_Status2': {
            'DBS_WarringCode': 0,
            'DBS_CheckSum2': 0,
            'DBS_Fault_Code': 0,
            'DBS_RollingCounter2': 0
        },
        'TPMS_Status': {
            'ID_Match_State': 0,
            'Hight_Temperature_Warning': 0,
            'Sensor_state': 0,
            'Sensor_Power_State': 0,
            'Pressure_Warning': 0,
            'Pressure_Unbalance': 0,
            'Tire_Temperature': 25,
            'Tire_Pressure': 250.0,
            'Tire_Num': 0,
            'Tire_Leak_State': 0
        },
        'DBS_Status': {
            'DBS_EstopFlag': 0,
            'DBS_PedaiFlag': 0,
            'BrakePressureReqACK': 0,
            'DBS_Work_Mode': 0,
            'DBS_RollingCounter': 0,
            'DBS_Ref_Iq': 0.0,
            'DBS_Park_Warning': 0,
            'DBS_PeadalOpening': 0,
            'DBS_CheckSum': 0,
            'DBS_HP_pressure': 0.0,
            'DBS_System_Status': 0
        },
        'VCU_DBS_Req': {
            'VCU_DBS_Work_Mode': 0,
            'VCU_ABS_Active': 0,
            'VCU_DBS_Pressure_Request': 0.0,
            'VCU_DBS_Request_Flag': 0
        },
        'Vehicle_Odometer_Status': {
            'Vehicle_Odometer_MsgCntr': 0,
            'Vehicle_AD_Mileage': 100.0,
            'Vehicle_Remote_Mileage': 50.0,
            'Vehicle_TRIP': 10.0,
            'Vehicle_ODO': 200.0
        },
        'VCU_Vehicle_HVBat_Status': {
            'Vehicle_Poweroff_Channel': 0,
            'Vehicle_Poweroff_Countdown_Time': 0.0,
            'Battery_Work_State': 0,
            'Vehicle_Soc': 80,
            'Vehicle_HVBat_MsgCntr': 0,
            'High_Voltage_Battery_Voltage': 400.0,
            'High_Voltage_Battery_MaxTem': 30,
            'High_Voltage_Battery_Current': 10.0
        },
        'VCU_RR_Wheel_Status': {
            'RR_Tire_Leak_State': 0,
            'RR_Wheel_Status_MsgCntr': 0,
            'RR_Tire_Temperature': 25,
            'RR_Tire_Pressure': 250.0,
            'RR_Sensor_state': 0,
            'RR_Pressure_Warning': 0,
            'RR_WhlSpd': 50.0,
            'RR_Reserved': 0
        },
        'VCU_FR_Wheel_Status': {
            'FR_Tire_Leak_State': 0,
            'FR_Pressure_Warning': 0,
            'FR_Wheel_Status_MsgCntr': 0,
            'FR_Tire_Temperature': 25,
            'FR_Tire_Pressure': 250.0,
            'FR_Sensor_state': 0,
            'FR_WhlSpd': 50.0,
            'FR_Reserved': 0
        },
        'VCU_RL_Wheel_Status': {
            'RL_Pressure_Warning': 0,
            'RL_Tire_Temperature': 25,
            'RL_Tire_Pressure': 250.0,
            'RL_Sensor_state': 0,
            'RL_Tire_Leak_State': 0,
            'RL_WhlSpd': 50.0,
            'RL_Reserved': 0,
            'RL_Wheel_Status_MsgCntr': 0
        },
        'VCU_FL_Wheel_Status': {
            'FL_Sensor_state': 0,
            'FL_Tire_Temperature': 25,
            'FL_Pressure_Warning': 0,
            'FL_Tire_Leak_State': 0,
            'FL_Tire_Pressure': 250.0,
            'FL_WhlSpd': 50.0,
            'FL_Reserved': 0,
            'FL_Wheel_Status_MsgCntr': 0
        },
        'VCU_Vehicle_Status_1': {
            'Vehicle_Range': 300,
            'Drive_Mode_State': 0,
            'EPB_Status': 0,
            'Accelerator_Pedal_Status': 0,
            'Brake_Pedal_Status': 0,
            'Vehicle_Status_1_MsgCntr': 0,
            'Vehicle_Gear': 0
        },
        'VCU_Vehicle_Status_2': {
            'Vehicle_Status_2_MsgCntr': 0,
            'Vehicle_Steering_Angle': 0.0,
            'Vehicle_Brake_Pressure': 0.0,
            'Vehicle_Speed': 50.0
        },
        'VCU_Vehicle_Diagnosis': {
            'Horn_2_State': 0,
            'VCU_VehRdy': 1,
            'ADS_Light_State': 0,
            'KERS_LimitedState': 0,
            'Oil_pot_State': 0,
            'Business_Relay_State': 0,
            'Relay_4G_State': 0,
            'Radar_Relay_State': 0,
            'Orin_Relay_State': 0,
            'Motor_Temp_State': 0,
            'Fog_Light_state': 0,
            'Power_Button_State': 0,
            'EPB_Button_State': 0,
            'Motor_Torque_Limit_State': 0,
            'B_Press_Switch_Collision_State': 0,
            'Remo_Touch_Switch_Disable_State': 0,
            'F_Press_Switch_Collision_State': 0,
            'B_Touch_Switch_Disable_State': 0,
            'L_Touch_Switch_Disable_State': 0,
            'R_Touch_Switch_Disable_State': 0,
            'F_Touch_Switch_Disable_State': 0,
            'R_Touch_Switch_Collision_State': 0,
            'L_Touch_Switch_Collision_State': 0,
            'AD_FaultCode': 0,
            'EPB_Diagnosis': 0,
            'Move_Switch': 0,
            'LowBeam_State': 0,
            'Reversing_Lights_State': 0,
            'Tire_Sensor_State': 0,
            'Brake_Light_State': 0,
            'Vehicle_Fault_Grade': 0,
            'EPS_State': 0,
            'Vehicle_Diagnosis_MsgCntr': 0,
            'Horn_1_State': 0,
            'HighBeam_State': 0,
            'Right_Turn_Light_State': 0,
            'Left_Turn_Light_State': 0,
            'B_Touch_Switch_Collision_State': 0,
            'F_Touch_Switch_Collision_State': 0,
            'BMS_State': 0,
            'Emergency_Button_State': 0,
            'DBS_State': 0,
            'AD_State': 0,
            'Remote_State': 0,
            'Motor_State': 0
        },
        'EPS_Status': {
            'EPS_Fault_Code': 0,
            'EPS_Current': 0.0,
            'EPS_Fault_Grade': 0,
            'EPS_Temperature': 25,
            'EPS_Angle_Spd': 50,
            'EPS_Calibration_Status': 0,
            'EPS_StrAngle_Act': 0.0,
            'EPS_Fault': 0,
            'EPS_Work_Mode_Status': 0
        },
        'VCU_EPS_Req': {
            'VCU_Request_EPS_Angle_Speed': 50,
            'VCU_Request_EPS_Angle_Calibrate': 0,
            'VCU_Req_EPS_Target_Angle': 0.0,
            'VCU_EPS_CtrlEnable': 0
        },
        'Remote_Control_Shake': {
            'Remote_X1': 0.0,
            'Remote_Y1': 0.0,
            'Remote_Y2': 0.0,
            'Remote_X2': 0.0
        },
        'Remote_Control_IO': {
            'Remote_A': 0,
            'Remote_D': 0,
            'Remote_C': 0,
            'Remote_B': 0,
            'Remote_F': 0,
            'Remote_E': 0
        },
        'EPB_Status': {
            'EPB_MANUAL_PARKING_KEY': 0,
            'EPB_FINAL_STATES_R': 0,
            'EPB_FINAL_STATES_L': 0,
            'EPB_FAULT': 0
        },
        'VCU_EPB_Req': {
            'VCU_EPB_Parking_Request_R': 0,
            'VCU_EPB_Parking_Request_L': 0,
            'VCU_EPB_Parking_Request_ALL': 0,
            'VCU_EPB_Parking_Flag_R': 0,
            'VCU_EPB_Parking_Flag_L': 0,
            'VCU_EPB_Parking_Flag_ALL': 0,
            'VCU_EPB_Clamping_force_R': 5000,
            'VCU_EPB_Clamping_force_L': 5000
        },
        'MCU_DrvMotSt': {
            'Motor_ActIdc': 0,
            'Motor_ActSpeed': 0.0,
            'Motor_ActGear': 0,
            'Motor_ActTorque': 0.0,
            'Motor_FalutCode': 0,
            'Motor_Temp': 25
        },
        'VCU_MCU_Req': {
            'VCU_SoftReset': 0,
            'VCU_ActiveHVReleaseReq': 0,
            'VCU_MotNegSpdLmt': -9000,
            'VCU_MotNegTrqLmt': -315,
            'VCU_ClampingBrakeReq': 0,
            'VCU_VehHVReady': 0,
            'VCU_ModeReq': 0,
            'VCU_TargetSpdReq': 0.0,
            'VCU_MotPosSpdLmt': 0,
            'VCU_MotPosTrqLmt': 0,
            'VCU_TargetTqReq': 0.0
        },
        'VCU_Version': {
            'Day': 16,
            'Month': 6,
            'Year': 2025,
            'Byte5': 0,
            'Byte4': 0,
            'Byte3': 0,
            'Byte2': 0,
            'Byte1': 0
        },
        'VCU_Vehicle_PwrCtrl_Status': {
            'VCU_LVMuteChargeSt': 0,
            'VCU_AbnPwrOff': 0,
            'VCU_PwrSt': 0,
            'VCU_PUM_FSM_Pointer': 0,
            'VCU_VehRdyDiagEnable': 0,
            'VCU_LowBattVolt': 12.0,
            'VCU_LVPwrActReq': 0,
            'VCU_HVPwrActReq': 0,
            'VCU_WeakUpSig': 0
        }
    }
    return signal_values

def main():
    # Configuration
    dbc_file = "Yokee_CAN2_VCU_500k.dbc"
    can_interface = "can0"
    
    # Load DBC file
    db = load_dbc(dbc_file)
    
    # Get cycle times from DBC
    cycle_times = get_cycle_times(db)
    
    # Initialize CAN bus
    try:
        bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
    except Exception as e:
        print(f"Error initializing CAN bus: {e}")
        return
    
    # Assign signal values
    signal_values = assign_signal_values(db)
    
    # Track last send time for each message
    last_send_times = {msg.frame_id: 0 for msg in db.messages}
    
    print(f"Publishing CAN messages to {can_interface}...")
    
    try:
        while True:
            current_time = time.time()
            for message in db.messages:
                # Check if it's time to send this message
                if current_time - last_send_times[message.frame_id] >= cycle_times[message.frame_id]:
                    try:
                        # Get signal values for this message
                        data = signal_values.get(message.name, {})
                        
                        # Increment counter signals if present
                        for signal in message.signals:
                            if "MsgCntr" in signal.name or "RollingCounter" in signal.name:
                                data[signal.name] = (data.get(signal.name, 0) + 1) % (signal.maximum + 1)
                        
                        # Encode the message
                        encoded_data = message.encode(data)
                        
                        # Create CAN message
                        can_msg = can.Message(
                            arbitration_id=message.frame_id,
                            data=encoded_data,
                            is_extended_id=False
                        )
                        
                        # Send the message
                        bus.send(can_msg)
                        last_send_times[message.frame_id] = current_time
                        print(f"Sent {message.name} (ID: {message.frame_id})")
                    except Exception as e:
                        print(f"Error encoding/sending {message.name}: {e}")
            
            # Sleep briefly to avoid CPU overload
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()