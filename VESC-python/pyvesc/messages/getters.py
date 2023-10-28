# NOTE: this is for VESC modified firmware 3.33  ( https://github.com/raess1/vesc-FW )

from pyvesc.messages.base import VESCMessage


class GetConfig(metaclass=VESCMessage):
    id = 14 # COMM_GET_MCCONF
    can_id = None
    
    fields = [
            ('pwm_mode', 'B', 1),            
            ('comm_mode', 'B', 1),            
            ('motor_type', 'B', 1),            
            ('sensor_mode', 'B', 1),            
            ('l_current_max', 'f', 1),                        
            ('l_current_min', 'f', 1),                        
            ('l_in_current_max', 'f', 1),                        
            ('l_in_current_min', 'f', 1),                        
            ('l_abs_current_max', 'f', 1),                        
            ('l_min_erpm', 'f', 1),                        
            ('l_max_erpm', 'f', 1),                        
            ('l_erpm_start', 'f', 1),                        
            ('l_max_erpm_fbrake', 'f', 1),                        
            ('l_max_erpm_fbrake_cc', 'f', 1),                        
            ('l_min_vin', 'f', 1),                        
            ('l_max_vin', 'f', 1),                        
            ('l_battery_cut_start', 'f', 1),                        
            ('l_battery_cut_end', 'f', 1),                        
            ('l_slow_abs_current', 'B', 1),                        
            ('l_temp_fet_start', 'f', 1),                        
            ('l_temp_fet_end', 'f', 1),                        
            ('l_temp_motor_start', 'f', 1),                        
            ('l_temp_motor_end', 'f', 1),                        
            ('l_temp_accel_dec', 'f', 1),                        
            ('l_min_duty', 'f', 1),                        
            ('l_max_duty', 'f', 1),                        
            ('l_watt_max', 'f', 1),                        
            ('l_watt_min', 'f', 1),                        
            ('sl_min_erpm', 'f', 1),                        
            ('sl_min_erpm_cycle_int_limit', 'f', 1),                        
            ('sl_max_fullbreak_current_dir_change', 'f', 1),                        
            ('sl_cycle_int_limit', 'f', 1),                        
            ('sl_phase_advance_at_br', 'f', 1),                        
            ('sl_cycle_int_rpm_br', 'f', 1),                                    
            ('sl_bemf_coupling_k', 'f', 1),                                    
            ('hall_table_0', 'b', 1),                                    
            ('hall_table_1', 'b', 1),                                    
            ('hall_table_2', 'b', 1),                                    
            ('hall_table_3', 'b', 1),                                    
            ('hall_table_4', 'b', 1),                                    
            ('hall_table_5', 'b', 1),                                    
            ('hall_table_6', 'b', 1),                                    
            ('hall_table_7', 'b', 1),                                    
            ('hall_sl_erpm', 'f', 1),                                    
            ('foc_current_kp', 'f', 1),                                    
            ('foc_current_ki', 'f', 1),                                                
            ('foc_f_sw', 'f', 1),                                                
            ('foc_dt_us', 'f', 1),                                                
            ('foc_encoder_inverted', 'B', 1),                                                
            ('foc_encoder_offset', 'f', 1),                                                
            ('foc_encoder_ratio', 'f', 1),                                                
            ('foc_sensor_mode', 'B', 1),                                                
            ('foc_pll_kp', 'f', 1),                                                
            ('foc_pll_ki', 'f', 1),                                                
            ('foc_motor_l', 'f', 1),                                                
            ('foc_motor_r', 'f', 1),                                                
            ('foc_motor_flux_linkage', 'f', 1),                                                
            ('foc_observer_gain', 'f', 1),                                                
            ('foc_observer_gain_slow', 'f', 1),                                                
            ('foc_duty_dowmramp_kp', 'f', 1),                                                
            ('foc_duty_dowmramp_ki', 'f', 1),                                                          
            ('foc_openloop_rpm', 'f', 1),                                                
            ('foc_sl_openloop_hyst', 'f', 1),                                                
            ('foc_sl_openloop_time', 'f', 1),                                                
            ('foc_sl_d_current_duty', 'f', 1),                                                
            ('foc_sl_d_current_factor', 'f', 1),                                                
            ('foc_hall_table_0', 'B', 1),                                                
            ('foc_hall_table_1', 'B', 1),                                                
            ('foc_hall_table_2', 'B', 1),                                                
            ('foc_hall_table_3', 'B', 1),                                                
            ('foc_hall_table_4', 'B', 1),                                                
            ('foc_hall_table_5', 'B', 1),                                                
            ('foc_hall_table_6', 'B', 1),                                                
            ('foc_hall_table_7', 'B', 1),                                                            
            ('foc_sl_erpm', 'f', 1),                                                
            ('foc_sample_v0_v7', 'B', 1),                                                
            ('foc_sample_high_current', 'B', 1),                                                
            ('foc_sat_comp', 'f', 1),                                                
            ('foc_temp_comp', 'B', 1),                                                
            ('foc_temp_comp_base_temp', 'f', 1),                                                
            ('s_pid_kp', 'f', 1),                                                
            ('s_pid_ki', 'f', 1),                                                
            ('s_pid_kd', 'f', 1),                                                
            ('s_pid_min_erpm', 'f', 1),                                                
            ('s_pid_allow_braking', 'B', 1),                                                
            ('p_pid_kp', 'f', 1),                                                
            ('p_pid_ki', 'f', 1),                                                
            ('p_pid_kd', 'f', 1),                                                
            ('p_pid_ang_div', 'f', 1),                                                
            ('cc_startup_boost_duty', 'f', 1),                                                
            ('cc_min_current', 'f', 1),                                                
            ('cc_gain', 'f', 1),                                                
            ('cc_ramp_step_max', 'f', 1),                                                
            ('m_fault_stop_time_ms', 'i', 1),                                                
            ('m_duty_ramp_step', 'f', 1),                                                
            ('m_current_backoff_gain', 'f', 1),                                                
            ('m_encoder_counts', 'I', 1),                                                
            ('m_sensor_port_mode', 'B', 1),                                                
            ('m_invert_direction', 'B', 1),                                                
            ('m_drv8301_oc_mode', 'B', 1),                                                
            ('m_drv8301_oc_adj', 'B', 1),                                                
            ('m_bldc_f_sw_min', 'f', 1),                                                
            ('m_bldc_f_sw_max', 'f', 1),                                                
            ('m_dc_f_sw', 'f', 1),                                                
            ('m_ntc_motor_beta', 'f', 1)                                                            
    ]


class GetPrint(metaclass=VESCMessage):        
    id = 21 # COMM_PRINT
    can_id = None

    fields = [
            ('msg', 's')            
    ]

  

class GetFirmwareVersion(metaclass=VESCMessage):        
    id = 0 # COMM_FW_VERSION 
    can_id = None

    fields = [
            ('version_major', 'b', 1),
            ('version_minor', 'b', 1)
    ]



class GetValues(metaclass=VESCMessage):    
    """ Gets internal sensor data
    """
    id = 4 #  COMM_GET_VALUES
    can_id = None

    fields = [
            ('temp_fet_filtered', 'H', 10),
            ('temp_motor_filtered', 'e', 1),
            ('avg_motor_current', 'i', 100),
            ('avg_input_current', 'i', 100),
            ('avg_id', 'f', 1),
            ('avg_iq', 'f', 1),
            ('duty_cycle_now', 'h', 10),
            ('rpm', 'i', 1),
            ('input_voltage', 'h', 10),
            ('amp_hours',  'i', 10000),
            ('amp_hours_charged',  'i', 10000),
            ('watt_hours', 'i', 10000),
            ('watt_hours_charged', 'i', 10000),
            ('tachometer_value', 'i', 1),
            ('tachometer_abs_value', 'i', 1),
            ('fault', 'b', 1)
    ]


class GetRotorPosition(metaclass=VESCMessage):
    """ Gets rotor position data
    
    Must be set to DISP_POS_MODE_ENCODER or DISP_POS_MODE_PID_POS (Mode 3 or 
    Mode 4). This is set by SetRotorPositionMode (id=21).
    """
    id = 22  # COMM_ROTOR_POSITION
    can_id = None

    fields = [
            ('rotor_pos', 'i', 100000)
    ]

class GetRotorPositionCumulative(metaclass=VESCMessage):    
    id = 38  # COMM_ROTOR_POSITION_CUMULATIVE
    can_id = None

    fields = [
            ('rotor_pos', 'i', 1)
    ]

class GetDetectEncoder(metaclass=VESCMessage):    
    id = 27  # COMM_DETECT_ENCODER
    can_id = None
    fields = [
            ('offset', 'f', 1000000),
            ('ratio', 'f', 1000000)
    ]
        