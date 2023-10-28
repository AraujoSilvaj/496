# NOTE: this is for VESC modified firmware 3.33  ( https://github.com/raess1/vesc-FW ) 

from pyvesc.messages.base import VESCMessage


class SetConfig(metaclass=VESCMessage):
    id = 13 # COMM_SET_MCCONF
    can_id = None
    
    fields = [
            ('pwm_mode', 'B'),            
            ('comm_mode', 'B'),            
            ('motor_type', 'B'),            
            ('sensor_mode', 'B'),            
            ('l_current_max', 'f'),                        
            ('l_current_min', 'f'),                        
            ('l_in_current_max', 'f'),                        
            ('l_in_current_min', 'f'),                        
            ('l_abs_current_max', 'f'),                        
            ('l_min_erpm', 'f'),                        
            ('l_max_erpm', 'f'),                        
            ('l_erpm_start', 'f'),                        
            ('l_max_erpm_fbrake', 'f'),                        
            ('l_max_erpm_fbrake_cc', 'f'),                        
            ('l_min_vin', 'f'),                        
            ('l_max_vin', 'f'),                        
            ('l_battery_cut_start', 'f'),                        
            ('l_battery_cut_end', 'f'),                        
            ('l_slow_abs_current', 'B'),                        
            ('l_temp_fet_start', 'f'),                        
            ('l_temp_fet_end', 'f'),                        
            ('l_temp_motor_start', 'f'),                        
            ('l_temp_motor_end', 'f'),                        
            ('l_temp_accel_dec', 'f'),                        
            ('l_min_duty', 'f'),                        
            ('l_max_duty', 'f'),                        
            ('l_watt_max', 'f'),                        
            ('l_watt_min', 'f'),                        
            ('sl_min_erpm', 'f'),                        
            ('sl_min_erpm_cycle_int_limit', 'f'),                        
            ('sl_max_fullbreak_current_dir_change', 'f'),                        
            ('sl_cycle_int_limit', 'f'),                        
            ('sl_phase_advance_at_br', 'f'),                        
            ('sl_cycle_int_rpm_br', 'f'),                                    
            ('sl_bemf_coupling_k', 'f'),                                    
            ('hall_table_0', 'b'),                                    
            ('hall_table_1', 'b'),                                    
            ('hall_table_2', 'b'),                                    
            ('hall_table_3', 'b'),                                    
            ('hall_table_4', 'b'),                                    
            ('hall_table_5', 'b'),                                    
            ('hall_table_6', 'b'),                                    
            ('hall_table_7', 'b'),                                    
            ('hall_sl_erpm', 'f'),                                    
            ('foc_current_kp', 'f'),                                    
            ('foc_current_ki', 'f'),                                                
            ('foc_f_sw', 'f'),                                                
            ('foc_dt_us', 'f'),                                                
            ('foc_encoder_inverted', 'B'),                                                
            ('foc_encoder_offset', 'f'),                                                
            ('foc_encoder_ratio', 'f'),                                                
            ('foc_sensor_mode', 'B'),                                                
            ('foc_pll_kp', 'f'),                                                
            ('foc_pll_ki', 'f'),                                                
            ('foc_motor_l', 'f'),                                                
            ('foc_motor_r', 'f'),                                                
            ('foc_motor_flux_linkage', 'f'),                                                
            ('foc_observer_gain', 'f'),                                                
            ('foc_observer_gain_slow', 'f'),                                                
            ('foc_duty_dowmramp_kp', 'f'),                                                
            ('foc_duty_dowmramp_ki', 'f'),                                                          
            ('foc_openloop_rpm', 'f'),                                                
            ('foc_sl_openloop_hyst', 'f'),                                                
            ('foc_sl_openloop_time', 'f'),                                                
            ('foc_sl_d_current_duty', 'f'),                                                
            ('foc_sl_d_current_factor', 'f'),                                                
            ('foc_hall_table_0', 'B'),                                                
            ('foc_hall_table_1', 'B'),                                                
            ('foc_hall_table_2', 'B'),                                                
            ('foc_hall_table_3', 'B'),                                                
            ('foc_hall_table_4', 'B'),                                                
            ('foc_hall_table_5', 'B'),                                                
            ('foc_hall_table_6', 'B'),                                                
            ('foc_hall_table_7', 'B'),                                                            
            ('foc_sl_erpm', 'f'),                                                
            ('foc_sample_v0_v7', 'B'),                                                
            ('foc_sample_high_current', 'B'),                                                
            ('foc_sat_comp', 'f'),                                                
            ('foc_temp_comp', 'B'),                                                
            ('foc_temp_comp_base_temp', 'f'),                                                
            ('s_pid_kp', 'f'),                                                
            ('s_pid_ki', 'f'),                                                
            ('s_pid_kd', 'f'),                                                
            ('s_pid_min_erpm', 'f'),                                                
            ('s_pid_allow_braking', 'B'),                                                
            ('p_pid_kp', 'f'),                                                
            ('p_pid_ki', 'f'),                                                
            ('p_pid_kd', 'f'),                                                
            ('p_pid_ang_div', 'f'),                                                
            ('cc_startup_boost_duty', 'f'),                                                
            ('cc_min_current', 'f'),                                                
            ('cc_gain', 'f'),                                                
            ('cc_ramp_step_max', 'f'),                                                
            ('m_fault_stop_time_ms', 'i'),                                                
            ('m_duty_ramp_step', 'f'),                                                
            ('m_current_backoff_gain', 'f'),                                                
            ('m_encoder_counts', 'I'),                                                
            ('m_sensor_port_mode', 'B'),                                                
            ('m_invert_direction', 'B'),                                                
            ('m_drv8301_oc_mode', 'B'),                                                
            ('m_drv8301_oc_adj', 'B'),                                                
            ('m_bldc_f_sw_min', 'f'),                                                
            ('m_bldc_f_sw_max', 'f'),                                                
            ('m_dc_f_sw', 'f'),                                                
            ('m_ntc_motor_beta', 'f')                                                            
    ]


class SetTerminalCommand(metaclass=VESCMessage):
    id = 20   # COMM_TERMINAL_CMD
    can_id = None
    fields = [
        ('msg', 's')
    ]

class SetDutyCycle(metaclass=VESCMessage):
    """ Set the duty cycle.

    :ivar duty_cycle: Value of duty cycle to be set (range [-1e5, 1e5]).
    """
    id = 5   # COMM_SET_DUTY
    can_id = None
    fields = [
        ('duty_cycle', 'i', 1)
    ]


class SetRPM(metaclass=VESCMessage):
    """ Set the RPM.

    :ivar rpm: Value to set the RPM to.
    """
    id = 8  # COMM_SET_RPM
    can_id = None
    fields = [
        ('rpm', 'i', 1)
    ]


class SetCurrent(metaclass=VESCMessage):
    """ Set the current (in milliamps) to the motor.

    :ivar current: Value to set the current to (in milliamps).
    """
    id = 6  # COMM_SET_CURRENT
    fields = [
        ('current', 'i', 1)
    ]


class SetCurrentBrake(metaclass=VESCMessage):
    """ Set the current brake (in milliamps).

    :ivar current_brake: Value to set the current brake to (in milliamps).
    """
    id = 7  # 	COMM_SET_CURRENT_BRAKE
    can_id = None
    fields = [
        ('current_brake', 'i', 1)
    ]

class SetPosition(metaclass=VESCMessage):
    """Set the rotor angle based off of an encoder or sensor
    
    :ivar pos: Value to set the current position or angle to.
    """
    id = 9  # COMM_SET_POS
    can_id = None
    fields = [
        ('pos', 'i', 1000000)
    ]
    
class SetPositionCumulative(metaclass=VESCMessage):
    """Set the rotor angle based off of an encoder or sensor
    
    :ivar pos: Value to set the current position or angle to.
    """
    id = 40  # COMM_SET_POS_CUMULATIVE
    can_id = None
    fields = [
        ('pos', 'i', 100000),
        ('erpm', 'i', 1)
    ]
    
class SetRotorPositionMode(metaclass=VESCMessage):
    """Sets the rotor position feedback mode.
        
    It is reccomended to use the defined modes as below:
        * DISP_POS_OFF
        * DISP_POS_MODE_ENCODER
        * DISP_POS_MODE_PID_POS
        * DISP_POS_MODE_PID_POS_ERROR
    
    :ivar pos_mode: Value of the mode
    """

    DISP_POS_OFF = 0
    DISP_POS_MODE_INDUCTANCE = 1
    DISP_POS_MODE_OBSERVER = 2
    DISP_POS_MODE_ENCODER = 3
    DISP_POS_MODE_PID_POS = 4
    DISP_POS_MODE_PID_POS_ERROR = 5
    DISP_POS_MODE_ENCODER_OBSERVER_ERROR = 6
    
    id = 11  #  COMM_SET_DETECT
    can_id = None
    fields = [
        ('pos_mode', 'b', 1)
    ]

    
class SetCurrentGetPosCumulative(metaclass=VESCMessage):    
    id = 39  # COMM_SET_CURRENT_GET_POSITION
    can_id = None
    fields = [
        ('current', 'i', 1)
    ]

class SetAlive(metaclass=VESCMessage):    
    id = 30  # COMM_ALIVE
    can_id = None
    fields = []


        