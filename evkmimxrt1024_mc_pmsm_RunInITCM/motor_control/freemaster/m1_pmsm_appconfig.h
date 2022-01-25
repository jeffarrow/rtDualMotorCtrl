/**********************************************************************/
// File Name: {FM_project_loc}/m1_pmsm_appconfig.h 
//
// Date:  November 3, 2021, 17:34:16
//
// Automatically generated file for static configuration of the PMSM FOC application
/**********************************************************************/

#ifndef __M1_PMSM_APPCONFIG_H
#define __M1_PMSM_APPCONFIG_H


//Motor Parameters                      
//----------------------------------------------------------------------
//Pole-pair number                      = 4 [-]
//Stator resistance                     = 1.35 [Ohms]
//Direct axis inductance                = 0.006835 [H]
//Quadrature axis inductance            = 0.006792 [H]
//Back-EMF constant                     = 0.1167 [V.sec/rad]
//Drive inertia                         = 0.0000016 [kg.m2]
//Nominal current                       = 2 [A]

#define M1_MOTOR_PP                        (4)
//----------------------------------------------------------------------

//Application scales                    
//----------------------------------------------------------------------
#define M1_I_MAX                           (8.0F)
#define M1_U_DCB_MAX                       (433.0F)
#define M1_U_MAX                           (250.0F)
#define M1_N_MAX                           (1843.0F)
#define M1_FREQ_MAX                        (293.0F)
#define M1_U_DCB_TRIP                      (346.4F)
#define M1_U_DCB_UNDERVOLTAGE              (45.0F)
#define M1_U_DCB_OVERVOLTAGE               (346.4F)
#define M1_N_OVERSPEED                     (1658.8F)
#define M1_N_MIN                           (125.7F)
#define M1_N_NOM                           (1675.5F)
#define M1_I_PH_NOM                        (2.0F)
#define M1_SCALAR_UQ_MIN                   (1.0F)
//DCB Voltage Filter                    
#define M1_UDCB_IIR_B0                     (0.030459027951F)
#define M1_UDCB_IIR_B1                     (0.030459027951F)
#define M1_UDCB_IIR_A1                     (0.939081944097F)
//Mechanical Alignment                  
#define M1_ALIGN_VOLTAGE                   (1.5F)
#define M1_ALIGN_DURATION                  (8000)
//Application counters                  
#define M1_CALIB_DURATION                  (200)
#define M1_FAULT_DURATION                  (3000)
#define M1_FREEWHEEL_DURATION              (1000)
//Miscellaneous                         
#define M1_E_BLOCK_TRH                     (0.8F)
#define M1_E_BLOCK_PER                     (2000)

//Current Loop Control                  
//----------------------------------------------------------------------
//Loop bandwidth                        = 280 [Hz]
//Loop attenuation                      = 1 [-]
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
//Current Controller Output Limit       
#define M1_CLOOP_LIMIT                     (0.519615484541F)
//D-axis Controller - Parallel type     
#define M1_D_KP_GAIN                       (22.699520081761F)
#define M1_D_KI_GAIN                       (2.115506277114F)
//Q-axis Controller - Parallel type     
#define M1_Q_KP_GAIN                       (22.548220979564F)
#define M1_Q_KI_GAIN                       (2.102197312971F)

//Speed Loop Control                    
//----------------------------------------------------------------------
//Loop bandwidth                        = 10 [Hz]
//Loop attenuation                      = 1 [-]
//Loop sample time                      = 0.001 [sec]
//----------------------------------------------------------------------
//Speed Controller - Parallel type      
#define M1_SPEED_PI_PROP_GAIN              (0.0003F)
#define M1_SPEED_PI_INTEG_GAIN             (9e-7F)
#define M1_SPEED_LOOP_HIGH_LIMIT           (1.0F)
#define M1_SPEED_LOOP_LOW_LIMIT            (-1.0F)

#define M1_SPEED_RAMP_UP                   (0.418879020479F)
#define M1_SPEED_RAMP_DOWN                 (0.418879020479F)

#define M1_SPEED_IIR_B0                    (0.030459027951F)
#define M1_SPEED_IIR_B1                    (0.030459027951F)
#define M1_SPEED_IIR_A1                    (0.939081944097F)

#define M1_POS_P_PROP_GAIN                 FRAC16(0.071)

//Position & Speed Sensors Module       
//----------------------------------------------------------------------
//Loop Bandwidth                        = 100 [Hz]
//Loop Attenuation                      = 1 [-]
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
#define M1_POSPE_KP_GAIN                   (1256.637061435917F)
#define M1_POSPE_KI_GAIN                   (32.88552186442975F)
#define M1_POSPE_INTEG_GAIN                (0.000026515213519109765F)
#define M1_POSPE_ENC_PULSES                (1024)
#define M1_POSPE_ENC_DIRECTION             (1)
#define M1_POSPE_ENC_N_MIN                 (0.0F)
#define M1_POSPE_MECH_POS_GAIN             ACC32(16.0)

//Sensorless BEMF DQ and Tracking Observer
//----------------------------------------------------------------------
//Loop bandwidth                        = 280 [Hz]
//Loop attenuation                      = 1 [-]
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
//Bemf DQ Observer                      
#define M1_I_SCALE                         (0.980631276902F)
#define M1_U_SCALE                         (0.014347202296F)
#define M1_E_SCALE                         (0.014347202296F)
#define M1_WI_SCALE                        (0.000097446198F)
#define M1_BEMF_DQ_KP_GAIN                 (22.699520081761F)
#define M1_BEMF_DQ_KI_GAIN                 (2.115506277115F)

//Bemf DQ Observer                      
#define M1_TO_KP_GAIN                      (226.194671058466F)
#define M1_TO_KI_GAIN                      (1.279100730382F)
#define M1_TO_THETA_GAIN                   (0.000031830989F)
//Observer speed output filter          
#define M1_TO_SPEED_IIR_B0                 (0.111635211705F)
#define M1_TO_SPEED_IIR_B1                 (0.111635211705F)
#define M1_TO_SPEED_IIR_A1                 (0.776729576591F)
//Open loop start-up                    
#define M1_OL_START_RAMP_INC               (0.041887902048F)
#define M1_OL_START_I                      (0.4F)
#define M1_MERG_SPEED_TRH                  (209.43951023932F)
#define M1_MERG_COEFF                      FRAC16(0.004577636719)

//Control Structure Module - Scalar Control
//----------------------------------------------------------------------
#define M1_SCALAR_VHZ_FACTOR_GAIN          (0.66675F)
#define M1_SCALAR_INTEG_GAIN               ACC32(0.058666666667)
#define M1_SCALAR_RAMP_UP                  (0.006666666667F)
#define M1_SCALAR_RAMP_DOWN                (0.006666666667F)

#endif

//End of generated file                 
/**********************************************************************/
