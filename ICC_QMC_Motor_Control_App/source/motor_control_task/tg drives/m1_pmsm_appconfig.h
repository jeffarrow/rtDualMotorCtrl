/**********************************************************************/
// File Name: {FM_project_loc}/../src/projects/twrkv58f/pmsm_appconfig.h 
//
// Date:  November 15, 2017, 14:7:31
//
// Automatically generated file for static configuration of the PMSM FOC application
/**********************************************************************/

#ifndef __M1_PMSM_APPCONFIG_H
#define __M1_PMSM_APPCONFIG_H


// Motor Parameters 
//-----------------------------------------------------------------------------------------------------------  
// Stator resistance = 0.58 [Ohms] 
// Pole-pair numbers = 3 [-] 
// Direct axis inductance = 0.000227 [H] 
// Quadrature axis inductance = 0.000249 [H] 
// Back-EMF constant = 0.0564 [V.sec/rad] 
// Drive inertia = 0.000092 [kg.m2] 
// Nominal current = 2 [A] 
  
#define M1_MOTOR_PP (3)  
  
// Application Scales 
//-----------------------------------------------------------------------------------------------------------  
#define M1_I_MAX (8.25F)  
#define M1_U_DCB_MAX (43.5)//(60.8F)  
#define M1_U_MAX (35.1F)  
#define M1_N_MAX (1382.0F)  
#define M1_FREQ_MAX (220.0F)  
  
#define M1_U_DCB_TRIP (28.0F)  
#define M1_U_DCB_UNDERVOLTAGE (16.0F)  
#define M1_U_DCB_OVERVOLTAGE (30.0F)  
#define M1_N_OVERSPEED (1350.9F)  
#define M1_N_MIN (94.2F)  
  
#define M1_N_NOM (1256.6F)  
#define M1_I_PH_NOM (2.0F)  
  
#define M1_UDCB_IIR_B0 (0.030459027951F)  
#define M1_UDCB_IIR_B1 (0.030459027951F)  
#define M1_UDCB_IIR_A1 (0.939081944097F)  
// Mechanical alignment 
//-----------------------------------------------------------------------------------------------------------  
#define M1_ALIGN_DURATION (4000)  
#define M1_ALIGN_VOLTAGE (1.2F)  
  
// Application counters 
//-----------------------------------------------------------------------------------------------------------  
#define M1_CALIB_DURATION (200)  
#define M1_FAULT_DURATION (6000)  
#define M1_FREEWHEEL_DURATION (1500)  
  
// Miscellaneous 
//-----------------------------------------------------------------------------------------------------------  
#define M1_E_BLOCK_TRH (1.4F)  
#define M1_E_BLOCK_PER (2000)  
  
// Current Loop Control 
//-----------------------------------------------------------------------------------------------------------  
// Loop bandwidth = 280 [Hz] 
// Loop attenuation = 1 [-] 
// Loop sample time = 0.0001 [sec] 
  
#define M1_CLOOP_LIMIT (0.519615484541F)  
  
// D - axis parameters 
#define M1_D_KP_GAIN (0.218718516249F)  
#define M1_D_KI_GAIN (0.070258950242F)  
  
// Q - axis parameters 
#define M1_Q_KP_GAIN (0.296127359233F)  
#define M1_Q_KI_GAIN (0.077068187711F)  
  
// Speed Loop Control 
//-----------------------------------------------------------------------------------------------------------  
// Loop bandwidth = 28 [Hz] 
// Loop attenuation = 1 [-] 
// Loop sample time = 0.001 [sec] 
#define M1_SPEED_PI_PROP_GAIN (0.034701209268F)  
#define M1_SPEED_PI_INTEG_GAIN (0.00030524778F)  
#define M1_SPEED_LOOP_HIGH_LIMIT (2.0F)  
#define M1_SPEED_LOOP_LOW_LIMIT (-2.0F)  
  
#define M1_SPEED_RAMP_UP (3.14159265359F)  
#define M1_SPEED_RAMP_DOWN (3.14159265359F)  
  
#define M1_SPEED_IIR_B0 (0.030459027951F)  
#define M1_SPEED_IIR_B1 (0.030459027951F)  
#define M1_SPEED_IIR_A1 (0.939081944097F)  
  
#define M1_POS_P_PROP_GAIN FRAC16(0.12)  
  
// Position & Speed Sensors Module 
//-----------------------------------------------------------------------------------------------------------  
// Loop bandwidth = 100 [Hz] 
// Loop attenuation = 1 [-] 
// Loop sample time = 0.0001 [sec] 
  
// ATO input parameters 
#define M1_POSPE_KP_GAIN (1256.637061435917F)  
#define M1_POSPE_KI_GAIN (39.47841760435744F)  
#define M1_POSPE_INTEG_GAIN (0.00003183098861837907F)  
#define M1_POSPE_ENC_PULSES (1024)  
#define M1_POSPE_ENC_DIRECTION             (0)
#define M1_POSPE_ENC_N_MIN (0.0F)  
#define M1_POSPE_MECH_POS_GAIN ACC32(16.0)  
  
// Sensorless BEMF DQ Observer 
//-----------------------------------------------------------------------------------------------------------  
// Loop bandwidth = 300 [Hz] 
// Loop attenuation = 1 [-] 
// Loop sample time = 0.0001 [sec] 
#define M1_I_SCALE (0.796491228071F)  
#define M1_U_SCALE (0.350877192983F)  
#define M1_E_SCALE (0.350877192983F)  
#define M1_WI_SCALE (0.000087368422F)  
#define M1_BEMF_DQ_KP_GAIN (0.275769838838F)  
#define M1_BEMF_DQ_KI_GAIN (0.080654407166F)  
#define M1_TO_KP_GAIN (376.991118430776F)  
#define M1_TO_KI_GAIN (3.553057584393F)  
#define M1_TO_THETA_GAIN (0.000031830989F)  
#define M1_TO_SPEED_IIR_B0 (0.111635211705F)  
#define M1_TO_SPEED_IIR_B1 (0.111635211705F)  
#define M1_TO_SPEED_IIR_A1 (0.776729576591F)  
#define M1_OL_START_RAMP_INC (0.094247779608F)  
#define M1_OL_START_I (0.65F)  
#define M1_MERG_SPEED_TRH (157.07963267949F)  
#define M1_MERG_COEFF FRAC16(0.006103515625)  
  
// Control Structure Module - Scalar Control 
//-----------------------------------------------------------------------------------------------------------  
#define M1_SCALAR_VHZ_FACTOR_GAIN (0.075F)  
#define M1_SCALAR_INTEG_GAIN ACC32(0.044)  
#define M1_SCALAR_RAMP_UP (3.14159265359F)  
#define M1_SCALAR_RAMP_DOWN (3.14159265359F) 


#endif

//End of generated file                 
/**********************************************************************/
