/*******************************************************************************
*
* Copyright 2007-2016 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale License
* distributed with this Material.
* See the LICENSE file distributed for more details.
* 
*
****************************************************************************//*!
*
* @brief Space vector modulation 
* 
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "GMCLIB_Svm_F16.h"

/***************************************************************************//*!
*
* @brief  Standard SVM function
*
* @param  ptr - GMCLIB_2COOR_ALBE_T_F16 *psIn - IN - Direct(alpha) and quadrature(beta)             
*               component of the stator voltage vector in the stationary reference 
*               in frac16_t <-1;1) format.                                                     
*             - GMCLIB_3COOR_T_F16 *psOut - OUT - Pointer to duty cycle of the A, B and C 
*               phases in frac16_t <-1;1) format.   
*
* @return This function returns - sector number of the stator voltage vector resides in.
*       
* @remarks  The function calculates the appropriate duty cycles needed to generate 
*           a given stator reference voltage using the Space Vector Modulation with 
*           a duty cycle of the null switching state from states O000 and O111 
*           in each sector of the hexagon.
*
*         The function fills variables a, b and c in the data structure psOut 
*         and returns the sector number of the stator voltage vector resides in.
*
*     X = ubeta
*     Y = 1/2(ubeta - sqrt(3)*ualfa)
*     Z = 1/2(ubeta + sqrt(3)*ualfa)
*
*     Sectors   U0,U60   U60,U120   U120,U180   U180,U240   U240,U300   U300,U0
*     t_1       X        Z          -X          Z           -Z          Y
*     t_2       -Z       Y          Z          -X           -Y          -X
*
*     t1 = (T - t_1 - t_2) / 2
*     t2 = t1 + t_1
*     t3 = t2 + t_2
*
*******************************************************************************/
uint16_t GMCLIB_SvmStd_F16_FCRam(const GMCLIB_2COOR_ALBE_T_F16 *psIn,
                              GMCLIB_3COOR_T_F16 *psOut)
{
    register frac32_t f32Sqrt3DivBy2;   
    register frac32_t f32A, f32B, f32C;
    register frac32_t f32temp;
    register frac32_t f32Alpha,f32Beta,f32BetaDiv2;
    
    f32Alpha = psIn->f16Alpha;
     f32Beta = psIn->f16Beta;
    f32BetaDiv2 = f32Beta >> 1;
    f32Sqrt3DivBy2 = GMCLIB_SQRT3_DIV_2_F16;
    
    /* 
    f32C =  f32Alpha * sqrt(3)/2
    f32A =  f32Alpha * sqrt(3)/2 - beta/2
    f32B = -f32Alpha * sqrt(3)/2 - beta/2
    */      
    f32C = (f32Alpha * f32Sqrt3DivBy2) >> 15 ; 
    f32A =  f32C - f32BetaDiv2 ; 
    f32B = -f32C - f32BetaDiv2 ;
    
    /* Sector determination: if B > 0 and A > 0, the sector 5 is calculated */
    if (f32B <= 0)
    {
        /*** Sector 2 calculation(s) ***/
        if (f32A <= 0 )            
        { 
            /* psOut -> f16C = (1 - beta) / 2
               psOut -> f16A = PhaseC + alpha * sqrt(3)/2 + beta/2 = PhaseC - f32B;
               psOut -> f16B = PhaseA - alpha * sqrt(3)/2 + beta/2 */
            
            f32temp = (INT16_MAX - f32Beta) >> 1;                   /* temp = (1 - beta) / 2 */ 
            f32temp = (f32temp > INT16_MAX)  ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16C = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase A  */
            f32temp -= f32B;                                        /* temp = temp + alpha * sqrt(3)/2 + beta/2 = temp + f32C  */
            psOut -> f16A = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase B */
            f32temp = f32temp - f32C + f32BetaDiv2;                 /* temp = temp - alpha * sqrt(3)/2 + beta/2 = temp - f32C + f32BetaDiv2 */
            f32temp = (f32temp > INT16_MAX)  ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16B = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase C */
            return((uint16_t)0x2);                                  /* return sector 2 */
        }  
        
        /*** Sector 6 calculation(s) ***/
        if (f32Beta <= 0)        
        {
            /* psOut -> f16B = (1 - alpha * sqrt(3)/2 + beta/2) / 2 = (1 - f32C + beta/2) / 2 
            psOut -> f16C = PhaseB - beta
            psOut -> f16A = PhaseC + alpha * sqrt(3)/2 + beta/2 = PhaseC - f32B  */
            
            f32temp = (INT16_MAX - f32C + f32BetaDiv2) >> 1;        /* temp =(1 - alpha * sqrt(3)/2 + beta/2) / 2 = (1 - f32C + beta/2) / 2 */ 
            psOut -> f16B = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase A */
            f32temp -= f32Beta;                                     /* temp = temp - beta  */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16C = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase C */
            f32temp -= f32B ;                                       /* temp = temp + alpha * sqrt(3)/2 + beta/2 = temp - f32B */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16A = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase C */
            return((uint16_t)0x6);                                  /* return sector 6 */
        }  
      
    /* Sector 1 calculation(s) */  
    /* psOut -> f16C = (1 - alpha * sqrt(3)/2 - beta/2 ) / 2 = (1 + f32B ) / 2 
    psOut -> f16B = PhaseC + beta
    psOut -> f16A = PhaseB + alpha * sqrt(3)/2 - beta/2 =PhaseB + f32A */
    
    f32temp = (INT16_MAX + f32B ) >> 1;                       /* temp =(1 + f32B ) / 2 */ 
    psOut -> f16C = (f32temp < 0) ? 0 : f32temp;              /* limits output and store phase A */
    f32temp += f32Beta;                                       /* temp = temp + beta  */
    f32temp = (f32temp > INT16_MAX)  ? INT16_MAX : f32temp;   /* high saturation */
    psOut -> f16B = (f32temp < 0) ? 0 : f32temp;              /* limits output and store phase C */
    f32temp += f32A;                                          /* temp = temp + alpha * sqrt(3)/2 - beta/2 = temp + f32A */
    f32temp = (f32temp > INT16_MAX)  ? INT16_MAX : f32temp;   /* high saturation */
    psOut -> f16A = (f32temp < 0) ? 0 : f32temp;              /* limits output and store phase C */
    return((uint16_t)0x1);                                    /* return sector 1 */
    }  
  
    if (f32A <= 0) 
    {
        /* Sector 4 calculation(s) */
        if (f32Beta <= 0) 
        /* psOut -> f16A = (1 + alpha * sqrt(3)/2 + beta/2) / 2 = (1 - f32B ) / 2
        psOut -> f16B = PhaseA - alpha * sqrt(3)/2 + beta / 2 = PhaseA - f32A 
        psOut -> f16C = PhaseB - beta */
        {
            f32temp = (INT16_MAX - f32B ) >> 1;                     /* temp =(1 + alpha * sqrt(3)/2 + beta/2) / 2  */ 
            psOut -> f16A = (f32temp < 0) ?         0 : f32temp;    /* limits output and store phase A */
            f32temp -= f32A ;                                       /* temp = temp - alpha * sqrt(3)/2 + beta/2 = temp - f32A */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16B = (f32temp < 0) ?         0 : f32temp;    /* limits output and store phase B */
            f32temp -= f32Beta;                                     /* temp = temp - beta  */
            f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
            psOut -> f16C = (f32temp < 0) ?         0 : f32temp;    /* limits output and store phase C */
            return((uint16_t)0x4);                                  /* return sector 4 */
        }  
      
        /* Sector 3 calculation(s) */
        /* psOut -> f16A = (1 + alpha * sqrt(3)/2 - beta/2 ) / 2 = ( 1 + f32A ) / 2
        psOut -> f16C = PhaseA - alpha * sqrt(3)/2 - beta/2 = PhaseA + f32B 
        psOut -> f16B = PhaseC + beta  */
        
        f32temp = (INT16_MAX + f32A) >> 1;                      /* temp = 1 + alpha * sqrt(3)/2 - beta/2 = 1 + f32A */
        psOut -> f16A = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase A */
        f32temp += f32B;                                        /* temp = temp - alpha * sqrt(3)/2 + beta/2 = temp + f32B */
        f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
        psOut -> f16C = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase B */
        f32temp += f32Beta;                                     /* temp = temp + beta  */
        f32temp = (f32temp > INT16_MAX ) ? INT16_MAX : f32temp; /* high saturation */
        psOut -> f16B = (f32temp < 0) ? 0 : f32temp;            /* limits output and store phase C */
        return((uint16_t)0x3);                                  /* return sector 3 */
    }

    /*** Sector 5 calculation(s) ***/
    
    /* B > 0 and A > 0, the sector 5 is calculated */ 
    /* pOut -> f32B = (1 + f32Beta) / 2
       pOut -> f32A = PhaseB + f32Alpha * sqrt(3)/2 - beta/2 = PhaseB + f32A
       pOut -> f32C = PhaseA - f32Alpha * sqrt(3)/2 - beta/2 = PhaseA + f32B */  
    
    f32temp = (INT16_MAX + f32Beta) >> 1;                         /* Phase B calculation*/
    psOut -> f16B = (f32temp < 0) ? 0 : f32temp;                  /* limits output and store phase B */
    f32temp += f32A;                                              /* Phase A calculation*/           
    f32temp = (f32temp > INT16_MAX)   ? INT16_MAX : f32temp;      /* high saturation */ 
    psOut -> f16A  = (f32temp < 0) ? 0 : f32temp;                 /* limits output and store phase A */
    f32temp += f32B;                                              /* Phase C calculation*/
    f32temp = (f32temp > INT16_MAX)   ? INT16_MAX : f32temp;      /* high saturation */        
    psOut -> f16C = (f32temp < 0)  ? 0 : f32temp;                 /* limits output and store phase C */   
    return((uint16_t)0x5);                                        /* return sector 2 */    
}  

