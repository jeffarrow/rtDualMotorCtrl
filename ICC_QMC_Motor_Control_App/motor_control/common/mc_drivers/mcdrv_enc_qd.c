/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mcdrv_enc_qd.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
static bool_t s_statusPass;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Function returns actual position and speed
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdEncGet(mcdrv_qd_enc_t *this)
{
    s_statusPass = TRUE;
    
    /* read number of pulses and get mechanical position */
    
    this ->ui16LPOSRegister = (this->pui32QdBase->LPOS);
    
    this->f16PosMe = (frac16_t)(MLIB_Mul_F16as(this->a32PosMeGain, 
        (frac16_t)(this ->ui16LPOSRegister)));  
       
    /* tracking observer calculation */			
    this->f16PosMeEst = (frac16_t)AMCLIB_TrackObsrv_A32af(this->a32PosErr, &this->sTo);

    /* calculation of error function for tracking observer */  
    this->a32PosErr  = (acc32_t)MLIB_Sub_F16(this->f16PosMe, this->f16PosMeEst);
    
    if(this->a32PosErr > ACC32(0.5))  this->a32PosErr -= ACC32(1.0);
    else if(this->a32PosErr < ACC32(-0.5))  this->a32PosErr += ACC32(1.0);

    /* speed estimation by the tracking observer */
    this->fltSpdMeEst = this->sTo.fltSpeed;
    /* read revolution counter */
    this->ui32RevCounter = (this->pui32QdBase->REV);
    
    /* calculating position for position control */   
    if(((this->ui32RevCounter) > 32767) & ((this->ui32RevCounter) <= 65535))
        this->a32PosMeReal = (acc32_t)((uint16_t)(this->f16PosMe) + (65535U * this->ui32RevCounter)+65535U);
    else
        this->a32PosMeReal = (acc32_t)((uint16_t)(this->f16PosMe) + (65535U * this->ui32RevCounter));
    
    /* pass estimator speed values lower than minimal encoder speed */
    if ((MLIB_Abs_FLT(this->fltSpdMeEst ) < (this->fltSpdEncMin))) 
    {
        this->fltSpdMeEst = 0U;
    }
        
    /* store results to user-defined variables */
    *this->pf16PosElEst = (frac16_t)(this->f16PosMeEst * this->ui16Pp );
    *this->pfltSpdMeEst = (this->fltSpdMeEst);

    return (s_statusPass);
}


/*!
 * @brief Function returns index offset between index and align position of rotor
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdEncGetIndexOffset(mcdrv_qd_enc_t *this)
{
     
    s_statusPass = TRUE;
    
    this ->ui16LPOSRegister = (this->pui32QdBase->LPOS);
    this->ui16LedIndexOffset = this->ui16LPOSRegister + this->ui16LedAlignOffset;
    this->a32PosIndexOffset = (acc32_t)((uint16_t)((frac16_t)(MLIB_Mul_F16as(this->a32PosMeGain, (frac16_t)(this ->ui16LPOSRegister)))));
  
    if(this->ui16LedIndexOffset > 4095U)
    this->ui16LedIndexOffset = this->ui16LedIndexOffset - 4095U;      
    
    return (s_statusPass);
      
}


/*!
 * @brief Led flashing function
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdEncLedPosition(mcdrv_qd_enc_t *this)
{
    s_statusPass = TRUE;
    
    if(this->bDemoOnOff)
    {
      /* If speed is smaller as value leds are off */
      if((MLIB_Abs_FLT((this->fltSpdMeEst))*(this->ui16Pp)) < 500) //MINIMAL FLASHING SPEED 500 = 1600rpm
      {
          GPIO_PortSet(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
      }
      else
      {
          this ->ui16EncCtrl2DirRegister = (((this->pui32QdBase->CTRL2) & ENC_CTRL2_DIR_MASK) >> ENC_CTRL2_DIR_SHIFT);
             
          /* positive speed direction */
          if((this ->ui16EncCtrl2DirRegister) > 0)
          {
              if((this->ui16OnLedPosition) > (this->ui16OffLedPosition))
              {
                  if((this->ui16LPOSRegister) >= (this->ui16OnLedPosition))
                  {
                      /* LED ON */
                      GPIO_PortSet(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
                  }
                  if(((this->ui16LPOSRegister) > (this->ui16OffLedPosition)) & ((this->ui16LPOSRegister) < (this->ui16OnLedPosition)))
                  {
                      /* LED OFF */  
                      GPIO_PortClear(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
                  }  
              }
              else
              {
                  if(((this->ui16LPOSRegister) >= (this->ui16OnLedPosition)) & (((this->ui16LPOSRegister) < (this->ui16OffLedPosition))))
                  {
                      /* LED ON */
                      GPIO_PortSet(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
                  }
                  else 
                  {
                      /* LED OFF */  
                      GPIO_PortClear(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
                  }       
              }               
              
          }
          /* negative speed direction */
          else
          {
              if((this->ui16OffLedPosition) > (this->ui16OnLedPosition))
              {
                  if((this->ui16LPOSRegister) <= (this->ui16OffLedPosition))
                  {
                      /* LED ON */
                      GPIO_PortSet(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
                  }
                  if((this->ui16LPOSRegister) < (this->ui16OnLedPosition) )
                  {
                      /* LED OFF */  
                      GPIO_PortClear(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
                  } 
              }
              else
              {
                  if((this->ui16LPOSRegister) <= (this->ui16OffLedPosition))
                  {
                      /* LED ON */
                      GPIO_PortSet(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
                  }
                  if(((this->ui16LPOSRegister) <= (this->ui16OnLedPosition)) & ((this->ui16LPOSRegister) > (this->ui16OffLedPosition)))
                  {
                      /* LED OFF */  
                      GPIO_PortClear(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
                  } 
              } 
              
          } 
      }
    }
    else
    {
        GPIO_PortClear(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
    }

    return (s_statusPass);  
     
}


/*!
 * @brief Function clears internal variables and decoder counter
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdEncClear(mcdrv_qd_enc_t *this)
{
    s_statusPass = TRUE;

    this->f16PosMe              = 0;
    this->f16PosMeEst           = 0;
    this->fltSpdMeEst           = 0;
    
    /* initilize tracking observer */
    this->sTo.f32Theta  = 0;
    this->sTo.fltSpeed  = 0;
    this->sTo.fltI_1    = 0;
    
    /* clear decoder counters */
    this->pui32QdBase->POSD     = 0;
    this->pui32QdBase->REV      = 0;
    this->pui32QdBase->LPOS     = 0;
    this->pui32QdBase->UPOS     = 0;

    return (s_statusPass);
}

/*!
 * @brief Function enable ENC index interrupt
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdEncIndexIRQOn(mcdrv_qd_enc_t *this)
{
    s_statusPass = TRUE;
    
    /* Do not execuite next ENC interrupt */ 
    this->pui32QdBase->CTRL = 0x0;
    
    /* Clear interrupt flag */
    this->pui32QdBase->CTRL = (this->pui32QdBase->CTRL & (uint16_t)(~((ENC_CTRL_HIRQ_MASK | ENC_CTRL_XIRQ_MASK | ENC_CTRL_DIRQ_MASK | ENC_CTRL_CMPIRQ_MASK)))) | ENC_CTRL_XIRQ_MASK;
    
    /* Enable ENC index interrupt */    
    this->pui32QdBase->CTRL = (ENC_CTRL_XIE_MASK);
    
    return (s_statusPass);
}

/*!
 * @brief Function set mechanical position of quadrature encoder
 *
 * @param this     Pointer to the current object
 *        f16PosMe Mechanical position
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdEncSetPosMe(mcdrv_qd_enc_t *this, frac16_t f16PosMe)
{
    frac16_t f16CntMod;
  
    s_statusPass = TRUE;
    f16CntMod = (frac16_t)(this->pui32QdBase->LMOD>>1);
    
    /* set mechnical position */
    this->f16PosMe = f16PosMe;
    this->sTo.f32Theta = MLIB_Conv_F32s(f16PosMe);
    this->pui32QdBase->LPOS =  (uint16_t)(MLIB_Mul_F16(f16PosMe, f16CntMod) + 
                               (uint16_t)f16CntMod);

    return(s_statusPass);
}


/*!
 * @brief Function set direction of quadrature encoder
 *
 * @param this       Pointer to the current object
 *        bDirection Encoder direction
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdEncSetDirection(mcdrv_qd_enc_t *this)
{
    s_statusPass = TRUE;
    
    /* forward/reverse */
    if(this->bDirection)
        this->pui32QdBase->CTRL |= ENC_CTRL_REV_MASK;
    else
        this->pui32QdBase->CTRL &= ~ENC_CTRL_REV_MASK;

    return(s_statusPass);
}


/*!
 * @brief 
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdEncLedDemoType(mcdrv_qd_enc_t *this)
{
    s_statusPass = TRUE;
    
    /* Demo type 1 */
    if(this->ui16EncLedDemoType ==  1U)
    {
        this->bDemoOnOff = 1U;
        
        if(this->fltSpdMeEst > 0.0)
        {
          this->ui16OnLedPosition = ((uint16_t)MLIB_Abs_FLT((this->fltSpdMeEst)*(this->ui16Pp) * 5) - 2000)+ this->ui16LedIndexOffset;
        }
        else
        {
          this->ui16OnLedPosition = (4000U - ((uint16_t)MLIB_Abs_FLT((this->fltSpdMeEst)*(this->ui16Pp) * 5) - 2000) + this->ui16LedIndexOffset );
        }
        
        if(this->ui16OnLedPosition > 4095U)
        this->ui16OnLedPosition = this->ui16OnLedPosition - 4095U;
     
        this->ui16OffLedPosition = this->ui16OnLedPosition + this->ui16OffLedDelta;

        if(this->ui16OffLedPosition > 4095U)
        this->ui16OffLedPosition = this->ui16OffLedPosition - 4095U; 
        
        if(this->ui16OnLedPosition < 25U)       
        this->ui16OnLedPosition = 25U;          
               
        if(this->ui16OffLedPosition < 25)   
        this->ui16OffLedPosition = 25U;     
          
    }
    /* Demo type 2 */
    else if(this->ui16EncLedDemoType ==  2U)
    {
        this->bDemoOnOff = 1U;
        
        if(this->fltSpdMeEst > 0.0)
        {
            this->ui16OnLedPosition = this->ui16LedIndexOffset;
            this->ui16OffLedPosition = (uint16_t)MLIB_Abs_FLT((this->fltSpdMeEst)*(this->ui16Pp) * 4) - 1500 + this->ui16OffLedDelta + this->ui16LedIndexOffset;
        
        }
        else
        {
            this->ui16OnLedPosition = (4095U - (uint16_t)MLIB_Abs_FLT((this->fltSpdMeEst)*(this->ui16Pp) * 4) + 1500 +this->ui16LedIndexOffset);
            this->ui16OffLedPosition = this->ui16LedIndexOffset;
                          
        }

        if(this->ui16OnLedPosition > 4095U)
        this->ui16OnLedPosition = this->ui16OnLedPosition - 4095U;
     
        if(this->ui16OffLedPosition > 4095U)
        this->ui16OffLedPosition = this->ui16OffLedPosition - 4095U; 
        
        if(this->ui16OnLedPosition < 25U)       
        this->ui16OnLedPosition = 25U;          

        if(this->ui16OffLedPosition < 25)   
        this->ui16OffLedPosition = 25U;     
        
    }
    /* Demo type 3 */
    else if(this->ui16EncLedDemoType ==  3U)
    {
        this->bDemoOnOff = 1U;
        
        if(this->fltSpdMeEst > 0.0)
        {
            this->ui16OnLedPosition = this->ui16OnLedPosition - ((uint16_t)MLIB_Abs_FLT((this->fltSpdMeEst)*(this->ui16Pp) / 100U));
            if(this->ui16OnLedPosition < 40U)
            this->ui16OnLedPosition = 4094U;   
        }
        else
        {
            this->ui16OnLedPosition = this->ui16OnLedPosition + ((uint16_t)MLIB_Abs_FLT((this->fltSpdMeEst)*(this->ui16Pp) / 100U));
            if(this->ui16OnLedPosition > 4094U)
            this->ui16OnLedPosition = 40U;  
        }
        
        if(this->ui16OnLedPosition > 4095U)
        this->ui16OnLedPosition = this->ui16OnLedPosition - 4095U;
     
        this->ui16OffLedPosition = this->ui16OnLedPosition + this->ui16OffLedDelta;

        if(this->ui16OffLedPosition > 4095U)
        this->ui16OffLedPosition = this->ui16OffLedPosition - 4095U;
        
        if(this->ui16OnLedPosition < 20U)       
        this->ui16OnLedPosition = 20U;          

        if(this->ui16OffLedPosition < 20)   
        this->ui16OffLedPosition = 20U;     
                
    }
    else
    {
        this->bDemoOnOff = 0U;
        
        /* clear led */
        GPIO_PortClear(this->pui32GpioLedBase, 1U << (this->ui16GpioLedPin));
        
    }
    
    
    return(s_statusPass);
}
