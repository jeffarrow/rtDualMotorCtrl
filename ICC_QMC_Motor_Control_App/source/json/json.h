/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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
#ifndef __json_h_
#define __json_h_

#include <stdio.h>
#include <stdlib.h>

#include "main.h"

//-----------------------------------------------------------------------
// KSDK Includes
//-----------------------------------------------------------------------

//------------------------------------------------------------------------------
// Enums
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Typedef
//------------------------------------------------------------------------------                                

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------

#define JSON_MSG_MC_STATUS        "{\r\n\"board_id\": \"imxrt1050_icc_daughter_card\","\
                                   "\r\n\t\"motor status\": ["\
                                   "\r\n\t\t{"\
                                   "\r\n\t\t\"motor_id\": \"%d\","\
                                   "\r\n\t\t\"app_status\": \"%d\","\
                                   "\r\n\t\t\"motor_state\": \"%d\","\
                                   "\r\n\t\t\"fault\": \"%d\","\
                                   "\r\n\t\t\"speed\": \"%d\","\
                                   "\r\n\t\t\"position\": \"%d\","\
                                   "\r\n\t\t\"current_iq [mA]\": \"%d\","\
                                   "\r\n\t\t\"voltage_vdc [mV]\": \"%d\""\
                                   "\r\n\t\t},"\
                                   "\r\n\t]"\
                                   "\r\n}" 
                                     
#define JSON_MSG_MC_COMMAND       "{\r\n\"board_id\": \"imxrt1050_icc_daughter_card\","\
                                   "\r\n\t\"motor command\": ["\
                                   "\r\n\t\t{"\
                                   "\r\n\t\t\"motor_id\": \"%d\","\
                                   "\r\n\t\t\"app_status_command\": \"%d\","\
                                   "\r\n\t\t\"control_method\": \"%d\","\
                                   "\r\n\t\t\"speed_mode_selection\": \"%d\","\
                                   "\r\n\t\t\"speed\": \"%d\","\
                                   "\r\n\t\t\"position_mode_selection\": \"%d\","\
                                   "\r\n\t\t\"position\": \"%d\","\
                                   "\r\n\t\t},"\
                                   "\r\n\t]"\
                                   "\r\n}"
 
                                     
/*                                     
{
"board_id": "imxrt1050_icc_daughter_card",
	"motor command": [
		{
		"motor_id": "1",
		"app_status_command": "1",
		"control_method": "2",
		"speed_mode_selection": "1",
		"speed": "11",
		"position_mode_selection": "0",
		"position": "3",
		},
	]
}
*/

//------------------------------------------------------------------------------     
// Function Prototypes                                                          
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
#endif /* __json_h_ */

