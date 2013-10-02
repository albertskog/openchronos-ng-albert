// *************************************************************************************************
//
//      Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
//
//
//        Redistribution and use in source and binary forms, with or without
//        modification, are permitted provided that the following conditions
//        are met:
//
//          Redistributions of source code must retain the above copyright
//          notice, this list of conditions and the following disclaimer.
//
//          Redistributions in binary form must reproduce the above copyright
//          notice, this list of conditions and the following disclaimer in the
//          documentation and/or other materials provided with the
//          distribution.
//
//          Neither the name of Texas Instruments Incorporated nor the names of
//          its contributors may be used to endorse or promote products derived
//          from this software without specific prior written permission.
//
//        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//        A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//        OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//        SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//        LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//        DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//        THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//        (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//        OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************

#ifndef ALTITUDE_H_
#define ALTITUDE_H_

// *************************************************************************************************
// Include section

// *************************************************************************************************
// Prototypes section



// menu functions
extern void edit_mode_callback(void);
extern void submenu_callback(void);
extern void calib_callback(void);
extern void display_altitude(int16_t alt, uint8_t scr);
extern void update(enum sys_message);
// extern void clock_altitude_timeout(void);
extern void read_altitude(void);


// edit mode functions
extern void edit_base1_sel(uint8_t pos);
extern void edit_base1_dsel(uint8_t pos);
extern void edit_base1_set(uint8_t pos, int8_t step);
extern void edit_base2_sel(uint8_t pos);
extern void edit_base2_dsel(uint8_t pos);
extern void edit_base2_set(uint8_t pos, int8_t step);
extern void edit_base3_sel(uint8_t pos);
extern void edit_base3_dsel(uint8_t pos);
extern void edit_base3_set(uint8_t pos, int8_t step);
extern void edit_base4_sel(uint8_t pos);
extern void edit_base4_dsel(uint8_t pos);
extern void edit_base4_set(uint8_t pos, int8_t step);
extern void edit_base5_sel(uint8_t pos);
extern void edit_base5_dsel(uint8_t pos);
extern void edit_base5_set(uint8_t pos, int8_t step);
extern void edit_consumption_sel(uint8_t pos);
extern void edit_consumption_dsel(uint8_t pos);
extern void edit_consumption_set(uint8_t pos, int8_t step);
extern void edit_unit_sel(uint8_t pos);
extern void edit_unit_dsel(uint8_t pos);
extern void edit_unit_set(uint8_t pos, int8_t step);
extern void edit_filter_sel(uint8_t pos);
extern void edit_filter_dsel(uint8_t pos);
extern void edit_filter_set(uint8_t pos, int8_t step);
extern void edit_threshold_sel(uint8_t pos);
extern void edit_threshold_dsel(uint8_t pos);
extern void edit_threshold_set(uint8_t pos, int8_t step);
extern void up_callback(void);
extern void down_callback(void);
extern void time_callback(enum sys_message msg);
extern void screenTimeout(void);


extern const uint8_t int_to_array_conversion_table[][3];
extern uint8_t *int_to_array(uint32_t n, uint8_t digits, uint8_t blanks);

// *************************************************************************************************
// Defines section


// *************************************************************************************************
// Global Variable section


// *************************************************************************************************
// Extern section

#endif                                             /*ALTITUDE_H_ */
