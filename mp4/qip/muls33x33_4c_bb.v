// megafunction wizard: %LPM_MULT%VBB%
// GENERATION: STANDARD
// VERSION: WM1.0
// MODULE: lpm_mult 

// ============================================================
// File Name: muls33x33_4c.v
// Megafunction Name(s):
// 			lpm_mult
//
// Simulation Library Files(s):
// 			lpm
// ============================================================
// ************************************************************
// THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//
// 18.1.0 Build 625 09/12/2018 SJ Standard Edition
// ************************************************************

//Copyright (C) 2018  Intel Corporation. All rights reserved.
//Your use of Intel Corporation's design tools, logic functions 
//and other software and tools, and its AMPP partner logic 
//functions, and any output files from any of the foregoing 
//(including device programming or simulation files), and any 
//associated documentation or information are expressly subject 
//to the terms and conditions of the Intel Program License 
//Subscription Agreement, the Intel Quartus Prime License Agreement,
//the Intel FPGA IP License Agreement, or other applicable license
//agreement, including, without limitation, that your use is for
//the sole purpose of programming logic devices manufactured by
//Intel and sold by Intel or its authorized distributors.  Please
//refer to the applicable agreement for further details.

module muls33x33_4c (
	clock,
	dataa,
	datab,
	result);

	input	  clock;
	input	[32:0]  dataa;
	input	[32:0]  datab;
	output	[65:0]  result;

endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: AutoSizeResult NUMERIC "1"
// Retrieval info: PRIVATE: B_isConstant NUMERIC "0"
// Retrieval info: PRIVATE: ConstantB NUMERIC "0"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Arria II GX"
// Retrieval info: PRIVATE: LPM_PIPELINE NUMERIC "4"
// Retrieval info: PRIVATE: Latency NUMERIC "1"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: SignedMult NUMERIC "1"
// Retrieval info: PRIVATE: USE_MULT NUMERIC "1"
// Retrieval info: PRIVATE: ValidConstant NUMERIC "0"
// Retrieval info: PRIVATE: WidthA NUMERIC "33"
// Retrieval info: PRIVATE: WidthB NUMERIC "33"
// Retrieval info: PRIVATE: WidthP NUMERIC "66"
// Retrieval info: PRIVATE: aclr NUMERIC "0"
// Retrieval info: PRIVATE: clken NUMERIC "0"
// Retrieval info: PRIVATE: new_diagram STRING "1"
// Retrieval info: PRIVATE: optimize NUMERIC "0"
// Retrieval info: LIBRARY: lpm lpm.lpm_components.all
// Retrieval info: CONSTANT: LPM_HINT STRING "MAXIMIZE_SPEED=5"
// Retrieval info: CONSTANT: LPM_PIPELINE NUMERIC "4"
// Retrieval info: CONSTANT: LPM_REPRESENTATION STRING "SIGNED"
// Retrieval info: CONSTANT: LPM_TYPE STRING "LPM_MULT"
// Retrieval info: CONSTANT: LPM_WIDTHA NUMERIC "33"
// Retrieval info: CONSTANT: LPM_WIDTHB NUMERIC "33"
// Retrieval info: CONSTANT: LPM_WIDTHP NUMERIC "66"
// Retrieval info: USED_PORT: clock 0 0 0 0 INPUT NODEFVAL "clock"
// Retrieval info: USED_PORT: dataa 0 0 33 0 INPUT NODEFVAL "dataa[32..0]"
// Retrieval info: USED_PORT: datab 0 0 33 0 INPUT NODEFVAL "datab[32..0]"
// Retrieval info: USED_PORT: result 0 0 66 0 OUTPUT NODEFVAL "result[65..0]"
// Retrieval info: CONNECT: @clock 0 0 0 0 clock 0 0 0 0
// Retrieval info: CONNECT: @dataa 0 0 33 0 dataa 0 0 33 0
// Retrieval info: CONNECT: @datab 0 0 33 0 datab 0 0 33 0
// Retrieval info: CONNECT: result 0 0 66 0 @result 0 0 66 0
// Retrieval info: GEN_FILE: TYPE_NORMAL muls33x33_4c.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL muls33x33_4c.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL muls33x33_4c.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL muls33x33_4c.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL muls33x33_4c_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL muls33x33_4c_bb.v TRUE
// Retrieval info: LIB_FILE: lpm
