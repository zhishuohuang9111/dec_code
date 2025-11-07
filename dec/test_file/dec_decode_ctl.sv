// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Western Digital Corporation or its affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
		tag: 0 变量的定义
		tag: 1 变量的赋值
		tag: 2 变量的使用
		
		tag_look_start: 从上次开始的地方继续
		
		assign dec_i0_select_pc_d = i0_dp.pc;  // pc 值   
	    assign dec_i1_select_pc_d = i1_dp.pc;  //  输出信号，本模块没有使用
		
		input logic [31:1]  dec_i0_pc_d,                    // pc, pc值, i0 pc at decode     from dec_ib_ctl.sv      
		
*/

//  tag_answer   重点关注 mul alu (mul_pkt_t, alu_pkt_t)
// 指令 和 加的id 一一对应

module dec_decode_ctl
   import veer_types::*;
(
   input logic [15:0] dec_i0_cinst_d,         // 16b compressed instruction 
   input logic [15:0] dec_i1_cinst_d,

   output logic [31:0] dec_i0_inst_wb1,       // 32b instruction at wb+1 for trace encoder
   output logic [31:0] dec_i1_inst_wb1,

   output logic [31:1] dec_i0_pc_wb1,         // 31b pc at wb+1 for trace encoder
   output logic [31:1] dec_i1_pc_wb1,


   input logic                                lsu_nonblock_load_valid_dc3,     // valid nonblock load at dc3
   input logic [`RV_LSU_NUM_NBLOAD_WIDTH-1:0] lsu_nonblock_load_tag_dc3,       // -> corresponding tag
   input logic                                lsu_nonblock_load_inv_dc5,       // invalidate request for nonblock load dc5
   input logic [`RV_LSU_NUM_NBLOAD_WIDTH-1:0] lsu_nonblock_load_inv_tag_dc5,   // -> corresponding tag
   input logic                                lsu_nonblock_load_data_valid,    // valid nonblock load data back
   input logic                                lsu_nonblock_load_data_error,    // nonblock load bus error
   input logic [`RV_LSU_NUM_NBLOAD_WIDTH-1:0] lsu_nonblock_load_data_tag,      // -> corresponding tag

   input logic [3:0] dec_i0_trigger_match_d,          // i0 decode trigger matches
   input logic [3:0] dec_i1_trigger_match_d,          // i1 decode trigger matches

   input logic dec_tlu_wr_pause_wb,                   // pause instruction at wb
   input logic dec_tlu_pipelining_disable,            // pipeline disable - presync, i0 decode only
   input logic dec_tlu_dual_issue_disable,            // i0 decode only

   input logic dec_tlu_sec_alu_disable,               // no alu ops sent to secondary alus

   input logic [3:0]  lsu_trigger_match_dc3,          // lsu trigger matches

   input logic lsu_pmu_misaligned_dc3,                // perf mon: load/store misalign
   input logic dec_tlu_debug_stall,                   // debug stall decode
   input logic dec_tlu_flush_leak_one_wb,             // leak1 instruction

   input logic dec_debug_fence_d,                     // debug fence instruction

   input logic [1:0] dbg_cmd_wrdata,                  // disambiguate fence, fence_i

   // from dec_ib_ctl.sv
   input logic dec_i0_icaf_d,                         // icache access fault
   input logic dec_i1_icaf_d,
   input logic dec_i0_icaf_second_d,                      // i0 instruction access fault on second 2B of 4B inst
   input logic dec_i0_perr_d,                         // icache parity error
   input logic dec_i1_perr_d,
   input logic dec_i0_sbecc_d,                        // icache/iccm single-bit error
   input logic dec_i1_sbecc_d,
   input logic dec_i0_dbecc_d,                        // icache/iccm double-bit error
   input logic dec_i1_dbecc_d,

   input br_pkt_t dec_i0_brp,                         // branch packet
   input br_pkt_t dec_i1_brp,

   input logic [15:0] ifu_illegal_inst,               // 16b illegal inst from aligner

   input logic [31:1] dec_i0_pc_d,                    // pc, pc值, i0 pc at decode     from dec_ib_ctl.sv      look_tag

   input logic [31:1] dec_i1_pc_d, 

   input logic lsu_freeze_dc3,                        // freeze pipe: decode -> dc3
   input logic lsu_halt_idle_any,                     // lsu idle: if fence instr & ~lsu_halt_idle_any then stall decode

   input logic lsu_load_stall_any,                    // stall any store at load
   input logic lsu_store_stall_any,                   // stall any store at decode
   input logic dma_dccm_stall_any,                    // stall any load/store at decode

   input logic exu_div_finish,                        // div finish this cycle
   input logic exu_div_stall,                         // div executing: stall decode
   input logic [31:0] exu_div_result,                 //  tag_result,  div result

   input logic dec_tlu_i0_kill_writeb_wb,    // I0 is flushed, don't writeback any results to arch state
   input logic dec_tlu_i1_kill_writeb_wb,    // I1 is flushed, don't writeback any results to arch state

   input logic dec_tlu_flush_lower_wb,          // trap lower flush

   output logic dec_tlu_flush_lower_wb_final,

   output logic exu_i0_flush_vp_wb, exu_i1_flush_vp_wb,


   input logic dec_tlu_flush_pause_wb,          // don't clear pause state on initial lower flush
   input logic dec_tlu_presync_d,               // CSR read needs to be presync'd
   input logic dec_tlu_postsync_d,              // CSR ops that need to be postsync'd

   input logic [31:0] exu_mul_result_e3,        // tag_result,  multiply result

   input logic dec_i0_pc4_d,               // inst is 4B inst else 2B    from dec_ib_ctl.sv
   input logic dec_i1_pc4_d,

   input logic [31:0] dec_csr_rddata_d,    // csr read data at wb
   input logic dec_csr_legal_d,            // csr indicates legal operation

   input logic [31:0] exu_csr_rs1_e1,      // rs1 for csr instr

   input logic [31:0] lsu_result_dc3,      // load result
   input logic [31:0] lsu_result_corr_dc4, // corrected load result

   input logic exu_i0_flush_final,         // lower flush or i0 flush at e2
   input logic exu_i1_flush_final,         // lower flush or i1 flush at e2

   input logic [31:1] exu_i0_pc_e1,        // pcs at e1
   input logic [31:1] exu_i1_pc_e1,

   input logic [31:0] dec_i0_instr_d,      // inst at decode    在解码阶段表示第 0 个指令（i0）的指令值  from dec_ib_ctl.sv 
   input logic [31:0] dec_i1_instr_d,

   input logic  dec_ib0_valid_d,          // inst valid at decode
   input logic  dec_ib1_valid_d,

   
   
   // 从 exu.sv 文件中传过来的
   input logic [31:0] exu_i0_result_e1,    //  tag_result,  e1 阶段的结果, from primary alu's
   input logic [31:0] exu_i1_result_e1,

   input logic [31:0] exu_i0_result_e4,    // tag_result e4 阶段的结果 from secondary alu's
   input logic [31:0] exu_i1_result_e4,

   input logic i0_is_jal_e4,  
   input logic i1_is_jal_e4,  

   input logic i0_sec_decode_e4,  
   input logic i1_sec_decode_e4,  

   input logic i0_alu_inst_e4,  
   input logic i1_alu_inst_e4,

   output logic i0_result_e4_flush,
   output logic i1_result_e4_flush,

   input logic [31:1] i0_value_flush_e4,
   input logic [31:1] i1_value_flush_e4,   


   input logic [31:1] npc_e4,

   input logic [31:1] pred_correct_npc_e4,  
   
   
   
   
   

   input logic  clk,                       // for rvdffe's
   input logic  active_clk,                // clk except for halt / pause
   input logic  free_clk,                  // free running clock

   input logic  clk_override,              // test stuff
   input logic  rst_l,


   output logic         dec_i0_rs1_en_d,   // rs1 enable at decode  
   output logic         dec_i0_rs2_en_d,

   output logic [4:0] dec_i0_rs1_d,        // rs1 logical source
   output logic [4:0] dec_i0_rs2_d,



   output logic [31:0] dec_i0_immed_d,     // 32b immediate data decode

   output logic          dec_i1_rs1_en_d,
   output logic          dec_i1_rs2_en_d,

   output logic [4:0]  dec_i1_rs1_d,
   output logic [4:0]  dec_i1_rs2_d,



   output logic [31:0] dec_i1_immed_d,

   output logic [12:1] dec_i0_br_immed_d,    // 12b branch immediate
   output logic [12:1] dec_i1_br_immed_d,  // 








   output alu_pkt_t i0_ap,                   // alu packets  tag_answer  控制包,   to exu.sv 文件中 
   output alu_pkt_t i1_ap,     // i1_ap：i1 alu packet




   output logic          dec_i0_decode_d,    // i0 decode, 
   output logic          dec_i1_decode_d,    // 译码完成 发射执行的有效信号 look_tag     // to dec_ib_ctl.sv and to to dec_tlu_ctl.sv


   output logic          dec_i0_decode_d_block,    // i0 decode, 
   output logic          dec_i1_decode_d_block, 

   input logic i0_rd_enable,  i1_rd_enable,

   input logic i0_rd_enable_next, i1_rd_enable_next,





   output logic          dec_ib0_valid_eff_d,   // effective valid taking decode into account
   output logic          dec_ib1_valid_eff_d,

   output logic          dec_i0_alu_decode_d,   // decode to primary alu's
   output logic          dec_i1_alu_decode_d,

    // tag_done
   output logic [31:0] i0_rs1_bypass_data_d,    // i0 rs1 bypass data
   output logic [31:0] i0_rs2_bypass_data_d,    // i0 rs2 bypass data
   output logic [31:0] i1_rs1_bypass_data_d,
   output logic [31:0] i1_rs2_bypass_data_d,   // 传到 exu.sv 文件中

   output logic [31:0] i0_rs1_bypass_data,    // i0 rs1 bypass data
   output logic [31:0] i0_rs2_bypass_data,    // i0 rs2 bypass data
   output logic [31:0] i1_rs1_bypass_data,
   output logic [31:0] i1_rs2_bypass_data,   // 传到 exu.sv 文件中
   
   
   
   
   // Added
   output logic [31:0] i0_rs1_predicted_data_d, 
   output logic [31:0] i0_rs2_predicted_data_d, 
   output logic [31:0] i1_rs1_predicted_data_d,
   output logic [31:0] i1_rs2_predicted_data_d, 
   
   


   output logic [4:0]  dec_i0_waddr_wb,         // i0 logical source to write to gpr's
   output logic          dec_i0_wen_wb,         // i0 write enable
   output logic [31:0] dec_i0_wdata_wb,         // i0 write data

   output logic [4:0]  dec_i1_waddr_wb,
   output logic          dec_i1_wen_wb,
   output logic [31:0] dec_i1_wdata_wb,

   output logic          dec_i0_select_pc_d,    // i0 select pc for rs1 - branches
   output logic          dec_i1_select_pc_d,

   output logic dec_i0_rs1_bypass_en_d,         // i0 rs1 bypass enable
   output logic dec_i0_rs2_bypass_en_d,         // i0 rs2 bypass enable
   output logic dec_i1_rs1_bypass_en_d,
   output logic dec_i1_rs2_bypass_en_d,    // 传到 exu.sv 文件中

   output logic dec_i0_rs1_bypass_en,         // i0 rs1 bypass enable
   output logic dec_i0_rs2_bypass_en,         // i0 rs2 bypass enable
   output logic dec_i1_rs1_bypass_en,
   output logic dec_i1_rs2_bypass_en,    // 传到 exu.sv 文件中
   
   

   
   
   
   // Added
   output logic dec_i0_rs1_predicted_en_d,
   output logic dec_i0_rs2_predicted_en_d,
   output logic dec_i1_rs1_predicted_en_d,
   output logic dec_i1_rs2_predicted_en_d,
   
   
   // tag_lsu_p: 0
   output lsu_pkt_t    lsu_p,                   // load/store packet   

   // tag_mul_p: 0
   output mul_pkt_t    mul_p,                   // multiply packet   tag_answer  乘法控制包

   // tag_div_p: 0
   output div_pkt_t    div_p,                   // divide packet     除法包

   
   
   output logic [11:0] dec_lsu_offset_d,
   
   // tag_dec_i0_lsu_d: 0
   output logic        dec_i0_lsu_d,        // chose which gpr value to use
   output logic        dec_i1_lsu_d,
   output logic        dec_i0_mul_d,        // chose which gpr value to use
   output logic        dec_i1_mul_d,
   output logic        dec_i0_div_d,        // chose which gpr value to use
   output logic        dec_i1_div_d,

   // review
   output logic        flush_final_e3,      // flush final at e3: i0  or i1
   output logic        i0_flush_final_e3,   // i0 flush final at e3

   output logic        dec_csr_ren_d,       // valid csr decode
   output logic        dec_csr_wen_unq_d,       // valid csr with write - for csr legal
   output logic        dec_csr_any_unq_d,       // valid csr - for csr legal
   output logic        dec_csr_wen_wb,      // csr write enable at wb
   output logic [11:0] dec_csr_rdaddr_d,      // read address for csr
   output logic [11:0] dec_csr_wraddr_wb,     // write address for csr
   output logic [31:0] dec_csr_wrdata_wb,   // csr write data at wb
   output logic        dec_csr_stall_int_ff, // csr is mie/mstatus

   output              dec_tlu_i0_valid_e4,  // i0 valid inst at e4
   output              dec_tlu_i1_valid_e4,

   output              trap_pkt_t dec_tlu_packet_e4,   // trap packet

   output logic        dec_fence_pending, // tell TLU to stall DMA
   output logic [31:1] dec_tlu_i0_pc_e4,  // i0 trap pc
   output logic [31:1] dec_tlu_i1_pc_e4,

   output logic [31:0] dec_illegal_inst,        // illegal inst
   output logic        dec_i1_valid_e1,         // i1 valid e1
   output logic        dec_div_decode_e4,       // i0 div e4
   output logic [31:1] pred_correct_npc_e2,     // npc e2 if the prediction is correct
   
   
   output logic        dec_i0_rs1_bypass_en_e3, // i0 rs1 bypass enables e3
   output logic        dec_i0_rs2_bypass_en_e3, // i1 rs1 bypass enables e3
   output logic        dec_i1_rs1_bypass_en_e3,
   output logic        dec_i1_rs2_bypass_en_e3,
   
   // tag_done
   output logic [31:0] i0_rs1_bypass_data_e3,   // i0 rs1 bypass data e3     
   output logic [31:0] i0_rs2_bypass_data_e3,   
   output logic [31:0] i1_rs1_bypass_data_e3,  // i1 rs1 bypass data e3
   output logic [31:0] i1_rs2_bypass_data_e3,
   
   
   output logic        dec_i0_sec_decode_e3,    // i0 secondary alu e3 // tag_dec_i0_sec_decode_e3: 0
   output logic        dec_i1_sec_decode_e3,    // i1 secondary alu e3
   
   output logic [31:1] dec_i0_pc_e3,            // i0 pc e3
   output logic [31:1] dec_i1_pc_e3,            // i1 pc e3

   
   
   output logic        dec_i0_rs1_bypass_en_e2, // i0 rs1 bypass enable e2
   output logic        dec_i0_rs2_bypass_en_e2, // i0 rs2 bypass enable e2
   output logic        dec_i1_rs1_bypass_en_e2,
   output logic        dec_i1_rs2_bypass_en_e2,
   
   // tag_done
   output logic [31:0] i0_rs1_bypass_data_e2,   // i0 rs1 bypass data e2    tag_i0_rs1_bypass_data_e2
   output logic [31:0] i0_rs2_bypass_data_e2,   // i0 rs2 bypass data e2
   output logic [31:0] i1_rs1_bypass_data_e2,
   output logic [31:0] i1_rs2_bypass_data_e2,
   
   

   output predict_pkt_t  i0_predict_p_d,        // i0 predict packet decode
   output predict_pkt_t  i1_predict_p_d,    // 输出到 exu.sv 文件中

   output logic          dec_i0_lsu_decode_d,   // i0 lsu decode

   output logic [31:0] i0_result_e4_eff,        // i0 e4 result taking freeze into account
   output logic [31:0] i1_result_e4_eff,
   output logic [31:0] i0_result_e2,            // i0 result e2
   
   // Added
   output logic [31:0] i0_predicted_result_e2,



   output logic [31:0] i0_predicted_result_e1,
   output logic [31:0] i1_predicted_result_e1,



   output logic [4:2] dec_i0_data_en,           // clock-gating logic
   output logic [4:1] dec_i0_ctl_en,
   output logic [4:2] dec_i1_data_en,
   output logic [4:1] dec_i1_ctl_en,

   output logic [1:0] dec_pmu_instr_decoded,    // number of instructions decode this cycle encoded
   output logic       dec_pmu_decode_stall,     // decode is stalled
   output logic       dec_pmu_presync_stall,    // decode has presync stall
   output logic       dec_pmu_postsync_stall,   // decode has postsync stall

   output logic       dec_nonblock_load_wen,        // write enable for nonblock load
   output logic [4:0] dec_nonblock_load_waddr,      // logical write addr for nonblock load
   output logic       dec_nonblock_load_freeze_dc2, // lsu must freeze nonblock load due to younger dependency in pipe
   output logic       dec_pause_state,              // core in pause state
   output logic       dec_pause_state_cg,           // pause state for clock-gating

   output logic       dec_i0_load_e4,          // pipe down if load is i0 or not in case of lsu_freeze

   input  logic        scan_mode,
   
   
   // input [31:0] 	  i0_predicted_result,
   // input [31:0]       i1_predicted_result,

   /*
   input logic		      i0_cond_corpredict_e1,
   input logic		      i1_cond_corpredict_e1,   

   input logic		      i0_cond_corpredict_e4,
   input logic		      i1_cond_corpredict_e4,  \

   */
   
   output 	logic	      exu_i0_flush_vp_e4,
   output 	logic		  exu_i1_flush_vp_e4, 

   output logic [31:1] exu_i0_flush_path_vp_e4, // slot 0 flush target for mp
   output logic [31:1] exu_i1_flush_path_vp_e4, // slot 1 flush target for mp


   output 	logic	      i0_i1_load_match_i0,
   output 	logic		  i0_i1_load_match_i1, 

   output logic i0_load_match_e4, i1_load_match_e4,
   
   
   output logic test // 在这个 module 里面对 test 进行赋值 test_tag 
   
   

   
   
   
   
   
   );




   dec_pkt_t i0_dp_raw, i0_dp;  // dec_pkt_t: 译码阶段(decode)产生的一些控制信号 包
   dec_pkt_t i1_dp_raw, i1_dp;  

   
   

   logic [31:0]        i0, i1;  // tag_i0: 0
   logic               i0_valid_d, i1_valid_d;

   logic [31:0]        i0_result_e1, i1_result_e1;
   logic [31:0]                      i1_result_e2;
   logic [31:0]        i0_result_e3, i1_result_e3;
   logic [31:0]        i0_result_e4, i1_result_e4;
   logic [31:0]        i0_result_wb, i1_result_wb; 
   
   // Added
   logic [31:0]        i0_predicted_result_d, i1_predicted_result_d;
   // logic [31:0]        i0_predicted_result_e1, i1_predicted_result_e1;
   logic [31:0]                      		   i1_predicted_result_e2;
   logic [31:0]        i0_predicted_result_e3, i1_predicted_result_e3;
   
   logic [31:0]        i0_predicted_result_e4_raw, i1_predicted_result_e4_raw; 
   logic [31:0]        i0_predicted_result_e4, i1_predicted_result_e4;

   logic [31:0]        i0_predicted_result_wb, i1_predicted_result_wb; 



 
   

   logic [31:1]        i0_pc_e1, i1_pc_e1;
   logic [31:1]        i0_pc_e2, i1_pc_e2;
   logic [31:1]        i0_pc_e3, i1_pc_e3;
   logic [31:1]        i0_pc_e4, i1_pc_e4;   


   logic                i0_prediction_valid_e1;
   logic                i0_prediction_valid_e2;
   logic                i0_prediction_valid_e3;
   logic                i0_prediction_valid_e4; 


   logic                i1_prediction_valid_e1;
   logic                i1_prediction_valid_e2;
   logic                i1_prediction_valid_e3;
   logic                i1_prediction_valid_e4; 
   

   logic [9:0]         i0_rs1bypass, i0_rs2bypass;
   logic [9:0]         i1_rs1bypass, i1_rs2bypass;
   
   // Added
   logic [3:0]         i0_rs1predicted, i0_rs2predicted;
   logic [3:0]         i1_rs1predicted, i1_rs2predicted;
 
 
   logic               i0_jalimm20, i1_jalimm20;
   logic               i0_uiimm20, i1_uiimm20;

   //logic             flush_final_e3;

   logic               lsu_decode_d;
   logic [31:0]        i0_immed_d;
   logic               i0_presync;
   logic               i0_postsync;

   logic               postsync_stall;
   logic               ps_stall;

   logic               prior_inflight, prior_inflight_e1e4, prior_inflight_wb;

   logic               csr_clr_d, csr_set_d, csr_write_d;


   logic        csr_clr_e1,csr_set_e1,csr_write_e1,csr_imm_e1;
   logic [31:0] csr_mask_e1;
   logic [31:0] write_csr_data_e1;
   logic [31:0] write_csr_data_in;
   logic [31:0] write_csr_data;
   logic               csr_data_wen;

   logic [4:0]         csrimm_e1;

   logic [31:0]        csr_rddata_e1;

   logic               flush_lower_wb;

   logic               i1_load_block_d;
   logic               i1_mul_block_d;
   logic               i1_load2_block_d;
   logic               i1_mul2_block_d;
   logic               mul_decode_d;
   logic               div_decode_d;
   logic [31:1]        div_pc;
   logic               div_stall, div_stall_ff;
   logic [3:0]         div_trigger;

   logic               i0_legal;
   logic               shift_illegal;
   logic               illegal_inst_en;
   logic [31:0]        illegal_inst;
   logic               illegal_lockout_in, illegal_lockout;  // tag_answer _d decode阶段
   
   logic               i0_legal_decode_d; //  tag_i0_legal_decode_d: 0, 第一条指令 (i0) 在解码阶段 (decode) 是否被合法解码 (legal) 的标志 

   logic               i1_flush_final_e3;

   logic [31:0]        i0_result_e3_final, i1_result_e3_final;
   logic [31:0]        i0_result_wb_raw,   i1_result_wb_raw;
   logic [12:1] last_br_immed_d;
   logic        i1_depend_i0_d;
   
   logic        i0_rs1_depend_i0_e1, i0_rs1_depend_i0_e2, i0_rs1_depend_i0_e3, i0_rs1_depend_i0_e4, i0_rs1_depend_i0_wb; // tag_i0_rs1_depend_i0_e1: 0
   logic        i0_rs1_depend_i1_e1, i0_rs1_depend_i1_e2, i0_rs1_depend_i1_e3, i0_rs1_depend_i1_e4, i0_rs1_depend_i1_wb;
   logic        i0_rs2_depend_i0_e1, i0_rs2_depend_i0_e2, i0_rs2_depend_i0_e3, i0_rs2_depend_i0_e4, i0_rs2_depend_i0_wb;
   logic        i0_rs2_depend_i1_e1, i0_rs2_depend_i1_e2, i0_rs2_depend_i1_e3, i0_rs2_depend_i1_e4, i0_rs2_depend_i1_wb;
   logic        i1_rs1_depend_i0_e1, i1_rs1_depend_i0_e2, i1_rs1_depend_i0_e3, i1_rs1_depend_i0_e4, i1_rs1_depend_i0_wb;
   logic        i1_rs1_depend_i1_e1, i1_rs1_depend_i1_e2, i1_rs1_depend_i1_e3, i1_rs1_depend_i1_e4, i1_rs1_depend_i1_wb;
   logic        i1_rs2_depend_i0_e1, i1_rs2_depend_i0_e2, i1_rs2_depend_i0_e3, i1_rs2_depend_i0_e4, i1_rs2_depend_i0_wb;
   logic        i1_rs2_depend_i1_e1, i1_rs2_depend_i1_e2, i1_rs2_depend_i1_e3, i1_rs2_depend_i1_e4, i1_rs2_depend_i1_wb;
   
   logic        i1_rs1_depend_i0_d, i1_rs2_depend_i0_d;

   logic        i0_secondary_d, i1_secondary_d;
   logic        i0_secondary_block_d, i1_secondary_block_d;
   logic        non_block_case_d;
   logic        i0_div_decode_d;
   logic [31:0] i0_result_e4_final, i1_result_e4_final;
   logic        i0_load_block_d;
   logic        i0_mul_block_d;
   
   logic [3:0]  i0_rs1_depth_d, i0_rs2_depth_d; // tag_i0_rs1_depth_d:0   i0 第一个操作数所处在流水线的那个阶段 ？ 变量名称含义  tag_question 
   logic [3:0]  i1_rs1_depth_d, i1_rs2_depth_d;

   logic        i0_rs1_match_e1_e2, i0_rs1_match_e1_e3;
   logic        i0_rs2_match_e1_e2, i0_rs2_match_e1_e3;
   logic        i1_rs1_match_e1_e2, i1_rs1_match_e1_e3;
   logic        i1_rs2_match_e1_e2, i1_rs2_match_e1_e3;

   logic        i0_load_stall_d,  i1_load_stall_d;
   logic        i0_store_stall_d, i1_store_stall_d;

   logic        i0_predict_nt, i0_predict_t;
   logic        i1_predict_nt, i1_predict_t;

   logic        i0_notbr_error, i0_br_toffset_error;
   logic        i1_notbr_error, i1_br_toffset_error;
   logic        i0_ret_error,   i1_ret_error;
   logic        i0_br_error, i1_br_error;
   
   logic        i0_br_error_all, i1_br_error_all; // tag_i0_br_error_all: 0
   
   logic [11:0] i0_br_offset, i1_br_offset;

   logic        freeze;

   logic [20:1] i0_pcall_imm, i1_pcall_imm;    // predicted jal's
   logic        i0_pcall_12b_offset, i1_pcall_12b_offset;
   logic        i0_pcall_raw,   i1_pcall_raw;
   logic        i0_pcall_case,  i1_pcall_case;
   logic        i0_pcall,  i1_pcall;

   logic        i0_pja_raw,   i1_pja_raw;
   logic        i0_pja_case,  i1_pja_case;
   logic        i0_pja,  i1_pja;

   logic        i0_pret_case, i1_pret_case;
   logic        i0_pret_raw, i0_pret;
   logic        i1_pret_raw, i1_pret;

   logic        i0_jal, i1_jal;  // jal's that are not predicted



   logic        i0_predict_br, i1_predict_br;
   logic        freeze_prior1, freeze_prior2;

   logic [31:0] i0_result_e4_freeze, i1_result_e4_freeze;
   logic [31:0] i0_result_wb_freeze, i1_result_wb_freeze;
   logic [31:0] i1_result_wb_eff, i0_result_wb_eff;
   logic [2:0]  i1rs1_intra, i1rs2_intra;
   logic        i1_rs1_intra_bypass, i1_rs2_intra_bypass;
   
   logic        store_data_bypass_c1, store_data_bypass_c2;
   logic [1:0]  store_data_bypass_e4_c1, store_data_bypass_e4_c2, store_data_bypass_e4_c3; // tag_store_data_bypass_e4_c1: 0
   logic        store_data_bypass_i0_e2_c2;

   class_pkt_t i0_rs1_class_d, i0_rs2_class_d;
   class_pkt_t i1_rs1_class_d, i1_rs2_class_d;

   class_pkt_t i0_dc, i0_e1c, i0_e2c, i0_e3c, i0_e4c, i0_wbc; // tag_i0_dc: 0
   class_pkt_t i1_dc, i1_e1c, i1_e2c, i1_e3c, i1_e4c, i1_wbc;  //  tag_answer  i0_e1c：i0    e1：e1阶段   c：class_pkt_t


   logic i0_rs1_match_e1, i0_rs1_match_e2, i0_rs1_match_e3;
   logic i1_rs1_match_e1, i1_rs1_match_e2, i1_rs1_match_e3;
   logic i0_rs2_match_e1, i0_rs2_match_e2, i0_rs2_match_e3;
   logic i1_rs2_match_e1, i1_rs2_match_e2, i1_rs2_match_e3;

   logic       i0_secondary_stall_d;

   logic       i0_ap_pc2, i0_ap_pc4;
   logic       i1_ap_pc2, i1_ap_pc4;

   logic        div_wen_wb;
   
   logic        i0_rd_en_d; // tag_i0_rd_en_d: 0
   logic        i1_rd_en_d; // tag_i1_rd_en_d: 0
   
   logic [4:0]  i1_rd_d;
   logic [4:0]  i0_rd_d;

   logic        load_ldst_bypass_c1;
   logic        load_mul_rs1_bypass_e1;
   logic        load_mul_rs2_bypass_e1;

   logic        leak1_i0_stall_in, leak1_i0_stall;
   logic        leak1_i1_stall_in, leak1_i1_stall;
   logic        leak1_mode;

   logic        i0_csr_write_only_d;

   logic        prior_inflight_e1e3, prior_inflight_eff;
   logic        any_csr_d;

   logic        prior_csr_write;


   logic [5:0] i0_pipe_en;
   logic       i0_e1_ctl_en, i0_e2_ctl_en, i0_e3_ctl_en, i0_e4_ctl_en, i0_wb_ctl_en; // 变量名称含义（ctl） tag_question
   logic       i0_e1_data_en, i0_e2_data_en, i0_e3_data_en, i0_e4_data_en, i0_wb_data_en, i0_wb1_data_en;

   logic [5:0] i1_pipe_en;
   logic       i1_e1_ctl_en, i1_e2_ctl_en, i1_e3_ctl_en, i1_e4_ctl_en, i1_wb_ctl_en;
   logic       i1_e1_data_en, i1_e2_data_en, i1_e3_data_en, i1_e4_data_en, i1_wb_data_en, i1_wb1_data_en;

   
   logic debug_fence_i;
   logic debug_fence;

   logic i0_csr_write;
   logic presync_stall;

   logic i0_instr_error;
   logic i0_icaf_d;
   logic i1_icaf_d;

   logic i0_not_alu_eff, i1_not_alu_eff;

   logic disable_secondary;

   logic clear_pause;
   logic pause_state_in, pause_state;
   logic pause_stall;

   logic [31:1] i1_pc_wb;

   logic        i0_brp_valid;
   logic        nonblock_load_cancel;
   logic        lsu_idle;
   logic        csr_read_e1;
   
   logic        i0_block_d; 
   logic        i1_block_d; // tag_i0_block_d: 0   阻塞信号：定义、赋值、使用    look_tag
   
   logic        ps_stall_in;

   logic        freeze_after_unfreeze1;
   logic        freeze_after_unfreeze2;
   logic        unfreeze_cycle1;
   logic        unfreeze_cycle2;

   logic        tlu_wr_pause_wb1, tlu_wr_pause_wb2;

   localparam NBLOAD_SIZE     = `RV_LSU_NUM_NBLOAD;
   localparam NBLOAD_SIZE_MSB = `RV_LSU_NUM_NBLOAD-1;
   localparam NBLOAD_TAG_MSB  = `RV_LSU_NUM_NBLOAD_WIDTH-1;

// non block load cam logic

   logic                     cam_write, cam_inv_reset, cam_data_reset;
   logic [NBLOAD_TAG_MSB:0]  cam_write_tag, cam_inv_reset_tag, cam_data_reset_tag;
   
   logic [NBLOAD_SIZE_MSB:0] cam_wen;  // tag_cam_wen: 0

   logic [NBLOAD_TAG_MSB:0]  load_data_tag;
   logic [NBLOAD_SIZE_MSB:0] nonblock_load_write;

   load_cam_pkt_t [NBLOAD_SIZE_MSB:0] cam;
   load_cam_pkt_t [NBLOAD_SIZE_MSB:0] cam_in;

   logic [4:0] nonblock_load_rd;
   logic i1_nonblock_load_stall,     i0_nonblock_load_stall;
   logic i1_nonblock_boundary_stall, i0_nonblock_boundary_stall;
   
   
   logic i0_depend_load_e1_d, i0_depend_load_e2_d; // tag_done
   logic i1_depend_load_e1_d, i1_depend_load_e2_d;
   logic    depend_load_e1_d,  depend_load_e2_d,  depend_load_same_cycle_d;
   logic    depend_load_e2_e1,                    depend_load_same_cycle_e1;
   logic                                          depend_load_same_cycle_e2;
   

   logic nonblock_load_valid_dc4, nonblock_load_valid_wb;
   logic i0_load_kill_wen, i1_load_kill_wen;

   logic found;

   logic cam_reset_same_dest_wb;
   logic [NBLOAD_SIZE_MSB:0] cam_inv_reset_val, cam_data_reset_val;
   logic       i1_wen_wb, i0_wen_wb;

   inst_t i0_itype, i1_itype; // tag_i0_itype: 0, inst_t：enum 类型的结构体

   logic csr_read, csr_write;
   logic i0_br_unpred, i1_br_unpred;

   logic debug_fence_raw;

   logic freeze_before;
   logic freeze_e3, freeze_e4;
   logic [3:0] e4t_i0trigger;
   logic [3:0] e4t_i1trigger;
   logic       e4d_i0load;

   logic [4:0] div_waddr_wb;
   logic [12:1] last_br_immed_e1, last_br_immed_e2; // tag_last_br_immed_e2: 0
   logic [31:0]        i0_inst_d, i1_inst_d;
   logic [31:0]        i0_inst_e1, i1_inst_e1;
   logic [31:0]        i0_inst_e2, i1_inst_e2;
   logic [31:0]        i0_inst_e3, i1_inst_e3;
   logic [31:0]        i0_inst_e4, i1_inst_e4;
   logic [31:0]        i0_inst_wb, i1_inst_wb;
   logic [31:0]        i0_inst_wb1,i1_inst_wb1;

   logic [31:0]        div_inst;
   logic [31:1] i0_pc_wb, i0_pc_wb1;
   logic [31:1]           i1_pc_wb1;
   logic [31:1] last_pc_e2;
   
   logic [30:0] prev_signal; // test_tag

   // tag_i0r: 0
   reg_pkt_t i0r, i1r;

   trap_pkt_t   dt, e1t_in, e1t, e2t_in, e2t, e3t_in, e3t, e4t;  // tag_e1t: 0

   class_pkt_t i0_e4c_in, i1_e4c_in;
    // tag_answer  dd：decode阶段 dest_pkt_t      e1d: e1阶段 dest_pkt_t
   dest_pkt_t  dd, e1d, e2d, e3d, e4d, wbd; //  tag_dd: 0  可能用于存储解码阶段的目标包信息  look_tag
   dest_pkt_t e1d_in, e2d_in, e3d_in, e4d_in; 







/*
                                          // 
   strdata_pkt_t [47 : 0] STR ; // NBWAYSTR = 3, LOGSTR = 4  结构体数组  0 : (3 * (1 << 4))-1

// 定义到调用 getPredStride 模块的 module 里面

   ForUpdate_pkt_t Update [255 : 0]; // 共用一个预测器
   
   ForUpdate_pkt_t i0_U;

   logic i0_predstride;

   // logic [31: 0] i0_seq_no; // 记录指令 i0 的数

   logic [31: 0] seq_no; // 记录 i0 i1 总指令数

   logic [31: 0] seq_commit; // 记录 i0 i1 提交指令的数   总指令数 - 已经提交的指令数 = 还在流水线中的指令数


   // logic [31:0] i0_predicted_value;

   logic eligible;  // 是否可以预测   只计算 alu 指令   i0   i1   


   always_comb begin
     //  i0_U = Update[ i0_seq_no[7 : 0] ]; // 将下标限制在 0 - 255 之间

      i0_U.pc[31:1] = i0_pc_e1[31:1];
      i0_U.predstride = 1'b0;

   end

   always_comb begin
      STR[0].LastValue[31:0] = exu_i0_result_e1[31:0];

      STR[0].Stride = 4'd6;
 
    end

    logic test_val;

    assign test_val = StrideAllocateOrNot();




   assign i0_predstride = 1'b1;

*/

/*
      dec_getStride_ctl dec_i0_predict_value (.*,

                          .STR                  ( STR                         ),   // I 

                          .U                    ( i0_U                        ),   // O                                                                                 
                          // .predstride           ( i0_predstride               ),   // O
                          .predicted_value      ( i0_predicted_value          )    // O
                         
                          );

      dec_updateStride_ctl dec_i0_update_value (.*,

                          

                          .U                    ( i0_U                        ),   //                                                 
                          .actual_value           ( exu_i0_result_e1[31:0]               ),   // 

                          .actual_latency           ( 32'd5               ),   // 

                          .STR                  ( STR                         )  // O

                         
                          );
      
*/


   // dec_dec_ctl i0_dec (.inst(i0[31:0]),.out(i0_dp_raw)); 
/*
	always_comb begin
		U = Update[seq_no & (MAXINFLIGHT - 1)];
		
		U.pc = pc + piece;
		U.predvtage = 1'b0;
		U.predstride = 1'B0;

	end
*/


   
   
   
   assign dd.test = 1'b1; // test_tag
   assign test = dd.test; // test_tag
   

   
   

   assign freeze = lsu_freeze_dc3; // lsu_freeze_dc3: freeze pipe: decode -> dc3 

`ifdef RV_NO_SECONDARY_ALU
   assign disable_secondary = 1;   // tag_disable_secondary: 1
`else
   assign disable_secondary        = dec_tlu_sec_alu_disable; // dec_tlu_sec_alu_disable: no alu ops sent to secondary alus  
`endif







// i0_predict_p_d, i1_predict_p_d 结构体变量的赋值    begin     //  i0_predict_p_d, i1_predict_p_d: output, to exu.sv

// branch prediction

   // in leak1_mode, ignore any predictions for i0, treat branch as if we haven't seen it before
   // in leak1 mode, also ignore branch errors for i0
   assign i0_brp_valid = dec_i0_brp.valid & ~leak1_mode;

   assign      i0_predict_p_d.misp    =  '0;  // tag_i0_predict_p_d: 1,   i0_predict_p_d: i0 predict packet decode
   assign      i0_predict_p_d.ataken  =  '0;
   assign      i0_predict_p_d.boffset =  '0;

   assign      i0_predict_p_d.pcall  =  i0_pcall;  // dont mark as pcall if branch error
   assign      i0_predict_p_d.pja    =  i0_pja;
   assign      i0_predict_p_d.pret   =  i0_pret;
   assign      i0_predict_p_d.prett[31:1] = dec_i0_brp.prett[31:1];
   assign      i0_predict_p_d.pc4 = dec_i0_pc4_d;
   assign      i0_predict_p_d.hist[1:0] = dec_i0_brp.hist[1:0];
   assign      i0_predict_p_d.valid = i0_brp_valid & i0_legal_decode_d;
   assign      i0_notbr_error = i0_brp_valid & ~(i0_dp_raw.condbr | i0_pcall_raw | i0_pja_raw | i0_pret_raw);

   // no toffset error for a pret
   assign      i0_br_toffset_error = i0_brp_valid & dec_i0_brp.hist[1] & (dec_i0_brp.toffset[11:0] != i0_br_offset[11:0]) & !i0_pret_raw;
   assign      i0_ret_error = i0_brp_valid & dec_i0_brp.ret & ~i0_pret_raw;
   assign      i0_br_error =  dec_i0_brp.br_error | i0_notbr_error | i0_br_toffset_error | i0_ret_error;
   
   assign      i0_predict_p_d.br_error = i0_br_error & i0_legal_decode_d & ~leak1_mode;
   assign      i0_predict_p_d.br_start_error = dec_i0_brp.br_start_error & i0_legal_decode_d & ~leak1_mode;
   assign      i0_predict_p_d.index[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO] = dec_i0_brp.index[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO];
   assign      i0_predict_p_d.bank[1:0] = dec_i0_brp.bank[1:0];
   assign      i0_predict_p_d.btag[`RV_BTB_BTAG_SIZE-1:0] = dec_i0_brp.btag[`RV_BTB_BTAG_SIZE-1:0];
   
   assign      i0_br_error_all = (i0_br_error | dec_i0_brp.br_start_error) & ~leak1_mode; // tag_i0_br_error_all: 1
   
   assign      i0_predict_p_d.toffset[11:0] = i0_br_offset[11:0];
   assign      i0_predict_p_d.fghr[`RV_BHT_GHR_RANGE] = dec_i0_brp.fghr[`RV_BHT_GHR_RANGE];
   assign      i0_predict_p_d.way = dec_i0_brp.way;


   assign      i1_predict_p_d.misp    =  '0;
   assign      i1_predict_p_d.ataken  =  '0;
   assign      i1_predict_p_d.boffset =  '0;

   assign      i1_predict_p_d.pcall  =  i1_pcall;
   assign      i1_predict_p_d.pja    =  i1_pja;
   assign      i1_predict_p_d.pret   =  i1_pret;
   assign      i1_predict_p_d.prett[31:1] = dec_i1_brp.prett[31:1];
   assign      i1_predict_p_d.pc4 = dec_i1_pc4_d;
   assign      i1_predict_p_d.hist[1:0] = dec_i1_brp.hist[1:0];
   assign      i1_predict_p_d.valid = dec_i1_brp.valid & dec_i1_decode_d;
   assign      i1_notbr_error = dec_i1_brp.valid & ~(i1_dp_raw.condbr | i1_pcall_raw | i1_pja_raw | i1_pret_raw);


   assign      i1_br_toffset_error = dec_i1_brp.valid & dec_i1_brp.hist[1] & (dec_i1_brp.toffset[11:0] != i1_br_offset[11:0]) & !i1_pret_raw;
   assign      i1_ret_error = dec_i1_brp.valid & dec_i1_brp.ret & ~i1_pret_raw;
   assign      i1_br_error = dec_i1_brp.br_error | i1_notbr_error | i1_br_toffset_error | i1_ret_error;
   
   assign      i1_predict_p_d.br_error = i1_br_error & dec_i1_decode_d;
   assign      i1_predict_p_d.br_start_error = dec_i1_brp.br_start_error & dec_i1_decode_d;
   assign      i1_predict_p_d.index[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO] = dec_i1_brp.index[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO];
   assign      i1_predict_p_d.bank[1:0] = dec_i1_brp.bank[1:0];
   assign      i1_predict_p_d.btag[`RV_BTB_BTAG_SIZE-1:0] = dec_i1_brp.btag[`RV_BTB_BTAG_SIZE-1:0];
   
   assign      i1_br_error_all = (i1_br_error | dec_i1_brp.br_start_error); // tag_i1_br_error_all: 1
   
   assign      i1_predict_p_d.toffset[11:0] = i1_br_offset[11:0];
   assign      i1_predict_p_d.fghr[`RV_BHT_GHR_RANGE] = dec_i1_brp.fghr[`RV_BHT_GHR_RANGE];
   assign      i1_predict_p_d.way = dec_i1_brp.way;

   //   end
   
// i0_predict_p_d, i1_predict_p_d 结构体变量的赋值    end     //  i0_predict_p_d, i1_predict_p_d: output, to exu.sv
   
   
   
   
   
   
   

   // on br error turn anything into a nop
   // on i0 instruction fetch access fault turn anything into a nop
   // nop =>   alu rs1 imm12 rd lor

   assign i0_icaf_d = dec_i0_icaf_d | dec_i0_dbecc_d;
   assign i1_icaf_d = dec_i1_icaf_d | dec_i1_dbecc_d;  // tag_i1_icaf_d: 1

   assign i0_instr_error = i0_icaf_d | dec_i0_perr_d | dec_i0_sbecc_d;  // tag_i0_instr_error: 1

  

 
   
// i0_dp, i1_dp 变量的赋值   begin   
   
   // tag_i0_dp: 1
   always_comb begin
      i0_dp = i0_dp_raw; // tag_i0_dp_raw: 2
      if (i0_br_error_all | i0_instr_error) begin  // tag_i0_br_error_all: 2, tag_i0_instr_error: 2
         i0_dp = '0;
         i0_dp.alu = 1'b1; // i0_dp.alu的赋值   look_tag
         i0_dp.rs1 = 1'b1;
         i0_dp.rs2 = 1'b1;
         i0_dp.lor = 1'b1;
         i0_dp.legal = 1'b1;
         i0_dp.postsync = 1'b1;
      end

	  // tag_i1_dp: i1_dp 变量的赋值 
      i1_dp = i1_dp_raw;  // tag_i1_dp_raw: 2
      if (i1_br_error_all) begin // tag_i1_br_error_all: 2
         i1_dp = '0;
         i1_dp.alu = 1'b1;
         i1_dp.rs1 = 1'b1;
         i1_dp.rs2 = 1'b1;
         i1_dp.lor = 1'b1;
         i1_dp.legal = 1'b1;
         i1_dp.postsync = 1'b1;
      end

   end

// i0_dp, i1_dp 变量的赋值   end   

   // logic exu_i0_flush_vp_wb, exu_i1_flush_vp_wb;

   

   // assign dec_tlu_flush_lower_wb_final = dec_tlu_flush_lower_wb | exu_i0_flush_vp_wb | exu_i1_flush_vp_wb;


   rvdff #(2)  free_value_ff (.*,   .clk(free_clk),
   .din( {exu_i0_flush_vp_e4, exu_i1_flush_vp_e4 } ),
   .dout({exu_i0_flush_vp_wb, exu_i1_flush_vp_wb }));
   
   
   
   
   
   // 值预测错误的时候,  刷新流水线,  但是应该 只有一个周期, 但是用 exu_i0_flush_vp_e4 和 exu_i1_flush_vp_e4  信号好像不行
   // assign flush_lower_wb = dec_tlu_flush_lower_wb | exu_i0_flush_vp_wb | exu_i1_flush_vp_wb; // 
   assign flush_lower_wb = dec_tlu_flush_lower_wb ;

   // tag_i0: 1
   assign i0[31:0] = dec_i0_instr_d[31:0];  // dec_i0_instr_d: inst at decode,    from dec_ib_ctl.sv
   assign i1[31:0] = dec_i1_instr_d[31:0];



   assign dec_i0_select_pc_d = i0_dp.pc;  // i0_dp.pc 为一位信号  
   assign dec_i1_select_pc_d = i1_dp.pc;  //  ouput, to exu.sv

   // branches that can be predicted

   assign i0_predict_br =  i0_dp.condbr | i0_pcall | i0_pja | i0_pret;
   assign i1_predict_br =  i1_dp.condbr | i1_pcall | i1_pja | i1_pret;

   assign i0_predict_nt = ~(dec_i0_brp.hist[1] & i0_brp_valid) & i0_predict_br;
   assign i0_predict_t  =  (dec_i0_brp.hist[1] & i0_brp_valid) & i0_predict_br;

   
   
   
   
   
// i0_ap: i0 alu_pkt_t, i1_ap: i1 alu_pkt_t 结构体变量的赋值   begin   tag_i0_ap: 1     tag_i0_dp: 2

	// i0_ap, i1_ap: 两个 output, to exu.sv
	
   assign i0_ap.valid =  (i0_dc.sec | i0_dc.alu | i0_dp.alu ); // tag_i0_dc: 2, i0_dc.sec: i0 decode class_pkt_t 
   assign i0_ap.add =    i0_dp.add;
   assign i0_ap.sub =    i0_dp.sub;
   assign i0_ap.land =   i0_dp.land;
   assign i0_ap.lor =    i0_dp.lor;
   assign i0_ap.lxor =   i0_dp.lxor;
   assign i0_ap.sll =    i0_dp.sll;
   assign i0_ap.srl =    i0_dp.srl;
   assign i0_ap.sra =    i0_dp.sra;
   assign i0_ap.slt =    i0_dp.slt;
   assign i0_ap.unsign = i0_dp.unsign;
   assign i0_ap.beq =    i0_dp.beq;
   assign i0_ap.bne =    i0_dp.bne;
   assign i0_ap.blt =    i0_dp.blt;
   assign i0_ap.bge =    i0_dp.bge;



   assign i0_ap.csr_write = i0_csr_write_only_d;
   assign i0_ap.csr_imm = i0_dp.csr_imm;  


   assign i0_ap.jal    =  i0_jal;


   assign i0_ap_pc2 = ~dec_i0_pc4_d;
   assign i0_ap_pc4 =  dec_i0_pc4_d;  // other 

   assign i0_ap.predict_nt = i0_predict_nt;
   assign i0_ap.predict_t  = i0_predict_t;

   assign i1_predict_nt = ~(dec_i1_brp.hist[1] & dec_i1_brp.valid) & i1_predict_br;
   assign i1_predict_t  =  (dec_i1_brp.hist[1] & dec_i1_brp.valid) & i1_predict_br;

   assign i1_ap.valid =  (i1_dc.sec | i1_dc.alu | i1_dp.alu); // tag_i1_ap: 1
   assign i1_ap.add =    i1_dp.add;
   assign i1_ap.sub =    i1_dp.sub;
   assign i1_ap.land =   i1_dp.land;
   assign i1_ap.lor =    i1_dp.lor;
   assign i1_ap.lxor =   i1_dp.lxor;
   assign i1_ap.sll =    i1_dp.sll;
   assign i1_ap.srl =    i1_dp.srl;
   assign i1_ap.sra =    i1_dp.sra;
   assign i1_ap.slt =    i1_dp.slt;
   assign i1_ap.unsign = i1_dp.unsign;
   assign i1_ap.beq =    i1_dp.beq;
   assign i1_ap.bne =    i1_dp.bne;
   assign i1_ap.blt =    i1_dp.blt;
   assign i1_ap.bge =    i1_dp.bge;

   assign i1_ap.csr_write = 1'b0;
   assign i1_ap.csr_imm   = 1'b0;

   assign i1_ap.jal    =    i1_jal;

   assign i1_ap_pc2 = ~dec_i1_pc4_d;
   assign i1_ap_pc4 =  dec_i1_pc4_d;

   assign i1_ap.predict_nt = i1_predict_nt;
   assign i1_ap.predict_t  = i1_predict_t;
   

// i0_ap, i1_ap 结构体变量的赋值   end

   
   
   
   
   
   always_comb begin
      found = 0;
      cam_wen[NBLOAD_SIZE_MSB:0] = '0;   
      for (int i=0; i<NBLOAD_SIZE; i++) begin
         if (~found) begin  // found 为 1 时, if 语句不会执行
            if (~cam[i].valid) begin
               cam_wen[i] = cam_write;  // tag_cam_wen: 1
               found = 1'b1;
            end
         end
      end
   end


   assign cam_reset_same_dest_wb = wbd.i0v & wbd.i1v & (wbd.i0rd[4:0] == wbd.i1rd[4:0]) &
                                   wbd.i0load & nonblock_load_valid_wb & ~dec_tlu_i0_kill_writeb_wb & ~dec_tlu_i1_kill_writeb_wb;


   assign cam_write          = lsu_nonblock_load_valid_dc3;
   assign cam_write_tag[NBLOAD_TAG_MSB:0] = lsu_nonblock_load_tag_dc3[NBLOAD_TAG_MSB:0];

   assign cam_inv_reset          = lsu_nonblock_load_inv_dc5 | cam_reset_same_dest_wb;
   assign cam_inv_reset_tag[NBLOAD_TAG_MSB:0] = lsu_nonblock_load_inv_tag_dc5[NBLOAD_TAG_MSB:0];

   assign cam_data_reset          = lsu_nonblock_load_data_valid | lsu_nonblock_load_data_error;
   assign cam_data_reset_tag[NBLOAD_TAG_MSB:0] = lsu_nonblock_load_data_tag[NBLOAD_TAG_MSB:0];

   assign nonblock_load_rd[4:0] = (e3d.i0load) ? e3d.i0rd[4:0] : e3d.i1rd[4:0];  // rd data


   // checks

`ifdef ASSERT_ON
   assert_dec_data_valid_data_error_onehot:    assert #0 ($onehot0({lsu_nonblock_load_data_valid,lsu_nonblock_load_data_error}));

   assert_dec_cam_inv_reset_onehot:    assert #0 ($onehot0(cam_inv_reset_val[NBLOAD_SIZE_MSB:0]));
   assert_dec_cam_data_reset_onehot:   assert #0 ($onehot0(cam_data_reset_val[NBLOAD_SIZE_MSB:0]));
`endif





/*
		实现了一个基于CAM（内容可寻址存储器）的机制，用于在系统中处理多个负载指令写入同一个目标寄存器（例如 x1），并在需要时使旧数据无效。
		核心思想是，每当有新数据写入时，旧数据必须失效，以保持一致性。
*/
    // case of multiple loads to same dest ie. x1 ... you have to invalidate the older one

   for (genvar i=0; i<NBLOAD_SIZE; i++) begin : cam_array

      assign cam_inv_reset_val[i] = cam_inv_reset   & (cam_inv_reset_tag[NBLOAD_TAG_MSB:0]  == cam[i].tag[NBLOAD_TAG_MSB:0]) & cam[i].valid;

      assign cam_data_reset_val[i] = cam_data_reset & (cam_data_reset_tag[NBLOAD_TAG_MSB:0] == cam[i].tag[NBLOAD_TAG_MSB:0]) & cam[i].valid;


      always_comb begin
         cam_in[i] = '0;

         if (cam_wen[i]) begin   // tag_cam_wen: 2
            cam_in[i].valid    = 1'b1;
            cam_in[i].wb       = 1'b0;
            cam_in[i].tag[NBLOAD_TAG_MSB:0] = cam_write_tag[NBLOAD_TAG_MSB:0];
            cam_in[i].rd[4:0]  = nonblock_load_rd[4:0];
         end
         else if ( (cam_inv_reset_val[i]) |
                   (cam_data_reset_val[i]) |
                   (i0_wen_wb & (wbd.i0rd[4:0] == cam[i].rd[4:0]) & cam[i].wb) |
                   (i1_wen_wb & (wbd.i1rd[4:0] == cam[i].rd[4:0]) & cam[i].wb) )
           cam_in[i].valid = 1'b0;
         else
           cam_in[i] = cam[i];


         if (nonblock_load_valid_wb & (lsu_nonblock_load_inv_tag_dc5[NBLOAD_TAG_MSB:0]==cam[i].tag[NBLOAD_TAG_MSB:0]) & cam[i].valid)
           cam_in[i].wb = 1'b1;
      end

   rvdff #( $bits(load_cam_pkt_t) ) cam_ff (.*, .clk(free_clk), .din(cam_in[i]), .dout(cam[i]));



   assign nonblock_load_write[i] = (load_data_tag[NBLOAD_TAG_MSB:0] == cam[i].tag[NBLOAD_TAG_MSB:0]) & cam[i].valid;



end : cam_array





   assign load_data_tag[NBLOAD_TAG_MSB:0] = lsu_nonblock_load_data_tag[NBLOAD_TAG_MSB:0];

`ifdef ASSERT_ON
   assert_dec_cam_nonblock_load_write_onehot:   assert #0 ($onehot0(nonblock_load_write[NBLOAD_SIZE_MSB:0]));
`endif


   assign nonblock_load_cancel = ((wbd.i0rd[4:0] == dec_nonblock_load_waddr[4:0]) & i0_wen_wb) |    // cancel if any younger inst (including another nonblock) committing this cycle
                                 ((wbd.i1rd[4:0] == dec_nonblock_load_waddr[4:0]) & i1_wen_wb);


   assign dec_nonblock_load_wen = lsu_nonblock_load_data_valid & |nonblock_load_write[NBLOAD_SIZE_MSB:0] & ~nonblock_load_cancel;

   always_comb begin
      dec_nonblock_load_waddr[4:0] = '0;
      i0_nonblock_load_stall = i0_nonblock_boundary_stall; // tag_i0_nonblock_boundary_stall: 2
      i1_nonblock_load_stall = i1_nonblock_boundary_stall;

      for (int i=0; i<NBLOAD_SIZE; i++) begin
         dec_nonblock_load_waddr[4:0] |= ({5{nonblock_load_write[i]}} & cam[i].rd[4:0]);

         i0_nonblock_load_stall |= dec_i0_rs1_en_d & cam[i].valid & (cam[i].rd[4:0] == i0r.rs1[4:0]);
         i0_nonblock_load_stall |= dec_i0_rs2_en_d & cam[i].valid & (cam[i].rd[4:0] == i0r.rs2[4:0]);

         i1_nonblock_load_stall |= dec_i1_rs1_en_d & cam[i].valid & (cam[i].rd[4:0] == i1r.rs1[4:0]);
         i1_nonblock_load_stall |= dec_i1_rs2_en_d & cam[i].valid & (cam[i].rd[4:0] == i1r.rs2[4:0]);

      end
   end

   assign i0_nonblock_boundary_stall = ((nonblock_load_rd[4:0]==i0r.rs1[4:0]) & lsu_nonblock_load_valid_dc3 & dec_i0_rs1_en_d) |
                                       ((nonblock_load_rd[4:0]==i0r.rs2[4:0]) & lsu_nonblock_load_valid_dc3 & dec_i0_rs2_en_d); // tag_i0_nonblock_boundary_stall: 1

   assign i1_nonblock_boundary_stall = ((nonblock_load_rd[4:0]==i1r.rs1[4:0]) & lsu_nonblock_load_valid_dc3 & dec_i1_rs1_en_d) |
                                       ((nonblock_load_rd[4:0]==i1r.rs2[4:0]) & lsu_nonblock_load_valid_dc3 & dec_i1_rs2_en_d);

	
	
	
	
	
	
	
//    dec_nonblock_load_freeze_dc2 变量赋值, output，本模块没有使用   begin
	
	// tag_i0_br_offset: 1
   assign i0_depend_load_e1_d = ((i0_rs1_class_d.load & (i0_rs1_depth_d[3:0]==4'd1 | i0_rs1_depth_d[3:0]==4'd2)) |
                                 (i0_rs2_class_d.load & (i0_rs2_depth_d[3:0]==4'd1 | i0_rs2_depth_d[3:0]==4'd2))) & dec_i0_decode_d;

   assign i0_depend_load_e2_d = ((i0_rs1_class_d.load & (i0_rs1_depth_d[3:0]==4'd3 | i0_rs1_depth_d[3:0]==4'd4)) |
                                 (i0_rs2_class_d.load & (i0_rs2_depth_d[3:0]==4'd3 | i0_rs2_depth_d[3:0]==4'd4))) & dec_i0_decode_d;

   assign i1_depend_load_e1_d = ((i1_rs1_class_d.load & (i1_rs1_depth_d[3:0]==4'd1 | i1_rs1_depth_d[3:0]==4'd2)) |
                                 (i1_rs2_class_d.load & (i1_rs2_depth_d[3:0]==4'd1 | i1_rs2_depth_d[3:0]==4'd2))) & dec_i1_decode_d;

   assign i1_depend_load_e2_d = ((i1_rs1_class_d.load & (i1_rs1_depth_d[3:0]==4'd3 | i1_rs1_depth_d[3:0]==4'd4)) |
                                 (i1_rs2_class_d.load & (i1_rs2_depth_d[3:0]==4'd3 | i1_rs2_depth_d[3:0]==4'd4))) & dec_i1_decode_d;



   assign depend_load_e1_d = i0_depend_load_e1_d | i1_depend_load_e1_d; // tag_i0_br_offset: 2

   assign depend_load_e2_d = i0_depend_load_e2_d | i1_depend_load_e2_d;

   assign depend_load_same_cycle_d = i1_depend_i0_d & i0_dp.load & dec_i1_decode_d;  // tag_dec_i1_decode_d: 2


   rvdffs #(2) e1loadff (.*,
                         .clk(active_clk),
                         .en(i0_e1_ctl_en),
                         .din( {depend_load_e1_d,  depend_load_same_cycle_d}),
                         .dout({depend_load_e2_e1, depend_load_same_cycle_e1})
                         );

   rvdffs #(1) e2loadff (.*,
                         .clk(active_clk),
                         .en(i0_e2_ctl_en),
                         .din( depend_load_same_cycle_e1),
                         .dout(depend_load_same_cycle_e2)
                         );

   assign dec_nonblock_load_freeze_dc2 = depend_load_e2_d | depend_load_e2_e1 | depend_load_same_cycle_e2;  // output, to lsu.sv

//    dec_nonblock_load_freeze_dc2 变量赋值, output，本模块没有使用   end
   
   
   
   
   
   
   
   
// don't writeback a nonblock load

   rvdffs #(1) e4nbloadff (.*, .clk(active_clk), .en(i0_e4_ctl_en), .din(lsu_nonblock_load_valid_dc3),  .dout(nonblock_load_valid_dc4) );
   rvdffs #(1) wbnbloadff (.*, .clk(active_clk), .en(i0_wb_ctl_en), .din(    nonblock_load_valid_dc4),  .dout(nonblock_load_valid_wb) );

   // illegal for i0load and i1load same time
   assign i0_load_kill_wen = nonblock_load_valid_wb &  wbd.i0load;
   assign i1_load_kill_wen = nonblock_load_valid_wb &  wbd.i1load;




// end non block load cam logic




// pmu start
// PMU: Performance Monitoring Unit （处理器中性能监控单元（PMU）的一部分，用于跟踪各种指令类型和执行事件，以收集性能指标。）


   assign csr_read = dec_csr_ren_d;
   assign csr_write = dec_csr_wen_unq_d;

   // tag_i0_br_unpred: 1
   assign i0_br_unpred = (i0_dp.condbr | i0_dp.jal) & ~i0_predict_br; // i0_br_unpred 表示分支预测失败的情况
   assign i1_br_unpred = (i1_dp.condbr | i1_dp.jal) & ~i1_predict_br; // 如果当前指令 (i0_dp 或 i1_dp) 是条件分支 (condbr) 或非条件跳转 (jal)，并且该指令未被预测为分支 (~i0_predict_br 或 ~i1_predict_br)，则该信号被置位。

   // the classes must be mutually exclusive with one another  每条指令类型必须是互斥的，即每次只能设置一个类型。

   // tag_i0_itype: 1
   always_comb begin
      i0_itype = NULL; 
      i1_itype = NULL; // i0_itype, i1_itype 结构体类型 inst_t: instruction type

      if (i0_legal_decode_d) begin
         if (i0_dp.mul)                  i0_itype = MUL;
         if (i0_dp.load)                 i0_itype = LOAD;
         if (i0_dp.store)                i0_itype = STORE;
         if (i0_dp.pm_alu)               i0_itype = ALU;
         if ( csr_read & ~csr_write)     i0_itype = CSRREAD;
         if (~csr_read &  csr_write)     i0_itype = CSRWRITE;
         if ( csr_read &  csr_write)     i0_itype = CSRRW;
         if (i0_dp.ebreak)               i0_itype = EBREAK;
         if (i0_dp.ecall)                i0_itype = ECALL;
         if (i0_dp.fence)                i0_itype = FENCE;
         if (i0_dp.fence_i)              i0_itype = FENCEI;  // fencei will set this even with fence attribute
         if (i0_dp.mret)                 i0_itype = MRET;
         if (i0_dp.condbr)               i0_itype = CONDBR;
         if (i0_dp.jal)                  i0_itype = JAL;
      end

      if (dec_i1_decode_d) begin
         if (i1_dp.mul)                  i1_itype = MUL;
         if (i1_dp.load)                 i1_itype = LOAD;
         if (i1_dp.store)                i1_itype = STORE;
         if (i1_dp.pm_alu)               i1_itype = ALU;
         if (i1_dp.condbr)               i1_itype = CONDBR;
         if (i1_dp.jal)                  i1_itype = JAL;
      end

   end


// end pmu





	// 实例化两个 dec_dec_ctl 译码模块,   通过实例化，对 i0_dp_raw, i1_dp_raw 进行赋值（译码模块原始的输出）
	// i0_dp_raw 的值仅仅通过 i0指令的 位逻辑运算 得到的
   dec_dec_ctl i0_dec (.inst(i0[31:0]),.out(i0_dp_raw)); 

   dec_dec_ctl i1_dec (.inst(i1[31:0]),.out(i1_dp_raw)); 




   
   
   
   rvdff #(1) lsu_idle_ff (.*, .clk(active_clk), .din(lsu_halt_idle_any), .dout(lsu_idle));


   // can't make this clock active_clock
   assign leak1_i1_stall_in = (dec_tlu_flush_leak_one_wb | (leak1_i1_stall & ~dec_tlu_flush_lower_wb));

   rvdff #(1) leak1_i1_stall_ff (.*, .clk(free_clk), .din(leak1_i1_stall_in), .dout(leak1_i1_stall));

   assign leak1_mode = leak1_i1_stall;

   assign leak1_i0_stall_in = ((dec_i0_decode_d & leak1_i1_stall) | (leak1_i0_stall & ~dec_tlu_flush_lower_wb));

   rvdff #(1) leak1_i0_stall_ff (.*, .clk(free_clk), .din(leak1_i0_stall_in), .dout(leak1_i0_stall));



   // 12b jal's can be predicted - these are calls

   assign i0_pcall_imm[20:1] = {i0[31],i0[19:12],i0[20],i0[30:21]};

   assign i0_pcall_12b_offset = (i0_pcall_imm[12]) ? (i0_pcall_imm[20:13] == 8'hff) : (i0_pcall_imm[20:13] == 8'h0);

   assign i0_pcall_case  = i0_pcall_12b_offset & i0_dp_raw.imm20 & i0r.rd[4:0]!=5'b0;
   assign i0_pja_case    = i0_pcall_12b_offset & i0_dp_raw.imm20 & i0r.rd[4:0]==5'b0;

   assign i1_pcall_imm[20:1] = {i1[31],i1[19:12],i1[20],i1[30:21]};

   assign i1_pcall_12b_offset = (i1_pcall_imm[12]) ? (i1_pcall_imm[20:13] == 8'hff) : (i1_pcall_imm[20:13] == 8'h0);

   assign i1_pcall_case  = i1_pcall_12b_offset & i1_dp_raw.imm20 & i1r.rd[4:0]!=5'b0;
   assign i1_pja_case    = i1_pcall_12b_offset & i1_dp_raw.imm20 & i1r.rd[4:0]==5'b0;


   assign i0_pcall_raw = i0_dp_raw.jal &   i0_pcall_case;   // this includes ja
   assign i0_pcall     = i0_dp.jal     &   i0_pcall_case;

   assign i1_pcall_raw = i1_dp_raw.jal &   i1_pcall_case;
   assign i1_pcall     = i1_dp.jal     &   i1_pcall_case;

   assign i0_pja_raw = i0_dp_raw.jal &   i0_pja_case;
   assign i0_pja     = i0_dp.jal     &   i0_pja_case;  

   assign i1_pja_raw = i1_dp_raw.jal &   i1_pja_case;
   assign i1_pja     = i1_dp.jal     &   i1_pja_case;



   assign i0_br_offset[11:0] = (i0_pcall_raw | i0_pja_raw) ? i0_pcall_imm[12:1] : {i0[31],i0[7],i0[30:25],i0[11:8]};  // tag_i0_br_offset: 1

   assign i1_br_offset[11:0] = (i1_pcall_raw | i1_pja_raw) ? i1_pcall_imm[12:1] : {i1[31],i1[7],i1[30:25],i1[11:8]};

   //

   assign i0_pret_case = (i0_dp_raw.jal & i0_dp_raw.imm12 & i0r.rd[4:0]==5'b0 & i0r.rs1[4:0]==5'b1);  // jalr with rd==0, rs1==1 is a ret
   assign i1_pret_case = (i1_dp_raw.jal & i1_dp_raw.imm12 & i1r.rd[4:0]==5'b0 & i1r.rs1[4:0]==5'b1);

   assign i0_pret_raw = i0_dp_raw.jal &   i0_pret_case;
   assign i0_pret    = i0_dp.jal     &   i0_pret_case;

   assign i1_pret_raw = i1_dp_raw.jal &   i1_pret_case;
   assign i1_pret     = i1_dp.jal     &   i1_pret_case;

   assign i0_jal    = i0_dp.jal  &  ~i0_pcall_case & ~i0_pja_case & ~i0_pret_case;
   assign i1_jal    = i1_dp.jal  &  ~i1_pcall_case & ~i1_pja_case & ~i1_pret_case;

   
   
   // lsu stuff
   // load/store mutually exclusive
   assign dec_lsu_offset_d[11:0] =
                                   ({12{ i0_dp.lsu & i0_dp.load}} &               i0[31:20]) |
                                   ({12{~i0_dp.lsu & i1_dp.lsu & i1_dp.load}} &   i1[31:20]) |
                                   ({12{ i0_dp.lsu & i0_dp.store}} &             {i0[31:25],i0[11:7]}) |
                                   ({12{~i0_dp.lsu & i1_dp.lsu & i1_dp.store}} & {i1[31:25],i1[11:7]});   //  output, to lsu.sv


								   
								   
								   
	
   assign dec_i0_lsu_d = i0_dp.lsu;  // dec_i0_lsu_d: chose which gpr value to use
   assign dec_i1_lsu_d = i1_dp.lsu;

   assign dec_i0_mul_d = i0_dp.mul;
   assign dec_i1_mul_d = i1_dp.mul;

   assign dec_i0_div_d = i0_dp.div;
   assign dec_i1_div_d = i1_dp.div; //  六个 output, to exu.sv

   
 
// div_p, mul_p, lsu_p 结构体的赋值  begin     三个 ouput 结构体，在本模块没有使用

		// div_p, mul_p: to exu.sv
		
		// lsu_p: to lsu.sv
   
   // tag_div_p: 1
   assign div_p.valid = div_decode_d;  // tag_div_decode_d: 2

   assign div_p.unsign = (i0_dp.div) ? i0_dp.unsign :   i1_dp.unsign;
   assign div_p.rem  =   (i0_dp.div) ? i0_dp.rem    :   i1_dp.rem;

   
	// tag_mul_p: 1
   assign mul_p.valid = mul_decode_d; // tag_mul_decode_d: 2

   assign mul_p.rs1_sign =   (i0_dp.mul) ? i0_dp.rs1_sign :   i1_dp.rs1_sign; // rs1_sign: 第一个源操作数的符号信息（sign），用于确定是有符号数还是无符号数乘法。
   assign mul_p.rs2_sign =   (i0_dp.mul) ? i0_dp.rs2_sign :   i1_dp.rs2_sign;
   assign mul_p.low      =   (i0_dp.mul) ? i0_dp.low      :   i1_dp.low;  // low: 只需要低位信息

   assign mul_p.load_mul_rs1_bypass_e1 = load_mul_rs1_bypass_e1;   // tag_load_mul_rs1_bypass_e1: 2
   assign mul_p.load_mul_rs2_bypass_e1 = load_mul_rs2_bypass_e1;
   

	// tag_lsu_p: 1
   assign lsu_p.valid = lsu_decode_d; // tag_lsu_decode_d: 2

   assign lsu_p.load =   (i0_dp.lsu) ? i0_dp.load :   i1_dp.load;
   assign lsu_p.store =  (i0_dp.lsu) ? i0_dp.store :  i1_dp.store;
   assign lsu_p.by =     (i0_dp.lsu) ? i0_dp.by :     i1_dp.by;
   assign lsu_p.half =   (i0_dp.lsu) ? i0_dp.half :   i1_dp.half;
   assign lsu_p.word =   (i0_dp.lsu) ? i0_dp.word :   i1_dp.word;
   assign lsu_p.dword = '0;
   assign lsu_p.dma = '0;
   assign lsu_p.store_data_bypass_i0_e2_c2   = store_data_bypass_i0_e2_c2;  // has priority over all else
   assign lsu_p.load_ldst_bypass_c1 = load_ldst_bypass_c1;  // tag_load_ldst_bypass_c1: 2
   
   assign lsu_p.store_data_bypass_c1 = store_data_bypass_c1 & ~store_data_bypass_i0_e2_c2;
   assign lsu_p.store_data_bypass_c2 = store_data_bypass_c2 & ~store_data_bypass_i0_e2_c2;
   assign lsu_p.store_data_bypass_e4_c1[1:0] = store_data_bypass_e4_c1[1:0] & ~{2{store_data_bypass_i0_e2_c2}};   // tag_store_data_bypass_e4_c1: 2
   assign lsu_p.store_data_bypass_e4_c2[1:0] = store_data_bypass_e4_c2[1:0] & ~{2{store_data_bypass_i0_e2_c2}};
   assign lsu_p.store_data_bypass_e4_c3[1:0] = store_data_bypass_e4_c3[1:0] & ~{2{store_data_bypass_i0_e2_c2}};  

   assign lsu_p.unsign = (i0_dp.lsu) ? i0_dp.unsign : i1_dp.unsign;
   
// div_p, mul_p, lsu_p 结构体的赋值  end     三个 ouput 结构体，在本模块没有使用





   // defined register packet
   
// i0r, i1r 变量的赋值  begin  tag_i0r: 1

   assign i0r.rs1[4:0] = i0[19:15];  // tag_i0: 2
   assign i0r.rs2[4:0] = i0[24:20];
   assign i0r.rd[4:0] = i0[11:7];     // logic [31:0]        i0, i1;  

   assign i1r.rs1[4:0] = i1[19:15];
   assign i1r.rs2[4:0] = i1[24:20];
   assign i1r.rd[4:0] = i1[11:7];    // 5位表示的是寄存器编号

// i0r, i1r 变量的赋值  end





	// tag_sy_i0_i1_1: 对称  // dec_i0_rs1_en_d: rs1 enable at decode
   assign dec_i0_rs1_en_d = i0_dp.rs1 & (i0r.rs1[4:0] != 5'd0);  // if rs1_en=0 then read will be all 0's 
   assign dec_i0_rs2_en_d = i0_dp.rs2 & (i0r.rs2[4:0] != 5'd0);
   
   assign i0_rd_en_d =  i0_dp.rd & (i0r.rd[4:0] != 5'd0); // tag_i0_rd_en_d: 1

   
   assign dec_i0_rs1_d[4:0] = i0r.rs1[4:0];
   assign dec_i0_rs2_d[4:0] = i0r.rs2[4:0]; // 两个output，在本模块未使用
   
   assign i0_rd_d[4:0] = i0r.rd[4:0];  // 只赋值了，没有使用


   
   
   
   assign i0_jalimm20 = i0_dp.jal & i0_dp.imm20;   // jal
   assign i1_jalimm20 = i1_dp.jal & i1_dp.imm20;


   assign i0_uiimm20 = ~i0_dp.jal & i0_dp.imm20;
   assign i1_uiimm20 = ~i1_dp.jal & i1_dp.imm20;


   // csr logic

   assign dec_csr_ren_d = i0_dp.csr_read & i0_legal_decode_d;

   assign csr_clr_d =   i0_dp.csr_clr   & i0_legal_decode_d;
   assign csr_set_d   = i0_dp.csr_set   & i0_legal_decode_d;
   assign csr_write_d = i0_csr_write    & i0_legal_decode_d;

   assign i0_csr_write_only_d = i0_csr_write & ~i0_dp.csr_read;  // tag_i0_csr_write_only_d: 1

   assign dec_csr_wen_unq_d = (i0_dp.csr_clr | i0_dp.csr_set | i0_csr_write);   // for csr legal, can't write read-only csr ,  tag_dec_csr_wen_unq_d: 1

   assign dec_csr_any_unq_d = any_csr_d;


   assign dec_csr_rdaddr_d[11:0] = i0[31:20];
   assign dec_csr_wraddr_wb[11:0] = wbd.csrwaddr[11:0];


   // make sure csr doesn't write same cycle as flush_lower_wb
   // also use valid so it's flushable
   assign dec_csr_wen_wb = wbd.csrwen & wbd.i0valid & ~dec_tlu_i0_kill_writeb_wb;

   // If we are writing MIE or MSTATUS, hold off the external interrupt for a cycle on the write.
   assign dec_csr_stall_int_ff = ((e4d.csrwaddr[11:0] == 12'h300) | (e4d.csrwaddr[11:0] == 12'h304)) & e4d.csrwen & e4d.i0valid & ~dec_tlu_i0_kill_writeb_wb;


   rvdffs #(5) csrmiscff (.*,
                        .en(~freeze),
                        .clk(active_clk),
                        .din({ dec_csr_ren_d,  csr_clr_d,  csr_set_d,  csr_write_d,  i0_dp.csr_imm}),
                        .dout({csr_read_e1,    csr_clr_e1, csr_set_e1, csr_write_e1, csr_imm_e1})
                       );




   // perform the update operation if any

   rvdffe #(37) csr_data_e1ff (.*, .en(i0_e1_data_en), .din( {i0[19:15],dec_csr_rddata_d[31:0]}), .dout({csrimm_e1[4:0],csr_rddata_e1[31:0]}));


   assign csr_mask_e1[31:0] = ({32{ csr_imm_e1}} & {27'b0,csrimm_e1[4:0]}) |
                              ({32{~csr_imm_e1}} &  exu_csr_rs1_e1[31:0]);  // exu_csr_rs1_e1: 从 exu.sv 文件中传过来的


   assign write_csr_data_e1[31:0] = ({32{csr_clr_e1}}   & (csr_rddata_e1[31:0] & ~csr_mask_e1[31:0])) |
                                    ({32{csr_set_e1}}   & (csr_rddata_e1[31:0] |  csr_mask_e1[31:0])) |
                                    ({32{csr_write_e1}} & (                       csr_mask_e1[31:0]));


// pause instruction
//   logic pause_state_ff;

   assign clear_pause = (dec_tlu_flush_lower_wb & ~dec_tlu_flush_pause_wb) |
                        (pause_state & (write_csr_data[31:1] == 31'b0));        // if 0 or 1 then exit pause state - 1 cycle pause

   assign pause_state_in = (dec_tlu_wr_pause_wb | pause_state) & ~clear_pause;

   rvdff #(1) pause_state_f (.*, .clk(free_clk), .din(pause_state_in), .dout(pause_state));


   assign dec_pause_state = pause_state;

   rvdff #(2) pause_state_wb_ff (.*, .clk(free_clk), .din({dec_tlu_wr_pause_wb,tlu_wr_pause_wb1}), .dout({tlu_wr_pause_wb1,tlu_wr_pause_wb2}));


   assign dec_pause_state_cg = pause_state & ~tlu_wr_pause_wb1 & ~tlu_wr_pause_wb2;


// end pause

   assign csr_data_wen = ((csr_clr_e1 | csr_set_e1 | csr_write_e1) & csr_read_e1 & ~freeze) | dec_tlu_wr_pause_wb | pause_state;

   assign write_csr_data_in[31:0] = (pause_state)         ? (write_csr_data[31:0] - 32'b1) :
                                    (dec_tlu_wr_pause_wb) ? dec_csr_wrdata_wb[31:0] : write_csr_data_e1[31:0];

   // will hold until write-back at which time the CSR will be updated while GPR is possibly written with prior CSR
   rvdffe #(32) write_csr_ff (.*, .en(csr_data_wen), .din(write_csr_data_in[31:0]), .dout(write_csr_data[31:0]));

   assign pause_stall = pause_state;

   // for csr write only data is produced by the alu
   assign dec_csr_wrdata_wb[31:0] = (wbd.csrwonly) ? i0_result_wb[31:0] : write_csr_data[31:0];



// read the csr value through rs2 immed port
   assign dec_i0_immed_d[31:0] = ({32{ i0_dp.csr_read}} & dec_csr_rddata_d[31:0]) |
                                 ({32{~i0_dp.csr_read}} & i0_immed_d[31:0]);  // output, to exu.sv

// end csr stuff

   assign     i0_immed_d[31:0] = ({32{i0_dp.imm12}} &   { {20{i0[31]}},i0[31:20] }) |  // jalr
                                 ({32{i0_dp.shimm5}} &    {27'b0, i0[24:20]}) |
                                 ({32{i0_jalimm20}} &   { {12{i0[31]}},i0[19:12],i0[20],i0[30:21],1'b0}) |
                                 ({32{i0_uiimm20}}  &     {i0[31:12],12'b0 }) |
                                 ({32{i0_csr_write_only_d & i0_dp.csr_imm}} & {27'b0,i0[19:15]});  // for csr's that only write csr, dont read csr







   
   






   // tag_sy_i0_i1_1: 对称
   assign dec_i1_rs1_en_d = i1_dp.rs1 & (i1r.rs1[4:0] != 5'd0); // tag_dec_i1_rs1_en_d: 1
   assign dec_i1_rs2_en_d = i1_dp.rs2 & (i1r.rs2[4:0] != 5'd0);
   
   assign i1_rd_en_d =  i1_dp.rd & (i1r.rd[4:0] != 5'd0); // tag_i1_rd_en_d: 1

   assign dec_i1_rs1_d[4:0] = i1r.rs1[4:0];
   assign dec_i1_rs2_d[4:0] = i1r.rs2[4:0]; // 两个 output，在本模块未使用
   
   assign i1_rd_d[4:0] = i1r.rd[4:0];  // 只赋值了，没有使用
   
   
   
   


   assign dec_i1_immed_d[31:0] = ({32{i1_dp.imm12}} &   { {20{i1[31]}},i1[31:20] }) |
                                 ({32{i1_dp.shimm5}} &    {27'b0, i1[24:20]}) |
                                 ({32{i1_jalimm20}} &   { {12{i1[31]}},i1[19:12],i1[20],i1[30:21],1'b0}) |
                                 ({32{i1_uiimm20}}  &     {i1[31:12],12'b0 });   // output, to exu.sv






   
   


   assign last_br_immed_d[12:1] = (dec_i1_decode_d) ?
                                  ((i1_ap.predict_nt) ? {10'b0,i1_ap_pc4,i1_ap_pc2} : i1_br_offset[11:0] ) :
                                  ((i0_ap.predict_nt) ? {10'b0,i0_ap_pc4,i0_ap_pc2} : i0_br_offset[11:0] );  // tag_i0_ap: 2, tag_i1_ap: 2

								  
   assign i0_valid_d = dec_ib0_valid_d;
   assign i1_valid_d = dec_ib1_valid_d;



   
   
   assign i0_load_stall_d = i0_dp.load & (lsu_load_stall_any | dma_dccm_stall_any);  // tag_i0_load_stall_d: 1
   assign i1_load_stall_d = i1_dp.load & (lsu_load_stall_any | dma_dccm_stall_any);

   assign i0_store_stall_d =  i0_dp.store & (lsu_store_stall_any | dma_dccm_stall_any);  // tag_i0_store_stall_d: 1
   assign i1_store_stall_d =  i1_dp.store & (lsu_store_stall_any | dma_dccm_stall_any);

   assign i1_depend_i0_d = (dec_i1_rs1_en_d & i0_dp.rd & (i1r.rs1[4:0] == i0r.rd[4:0])) |
                           (dec_i1_rs2_en_d & i0_dp.rd & (i1r.rs2[4:0] == i0r.rd[4:0]));



   assign i1_load2_block_d = i1_dp.lsu & i0_dp.lsu;


// some CSR reads need to be presync'd
   assign i0_presync = i0_dp.presync | dec_tlu_presync_d | debug_fence_i | debug_fence_raw | dec_tlu_pipelining_disable;  // both fence's presync
// some CSR writes need to be postsync'd
   assign i0_postsync = i0_dp.postsync | dec_tlu_postsync_d | debug_fence_i | // only fence_i postsync
                        (i0_csr_write_only_d & (i0[31:20] == 12'h7c2));   // wr_pause must postsync

   assign i1_mul2_block_d  = i1_dp.mul & i0_dp.mul;


// debug fence csr

   assign debug_fence_i     = dec_debug_fence_d & dbg_cmd_wrdata[0];
   assign debug_fence_raw   = dec_debug_fence_d & dbg_cmd_wrdata[1];

   assign debug_fence = debug_fence_raw | debug_fence_i;    // fence_i causes a fence


   assign i0_csr_write = i0_dp.csr_write & ~dec_debug_fence_d;


// end debug


   // lets make ebreak, ecall, mret postsync, so break sync into pre and post

   assign presync_stall = (i0_presync & prior_inflight_eff);  // tag_presync_stall: 1

   assign prior_inflight_eff = (i0_dp.div) ? prior_inflight_e1e3 : prior_inflight; // tag_prior_inflight_e1e3: 2, tag_prior_inflight: 2

   // to TLU to set dma_stall
   assign dec_fence_pending = (i0_valid_d & i0_dp.fence) | debug_fence;


   localparam delay_cycle = 20;

   logic [delay_cycle - 1 : 0] i0_shift_reg;

   logic i0_block_result;
   
   assign i0_block_result = i0_load_block_d | (|i0_shift_reg);

   // assign i0_block_result =  (|i0_shift_reg);

   
   always_ff @( posedge clk or negedge rst_l ) begin : i0_block_test
      if(!rst_l) 
         i0_shift_reg <= '0;
    else if(i0_load_block_d)   // 有延迟, i0_load_block_d 为 1, i0_shift_reg 还是 0, i0_shift_reg 在下一
         i0_shift_reg <= {delay_cycle{1'b1}};
      else 
         i0_shift_reg <= {i0_shift_reg[delay_cycle - 2 : 0], 1'b0};
         
   end



   logic [delay_cycle - 1 : 0] i1_shift_reg;
   logic i1_block_result;

   

   assign i1_block_result = i1_load_block_d | (|i1_shift_reg);


   always_ff @( posedge clk or negedge rst_l ) begin : i1_block_test
      if(!rst_l) 
         i1_shift_reg <= '0;
    else if(i1_load_block_d)   
         i1_shift_reg <= {delay_cycle{1'b1}};
      else 
         i1_shift_reg <= {i1_shift_reg[delay_cycle - 2 : 0], 1'b0};
         
      
      
   end
   

   
// i0, i1 阻塞判断   begin   
   
   // 只要任意一个条件为真（即为 1），i0_block_d 就会被置为 1，表示阻塞。  
   // tag_i0_block_d: 1    tag_i0_i1_var_block: 2
   assign i0_block_d = (i0_dp.csr_read & prior_csr_write) |   // tag_prior_csr_write: 2  
                       pause_stall |
                       leak1_i0_stall |
                       dec_tlu_debug_stall |
                       postsync_stall |
                       presync_stall  |  // tag_presync_stall: 2
                       ((i0_dp.fence | debug_fence) & ~lsu_idle) |
                       i0_nonblock_load_stall |
                    
                      // i0_load_block_d |  

                       // i0_block_result |  

                       i0_mul_block_d |   

                       i0_store_stall_d |  // tag_i0_store_stall_d: 2
                       i0_load_stall_d |  // tag_i0_load_stall_d: 2
                       i0_secondary_stall_d |  // for performance, dont make i0 secondary if i1 not alu and depends on i0  为了性能优化，如果i1不是ALU指令，并且依赖于i0的结果，不要将i0标记为次要指令。
												// tag_i0_secondary_stall_d: 2
                       i0_secondary_block_d;  // ?：i0 阻塞 i0先发射   i0 i1发生阻塞，所有来的指令集 有一个有效的预测值 ，送到数据通路，把阻塞信息屏蔽   look_tag
					   
	// tag_i1_block_d: 1 
	// i1_block_d 只用于为 dec_i1_decode_d 判断
   assign i1_block_d = leak1_i1_stall |
                      (i0_jal) |            // no i1 after a jal, will flush
              (((|dec_i0_trigger_match_d[3:0]) | ((i0_dp.condbr | i0_dp.jal) & i0_secondary_d)) & i1_dp.load ) | // if branch or branch error then don't allow i1 load
                       i0_presync | i0_postsync |
                       i1_dp.presync | i1_dp.postsync |
                       i1_icaf_d |        // instruction access fault is i0 only ,   tag_i1_icaf_d: 2
                       dec_i1_perr_d |    // instruction parity error is i0 only
                       dec_i1_sbecc_d |
                       i0_dp.csr_read |
                       i0_dp.csr_write |
                       i1_dp.csr_read |
                       i1_dp.csr_write |  // optimized csr write with rd==0
					   
                       i1_nonblock_load_stall |
                       i1_store_stall_d |
                      
                     // i1_load_block_d |    // load data not ready

                     // i1_block_result |  

                      i1_mul_block_d |     // mul data not ready

                       (i1_depend_i0_d & ~non_block_case_d & ~store_data_bypass_i0_e2_c2) | // tag_store_data_bypass_i0_e2_c2: 2
                       i1_load2_block_d |  // back-to-back load's at decode
                       i1_mul2_block_d |
                       i1_load_stall_d |  // prior stores
                       i1_secondary_block_d | // secondary alu data not ready and op is not alu
                       dec_tlu_dual_issue_disable; // dual issue is disabled
					   // i1_load_block_d, i1_mul_block_d, i1_secondary_block_d 变量的作用在这

// i0, i1 阻塞判断   end   					   
					   
					   
	

	
					   
   // all legals go here

   // block reads if there is a prior csr write in the pipeline  如果流水线中存在之前的 CSR 写操作，则阻止读操作
   // tag_prior_csr_write: 1
   assign prior_csr_write = e1d.csrwonly | // e1d.csrwonly: e1 阶段的数据寄存器写入（write-only）标志。
                            e2d.csrwonly |
                            e3d.csrwonly |
                            e4d.csrwonly |
                            wbd.csrwonly; // wbd.csrwonly: 写回阶段的数据寄存器写入（write-only）标志。
	// prior_csr_write: 这是一个信号，用于指示在处理器流水线的不同阶段是否存在 CSR 写入操作。


   assign any_csr_d = i0_dp.csr_read | i0_csr_write;


   assign i0_legal = i0_dp.legal & (~any_csr_d | dec_csr_legal_d);

   // illegal inst handling


   assign shift_illegal = dec_i0_decode_d & ~i0_legal;

   assign illegal_inst_en = shift_illegal & ~illegal_lockout & ~freeze;

   assign illegal_inst[31:0] = (dec_i0_pc4_d) ? i0[31:0] : { 16'b0, ifu_illegal_inst[15:0] };

   rvdffe #(32) illegal_any_ff (.*, .en(illegal_inst_en), .din(illegal_inst[31:0]), .dout(dec_illegal_inst[31:0]));

   assign illegal_lockout_in = (shift_illegal | illegal_lockout) & ~flush_final_e3;

   rvdffs #(1) illegal_lockout_any_ff (.*, .clk(active_clk), .en(~freeze), .din(illegal_lockout_in), .dout(illegal_lockout));







   // allow illegals to flow down the pipe  

   // dec_i0_decode_d_block 为 1 的时候, 表示消耗了 1 条指令
   assign dec_i0_decode_d_block = (i0_rd_enable_next ) ? 0 : dec_i0_decode_d; 

   // assign dec_i0_decode_d_block =  dec_i0_decode_d; 
   
   // (~i0_load_block_d & i0_rd_enable_next): 从队列读的时候, 如果遇到跳转指令, 则进行阻塞
   // assign dec_i0_decode_d = i0_rd_enable_next ? 1 : i0_valid_d & ~i0_block_d & ~flush_lower_wb & ~flush_final_e3 & ~freeze; // tag_i0_block_d: 2  , i0_block_d 为 1 时，表示 i0 是阻塞的 look_tag
   // assign dec_i0_decode_d = i0_rd_enable_next ? (~i0_load_block_d ? 1 : 0) : i0_valid_d & ~i0_block_d & ~flush_lower_wb & ~flush_final_e3 & ~freeze;

   assign dec_i0_decode_d = i0_valid_d & ~i0_block_d & ~flush_lower_wb & ~flush_final_e3 & ~freeze;

   // define i0 legal decode
   assign i0_legal_decode_d = dec_i0_decode_d & i0_legal & ~freeze; 


   // only decode i1 if legal and i0 not illegal - csr's cant decode as i1
   
   assign dec_i1_decode_d_block = (i0_rd_enable_next ) ? 0 :  dec_i1_decode_d; 

   // assign dec_i1_decode_d_block =  dec_i1_decode_d; 

   assign dec_i1_decode_d = i0_legal_decode_d & i1_valid_d & i1_dp.legal & ~i1_block_d  & ~freeze; // tag_i1_block_d: 2

   // assign dec_i1_decode_d = i0_rd_enable_next ? (~i0_load_block_d ? 1 : (~i1_load_block_d ? 1 : 0) ) : i0_legal_decode_d & i1_valid_d & i1_dp.legal & ~i1_block_d  & ~freeze;

   

   
   // assign dec_ib0_valid_eff_d = i0_valid_d & ~dec_i0_decode_d_block; 
   // assign dec_ib1_valid_eff_d = i1_valid_d & ~dec_i1_decode_d_block;  

   // i0 指令有效, 但并 没有进行解码
   assign dec_ib0_valid_eff_d = (i0_rd_enable_next ) ? 1 : i0_valid_d & ~dec_i0_decode_d; // dec_ib0_valid_eff_d: effective valid taking decode into account
   assign dec_ib1_valid_eff_d = (i0_rd_enable_next ) ? 1 : i1_valid_d & ~dec_i1_decode_d;  // 两个 output, to dec_ib_ctl.sv

   



   
   
   // performance monitor signals
   assign dec_pmu_instr_decoded[1:0] = { dec_i1_decode_d, dec_i0_decode_d };

   assign dec_pmu_decode_stall = i0_valid_d & ~dec_i0_decode_d;

   assign dec_pmu_postsync_stall = postsync_stall;
   assign dec_pmu_presync_stall  = presync_stall; // tag_presync_stall: 2,      四个 ouput，在本模块没有使用



   // illegals will postsync
   // jal's will flush, so postsync    
   assign ps_stall_in =  (dec_i0_decode_d & (i0_jal | (i0_postsync) | ~i0_legal))  |
                         (dec_i1_decode_d &  i1_jal ) |
                         ((ps_stall & prior_inflight_e1e4) & ~div_wen_wb);


    rvdffs #(1) postsync_stallff (.*, .clk(free_clk), .en(~freeze), .din(ps_stall_in), .dout(ps_stall));

   assign postsync_stall = (ps_stall | div_stall);


// tag_answer   之前 e1 - e3 阶段是否有指令在执行
   assign prior_inflight_e1e3 =       |{ e1d.i0valid,
                                         e2d.i0valid,
                                         e3d.i0valid,
										 
                                         e1d.i1valid,
                                         e2d.i1valid,
                                         e3d.i1valid
                                         }; // tag_prior_inflight_e1e3: 1



   assign prior_inflight_e1e4 =       |{ e1d.i0valid,
                                         e2d.i0valid,
                                         e3d.i0valid,
                                         e4d.i0valid,
										 
                                         e1d.i1valid,
                                         e2d.i1valid,
                                         e3d.i1valid,
                                         e4d.i1valid
                                         };


   assign prior_inflight_wb =            |{
                                           wbd.i0valid,
                                           wbd.i1valid
                                           };

   assign prior_inflight = prior_inflight_e1e4 | prior_inflight_wb;  // tag_prior_inflight: 1

   
   
   
   
   assign dec_i0_alu_decode_d = i0_legal_decode_d & i0_dp.alu & ~i0_secondary_d;  // dec_i0_alu_decode_d: decode to primary alu's
   assign dec_i1_alu_decode_d = dec_i1_decode_d & i1_dp.alu & ~i1_secondary_d; // 两个 output, to exu.sv,   tag_i1_secondary_d: 2

   assign dec_i0_lsu_decode_d = i0_legal_decode_d & i0_dp.lsu; // output，在本模块未使用
   

   
   // 只用于 lsu_p, mul_p, div_p 中
   assign lsu_decode_d = (i0_legal_decode_d & i0_dp.lsu) |
                         (dec_i1_decode_d & i1_dp.lsu); // tag_lsu_decode_d: 1, tag_dec_i1_decode_d: 2

   assign mul_decode_d = (i0_legal_decode_d & i0_dp.mul) |
                         (dec_i1_decode_d & i1_dp.mul); // tag_mul_decode_d: 1

   assign div_decode_d = (i0_legal_decode_d & i0_dp.div) |
                         (dec_i1_decode_d & i1_dp.div); //  tag_div_decode_d: 1




   rvdffs #(2) flushff (.*, .en(~freeze), .clk(free_clk), .din({exu_i0_flush_final,exu_i1_flush_final}), .dout({i0_flush_final_e3, i1_flush_final_e3}));


   assign flush_final_e3 = i0_flush_final_e3 | i1_flush_final_e3 ; // flush_final_e3: flush final at e3: i0  or i1





// scheduling logic for primary and secondary alu's 为 主算术逻辑单元和次算术逻辑单元 设计调度逻辑。   

// 依赖检查 begin   


   // i0 的 rs1、rs2 依赖检测
   // i0_rs1_depend_i0_wb： i0流水线的 当前指令的第一个源操作数（rs1） 是否依赖于 i0流水线的 在wb阶段的结果。
   // i0_rs1_depend_i0_e1 是一个布尔信号, 即结果是 0 或 1
   // tag_i0_rs1_depend_i0_e1: 1
   assign i0_rs1_depend_i0_e1 = dec_i0_rs1_en_d & e1d.i0v & (e1d.i0rd[4:0] == i0r.rs1[4:0]);  // tag_dec_i0_rs1_en_d: 2, tag_e_d: 2, tag_i0r: 2
   assign i0_rs1_depend_i0_e2 = dec_i0_rs1_en_d & e2d.i0v & (e2d.i0rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i0_e3 = dec_i0_rs1_en_d & e3d.i0v & (e3d.i0rd[4:0] == i0r.rs1[4:0]); 
   assign i0_rs1_depend_i0_e4 = dec_i0_rs1_en_d & e4d.i0v & (e4d.i0rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i0_wb = dec_i0_rs1_en_d & wbd.i0v & (wbd.i0rd[4:0] == i0r.rs1[4:0]); 
   
   // e1d: e1 dest_pkt_t    wbd: writeback dest_pkt_t
  

   /*
		dec_i0_rs1_en_d：表示指令 i0 的源寄存器 rs1 是否有效或使能的信号。
		e1d.i0v：表示执行阶段的指令 e1 是否有效。
		(e1d.i0rd[4:0] == i0r.rs1[4:0])：比较执行指令 e1 中目标寄存器 i0rd 和指令 i0 的源寄存器 rs1 是否相同，确保它们引用的是同一个寄存器。	
   */
   
   // 目的是为了检测数据冒险，确保在执行指令时能够及时处理可能的依赖，以提高指令的正确性和效率
   assign i0_rs1_depend_i1_e1 = dec_i0_rs1_en_d & e1d.i1v & (e1d.i1rd[4:0] == i0r.rs1[4:0]);   
   assign i0_rs1_depend_i1_e2 = dec_i0_rs1_en_d & e2d.i1v & (e2d.i1rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i1_e3 = dec_i0_rs1_en_d & e3d.i1v & (e3d.i1rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i1_e4 = dec_i0_rs1_en_d & e4d.i1v & (e4d.i1rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i1_wb = dec_i0_rs1_en_d & wbd.i1v & (wbd.i1rd[4:0] == i0r.rs1[4:0]);

   
   
   
   
   assign i0_rs2_depend_i0_e1 = dec_i0_rs2_en_d & e1d.i0v & (e1d.i0rd[4:0] == i0r.rs2[4:0]);  // tag_dec_i0_rs1_en_d: 2
   assign i0_rs2_depend_i0_e2 = dec_i0_rs2_en_d & e2d.i0v & (e2d.i0rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i0_e3 = dec_i0_rs2_en_d & e3d.i0v & (e3d.i0rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i0_e4 = dec_i0_rs2_en_d & e4d.i0v & (e4d.i0rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i0_wb = dec_i0_rs2_en_d & wbd.i0v & (wbd.i0rd[4:0] == i0r.rs2[4:0]);

   assign i0_rs2_depend_i1_e1 = dec_i0_rs2_en_d & e1d.i1v & (e1d.i1rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i1_e2 = dec_i0_rs2_en_d & e2d.i1v & (e2d.i1rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i1_e3 = dec_i0_rs2_en_d & e3d.i1v & (e3d.i1rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i1_e4 = dec_i0_rs2_en_d & e4d.i1v & (e4d.i1rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i1_wb = dec_i0_rs2_en_d & wbd.i1v & (wbd.i1rd[4:0] == i0r.rs2[4:0]);

   
   // i1 的 rs1、rs2 依赖检测
   assign i1_rs1_depend_i0_e1 = dec_i1_rs1_en_d & e1d.i0v & (e1d.i0rd[4:0] == i1r.rs1[4:0]); // tag_dec_i1_rs1_en_d: 2
   assign i1_rs1_depend_i0_e2 = dec_i1_rs1_en_d & e2d.i0v & (e2d.i0rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i0_e3 = dec_i1_rs1_en_d & e3d.i0v & (e3d.i0rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i0_e4 = dec_i1_rs1_en_d & e4d.i0v & (e4d.i0rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i0_wb = dec_i1_rs1_en_d & wbd.i0v & (wbd.i0rd[4:0] == i1r.rs1[4:0]);

   assign i1_rs1_depend_i1_e1 = dec_i1_rs1_en_d & e1d.i1v & (e1d.i1rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i1_e2 = dec_i1_rs1_en_d & e2d.i1v & (e2d.i1rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i1_e3 = dec_i1_rs1_en_d & e3d.i1v & (e3d.i1rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i1_e4 = dec_i1_rs1_en_d & e4d.i1v & (e4d.i1rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i1_wb = dec_i1_rs1_en_d & wbd.i1v & (wbd.i1rd[4:0] == i1r.rs1[4:0]);

   
   assign i1_rs2_depend_i0_e1 = dec_i1_rs2_en_d & e1d.i0v & (e1d.i0rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i0_e2 = dec_i1_rs2_en_d & e2d.i0v & (e2d.i0rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i0_e3 = dec_i1_rs2_en_d & e3d.i0v & (e3d.i0rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i0_e4 = dec_i1_rs2_en_d & e4d.i0v & (e4d.i0rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i0_wb = dec_i1_rs2_en_d & wbd.i0v & (wbd.i0rd[4:0] == i1r.rs2[4:0]);

   assign i1_rs2_depend_i1_e1 = dec_i1_rs2_en_d & e1d.i1v & (e1d.i1rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i1_e2 = dec_i1_rs2_en_d & e2d.i1v & (e2d.i1rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i1_e3 = dec_i1_rs2_en_d & e3d.i1v & (e3d.i1rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i1_e4 = dec_i1_rs2_en_d & e4d.i1v & (e4d.i1rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i1_wb = dec_i1_rs2_en_d & wbd.i1v & (wbd.i1rd[4:0] == i1r.rs2[4:0]);
   
// 依赖检查 end




   rvdff #(2) freezeff (.*, .clk(active_clk), .din({freeze,freeze_prior1}), .dout({freeze_prior1,freeze_prior2}));

   // take snapshot of e4 and wb
   assign freeze_after_unfreeze1 = freeze & ~freeze_prior1;
   assign freeze_after_unfreeze2 = freeze & ~freeze_prior1 & ~freeze_prior2;

   assign unfreeze_cycle1 = ~freeze & freeze_prior1;
   assign unfreeze_cycle2 = ~freeze & ~freeze_prior1 & freeze_prior2;

   rvdffe #(32) freeze_i0_e4ff (.*, .en(freeze_after_unfreeze1), .din(i0_result_e4_final[31:0]), .dout(i0_result_e4_freeze[31:0]));
   rvdffe #(32) freeze_i1_e4ff (.*, .en(freeze_after_unfreeze1), .din(i1_result_e4_final[31:0]), .dout(i1_result_e4_freeze[31:0]));


   rvdffe #(32) freeze_i0_wbff (.*,
                              .en(freeze_after_unfreeze1),
                              .din( (freeze_after_unfreeze2) ? i0_result_wb[31:0] : i0_result_e4_freeze[31:0]),
                              .dout(i0_result_wb_freeze[31:0])
                              );

   rvdffe #(32) freeze_i1_wbff (.*,
                              .en(freeze_after_unfreeze1),
                              .din( (freeze_after_unfreeze2) ? i1_result_wb[31:0] : i1_result_e4_freeze[31:0]),
                              .dout(i1_result_wb_freeze[31:0])
                              );


/*
		1 is youngest 是指在旁路选择逻辑中，1 表示最“年轻”的数据，即在不同来源的数据中，优先选择最新生成的那个。
		例如，如果同一个数据可能来自多个不同的流水线阶段或多个执行单元，那么应优先选择最近生成的结果，因为它最接近当前时钟周期的执行需求。
*/							  

// define bypasses for e2 stage - 1 is youngest 定义了 e2 阶段的旁路逻辑   

	// dd.i0rs1bype2: i0 rs1 bypass e2 的可能结果：00 01 10 11
	
	// tag_dd: 1    每一行表达式 为 每一位赋值
   assign dd.i0rs1bype2[1:0] = {  i0_dp.alu & i0_rs1_depth_d[3:0] == 4'd5 & i0_rs1_class_d.sec, 
                                  i0_dp.alu & i0_rs1_depth_d[3:0] == 4'd6 & i0_rs1_class_d.sec };  // 处于流水线 e3 阶段, 并且 rs1 指令类型是 sec
																								   																							   
   // dd.i0rs2bype2: i0 rs2 bypass e2
   assign dd.i0rs2bype2[1:0] = {  i0_dp.alu & i0_rs2_depth_d[3:0] == 4'd5 & i0_rs2_class_d.sec,
                                  i0_dp.alu & i0_rs2_depth_d[3:0] == 4'd6 & i0_rs2_class_d.sec };

	// 	dd.i1rs1bype2: 	i1 rs1 bypass e2				  
   assign dd.i1rs1bype2[1:0] = {  i1_dp.alu & i1_rs1_depth_d[3:0] == 4'd5 & i1_rs1_class_d.sec,
                                  i1_dp.alu & i1_rs1_depth_d[3:0] == 4'd6 & i1_rs1_class_d.sec };

	// 	dd.i1rs2bype2: 	i1 rs2 bypass e2					  
   assign dd.i1rs2bype2[1:0] = {  i1_dp.alu & i1_rs2_depth_d[3:0] == 4'd5 & i1_rs2_class_d.sec,
                                  i1_dp.alu & i1_rs2_depth_d[3:0] == 4'd6 & i1_rs2_class_d.sec };

								  
		
		
// 对i0_rs1_bypass_data_e2 等变量的赋值   begin	
		
   assign i1_result_wb_eff[31:0] = (unfreeze_cycle1) ? i1_result_wb_freeze[31:0] :
                                   (unfreeze_cycle2) ? i1_result_e4_freeze[31:0] :
                                   i1_result_wb[31:0]; // tag_i1_result_wb_eff: 1

   assign i0_result_wb_eff[31:0] = (unfreeze_cycle1) ? i0_result_wb_freeze[31:0] :
                                   (unfreeze_cycle2) ? i0_result_e4_freeze[31:0] :
                                   i0_result_wb[31:0];

								  								
								   
													   
   assign i0_rs1_bypass_data_e2[31:0] = ({32{e2d.i0rs1bype2[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e2d.i0rs1bype2[0]}} & i0_result_wb_eff[31:0]); 

   assign i0_rs2_bypass_data_e2[31:0] = ({32{e2d.i0rs2bype2[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e2d.i0rs2bype2[0]}} & i0_result_wb_eff[31:0]);

   assign i1_rs1_bypass_data_e2[31:0] = ({32{e2d.i1rs1bype2[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e2d.i1rs1bype2[0]}} & i0_result_wb_eff[31:0]);

   assign i1_rs2_bypass_data_e2[31:0] = ({32{e2d.i1rs2bype2[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e2d.i1rs2bype2[0]}} & i0_result_wb_eff[31:0]); // 四个变量都是 output，输出到 exu.sv 文件中

										
   assign dec_i0_rs1_bypass_en_e2 = |e2d.i0rs1bype2[1:0]; // // dec_i0_rs1_bypass_en_e2: i0 rs1 bypass enable e2  使能
   assign dec_i0_rs2_bypass_en_e2 = |e2d.i0rs2bype2[1:0];
   assign dec_i1_rs1_bypass_en_e2 = |e2d.i1rs1bype2[1:0];
   assign dec_i1_rs2_bypass_en_e2 = |e2d.i1rs2bype2[1:0]; // 四个变量都是 output，to exu.sv 文件中

// 对i0_rs1_bypass_data_e2 等变量的赋值   end


   
   
   
   
// i0  i1  begin  
   
// define bypasses for e3 stage before secondary alu's


   assign i1_rs1_depend_i0_d = dec_i1_rs1_en_d & i0_dp.rd & (i1r.rs1[4:0] == i0r.rd[4:0]); // tag_dec_i1_rs1_en_d: 2, i1_rs1_depend_i0_d: 指令 i1 的源寄存器 rs1 是否依赖于指令 i0 的结果
   assign i1_rs2_depend_i0_d = dec_i1_rs2_en_d & i0_dp.rd & (i1r.rs2[4:0] == i0r.rd[4:0]);


// i0       每一行表达式 为 每一位赋值
	// i0rs1bype3[3:0] 四位中 只有一位为 1
   assign dd.i0rs1bype3[3:0] = { i0_dp.alu & i0_rs1_depth_d[3:0]==4'd1 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul), 
                                 i0_dp.alu & i0_rs1_depth_d[3:0]==4'd2 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul),  // e1
                                 i0_dp.alu & i0_rs1_depth_d[3:0]==4'd3 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                 i0_dp.alu & i0_rs1_depth_d[3:0]==4'd4 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul) };  // e2
																								
																								
																								

   assign dd.i0rs2bype3[3:0] = { i0_dp.alu & i0_rs2_depth_d[3:0]==4'd1 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                 i0_dp.alu & i0_rs2_depth_d[3:0]==4'd2 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                 i0_dp.alu & i0_rs2_depth_d[3:0]==4'd3 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                 i0_dp.alu & i0_rs2_depth_d[3:0]==4'd4 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul) };

								 
								 
// i1
    // 应该也是 三位中 只有一位为 1
   assign i1rs1_intra[2:0] = {   i1_dp.alu & i0_dp.alu  & i1_rs1_depend_i0_d,
                                 i1_dp.alu & i0_dp.mul  & i1_rs1_depend_i0_d,
                                 i1_dp.alu & i0_dp.load & i1_rs1_depend_i0_d
                                 };

   assign i1rs2_intra[2:0] = {   i1_dp.alu & i0_dp.alu  & i1_rs2_depend_i0_d,
                                 i1_dp.alu & i0_dp.mul  & i1_rs2_depend_i0_d,
                                 i1_dp.alu & i0_dp.load & i1_rs2_depend_i0_d
                                 };

   assign i1_rs1_intra_bypass = |i1rs1_intra[2:0]; // i1rs1_intra 只要有一位为 1, 则 i1_rs1_intra_bypass 的结果为 1

   assign i1_rs2_intra_bypass = |i1rs2_intra[2:0];

	// tag_dd: 1  tag_i1_rs1_depth_d
   assign dd.i1rs1bype3[6:0] = { i1rs1_intra[2:0],  // i1rs1_intra 提供高三位 [6 5 4]   // sec 在 e4阶段出结果
   
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd1 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass,
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd2 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass,
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd3 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass,
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd4 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass };
																								
																								
																								
																								
																								

   assign dd.i1rs2bype3[6:0] = { i1rs2_intra[2:0],
   
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd1 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass,
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd2 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass,
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd3 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass,
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd4 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass };


// i0  i1  end

				

							
							   
// 对i0_rs1_bypass_data_e3 等变量的赋值   begin	


	
	
   assign dec_i0_rs1_bypass_en_e3 = |e3d.i0rs1bype3[3:0];
   assign dec_i0_rs2_bypass_en_e3 = |e3d.i0rs2bype3[3:0];
   assign dec_i1_rs1_bypass_en_e3 = |e3d.i1rs1bype3[6:0];
   assign dec_i1_rs2_bypass_en_e3 = |e3d.i1rs2bype3[6:0];  // 四个变量都是 output，在本模块没有使用
	
	
   assign i1_result_e4_eff[31:0] = (unfreeze_cycle1) ? i1_result_e4_freeze[31:0] :
                                   i1_result_e4_final[31:0];
	// i0_result_e4_eff: i0 e4 result taking freeze into account
   assign i0_result_e4_eff[31:0] = (unfreeze_cycle1) ? i0_result_e4_freeze[31:0] :
                                   i0_result_e4_final[31:0];
								   
					   
	// i0_rs1_bypass_data_e3: i0 rs1 bypass data e3
   assign i0_rs1_bypass_data_e3[31:0] = ({32{e3d.i0rs1bype3[3]}} & i1_result_e4_eff[31:0]) | // tag_e_d: 2, 
                                        ({32{e3d.i0rs1bype3[2]}} & i0_result_e4_eff[31:0]) |
                                        ({32{e3d.i0rs1bype3[1]}} & i1_result_wb_eff[31:0]) | // tag_i1_result_wb_eff: 2
                                        ({32{e3d.i0rs1bype3[0]}} & i0_result_wb_eff[31:0]);    

   assign i0_rs2_bypass_data_e3[31:0] = ({32{e3d.i0rs2bype3[3]}} & i1_result_e4_eff[31:0]) |
                                        ({32{e3d.i0rs2bype3[2]}} & i0_result_e4_eff[31:0]) |
                                        ({32{e3d.i0rs2bype3[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e3d.i0rs2bype3[0]}} & i0_result_wb_eff[31:0]);

										
										
   assign i1_rs1_bypass_data_e3[31:0] = ({32{e3d.i1rs1bype3[6]}} & i0_result_e3[31:0]) |				//  alu
                                        ({32{e3d.i1rs1bype3[5]}} & exu_mul_result_e3[31:0]) |           //  mul
                                       ({32{e3d.i1rs1bype3[4]}} & lsu_result_dc3[31:0]) |              //  load   i1 依赖于 decode 阶段 i0
                                        // ({32{e3d.i1rs1bype3[4]}} & lsu_result_dc3_d4[31:0]) |
										
                                        ({32{e3d.i1rs1bype3[3]}} & i1_result_e4_eff[31:0]) |
                                        ({32{e3d.i1rs1bype3[2]}} & i0_result_e4_eff[31:0]) |
                                        ({32{e3d.i1rs1bype3[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e3d.i1rs1bype3[0]}} & i0_result_wb_eff[31:0]);


   assign i1_rs2_bypass_data_e3[31:0] = ({32{e3d.i1rs2bype3[6]}} & i0_result_e3[31:0]) |				//  alu
                                        ({32{e3d.i1rs2bype3[5]}} & exu_mul_result_e3[31:0]) |           //  mul
                                        ({32{e3d.i1rs2bype3[4]}} & lsu_result_dc3[31:0]) |              //  load   i1 依赖于 decode 阶段 i0    
                                        // ({32{e3d.i1rs2bype3[4]}} & lsu_result_dc3_d4[31:0]) |  
										
                                        ({32{e3d.i1rs2bype3[3]}} & i1_result_e4_eff[31:0]) |
                                        ({32{e3d.i1rs2bype3[2]}} & i0_result_e4_eff[31:0]) |
                                        ({32{e3d.i1rs2bype3[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e3d.i1rs2bype3[0]}} & i0_result_wb_eff[31:0]);  // 四个值都是 output，to exu.sv 文件中


// 对i0_rs1_bypass_data_e3 等变量的赋值   end

	


	
// i0 i1 第一、二个源操作数 类型 和 所在流水线阶段     begin

// order the producers as follows:  i1_e1 - 1, i0_e1 - 2, i1_e2 - 3, ..., i1_wb - 9, i0_wb - 10

    
	// i0_rs1_class_d: i0 的第一个源操作数类型, i0_rs1_depth_d: i0 第一个操作数所处在流水线的那个阶段  
	// i0_rs1_depend_i1_e1: i0 的 rs1 依赖于 i1 的 e1 阶段
	
	
	//  tag_i0_rs1_class_d: 1,   tag_i0_rs1_depth_d: 1
   assign {i0_rs1_class_d, i0_rs1_depth_d[3:0]} =
                                                  (i0_rs1_depend_i1_e1) ? { i1_e1c, 4'd1 } : 
                                                  (i0_rs1_depend_i0_e1) ? { i0_e1c, 4'd2 } : // tag_i0_rs1_depend_i0_e1: 2
												  
                                                  (i0_rs1_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i0_rs1_depend_i0_e2) ? { i0_e2c, 4'd4 } :
												  
                                                  (i0_rs1_depend_i1_e3) ? { i1_e3c, 4'd5 } : 
                                                  (i0_rs1_depend_i0_e3) ? { i0_e3c, 4'd6 } :
												  
                                                  (i0_rs1_depend_i1_e4) ? { i1_e4c, 4'd7 } : // i1_e4c: i1 e4 class_pkt_t
                                                  (i0_rs1_depend_i0_e4) ? { i0_e4c, 4'd8 } :
												  
                                                  (i0_rs1_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i0_rs1_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0; // 依赖检测结果的使用，只有这一个地方用了 look_tag
																								 // '0：按需求补零 的方式 ？ tag_question 
																								 
			// i0_e1c, i0_e2c, i0_e3c, i0_e4c, i0_wbc, i1_dc, i1_e1c, i1_e2c, i1_e3c, i1_e4c, i1_wbc 变量的使用  tag_i0_class_pkt_t: 2
																							

   assign {i0_rs2_class_d, i0_rs2_depth_d[3:0]} =
                                                  (i0_rs2_depend_i1_e1) ? { i1_e1c, 4'd1 } :
                                                  (i0_rs2_depend_i0_e1) ? { i0_e1c, 4'd2 } :
												  
                                                  (i0_rs2_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i0_rs2_depend_i0_e2) ? { i0_e2c, 4'd4 } :
												  
                                                  (i0_rs2_depend_i1_e3) ? { i1_e3c, 4'd5 } :
                                                  (i0_rs2_depend_i0_e3) ? { i0_e3c, 4'd6 } :
												  
                                                  (i0_rs2_depend_i1_e4) ? { i1_e4c, 4'd7 } :
                                                  (i0_rs2_depend_i0_e4) ? { i0_e4c, 4'd8 } :
												  
                                                  (i0_rs2_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i0_rs2_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0;
												  
												  
												  

   assign {i1_rs1_class_d, i1_rs1_depth_d[3:0]} =
                                                  (i1_rs1_depend_i1_e1) ? { i1_e1c, 4'd1 } :
                                                  (i1_rs1_depend_i0_e1) ? { i0_e1c, 4'd2 } :  // e1
												  
                                                  (i1_rs1_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i1_rs1_depend_i0_e2) ? { i0_e2c, 4'd4 } :  // e2
												  
                                                  (i1_rs1_depend_i1_e3) ? { i1_e3c, 4'd5 } :
                                                  (i1_rs1_depend_i0_e3) ? { i0_e3c, 4'd6 } :  // e3
												  
                                                  (i1_rs1_depend_i1_e4) ? { i1_e4c, 4'd7 } :
                                                  (i1_rs1_depend_i0_e4) ? { i0_e4c, 4'd8 } :  // e4
												  
                                                  (i1_rs1_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i1_rs1_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0;  // wb

												  
												  
												  
   assign {i1_rs2_class_d, i1_rs2_depth_d[3:0]} =
                                                  (i1_rs2_depend_i1_e1) ? { i1_e1c, 4'd1 } :
                                                  (i1_rs2_depend_i0_e1) ? { i0_e1c, 4'd2 } :
                                                  (i1_rs2_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i1_rs2_depend_i0_e2) ? { i0_e2c, 4'd4 } :
                                                  (i1_rs2_depend_i1_e3) ? { i1_e3c, 4'd5 } :
                                                  (i1_rs2_depend_i0_e3) ? { i0_e3c, 4'd6 } :
                                                  (i1_rs2_depend_i1_e4) ? { i1_e4c, 4'd7 } :
                                                  (i1_rs2_depend_i0_e4) ? { i0_e4c, 4'd8 } :
                                                  (i1_rs2_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i1_rs2_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0;

// i0 i1 第一、二个源操作数 类型 和 所在流水线阶段     end


									  
	


	
// 所得结果用于  i0, i1 block变量 的输入   begin    tag_i0_i1_match: 1
												  
   assign i0_rs1_match_e1 = (i0_rs1_depth_d[3:0] == 4'd1 |  //   tag_i0_rs1_depth_d: 2,    tag_answer 依赖的值 在译码的那个阶段
                             i0_rs1_depth_d[3:0] == 4'd2); //  e1 阶段匹配

   assign i0_rs1_match_e2 = (i0_rs1_depth_d[3:0] == 4'd3 |
                             i0_rs1_depth_d[3:0] == 4'd4); //  e2 阶段匹配

   assign i0_rs1_match_e3 = (i0_rs1_depth_d[3:0] == 4'd5 |
                             i0_rs1_depth_d[3:0] == 4'd6); //  e3 阶段匹配

   assign i0_rs2_match_e1 = (i0_rs2_depth_d[3:0] == 4'd1 |
                             i0_rs2_depth_d[3:0] == 4'd2); 

   assign i0_rs2_match_e2 = (i0_rs2_depth_d[3:0] == 4'd3 |
                             i0_rs2_depth_d[3:0] == 4'd4);

   assign i0_rs2_match_e3 = (i0_rs2_depth_d[3:0] == 4'd5 |
                             i0_rs2_depth_d[3:0] == 4'd6);

							 
   assign i0_rs1_match_e1_e2 = i0_rs1_match_e1 | i0_rs1_match_e2; // e1 到 e2 阶段
   assign i0_rs1_match_e1_e3 = i0_rs1_match_e1 | i0_rs1_match_e2 | i0_rs1_match_e3; // e1 到 e3 阶段

   assign i0_rs2_match_e1_e2 = i0_rs2_match_e1 | i0_rs2_match_e2;
   assign i0_rs2_match_e1_e3 = i0_rs2_match_e1 | i0_rs2_match_e2 | i0_rs2_match_e3;


	// tag_i0_secondary_d: 1
   assign i0_secondary_d = ((i0_dp.alu & (i0_rs1_class_d.load | i0_rs1_class_d.mul) & i0_rs1_match_e1_e2) |  // tag_i0_dp: 2
                            (i0_dp.alu & (i0_rs2_class_d.load | i0_rs2_class_d.mul) & i0_rs2_match_e1_e2) |
                            (i0_dp.alu & i0_rs1_class_d.sec & i0_rs1_match_e1_e3) |
                            (i0_dp.alu & i0_rs2_class_d.sec & i0_rs2_match_e1_e3)) & ~disable_secondary;  // tag_disable_secondary: 2

							
  // stall i0 until it's not a secondary for performance     对指令 i0 进行暂停（stall），直到它不再作为“次级”（secondary）指令以提升性能。
  // tag_i0_secondary_stall_d: 1
   assign i0_secondary_stall_d = ((i0_dp.alu & i1_rs1_depend_i0_d & ~i1_dp.alu & i0_secondary_d) |    // 如果 i0 是一个 ALU 操作指令，而 i1 依赖于 i0 的结果（通过 rs1），并且 i1 不是一个 ALU 操作指令，同时 i0 是次级指令，那么这个条件为真。
                                  (i0_dp.alu & i1_rs2_depend_i0_d & ~i1_dp.alu & ~i1_dp.store & i0_secondary_d)) & ~disable_secondary;  // tag_disable_secondary: 2



								  
								  
   assign i1_rs1_match_e1 = (i1_rs1_depth_d[3:0] == 4'd1 |
                             i1_rs1_depth_d[3:0] == 4'd2);

   assign i1_rs1_match_e2 = (i1_rs1_depth_d[3:0] == 4'd3 |
                             i1_rs1_depth_d[3:0] == 4'd4);

   assign i1_rs1_match_e3 = (i1_rs1_depth_d[3:0] == 4'd5 |
                             i1_rs1_depth_d[3:0] == 4'd6);

   assign i1_rs2_match_e1 = (i1_rs2_depth_d[3:0] == 4'd1 |
                             i1_rs2_depth_d[3:0] == 4'd2);

   assign i1_rs2_match_e2 = (i1_rs2_depth_d[3:0] == 4'd3 |
                             i1_rs2_depth_d[3:0] == 4'd4);

   assign i1_rs2_match_e3 = (i1_rs2_depth_d[3:0] == 4'd5 |
                             i1_rs2_depth_d[3:0] == 4'd6);

							 
   assign i1_rs1_match_e1_e2 = i1_rs1_match_e1 | i1_rs1_match_e2;
   assign i1_rs1_match_e1_e3 = i1_rs1_match_e1 | i1_rs1_match_e2 | i1_rs1_match_e3;

   assign i1_rs2_match_e1_e2 = i1_rs2_match_e1 | i1_rs2_match_e2;
   assign i1_rs2_match_e1_e3 = i1_rs2_match_e1 | i1_rs2_match_e2 | i1_rs2_match_e3;



	// tag_i1_secondary_d: 1
   assign i1_secondary_d = ((i1_dp.alu & (i1_rs1_class_d.load | i1_rs1_class_d.mul) & i1_rs1_match_e1_e2) |
                            (i1_dp.alu & (i1_rs2_class_d.load | i1_rs2_class_d.mul) & i1_rs2_match_e1_e2) |
                            (i1_dp.alu & (i1_rs1_class_d.sec) & i1_rs1_match_e1_e3) |
                            (i1_dp.alu & (i1_rs2_class_d.sec) & i1_rs2_match_e1_e3) |
                            (non_block_case_d & i1_depend_i0_d)) & ~disable_secondary;  // tag_disable_secondary: 2

							
// 所得结果用于  i0, i1 block变量 的输入   end

   logic i0_load_all_d, i1_load_all_d;

   assign i0_load_all_d = i0_dp.load ;
   assign i1_load_all_d = i1_dp.load ;

   // assign i0_load_all_d = i0_dp.load & ~disable_secondary;
   // assign i1_load_all_d = i1_dp.load & ~disable_secondary;  // 两者都可以





	// tag_store_data_bypass_i0_e2_c2: 1
   assign store_data_bypass_i0_e2_c2 = i0_dp.alu & ~i0_secondary_d & i1_rs2_depend_i0_d & ~i1_rs1_depend_i0_d & i1_dp.store;

   assign non_block_case_d = (  // (i1_dp.alu & i0_dp.alu & ~i0_secondary_d) | - not a good idea, bad for performance
                                (i1_dp.alu & i0_dp.load) |
                                (i1_dp.alu & i0_dp.mul)
                                ) & ~disable_secondary;  // tag_disable_secondary: 2




   assign store_data_bypass_c2 =  ((             i0_dp.store & i0_rs2_depth_d[3:0] == 4'd1 & i0_rs2_class_d.load) |
                              (             i0_dp.store & i0_rs2_depth_d[3:0] == 4'd2 & i0_rs2_class_d.load) |
                              (~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd1 & i1_rs2_class_d.load) |
                              (~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd2 & i1_rs2_class_d.load));

   assign store_data_bypass_c1 =  ((             i0_dp.store & i0_rs2_depth_d[3:0] == 4'd3 & i0_rs2_class_d.load) |
                              (             i0_dp.store & i0_rs2_depth_d[3:0] == 4'd4 & i0_rs2_class_d.load) |
                              (~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd3 & i1_rs2_class_d.load) |
                              (~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd4 & i1_rs2_class_d.load));
	
	// tag_load_ldst_bypass_c1: 1
   assign load_ldst_bypass_c1 =  ((         (i0_dp.load | i0_dp.store) & i0_rs1_depth_d[3:0] == 4'd3 & i0_rs1_class_d.load) |
                              (             (i0_dp.load | i0_dp.store) & i0_rs1_depth_d[3:0] == 4'd4 & i0_rs1_class_d.load) |
                              (~i0_dp.lsu & (i1_dp.load | i1_dp.store) & i1_rs1_depth_d[3:0] == 4'd3 & i1_rs1_class_d.load) |
                              (~i0_dp.lsu & (i1_dp.load | i1_dp.store) & i1_rs1_depth_d[3:0] == 4'd4 & i1_rs1_class_d.load));

							  

// tag_load_mul_rs1_bypass_e1: 1						  
   assign load_mul_rs1_bypass_e1 =  ((             (i0_dp.mul) & i0_rs1_depth_d[3:0] == 4'd3 & i0_rs1_class_d.load) |
                                     (             (i0_dp.mul) & i0_rs1_depth_d[3:0] == 4'd4 & i0_rs1_class_d.load) |
                                     (~i0_dp.mul & (i1_dp.mul) & i1_rs1_depth_d[3:0] == 4'd3 & i1_rs1_class_d.load) |
                                     (~i0_dp.mul & (i1_dp.mul) & i1_rs1_depth_d[3:0] == 4'd4 & i1_rs1_class_d.load));

   assign load_mul_rs2_bypass_e1 =  ((             (i0_dp.mul) & i0_rs2_depth_d[3:0] == 4'd3 & i0_rs2_class_d.load) |
                                     (             (i0_dp.mul) & i0_rs2_depth_d[3:0] == 4'd4 & i0_rs2_class_d.load) |
                                     (~i0_dp.mul & (i1_dp.mul) & i1_rs2_depth_d[3:0] == 4'd3 & i1_rs2_class_d.load) |
                                     (~i0_dp.mul & (i1_dp.mul) & i1_rs2_depth_d[3:0] == 4'd4 & i1_rs2_class_d.load));


									 
									 
   assign store_data_bypass_e4_c3[1:0] = {
                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd1 & i1_rs2_class_d.sec ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd1 & i0_rs2_class_d.sec ),

                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd2 & i1_rs2_class_d.sec ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd2 & i0_rs2_class_d.sec )
                                     };

   assign store_data_bypass_e4_c2[1:0] = {
                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd3 & i1_rs2_class_d.sec ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd3 & i0_rs2_class_d.sec ),

                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd4 & i1_rs2_class_d.sec ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd4 & i0_rs2_class_d.sec )
                                     };

	// tag_store_data_bypass_e4_c1: 1
   assign store_data_bypass_e4_c1[1:0] = {
                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd5 & i1_rs2_class_d.sec ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd5 & i0_rs2_class_d.sec ),

                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd6 & i1_rs2_class_d.sec ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd6 & i0_rs2_class_d.sec )
                                     };




   assign i0_mul_block_d = (i0_not_alu_eff & i0_rs1_class_d.mul & i0_rs1_match_e1_e2) |
                           (i0_not_alu_eff & i0_rs2_class_d.mul & i0_rs2_match_e1_e2); // i0 因为乘法指令阻塞 look_tag

   assign i1_mul_block_d = (i1_not_alu_eff & i1_rs1_class_d.mul & i1_rs1_match_e1_e2) |
                           (i1_not_alu_eff & i1_rs2_class_d.mul & i1_rs2_match_e1_e2);

   assign i0_secondary_block_d = ((~i0_dp.alu & i0_rs1_class_d.sec & i0_rs1_match_e1_e3) |
                                  (~i0_dp.alu & i0_rs2_class_d.sec & i0_rs2_match_e1_e3 & ~i0_dp.store)) & ~disable_secondary; // 第4阶段加法器 E4阶段 look_tag

   assign i1_secondary_block_d = ((~i1_dp.alu & i1_rs1_class_d.sec & i1_rs1_match_e1_e3) |
                                  (~i1_dp.alu & i1_rs2_class_d.sec & i1_rs2_match_e1_e3 & ~i1_dp.store) & ~disable_secondary); // tag_disable_secondary: 2

// i0 i1 阻塞赋值   end





								  
								  
   // use this to flop the npc address of a divide for externals
   // this logic will change for delay wb of divides
   assign dec_div_decode_e4 = e4d.i0div;  // ouput，在本模块没有使用


   assign dec_tlu_i0_valid_e4 = (e4d.i0valid & ~e4d.i0div & ~flush_lower_wb) | exu_div_finish;
   assign dec_tlu_i1_valid_e4 = e4d.i1valid & ~flush_lower_wb;   // 两个 ouput，在本模块没有使用



   assign dt.legal     =  i0_legal_decode_d                ;
   assign dt.icaf      =  i0_icaf_d & i0_legal_decode_d;            // dbecc is icaf exception
   assign dt.icaf_second   =  dec_i0_icaf_second_d & i0_legal_decode_d;     // this includes icaf and dbecc
   assign dt.perr      =   dec_i0_perr_d & i0_legal_decode_d;
   assign dt.sbecc     =   dec_i0_sbecc_d & i0_legal_decode_d;
   assign dt.fence_i   = (i0_dp.fence_i | debug_fence_i) & i0_legal_decode_d;

// put pmu info into the trap packet
   assign dt.pmu_i0_itype = i0_itype; // tag_i0_itype: 2
   assign dt.pmu_i1_itype = i1_itype;
   
   
   assign dt.pmu_i0_br_unpred = i0_br_unpred;  // // tag_i0_br_unpred: 2
   assign dt.pmu_i1_br_unpred = i1_br_unpred;
   assign dt.pmu_divide = 1'b0;
   assign dt.pmu_lsu_misaligned = 1'b0;

   
   assign dt.i0trigger[3:0] = dec_i0_trigger_match_d[3:0] & {4{dec_i0_decode_d & ~i0_div_decode_d}};
   assign dt.i1trigger[3:0] = dec_i1_trigger_match_d[3:0] & {4{dec_i1_decode_d}};


   
  
// e1t, e2t, e3t, e4t 变量的赋值     begin
   rvdffe #( $bits(trap_pkt_t) ) trap_e1ff (.*, .en(i0_e1_ctl_en), .din( dt),  .dout(e1t));  // tag_e1t: 1

  
   always_comb begin
      e1t_in = e1t;
      e1t_in.i0trigger[3:0] = e1t.i0trigger & ~{4{flush_final_e3}};
      e1t_in.i1trigger[3:0] = e1t.i1trigger & ~{4{flush_final_e3}};
   end

   rvdffe #( $bits(trap_pkt_t) ) trap_e2ff (.*, .en(i0_e2_ctl_en), .din(e1t_in),  .dout(e2t));

   always_comb begin
      e2t_in = e2t;
      e2t_in.i0trigger[3:0] = e2t.i0trigger & ~{4{flush_final_e3 | flush_lower_wb}};
      e2t_in.i1trigger[3:0] = e2t.i1trigger & ~{4{flush_final_e3 | flush_lower_wb}};
   end

   rvdffe  #($bits(trap_pkt_t) ) trap_e3ff (.*, .en(i0_e3_ctl_en), .din(e2t_in),  .dout(e3t));


    always_comb begin
      e3t_in = e3t;

       e3t_in.i0trigger[3:0] = ({4{(e3d.i0load | e3d.i0store)}} & lsu_trigger_match_dc3[3:0]) | e3t.i0trigger[3:0];
       e3t_in.i1trigger[3:0] = ~{4{i0_flush_final_e3}} & (({4{~(e3d.i0load | e3d.i0store)}} & lsu_trigger_match_dc3[3:0]) | e3t.i1trigger[3:0]);

       e3t_in.pmu_lsu_misaligned = lsu_pmu_misaligned_dc3;   // only valid if a load/store is valid in e3 stage

      if (freeze | flush_lower_wb) e3t_in = '0 ;
   end


   rvdffe #( $bits(trap_pkt_t) ) trap_e4ff (.*, .en(i0_e4_ctl_en), .din(e3t_in),  .dout(e4t));

// e1t, e2t, e3t, e4t 变量的赋值     end
   
   
   
   
   

   assign freeze_e3 = freeze & ~freeze_before;

   rvdff #(1) freeze_before_ff (.*, .clk(active_clk), .din(freeze), .dout(freeze_before));

   rvdff #(1) freeze_e4_ff     (.*, .clk(active_clk), .din(freeze_e3), .dout(freeze_e4));

// these signals pipe down in the event of a freeze at dc3, needed by trap to compute triggers for a load
   rvdffe #(9) e4_trigger_ff   (.*, .en(freeze_e3), .din({e3d.i0load,e3t.i0trigger[3:0],e3t.i1trigger[3:0]}), .dout({e4d_i0load,e4t_i0trigger[3:0],e4t_i1trigger[3:0]}));

   always_comb begin

      if (exu_div_finish)    // wipe data for exu_div_finish - safer
        dec_tlu_packet_e4 = '0;
      else
        dec_tlu_packet_e4 = e4t;

      dec_tlu_packet_e4.legal = e4t.legal | exu_div_finish;
      dec_tlu_packet_e4.i0trigger[3:0] = (exu_div_finish) ? div_trigger[3:0] : e4t.i0trigger[3:0];

      dec_tlu_packet_e4.pmu_divide = exu_div_finish;

      if (freeze_e4) begin  // in case of freeze, pipe down trigger information
         dec_tlu_packet_e4.i0trigger[3:0] = e4t_i0trigger[3:0];
         dec_tlu_packet_e4.i1trigger[3:0] = e4t_i1trigger[3:0];
      end

   end

   assign dec_i0_load_e4 = e4d_i0load;


//   assign dec_tlu_packet_e4 = e4t;



// end tlu stuff














// 为 i0_dc 变量赋值, 通过 i0_dc 为 i0_e1c, i0_e2c, i0_e3c, i0_e4c, i0_wbc,  i1_dc, i1_e1c, i1_e2c, i1_e3c, i1_e4c, i1_wbc 变量赋值   begin   

   // i0_dc: i0 decode class_pkt_t 
   // i0_e1c: i0 e1 class_pkt_t
   // i0_wbc: i0 write-back class_pkt_t
   
   // i0_rs1_class_d 使用到的变量: i0_e1c, i0_e2c, i0_e3c, i0_e4c, i0_wbc
   // 							   i1_e1c, i1_e2c, i1_e3c, i1_e4c, i1_wbc        tag_i0_class_pkt_t: 1 
   
   // i0,   tag_i0_dc: 1
   assign i0_dc.mul   = i0_dp.mul & i0_legal_decode_d; // tag_i0_dp: 2, tag_i0_legal_decode_d: 2
   assign i0_dc.load  = i0_dp.load & i0_legal_decode_d;
   assign i0_dc.sec = i0_dp.alu &  i0_secondary_d & i0_legal_decode_d; // tag_i0_secondary_d: 2
   assign i0_dc.alu = i0_dp.alu & ~i0_secondary_d & i0_legal_decode_d;

   rvdffs #( $bits(class_pkt_t) ) i0_e1c_ff (.*, .en(i0_e1_ctl_en), .clk(active_clk), .din(i0_dc),   .dout(i0_e1c)); //   active_clk: clk except for halt / pause
   rvdffs #( $bits(class_pkt_t) ) i0_e2c_ff (.*, .en(i0_e2_ctl_en), .clk(active_clk), .din(i0_e1c),  .dout(i0_e2c));
   rvdffs #( $bits(class_pkt_t) ) i0_e3c_ff (.*, .en(i0_e3_ctl_en), .clk(active_clk), .din(i0_e2c),  .dout(i0_e3c));

   assign i0_e4c_in = (freeze) ? '0 : i0_e3c;  // 临时中间变量

   rvdffs  #( $bits(class_pkt_t) ) i0_e4c_ff (.*, .en(i0_e4_ctl_en),              .clk(active_clk), .din(i0_e4c_in), .dout(i0_e4c));

   rvdffs  #( $bits(class_pkt_t) ) i0_wbc_ff (.*, .en(i0_wb_ctl_en),              .clk(active_clk), .din(i0_e4c),    .dout(i0_wbc));


   // i1
   assign i1_dc.mul   = i1_dp.mul & dec_i1_decode_d;
   assign i1_dc.load  = i1_dp.load & dec_i1_decode_d;
   assign i1_dc.sec = i1_dp.alu &  i1_secondary_d & dec_i1_decode_d;
   assign i1_dc.alu = i1_dp.alu & ~i1_secondary_d & dec_i1_decode_d;

   rvdffs #( $bits(class_pkt_t) ) i1_e1c_ff (.*, .en(i0_e1_ctl_en), .clk(active_clk), .din(i1_dc),   .dout(i1_e1c));
   rvdffs #( $bits(class_pkt_t) ) i1_e2c_ff (.*, .en(i0_e2_ctl_en), .clk(active_clk), .din(i1_e1c),  .dout(i1_e2c));
   rvdffs #( $bits(class_pkt_t) ) i1_e3c_ff (.*, .en(i0_e3_ctl_en), .clk(active_clk), .din(i1_e2c),  .dout(i1_e3c));

   assign i1_e4c_in = (freeze) ? '0 : i1_e3c; // 临时中间变量

   rvdffs #( $bits(class_pkt_t) ) i1_e4c_ff (.*, .en(i0_e4_ctl_en), .clk(active_clk), .din(i1_e4c_in), .dout(i1_e4c));

   rvdffs #( $bits(class_pkt_t) ) i1_wbc_ff (.*, .en(i0_wb_ctl_en), .clk(active_clk), .din(i1_e4c),    .dout(i1_wbc));
   
   
// 为 i0_dc 变量赋值, 通过 i0_dc 为 i0_e1c, i0_e2c, i0_e3c, i0_e4c, i0_wbc,  i1_dc, i1_e1c, i1_e2c, i1_e3c, i1_e4c, i1_wbc 变量赋值    end






	// tag_dd: 1,  dd:decode dest_pkt_t
   assign dd.i0rd[4:0] = i0r.rd[4:0]; // tag_i0r: 2,  dd.i0rd 目的寄存器赋值   

   assign dd.i0v = i0_rd_en_d & i0_legal_decode_d;   // tag_i0_rd_en_d: 2, tag_i0_legal_decode_d: 2                  tag_answer 判断是否要写寄存器
   assign dd.i0valid =              dec_i0_decode_d;  // has flush_final_e3     tag_dec_i0_decode_d: 2          tag_answer    
   

   assign dd.i0mul = i0_dp.mul & i0_legal_decode_d;   // tag_i0_dp: 2
   assign dd.i0load = i0_dp.load & i0_legal_decode_d;
   assign dd.i0store = i0_dp.store & i0_legal_decode_d;
   assign dd.i0div = i0_dp.div & i0_legal_decode_d;

   assign dd.i0secondary = i0_secondary_d & i0_legal_decode_d; // tag_i0_secondary_d: 2
   assign dd.i0secload = i0_load_all_d & i0_legal_decode_d; // tag_i0_secondary_d: 2


   assign dd.i0secblock = i0_load_block_d & i0_legal_decode_d; // tag_i0_secondary_d: 2 
   
   // assign dd.i0secload = i0_dp.load & i0_legal_decode_d; // tag_i0_secondary_d: 2


   assign dd.i1rd[4:0] = i1r.rd[4:0];

   assign dd.i1v = i1_rd_en_d & dec_i1_decode_d; 
   assign dd.i1valid =              dec_i1_decode_d;
   
   assign dd.i1mul = i1_dp.mul;  // tag_i1_dp
   assign dd.i1load = i1_dp.load;
   assign dd.i1store = i1_dp.store;

   assign dd.i1secondary = i1_secondary_d & dec_i1_decode_d; // tag_i1_secondary_d: 2
   assign dd.i1secload = i1_load_all_d & dec_i1_decode_d; // tag_i0_secondary_d: 2

   // assign dd.i1secblock = i1_load_block_d;

   assign dd.i1secblock = i1_load_block_d & dec_i1_decode_d;

   assign dd.csrwen = dec_csr_wen_unq_d & i0_legal_decode_d; // tag_dec_csr_wen_unq_d: 2
   assign dd.csrwonly = i0_csr_write_only_d & dec_i0_decode_d;  // tag_i0_csr_write_only_d: 2
   assign dd.csrwaddr[11:0] = i0[31:20];    // csr write address for rd==0 case ,      tag_i0: 2


   
   
   
   
// 变量的赋值    begin   tag_define: 1
	// i0 赋值
   assign i0_pipe_en[5] = dec_i0_decode_d;

   rvdffs #(3) i0cg0ff (.*, .clk(active_clk), .en(~freeze), .din(i0_pipe_en[5:3]), .dout(i0_pipe_en[4:2]));
   rvdff  #(2) i0cg1ff (.*, .clk(active_clk),               .din(i0_pipe_en[2:1]), .dout(i0_pipe_en[1:0]));


   assign i0_e1_ctl_en = (|i0_pipe_en[5:4] | clk_override) & ~freeze;  // tag_answer  不用关注
   assign i0_e2_ctl_en = (|i0_pipe_en[4:3] | clk_override) & ~freeze;
   assign i0_e3_ctl_en = (|i0_pipe_en[3:2] | clk_override) & ~freeze;
   assign i0_e4_ctl_en = (|i0_pipe_en[2:1] | clk_override);
   assign i0_wb_ctl_en = (|i0_pipe_en[1:0] | clk_override);

   assign i0_e1_data_en = (i0_pipe_en[5] | clk_override) & ~freeze;
   assign i0_e2_data_en = (i0_pipe_en[4] | clk_override) & ~freeze;  // tag_i0_e2_data_en: 1
   assign i0_e3_data_en = (i0_pipe_en[3] | clk_override) & ~freeze;
   assign i0_e4_data_en = (i0_pipe_en[2] | clk_override);
   assign i0_wb_data_en = (i0_pipe_en[1] | clk_override);
   assign i0_wb1_data_en = (i0_pipe_en[0] | clk_override);

   assign dec_i0_data_en[4:2] = {i0_e1_data_en, i0_e2_data_en, i0_e3_data_en}; // dec_i0_data_en: clock-gating logic
   assign dec_i0_ctl_en[4:1]  = {i0_e1_ctl_en, i0_e2_ctl_en, i0_e3_ctl_en, i0_e4_ctl_en};  // dec_i0_data_en, dec_i0_ctl_en: output, to exu.sv


   // i1 赋值
   assign i1_pipe_en[5] = dec_i1_decode_d;  //  tag_dec_i1_decode_d: 2

   rvdffs #(3) i1cg0ff (.*, .clk(free_clk), .en(~freeze), .din(i1_pipe_en[5:3]), .dout(i1_pipe_en[4:2]));
   rvdff  #(2) i1cg1ff (.*, .clk(free_clk),               .din(i1_pipe_en[2:1]), .dout(i1_pipe_en[1:0]));


   assign i1_e1_ctl_en = (|i1_pipe_en[5:4] | clk_override) & ~freeze;
   assign i1_e2_ctl_en = (|i1_pipe_en[4:3] | clk_override) & ~freeze; 
   assign i1_e3_ctl_en = (|i1_pipe_en[3:2] | clk_override) & ~freeze;
   assign i1_e4_ctl_en = (|i1_pipe_en[2:1] | clk_override);
   assign i1_wb_ctl_en = (|i1_pipe_en[1:0] | clk_override);

   assign i1_e1_data_en = (i1_pipe_en[5] | clk_override) & ~freeze;
   assign i1_e2_data_en = (i1_pipe_en[4] | clk_override) & ~freeze; // tag_i1_e2_data_en: 1
   assign i1_e3_data_en = (i1_pipe_en[3] | clk_override) & ~freeze;
   assign i1_e4_data_en = (i1_pipe_en[2] | clk_override);
   assign i1_wb_data_en = (i1_pipe_en[1] | clk_override);
   assign i1_wb1_data_en = (i1_pipe_en[0] | clk_override);

   assign dec_i1_data_en[4:2] = {i1_e1_data_en, i1_e2_data_en, i1_e3_data_en};
   assign dec_i1_ctl_en[4:1]  = {i1_e1_ctl_en, i1_e2_ctl_en, i1_e3_ctl_en, i1_e4_ctl_en};  // dec_i1_data_en, dec_i1_data_en: output, to exu.sv
   
// 变量的赋值    end  


/*
   logic i0_match_e1_i1, i0_match_e1_i0;
   logic i1_match_e1_i1, i1_match_e1_i0;

   logic i0_match_e2_i1, i0_match_e2_i0;
   logic i1_match_e2_i1, i1_match_e2_i0;



   assign i0_match_e1_i1 = i0_rs1_depth_d[3:0] == 4'd1 | i0_rs2_depth_d[3:0] == 4'd1;

   assign i0_match_e1_i0 = i0_rs1_depth_d[3:0] == 4'd2 | i0_rs2_depth_d[3:0] == 4'd2;

   assign i1_match_e1_i1 = i1_rs1_depth_d[3:0] == 4'd1 | i1_rs2_depth_d[3:0] == 4'd1;

   assign i1_match_e1_i0 = i1_rs1_depth_d[3:0] == 4'd2 | i1_rs2_depth_d[3:0] == 4'd2;


   assign i0_match_e2_i1 = i0_rs1_depth_d[3:0] == 4'd3 | i0_rs2_depth_d[3:0] == 4'd3;

   assign i0_match_e2_i0 = i0_rs1_depth_d[3:0] == 4'd4 | i0_rs2_depth_d[3:0] == 4'd4;

   assign i1_match_e2_i1 = i1_rs1_depth_d[3:0] == 4'd3 | i1_rs2_depth_d[3:0] == 4'd3;

   assign i1_match_e2_i0 = i1_rs1_depth_d[3:0] == 4'd4 | i1_rs2_depth_d[3:0] == 4'd4;
*/
   

  
   
   
   
   
   
// tag_not_look  begin

   assign dec_i0_waddr_wb[4:0] = wbd.i0rd[4:0];

   // squash same write, take last write assuming we don't kill the I1 write for some reason.
   assign     i0_wen_wb = wbd.i0v & ~(~dec_tlu_i1_kill_writeb_wb & ~i1_load_kill_wen & wbd.i0v & wbd.i1v & (wbd.i0rd[4:0] == wbd.i1rd[4:0])) & ~dec_tlu_i0_kill_writeb_wb;
   assign dec_i0_wen_wb = i0_wen_wb & ~i0_load_kill_wen;  // don't write a nonblock load 1st time down the pipe

   assign dec_i0_wdata_wb[31:0] = i0_result_wb[31:0]; // i0 write data 结果 look_tag    tag_answer dec_i0_wdata_wb   wdata 写回的数据  wb write back

   assign dec_i1_waddr_wb[4:0] = wbd.i1rd[4:0];

   assign     i1_wen_wb = wbd.i1v & ~dec_tlu_i1_kill_writeb_wb;
   assign dec_i1_wen_wb = i1_wen_wb & ~i1_load_kill_wen;

   assign dec_i1_wdata_wb[31:0] = i1_result_wb[31:0]; // i1 write data ？

// divide stuff


   assign div_stall = exu_div_stall | div_stall_ff;   // hold for 1 extra cycle so wb can happen before next inst

   rvdff  #(1) divstallff (.*, .clk(active_clk), .din(exu_div_stall), .dout(div_stall_ff));


   assign i0_div_decode_d = i0_legal_decode_d & i0_dp.div;

   rvdffe #(31) divpcff (.*, .en(i0_div_decode_d), .din(dec_i0_pc_d[31:1]), .dout(div_pc[31:1]));

   rvdffs #(4) divtriggerff (.*, .en(i0_div_decode_d), .clk(active_clk), .din(dec_i0_trigger_match_d[3:0]), .dout(div_trigger[3:0]));

   rvdffs #(5) divwbaddrff (.*, .en(i0_div_decode_d), .clk(active_clk), .din(i0r.rd[4:0]), .dout(div_waddr_wb[4:0]));

   // active_clk -> used for clockgating for wb stage ctl logic
   rvdff  #(1) divwbff (.*, .clk(active_clk), .din(exu_div_finish), .dout(div_wen_wb));


// tag_not_look  end  
   
   
   
  


  
// i0_rs1_bypass_data_d 赋值的 输入值的赋值   begin   tag_i_rs_bypass_input: 1



// i0_rs1_bypass_data_d 使用到的变量: i0_result_e1, i0_result_e2, i0_result_e3_final, i0_result_e4_final, i0_result_wb
// 									  i1_result_e1, i1_result_e2, i1_result_e3_final, i1_result_e4_final, i1_result_wb

	// assign 赋值没有延迟, 只要 exu_i0_result_e1 的值变化, i0_result_e1 的值就跟着变化
    assign i0_result_e1[31:0] = exu_i0_result_e1[31:0];  		// exu_i0_result_e1: from primary alu's  
    assign i1_result_e1[31:0] = exu_i1_result_e1[31:0];  		

   /*
			由于每个寄存器都依赖于各自的使能信号来更新数据，如果这些使能信号在某些时钟周期内无效，则寄存器保持不变。
			因此，i0_result_e1、i0_result_e2 和 i0_result_e3 在不同的时钟周期可能会出现不同的值。
   */  

   //  assign i0_result_e1[31:0] = (dd.i0v & dd.i0load) ? i0_predicted_result_e1[31:0] : exu_i0_result_e1[31:0];  		
   //  assign i1_result_e1[31:0] = (dd.i1v & dd.i1load) ? i1_predicted_result_e1[31:0] : exu_i1_result_e1[31:0];  		
   
   // pipe the results down the pipe
   rvdffe #(32) i0e2resultff (.*, .en(i0_e2_data_en), .din(i0_result_e1[31:0]), .dout(i0_result_e2[31:0])); // tag_i0_e2_data_en: 2
   rvdffe #(32) i1e2resultff (.*, .en(i1_e2_data_en), .din(i1_result_e1[31:0]), .dout(i1_result_e2[31:0])); // tag_i1_e2_data_en: 2

   rvdffe #(32) i0e3resultff (.*, .en(i0_e3_data_en), .din(i0_result_e2[31:0]), .dout(i0_result_e3[31:0]));
   rvdffe #(32) i1e3resultff (.*, .en(i1_e3_data_en), .din(i1_result_e2[31:0]), .dout(i1_result_e3[31:0]));


   // tag_e3d: 2,    e3d: e3 dest_pkt_t    
   // assign i0_result_e3_final[31:0] = (e3d.i0v & e3d.i0load) ? lsu_result_dc3[31:0] : (e3d.i0v & e3d.i0mul) ? exu_mul_result_e3[31:0] : i0_result_e3[31:0];  // lsu_result_dc3: from lsu_lsc_ctl.sv
   // assign i1_result_e3_final[31:0] = (e3d.i1v & e3d.i1load) ? lsu_result_dc3[31:0] : (e3d.i1v & e3d.i1mul) ? exu_mul_result_e3[31:0] : i1_result_e3[31:0];
	// tag_answer final  可选的值有多个，确定选择最终的值  
   
   // 
   assign i0_result_e3_final[31:0] = (e3d.i0v & e3d.i0load) ? lsu_result_dc3[31:0] : (e3d.i0v & e3d.i0mul) ? exu_mul_result_e3[31:0] : i0_result_e3[31:0];  // lsu_result_dc3: from lsu_lsc_ctl.sv
   assign i1_result_e3_final[31:0] = (e3d.i1v & e3d.i1load) ? lsu_result_dc3[31:0] : (e3d.i1v & e3d.i1mul) ? exu_mul_result_e3[31:0] : i1_result_e3[31:0];

   // assign i0_result_e3_final[31:0] = (e3d_d1.i0v & e3d_d1.i0load) ? lsu_result_dc3[31:0] : (e3d_d1.i0v & e3d_d1.i0mul) ? exu_mul_result_e3[31:0] : i0_result_e3[31:0];  // lsu_result_dc3: from lsu_lsc_ctl.sv
   // assign i1_result_e3_final[31:0] = (e3d_d1.i1v & e3d_d1.i1load) ? lsu_result_dc3[31:0] : (e3d_d1.i1v & e3d_d1.i1mul) ? exu_mul_result_e3[31:0] : i1_result_e3[31:0];


   rvdffe #(32) i0e4resultff (.*, .en(i0_e4_data_en), .din(i0_result_e3_final[31:0]), .dout(i0_result_e4[31:0]));
   rvdffe #(32) i1e4resultff (.*, .en(i1_e4_data_en), .din(i1_result_e3_final[31:0]), .dout(i1_result_e4[31:0]));

   
   // exu_i0_result_e4 是第二个 alu 计算出来的值
   // i0_result_e4 是通过流水线传递出来的值
   
   /*
   assign i0_result_e4_final[31:0] =
                                     (          e4d_d1.i0secondary) ? exu_i0_result_e4[31:0] : (e4d_d1.i0v & e4d_d1.i0load) ? lsu_result_corr_dc4[31:0] : i0_result_e4[31:0];

   assign i1_result_e4_final[31:0] =
                                     (e4d_d1.i1v & e4d_d1.i1secondary) ? exu_i1_result_e4[31:0] : (e4d_d1.i1v & e4d_d1.i1load) ? lsu_result_corr_dc4[31:0] :i1_result_e4[31:0];
	
    
   */
    assign i0_result_e4_final[31:0] =
                                     (          e4d.i0secondary) ? exu_i0_result_e4[31:0] : (e4d.i0v & e4d.i0load) ? lsu_result_corr_dc4[31:0] : i0_result_e4[31:0];

   assign i1_result_e4_final[31:0] =
                                     (e4d.i1v & e4d.i1secondary) ? exu_i1_result_e4[31:0] : (e4d.i1v & e4d.i1load) ? lsu_result_corr_dc4[31:0] :i1_result_e4[31:0];
	
   rvdffe #(32) i0wbresultff (.*, .en(i0_wb_data_en), .din(i0_result_e4_final[31:0]), .dout(i0_result_wb_raw[31:0]));
   rvdffe #(32) i1wbresultff (.*, .en(i1_wb_data_en), .din(i1_result_e4_final[31:0]), .dout(i1_result_wb_raw[31:0]));


   // assign i0_result_wb[31:0] = (div_wen_wb) ? exu_div_result[31:0] : i0_result_wb_raw[31:0];  // exu_div_result: div result   from exu.sv

   // assign i1_result_wb[31:0] = i1_result_wb_raw[31:0];

    assign i0_result_wb[31:0] = (div_wen_wb) ? exu_div_result[31:0] : i0_result_wb_raw[31:0];  // exu_div_result: div result   from exu.sv

   assign i1_result_wb[31:0] =  i1_result_wb_raw[31:0];

   /*
// 
   assign i0_result_e3_final[31:0] =  (e3d.i0v & e3d.i0mul) ? exu_mul_result_e3[31:0] : i0_result_e3[31:0];  // lsu_result_dc3: from lsu_lsc_ctl.sv
   assign i1_result_e3_final[31:0] =  (e3d.i1v & e3d.i1mul) ? exu_mul_result_e3[31:0] : i1_result_e3[31:0];


   rvdffe #(32) i0e4resultff (.*, .en(i0_e4_data_en), .din(i0_result_e3_final[31:0]), .dout(i0_result_e4[31:0]));
   rvdffe #(32) i1e4resultff (.*, .en(i1_e4_data_en), .din(i1_result_e3_final[31:0]), .dout(i1_result_e4[31:0]));

   
   // exu_i0_result_e4 是第二个 alu 计算出来的值
   // i0_result_e4 是通过流水线传递出来的值
  
    assign i0_result_e4_final[31:0] =
                                     (          e4d.i0secondary) ? exu_i0_result_e4[31:0] :  i0_result_e4[31:0];

   assign i1_result_e4_final[31:0] =
                                     (e4d.i1v & e4d.i1secondary) ? exu_i1_result_e4[31:0] : i1_result_e4[31:0];
	
   rvdffe #(32) i0wbresultff (.*, .en(i0_wb_data_en), .din(i0_result_e4_final[31:0]), .dout(i0_result_wb_raw[31:0]));
   rvdffe #(32) i1wbresultff (.*, .en(i1_wb_data_en), .din(i1_result_e4_final[31:0]), .dout(i1_result_wb_raw[31:0]));




   assign i0_result_wb[31:0] = (div_wen_wb) ? exu_div_result[31:0] :  (wbd.i0v & wbd.i0load) ? lsu_result_corr_dc4[31:0] : i0_result_wb_raw[31:0];  // exu_div_result: div result   from exu.sv

   assign i1_result_wb[31:0] = (wbd.i1v & wbd.i1load) ? lsu_result_corr_dc4[31:0] : i1_result_wb_raw[31:0];

   // assign i0_result_wb[31:0] = (div_wen_wb) ? exu_div_result[31:0] :  (i0_e3_load_wen_wb) ? lsu_result_dc3[31:0] : (i0_e4_load_wen_wb) ? lsu_result_corr_dc4[31:0] :  i0_result_wb_raw[31:0];  // exu_div_result: div result   from exu.sv

   // assign i1_result_wb[31:0] = (i1_e3_load_wen_wb) ? lsu_result_dc3[31:0] : (i1_e4_load_wen_wb) ? lsu_result_corr_dc4[31:0] : i1_result_wb_raw[31:0];

   */


      // active_clk -> used for clockgating for wb stage ctl logic
   // rvdff  #(1) divwbff (.*, .clk(active_clk), .din(exu_div_finish), .dout(div_wen_wb));

   logic i0_e3_load_wen_wb, i1_e3_load_wen_wb;
   logic i0_e4_load_wen_wb, i1_e4_load_wen_wb;

   rvdff  #(1) i0_e3_loadwbff (.*, .clk(active_clk), .din(wbd.i0v & wbd.i0load), .dout( i0_e3_load_wen_wb ));
   rvdff  #(1) i1_e3_loadwbff (.*, .clk(active_clk), .din(wbd.i1v & wbd.i1load), .dout( i1_e3_load_wen_wb ));

   rvdff  #(1) i0_e4_loadwbff (.*, .clk(active_clk), .din(wbd.i0v & wbd.i0load), .dout( i0_e4_load_wen_wb ));
   rvdff  #(1) i1_e4_loadwbff (.*, .clk(active_clk), .din(wbd.i1v & wbd.i1load), .dout( i1_e4_load_wen_wb ));




   logic [31 : 0] lsu_result_dc3_d1, lsu_result_dc3_d2, lsu_result_dc3_d3, lsu_result_dc3_d4;  

   logic i0_e2_data_en_d1, i0_e2_data_en_d2, i0_e2_data_en_d3, i0_e2_data_en_d4;
   logic i1_e2_data_en_d1, i1_e2_data_en_d2, i1_e2_data_en_d3, i1_e2_data_en_d4;

   logic i0_e3_data_en_d1, i0_e3_data_en_d2, i0_e3_data_en_d3, i0_e3_data_en_d4;
   logic i1_e3_data_en_d1, i1_e3_data_en_d2, i1_e3_data_en_d3, i1_e3_data_en_d4;

   logic i0_e4_data_en_d1, i0_e4_data_en_d2, i0_e4_data_en_d3, i0_e4_data_en_d4;
   logic i1_e4_data_en_d1, i1_e4_data_en_d2, i1_e4_data_en_d3, i1_e4_data_en_d4;

   logic i0_wb_data_en_d1, i0_wb_data_en_d2, i0_wb_data_en_d3, i0_wb_data_en_d4;
   logic i1_wb_data_en_d1, i1_wb_data_en_d2, i1_wb_data_en_d3, i1_wb_data_en_d4;


   dest_pkt_t e3d_d1, e3d_d2, e3d_d3, e3d_d4;

   dest_pkt_t e4d_d1, e4d_d2, e4d_d3, e4d_d4;


   

   
   
   // 能够保证 10 ns 后传值
   always_ff @( posedge clk or negedge rst_l ) begin : delay_result
      /*
      if(!rst_l) begin
         lsu_result_dc3_d1 <= '0;
         lsu_result_dc3_d2 <= '0;
         lsu_result_dc3_d3 <= '0;
         lsu_result_dc3_d4 <= '0;

         i0_e2_data_en_d1 <= '0;
         i0_e2_data_en_d2 <= '0;
         i0_e2_data_en_d3 <= '0;
         i0_e2_data_en_d4 <= '0;

         i1_e2_data_en_d1 <= '0;
         i1_e2_data_en_d2 <= '0;
         i1_e2_data_en_d3 <= '0;
         i1_e2_data_en_d4 <= '0;


         i0_e3_data_en_d1 <= '0;
         i0_e3_data_en_d2 <= '0;
         i0_e3_data_en_d3 <= '0;
         i0_e3_data_en_d4 <= '0;

         i1_e3_data_en_d1 <= '0;
         i1_e3_data_en_d2 <= '0;
         i1_e3_data_en_d3 <= '0;
         i1_e3_data_en_d4 <= '0;


         i0_e4_data_en_d1 <= '0;
         i0_e4_data_en_d2 <= '0;
         i0_e4_data_en_d3 <= '0;
         i0_e4_data_en_d4 <= '0;

         i1_e4_data_en_d1 <= '0;
         i1_e4_data_en_d2 <= '0;
         i1_e4_data_en_d3 <= '0;
         i1_e4_data_en_d4 <= '0;


         i0_wb_data_en_d1 <= '0;
         i0_wb_data_en_d2 <= '0;
         i0_wb_data_en_d3 <= '0;
         i0_wb_data_en_d4 <= '0;

         i1_wb_data_en_d1 <= '0;
         i1_wb_data_en_d2 <= '0;
         i1_wb_data_en_d3 <= '0;
         i1_wb_data_en_d4 <= '0;

         e3d_d1 <= '0;
         e3d_d2 <= '0;
         e3d_d3 <= '0;
         e3d_d4 <= '0;

         e4d_d1 <= '0;
         e4d_d2 <= '0;
         e4d_d3 <= '0;
         e4d_d4 <= '0;


         
      end else begin
         */
      begin
         
         lsu_result_dc3_d1 <= lsu_result_dc3;
         lsu_result_dc3_d2 <= lsu_result_dc3_d1;
         lsu_result_dc3_d3 <= lsu_result_dc3_d2;
         lsu_result_dc3_d4 <= lsu_result_dc3_d3;


         i0_e2_data_en_d1 <= i0_e2_data_en;
         i0_e2_data_en_d2 <= i0_e2_data_en_d1;
         i0_e2_data_en_d3 <= i0_e2_data_en_d2;
         i0_e2_data_en_d4 <= i0_e2_data_en_d3;

         i1_e2_data_en_d1 <= i1_e2_data_en;
         i1_e2_data_en_d2 <= i1_e2_data_en_d1;
         i1_e2_data_en_d3 <= i1_e2_data_en_d2;
         i1_e2_data_en_d4 <= i1_e2_data_en_d3;



         i0_e3_data_en_d1 <= i0_e3_data_en;
         i0_e3_data_en_d2 <= i0_e3_data_en_d1;
         i0_e3_data_en_d3 <= i0_e3_data_en_d2;
         i0_e3_data_en_d4 <= i0_e3_data_en_d3;

         i1_e3_data_en_d1 <= i1_e3_data_en;
         i1_e3_data_en_d2 <= i1_e3_data_en_d1;
         i1_e3_data_en_d3 <= i1_e3_data_en_d2;
         i1_e3_data_en_d4 <= i1_e3_data_en_d3;


         i0_e4_data_en_d1 <= i0_e4_data_en;
         i0_e4_data_en_d2 <= i0_e4_data_en_d1;
         i0_e4_data_en_d3 <= i0_e4_data_en_d2;
         i0_e4_data_en_d4 <= i0_e4_data_en_d3;

         i1_e4_data_en_d1 <= i1_e4_data_en;
         i1_e4_data_en_d2 <= i1_e4_data_en_d1;
         i1_e4_data_en_d3 <= i1_e4_data_en_d2;
         i1_e4_data_en_d4 <= i1_e4_data_en_d3;


         i0_wb_data_en_d1 <= i0_wb_data_en;
         i0_wb_data_en_d2 <= i0_wb_data_en_d1;
         i0_wb_data_en_d3 <= i0_wb_data_en_d2;
         i0_wb_data_en_d4 <= i0_wb_data_en_d3;

         i1_wb_data_en_d1 <= i1_wb_data_en;
         i1_wb_data_en_d2 <= i1_wb_data_en_d1;
         i1_wb_data_en_d3 <= i1_wb_data_en_d2;
         i1_wb_data_en_d4 <= i1_wb_data_en_d3;

         e3d_d1 <= e3d;
         e3d_d2 <= e3d_d1;
         e3d_d3 <= e3d_d2;
         e3d_d4 <= e3d_d3;

         e4d_d1 <= e4d;
         e4d_d2 <= e4d_d1;
         e4d_d3 <= e4d_d2;
         e4d_d4 <= e4d_d3;

         
         
      end
   end

   
// i0_rs1_bypass_data_d 赋值的 输入值的赋值    end


// Added
	// 解码阶段得到预测值，然后随着流水线将预测值进行传递
	// 实际结果已经算出来的情况下，预测值是否需要更新？

  // assign i0_predicted_result_e1[31:0] = lsu_result_dc3[31:0];  // 解码指令之后，预测的结果
  // assign i1_predicted_result_e1[31:0] = lsu_result_dc3[31:0];  

   // assign i0_predicted_result_e1[31:0] = exu_i0_result_e1[31:0];  // 解码指令之后，预测的结果
   // assign i1_predicted_result_e1[31:0] = exu_i1_result_e1[31:0];  

  // assign i0_predicted_result_d[31:0] = lsu_result_dc3[31:0];  // 解码指令之后，预测的结果
  // assign i1_predicted_result_d[31:0] = lsu_result_dc3[31:0];  
	
	// 在没有实际值的情况下，使用预测值    上面的效果都没有使用 预测器产生预测值的 效果好
   assign i0_predicted_result_d[31:0] = i0_predicted_value_d[31:0];  // 解码指令之后，预测的结果
   assign i1_predicted_result_d[31:0] = i1_predicted_value_d[31:0];  	// i0_predicted_value_d


  rvdffe #(32) i0e1predictedresultff (.*, .en(i0_e1_data_en), .din(i0_predicted_result_d[31:0]), .dout(i0_predicted_result_e1[31:0]));
  rvdffe #(32) i1e1predictedresultff (.*, .en(i1_e1_data_en), .din(i1_predicted_result_d[31:0]), .dout(i1_predicted_result_e1[31:0]));


  rvdffe #(32) i0e2predictedresultff (.*, .en(i0_e2_data_en), .din(i0_predicted_result_e1[31:0]), .dout(i0_predicted_result_e2[31:0]));
  rvdffe #(32) i1e2predictedresultff (.*, .en(i1_e2_data_en), .din(i1_predicted_result_e1[31:0]), .dout(i1_predicted_result_e2[31:0]));

  rvdffe #(32) i0e3predictedresultff (.*, .en(i0_e3_data_en), .din(i0_predicted_result_e2[31:0]), .dout(i0_predicted_result_e3[31:0]));
  rvdffe #(32) i1e3predictedresultff (.*, .en(i1_e3_data_en), .din(i1_predicted_result_e2[31:0]), .dout(i1_predicted_result_e3[31:0]));

  rvdffe #(32) i0e4predictedresultff (.*, .en(i0_e4_data_en), .din(i0_predicted_result_e3[31:0]), .dout(i0_predicted_result_e4_raw[31:0]));
  rvdffe #(32) i1e4predictedresultff (.*, .en(i1_e4_data_en), .din(i1_predicted_result_e3[31:0]), .dout(i1_predicted_result_e4_raw[31:0]));
   



//   assign dec_i0_br_immed_d[12:1] = ({12{ i0_ap.predict_nt }} &           {i0[31],i0[7],i0[30:25],i0[11:8]}) |
//                                    ({12{ i0_ap.predict_t | i0_ap.jal}} & {10'b0,i0_ap_pc4,i0_ap_pc2});

   // all conditional branches are currently predict_nt
   // change this to generate the sequential address for all other cases for NPC requirements at commit
 assign dec_i0_br_immed_d[12:1] = (i0_ap.predict_nt & ~i0_dp.jal) ? i0_br_offset[11:0] : {10'b0,i0_ap_pc4,i0_ap_pc2}; // tag_i0_ap: 2, tag_i0_br_offset: 2

assign dec_i1_br_immed_d[12:1] = (i1_ap.predict_nt & ~i1_dp.jal) ? i1_br_offset[11:0] : {10'b0,i1_ap_pc4,i1_ap_pc2}; // tag_i1_ap: 2


   logic i0_valid_e3, i1_valid_e3;


   logic [8 : 0] i0_load_match_e1_test,  i0_load_match_e2_test, i0_load_match_e2_final_test, i0_load_match_e3_test, i0_load_match_e4_test;

   logic [8 : 0] i1_load_match_e1_test,  i1_load_match_e2_test, i1_load_match_e2_final_test, i1_load_match_e3_test, i1_load_match_e4_test;



   logic load_match_e1_i0,  load_match_e2_i0;

   logic load_match_e1_i1,  load_match_e2_i1;

   assign load_match_e1_i0 = i0_load_match_e1_i0 | i1_load_match_e1_i0;
   assign load_match_e2_i0 = i0_load_match_e2_i0 | i1_load_match_e2_i0;

   assign load_match_e1_i1 = i0_load_match_e1_i1 | i1_load_match_e1_i1;
   assign load_match_e2_i1 = i0_load_match_e2_i1 | i1_load_match_e2_i1;


   logic i0_load_match_e1,  i0_load_match_e2, i0_load_match_e2_final, i0_load_match_e3;
   logic i1_load_match_e1,  i1_load_match_e2, i1_load_match_e2_final, i1_load_match_e3;

  //  logic i0_load_match_e1,  i0_load_match_e2, i0_load_match_e2_final, i0_load_match_e3, i0_load_match_e4;

   // logic i1_load_match_e1,  i1_load_match_e2, i1_load_match_e2_final, i1_load_match_e3, i1_load_match_e4;


   
   assign i0_load_match_e1 = load_match_e1_i0;
   assign i0_load_match_e2_final = i0_load_match_e2 | load_match_e2_i0;

   assign i1_load_match_e1 = load_match_e1_i1;
   assign i1_load_match_e2_final = i1_load_match_e2 | load_match_e2_i1;



   // pipe the results down the pipe
   rvdffe #(10) i0e2matchff (.*, .en(i0_e2_data_en), .din({i0_load_match_e1, i0_load_match_e1_test}), .dout({ i0_load_match_e2, i0_load_match_e2_test} )); 

   rvdffe #(10) i1e2matchff (.*, .en(i1_e2_data_en), .din({i1_load_match_e1, i1_load_match_e1_test}), .dout({ i1_load_match_e2, i1_load_match_e2_test} ));
   

   rvdffe #(10) i0e3matchff (.*, .en(i0_e3_data_en), .din({i0_load_match_e2_final, i0_load_match_e2_test}), .dout({ i0_load_match_e3, i0_load_match_e3_test}));

   rvdffe #(10) i1e3matchff (.*, .en(i1_e3_data_en), .din({i1_load_match_e2_final, i1_load_match_e2_test}), .dout({ i1_load_match_e3, i1_load_match_e3_test}));

   rvdffe #(10) i0e4matchff (.*, .en(i0_e4_data_en), .din({i0_load_match_e3, i0_load_match_e3_test}), .dout({ i0_load_match_e4, i0_load_match_e4_test}));

   rvdffe #(10) i1e4matchff (.*, .en(i1_e4_data_en), .din({i1_load_match_e3, i1_load_match_e3_test}), .dout({ i1_load_match_e4, i1_load_match_e4_test}));










   logic i0_rs1_match_e1_i0, i0_rs1_match_e1_i1;
   logic i0_rs2_match_e1_i0, i0_rs2_match_e1_i1;

   logic i1_rs1_match_e1_i0, i1_rs1_match_e1_i1;
   logic i1_rs2_match_e1_i0, i1_rs2_match_e1_i1;


   logic i0_rs1_match_e2_i0, i0_rs1_match_e2_i1;
   logic i0_rs2_match_e2_i0, i0_rs2_match_e2_i1;

   logic i1_rs1_match_e2_i0, i1_rs1_match_e2_i1;
   logic i1_rs2_match_e2_i0, i1_rs2_match_e2_i1;


   assign i0_rs1_match_e1_i1 = i0_rs1_depth_d[3:0] == 4'd1;      // 1：i1 e1   2: i0 e1
   assign i0_rs2_match_e1_i1 = i0_rs2_depth_d[3:0] == 4'd1;      // 1：i1 e1   2: i0 e1

   assign i0_rs1_match_e1_i0 = i0_rs1_depth_d[3:0] == 4'd2;      // 1：i1 e1   2: i0 e1
   assign i0_rs2_match_e1_i0 = i0_rs2_depth_d[3:0] == 4'd2;      // 1：i1 e1   2: i0 e1


   assign i1_rs1_match_e1_i1 = i1_rs1_depth_d[3:0] == 4'd1;      // 1：i1 e1   2: i0 e1
   assign i1_rs2_match_e1_i1 = i1_rs2_depth_d[3:0] == 4'd1;      // 1：i1 e1   2: i0 e1

   assign i1_rs1_match_e1_i0 = i1_rs1_depth_d[3:0] == 4'd2;      // 1：i1 e1   2: i0 e1
   assign i1_rs2_match_e1_i0 = i1_rs2_depth_d[3:0] == 4'd2;      // 1：i1 e1   2: i0 e1
   



   assign i0_rs1_match_e2_i1 = i0_rs1_depth_d[3:0] == 4'd3;      // 1：i1 e1   2: i0 e1
   assign i0_rs2_match_e2_i1 = i0_rs2_depth_d[3:0] == 4'd3;      // 1：i1 e1   2: i0 e1

   assign i0_rs1_match_e2_i0 = i0_rs1_depth_d[3:0] == 4'd4;      // 1：i1 e1   2: i0 e1
   assign i0_rs2_match_e2_i0 = i0_rs2_depth_d[3:0] == 4'd4;      // 1：i1 e1   2: i0 e1


   assign i1_rs1_match_e2_i1 = i1_rs1_depth_d[3:0] == 4'd3;      // 1：i1 e1   2: i0 e1
   assign i1_rs2_match_e2_i1 = i1_rs2_depth_d[3:0] == 4'd3;      // 1：i1 e1   2: i0 e1

   assign i1_rs1_match_e2_i0 = i1_rs1_depth_d[3:0] == 4'd4;      // 1：i1 e1   2: i0 e1
   assign i1_rs2_match_e2_i0 = i1_rs2_depth_d[3:0] == 4'd4;      // 1：i1 e1   2: i0 e1
   

   
         // i0, i1 在 e1 的依赖 load
         logic rs1_load_match_e1_i0, rs2_load_match_e1_i0;
         logic rs1_load_match_e1_i1, rs2_load_match_e1_i1;
         


         logic i0_mul_vp, i1_mul_vp;

         // assign i0_mul_vp = (~i0_dp.alu & i0_dp.mul);  
         // assign i1_mul_vp = (~i1_dp.alu & i1_dp.mul);

         // assign i0_mul_vp = (i0_not_alu_eff & (i0_dp.mul |  i0_dp.div |  i0_dp.lsu));  
         // assign i1_mul_vp = ( i1_not_alu_eff & (i1_dp.mul |  i1_dp.div |  i1_dp.lsu));

         // assign i0_mul_vp = (i0_dp.mul |  i0_dp.div );  
         // assign i1_mul_vp = (i1_dp.mul |  i1_dp.div );

         assign i0_mul_vp = (i0_dp.mul |  i0_dp.div |  i0_dp.lsu);  
         assign i1_mul_vp = (i1_dp.mul |  i1_dp.div |  i1_dp.lsu);


         assign mul_p.rs1_load_match_e1_i0 = rs1_load_match_e1_i0;
         assign mul_p.rs1_load_match_e1_i1 = rs1_load_match_e1_i1;

         assign mul_p.rs2_load_match_e1_i0 = rs2_load_match_e1_i0;
         assign mul_p.rs2_load_match_e1_i1 = rs2_load_match_e1_i1;  // 用 i1 在 e1 的预测结果

         
         /*
         // 用 rs1, rs2  的原因   使用的时候， 需要判断是 rs1, 还是 rs2  

                   //  匹配到 e1 阶段 i1   是否传递依赖值的信号
         assign rs1_load_match_e1_i1 = (i0_mul_vp & i0_rs1_class_d.load & i0_rs1_match_e1_i1) |
                                          (i1_mul_vp & i1_rs1_class_d.load & i1_rs1_match_e1_i1);  //dec i0, match 的 e1 包含 i1

         assign rs2_load_match_e1_i1 =  (i0_mul_vp & i0_rs2_class_d.load &   i0_rs2_match_e1_i1 & ~i0_dp.store) |
                                 (i1_mul_vp & i1_rs2_class_d.load &      i1_rs2_match_e1_i1 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i1


          //  匹配到 e1 阶段 i0              
          assign rs1_load_match_e1_i0 = (i0_mul_vp & i0_rs1_class_d.load & i0_rs1_match_e1_i0) |
                                       (i1_mul_vp & i1_rs1_class_d.load & i1_rs1_match_e1_i0);  // dec i0, match 的 e1 包含 i0                         

         
         assign rs2_load_match_e1_i0 =  (i0_mul_vp & i0_rs2_class_d.load &    i0_rs2_match_e1_i0 & ~i0_dp.store) |
                                 (i1_mul_vp & i1_rs2_class_d.load &      i1_rs2_match_e1_i0 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i0                       
                      
            
*/
         
            // 上面这个和下面这个效果一致
               
          // rs1 e1   rs2  e1

                   //  匹配到 e1 阶段 i1   是否传递依赖值的信号     有依赖,进行刷新
         assign rs1_load_match_e1_i1 = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e1_i1) |
                                          (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e1_i1);  //dec i0, match 的 e1 包含 i1

           //  匹配到 e1 阶段 i0              
          assign rs1_load_match_e1_i0 = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e1_i0) |
                                       (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e1_i0);  // dec i0, match 的 e1 包含 i0    

         assign rs2_load_match_e1_i1 =  (i0_not_alu_eff & i0_rs2_class_d.load &   i0_rs2_match_e1_i1 & ~i0_dp.store) |
                                 (i1_not_alu_eff & i1_rs2_class_d.load &      i1_rs2_match_e1_i1 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i1

              
         assign rs2_load_match_e1_i0 =  (i0_not_alu_eff & i0_rs2_class_d.load &    i0_rs2_match_e1_i0 & ~i0_dp.store) |
                                 (i1_not_alu_eff & i1_rs2_class_d.load &      i1_rs2_match_e1_i0 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i0                       
                    
              
         
          
          
         logic i0_load_match_e1_i0, i1_load_match_e1_i0;
         logic i0_load_match_e1_i1, i1_load_match_e1_i1;  

         logic i0_load_match_e2_i1, i0_load_match_e2_i0;
         logic i1_load_match_e2_i1, i1_load_match_e2_i0;


/*
       //  匹配到 e1 阶段 i0    dec 阶段的 i0, i1 依赖于 e1, e2 阶段的 i0 load              
       assign i0_load_match_e1_i0 = (~i0_inst_block_d & i0_rs1_class_d.load & i0_rs1_match_e1_i0) |
                                 (~i0_inst_block_d & i0_rs2_class_d.load &    i0_rs2_match_e1_i0 & ~i0_dp.store);  // dec i0, match 的 e1 包含 i0                         

      
      assign i1_load_match_e1_i0 = (~i1_inst_block_d & i1_rs1_class_d.load & i1_rs1_match_e1_i0) |
                              (~i1_inst_block_d & i1_rs2_class_d.load &      i1_rs2_match_e1_i0 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i0                       
                     
         // i0, i1在 e2 的依赖 load
             //  匹配到 e2 阶段 i0
      assign i0_load_match_e2_i0 = (~i0_inst_block_d & i0_rs1_class_d.load & i0_rs1_match_e2_i0 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) |
                                    (~i0_inst_block_d & i0_rs2_class_d.load &    i0_rs2_match_e2_i0 & ~i0_dp.store & ~i0_dp.mul);  //dec i0, match 的 e2 包含 i0                          

        
      assign i1_load_match_e2_i0 = (~i1_inst_block_d & i1_rs1_class_d.load & i1_rs1_match_e2_i0 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                                    (~i1_inst_block_d & i1_rs2_class_d.load &   i1_rs2_match_e2_i0 & ~i1_dp.store & ~i1_dp.mul);   //dec i1, match 的 e2 包含 i0



           //  匹配到 e1 阶段 i1      dec 阶段的 i0, i1 依赖于 e1, e2 阶段的 i1 load  
       assign i0_load_match_e1_i1 = (~i0_inst_block_d & i0_rs1_class_d.load & i0_rs1_match_e1_i1) |
                                 (~i0_inst_block_d & i0_rs2_class_d.load &   i0_rs2_match_e1_i1 & ~i0_dp.store);  //dec i0, match 的 e1 包含 i1

      assign i1_load_match_e1_i1 = (~i1_inst_block_d & i1_rs1_class_d.load & i1_rs1_match_e1_i1) |
                              (~i1_inst_block_d & i1_rs2_class_d.load &      i1_rs2_match_e1_i1 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i1


         //  匹配到 e2 阶段 i1
         assign i0_load_match_e2_i1 = (~i0_inst_block_d & i0_rs1_class_d.load & i0_rs1_match_e2_i1 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) |
                                    (~i0_inst_block_d & i0_rs2_class_d.load &   i0_rs2_match_e2_i1 & ~i0_dp.store & ~i0_dp.mul);  //dec i0, match 的 e2 包含 i1


         assign i1_load_match_e2_i1 = (~i1_inst_block_d & i1_rs1_class_d.load & i1_rs1_match_e2_i1 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                                    (~i1_inst_block_d & i1_rs2_class_d.load &   i1_rs2_match_e2_i1 & ~i1_dp.store & ~i1_dp.mul);  //dec i1, match 的 e2 包含 i1

*/
       
       
       
         //  匹配到 e1 阶段 i1
       assign i0_load_match_e1_i1 = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e1_i1) |
                                 (i0_not_alu_eff & i0_rs2_class_d.load &   i0_rs2_match_e1_i1 & ~i0_dp.store);  //dec i0, match 的 e1 包含 i1

      assign i1_load_match_e1_i1 = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e1_i1) |
                              (i1_not_alu_eff & i1_rs2_class_d.load &      i1_rs2_match_e1_i1 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i1


                    
       //  匹配到 e1 阶段 i0              
       assign i0_load_match_e1_i0 = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e1_i0) |
                                 (i0_not_alu_eff & i0_rs2_class_d.load &    i0_rs2_match_e1_i0 & ~i0_dp.store);  // dec i0, match 的 e1 包含 i0                         

      
      assign i1_load_match_e1_i0 = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e1_i0) |
                              (i1_not_alu_eff & i1_rs2_class_d.load &      i1_rs2_match_e1_i0 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i0                       
                     
      


         // i0, i1在 e2 的依赖 load

             //  匹配到 e2 阶段 i0
         assign i0_load_match_e2_i0 = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e2_i0 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) |
                                    (i0_not_alu_eff & i0_rs2_class_d.load &    i0_rs2_match_e2_i0 & ~i0_dp.store & ~i0_dp.mul);  //dec i0, match 的 e2 包含 i0                          

        
         assign i1_load_match_e2_i0 = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e2_i0 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                                    (i1_not_alu_eff & i1_rs2_class_d.load &   i1_rs2_match_e2_i0 & ~i1_dp.store & ~i1_dp.mul);   //dec i1, match 的 e2 包含 i0




         //  匹配到 e2 阶段 i1
         assign i0_load_match_e2_i1 = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e2_i1 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) |
                                    (i0_not_alu_eff & i0_rs2_class_d.load &   i0_rs2_match_e2_i1 & ~i0_dp.store & ~i0_dp.mul);  //dec i0, match 的 e2 包含 i1


         assign i1_load_match_e2_i1 = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e2_i1 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                                    (i1_not_alu_eff & i1_rs2_class_d.load &   i1_rs2_match_e2_i1 & ~i1_dp.store & ~i1_dp.mul);  //dec i1, match 的 e2 包含 i1

      




      /*
          //  匹配到 e1 阶段 i1   这种情况下,  值预测有效信号只有一个为 1
       assign i0_load_match_e1_i1 = (i0_not_alu_eff & ~i0_inst_block_d & i0_rs1_class_d.load & i0_rs1_match_e1_i1) |
                                 (i0_not_alu_eff & ~i0_inst_block_d & i0_rs2_class_d.load &   i0_rs2_match_e1_i1 & ~i0_dp.store);  //dec i0, match 的 e1 包含 i1

      assign i1_load_match_e1_i1 = (i1_not_alu_eff & ~i1_inst_block_d & i1_rs1_class_d.load & i1_rs1_match_e1_i1) |
                              (i1_not_alu_eff & ~i1_inst_block_d & i1_rs2_class_d.load &      i1_rs2_match_e1_i1 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i1


                    
       //  匹配到 e1 阶段 i0              
       assign i0_load_match_e1_i0 = (i0_not_alu_eff & ~i0_inst_block_d & i0_rs1_class_d.load & i0_rs1_match_e1_i0) |
                                 (i0_not_alu_eff & ~i0_inst_block_d & i0_rs2_class_d.load &    i0_rs2_match_e1_i0 & ~i0_dp.store);  // dec i0, match 的 e1 包含 i0                         

      
      assign i1_load_match_e1_i0 = (i1_not_alu_eff & ~i1_inst_block_d & i1_rs1_class_d.load & i1_rs1_match_e1_i0) |
                              (i1_not_alu_eff & ~i1_inst_block_d & i1_rs2_class_d.load &      i1_rs2_match_e1_i0 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i0                       
                     
      


         // i0, i1在 e2 的依赖 load

             //  匹配到 e2 阶段 i0
         assign i0_load_match_e2_i0 = (i0_not_alu_eff & ~i0_inst_block_d & i0_rs1_class_d.load & i0_rs1_match_e2_i0 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) |
                                    (i0_not_alu_eff & ~i0_inst_block_d & i0_rs2_class_d.load &    i0_rs2_match_e2_i0 & ~i0_dp.store & ~i0_dp.mul);  //dec i0, match 的 e2 包含 i0                          

        
         assign i1_load_match_e2_i0 = (i1_not_alu_eff & ~i1_inst_block_d & i1_rs1_class_d.load & i1_rs1_match_e2_i0 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                                    (i1_not_alu_eff & ~i1_inst_block_d & i1_rs2_class_d.load &   i1_rs2_match_e2_i0 & ~i1_dp.store & ~i1_dp.mul);   //dec i1, match 的 e2 包含 i0




         //  匹配到 e2 阶段 i1
         assign i0_load_match_e2_i1 = (i0_not_alu_eff & ~i0_inst_block_d & i0_rs1_class_d.load & i0_rs1_match_e2_i1 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) |
                                    (i0_not_alu_eff & ~i0_inst_block_d & i0_rs2_class_d.load &   i0_rs2_match_e2_i1 & ~i0_dp.store & ~i0_dp.mul);  //dec i0, match 的 e2 包含 i1


         assign i1_load_match_e2_i1 = (i1_not_alu_eff & ~i1_inst_block_d & i1_rs1_class_d.load & i1_rs1_match_e2_i1 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                                    (i1_not_alu_eff & ~i1_inst_block_d & i1_rs2_class_d.load &   i1_rs2_match_e2_i1 & ~i1_dp.store & ~i1_dp.mul);  //dec i1, match 的 e2 包含 i1

      
      */



	// i0_not_alu_eff：i0 不是一个有效的ALU指令

   // assign i0_not_alu_eff = (~i0_dp.alu | disable_secondary); // tag_disable_secondary: 2
   // assign i1_not_alu_eff = (~i1_dp.alu | disable_secondary);

   assign i0_not_alu_eff = (~i0_dp.alu |  disable_secondary); // tag_disable_secondary: 2
   assign i1_not_alu_eff = (~i1_dp.alu | disable_secondary);

    logic i0_inst_block_d, i1_inst_block_d;        

  

   // assign i0_inst_block_d = i0_dp.load;
   // assign i1_inst_block_d = i1_dp.load;


   // 遇到 load 和 jump,  并且识别到有依赖, 就进行阻塞
    assign i0_inst_block_d = i0_dp.load | i0_dp.condbr | i0_dp.beq | i0_dp.bne | i0_dp.bge | i0_dp.blt | i0_dp.jal | i0_dp.ecall | i0_dp.ebreak | i0_dp.mret; // tag_disable_secondary: 2
    assign i1_inst_block_d = i1_dp.load | i1_dp.condbr | i1_dp.beq | i1_dp.bne | i1_dp.bge | i1_dp.blt | i1_dp.jal | i1_dp.ecall | i1_dp.ebreak | i1_dp.mret;
    
    // assign i0_inst_block_d = i0_dp.condbr | i0_dp.beq | i0_dp.bne | i0_dp.bge | i0_dp.blt | i0_dp.jal | i0_dp.ecall | i0_dp.ebreak | i0_dp.mret; // tag_disable_secondary: 2
    // assign i1_inst_block_d =  i1_dp.condbr | i1_dp.beq | i1_dp.bne | i1_dp.bge | i1_dp.blt | i1_dp.jal | i1_dp.ecall | i1_dp.ebreak | i1_dp.mret;
    

   
   // 这里所说的是当前 dec 阶段的 i0 或 i1 指令 依赖于  e1 或 e2 阶段的 load 指令的结果
   assign i0_load_block_d = ( ( (i0_inst_block_d )  & i0_rs1_class_d.load & i0_rs1_match_e1) |  //  tag_i0_rs1_class_d: 2, tag_i0_i1_match: 2
                            ((i0_inst_block_d ) & i0_rs1_class_d.load & i0_rs1_match_e2 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) | // can bypass load to address of load/store

                            ((i0_inst_block_d ) & i0_rs2_class_d.load & i0_rs2_match_e1 & ~i0_dp.store) |
                            ((i0_inst_block_d ) & i0_rs2_class_d.load & i0_rs2_match_e2 & ~i0_dp.store & ~i0_dp.mul) ) & ~disable_secondary; // tag_i0_dp: 2 
						

   
   // 这里找到的是被阻塞的指令   是当前解码阶段的指令不发射
   assign i1_load_block_d = ( ( (i1_inst_block_d ) & i1_rs1_class_d.load & i1_rs1_match_e1) |
                            ( (i1_inst_block_d ) & i1_rs1_class_d.load & i1_rs1_match_e2 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                            ( (i1_inst_block_d ) & i1_rs2_class_d.load & i1_rs2_match_e1 & ~i1_dp.store) |
                            ( (i1_inst_block_d ) & i1_rs2_class_d.load & i1_rs2_match_e2 & ~i1_dp.store & ~i1_dp.mul) )  & ~disable_secondary;
   
 /*

   
   // 这里所说的是当前 dec 阶段的 i0 或 i1 指令 依赖于  e1 或 e2 阶段的 load 指令的结果
   assign i0_load_block_d = ( ( (i0_inst_block_d & i0_not_alu_eff)  & i0_rs1_class_d.load & i0_rs1_match_e1) |  //  tag_i0_rs1_class_d: 2, tag_i0_i1_match: 2
                            ((i0_inst_block_d & i0_not_alu_eff) & i0_rs1_class_d.load & i0_rs1_match_e2 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) | // can bypass load to address of load/store

                            ((i0_inst_block_d & i0_not_alu_eff) & i0_rs2_class_d.load & i0_rs2_match_e1 & ~i0_dp.store) |
                            ((i0_inst_block_d & i0_not_alu_eff) & i0_rs2_class_d.load & i0_rs2_match_e2 & ~i0_dp.store & ~i0_dp.mul) ) & ~disable_secondary; // tag_i0_dp: 2 
						

   // 这里找到的是被阻塞的指令   是当前解码阶段的指令不发射
   assign i1_load_block_d = ( ( (i1_inst_block_d & i1_not_alu_eff) & i1_rs1_class_d.load & i1_rs1_match_e1) |
                            ( (i1_inst_block_d & i1_not_alu_eff) & i1_rs1_class_d.load & i1_rs1_match_e2 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                            ( (i1_inst_block_d & i1_not_alu_eff) & i1_rs2_class_d.load & i1_rs2_match_e1 & ~i1_dp.store) |
                            ( (i1_inst_block_d & i1_not_alu_eff) & i1_rs2_class_d.load & i1_rs2_match_e2 & ~i1_dp.store & ~i1_dp.mul) )  & ~disable_secondary;

 
*/


   
// i0 i1 阻塞赋值   begin    tag_i0_i1_var_block: 1,

// stores will bypass load data in the lsu pipe        lsu: load store unit    i0 不是 alu 或 不是 mul
/*
   assign i0_load_block_d = ( ( (i0_not_alu_eff)  & i0_rs1_class_d.load & i0_rs1_match_e1) |  //  tag_i0_rs1_class_d: 2, tag_i0_i1_match: 2
                            ((i0_not_alu_eff ) & i0_rs1_class_d.load & i0_rs1_match_e2 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) | // can bypass load to address of load/store

                            ((i0_not_alu_eff ) & i0_rs2_class_d.load & i0_rs2_match_e1 & ~i0_dp.store) |
                            ((i0_not_alu_eff ) & i0_rs2_class_d.load & i0_rs2_match_e2 & ~i0_dp.store & ~i0_dp.mul) ) & ~disable_secondary; // tag_i0_dp: 2 
						

   // 这里找到的是被阻塞的指令   是当前解码阶段的指令不发射
   assign i1_load_block_d = ( ( (i1_not_alu_eff ) & i1_rs1_class_d.load & i1_rs1_match_e1) |
                            ( (i1_not_alu_eff ) & i1_rs1_class_d.load & i1_rs1_match_e2 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                            ( (i1_not_alu_eff ) & i1_rs2_class_d.load & i1_rs2_match_e1 & ~i1_dp.store) |
                            ( (i1_not_alu_eff ) & i1_rs2_class_d.load & i1_rs2_match_e2 & ~i1_dp.store & ~i1_dp.mul) )  & ~disable_secondary;

*/

      
         


   /*                         
//  e1d, e2d, e3d, e4d 变量的赋值   begin    tag_e_d: 1       整体的输入是 dd,     tag_dd: 2

// 临时中间变量: e1d_in, e2d_in, e3d_in, e4d_in
   
   rvdffe #( $bits(dest_pkt_t) ) e1ff (.*, .en(i0_e1_ctl_en), .din(dd),  .dout(e1d)); 

   always_comb begin
      e1d_in = e1d;

      e1d_in.i0v = e1d.i0v & ~flush_final_e3 & ~exu_i0_flush_vp_e4;  // flush_final_e3: flush final at e3: i0  or i1
      e1d_in.i1v = e1d.i1v & ~flush_final_e3 & ~exu_i0_flush_vp_e4;
      e1d_in.i0valid = e1d.i0valid & ~flush_final_e3 ;
      e1d_in.i1valid = e1d.i1valid & ~flush_final_e3 ;

      e1d_in.i0secondary = e1d.i0secondary & ~flush_final_e3;
      e1d_in.i1secondary = e1d.i1secondary & ~flush_final_e3;


      

      e1d_in.i0secblock = e1d.i0secblock & ~flush_final_e3 ;  // e1 阶段 i1
      e1d_in.i1secblock = e1d.i1secblock & ~flush_final_e3 ; 

      // 在 e1, e2 两个阶段进行筛选 后续对 load 有依赖的 load 指令
      // e1d_in.i0secload = e1d.i0secload & ~flush_final_e3 ;  // e1 阶段 i1
      // e1d_in.i1secload = e1d.i1secload & ~flush_final_e3 ;  // e1 存在匹配, 在 e2 判断没有依赖, 就没有刷新了

      
      e1d_in.i0secload = e1d.i0secload & ~flush_final_e3 & (i0_load_match_e1_i0 | i1_load_match_e1_i0);  // e1 阶段 i1
      e1d_in.i1secload = e1d.i1secload & ~flush_final_e3 & (i0_load_match_e1_i1 | i1_load_match_e1_i1);  // e1 存在匹配, 在 e2 判断没有依赖, 就没有刷新了
      
      // e1d_in.i0secload = e1d.i0secload & ~flush_final_e3 & (rs1_load_match_e1_i0 | rs2_load_match_e1_i0);  // e1 阶段 i1
      // e1d_in.i1secload = e1d.i1secload & ~flush_final_e3 & (rs1_load_match_e1_i1 | rs2_load_match_e1_i1);  // e1 存在匹配, 在 e2 判断没有依赖, 就没有刷新了
      

      

   end

   assign dec_i1_valid_e1 = e1d.i1valid;  // dec_i1_valid_e1: i1 valid e1


   rvdffe #( $bits(dest_pkt_t) ) e2ff (.*, .en(i0_e2_ctl_en), .din(e1d_in), .dout(e2d));   //  tag_answer  e1d_in 控制到下一拍输入

   always_comb begin
      e2d_in = e2d;

      e2d_in.i0v = e2d.i0v &         ~flush_final_e3 & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e2d_in.i1v = e2d.i1v &         ~flush_final_e3 & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e2d_in.i0valid = e2d.i0valid & ~flush_final_e3 & ~flush_lower_wb ;
      e2d_in.i1valid = e2d.i1valid & ~flush_final_e3 & ~flush_lower_wb ;

      e2d_in.i0secondary = e2d.i0secondary & ~flush_final_e3 & ~flush_lower_wb;
      e2d_in.i1secondary = e2d.i1secondary & ~flush_final_e3 & ~flush_lower_wb;



      e2d_in.i0secblock = e2d.i0secblock & ~flush_final_e3 & ~flush_lower_wb ;
      e2d_in.i1secblock = e2d.i1secblock & ~flush_final_e3 & ~flush_lower_wb ;
 



     e2d_in.i0secload = e2d.i0secload & ~flush_final_e3 & ~flush_lower_wb ;
     e2d_in.i1secload = e2d.i1secload & ~flush_final_e3 & ~flush_lower_wb ;  // 1


      
   end

   // tag_e3d: 1
   rvdffe #( $bits(dest_pkt_t) ) e3ff (.*, .en(i0_e3_ctl_en), .din(e2d_in), .dout(e3d));

   always_comb begin
      e3d_in = e3d;

      e3d_in.i0v = e3d.i0v                              & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e3d_in.i0valid = e3d.i0valid                      & ~flush_lower_wb ;

      e3d_in.i0secondary = e3d.i0secondary & ~flush_lower_wb;

      e3d_in.i0secload = e3d.i0secload & ~flush_lower_wb;


      e3d_in.i1v = e3d.i1v         & ~i0_flush_final_e3 & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e3d_in.i1valid = e3d.i1valid & ~i0_flush_final_e3 & ~flush_lower_wb ;

      e3d_in.i1secondary = e3d.i1secondary & ~i0_flush_final_e3 & ~flush_lower_wb;

      e3d_in.i1secload = e3d.i1secload & ~i0_flush_final_e3 & ~flush_lower_wb;




      e3d_in.i0secblock = e3d.i0secblock & ~flush_lower_wb;

      e3d_in.i1secblock = e3d.i1secblock & ~i0_flush_final_e3 & ~flush_lower_wb;

      if (freeze) e3d_in = '0;

   end


   assign dec_i0_sec_decode_e3 = e3d.i0secondary & ~flush_lower_wb & ~freeze; // dec_i0_sec_decode_e3: i0 secondary alu e3
   assign dec_i1_sec_decode_e3 = e3d.i1secondary & ~i0_flush_final_e3 & ~flush_lower_wb & ~freeze; // dec_i1_sec_decode_e3: i1 secondary alu e3, output, to exu.sv



   rvdffe #( $bits(dest_pkt_t) ) e4ff (.*, .en(i0_e4_ctl_en), .din(e3d_in), .dout(e4d));

   always_comb begin

      if (exu_div_finish)    // wipe data for exu_div_finish - bug where csr_wen was set for fast divide
        e4d_in = '0;
      else
        e4d_in = e4d;


      e4d_in.i0rd[4:0] = (exu_div_finish) ? div_waddr_wb[4:0] : e4d.i0rd[4:0];

      e4d_in.i0v = ((e4d.i0v         & ~e4d.i0div & ~flush_lower_wb) | (exu_div_finish & div_waddr_wb[4:0]!=5'b0) ) & ~exu_i0_flush_vp_e4;
      e4d_in.i0valid = ((e4d.i0valid              & ~flush_lower_wb) | exu_div_finish) ;
      // qual the following with div finish; necessary for divides with early exit
      e4d_in.i0secondary = e4d.i0secondary & ~flush_lower_wb & ~exu_div_finish;

      e4d_in.i0load = e4d.i0load & ~flush_lower_wb & ~exu_div_finish;
      e4d_in.i0store = e4d.i0store & ~flush_lower_wb & ~exu_div_finish;


      e4d_in.i1v = e4d.i1v         & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e4d_in.i1valid = e4d.i1valid & ~flush_lower_wb ;
      
      e4d_in.i1secondary = e3d.i1secondary & ~flush_lower_wb;

   end


   rvdffe #( $bits(dest_pkt_t) ) wbff (.*, .en(i0_wb_ctl_en | exu_div_finish | div_wen_wb), .din(e4d_in), .dout(wbd));  // wbd：write back decode   tag_answer  
     
//  e1d, e2d, e3d, e4d, wbd 变量的赋值   end

*/
  
  /*

  //  e1d, e2d, e3d, e4d 变量的赋值   begin    tag_e_d: 1       整体的输入是 dd,     tag_dd: 2

// 临时中间变量: e1d_in, e2d_in, e3d_in, e4d_in
   
   rvdffe #( $bits(dest_pkt_t) ) e1ff (.*, .en(i0_e1_ctl_en), .din(dd),  .dout(e1d)); 

   always_comb begin
      e1d_in = e1d;

      e1d_in.i0v = e1d.i0v & ~flush_final_e3 & ~exu_i0_flush_vp_e4;  // flush_final_e3: flush final at e3: i0  or i1
      e1d_in.i1v = e1d.i1v & ~flush_final_e3 & ~exu_i0_flush_vp_e4;
      e1d_in.i0valid = e1d.i0valid & ~flush_final_e3 & ~exu_i0_flush_vp_e4;
      e1d_in.i1valid = e1d.i1valid & ~flush_final_e3 & ~exu_i0_flush_vp_e4;

      e1d_in.i0secondary = e1d.i0secondary & ~flush_final_e3;
      e1d_in.i1secondary = e1d.i1secondary & ~flush_final_e3;


      

      e1d_in.i0secblock = e1d.i0secblock & ~flush_final_e3 ;  // e1 阶段 i1
      e1d_in.i1secblock = e1d.i1secblock & ~flush_final_e3 ; 

      // 在 e1, e2 两个阶段进行筛选 后续对 load 有依赖的 load 指令
      // e1d_in.i0secload = e1d.i0secload & ~flush_final_e3 ;  // e1 阶段 i1
      // e1d_in.i1secload = e1d.i1secload & ~flush_final_e3 ;  // e1 存在匹配, 在 e2 判断没有依赖, 就没有刷新了

      
      e1d_in.i0secload = e1d.i0secload & ~flush_final_e3 & (i0_load_match_e1_i0 | i1_load_match_e1_i0);  // e1 阶段 i1
      e1d_in.i1secload = e1d.i1secload & ~flush_final_e3 & (i0_load_match_e1_i1 | i1_load_match_e1_i1);  // e1 存在匹配, 在 e2 判断没有依赖, 就没有刷新了
      
      // e1d_in.i0secload = e1d.i0secload & ~flush_final_e3 & (rs1_load_match_e1_i0 | rs2_load_match_e1_i0);  // e1 阶段 i1
      // e1d_in.i1secload = e1d.i1secload & ~flush_final_e3 & (rs1_load_match_e1_i1 | rs2_load_match_e1_i1);  // e1 存在匹配, 在 e2 判断没有依赖, 就没有刷新了
      

      

   end

   assign dec_i1_valid_e1 = e1d.i1valid;  // dec_i1_valid_e1: i1 valid e1


   rvdffe #( $bits(dest_pkt_t) ) e2ff (.*, .en(i0_e2_ctl_en), .din(e1d_in), .dout(e2d));   //  tag_answer  e1d_in 控制到下一拍输入

   always_comb begin
      e2d_in = e2d;

      e2d_in.i0v = e2d.i0v &         ~flush_final_e3 & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e2d_in.i1v = e2d.i1v &         ~flush_final_e3 & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e2d_in.i0valid = e2d.i0valid & ~flush_final_e3 & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e2d_in.i1valid = e2d.i1valid & ~flush_final_e3 & ~flush_lower_wb & ~exu_i0_flush_vp_e4;

      e2d_in.i0secondary = e2d.i0secondary & ~flush_final_e3 & ~flush_lower_wb;
      e2d_in.i1secondary = e2d.i1secondary & ~flush_final_e3 & ~flush_lower_wb;



      e2d_in.i0secblock = e2d.i0secblock & ~flush_final_e3 & ~flush_lower_wb ;
      e2d_in.i1secblock = e2d.i1secblock & ~flush_final_e3 & ~flush_lower_wb ;
 



     e2d_in.i0secload = e2d.i0secload & ~flush_final_e3 & ~flush_lower_wb ;
     e2d_in.i1secload = e2d.i1secload & ~flush_final_e3 & ~flush_lower_wb ;  // 1




   // e2d_in.i0secload = e2d.i0secload & ~flush_final_e3 & ~flush_lower_wb & ((i0_load_match_e2_i0 | i1_load_match_e2_i0)  | e2d.i0secload);
   //  e2d_in.i1secload = e2d.i1secload & ~flush_final_e3 & ~flush_lower_wb &  ((i0_load_match_e2_i1 | i1_load_match_e2_i1) | e2d.i1secload);  // 2   1 和 2 等价


   //e2d_in.i0secload = e2d.i0secload & ~flush_final_e3 & ~flush_lower_wb & (i0_load_match_e2_i0 | i1_load_match_e2_i0);
  //  e2d_in.i1secload = e2d.i1secload & ~flush_final_e3 & ~flush_lower_wb &  (i0_load_match_e2_i1 | i1_load_match_e2_i1);

   //  e2d_in.i0secload = ( e2d.i0secload & ~flush_final_e3 & ~flush_lower_wb & (i0_load_match_e2_i0 | i1_load_match_e2_i0) )  | e2d.i0secload;
   // e2d_in.i1secload = ( e2d.i1secload & ~flush_final_e3 & ~flush_lower_wb & (i0_load_match_e2_i1 | i1_load_match_e2_i1) ) | e2d.i1secload;


      
   end

   // tag_e3d: 1
   rvdffe #( $bits(dest_pkt_t) ) e3ff (.*, .en(i0_e3_ctl_en), .din(e2d_in), .dout(e3d));

   always_comb begin
      e3d_in = e3d;

      e3d_in.i0v = e3d.i0v                              & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e3d_in.i0valid = e3d.i0valid                      & ~flush_lower_wb & ~exu_i0_flush_vp_e4;

      e3d_in.i0secondary = e3d.i0secondary & ~flush_lower_wb;

      e3d_in.i0secload = e3d.i0secload & ~flush_lower_wb;


      e3d_in.i1v = e3d.i1v         & ~i0_flush_final_e3 & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e3d_in.i1valid = e3d.i1valid & ~i0_flush_final_e3 & ~flush_lower_wb & ~exu_i0_flush_vp_e4;

      e3d_in.i1secondary = e3d.i1secondary & ~i0_flush_final_e3 & ~flush_lower_wb;

      e3d_in.i1secload = e3d.i1secload & ~i0_flush_final_e3 & ~flush_lower_wb;




      e3d_in.i0secblock = e3d.i0secblock & ~flush_lower_wb;

      e3d_in.i1secblock = e3d.i1secblock & ~i0_flush_final_e3 & ~flush_lower_wb;

      if (freeze) e3d_in = '0;

   end


   assign dec_i0_sec_decode_e3 = e3d.i0secondary & ~flush_lower_wb & ~freeze; // dec_i0_sec_decode_e3: i0 secondary alu e3
   assign dec_i1_sec_decode_e3 = e3d.i1secondary & ~i0_flush_final_e3 & ~flush_lower_wb & ~freeze; // dec_i1_sec_decode_e3: i1 secondary alu e3, output, to exu.sv



   rvdffe #( $bits(dest_pkt_t) ) e4ff (.*, .en(i0_e4_ctl_en), .din(e3d_in), .dout(e4d));

   always_comb begin

      if (exu_div_finish)    // wipe data for exu_div_finish - bug where csr_wen was set for fast divide
        e4d_in = '0;
      else
        e4d_in = e4d;


      e4d_in.i0rd[4:0] = (exu_div_finish) ? div_waddr_wb[4:0] : e4d.i0rd[4:0];

      e4d_in.i0v = ((e4d.i0v         & ~e4d.i0div & ~flush_lower_wb) | (exu_div_finish & div_waddr_wb[4:0]!=5'b0) ) & ~exu_i0_flush_vp_e4;
      e4d_in.i0valid = ((e4d.i0valid              & ~flush_lower_wb) | exu_div_finish) & ~exu_i0_flush_vp_e4;
      // qual the following with div finish; necessary for divides with early exit
      e4d_in.i0secondary = e4d.i0secondary & ~flush_lower_wb & ~exu_div_finish;

      e4d_in.i0load = e4d.i0load & ~flush_lower_wb & ~exu_div_finish;
      e4d_in.i0store = e4d.i0store & ~flush_lower_wb & ~exu_div_finish;


      e4d_in.i1v = e4d.i1v         & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      e4d_in.i1valid = e4d.i1valid & ~flush_lower_wb & ~exu_i0_flush_vp_e4;
      
      e4d_in.i1secondary = e3d.i1secondary & ~flush_lower_wb;

   end


   rvdffe #( $bits(dest_pkt_t) ) wbff (.*, .en(i0_wb_ctl_en | exu_div_finish | div_wen_wb), .din(e4d_in), .dout(wbd));  // wbd：write back decode   tag_answer  
     
//  e1d, e2d, e3d, e4d, wbd 变量的赋值   end

*/




//  e1d, e2d, e3d, e4d 变量的赋值   begin    tag_e_d: 1       整体的输入是 dd,     tag_dd: 2

// 临时中间变量: e1d_in, e2d_in, e3d_in, e4d_in
   
   rvdffe #( $bits(dest_pkt_t) ) e1ff (.*, .en(i0_e1_ctl_en), .din(dd),  .dout(e1d)); 

   always_comb begin
      e1d_in = e1d;

      e1d_in.i0v = e1d.i0v & ~flush_final_e3;  // flush_final_e3: flush final at e3: i0  or i1
      e1d_in.i1v = e1d.i1v & ~flush_final_e3;
      e1d_in.i0valid = e1d.i0valid & ~flush_final_e3;
      e1d_in.i1valid = e1d.i1valid & ~flush_final_e3;

      e1d_in.i0secondary = e1d.i0secondary & ~flush_final_e3;
      e1d_in.i1secondary = e1d.i1secondary & ~flush_final_e3;


      

      e1d_in.i0secblock = e1d.i0secblock & ~flush_final_e3 ;  // e1 阶段 i1
      e1d_in.i1secblock = e1d.i1secblock & ~flush_final_e3 ; 

      // 在 e1, e2 两个阶段进行筛选 后续对 load 有依赖的 load 指令
      // e1d_in.i0secload = e1d.i0secload & ~flush_final_e3 ;  // e1 阶段 i1
      // e1d_in.i1secload = e1d.i1secload & ~flush_final_e3 ;  // e1 存在匹配, 在 e2 判断没有依赖, 就没有刷新了

      
      e1d_in.i0secload = e1d.i0secload & ~flush_final_e3 & (i0_load_match_e1_i0 | i1_load_match_e1_i0);  // e1 阶段 i1
      e1d_in.i1secload = e1d.i1secload & ~flush_final_e3 & (i0_load_match_e1_i1 | i1_load_match_e1_i1);  // e1 存在匹配, 在 e2 判断没有依赖, 就没有刷新了
      
      // e1d_in.i0secload = e1d.i0secload & ~flush_final_e3 & (rs1_load_match_e1_i0 | rs2_load_match_e1_i0);  // e1 阶段 i1
      // e1d_in.i1secload = e1d.i1secload & ~flush_final_e3 & (rs1_load_match_e1_i1 | rs2_load_match_e1_i1);  // e1 存在匹配, 在 e2 判断没有依赖, 就没有刷新了
      

      

   end

   assign dec_i1_valid_e1 = e1d.i1valid;  // dec_i1_valid_e1: i1 valid e1


   rvdffe #( $bits(dest_pkt_t) ) e2ff (.*, .en(i0_e2_ctl_en), .din(e1d_in), .dout(e2d));   //  tag_answer  e1d_in 控制到下一拍输入

   always_comb begin
      e2d_in = e2d;

      e2d_in.i0v = e2d.i0v &         ~flush_final_e3 & ~flush_lower_wb;
      e2d_in.i1v = e2d.i1v &         ~flush_final_e3 & ~flush_lower_wb;
      e2d_in.i0valid = e2d.i0valid & ~flush_final_e3 & ~flush_lower_wb;
      e2d_in.i1valid = e2d.i1valid & ~flush_final_e3 & ~flush_lower_wb;

      e2d_in.i0secondary = e2d.i0secondary & ~flush_final_e3 & ~flush_lower_wb;
      e2d_in.i1secondary = e2d.i1secondary & ~flush_final_e3 & ~flush_lower_wb;



      e2d_in.i0secblock = e2d.i0secblock & ~flush_final_e3 & ~flush_lower_wb ;
      e2d_in.i1secblock = e2d.i1secblock & ~flush_final_e3 & ~flush_lower_wb ;
 



     e2d_in.i0secload = e2d.i0secload & ~flush_final_e3 & ~flush_lower_wb ;
     e2d_in.i1secload = e2d.i1secload & ~flush_final_e3 & ~flush_lower_wb ;  // 1




   // e2d_in.i0secload = e2d.i0secload & ~flush_final_e3 & ~flush_lower_wb & ((i0_load_match_e2_i0 | i1_load_match_e2_i0)  | e2d.i0secload);
   //  e2d_in.i1secload = e2d.i1secload & ~flush_final_e3 & ~flush_lower_wb &  ((i0_load_match_e2_i1 | i1_load_match_e2_i1) | e2d.i1secload);  // 2   1 和 2 等价


   //e2d_in.i0secload = e2d.i0secload & ~flush_final_e3 & ~flush_lower_wb & (i0_load_match_e2_i0 | i1_load_match_e2_i0);
  //  e2d_in.i1secload = e2d.i1secload & ~flush_final_e3 & ~flush_lower_wb &  (i0_load_match_e2_i1 | i1_load_match_e2_i1);

   //  e2d_in.i0secload = ( e2d.i0secload & ~flush_final_e3 & ~flush_lower_wb & (i0_load_match_e2_i0 | i1_load_match_e2_i0) )  | e2d.i0secload;
   // e2d_in.i1secload = ( e2d.i1secload & ~flush_final_e3 & ~flush_lower_wb & (i0_load_match_e2_i1 | i1_load_match_e2_i1) ) | e2d.i1secload;


      
   end

   // tag_e3d: 1
   rvdffe #( $bits(dest_pkt_t) ) e3ff (.*, .en(i0_e3_ctl_en), .din(e2d_in), .dout(e3d));

   always_comb begin
      e3d_in = e3d;

      e3d_in.i0v = e3d.i0v                              & ~flush_lower_wb;
      e3d_in.i0valid = e3d.i0valid                      & ~flush_lower_wb;

      e3d_in.i0secondary = e3d.i0secondary & ~flush_lower_wb;

      e3d_in.i0secload = e3d.i0secload & ~flush_lower_wb;


      e3d_in.i1v = e3d.i1v         & ~i0_flush_final_e3 & ~flush_lower_wb;
      e3d_in.i1valid = e3d.i1valid & ~i0_flush_final_e3 & ~flush_lower_wb;

      e3d_in.i1secondary = e3d.i1secondary & ~i0_flush_final_e3 & ~flush_lower_wb;

      e3d_in.i1secload = e3d.i1secload & ~i0_flush_final_e3 & ~flush_lower_wb;




      e3d_in.i0secblock = e3d.i0secblock & ~flush_lower_wb;

      e3d_in.i1secblock = e3d.i1secblock & ~i0_flush_final_e3 & ~flush_lower_wb;

      if (freeze) e3d_in = '0;

   end


   assign dec_i0_sec_decode_e3 = e3d.i0secondary & ~flush_lower_wb & ~freeze; // dec_i0_sec_decode_e3: i0 secondary alu e3
   assign dec_i1_sec_decode_e3 = e3d.i1secondary & ~i0_flush_final_e3 & ~flush_lower_wb & ~freeze; // dec_i1_sec_decode_e3: i1 secondary alu e3, output, to exu.sv



   rvdffe #( $bits(dest_pkt_t) ) e4ff (.*, .en(i0_e4_ctl_en), .din(e3d_in), .dout(e4d));

   always_comb begin

      if (exu_div_finish)    // wipe data for exu_div_finish - bug where csr_wen was set for fast divide
        e4d_in = '0;
      else
        e4d_in = e4d;


      e4d_in.i0rd[4:0] = (exu_div_finish) ? div_waddr_wb[4:0] : e4d.i0rd[4:0];

      e4d_in.i0v = (e4d.i0v         & ~e4d.i0div & ~flush_lower_wb) | (exu_div_finish & div_waddr_wb[4:0]!=5'b0);
      e4d_in.i0valid = (e4d.i0valid              & ~flush_lower_wb) | exu_div_finish;
      // qual the following with div finish; necessary for divides with early exit
      e4d_in.i0secondary = e4d.i0secondary & ~flush_lower_wb & ~exu_div_finish;

      e4d_in.i0load = e4d.i0load & ~flush_lower_wb & ~exu_div_finish;
      e4d_in.i0store = e4d.i0store & ~flush_lower_wb & ~exu_div_finish;


      e4d_in.i1v = e4d.i1v         & ~flush_lower_wb;
      e4d_in.i1valid = e4d.i1valid & ~flush_lower_wb;
      e4d_in.i1secondary = e3d.i1secondary & ~flush_lower_wb;

   end


   rvdffe #( $bits(dest_pkt_t) ) wbff (.*, .en(i0_wb_ctl_en | exu_div_finish | div_wen_wb), .din(e4d_in), .dout(wbd));  // wbd：write back decode   tag_answer  
     
//  e1d, e2d, e3d, e4d, wbd 变量的赋值   end
   

   
   


   // 只刷新依赖指令 及 后须指令   
   assign i0_valid_e3 = e3d.i0secload  & ~flush_lower_wb & ~freeze;  // e3 阶段的 i0 指令是一条 load 指令
   assign i1_valid_e3 = e3d.i1secload & ~i0_flush_final_e3 & ~flush_lower_wb & ~freeze; //   e3 阶段的 i1 指令是一条 load 指令

   // logic i0_block_e2, i1_block_e2;

   // assign i0_block_e2 = e3d.i0secblock  & ~flush_lower_wb & ~freeze;    // 刷新哪些依赖的指令 及 后面指令
   // assign i1_block_e2 = e3d.i1secblock & ~i0_flush_final_e3 & ~flush_lower_wb & ~freeze; 



   dec_vp_flush_ctl i0_dec_vp_e4 (.*,
                        .freeze                ( 1'b0                        ),   // I   
                         // .freeze                ( freeze                        ),   // I   
                          .enable                ( i0_e4_ctl_en                ),   // I  

                          // .valid                 ( i0_block_e2             ),   // I     

                          .valid                 ( i0_valid_e3             ), 

                          
                          .flush                 ( dec_tlu_flush_lower_wb      ),   // I   

                          .predicted_result      ( i0_predicted_result_e4_raw[31:0]     ),   // I

                          // .predicted_result      ( lsu_result_dc3[31:0]     ),   // I   


                          .actual_result         ( i0_result_e4_final[31:0]             ),   // I

                          .pred_path           ( npc_e4[31:1]                ),   // I  // 分支预测的地址, 无论正确与否

                          .any_jal_flush_path    ( i0_value_flush_e4[31:1]     ),   // I   any_jal

                          .any_jal               ( i0_is_jal_e4                ),   // I       
                          

                          .inst_type             ( 1'b1                        ),   // I     

                         
                                                                                                                    
                          .flush_upper_vp        ( exu_i0_flush_vp_e4          ),   // O 
						                   
                          .flush_path_vp         ( exu_i0_flush_path_vp_e4[31:1]         ),   // O

                          .predicted_result_new  ( i0_predicted_result_e4[31:0]          )   // O    

                          );



   dec_vp_flush_ctl i1_dec_vp_e4 (.*,
                          .freeze                ( 1'b0                        ),   // I   
                         //  .freeze                ( freeze                        ),   // I   
                          .enable                ( i1_e4_ctl_en                ),   // I  

                          // .valid                 ( i1_block_e2             ),   // I  

                          // .valid                 ( 0             ), 

                          .valid                 ( i1_valid_e3             ), 

                     
                         .flush                 ( dec_tlu_flush_lower_wb      ),   // I 

                          .predicted_result      ( i1_predicted_result_e4_raw[31:0]     ),   // I   

                          // .predicted_result      ( lsu_result_dc3[31:0]     ),   // I   


                          .actual_result         ( i1_result_e4_final[31:0]             ),   // I

                          .pred_path             ( npc_e4[31:1]                ),   // I  // 分支预测的地址, 无论正确与否

                          .any_jal_flush_path    ( i1_value_flush_e4[31:1]     ),   // I   any_jal

                          .any_jal               ( i1_is_jal_e4                ),   // I       
                          

                          .inst_type             ( 1'b1                        ),   // I     
                                                                                                                             
                          .flush_upper_vp        ( exu_i1_flush_vp_e4          ),   // O 
						                   
                          .flush_path_vp         ( exu_i1_flush_path_vp_e4[31:1]         ),   // O

                          .predicted_result_new  ( i1_predicted_result_e4[31:0]          )   // O    

                          );                       





   logic [31 : 1] i0_test_e1, i1_test_e1;

   rvdffe #(32) i0e1pcff (.*, .en(i0_e1_data_en), .din( {dec_i0_pc_d[31:1], i0_prediction_valid_d} ), .dout({i0_test_e1[31:1], i0_prediction_valid_e1}));  

   rvdffe #(32) i0e2pcff (.*, .en(i0_e2_data_en), .din( {i0_pc_e1[31:1], i0_prediction_valid_e1} ), .dout({i0_pc_e2[31:1], i0_prediction_valid_e2}));
   rvdffe #(32) i0e3pcff (.*, .en(i0_e3_data_en), .din( {i0_pc_e2[31:1], i0_prediction_valid_e2}), .dout( {i0_pc_e3[31:1], i0_prediction_valid_e3}));
   rvdffe #(32) i0e4pcff (.*, .en(i0_e4_data_en), .din( {i0_pc_e3[31:1], i0_prediction_valid_e3}), .dout( {i0_pc_e4[31:1], i0_prediction_valid_e4})); // 上一级流水线阶段 pc 的输出 是 下一级流水线阶段 pc 的输入 
   
   rvdffe #(32) i1e1pcff (.*, .en(i1_e1_data_en), .din( {dec_i1_pc_d[31:1], i1_prediction_valid_d} ), .dout({i1_test_e1[31:1], i1_prediction_valid_e1}));

   rvdffe #(32) i1e2pcff (.*, .en(i1_e2_data_en), .din( {i1_pc_e1[31:1], i1_prediction_valid_e1} ), .dout({i1_pc_e2[31:1], i1_prediction_valid_e2}));
   rvdffe #(32) i1e3pcff (.*, .en(i1_e3_data_en), .din( {i1_pc_e2[31:1], i1_prediction_valid_e2}), .dout( {i1_pc_e3[31:1], i1_prediction_valid_e3}));
   rvdffe #(32) i1e4pcff (.*, .en(i1_e4_data_en), .din( {i1_pc_e3[31:1], i1_prediction_valid_e3}), .dout( {i1_pc_e4[31:1], i1_prediction_valid_e4}));
   


   logic [31 : 0] i0_last_value;

   logic [31 : 0] i0_predicted_value_d;

   logic i0_prediction_valid_d;

   logic [31 : 0] i1_last_value;


   logic [31 : 0] i1_predicted_value_d;

   logic i1_prediction_valid_d;



   dec_predictor_ctl i0_dec_predictor (.*,
                            .pc_d                ( dec_i0_pc_d[31:1]                 ),   // I   
                          
                            .predict_en                ( 1'b1                ),   // I  
  
                            .prediction_valid_d                 ( i0_prediction_valid_d             ),   //   O                       
                            .prediction_value_d                 ( i0_predicted_value_d[31:0]       ),   // O

  
                            .pc_e4      ( i0_pc_e4[31:1]     ),   // I
  
                            .update_en         ( exu_i0_flush_vp_e4          ),   // I   刷新信号, 刷新了, 就需要更新预测器   
  
                            .prediction_valid_e4           ( i0_prediction_valid_e4                ),   // I 
  
                            .actual_value    ( i0_result_e4_final[31:0]     )   // I   any_jal
  
                            // .last_value               ( i0_is_jal_e4                )  // I       
                             
                            );
  
  
  

   dec_predictor_ctl i1_dec_predictor (.*,
                            .pc_d                ( dec_i1_pc_d[31:1]                 ),   // I   
                          
                            .predict_en                ( 1'b1                ),   // I  
  
                            .prediction_valid_d                 ( i1_prediction_valid_d             ),   //   O                       
                            .prediction_value_d                 ( i1_predicted_value_d[31:0]       ),   // O

  
                            .pc_e4      ( i1_pc_e4[31:1]     ),   // I
  
                            .update_en         ( exu_i1_flush_vp_e4          ),   // I
  
                            .prediction_valid_e4           ( i1_prediction_valid_e4                ),   // I  // 分支预测的地址, 无论正确与否
  
                            .actual_value    ( i1_result_e4_final[31:0]     )   // I   any_jal
  
                            // .last_value               ( i0_is_jal_e4                )  // I       
                             
                            );
   






                

                                       
   /*                    
   alu_pkt_t i0_ap_e1, i0_ap_e2, i0_ap_e3, i0_ap_e4;
   alu_pkt_t i1_ap_e1, i1_ap_e2, i1_ap_e3, i1_ap_e4;

   rvdffe #($bits(alu_pkt_t)) i0_ap_e1_ff (.*,  .en(i0_e1_ctl_en), .din(i0_ap),   .dout(i0_ap_e1) );
   rvdffe #($bits(alu_pkt_t)) i0_ap_e2_ff (.*,  .en(i0_e2_ctl_en), .din(i0_ap_e1),.dout(i0_ap_e2) );
   rvdffe #($bits(alu_pkt_t)) i0_ap_e3_ff (.*,  .en(i0_e3_ctl_en), .din(i0_ap_e2),.dout(i0_ap_e3) );
   rvdffe #($bits(alu_pkt_t)) i0_ap_e4_ff (.*,  .en(i0_e4_ctl_en), .din(i0_ap_e3),.dout(i0_ap_e4) );


   rvdffe #($bits(alu_pkt_t)) i1_ap_e1_ff (.*,  .en(i1_e1_ctl_en), .din(i1_ap),   .dout(i1_ap_e1) );
   rvdffe #($bits(alu_pkt_t)) i1_ap_e2_ff (.*,  .en(i1_e2_ctl_en), .din(i1_ap_e1),.dout(i1_ap_e2) );
   rvdffe #($bits(alu_pkt_t)) i1_ap_e3_ff (.*,  .en(i1_e3_ctl_en), .din(i1_ap_e2),.dout(i1_ap_e3) );
   rvdffe #($bits(alu_pkt_t)) i1_ap_e4_ff (.*,  .en(i1_e4_ctl_en), .din(i1_ap_e3),.dout(i1_ap_e4) );
   */

   
   rvdffe #(32) i0wbpredictedresultff (.*, .en(i0_wb_data_en), .din(i0_predicted_result_e4[31:0]), .dout(i0_predicted_result_wb[31:0])); // 使用更新后的预测值来更新值预测器
   rvdffe #(32) i1wbpredictedresultff (.*, .en(i1_wb_data_en), .din(i1_predicted_result_e4[31:0]), .dout(i1_predicted_result_wb[31:0]));
   

   rvdffe #(12) e1brpcff (.*, .en(i0_e1_data_en), .din(last_br_immed_d[12:1] ), .dout(last_br_immed_e1[12:1]));
   rvdffe #(12) e2brpcff (.*, .en(i0_e2_data_en), .din(last_br_immed_e1[12:1]), .dout(last_br_immed_e2[12:1]));  // tag_last_br_immed_e2: 1



// trace stuff

   rvdffe #(32) divinstff   (.*, .en(i0_div_decode_d), .din(i0_inst_d[31:0]), .dout(div_inst[31:0]));

   assign i0_inst_d[31:0] = (dec_i0_pc4_d) ? i0[31:0] : {16'b0, dec_i0_cinst_d[15:0] };

   rvdffe #(32) i0e1instff  (.*, .en(i0_e1_data_en), .din(i0_inst_d[31:0]),  .dout(i0_inst_e1[31:0]));
   rvdffe #(32) i0e2instff  (.*, .en(i0_e2_data_en), .din(i0_inst_e1[31:0]), .dout(i0_inst_e2[31:0]));
   rvdffe #(32) i0e3instff  (.*, .en(i0_e3_data_en), .din(i0_inst_e2[31:0]), .dout(i0_inst_e3[31:0]));
   rvdffe #(32) i0e4instff  (.*, .en(i0_e4_data_en), .din(i0_inst_e3[31:0]), .dout(i0_inst_e4[31:0]));
   rvdffe #(32) i0wbinstff  (.*, .en(i0_wb_data_en | exu_div_finish), .din( (exu_div_finish) ? div_inst[31:0] : i0_inst_e4[31:0]), .dout(i0_inst_wb[31:0]));
   rvdffe #(32) i0wb1instff (.*, .en(i0_wb1_data_en | div_wen_wb),    .din(i0_inst_wb[31:0]),                                      .dout(i0_inst_wb1[31:0]));

   assign i1_inst_d[31:0] = (dec_i1_pc4_d) ? i1[31:0] : {16'b0, dec_i1_cinst_d[15:0] };

   rvdffe #(32) i1e1instff  (.*, .en(i1_e1_data_en), .din(i1_inst_d[31:0]),  .dout(i1_inst_e1[31:0]));
   rvdffe #(32) i1e2instff  (.*, .en(i1_e2_data_en), .din(i1_inst_e1[31:0]), .dout(i1_inst_e2[31:0]));
   rvdffe #(32) i1e3instff  (.*, .en(i1_e3_data_en), .din(i1_inst_e2[31:0]), .dout(i1_inst_e3[31:0]));
   rvdffe #(32) i1e4instff  (.*, .en(i1_e4_data_en), .din(i1_inst_e3[31:0]), .dout(i1_inst_e4[31:0]));
   rvdffe #(32) i1wbinstff  (.*, .en(i1_wb_data_en), .din(i1_inst_e4[31:0]), .dout(i1_inst_wb[31:0]));
   rvdffe #(32) i1wb1instff (.*, .en(i1_wb1_data_en),.din(i1_inst_wb[31:0]), .dout(i1_inst_wb1[31:0]));

   assign dec_i0_inst_wb1[31:0] = i0_inst_wb1[31:0]; // dec_i0_inst_wb1: 32b instruction at wb+1 for trace encoder
   assign dec_i1_inst_wb1[31:0] = i1_inst_wb1[31:0];


   rvdffe #(31) i0wbpcff  (.*, .en(i0_wb_data_en | exu_div_finish), .din(dec_tlu_i0_pc_e4[31:1]), .dout(i0_pc_wb[31:1]));
   rvdffe #(31) i0wb1pcff (.*, .en(i0_wb1_data_en | div_wen_wb),    .din(i0_pc_wb[31:1]),         .dout(i0_pc_wb1[31:1]));

   rvdffe #(31) i1wb1pcff (.*, .en(i1_wb1_data_en),.din(i1_pc_wb[31:1]),         .dout(i1_pc_wb1[31:1]));

   assign dec_i0_pc_wb1[31:1] = i0_pc_wb1[31:1];
   assign dec_i1_pc_wb1[31:1] = i1_pc_wb1[31:1];


   
   
   
   
   
// i0_pc_e1, i1_pc_e1;  i0_pc_e2, i1_pc_e2;   i0_pc_e3, i1_pc_e3;   i0_pc_e4, i1_pc_e4; 变量的赋值    begin
   
   
   // pipe the pc's down the pipe 在处理器的流水线（pipeline）中传递程序计数器（PC）的值。
   assign i0_pc_e1[31:1] = exu_i0_pc_e1[31:1]; // i0_pc_e1[31:1] 和 i1_pc_e1[31:1]：表示当前流水线阶段 E1 的程序计数器值。
   assign i1_pc_e1[31:1] = exu_i1_pc_e1[31:1]; // exu_i0_pc_e1[31:1] 和 exu_i1_pc_e1[31:1]：表示上一级流水线阶段（如执行单元（EXU）阶段）的PC值。

   /*
			使用了多个 rvdffe 模块来实现带有使能信号的D触发器（Flip-Flop），用于在处理器的流水线中存储和传递程序计数器（PC）的值。
			每个 rvdffe 实例将PC值从一个流水线阶段传递到下一个阶段，以确保每条指令的PC值能够在各个流水线阶段之间正确传播。

   rvdffe #(31) i0e2pcff (.*, .en(i0_e2_data_en), .din(i0_pc_e1[31:1]), .dout(i0_pc_e2[31:1]));
   rvdffe #(31) i0e3pcff (.*, .en(i0_e3_data_en), .din(i0_pc_e2[31:1]), .dout(i0_pc_e3[31:1]));
   rvdffe #(31) i0e4pcff (.*, .en(i0_e4_data_en), .din(i0_pc_e3[31:1]), .dout(i0_pc_e4[31:1])); // 上一级流水线阶段 pc 的输出 是 下一级流水线阶段 pc 的输入 
   
   rvdffe #(31) i1e2pcff (.*, .en(i1_e2_data_en), .din(i1_pc_e1[31:1]), .dout(i1_pc_e2[31:1]));
   rvdffe #(31) i1e3pcff (.*, .en(i1_e3_data_en), .din(i1_pc_e2[31:1]), .dout(i1_pc_e3[31:1]));
   rvdffe #(31) i1e4pcff (.*, .en(i1_e4_data_en), .din(i1_pc_e3[31:1]), .dout(i1_pc_e4[31:1]));   // i0_prediction_valid
     */ 
// i0_pc_e1, i1_pc_e1;  i0_pc_e2, i1_pc_e2;   i0_pc_e3, i1_pc_e3;   i0_pc_e4, i1_pc_e4; 变量的赋值    begin   
   
   
   
   
   assign dec_i0_pc_e3[31:1] = i0_pc_e3[31:1];  
   assign dec_i1_pc_e3[31:1] = i1_pc_e3[31:1]; // dec_i0_pc_e3, dec_i1_pc_e3: output，在本模块没有使用, to exu.sv 


   assign dec_tlu_i0_pc_e4[31:1] = (exu_div_finish) ? div_pc[31:1] : i0_pc_e4[31:1]; // i0 trap pc
   assign dec_tlu_i1_pc_e4[31:1] = i1_pc_e4[31:1];
   
   
   

   // generate the correct npc for correct br predictions
   assign last_pc_e2[31:1] = (e2d.i1valid) ? i1_pc_e2[31:1] : i0_pc_e2[31:1];

   rvbradder ibradder_correct (
                     .pc(last_pc_e2[31:1]),
                     .offset(last_br_immed_e2[12:1]), // tag_last_br_immed_e2: 2
                     .dout(pred_correct_npc_e2[31:1])
                     ); // rvbradder: 分支地址计算器”（Branch Adder）



   // needed for debug triggers
   rvdffe #(31) i1wbpcff (.*, .en(i1_wb_data_en), .din(dec_tlu_i1_pc_e4[31:1]), .dout(i1_pc_wb[31:1]));
   // #(31)：这是参数化模块的实例化，表示 rvdffe 是一个宽度为31位的模块实例。






   
   
   
   // bit 9 is priority match, bit 0 lowest priority, i1_e1, i0_e1, i1_e2, ... i1_wb, i0_wb


// i0_rs1bypass, i0_rs2bypass, i1_rs1bypass, i1_rs2bypass 变量的赋值   begin    

		// e1: 1, 2   e2: 3, 4   e3: 5, 6  e4: 7, 8  wb: 9, 10

// 下面10行，最多只有一行的结果为 1, 其他行的结果为 0, 也就是说 i0_rs1bypass[9:0] 最多只有一位为 1, 其他位全部为 0, 每一行表达式 为 每一位赋值, 即 i0_rs1bypass[9] = i0_rs1_depth_d[3:0] == 4'd1  &  i0_rs1_class_d.alu,
	// tag_i0_rs1bypass: 1 // tag_i0_rs1_depth_d: 2, tag_i0_rs1_class_d: 2
	
   assign i0_rs1bypass[9:0] = {   i0_rs1_depth_d[3:0] == 4'd1  &  i0_rs1_class_d.alu, 
                                  i0_rs1_depth_d[3:0] == 4'd2  &  i0_rs1_class_d.alu, // e1
								  
                                  i0_rs1_depth_d[3:0] == 4'd3  &  i0_rs1_class_d.alu,
                                  i0_rs1_depth_d[3:0] == 4'd4  &  i0_rs1_class_d.alu, // e2
								  
                                  i0_rs1_depth_d[3:0] == 4'd5  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                  i0_rs1_depth_d[3:0] == 4'd6  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul), // e3
								  
                                  i0_rs1_depth_d[3:0] == 4'd7  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec),
                                  i0_rs1_depth_d[3:0] == 4'd8  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec), // e4
								  
                                  i0_rs1_depth_d[3:0] == 4'd9  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec),
                                  i0_rs1_depth_d[3:0] == 4'd10 & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec) }; // wb
								  
								  
								  
								  // 可以灵活地选择在处理 不同深度和操作类别时 是否使用旁路。


   assign i0_rs2bypass[9:0] = {   i0_rs2_depth_d[3:0] == 4'd1  &  i0_rs2_class_d.alu, 
                                  i0_rs2_depth_d[3:0] == 4'd2  &  i0_rs2_class_d.alu,
                                  i0_rs2_depth_d[3:0] == 4'd3  &  i0_rs2_class_d.alu,
                                  i0_rs2_depth_d[3:0] == 4'd4  &  i0_rs2_class_d.alu,
                                  i0_rs2_depth_d[3:0] == 4'd5  & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                  i0_rs2_depth_d[3:0] == 4'd6  & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                  i0_rs2_depth_d[3:0] == 4'd7  & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul | i0_rs2_class_d.sec),
                                  i0_rs2_depth_d[3:0] == 4'd8  & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul | i0_rs2_class_d.sec),
                                  i0_rs2_depth_d[3:0] == 4'd9  & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul | i0_rs2_class_d.sec),
                                  i0_rs2_depth_d[3:0] == 4'd10 & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul | i0_rs2_class_d.sec) };


   assign i1_rs1bypass[9:0] = {   i1_rs1_depth_d[3:0] == 4'd1  &  i1_rs1_class_d.alu,  
                                  i1_rs1_depth_d[3:0] == 4'd2  &  i1_rs1_class_d.alu,  // e1
								  
                                  i1_rs1_depth_d[3:0] == 4'd3  &  i1_rs1_class_d.alu,
                                  i1_rs1_depth_d[3:0] == 4'd4  &  i1_rs1_class_d.alu,  // e2
								  
                                  i1_rs1_depth_d[3:0] == 4'd5  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul),
                                  i1_rs1_depth_d[3:0] == 4'd6  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul),  // e3
								  
                                  i1_rs1_depth_d[3:0] == 4'd7  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec),
                                  i1_rs1_depth_d[3:0] == 4'd8  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec),  // e4
								  
                                  i1_rs1_depth_d[3:0] == 4'd9  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec),
                                  i1_rs1_depth_d[3:0] == 4'd10 & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec) };  // wb


   assign i1_rs2bypass[9:0] = {   i1_rs2_depth_d[3:0] == 4'd1  &  i1_rs2_class_d.alu, 
                                  i1_rs2_depth_d[3:0] == 4'd2  &  i1_rs2_class_d.alu,
								  
                                  i1_rs2_depth_d[3:0] == 4'd3  &  i1_rs2_class_d.alu,
                                  i1_rs2_depth_d[3:0] == 4'd4  &  i1_rs2_class_d.alu,
								  
                                  i1_rs2_depth_d[3:0] == 4'd5  & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul),
                                  i1_rs2_depth_d[3:0] == 4'd6  & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul),
								  
                                  i1_rs2_depth_d[3:0] == 4'd7  & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul | i1_rs2_class_d.sec),
                                  i1_rs2_depth_d[3:0] == 4'd8  & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul | i1_rs2_class_d.sec),
								  
                                  i1_rs2_depth_d[3:0] == 4'd9  & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul | i1_rs2_class_d.sec),
                                  i1_rs2_depth_d[3:0] == 4'd10 & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul | i1_rs2_class_d.sec) };

								  
// i0_rs1bypass, i0_rs2bypass, i1_rs1bypass, i1_rs2bypass 变量的赋值   end



// depth 不为 0, class 可能全部为 0, depth 为 0, class 一定为 0
// depth 匹配, class 不匹配
// depth = 8, 但是 class 类型全部为 0, 即 depth 匹配上了, 但是 class 全部不匹配
// class 指令类别 最多只有一个为 1






						  

				


// 可以找到使用预测值, 但是指令的实际结果在哪个阶段产生 无法找到
// 这一部分是使用预测值
// Added     没有依赖关系 depth 为 0, depth, class 可以同时为 0,
  
  /*
   assign i0_rs1predicted[9:0] = {    i0_rs1_depth_d[3:0] == 4'd1  &  ~i0_rs1_class_d.alu,  // 10,0000,0000                                                                        512   有
									  i0_rs1_depth_d[3:0] == 4'd2  &  ~i0_rs1_class_d.alu,  // 01,0000,0000                                                                        256   有
									  
									  i0_rs1_depth_d[3:0] == 4'd3  &  ~i0_rs1_class_d.alu,  // 00,1000,0000                                                                        128   有
									  i0_rs1_depth_d[3:0] == 4'd4  &  ~i0_rs1_class_d.alu,  // 00,0100,0000                                                                        64    有
									  
																																													// 下面这几种情况不可能出现
									  
									  i0_rs1_depth_d[3:0] == 4'd5  & (~i0_rs1_class_d.alu & ~i0_rs1_class_d.load & ~i0_rs1_class_d.mul),  // 00,0010,0000                          32
									  i0_rs1_depth_d[3:0] == 4'd6  & (~i0_rs1_class_d.alu & ~i0_rs1_class_d.load & ~i0_rs1_class_d.mul),  // 00,0001,0000                          16  有
									                                                    
									  i0_rs1_depth_d[3:0] == 4'd7  & (~i0_rs1_class_d.alu & ~i0_rs1_class_d.load & ~i0_rs1_class_d.mul & ~i0_rs1_class_d.sec), // 00,0000,1000       8
									  i0_rs1_depth_d[3:0] == 4'd8  & (~i0_rs1_class_d.alu & ~i0_rs1_class_d.load & ~i0_rs1_class_d.mul & ~i0_rs1_class_d.sec), // 00,0000,0100      4   出现了
									                                                      
									  i0_rs1_depth_d[3:0] == 4'd9  & (~i0_rs1_class_d.alu & ~i0_rs1_class_d.load & ~i0_rs1_class_d.mul & ~i0_rs1_class_d.sec),  // 00,0000,0010     2 
									  i0_rs1_depth_d[3:0] == 4'd10 & (~i0_rs1_class_d.alu & ~i0_rs1_class_d.load & ~i0_rs1_class_d.mul & ~i0_rs1_class_d.sec) }; // 00,0000,0001    1
			
			
   assign i0_rs2predicted[9:0] = {    i0_rs2_depth_d[3:0] == 4'd1  &  ~i0_rs2_class_d.alu,
									  i0_rs2_depth_d[3:0] == 4'd2  &  ~i0_rs2_class_d.alu,
									                                        
									  i0_rs2_depth_d[3:0] == 4'd3  &  ~i0_rs2_class_d.alu,
									  i0_rs2_depth_d[3:0] == 4'd4  &  ~i0_rs2_class_d.alu,
									                                        
									  i0_rs2_depth_d[3:0] == 4'd5  & (~i0_rs2_class_d.alu & ~i0_rs2_class_d.load & ~i0_rs2_class_d.mul),
									  i0_rs2_depth_d[3:0] == 4'd6  & (~i0_rs2_class_d.alu & ~i0_rs2_class_d.load & ~i0_rs2_class_d.mul),
									                                                                                     
									  i0_rs2_depth_d[3:0] == 4'd7  & (~i0_rs2_class_d.alu & ~i0_rs2_class_d.load & ~i0_rs2_class_d.mul & ~i0_rs2_class_d.sec),
									  i0_rs2_depth_d[3:0] == 4'd8  & (~i0_rs2_class_d.alu & ~i0_rs2_class_d.load & ~i0_rs2_class_d.mul & ~i0_rs2_class_d.sec),
									                                                                                                           
									  i0_rs2_depth_d[3:0] == 4'd9  & (~i0_rs2_class_d.alu & ~i0_rs2_class_d.load & ~i0_rs2_class_d.mul & ~i0_rs2_class_d.sec),
									  i0_rs2_depth_d[3:0] == 4'd10 & (~i0_rs2_class_d.alu & ~i0_rs2_class_d.load & ~i0_rs2_class_d.mul & ~i0_rs2_class_d.sec) };							   

   assign i1_rs1predicted[9:0] = {    i1_rs1_depth_d[3:0] == 4'd1  &  ~i1_rs1_class_d.alu,
									  i1_rs1_depth_d[3:0] == 4'd2  &  ~i1_rs1_class_d.alu,
									                                    
									  i1_rs1_depth_d[3:0] == 4'd3  &  ~i1_rs1_class_d.alu,
									  i1_rs1_depth_d[3:0] == 4'd4  &  ~i1_rs1_class_d.alu,
									                                    
									  i1_rs1_depth_d[3:0] == 4'd5  & (~i1_rs1_class_d.alu & ~i1_rs1_class_d.load & ~i1_rs1_class_d.mul),
									  i1_rs1_depth_d[3:0] == 4'd6  & (~i1_rs1_class_d.alu & ~i1_rs1_class_d.load & ~i1_rs1_class_d.mul),
									                                                                                 
									  i1_rs1_depth_d[3:0] == 4'd7  & (~i1_rs1_class_d.alu & ~i1_rs1_class_d.load & ~i1_rs1_class_d.mul & ~i1_rs1_class_d.sec),
									  i1_rs1_depth_d[3:0] == 4'd8  & (~i1_rs1_class_d.alu & ~i1_rs1_class_d.load & ~i1_rs1_class_d.mul & ~i1_rs1_class_d.sec),
									                                                                                                       
									  i1_rs1_depth_d[3:0] == 4'd9  & (~i1_rs1_class_d.alu & ~i1_rs1_class_d.load & ~i1_rs1_class_d.mul & ~i1_rs1_class_d.sec),
									  i1_rs1_depth_d[3:0] == 4'd10 & (~i1_rs1_class_d.alu & ~i1_rs1_class_d.load & ~i1_rs1_class_d.mul & ~i1_rs1_class_d.sec) };									  
  
   assign i1_rs2predicted[9:0] = {    i1_rs2_depth_d[3:0] == 4'd1  &  ~i1_rs2_class_d.alu,
									  i1_rs2_depth_d[3:0] == 4'd2  &  ~i1_rs2_class_d.alu,
									                                        
									  i1_rs2_depth_d[3:0] == 4'd3  &  ~i1_rs2_class_d.alu,
									  i1_rs2_depth_d[3:0] == 4'd4  &  ~i1_rs2_class_d.alu,
									                                        
									  i1_rs2_depth_d[3:0] == 4'd5  & (~i1_rs2_class_d.alu & ~i1_rs2_class_d.load & ~i1_rs2_class_d.mul),
									  i1_rs2_depth_d[3:0] == 4'd6  & (~i1_rs2_class_d.alu & ~i1_rs2_class_d.load & ~i1_rs2_class_d.mul),
									                                                                                     
									  i1_rs2_depth_d[3:0] == 4'd7  & (~i1_rs2_class_d.alu & ~i1_rs2_class_d.load & ~i1_rs2_class_d.mul & ~i1_rs2_class_d.sec),
									  i1_rs2_depth_d[3:0] == 4'd8  & (~i1_rs2_class_d.alu & ~i1_rs2_class_d.load & ~i1_rs2_class_d.mul & ~i1_rs2_class_d.sec),
									                                                                                                           
									  i1_rs2_depth_d[3:0] == 4'd9  & (~i1_rs2_class_d.alu & ~i1_rs2_class_d.load & ~i1_rs2_class_d.mul & ~i1_rs2_class_d.sec),
									  i1_rs2_depth_d[3:0] == 4'd10 & (~i1_rs2_class_d.alu & ~i1_rs2_class_d.load & ~i1_rs2_class_d.mul & ~i1_rs2_class_d.sec) };
									  
*/	


  
     

    assign i0_rs1predicted[3:0] = {   
      
                             i0_rs1_depth_d[3:0] == 4'd1  &  i0_rs1_class_d.load ,  // 10,0000,0000                                                                        512   有
									  i0_rs1_depth_d[3:0] == 4'd2  &  i0_rs1_class_d.load ,  // 01,0000,0000                                                                        256   有
									  
									  i0_rs1_depth_d[3:0] == 4'd3  &  i0_rs1_class_d.load ,  // 00,1000,0000                                                                        128   有
									  i0_rs1_depth_d[3:0] == 4'd4  &  i0_rs1_class_d.load  // 00,0100,0000    
      
      
      };


      assign i0_rs2predicted[3:0] = {   
      
                           i0_rs2_depth_d[3:0] == 4'd1  &  i0_rs2_class_d.load ,  // 10,0000,0000                                                                        512   有
                           i0_rs2_depth_d[3:0] == 4'd2  &  i0_rs2_class_d.load ,  // 01,0000,0000                                                                        256   有
      
                           i0_rs2_depth_d[3:0] == 4'd3  &  i0_rs2_class_d.load ,  // 00,1000,0000                                                                        128   有
                           i0_rs2_depth_d[3:0] == 4'd4  &  i0_rs2_class_d.load // 00,0100,0000    


      };


      assign i1_rs1predicted[3:0] = {   
      
                        i1_rs1_depth_d[3:0] == 4'd1  &  i1_rs1_class_d.load,  // 10,0000,0000                                                                        512   有
                        i1_rs1_depth_d[3:0] == 4'd2  &  i1_rs1_class_d.load ,  // 01,0000,0000                                                                        256   有
                                       
                        i1_rs1_depth_d[3:0] == 4'd3  &  i1_rs1_class_d.load ,  // 00,1000,0000                                                                        128   有
                        i1_rs1_depth_d[3:0] == 4'd4  &  i1_rs1_class_d.load };  // 00,0100,0000    





      assign i1_rs2predicted[3:0] = {   

                        i1_rs2_depth_d[3:0] == 4'd1  &  i1_rs2_class_d.load ,  // 10,0000,0000                                                                        512   有
                        i1_rs2_depth_d[3:0] == 4'd2  &  i1_rs2_class_d.load ,  // 01,0000,0000                                                                        256   有

                        i1_rs2_depth_d[3:0] == 4'd3  &  i1_rs2_class_d.load ,  // 00,1000,0000                                                                        128   有
                        i1_rs2_depth_d[3:0] == 4'd4  &  i1_rs2_class_d.load    // 00,0100,0000    


      };
      
      /*

    assign i0_rs1predicted[3:0] = {   
      
                             i0_rs1_depth_d[3:0] == 4'd1  &  i0_rs1_class_d.load & i0_not_alu_eff,  // 10,0000,0000                                                                        512   有
									  i0_rs1_depth_d[3:0] == 4'd2  &  i0_rs1_class_d.load & i0_not_alu_eff,  // 01,0000,0000                                                                        256   有
									  
									  i0_rs1_depth_d[3:0] == 4'd3  &  i0_rs1_class_d.load & (i0_not_alu_eff & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul),  // 00,1000,0000                                                                        128   有
									  i0_rs1_depth_d[3:0] == 4'd4  &  i0_rs1_class_d.load & (i0_not_alu_eff & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) // 00,0100,0000    
      
      
      };


      assign i0_rs2predicted[3:0] = {   
      
                           i0_rs2_depth_d[3:0] == 4'd1  &  i0_rs2_class_d.load & ( i0_not_alu_eff & ~i0_dp.store),  // 10,0000,0000                                                                        512   有
                           i0_rs2_depth_d[3:0] == 4'd2  &  i0_rs2_class_d.load & ( i0_not_alu_eff & ~i0_dp.store),  // 01,0000,0000                                                                        256   有
      
                           i0_rs2_depth_d[3:0] == 4'd3  &  i0_rs2_class_d.load & (i0_not_alu_eff & ~i0_dp.store & ~i0_dp.mul),  // 00,1000,0000                                                                        128   有
                           i0_rs2_depth_d[3:0] == 4'd4  &  i0_rs2_class_d.load & (i0_not_alu_eff & ~i0_dp.store & ~i0_dp.mul)// 00,0100,0000    


      };


      assign i1_rs1predicted[3:0] = {   
      
                        i1_rs1_depth_d[3:0] == 4'd1  &  i1_rs1_class_d.load & i1_not_alu_eff,  // 10,0000,0000                                                                        512   有
                        i1_rs1_depth_d[3:0] == 4'd2  &  i1_rs1_class_d.load & i1_not_alu_eff,  // 01,0000,0000                                                                        256   有
                                       
                        i1_rs1_depth_d[3:0] == 4'd3  &  i1_rs1_class_d.load & (i1_not_alu_eff & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul),  // 00,1000,0000                                                                        128   有
                        i1_rs1_depth_d[3:0] == 4'd4  &  i1_rs1_class_d.load & (i1_not_alu_eff & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul)};  // 00,0100,0000    





      assign i1_rs2predicted[3:0] = {   

                        i1_rs2_depth_d[3:0] == 4'd1  &  i1_rs2_class_d.load & ( i1_not_alu_eff & ~i1_dp.store),  // 10,0000,0000                                                                        512   有
                        i1_rs2_depth_d[3:0] == 4'd2  &  i1_rs2_class_d.load & ( i1_not_alu_eff & ~i1_dp.store),  // 01,0000,0000                                                                        256   有

                        i1_rs2_depth_d[3:0] == 4'd3  &  i1_rs2_class_d.load & (i1_not_alu_eff & ~i1_dp.store & ~i1_dp.mul),  // 00,1000,0000                                                                        128   有
                        i1_rs2_depth_d[3:0] == 4'd4  &  i1_rs2_class_d.load & (i1_not_alu_eff & ~i1_dp.store & ~i1_dp.mul)   // 00,0100,0000    


      };

      */     											               


    logic [3:0] i0_rs1_location, i0_rs2_location, i1_rs1_location, i1_rs2_location;
	/*
	assign i0_rs1_location[3:0] = location(i0_rs1bypass[9:0], i0_rs1predicted[9:0]);
	assign i0_rs2_location[3:0] = location(i0_rs2bypass[9:0], i0_rs2predicted[9:0]);
	assign i1_rs1_location[3:0] = location(i1_rs1bypass[9:0], i1_rs1predicted[9:0]);
	assign i1_rs2_location[3:0] = location(i1_rs2bypass[9:0], i1_rs2predicted[9:0]);
	*/
	
	// 1, 判断使用预测值的指令: 该指令使用了预测值, 判断该指令的实际值在哪个阶段产生, 判断 实际值 与 使用了的预测值 是否相同, 如果不相同, 使用了预测值的指令也刷新掉
					// 需要找出 使用了预测值的指令在哪一阶段产生实际值
	
	
	// 2, 判断 产生预测值的指令的预测值是否正确:   产生预测值的指令 在 e4 阶段 与产生的实际值进行比较, 如果预测值与实际值相等, 则不刷新,  如果预测值与实际值不相等, 则刷新当前指令之后的所有指令
	               //  将产生预测值的指令 与 指令的实际值  在 e4阶段 进行比较	  如何将产生的预测值 给 依赖指令 使用
    
	// 只能用方法2, 方法1 使用预测值的指令的实际结果 在哪个阶段产生判断不了
	// 如果预测值不正确, 刷新后面全部的指令
	/*		   
	if(i0_rs1_location[3:0] != 4'd10) begin   // if 条件判断是 是否使用了预测值
	case(i0_rs1_location[3:0])
		4'd9 : flag = 	i1_predict_e1[31:0] == i1_result_e1[31:0];	
		4'd8 : flag = 	i0_predict_e1[31:0] == i0_result_e1[31:0];
		
		4'd7 : flag = 	i1_predict_e2[31:0] == i1_result_e2[31:0];
		4'd6 : flag = 	i0_predict_e2[31:0] == i0_result_e2[31:0];
		
		4'd5 : flag = 	i1_predict_e3[31:0] == i1_result_e3_final[31:0];
		4'd4 : flag = 	i1_predict_e3[31:0] == i0_result_e3_final[31:0];
		
		4'd3 : flag = 	i1_predict_e4[31:0] == i1_result_e4_final[31:0];
		4'd2 : flag = 	i1_predict_e4[31:0] == i0_result_e4_final[31:0];
		
		4'd1 : flag = 	i1_predict_wb[31:0] == i1_result_wb[31:0];
		4'd0 : flag = 	i1_predict_wb[31:0] == i0_result_wb[31:0];
	
	endcase
	
	end
	*/


	
											
										
// 有数据依赖，通过旁路来获取 i0 的第一个源操作数 look_tag  
                            

   assign dec_i0_rs1_bypass_en_d = |i0_rs1bypass[9:0];
   assign dec_i0_rs2_bypass_en_d = |i0_rs2bypass[9:0];
   assign dec_i1_rs1_bypass_en_d = |i1_rs1bypass[9:0];
   assign dec_i1_rs2_bypass_en_d = |i1_rs2bypass[9:0];



   assign i0_rs1_bypass_data_d[31:0] = ({32{i0_rs1bypass[9]}} & i1_result_e1[31:0]) |
                                       ({32{i0_rs1bypass[8]}} & i0_result_e1[31:0]) |
                                       ({32{i0_rs1bypass[7]}} & i1_result_e2[31:0]) |
                                       ({32{i0_rs1bypass[6]}} & i0_result_e2[31:0]) |
                                       ({32{i0_rs1bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i0_rs1bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i0_rs1bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i0_rs1bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i0_rs1bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i0_rs1bypass[0]}} & i0_result_wb[31:0]);


   assign i0_rs2_bypass_data_d[31:0] = ({32{i0_rs2bypass[9]}} & i1_result_e1[31:0]) |
                                       ({32{i0_rs2bypass[8]}} & i0_result_e1[31:0]) |
                                       ({32{i0_rs2bypass[7]}} & i1_result_e2[31:0]) |
                                       ({32{i0_rs2bypass[6]}} & i0_result_e2[31:0]) |
                                       ({32{i0_rs2bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i0_rs2bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i0_rs2bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i0_rs2bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i0_rs2bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i0_rs2bypass[0]}} & i0_result_wb[31:0]);

   assign i1_rs1_bypass_data_d[31:0] = ({32{i1_rs1bypass[9]}} & i1_result_e1[31:0]) |
                                       ({32{i1_rs1bypass[8]}} & i0_result_e1[31:0]) |
                                       ({32{i1_rs1bypass[7]}} & i1_result_e2[31:0]) |
                                       ({32{i1_rs1bypass[6]}} & i0_result_e2[31:0]) |
                                       ({32{i1_rs1bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i1_rs1bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i1_rs1bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i1_rs1bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i1_rs1bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i1_rs1bypass[0]}} & i0_result_wb[31:0]);


   assign i1_rs2_bypass_data_d[31:0] = ({32{i1_rs2bypass[9]}} & i1_result_e1[31:0]) |
                                       ({32{i1_rs2bypass[8]}} & i0_result_e1[31:0]) |
                                       ({32{i1_rs2bypass[7]}} & i1_result_e2[31:0]) |
                                       ({32{i1_rs2bypass[6]}} & i0_result_e2[31:0]) |
                                       ({32{i1_rs2bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i1_rs2bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i1_rs2bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i1_rs2bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i1_rs2bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i1_rs2bypass[0]}} & i0_result_wb[31:0]);



                           


   logic i0_predict_valid, i1_predict_valid;

   assign i0_predict_valid = i0_prediction_valid_d;
   assign i1_predict_valid = i1_prediction_valid_d;
   
   /*
   // |: 按位或    i0_rs1bypass[9:0] 中有任何一位为 1，则 |i0_rs1bypass[9:0] 的结果为 1，表示 可以旁路；如果所有位都是 0，则结果为 0，表示 不可以旁路。
   assign dec_i0_rs1_bypass_en = (|i0_rs1bypass[9:0]) | (| {i0_rs1predicted[3:0] & 4{i0_prediction_valid_d}}); // i0_prediction_valid_d = 1: 表示产生的预测值有效
   assign dec_i0_rs2_bypass_en = (|i0_rs2bypass[9:0]) | (| {i0_rs2predicted[3:0] & 4{i0_prediction_valid_d}});
   assign dec_i1_rs1_bypass_en = (|i1_rs1bypass[9:0]) | (| {i1_rs1predicted[3:0] & 4{i1_prediction_valid_d}}); 
   assign dec_i1_rs2_bypass_en = (|i1_rs2bypass[9:0]) | (| {i1_rs2predicted[3:0] & 4{i1_prediction_valid_d}}); // 四个变量都是 output, to exu.sv 
   */

    // |: 按位或    i0_rs1bypass[9:0] 中有任何一位为 1，则 |i0_rs1bypass[9:0] 的结果为 1，表示 可以旁路；如果所有位都是 0，则结果为 0，表示 不可以旁路。
   assign dec_i0_rs1_bypass_en = (|i0_rs1bypass[9:0]) | (| i0_rs1predicted[3:0]); // i0_prediction_valid_d = 1: 表示产生的预测值有效
   assign dec_i0_rs2_bypass_en = (|i0_rs2bypass[9:0]) | (|i0_rs2predicted[3:0] );
   assign dec_i1_rs1_bypass_en = (|i1_rs1bypass[9:0]) | (| i1_rs1predicted[3:0]); 
   assign dec_i1_rs2_bypass_en = (|i1_rs2bypass[9:0]) | (| i1_rs2predicted[3:0] ); // 四个变量都是 output, to exu.sv 


      // 带有预测值的  旁路值   32'd1
   assign i0_rs1_bypass_data[31:0] = ( ({32{i0_rs1bypass[9]}} & i1_result_e1[31:0]) |   ({32{i0_rs1predicted[3]}} & i1_predicted_result_e1[31:0])  ) |   // 如果 i0_rs1bypass[9] 为1，那么 i0_rs1_bypass_data_d 将会等于 i1_result_e1；
                                       ( ({32{i0_rs1bypass[8]}} & i0_result_e1[31:0]) | ({32{i0_rs1predicted[2]}} & i0_predicted_result_e1[31:0]) ) |  // e1   
									   
                                       ( ({32{i0_rs1bypass[7]}} & i1_result_e2[31:0]) | ({32{i0_rs1predicted[1]}} & i1_predicted_result_e2[31:0]) ) |
                                       ( ({32{i0_rs1bypass[6]}} & i0_result_e2[31:0]) | ({32{i0_rs1predicted[0]}} & i0_predicted_result_e2[31:0]) ) |  // e2     //  i0_result_e2 单独是一个 output 信号


									   
                                       ({32{i0_rs1bypass[5]}} & i1_result_e3_final[31:0]) | 
                                       ({32{i0_rs1bypass[4]}} & i0_result_e3_final[31:0]) |  // e3
									   
                                       ({32{i0_rs1bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i0_rs1bypass[2]}} & i0_result_e4_final[31:0]) |  // e4
									   
                                       ({32{i0_rs1bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i0_rs1bypass[0]}} & i0_result_wb[31:0]);  // wb  // tag_i0_rs1bypass: 2, tag_i_rs_bypass_input: 2

		// 从 exu 上一条指令的结果，   exu用 上一条或其他指令 用于下一条指令的输入
   assign i0_rs2_bypass_data[31:0] = 
                                       ( ({32{i0_rs2bypass[9]}} & i1_result_e1[31:0]) | ({32{i0_rs2predicted[3]}} & i1_predicted_result_e1[31:0])  ) |   // 如果 i0_rs1bypass[9] 为1，那么 i0_rs1_bypass_data_d 将会等于 i1_result_e1；
                                       ( ({32{i0_rs2bypass[8]}} & i0_result_e1[31:0]) | ({32{i0_rs2predicted[2]}} & i0_predicted_result_e1[31:0]) ) |  // e1   
									   
                                       ( ({32{i0_rs2bypass[7]}} & i1_result_e2[31:0]) | ({32{i0_rs2predicted[1]}} & i1_predicted_result_e2[31:0]) ) |
                                       ( ({32{i0_rs2bypass[6]}} & i0_result_e2[31:0]) | ({32{i0_rs2predicted[0]}} & i0_predicted_result_e2[31:0]) ) |



                                       ({32{i0_rs2bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i0_rs2bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i0_rs2bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i0_rs2bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i0_rs2bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i0_rs2bypass[0]}} & i0_result_wb[31:0]);

   assign i1_rs1_bypass_data[31:0] = 
                                       ( ({32{i1_rs1bypass[9]}} & i1_result_e1[31:0]) | ({32{i1_rs1predicted[3]}} & i1_predicted_result_e1[31:0])  ) |   // 如果 i0_rs1bypass[9] 为1，那么 i0_rs1_bypass_data_d 将会等于 i1_result_e1；
                                       ( ({32{i1_rs1bypass[8]}} & i0_result_e1[31:0]) | ({32{i1_rs1predicted[2]}} & i0_predicted_result_e1[31:0]) ) |  // e1   
									   
                                       ( ({32{i1_rs1bypass[7]}} & i1_result_e2[31:0]) | ({32{i1_rs1predicted[1]}} & i1_predicted_result_e2[31:0]) ) |
                                       ( ({32{i1_rs1bypass[6]}} & i0_result_e2[31:0]) | ({32{i1_rs1predicted[0]}} & i0_predicted_result_e2[31:0]) ) |

                                       ({32{i1_rs1bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i1_rs1bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i1_rs1bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i1_rs1bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i1_rs1bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i1_rs1bypass[0]}} & i0_result_wb[31:0]);


   assign i1_rs2_bypass_data[31:0] = 
                                       ( ({32{i1_rs2bypass[9]}} & i1_result_e1[31:0]) | ({32{i1_rs2predicted[3]}} & i1_predicted_result_e1[31:0])  ) |   // 如果 i0_rs1bypass[9] 为1，那么 i0_rs1_bypass_data_d 将会等于 i1_result_e1；
                                       ( ({32{i1_rs2bypass[8]}} & i0_result_e1[31:0]) | ({32{i1_rs2predicted[2]}} & i0_predicted_result_e1[31:0]) ) |  // e1   
									   
                                       ( ({32{i1_rs2bypass[7]}} & i1_result_e2[31:0]) | ({32{i1_rs2predicted[1]}} & i1_predicted_result_e2[31:0]) ) |
                                       ( ({32{i1_rs2bypass[6]}} & i0_result_e2[31:0]) | ({32{i1_rs2predicted[0]}} & i0_predicted_result_e2[31:0]) ) |


                                       ({32{i1_rs2bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i1_rs2bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i1_rs2bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i1_rs2bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i1_rs2bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i1_rs2bypass[0]}} & i0_result_wb[31:0]); // 四个变量都是输出信号，输出到 exu.sv 文件中

   


// i0_rs1bypass, i0_rs2bypass, i1_rs1bypass, i1_rs2bypass 变量的使用   end


assign i0_i1_load_match_i0 = i0_load_match_e1_i0 | i1_load_match_e1_i0 | i0_load_match_e2_i0 | i1_load_match_e2_i0;  // 只要依赖 i0, 将 dec 阶段的 i0 i1 指令放到 i0 队列

assign i0_i1_load_match_i1 = i0_load_match_e1_i1 | i1_load_match_e1_i1 | i0_load_match_e2_i1 | i1_load_match_e2_i1;
// assign i0_i1_load_match_i1 = 0;









   

                   
        
				

endmodule












function signed [31 : 0] Predicted_Value;

   input signed [31 : 0] Last_Value;
   input signed [31 : 0] Last_Last_Value;

   logic signed [31 : 0] Stride_Value;

   begin

      Stride_Value[31:0] = Last_Value[31:0] - Last_Last_Value[31:0];
      Predicted_Value[31:0] = Last_Value[31:0] + Stride_Value[31:0];

   end

   
   
endfunction



										
// 先判断 i1_rs2bypass[9] 的值是否为 1, 再判断 i1_rs2predicted[9] 的值是否为 1, 然后再判断 i1_rs2bypass[8] 的值是否为 1
// 设置一个输出 count, count[0] 最低位为 0, 表示使用的是 实际值, count[1] 最低位为 1, 表示使用的是 预测值, 通过判断最低位 来判断是否使用了预测值
// rsbypass[9:0], rspredicted[9:0] 最多只有一位为 1, 比较 rsbypass[9:0], rspredicted[9:0] 的大小, 哪一个大, 所使用的就是哪一个信号了   好像是不行     1000  0010



// 通过该函数来判断是否使用了预测值, 以及使用的是哪一流水阶段的预测值     函数没有问题, 能准确找到 使用哪一流水阶段的预测值     就可以方便记录预测值, 待实际值产生时进行验证

// 同一流水阶段中, 有实际值先用实际值, 没有实际值用预测值,
   function [3:0] location;  
   
      input [9:0] rsbypass;
	  input [9:0] rspredicted;
	  
	  reg [10:0] count;

      begin
		// count 的结果为 下面所有行中的某一行
		count = rsbypass[9]    ? 11'b100_0000_0000 :
			    rspredicted[9] ? 11'b100_0000_0001 :
			    
			    rsbypass[8]    ? 11'b010_0000_0000 :
			    rspredicted[8] ? 11'b010_0000_0001 :
			    
			    rsbypass[7]    ? 11'b001_0000_0000 :
			    rspredicted[7] ? 11'b001_0000_0001 :
			    
			    rsbypass[6]    ? 11'b000_1000_0000 :
			    rspredicted[6] ? 11'b000_1000_0001 :
			    
			    rsbypass[5]    ? 11'b000_0100_0000 :
			    rspredicted[5] ? 11'b000_0100_0001 :
			    
			    rsbypass[4]    ? 11'b000_0010_0000 :
			    rspredicted[4] ? 11'b000_0010_0001 :
			    
			    rsbypass[3]    ? 11'b000_0001_0000 :
			    rspredicted[3] ? 11'b000_0001_0001 :
			    
			    rsbypass[2]    ? 11'b000_0000_1000 :
			    rspredicted[2] ? 11'b000_0000_1001 :
			    
			    rsbypass[1]    ? 11'b000_0000_0100 :
			    rspredicted[1] ? 11'b000_0000_0101 :
			    
			    rsbypass[0]    ? 11'b000_0000_0010 :
			    rspredicted[0] ? 11'b000_0000_0011 : 11'b000_0000_0000;
				 
				 
		if(count[0]) begin
			case(count[10:1])
				10'b10_0000_0000: location = 4'd9;
				10'b01_0000_0000: location = 4'd8;
				10'b00_1000_0000: location = 4'd7;
				10'b00_0100_0000: location = 4'd6;
				10'b00_0010_0000: location = 4'd5;
				10'b00_0001_0000: location = 4'd4;
				10'b00_0000_1000: location = 4'd3;
				10'b00_0000_0100: location = 4'd2;
				10'b00_0000_0010: location = 4'd1;
				10'b00_0000_0001: location = 4'd0;
				default: location = 4'd10;
			endcase
		end else begin
			location = 4'd10;
		end
	
	 	  
      end
   endfunction	



// file "decode" is human readable file that has all of the instruction decodes defined and is part of git repo
// modify this file as needed

// to generate all the equations below from "decode" except legal equation:

// 1) coredecode -in decode > coredecode.e

// 2) espresso -Dso -oeqntott coredecode.e | addassign -pre out.  > equations

// to generate the legal (32b instruction is legal) equation below:

// 1) coredecode -in decode -legal > legal.e

// 2) espresso -Dso -oeqntott legal.e | addassign -pre out. > legal_equation

module dec_dec_ctl    // tag_answer  译码模块  
   import veer_types::*;
(
   input logic [31:0] inst,

   output dec_pkt_t out
   );

	logic [31:0] i;


	assign i[31:0] = inst[31:0];


	assign out.alu = (i[2]) | (i[6]) | (!i[25]&i[4]) | (!i[5]&i[4]);

	assign out.rs1 = (!i[14]&!i[13]&!i[2]) | (!i[13]&i[11]&!i[2]) | (i[19]&i[13]&!i[2]) | (
		!i[13]&i[10]&!i[2]) | (i[18]&i[13]&!i[2]) | (!i[13]&i[9]&!i[2]) | (
		i[17]&i[13]&!i[2]) | (!i[13]&i[8]&!i[2]) | (i[16]&i[13]&!i[2]) | (
		!i[13]&i[7]&!i[2]) | (i[15]&i[13]&!i[2]) | (!i[4]&!i[3]) | (!i[6]
		&!i[2]);

	assign out.rs2 = (i[5]&!i[4]&!i[2]) | (!i[6]&i[5]&!i[2]);

	assign out.imm12 = (!i[4]&!i[3]&i[2]) | (i[13]&!i[5]&i[4]&!i[2]) | (!i[13]&!i[12]
		&i[6]&i[4]) | (!i[12]&!i[5]&i[4]&!i[2]);

	assign out.rd = (!i[5]&!i[2]) | (i[5]&i[2]) | (i[4]);

	assign out.shimm5 = (!i[13]&i[12]&!i[5]&i[4]&!i[2]);

	assign out.imm20 = (i[5]&i[3]) | (i[4]&i[2]);

	assign out.pc = (!i[5]&!i[3]&i[2]) | (i[5]&i[3]);   // out.pc 对结构体中的 pc 进行赋值

	assign out.load = (!i[5]&!i[4]&!i[2]);

	assign out.store = (!i[6]&i[5]&!i[4]);

	assign out.lsu = (!i[6]&!i[4]&!i[2]);

	assign out.add = (!i[14]&!i[13]&!i[12]&!i[5]&i[4]) | (!i[5]&!i[3]&i[2]) | (!i[30]
		&!i[25]&!i[14]&!i[13]&!i[12]&!i[6]&i[4]&!i[2]);

	assign out.sub = (i[30]&!i[12]&!i[6]&i[5]&i[4]&!i[2]) | (!i[25]&!i[14]&i[13]&!i[6]
		&i[4]&!i[2]) | (!i[14]&i[13]&!i[5]&i[4]&!i[2]) | (i[6]&!i[4]&!i[2]);

	assign out.land = (i[14]&i[13]&i[12]&!i[5]&!i[2]) | (!i[25]&i[14]&i[13]&i[12]&!i[6]
		&!i[2]);

	assign out.lor = (!i[6]&i[3]) | (!i[25]&i[14]&i[13]&!i[12]&i[4]&!i[2]) | (i[5]&i[4]
		&i[2]) | (!i[12]&i[6]&i[4]) | (i[13]&i[6]&i[4]) | (i[14]&i[13]&!i[12]
		&!i[5]&!i[2]) | (i[7]&i[6]&i[4]) | (i[8]&i[6]&i[4]) | (i[9]&i[6]&i[4]) | (
		i[10]&i[6]&i[4]) | (i[11]&i[6]&i[4]);

	assign out.lxor = (!i[25]&i[14]&!i[13]&!i[12]&i[4]&!i[2]) | (i[14]&!i[13]&!i[12]
		&!i[5]&i[4]&!i[2]);

	assign out.sll = (!i[25]&!i[14]&!i[13]&i[12]&!i[6]&i[4]&!i[2]);

	assign out.sra = (i[30]&!i[13]&i[12]&!i[6]&i[4]&!i[2]);

	assign out.srl = (!i[30]&!i[25]&i[14]&!i[13]&i[12]&!i[6]&i[4]&!i[2]);

	assign out.slt = (!i[25]&!i[14]&i[13]&!i[6]&i[4]&!i[2]) | (!i[14]&i[13]&!i[5]&i[4]
		&!i[2]);

	assign out.unsign = (!i[14]&i[13]&i[12]&!i[5]&!i[2]) | (i[13]&i[6]&!i[4]&!i[2]) | (
		i[14]&!i[5]&!i[4]) | (!i[25]&!i[14]&i[13]&i[12]&!i[6]&!i[2]) | (
		i[25]&i[14]&i[12]&!i[6]&i[5]&!i[2]);

	assign out.condbr = (i[6]&!i[4]&!i[2]);

	assign out.beq = (!i[14]&!i[12]&i[6]&!i[4]&!i[2]);

	assign out.bne = (!i[14]&i[12]&i[6]&!i[4]&!i[2]);

	assign out.bge = (i[14]&i[12]&i[5]&!i[4]&!i[2]);

	assign out.blt = (i[14]&!i[12]&i[5]&!i[4]&!i[2]);

	assign out.jal = (i[6]&i[2]);

	assign out.by = (!i[13]&!i[12]&!i[6]&!i[4]&!i[2]);

	assign out.half = (i[12]&!i[6]&!i[4]&!i[2]);

	assign out.word = (i[13]&!i[6]&!i[4]);

	assign out.csr_read = (i[13]&i[6]&i[4]) | (i[7]&i[6]&i[4]) | (i[8]&i[6]&i[4]) | (
		i[9]&i[6]&i[4]) | (i[10]&i[6]&i[4]) | (i[11]&i[6]&i[4]);

	assign out.csr_clr = (i[15]&i[13]&i[12]&i[6]&i[4]) | (i[16]&i[13]&i[12]&i[6]&i[4]) | (
		i[17]&i[13]&i[12]&i[6]&i[4]) | (i[18]&i[13]&i[12]&i[6]&i[4]) | (
		i[19]&i[13]&i[12]&i[6]&i[4]);

	assign out.csr_set = (i[15]&!i[12]&i[6]&i[4]) | (i[16]&!i[12]&i[6]&i[4]) | (i[17]
		&!i[12]&i[6]&i[4]) | (i[18]&!i[12]&i[6]&i[4]) | (i[19]&!i[12]&i[6]
		&i[4]);

	assign out.csr_write = (!i[13]&i[12]&i[6]&i[4]);

	assign out.csr_imm = (i[14]&!i[13]&i[6]&i[4]) | (i[15]&i[14]&i[6]&i[4]) | (i[16]
		&i[14]&i[6]&i[4]) | (i[17]&i[14]&i[6]&i[4]) | (i[18]&i[14]&i[6]&i[4]) | (
		i[19]&i[14]&i[6]&i[4]);

	assign out.presync = (!i[5]&i[3]) | (i[25]&i[14]&!i[6]&i[5]&!i[2]) | (!i[13]&i[7]
		&i[6]&i[4]) | (!i[13]&i[8]&i[6]&i[4]) | (!i[13]&i[9]&i[6]&i[4]) | (
		!i[13]&i[10]&i[6]&i[4]) | (!i[13]&i[11]&i[6]&i[4]) | (i[15]&i[13]
		&i[6]&i[4]) | (i[16]&i[13]&i[6]&i[4]) | (i[17]&i[13]&i[6]&i[4]) | (
		i[18]&i[13]&i[6]&i[4]) | (i[19]&i[13]&i[6]&i[4]);

	assign out.postsync = (i[12]&!i[5]&i[3]) | (!i[22]&!i[13]&!i[12]&i[6]&i[4]) | (
		i[25]&i[14]&!i[6]&i[5]&!i[2]) | (!i[13]&i[7]&i[6]&i[4]) | (!i[13]
		&i[8]&i[6]&i[4]) | (!i[13]&i[9]&i[6]&i[4]) | (!i[13]&i[10]&i[6]&i[4]) | (
		!i[13]&i[11]&i[6]&i[4]) | (i[15]&i[13]&i[6]&i[4]) | (i[16]&i[13]&i[6]
		&i[4]) | (i[17]&i[13]&i[6]&i[4]) | (i[18]&i[13]&i[6]&i[4]) | (i[19]
		&i[13]&i[6]&i[4]);

	assign out.ebreak = (!i[22]&i[20]&!i[13]&!i[12]&i[6]&i[4]);

	assign out.ecall = (!i[21]&!i[20]&!i[13]&!i[12]&i[6]&i[4]);

	assign out.mret = (i[29]&!i[13]&!i[12]&i[6]&i[4]);

	assign out.mul = (i[25]&!i[14]&!i[6]&i[5]&i[4]&!i[2]);

	assign out.rs1_sign = (i[25]&!i[14]&i[13]&!i[12]&!i[6]&i[5]&i[4]&!i[2]) | (i[25]
		&!i[14]&!i[13]&i[12]&!i[6]&i[4]&!i[2]);

	assign out.rs2_sign = (i[25]&!i[14]&!i[13]&i[12]&!i[6]&i[4]&!i[2]);

	assign out.low = (i[25]&!i[14]&!i[13]&!i[12]&i[5]&i[4]&!i[2]);

	assign out.div = (i[25]&i[14]&!i[6]&i[5]&!i[2]);

	assign out.rem = (i[25]&i[14]&i[13]&!i[6]&i[5]&!i[2]);

	assign out.fence = (!i[5]&i[3]);

	assign out.fence_i = (i[12]&!i[5]&i[3]);

	assign out.pm_alu = (i[28]&i[22]&!i[13]&!i[12]&i[4]) | (i[4]&i[2]) | (!i[25]&!i[6]
		&i[4]) | (!i[5]&i[4]);


	assign out.legal = (!i[31]&!i[30]&i[29]&i[28]&!i[27]&!i[26]&!i[25]&!i[24]&!i[23]
		&!i[22]&i[21]&!i[20]&!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[11]
		&!i[10]&!i[9]&!i[8]&!i[7]&i[6]&i[5]&i[4]&!i[3]&!i[2]&i[1]&i[0]) | (
		!i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]&!i[25]&!i[24]&!i[23]&i[22]
		&!i[21]&i[20]&!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[11]&!i[10]
		&!i[9]&!i[8]&!i[7]&i[6]&i[5]&i[4]&!i[3]&!i[2]&i[1]&i[0]) | (!i[31]
		&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&!i[24]&!i[23]&!i[22]&!i[21]
		&!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[11]&!i[10]&!i[9]&!i[8]
		&!i[7]&i[5]&i[4]&!i[3]&!i[2]&i[1]&i[0]) | (!i[31]&!i[30]&!i[29]&!i[28]
		&!i[27]&!i[26]&!i[25]&!i[6]&i[4]&!i[3]&i[1]&i[0]) | (!i[31]&!i[29]
		&!i[28]&!i[27]&!i[26]&!i[25]&!i[14]&!i[13]&!i[12]&!i[6]&!i[3]&!i[2]
		&i[1]&i[0]) | (!i[31]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&i[14]&!i[13]
		&i[12]&!i[6]&i[4]&!i[3]&i[1]&i[0]) | (!i[31]&!i[30]&!i[29]&!i[28]
		&!i[27]&!i[26]&!i[6]&i[5]&i[4]&!i[3]&i[1]&i[0]) | (!i[14]&!i[13]
		&!i[12]&i[6]&i[5]&!i[4]&!i[3]&i[1]&i[0]) | (i[14]&i[6]&i[5]&!i[4]
		&!i[3]&!i[2]&i[1]&i[0]) | (!i[12]&!i[6]&!i[5]&i[4]&!i[3]&i[1]&i[0]) | (
		!i[14]&!i[13]&i[5]&!i[4]&!i[3]&!i[2]&i[1]&i[0]) | (i[12]&i[6]&i[5]
		&i[4]&!i[3]&!i[2]&i[1]&i[0]) | (!i[31]&!i[30]&!i[29]&!i[28]&!i[27]
		&!i[26]&!i[25]&!i[24]&!i[23]&!i[22]&!i[21]&!i[20]&!i[19]&!i[18]&!i[17]
		&!i[16]&!i[15]&!i[14]&!i[13]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]&!i[6]
		&!i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]) | (!i[31]&!i[30]&!i[29]&!i[28]
		&!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[13]&!i[12]&!i[11]&!i[10]
		&!i[9]&!i[8]&!i[7]&!i[6]&!i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]) | (i[13]
		&i[6]&i[5]&i[4]&!i[3]&!i[2]&i[1]&i[0]) | (!i[13]&!i[6]&!i[5]&!i[4]
		&!i[3]&!i[2]&i[1]&i[0]) | (i[6]&i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]) | (
		i[13]&!i[6]&!i[5]&i[4]&!i[3]&i[1]&i[0]) | (!i[14]&!i[12]&!i[6]&!i[4]
		&!i[3]&!i[2]&i[1]&i[0]) | (!i[6]&i[4]&!i[3]&i[2]&i[1]&i[0]);


endmodule


