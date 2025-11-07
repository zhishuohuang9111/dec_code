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

/* ---------------------------------------- */
// the method of handle fetch_page_fault is same as dec_i0_icaf_d ...
/* ---------------------------------------- */

module dec_decode_ctl
   import swerv_types::*;
   import ariane_pkg::*;
   import rvv_pkg::*;
(
   input logic [15:0] dec_i0_cinst_d,         // 16b compressed instruction
   input logic [15:0] dec_i1_cinst_d,

   output logic [31:0] dec_i0_inst_wb1,       // 32b instruction at wb+1 for trace encoder
   output logic [31:0] dec_i1_inst_wb1,  

   output logic [63:1] dec_i0_pc_wb1,         // 31b pc at wb+1 for trace encoder
   output logic [63:1] dec_i1_pc_wb1,  

   
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

   input logic dec_i0_fetch_page_fault,               // fetch page fault
   input logic dec_i1_fetch_page_fault,               // fetch page fault

   input logic dec_i0_icaf_d,                         // icache access fault         
   input logic dec_i1_icaf_d,                                                        
   input logic dec_i0_icaf_f1_d,                      // i0 instruction access fault at decode for f1 fetch group
   input logic dec_i0_perr_d,                         // icache parity error         
   input logic dec_i1_perr_d,                                                        
   input logic dec_i0_sbecc_d,                        // icache/iccm single-bit error
   input logic dec_i1_sbecc_d,                                                       
   input logic dec_i0_dbecc_d,                        // icache/iccm double-bit error
   input logic dec_i1_dbecc_d,

   input br_pkt_t dec_i0_brp,                         // branch packet
   input br_pkt_t dec_i1_brp,

   input logic [15:0] ifu_illegal_inst,               // 16b illegal inst from aligner

   input logic [63:1] dec_i0_pc_d,                    // pc
   input logic [63:1] dec_i1_pc_d,                    // pc
   
   input logic lsu_freeze_dc1,                        // freeze pipe: decode -> dc1
   input logic lsu_freeze_dc2,                        // freeze pipe: decode -> dc2
   input logic lsu_freeze_dc3,                        // freeze pipe: decode -> dc3
   input logic lsu_idle_any,                          // lsu idle: if fence instr & ~lsu_idle_any then stall decode
   
   input logic lsu_load_stall_any,                    // stall any store at load
   input logic lsu_store_stall_any,                   // stall any store at decode
   input logic ara_store_pending,                     // stall load and store
   //input logic dma_dccm_stall_any,                    // stall any load/store at decode

   input logic exu_div_finish,                        // div finish this cycle
   input logic exu_div_stall,                         // div executing: stall decode
   input logic [63:0] exu_div_result,                 // div result

   input logic dec_tlu_i0_kill_writeb_wb,    // I0 is flushed, don't writeback any results to arch state 
   input logic dec_tlu_i1_kill_writeb_wb,    // I1 is flushed, don't writeback any results to arch state 

   input logic dec_tlu_flush_lower_wb,          // trap lower flush
   input logic dec_tlu_flush_pause_wb,          // don't clear pause state on initial lower flush
   input logic dec_tlu_presync_d,               // CSR read needs to be presync'd
   input logic dec_tlu_postsync_d,              // CSR ops that need to be postsync'd 
   
   input logic [63:0] exu_mul_result_e3,        // multiply result

   input logic dec_i0_pc4_d,               // inst is 4B inst else 2B
   input logic dec_i1_pc4_d,
   
   input logic [63:0] dec_csr_rddata_d,    // csr read data at dec
   input logic dec_csr_legal_d,            // csr indicates legal operation

   input logic [63:0] exu_csr_rs1_e1,      // rs1 for csr instr
   
   input logic [63:0] lsu_result_dc3,      // load result
   input logic [63:0] lsu_result_corr_dc4, // corrected load result
   input logic [63:0] lsu_amo_result_e4,   // from lsu amo instruction result
 
   input logic exu_i0_flush_final,         // lower flush or i0 flush at e2
   input logic exu_i1_flush_final,         // lower flush or i1 flush at e2

   input logic [63:1] exu_i0_pc_e1,        // pcs at e1 
   input logic [63:1] exu_i1_pc_e1,  
  
   input logic [31:0] dec_i0_instr_d,      // inst at decode
   input logic [31:0] dec_i1_instr_d,

   input logic  dec_ib0_valid_d,          // inst valid at decode
   input logic  dec_ib1_valid_d,
  
   input logic [63:0] exu_i0_result_e1,    // from primary alu's
   input logic [63:0] exu_i1_result_e1,   
  
   input logic [63:0] exu_i0_result_e4,    // from secondary alu's
   input logic [63:0] exu_i1_result_e4,   

   // Added
   input logic [63:1] npc_e4,
  
   input logic  clk,                       // for rvdffe's
   input logic  active_clk,                // clk except for halt / pause
   input logic  free_clk,                  // free running clock
   
   input logic  clk_override,              // test stuff
   input logic  rst_l,


   output logic         dec_i0_rs1_en_d,   // rs1 enable at decode
   output logic         dec_i0_rs2_en_d,   
   output logic         dec_i0_rs3_en_d,

   output logic [4:0] dec_i0_rs1_d,        // rs1 logical source
   output logic [4:0] dec_i0_rs2_d,
   output logic [4:0] dec_i0_rs3_d,
  

  
   output logic [63:0] dec_i0_immed_d,     // 32b immediate data decode
  
   output logic          dec_i1_rs1_en_d,
   output logic          dec_i1_rs2_en_d,

   output logic [4:0]  dec_i1_rs1_d,
   output logic [4:0]  dec_i1_rs2_d,
  

  
   output logic [63:0] dec_i1_immed_d,

   output logic [12:1] dec_i0_br_immed_d,    // 12b branch immediate
   output logic [12:1] dec_i1_br_immed_d,   
  
   output alu_pkt_t i0_ap,                   // alu packets
   output alu_pkt_t i1_ap,

   output logic          dec_i0_decode_d,    // i0 decode
   output logic          dec_i1_decode_d,

   output logic          dec_ib0_valid_eff_d,   // effective valid taking decode into account 
   output logic          dec_ib1_valid_eff_d,

   // Added 
   output   logic        exu_i0_flush_vp_e4, 
   output   logic        exu_i1_flush_vp_e4,
  
   output logic          dec_i0_alu_decode_d,   // decode to primary alu's
   output logic          dec_i1_alu_decode_d,

  
   output logic [63:0] i0_rs1_bypass_data_d,    // i0 rs1 bypass data
   output logic [63:0] i0_rs2_bypass_data_d,    // i0 rs2 bypass data
   output logic [63:0] i0_rs3_bypass_data_d,
   output logic [63:0] i1_rs1_bypass_data_d,
   output logic [63:0] i1_rs2_bypass_data_d,
  
  
   output logic [4:0]  dec_i0_waddr_wb,         // i0 logical source to write to gpr's
   output logic          dec_i0_wen_wb,         // i0 write enable
   output logic [63:0] dec_i0_wdata_wb,         // i0 write data
  
   output logic [4:0]  dec_i1_waddr_wb,
   output logic          dec_i1_wen_wb,
   output logic [63:0] dec_i1_wdata_wb,
  
   output logic          dec_i0_select_pc_d,    // i0 select pc for rs1 - branches
   output logic          dec_i1_select_pc_d,   
  
   output logic dec_i0_rs1_bypass_en_d,         // i0 rs1 bypass enable
   output logic dec_i0_rs2_bypass_en_d,         // i0 rs2 bypass enable
   output logic dec_i0_rs3_bypass_en_d,
   output logic dec_i1_rs1_bypass_en_d,
   output logic dec_i1_rs2_bypass_en_d,

   output lsu_pkt_t    lsu_p,                   // load/store packet

   output mul_pkt_t    mul_p,                   // multiply packet

   output div_pkt_t    div_p,                   // divide packet
   
   output logic [11:0] dec_lsu_offset_d,
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
   output logic [63:0] dec_csr_wrdata_wb,   // csr write data at wb 
   output logic        dec_csr_stall_int_ff, // csr is mie/mstatus
   
   output              dec_tlu_i0_valid_e4,  // i0 valid inst at e4
   output              dec_tlu_i1_valid_e4,
               
   output              trap_pkt_t dec_tlu_packet_e4,   // trap packet
   
   output logic        dec_fence_pending, // tell TLU to stall DMA
   output logic [63:1] dec_tlu_i0_pc_e4,  // i0 trap pc
   output logic [63:1] dec_tlu_i1_pc_e4,

   output logic [31:0] dec_illegal_inst,        // illegal inst
   output logic        dec_i1_valid_e1,         // i1 valid e1
   output logic        dec_div_decode_e4,       // i0 div e4
   output logic [63:1] pred_correct_npc_e2,     // npc e2 if the prediction is correct    
   output logic        dec_i0_rs1_bypass_en_e3, // i0 rs1 bypass enables e3
   output logic        dec_i0_rs2_bypass_en_e3, // i1 rs1 bypass enables e3
   output logic        dec_i1_rs1_bypass_en_e3,
   output logic        dec_i1_rs2_bypass_en_e3,   
   output logic [63:0] i0_rs1_bypass_data_e3,   // i0 rs1 bypass data e3
   output logic [63:0] i0_rs2_bypass_data_e3,   // i1 rs1 bypass data e3
   output logic [63:0] i1_rs1_bypass_data_e3,
   output logic [63:0] i1_rs2_bypass_data_e3,
   output logic        dec_i0_sec_decode_e3,    // i0 secondary alu e3
   output logic        dec_i1_sec_decode_e3,    // i1 secondary alu e3
   output logic [63:1] dec_i0_pc_e3,            // i0 pc e3
   output logic [63:1] dec_i1_pc_e3,            // i1 pc e3

   output logic        dec_i0_rs1_bypass_en_e2, // i0 rs1 bypass enable e2
   output logic        dec_i0_rs2_bypass_en_e2, // i0 rs2 bypass enable e2
   output logic        dec_i1_rs1_bypass_en_e2,
   output logic        dec_i1_rs2_bypass_en_e2,
   output logic [63:0] i0_rs1_bypass_data_e2,   // i0 rs1 bypass data e2
   output logic [63:0] i0_rs2_bypass_data_e2,   // i0 rs2 bypass data e2
   output logic [63:0] i1_rs1_bypass_data_e2,
   output logic [63:0] i1_rs2_bypass_data_e2,

   output predict_pkt_t  i0_predict_p_d,        // i0 predict packet decode
   output predict_pkt_t  i1_predict_p_d,
   
   output logic          dec_i0_lsu_decode_d,   // i0 lsu decode
   
   output logic [63:0] i0_result_e4_to_lsu,        // i0 e4 result taking freeze into account
   output logic [63:0] i1_result_e4_to_lsu,
   output logic [63:0] i0_result_e2,            // i0 result e2

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

   input  logic mstatus_tvm_i,
   input  logic mstatus_tsr_i,
   input  logic fp_is_off_i,
   input  logic vector_is_off_i,

   output logic dec_i0_fence_i_e4_o,
   output logic dec_i0_sfence_vma_e4_o,

   output logic have_instr_inflight_before_load_o,

   //浮点扩展添加
   //---------------------fpu & vector---------------------
   output logic        dec_i0_vec_decode_e4,
   output logic [31:0] i0_inst_e4,
   input logic exu_freeze_dc4,

   output fpu_pkt_t    fpu_p,                   //浮点单元译码包
   output logic dec_i0_fpu_d,                   //i0使用fpu
   output logic dec_i1_fpu_d,                   //i1使用fpu
   output logic dec_i0_fpu_decode_e4,
   output logic dec_i1_fpu_decode_e4,
   output logic [1:0] fpu_fmt_d,                //浮点数的格式
   output logic [2:0] fpu_rm_d,                 //舍入模式
   output logic i0_rs1_arf,
   output logic i0_rs2_arf,
   output logic i1_rs1_arf,
   output logic i1_rs2_arf,

   input logic fpu_ready_i,
   input logic [4:0] fpu_flags_e4_i,

   output logic [4:0] fpu_flags_wb_o,
   output logic fpu_flags_wen_wb_o,
   output logic i0_f_arf_wen_wb,                //i0写浮点寄存器
   output logic i1_f_arf_wen_wb                //i1写浮点寄存器

);
   



   dec_pkt_t i0_dp_raw;
   dec_pkt_t i0_dp;
   dec_pkt_t i1_dp_raw;
   dec_pkt_t i1_dp;
   


   logic [31:0]        i0, i1;
   logic               i0_valid_d, i1_valid_d;

   logic [63:0]        i0_result_e1, i1_result_e1;
   logic [63:0]                      i1_result_e2;
   logic [63:0]        i0_result_e3, i1_result_e3;
   logic [63:0]        i0_result_e4, i1_result_e4;
   logic [63:0]        i0_result_wb, i1_result_wb; 

   // Added
   logic [63:0]        i0_result_vp_e1, i1_result_vp_e1;
   logic [63:0]        i0_result_vp_e2, i1_result_vp_e2;
   logic [63:0]        i0_result_vp_e3, i1_result_vp_e3;

   logic [63:0]        i0_result_vp_e4_raw, i1_result_vp_e4_raw;

   logic [63:0]        i0_result_vp_e4, i1_result_vp_e4;
   logic [63:0]        i0_result_vp_wb, i1_result_vp_wb;  

   logic [63:1]        i0_pc_e1, i1_pc_e1;
   logic [63:1]        i0_pc_e2, i1_pc_e2;
   logic [63:1]        i0_pc_e3, i1_pc_e3;
   logic [63:1]        i0_pc_e4, i1_pc_e4;

   logic [9:0]         i0_rs1bypass, i0_rs2bypass, i0_rs3bypass;
   logic [9:0]         i1_rs1bypass, i1_rs2bypass;   

   logic               i0_jalimm20, i1_jalimm20;
   logic               i0_uiimm20, i1_uiimm20;

   //logic             flush_final_e3;

   logic               lsu_decode_d;
   logic [63:0]        i0_immed_d;
   logic               i0_presync;      
   logic               i0_postsync;            

   logic               postsync_stall;
   logic               ps_stall;
 
   logic               prior_inflight, prior_inflight_e1e4, prior_inflight_wb;
   
   logic               csr_clr_d, csr_set_d, csr_write_d;
   

   logic        csr_clr_e1,csr_set_e1,csr_write_e1,csr_imm_e1;
   logic [63:0] csr_mask_e1;
   logic [63:0] write_csr_data_e1;
   logic [63:0] write_csr_data_in;
   logic [63:0] write_csr_data;
   logic               csr_data_wen;

   logic [4:0]         csrimm_e1;

   logic [63:0]        csr_rddata_e1;

   logic               flush_lower_wb;

   logic               i1_load_block_d;
   logic               i1_mul_block_d;
   logic               i1_load2_block_d;
   logic               i1_mul2_block_d;
   logic               mul_decode_d;
   logic               div_decode_d;   
   logic               fpu_decode_d;
   logic [63:1]        div_pc;
   logic               div_stall, div_stall_ff;
   logic [3:0]         div_trigger;   
   //----------------------vector & fpu---------------
   logic               i1_vec_block_d;
   logic               i1_fpu_3op_block_d;
   logic               i1_fpu2_block_d;
   logic               i1_fpu_block_d;
   logic               i1_fp_off_block_d;
   logic               i0_fpu_block_d;
   logic               i0_rs3_arf;
   //------------------------------------------- 
   
   logic               i0_legal;
   logic               shift_illegal;
   logic               illegal_inst_en;
   logic [31:0]        illegal_inst;
   logic               illegal_lockout_in, illegal_lockout;
   logic               i0_legal_decode_d;

   logic               i1_flush_final_e3;

   logic [63:0]        i0_result_e3_final, i1_result_e3_final;
   logic [63:0]        i0_result_wb_raw,   i1_result_wb_raw;
   logic [12:1] last_br_immed_d;
   logic        i1_depend_i0_d;
   logic        i0_rs1_depend_i0_e1, i0_rs1_depend_i0_e2, i0_rs1_depend_i0_e3, i0_rs1_depend_i0_e4, i0_rs1_depend_i0_wb;
   logic        i0_rs1_depend_i1_e1, i0_rs1_depend_i1_e2, i0_rs1_depend_i1_e3, i0_rs1_depend_i1_e4, i0_rs1_depend_i1_wb;   
   logic        i0_rs2_depend_i0_e1, i0_rs2_depend_i0_e2, i0_rs2_depend_i0_e3, i0_rs2_depend_i0_e4, i0_rs2_depend_i0_wb;
   logic        i0_rs2_depend_i1_e1, i0_rs2_depend_i1_e2, i0_rs2_depend_i1_e3, i0_rs2_depend_i1_e4, i0_rs2_depend_i1_wb;   
   logic        i0_rs3_depend_i0_e1, i0_rs3_depend_i0_e2, i0_rs3_depend_i0_e3, i0_rs3_depend_i0_e4, i0_rs3_depend_i0_wb;
   logic        i0_rs3_depend_i1_e1, i0_rs3_depend_i1_e2, i0_rs3_depend_i1_e3, i0_rs3_depend_i1_e4, i0_rs3_depend_i1_wb;   
   logic        i1_rs1_depend_i0_e1, i1_rs1_depend_i0_e2, i1_rs1_depend_i0_e3, i1_rs1_depend_i0_e4, i1_rs1_depend_i0_wb;
   logic        i1_rs1_depend_i1_e1, i1_rs1_depend_i1_e2, i1_rs1_depend_i1_e3, i1_rs1_depend_i1_e4, i1_rs1_depend_i1_wb;   
   logic        i1_rs2_depend_i0_e1, i1_rs2_depend_i0_e2, i1_rs2_depend_i0_e3, i1_rs2_depend_i0_e4, i1_rs2_depend_i0_wb;
   logic        i1_rs2_depend_i1_e1, i1_rs2_depend_i1_e2, i1_rs2_depend_i1_e3, i1_rs2_depend_i1_e4, i1_rs2_depend_i1_wb;   
   logic        i1_rs1_depend_i0_d, i1_rs2_depend_i0_d;

   logic        i0_secondary_d, i1_secondary_d;
   logic        i0_secondary_block_d, i1_secondary_block_d;
   logic        non_block_case_d;
   logic        i0_div_decode_d;

   logic [63:0] i0_result_e4_final, i1_result_e4_final;
   logic        i0_load_block_d;
   logic        i0_mul_block_d;
   logic [3:0]  i0_rs1_depth_d, i0_rs2_depth_d, i0_rs3_depth_d;
   logic [3:0]  i1_rs1_depth_d, i1_rs2_depth_d;   

   logic        i0_rs1_match_e1_e2, i0_rs1_match_e1_e3;
   logic        i0_rs2_match_e1_e2, i0_rs2_match_e1_e3;   
   logic        i0_rs3_match_e1_e2, i0_rs3_match_e1_e3;   
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
   logic        i0_br_error_all, i1_br_error_all;   
   logic [11:0] i0_br_offset, i1_br_offset;

   logic        freeze_before_dc1, freeze_before_dc2, freeze_before_dc3, freeze_before_dc4;

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

   // logic [63:0] i1_result_wb_eff, i0_result_wb_eff;
   logic [2:0]  i1rs1_intra, i1rs2_intra;
   logic        i1_rs1_intra_bypass, i1_rs2_intra_bypass;
   logic        store_data_bypass_c1, store_data_bypass_c2;
   logic [1:0]  store_data_bypass_e4_c1, store_data_bypass_e4_c2, store_data_bypass_e4_c3;
   logic        store_data_bypass_i0_e2_c2;
   
   class_pkt_t i0_rs1_class_d, i0_rs2_class_d, i0_rs3_class_d;
   class_pkt_t i1_rs1_class_d, i1_rs2_class_d;   

   class_pkt_t i0_dc, i0_e1c, i0_e2c, i0_e3c, i0_e4c, i0_wbc;
   class_pkt_t i1_dc, i1_e1c, i1_e2c, i1_e3c, i1_e4c, i1_wbc;


   logic i0_rs1_match_e1, i0_rs1_match_e2, i0_rs1_match_e3;
   logic i1_rs1_match_e1, i1_rs1_match_e2, i1_rs1_match_e3;   
   logic i0_rs2_match_e1, i0_rs2_match_e2, i0_rs2_match_e3;
   logic i1_rs2_match_e1, i1_rs2_match_e2, i1_rs2_match_e3;   
   logic i0_rs3_match_e1, i0_rs3_match_e2, i0_rs3_match_e3;
   
   logic       i0_secondary_stall_d;
   logic       i1_secondary_stall_d;

   logic       i0_ap_pc2, i0_ap_pc4;
   logic       i1_ap_pc2, i1_ap_pc4;   

   logic        div_wen_wb;
   logic        i0_rd_en_d;
   logic        i1_rd_en_d;
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
   logic       i0_e1_ctl_en, i0_e2_ctl_en, i0_e3_ctl_en, i0_e4_ctl_en, i0_wb_ctl_en;
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
   
   logic [63:1] i1_pc_wb;

   logic        i0_brp_valid;
   logic        nonblock_load_cancel;
   logic        lsu_idle;
   logic        csr_read_e1;
   logic        i0_block_d;
   logic        i1_block_d;
   logic        ps_stall_in;
   
   // logic        freeze_after_unfreeze1;
   // logic        freeze_after_unfreeze2;
   /* ----------------- 20220418 bypass network ------------------------- */
   logic freeze_dc3_posedge, freeze_dc3_posedge_f1;
   logic freeze_dc4_posedge;
   logic freeze_dc2_posedge, freeze_dc2_posedge_f1, freeze_dc2_posedge_f2;
   logic freeze_dc1_posedge, freeze_dc1_posedge_f1, freeze_dc1_posedge_f2, freeze_dc1_posedge_f3;

   logic exu_freeze_dc4_f1;
   logic lsu_freeze_dc3_f1, lsu_freeze_dc3_f2;
   logic lsu_freeze_dc2_f1, lsu_freeze_dc2_f2, lsu_freeze_dc2_f3;
   logic lsu_freeze_dc1_f1, lsu_freeze_dc1_f2, lsu_freeze_dc1_f3;

   logic dc1_freeze_after_unfreeze1, dc1_freeze_after_unfreeze2, dc1_freeze_after_unfreeze3;
   logic dc2_freeze_after_unfreeze1, dc2_freeze_after_unfreeze2, dc2_freeze_after_unfreeze3, dc2_freeze_after_unfreeze2_f1;
   logic dc3_freeze_after_unfreeze1, dc3_freeze_after_unfreeze2;
   logic dc1_freeze_after_dc2_unfreeze, dc1_freeze_after_dc2_unfreeze_f1, dc1_freeze_after_dc2_unfreeze_f2;
   logic dc3_freeze_after_dc1_unfreeze; // 指令在dc1阻塞后又在dc3阻塞
   logic dc3_freeze_after_dc2_unfreeze; // 指令在dc2阻塞后又在dc3阻塞

   logic dc3_unfreeze_cycle1, dc3_unfreeze_cycle2;
   logic dc2_unfreeze_cycle1, dc2_unfreeze_cycle2, dc2_unfreeze_cycle3;
   logic dc1_unfreeze_cycle1, dc1_unfreeze_cycle2, dc1_unfreeze_cycle3;
   logic instr_both_freeze_dc1_dc2, instr_both_freeze_dc1_dc2_f1, instr_both_freeze_dc1_dc2_f2;

   logic [63:0] i0_result_wb_to_freeze, i1_result_wb_to_freeze;
   logic [63:0] i0_result_wb_to_freeze_2, i1_result_wb_to_freeze_2;
   logic [63:0] i0_result_e4_freeze, i1_result_e4_freeze;
   logic [63:0] i0_result_wb_freeze, i1_result_wb_freeze;
   logic [63:0] i0_result_wb_freeze_2, i1_result_wb_freeze_2;

   // logic [63:0] i0_result_e4_freeze_for_lsu, i1_result_e4_freeze_for_lsu;
   logic [63:0] i0_result_e4_eff, i1_result_e4_eff;


   // 20220804 fix bypass bug
   logic [63:0] i0_result_wb_to_e2, i0_result_wb_to_e3, i1_result_wb_to_e2, i1_result_wb_to_e3; 

   /* ----------------- 20220418 bypass network ------------------------- */
   
   logic        tlu_wr_pause_wb1, tlu_wr_pause_wb2;

   localparam NBLOAD_SIZE     = `RV_LSU_NUM_NBLOAD;
   localparam NBLOAD_SIZE_MSB = `RV_LSU_NUM_NBLOAD-1;
   localparam NBLOAD_TAG_MSB  = `RV_LSU_NUM_NBLOAD_WIDTH-1;   

// non block load cam logic
 
   logic 	             cam_write, cam_inv_reset, cam_data_reset;
   logic [NBLOAD_TAG_MSB:0]  cam_write_tag, cam_inv_reset_tag, cam_data_reset_tag;
   logic [NBLOAD_SIZE_MSB:0] cam_wen; 

   logic [NBLOAD_TAG_MSB:0]  load_data_tag;
   logic [NBLOAD_SIZE_MSB:0] nonblock_load_write;
  
   load_cam_pkt_t [NBLOAD_SIZE_MSB:0] cam;
   load_cam_pkt_t [NBLOAD_SIZE_MSB:0] cam_in;   

   logic [4:0] nonblock_load_rd;
   logic i1_nonblock_load_stall,     i0_nonblock_load_stall;
   logic i1_nonblock_boundary_stall, i0_nonblock_boundary_stall;
   logic i0_depend_load_e1_d, i0_depend_load_e2_d;
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

   inst_t i0_itype, i1_itype;

   logic csr_read, csr_write;
   logic i0_br_unpred, i1_br_unpred;

   logic debug_fence_raw;

   logic freeze_before;
   logic freeze_e3, freeze_e4;
   logic [3:0] e4t_i0trigger;
   logic [3:0] e4t_i1trigger;
   logic       e4d_i0load;

   logic [4:0] div_waddr_wb;

   logic [12:1] last_br_immed_e1, last_br_immed_e2;
   logic [31:0]        i0_inst_d, i1_inst_d;
   logic [31:0]        i0_inst_e1, i1_inst_e1;   
   logic [31:0]        i0_inst_e2, i1_inst_e2;
   logic [31:0]        i0_inst_e3, i1_inst_e3;
   logic [31:0]        /*i0_inst_e4, */i1_inst_e4;
   logic [31:0]        i0_inst_wb, i1_inst_wb;
   logic [31:0]        i0_inst_wb1,i1_inst_wb1;   

   logic [31:0]        div_inst;
   logic [63:1] i0_pc_wb, i0_pc_wb1;
   logic [63:1]           i1_pc_wb1;   
   logic [63:1] last_pc_e2;

   reg_pkt_t i0r, i1r;

   trap_pkt_t   dt, e1t_in, e1t, e2t_in, e2t, e3t_in, e3t, e4t;


   class_pkt_t i0_e2c_in, i1_e2c_in;
   class_pkt_t i0_e3c_in, i1_e3c_in;
   class_pkt_t i0_e4c_in, i1_e4c_in;
   class_pkt_t i0_wbc_in, i1_wbc_in;

   dest_pkt_t  dd, e1d, e2d, e3d, e4d, wbd;
   dest_pkt_t e1d_in, e2d_in, e3d_in, e4d_in;

   
   assign freeze_before_dc1 = exu_freeze_dc4 | lsu_freeze_dc3 | lsu_freeze_dc2 | lsu_freeze_dc1;
   assign freeze_before_dc2 = exu_freeze_dc4 | lsu_freeze_dc3 | lsu_freeze_dc2;
   assign freeze_before_dc3 = exu_freeze_dc4 | lsu_freeze_dc3;
   assign freeze_before_dc4 = exu_freeze_dc4;

`ifdef RV_NO_SECONDARY_ALU
   assign disable_secondary = 1;
`else   
   assign disable_secondary        = dec_tlu_sec_alu_disable;
`endif
   
   
// branch prediction


   // in leak1_mode, ignore any predictions for i0, treat branch as if we haven't seen it before
   // in leak1 mode, also ignore branch errors for i0
   assign i0_brp_valid = dec_i0_brp.valid & ~leak1_mode;
   
   assign      i0_predict_p_d.misp    =  '0;
   assign      i0_predict_p_d.ataken  =  '0;
   assign      i0_predict_p_d.boffset =  '0;

   assign      i0_predict_p_d.pcall  =  i0_pcall;  // dont mark as pcall if branch error
   assign      i0_predict_p_d.pja    =  i0_pja;      
   assign      i0_predict_p_d.pret   =  i0_pret;   
   assign      i0_predict_p_d.prett[63:1] = dec_i0_brp.prett[63:1];
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
   assign      i0_br_error_all = (i0_br_error | dec_i0_brp.br_start_error) & ~leak1_mode;
   assign      i0_predict_p_d.toffset[11:0] = i0_br_offset[11:0];
   assign      i0_predict_p_d.fghr[`RV_BHT_GHR_RANGE] = dec_i0_brp.fghr[`RV_BHT_GHR_RANGE];
   assign      i0_predict_p_d.way = dec_i0_brp.way;
   

   assign      i1_predict_p_d.misp    =  '0;
   assign      i1_predict_p_d.ataken  =  '0;
   assign      i1_predict_p_d.boffset =  '0;

   assign      i1_predict_p_d.pcall  =  i1_pcall;
   assign      i1_predict_p_d.pja    =  i1_pja;      
   assign      i1_predict_p_d.pret   =  i1_pret;   
   assign      i1_predict_p_d.prett[63:1] = dec_i1_brp.prett[63:1];
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
   assign      i1_br_error_all = (i1_br_error | dec_i1_brp.br_start_error);
   assign      i1_predict_p_d.toffset[11:0] = i1_br_offset[11:0];   
   assign      i1_predict_p_d.fghr[`RV_BHT_GHR_RANGE] = dec_i1_brp.fghr[`RV_BHT_GHR_RANGE];
   assign      i1_predict_p_d.way = dec_i1_brp.way;

   //   end

   // on br error turn anything into a nop
   // on i0 instruction fetch access fault turn anything into a nop
   // nop =>   alu rs1 imm12 rd lor

   assign i0_icaf_d = dec_i0_icaf_d | dec_i0_dbecc_d;
   assign i1_icaf_d = dec_i1_icaf_d | dec_i1_dbecc_d | dec_i1_fetch_page_fault; // this signal will block the i1 pipe, the handle these exceptions in i0 pipe.
   
   assign i0_instr_error = i0_icaf_d | dec_i0_perr_d | dec_i0_sbecc_d | dec_i0_fetch_page_fault;

   always_comb begin
      i0_dp = i0_dp_raw;
      if (i0_br_error_all | i0_instr_error) begin
         i0_dp = '0;
         i0_dp.alu = 1'b1;
         i0_dp.rs1 = 1'b1;
         i0_dp.rs2 = 1'b1;
         i0_dp.lor = 1'b1;
         i0_dp.legal = 1'b1;
         i0_dp.postsync = 1'b1;
      end

      i1_dp = i1_dp_raw;
      if (i1_br_error_all) begin
         i1_dp = '0;
         i1_dp.alu = 1'b1;
         i1_dp.rs1 = 1'b1;
         i1_dp.rs2 = 1'b1;
         i1_dp.lor = 1'b1;
         i1_dp.legal = 1'b1;
         i1_dp.postsync = 1'b1;
      end
      
   end
   
   assign flush_lower_wb = dec_tlu_flush_lower_wb;
      
   assign i0[31:0] = dec_i0_instr_d[31:0];

   assign i1[31:0] = dec_i1_instr_d[31:0];   
   
   assign dec_i0_select_pc_d = i0_dp.pc;
   assign dec_i1_select_pc_d = i1_dp.pc;   

   // branches that can be predicted

   assign i0_predict_br =  i0_dp.condbr | i0_pcall | i0_pja | i0_pret;
   assign i1_predict_br =  i1_dp.condbr | i1_pcall | i1_pja | i1_pret;   
   
   assign i0_predict_nt = ~(dec_i0_brp.hist[1] & i0_brp_valid) & i0_predict_br;
   assign i0_predict_t  =  (dec_i0_brp.hist[1] & i0_brp_valid) & i0_predict_br;   

   // assign i0_ap.valid =  (i0_dc.sec | i0_dc.alu | i0_dp.alu );
   assign i0_ap.valid =  i0_dp.alu;
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

   //64位添加　32位操作
   assign i0_ap.opt_32 = i0_dp.opt_32;


   
   assign i0_ap.csr_write = i0_csr_write_only_d;
   assign i0_ap.csr_imm = i0_dp.csr_imm;
   
   
   assign i0_ap.jal    =  i0_jal;

   
   assign i0_ap_pc2 = ~dec_i0_pc4_d;
   assign i0_ap_pc4 =  dec_i0_pc4_d;
   
   assign i0_ap.predict_nt = i0_predict_nt;
   assign i0_ap.predict_t  = i0_predict_t;
   
   assign i1_predict_nt = ~(dec_i1_brp.hist[1] & dec_i1_brp.valid) & i1_predict_br;
   assign i1_predict_t  =  (dec_i1_brp.hist[1] & dec_i1_brp.valid) & i1_predict_br;   
 
   // assign i1_ap.valid =  (i1_dc.sec | i1_dc.alu | i1_dp.alu);
   assign i1_ap.valid =  i1_dp.alu;
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

   //64位添加　32位操作
   assign i1_ap.opt_32 = i1_dp.opt_32;

   assign i1_ap.csr_write = 1'b0;
   assign i1_ap.csr_imm   = 1'b0;
  
   assign i1_ap.jal    =    i1_jal;
   
   assign i1_ap_pc2 = ~dec_i1_pc4_d;
   assign i1_ap_pc4 =  dec_i1_pc4_d;
   
   assign i1_ap.predict_nt = i1_predict_nt;
   assign i1_ap.predict_t  = i1_predict_t;

   always_comb begin
      found = 0;
      cam_wen[NBLOAD_SIZE_MSB:0] = '0;
      for (int i=0; i<NBLOAD_SIZE; i++) begin
         if (~found) begin
            if (~cam[i].valid) begin
               cam_wen[i] = cam_write;
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
   
    // case of multiple loads to same dest ie. x1 ... you have to invalidate the older one  

   for (genvar i=0; i<NBLOAD_SIZE; i++) begin : cam_array

      assign cam_inv_reset_val[i] = cam_inv_reset   & (cam_inv_reset_tag[NBLOAD_TAG_MSB:0]  == cam[i].tag[NBLOAD_TAG_MSB:0]) & cam[i].valid;
      
      assign cam_data_reset_val[i] = cam_data_reset & (cam_data_reset_tag[NBLOAD_TAG_MSB:0] == cam[i].tag[NBLOAD_TAG_MSB:0]) & cam[i].valid;

      
      always_comb begin
         cam_in[i] = '0;
         
         if (cam_wen[i]) begin
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
      i0_nonblock_load_stall = i0_nonblock_boundary_stall;
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
                                       ((nonblock_load_rd[4:0]==i0r.rs2[4:0]) & lsu_nonblock_load_valid_dc3 & dec_i0_rs2_en_d);
   
   assign i1_nonblock_boundary_stall = ((nonblock_load_rd[4:0]==i1r.rs1[4:0]) & lsu_nonblock_load_valid_dc3 & dec_i1_rs1_en_d) |
                                       ((nonblock_load_rd[4:0]==i1r.rs2[4:0]) & lsu_nonblock_load_valid_dc3 & dec_i1_rs2_en_d);

   assign i0_depend_load_e1_d = ((i0_rs1_class_d.load & (i0_rs1_depth_d[3:0]==4'd1 | i0_rs1_depth_d[3:0]==4'd2)) |
                                 (i0_rs2_class_d.load & (i0_rs2_depth_d[3:0]==4'd1 | i0_rs2_depth_d[3:0]==4'd2)) |
                                 (i0_rs3_class_d.load & (i0_rs3_depth_d[3:0]==4'd1 | i0_rs3_depth_d[3:0]==4'd2))) & dec_i0_decode_d;
   
   assign i0_depend_load_e2_d = ((i0_rs1_class_d.load & (i0_rs1_depth_d[3:0]==4'd3 | i0_rs1_depth_d[3:0]==4'd4)) |
                                 (i0_rs2_class_d.load & (i0_rs2_depth_d[3:0]==4'd3 | i0_rs2_depth_d[3:0]==4'd4)) |
                                 (i0_rs3_class_d.load & (i0_rs3_depth_d[3:0]==4'd3 | i0_rs3_depth_d[3:0]==4'd4))) & dec_i0_decode_d;
   
   assign i1_depend_load_e1_d = ((i1_rs1_class_d.load & (i1_rs1_depth_d[3:0]==4'd1 | i1_rs1_depth_d[3:0]==4'd2)) |
                                 (i1_rs2_class_d.load & (i1_rs2_depth_d[3:0]==4'd1 | i1_rs2_depth_d[3:0]==4'd2))) & dec_i1_decode_d;
   
   assign i1_depend_load_e2_d = ((i1_rs1_class_d.load & (i1_rs1_depth_d[3:0]==4'd3 | i1_rs1_depth_d[3:0]==4'd4)) |
                                 (i1_rs2_class_d.load & (i1_rs2_depth_d[3:0]==4'd3 | i1_rs2_depth_d[3:0]==4'd4))) & dec_i1_decode_d;


   
   assign depend_load_e1_d = i0_depend_load_e1_d | i1_depend_load_e1_d;
   
   assign depend_load_e2_d = i0_depend_load_e2_d | i1_depend_load_e2_d;
   
   assign depend_load_same_cycle_d = i1_depend_i0_d & i0_dp.load & dec_i1_decode_d;
   
   
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
   
   assign dec_nonblock_load_freeze_dc2 = depend_load_e2_d | depend_load_e2_e1 | depend_load_same_cycle_e2;
   

// don't writeback a nonblock load

   rvdffs #(1) e4nbloadff (.*, .clk(active_clk), .en(i0_e4_ctl_en), .din(lsu_nonblock_load_valid_dc3),  .dout(nonblock_load_valid_dc4) );
   rvdffs #(1) wbnbloadff (.*, .clk(active_clk), .en(i0_wb_ctl_en), .din(    nonblock_load_valid_dc4),  .dout(nonblock_load_valid_wb) );   

   // illegal for i0load and i1load same time
   assign i0_load_kill_wen = nonblock_load_valid_wb &  wbd.i0load;
   assign i1_load_kill_wen = nonblock_load_valid_wb &  wbd.i1load;   
   

   

// end non block load cam logic
   
// pmu start
   

   assign csr_read = dec_csr_ren_d;
   assign csr_write = dec_csr_wen_unq_d;

   assign i0_br_unpred = (i0_dp.condbr | i0_dp.jal) & ~i0_predict_br;
   assign i1_br_unpred = (i1_dp.condbr | i1_dp.jal) & ~i1_predict_br;   
// always @(posedge clk) begin
//    if(i0_br_unpred) $display("%h",dec_i0_pc_d);
//    if(i1_br_unpred) $display("%h",dec_i0_pc_d+1);
// end
   // the classes must be mutually exclusive with one another
   
   always_comb begin
      i0_itype = NULL;
      i1_itype = NULL;      
   
      if (i0_legal_decode_d) begin
         if (i0_dp.mul)                  i0_itype = MUL;
         if (i0_dp.load)                 i0_itype = LOAD;
         if (i0_dp.store)                i0_itype = STORE;
         if (i0_dp.pm_alu)               i0_itype = ALU;
         if ( csr_read & ~csr_write)     i0_itype = CSRREAD;
         if (~csr_read &  csr_write)     i0_itype = CSRWRITE;
         if ( csr_read &  csr_write)     i0_itype = CSRRDWR;            
         if (i0_dp.ebreak)               i0_itype = EBREAK;
         if (i0_dp.ecall)                i0_itype = ECALL;
         if (i0_dp.fence)                i0_itype = FENCE;
         if (i0_dp.sfence_vma)           i0_itype = SFENCE_VMA;
         if (i0_dp.fence_i)              i0_itype = FENCEI;  // fencei will set this even with fence attribute
         if (i0_dp.mret)                 i0_itype = MRET;
         if (i0_dp.sret)                 i0_itype = SRET;
         if (i0_dp.condbr)               i0_itype = CONDBR;
         if (i0_dp.jal)                  i0_itype = JAL;
         if (i0_dp.dret)                 i0_itype = DRET;
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
   
   dec_dec_ctl i0_dec (.inst(i0[31:0]),.out(i0_dp_raw));
   
   dec_dec_ctl i1_dec (.inst(i1[31:0]),.out(i1_dp_raw));                                

   rvdff #(1) lsu_idle_ff (.*, .clk(active_clk), .din(lsu_idle_any), .dout(lsu_idle));
   

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

   
   
   assign i0_br_offset[11:0] = (i0_pcall_raw | i0_pja_raw) ? i0_pcall_imm[12:1] : {i0[31],i0[7],i0[30:25],i0[11:8]};

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
   assign dec_lsu_offset_d[11:0] = (i0_dp.lsu && i0_dp.amo) ? 12'b0 : (
                                   ({12{ i0_dp.lsu & i0_dp.load}} &               i0[31:20]) |
                                   ({12{~i0_dp.lsu & i1_dp.lsu & i1_dp.load}} &   i1[31:20]) |
                                   ({12{ i0_dp.lsu & i0_dp.store}} &             {i0[31:25],i0[11:7]}) |
                                   ({12{~i0_dp.lsu & i1_dp.lsu & i1_dp.store}} & {i1[31:25],i1[11:7]}));
   
   
                                   
   assign dec_i0_lsu_d = i0_dp.lsu;
   assign dec_i1_lsu_d = i1_dp.lsu;   

   assign dec_i0_mul_d = i0_dp.mul;
   assign dec_i1_mul_d = i1_dp.mul;   

   assign dec_i0_div_d = i0_dp.div;
   assign dec_i1_div_d = i1_dp.div;   
   //---------------------vector & fpu---------------------
   assign dec_i0_fpu_d = i0_dp.fpu;
   assign dec_i1_fpu_d = i1_dp.fpu;
   //------------------------------------------------

   
   assign div_p.valid = div_decode_d;

   //-----------------------vector & fpu--------------------
   assign fpu_p.valid = fpu_decode_d;
   //-------------------------------------------------

   assign div_p.unsign = (i0_dp.div) ? i0_dp.unsign :   i1_dp.unsign;
   assign div_p.rem  =   (i0_dp.div) ? i0_dp.rem    :   i1_dp.rem;
   // 新增判断是否为除法的四个W操作
   assign div_p.is_div_w = (i0_dp.div) ? i0_dp.is_div_w    :   i1_dp.is_div_w;


   assign mul_p.valid = mul_decode_d;

   assign mul_p.rs1_sign =   (i0_dp.mul) ? i0_dp.rs1_sign :   i1_dp.rs1_sign;
   assign mul_p.rs2_sign =   (i0_dp.mul) ? i0_dp.rs2_sign :   i1_dp.rs2_sign;
   assign mul_p.low      =   (i0_dp.mul) ? i0_dp.low      :   i1_dp.low;   

   // 新增判断是否为MULW
   assign mul_p.is_mulw = (i0_dp.mul) ? i0_dp.is_mulw      :   i1_dp.is_mulw;   

   assign mul_p.load_mul_rs1_bypass_e1 = load_mul_rs1_bypass_e1;
   assign mul_p.load_mul_rs2_bypass_e1 = load_mul_rs2_bypass_e1;   

   // logic [63:0] lsu_instr_pc;
   // assign lsu_instr_pc = {((i0_dp.lsu)? dec_i0_pc_d[63:1] : dec_i1_pc_d[63:1]), 1'b0};
   // assign lsu_p.is_translation = (lsu_instr_pc >= 64'h0000_0000_0001_00b0) && (lsu_instr_pc < 64'h0000_0000_8000_0000);
   assign lsu_p.valid = lsu_decode_d;

// `ifdef PITON_EH1
   assign lsu_p.load =   (i0_dp.lsu) ? (i0_dp.load) :
                           (i1_dp.load);
// `else
//    assign lsu_p.load =   (i0_dp.lsu) ? (i0_dp.load | (i0_dp.amo & ~i0_rd_en_d)) :
//                            (i1_dp.load | (i1_dp.amo & ~i0_rd_en_d));
// `endif

   assign lsu_p.store =  (i0_dp.lsu) ? i0_dp.store :  i1_dp.store;
   assign lsu_p.by =     (i0_dp.lsu) ? i0_dp.by :     i1_dp.by;
   assign lsu_p.half =   (i0_dp.lsu) ? i0_dp.half :   i1_dp.half;
   assign lsu_p.word =   (i0_dp.lsu) ? i0_dp.word :   i1_dp.word;
   assign lsu_p.dword =  (i0_dp.lsu) ? i0_dp.dword :  i1_dp.dword;
   //assign lsu_p.dword = '0;
   assign lsu_p.dma = '0;
   assign lsu_p.store_data_bypass_i0_e2_c2   = store_data_bypass_i0_e2_c2;  // has priority over all else
   assign lsu_p.load_ldst_bypass_c1 = load_ldst_bypass_c1;
   assign lsu_p.store_data_bypass_c1 = store_data_bypass_c1 & ~store_data_bypass_i0_e2_c2;
   assign lsu_p.store_data_bypass_c2 = store_data_bypass_c2 & ~store_data_bypass_i0_e2_c2;
   assign lsu_p.store_data_bypass_e4_c1[1:0] = store_data_bypass_e4_c1[1:0] & ~{2{store_data_bypass_i0_e2_c2}};
   assign lsu_p.store_data_bypass_e4_c2[1:0] = store_data_bypass_e4_c2[1:0] & ~{2{store_data_bypass_i0_e2_c2}};
   assign lsu_p.store_data_bypass_e4_c3[1:0] = store_data_bypass_e4_c3[1:0] & ~{2{store_data_bypass_i0_e2_c2}};      

   assign lsu_p.unsign = (i0_dp.lsu) ? i0_dp.unsign : i1_dp.unsign;      
   
   // defined register packet
   //---------------for atomic instruction--------------------------
// `ifdef PITON_EH1
   assign lsu_p.amo = (i0_dp.lsu) ? (i0_dp.amo) : (i1_dp.amo);
// `else
//    assign lsu_p.amo = (i0_dp.lsu) ? (i0_dp.amo & i0_rd_en_d) : (i1_dp.amo & i1_rd_en_d);
// `endif
   assign lsu_p.amo_size = (i0_dp.lsu) ? i0[13:12] : i1[13:12];
   assign lsu_p.amo_op = (i0_dp.lsu) ?    (
                                             ({4{i0_dp.amo_lr}} & AMO_LR) |
                                             ({4{i0_dp.amo_sc}} & AMO_SC) |
                                             ({4{i0_dp.amo_swap}} & AMO_SWAP) |
                                             ({4{i0_dp.amo_add}} & AMO_ADD) |
                                             ({4{i0_dp.amo_xor}} & AMO_XOR) |
                                             ({4{i0_dp.amo_and}} & AMO_AND) |
                                             ({4{i0_dp.amo_or}} & AMO_OR) |
                                             ({4{i0_dp.amo_min}} & AMO_MIN) |
                                             ({4{i0_dp.amo_max}} & AMO_MAX) |
                                             ({4{i0_dp.amo_minu}} & AMO_MINU) |
                                             ({4{i0_dp.amo_maxu}} & AMO_MAXU) 
                                          ) : (
                                             ({4{i1_dp.amo_lr}} & AMO_LR) |
                                             ({4{i1_dp.amo_sc}} & AMO_SC) |
                                             ({4{i1_dp.amo_swap}} & AMO_SWAP) |
                                             ({4{i1_dp.amo_add}} & AMO_ADD) |
                                             ({4{i1_dp.amo_xor}} & AMO_XOR) |
                                             ({4{i1_dp.amo_and}} & AMO_AND) |
                                             ({4{i1_dp.amo_or}} & AMO_OR) |
                                             ({4{i1_dp.amo_min}} & AMO_MIN) |
                                             ({4{i1_dp.amo_max}} & AMO_MAX) |
                                             ({4{i1_dp.amo_minu}} & AMO_MINU) |
                                             ({4{i1_dp.amo_maxu}} & AMO_MAXU) 
                                          );
   //---------------for atomic instruction--------------------------


   //---------------浮点扩展添加--------------------
   assign lsu_p.ls_fp = (i0_dp.lsu)? i0_dp.ls_fp : i1_dp.ls_fp;



   assign fpu_p.fmadd = (i0_dp.fpu)? i0_dp.fmadd : i1_dp.fmadd;
   assign fpu_p.fmsub = (i0_dp.fpu)? i0_dp.fmsub : i1_dp.fmsub;
   assign fpu_p.fnmsub = (i0_dp.fpu)? i0_dp.fnmsub : i1_dp.fnmsub;
   assign fpu_p.fnmadd = (i0_dp.fpu)? i0_dp.fnmadd : i1_dp.fnmadd;
   assign fpu_p.fadd = (i0_dp.fpu)? i0_dp.fadd : i1_dp.fadd;
   assign fpu_p.fsub = (i0_dp.fpu)? i0_dp.fsub : i1_dp.fsub;
   assign fpu_p.fmul = (i0_dp.fpu)? i0_dp.fmul : i1_dp.fmul;
   assign fpu_p.fdiv = (i0_dp.fpu)? i0_dp.fdiv : i1_dp.fdiv;
   assign fpu_p.fsqrt = (i0_dp.fpu)? i0_dp.fsqrt : i1_dp.fsqrt;
   assign fpu_p.fsgnj = (i0_dp.fpu)? i0_dp.fsgnj : i1_dp.fsgnj;
   assign fpu_p.fmin_max = (i0_dp.fpu)? i0_dp.fmin_max : i1_dp.fmin_max;
   assign fpu_p.fcvt_fti = (i0_dp.fpu)? i0_dp.fcvt_fti : i1_dp.fcvt_fti;
   assign fpu_p.fcvt_itf = (i0_dp.fpu)? i0_dp.fcvt_itf : i1_dp.fcvt_itf;
   assign fpu_p.mv_fti = (i0_dp.fpu)? i0_dp.mv_fti : i1_dp.mv_fti;
   assign fpu_p.mv_itf = (i0_dp.fpu)? i0_dp.mv_itf : i1_dp.mv_itf;
   assign fpu_p.fcmp = (i0_dp.fpu)? i0_dp.fcmp : i1_dp.fcmp;
   assign fpu_p.fclass = (i0_dp.fpu)? i0_dp.fclass : i1_dp.fclass;
   assign fpu_p.fcvt_dts = (i0_dp.fpu)? i0_dp.fcvt_dts : i1_dp.fcvt_dts;
   assign fpu_p.fcvt_std = (i0_dp.fpu)? i0_dp.fcvt_std : i1_dp.fcvt_std;
   assign fpu_p.is_double = (i0_dp.fpu)? i0_dp.is_double : i1_dp.is_double;

   // assign dec_fpu_rs3_d = (i0_dp.fpu)? i0[31:27] : i1[31:27];
   assign fpu_fmt_d = (i0_dp.fpu)? i0[26:25] : i1[26:25];
   assign fpu_rm_d = (i0_dp.fpu)? i0[14:12] : i1[14:12];

   //---------------添加结束------------------------


   assign i0r.rs1[4:0] = i0[19:15];
   assign i0r.rs2[4:0] = i0[24:20];
   assign i0r.rs3[4:0] = i0[31:27];
   assign i0r.rd[4:0] = i0[11:7];
   
   assign i1r.rs1[4:0] = i1[19:15];
   assign i1r.rs2[4:0] = i1[24:20];
   assign i1r.rd[4:0] = i1[11:7];
   
   
   assign dec_i0_rs1_en_d = i0_dp.rs1 & (i0_rs1_arf | (i0r.rs1[4:0] != 5'd0));  // if rs1_en=0 then read will be all 0's
   assign dec_i0_rs2_en_d = i0_dp.rs2 & (i0_rs2_arf | (i0r.rs2[4:0] != 5'd0));
   assign dec_i0_rs3_en_d = i0_dp.rs3;
   assign i0_rd_en_d =  i0_dp.rd & (dd.i0_rd_arf | (i0r.rd[4:0] != 5'd0));

   //浮点扩展添加
   //源操作数寄存器文件
   //--------------fpu & vector--------------
   assign i0_rs1_arf = (i0_dp.fpu & ~i0_dp.fcvt_itf & ~i0_dp.mv_itf) | (i0_dp.vec & i0_dp.vec_fs1);
   assign i0_rs2_arf = (i0_dp.store & i0_dp.ls_fp) | (i0_dp.fpu & ~i0_dp.fsqrt); // fsqrt only one operand

   assign i0_rs3_arf = 1'b1;  // rs3 is always float

   assign i1_rs1_arf = (~i0_dp.fpu & i1_dp.fpu & ~i1_dp.fcvt_itf & ~i1_dp.mv_itf) /*| (~i0_dp.vec & i1_dp.vec & i1_dp.vec_fs1)*/;
   assign i1_rs2_arf = (i1_dp.store & i1_dp.ls_fp) | (~i0_dp.fpu & i1_dp.fpu & ~i1_dp.fsqrt);
   //添加结束

   assign dec_i0_rs1_d[4:0] = i0r.rs1[4:0];
   assign dec_i0_rs2_d[4:0] = i0r.rs2[4:0];   
   assign dec_i0_rs3_d[4:0] = i0r.rs3[4:0];
   assign i0_rd_d[4:0] = i0r.rd[4:0];

   
   assign i0_jalimm20 = i0_dp.jal & i0_dp.imm20;   // jal 
   assign i1_jalimm20 = i1_dp.jal & i1_dp.imm20;   


   assign i0_uiimm20 = ~i0_dp.jal & i0_dp.imm20;
   assign i1_uiimm20 = ~i1_dp.jal & i1_dp.imm20;


   // csr logic

   assign dec_csr_ren_d = i0_dp.csr_read & i0_legal_decode_d;
   
   assign csr_clr_d =   i0_dp.csr_clr   & i0_legal_decode_d;
   assign csr_set_d   = i0_dp.csr_set   & i0_legal_decode_d;
   assign csr_write_d = i0_csr_write    & i0_legal_decode_d;

   assign i0_csr_write_only_d = i0_csr_write & ~i0_dp.csr_read;
   
   assign dec_csr_wen_unq_d = (i0_dp.csr_clr | i0_dp.csr_set | i0_csr_write);   // for csr legal, can't write read-only csr

   assign dec_csr_any_unq_d = any_csr_d;  


   assign dec_csr_rdaddr_d[11:0] = i0[31:20];
   assign dec_csr_wraddr_wb[11:0] = wbd.csrwaddr[11:0];


   // make sure csr doesn't write same cycle as flush_lower_wb
   // also use valid so it's flushable
   assign dec_csr_wen_wb = wbd.csrwen & wbd.i0valid & ~dec_tlu_i0_kill_writeb_wb;

   // If we are writing MIE/SIE or MSTATUS/SSTATUS, hold off the external interrupt for a cycle on the write.
   assign dec_csr_stall_int_ff = ((e4d.csrwaddr[11:0] == 12'h300) | (e4d.csrwaddr[11:0] == 12'h304) | (e4d.csrwaddr[11:0] == 12'h100) | (e4d.csrwaddr[11:0] == 12'h104)) & e4d.csrwen & e4d.i0valid & ~dec_tlu_i0_kill_writeb_wb;

   
   rvdffs #(5) csrmiscff (.*, 
                        .en(~freeze_before_dc1),
                        .clk(active_clk),
                        .din({ dec_csr_ren_d,  csr_clr_d,  csr_set_d,  csr_write_d,  i0_dp.csr_imm}),
                        .dout({csr_read_e1,    csr_clr_e1, csr_set_e1, csr_write_e1, csr_imm_e1})
                       );
   
   

   
   // perform the update operation if any

   rvdffe #(69) csr_data_e1ff (.*, .en(i0_e1_data_en), .din( {i0[19:15],dec_csr_rddata_d[63:0]}), .dout({csrimm_e1[4:0],csr_rddata_e1[63:0]}));
   
   
   assign csr_mask_e1[63:0] = ({64{ csr_imm_e1}} & {59'b0,csrimm_e1[4:0]}) |
                              ({64{~csr_imm_e1}} &  exu_csr_rs1_e1[63:0]);


   assign write_csr_data_e1[63:0] = ({64{csr_clr_e1}}   & (csr_rddata_e1[63:0] & ~csr_mask_e1[63:0])) |
                                    ({64{csr_set_e1}}   & (csr_rddata_e1[63:0] |  csr_mask_e1[63:0])) |
                                    ({64{csr_write_e1}} & (                       csr_mask_e1[63:0]));


// pause instruction
//   logic pause_state_ff;
   
   assign clear_pause = (dec_tlu_flush_lower_wb & ~dec_tlu_flush_pause_wb) | 
                        (pause_state & (write_csr_data[63:1] == 63'b0));        // if 0 or 1 then exit pause state - 1 cycle pause
   
   assign pause_state_in = (dec_tlu_wr_pause_wb | pause_state) & ~clear_pause;
                            
   rvdff #(1) pause_state_f (.*, .clk(free_clk), .din(pause_state_in), .dout(pause_state));

   
   assign dec_pause_state = pause_state;
   
   rvdff #(2) pause_state_wb_ff (.*, .clk(free_clk), .din({dec_tlu_wr_pause_wb,tlu_wr_pause_wb1}), .dout({tlu_wr_pause_wb1,tlu_wr_pause_wb2}));

   
   assign dec_pause_state_cg = pause_state & ~tlu_wr_pause_wb1 & ~tlu_wr_pause_wb2;
   
   
// end pause
   
   assign csr_data_wen = ((csr_clr_e1 | csr_set_e1 | csr_write_e1) & csr_read_e1 & ~freeze_before_dc1) | dec_tlu_wr_pause_wb | pause_state;
   
   assign write_csr_data_in[63:0] = (pause_state)         ? (write_csr_data[63:0] - 64'b1) :
                                    (dec_tlu_wr_pause_wb) ? dec_csr_wrdata_wb[63:0] : write_csr_data_e1[63:0];
                                  
   // will hold until write-back at which time the CSR will be updated while GPR is possibly written with prior CSR
   rvdffe #(64) write_csr_ff (.*, .en(csr_data_wen), .din(write_csr_data_in[63:0]), .dout(write_csr_data[63:0]));

   assign pause_stall = pause_state;
   
   // for csr write only data is produced by the alu
   assign dec_csr_wrdata_wb[63:0] = wbd.csrwonly ? i0_result_wb[63:0] : write_csr_data[63:0];
   

   
// read the csr value through rs2 immed port
   assign dec_i0_immed_d[63:0] = ({64{ i0_dp.csr_read}} & dec_csr_rddata_d[63:0]) |
                                 ({64{~i0_dp.csr_read}} & i0_immed_d[63:0]);

// end csr stuff   
      
   assign     i0_immed_d[63:0] = ({64{i0_dp.imm12}} &   { {52{i0[31]}},i0[31:20] }) |  // jalr
                                 ({64{i0_dp.shimm5}} &    {58'b0, i0[25:20]}) |
                                 ({64{i0_jalimm20}} &   { {44{i0[31]}},i0[19:12],i0[20],i0[30:21],1'b0}) |
                                 ({64{i0_uiimm20}}  &     {{32{i0[31]}},i0[31:12],12'b0 }) |
                                 ({64{i0_csr_write_only_d & i0_dp.csr_imm}} & {59'b0,i0[19:15]});  // for csr's that only write csr, dont read csr
   

//   assign dec_i0_br_immed_d[12:1] = ({12{ i0_ap.predict_nt }} &           {i0[31],i0[7],i0[30:25],i0[11:8]}) |
//                                    ({12{ i0_ap.predict_t | i0_ap.jal}} & {10'b0,i0_ap_pc4,i0_ap_pc2});           

   // all conditional branches are currently predict_nt
   // change this to generate the sequential address for all other cases for NPC requirements at commit
   assign dec_i0_br_immed_d[12:1] = (i0_ap.predict_nt & ~i0_dp.jal) ? i0_br_offset[11:0] : {10'b0,i0_ap_pc4,i0_ap_pc2};
   
   
   assign dec_i1_rs1_en_d = i1_dp.rs1 & (i1_rs1_arf | (i1r.rs1[4:0] != 5'd0));
   assign dec_i1_rs2_en_d = i1_dp.rs2 & (i1_rs2_arf | (i1r.rs2[4:0] != 5'd0));
   assign i1_rd_en_d =  i1_dp.rd & (dd.i1_rd_arf | (i1r.rd[4:0] != 5'd0));

   assign dec_i1_rs1_d[4:0] = i1r.rs1[4:0];
   assign dec_i1_rs2_d[4:0] = i1r.rs2[4:0];   
   assign i1_rd_d[4:0] = i1r.rd[4:0];


   assign dec_i1_immed_d[63:0] = ({64{i1_dp.imm12}} &   { {52{i1[31]}},i1[31:20] }) |
                                 ({64{i1_dp.shimm5}} &    {58'b0, i1[25:20]}) |
                                 ({64{i1_jalimm20}} &   { {44{i1[31]}},i1[19:12],i1[20],i1[30:21],1'b0}) |
                                 ({64{i1_uiimm20}}  &     {{32{i1[31]}},i1[31:12],12'b0 });
   

   // jal is always +2 or +4
   assign dec_i1_br_immed_d[12:1] = (i1_ap.predict_nt & ~i1_dp.jal) ? i1_br_offset[11:0] : {10'b0,i1_ap_pc4,i1_ap_pc2};
   

   
   
   assign last_br_immed_d[12:1] = (dec_i1_decode_d) ? 
                                  ((i1_ap.predict_nt) ? {10'b0,i1_ap_pc4,i1_ap_pc2} : i1_br_offset[11:0] ) :
                                  ((i0_ap.predict_nt) ? {10'b0,i0_ap_pc4,i0_ap_pc2} : i0_br_offset[11:0] );
   
   assign i0_valid_d = dec_ib0_valid_d;
   assign i1_valid_d = dec_ib1_valid_d;

   /*

   assign i0_valid_d = dec_ib0_valid_fifo;  读的时候, 用 dec_ib0_valid_fifo
   assign i1_valid_d = dec_ib1_valid_fifo;

   */


   logic vec_store_in_e1_e3;
   assign vec_store_in_e1_e3 =   (e1d.i0valid & e1d.i0store & e1d.i0vec) |
                                 (e2d.i0valid & e2d.i0store & e2d.i0vec) |
                                 (e3d.i0valid & e3d.i0store & e3d.i0vec);

   assign i0_load_stall_d = i0_dp.lsu & i0_dp.load & (lsu_load_stall_any /*| dma_dccm_stall_any*/ | ara_store_pending | vec_store_in_e1_e3);
   assign i1_load_stall_d = i1_dp.lsu & i1_dp.load & (lsu_load_stall_any /*| dma_dccm_stall_any*/ | ara_store_pending | vec_store_in_e1_e3);
   
   assign i0_store_stall_d = i0_dp.lsu & i0_dp.store & (lsu_store_stall_any /*| dma_dccm_stall_any*/ | ara_store_pending | vec_store_in_e1_e3);
   assign i1_store_stall_d = i1_dp.lsu & i1_dp.store & (lsu_store_stall_any /*| dma_dccm_stall_any*/ | ara_store_pending | vec_store_in_e1_e3);   

   assign i1_depend_i0_d = (dec_i1_rs1_en_d & i0_dp.rd & (i1_rs1_arf == dd.i0_rd_arf) & (i1r.rs1[4:0] == i0r.rd[4:0])) |
                           (dec_i1_rs2_en_d & i0_dp.rd & (i1_rs2_arf == dd.i0_rd_arf) & (i1r.rs2[4:0] == i0r.rd[4:0]));

   

   assign i1_load2_block_d = i1_dp.lsu & i0_dp.lsu;

   //------------------vector & fpu---------------
   assign i1_vec_block_d = i1_dp.vec;
   assign i0_fpu_block_d = i0_dp.fpu & ~fpu_ready_i;
   assign i1_fpu_block_d = i1_dp.fpu & ~fpu_ready_i;
   assign i1_fpu2_block_d = i0_dp.fpu & i1_dp.fpu;
   assign i1_fpu_3op_block_d = i1_dp.fpu & ~i1_dp.imm12;
   assign i1_fp_off_block_d = fp_is_off_i & (i1_rs1_arf | i1_rs2_arf | dd.i1_rd_arf);

   
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
   

   // lets make ebreak, ecall, mret, sret postsync, so break sync into pre and post

   assign presync_stall = (i0_presync & prior_inflight_eff);

   //浮点操作的同步也等待前面所有指令执行完成
   //----------------vector & fpu-------------------------
   assign prior_inflight_eff = (i0_dp.div) ? prior_inflight_e1e3 : prior_inflight;

   // to TLU to set dma_stall
   assign dec_fence_pending = (i0_valid_d & i0_dp.fence) | debug_fence;
   
   assign i0_block_d = (i0_dp.csr_read & prior_csr_write) |
                       pause_stall |
                       leak1_i0_stall |
                       dec_tlu_debug_stall |
                       postsync_stall | 
                       presync_stall  |
                       ((i0_dp.fence | debug_fence | i0_dp.amo) & ~lsu_idle) |  // add atomic instr
                       i0_nonblock_load_stall |


                       i0_load_block_d |


                       i0_mul_block_d |
                       i0_store_stall_d |
                       i0_load_stall_d |       
                       i0_secondary_stall_d |  // for performance, dont make i0 secondary if i1 not alu and depends on i0                      
                       i0_secondary_block_d |
                       i0_fpu_block_d; // i0 is fpu instruction but fpu is not ready

   assign i1_block_d = leak1_i1_stall | 
                       (i0_jal) |            // no i1 after a jal, will flush
                       ((i0_br_error_all | (|dec_i0_trigger_match_d[3:0])| ((i0_dp.condbr | i0_dp.jal) & i0_secondary_d)) & i1_dp.load) |  // if branch or branch error then don't allow i1 load
                       i0_presync | i0_postsync |
                       i1_dp.presync | i1_dp.postsync |
                       i1_icaf_d |        // instruction access fault is i0 only
                       dec_i1_perr_d |    // instruction parity error is i0 only
                       dec_i1_sbecc_d |  
                       i0_instr_error |   // i0 instr is error need flush pipeline
                       i0_dp.csr_read |
                       i0_dp.csr_write |
                       i1_dp.csr_read |
                       i1_dp.csr_write |  // optimized csr write with rd==0
                       i1_nonblock_load_stall |
                       i1_store_stall_d |


                       i1_load_block_d |    // load data not ready


                       i1_mul_block_d |     // mul data not ready
                       (i1_depend_i0_d & ~non_block_case_d & ~store_data_bypass_i0_e2_c2) | 
                       i1_load2_block_d |  // back-to-back load's at decode
                       i1_mul2_block_d |                       
                       i1_load_stall_d |  // prior stores
                       i1_secondary_block_d | // secondary alu data not ready and op is not alu
                       i1_secondary_stall_d |
                       dec_tlu_dual_issue_disable | // dual issue is disabled
                       i1_vec_block_d | // i1 can not issue a vector instruction
                       (i1_dp.lsu & i0_dp.vec & (i0_dp.load | i0_dp.store)) | // i1 not issue a ls instr when i0 is a vector ls
                       i1_fpu_block_d | // i1 is fpu but fpu is not ready
                       i1_fpu2_block_d | // back-to-back fpu instr
                       i1_fpu_3op_block_d | // i1 can not issue 3 ops fpu instr
                       i1_fp_off_block_d; // i1 need fp operands while fp extend is off, raise a exception by i0

   // all legals go here

   // block reads if there is a prior csr write in the pipeline
   assign prior_csr_write = e1d.csrwonly |
                            e2d.csrwonly |
                            e3d.csrwonly |
                            e4d.csrwonly |
                            wbd.csrwonly;
   
   
   assign any_csr_d = i0_dp.csr_read | i0_csr_write;
   
   // illegal instruction:
   // 1. decode miss 
   // 2. csr access miss
   // 3. sfence_vma when mstatus.tvm = 1
   // 4. sret when mstatus.tsr = 1
   // 5. access fp context when mstatus.fs = off
   // 6. vector instruction but mstatus.vs = off
   assign i0_legal = i0_dp.legal & 
                     (~any_csr_d | dec_csr_legal_d) & 
                     ~(i0_dp.sfence_vma & mstatus_tvm_i) &
                     ~(i0_dp.sret & mstatus_tsr_i) &
                     ~(fp_is_off_i & (i0_rs1_arf | i0_rs2_arf | dd.i0_rd_arf)) &
                     ~(vector_is_off_i & i0_dp.vec);

   // illegal inst handling

   
   assign shift_illegal = dec_i0_decode_d & ~i0_legal;
   
   assign illegal_inst_en = shift_illegal & ~illegal_lockout & ~freeze_before_dc1;

   assign illegal_inst[31:0] = (dec_i0_pc4_d) ? i0[31:0] : { 16'b0, ifu_illegal_inst[15:0] };

   rvdffe #(32) illegal_any_ff (.*, .en(illegal_inst_en), .din(illegal_inst[31:0]), .dout(dec_illegal_inst[31:0]));

   assign illegal_lockout_in = (shift_illegal | illegal_lockout) & ~flush_final_e3;
   
   rvdffs #(1) illegal_lockout_any_ff (.*, .clk(active_clk), .en(~freeze_before_dc1), .din(illegal_lockout_in), .dout(illegal_lockout));
   

   
   // allow illegals to flow down the pipe
   assign dec_i0_decode_d = i0_valid_d & ~i0_block_d & ~flush_lower_wb & ~flush_final_e3 & ~freeze_before_dc1;

   // define i0 legal decode
   assign i0_legal_decode_d = dec_i0_decode_d & i0_legal & ~freeze_before_dc1;
   
   // only decode i1 if legal and i0 not illegal - csr's cant decode as i1
   // 
   assign dec_i1_decode_d = i0_legal_decode_d & i1_valid_d & i1_dp.legal & ~i1_block_d & ~freeze_before_dc1;

   assign dec_ib0_valid_eff_d = i0_valid_d & ~dec_i0_decode_d;
   assign dec_ib1_valid_eff_d = i1_valid_d & ~dec_i1_decode_d; 

   /*


   assign dec_ib0_valid_eff_fifo = i0_valid_d & ~dec_i0_decode_d;
   assign dec_ib1_valid_eff_fifo = i1_valid_d & ~dec_i1_decode_d;   

   */  
   

   
   // performance monitor signals
   assign dec_pmu_instr_decoded[1:0] = { dec_i1_decode_d, dec_i0_decode_d };

   assign dec_pmu_decode_stall = i0_valid_d & ~dec_i0_decode_d;

   assign dec_pmu_postsync_stall = postsync_stall;
   assign dec_pmu_presync_stall  = presync_stall;   
   
   
   // illegals will postsync
   // jal's will flush, so postsync
   assign ps_stall_in =  (dec_i0_decode_d & (i0_jal | (i0_postsync) | ~i0_legal))  |
                         (dec_i1_decode_d &  i1_jal ) |
                         ((ps_stall & prior_inflight_e1e4) & ~div_wen_wb);
 
  
   rvdffs #(1) postsync_stallff (.*, .clk(free_clk), .en(~freeze_before_dc1), .din(ps_stall_in), .dout(ps_stall));
   assign postsync_stall = (ps_stall | div_stall); 

   // load from sideeffect region will be block when there is a instruction do not commit before it
   assign have_instr_inflight_before_load_o = |{e4d.i0valid,
                                                e4d.i1valid,
                                                e3d.i0valid,
                                                e3d.i1valid
                                                };
   
   assign prior_inflight_e1e3 =       |{ e1d.i0valid,
                                         e2d.i0valid,
                                         e3d.i0valid,                                    
                                         e1d.i1valid,
                                         e2d.i1valid,
                                         e3d.i1valid
                                         };
   

   
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

   assign prior_inflight = prior_inflight_e1e4 | prior_inflight_wb;
   
   assign dec_i0_alu_decode_d = i0_legal_decode_d & i0_dp.alu & ~i0_secondary_d;
   assign dec_i1_alu_decode_d = dec_i1_decode_d & i1_dp.alu & ~i1_secondary_d;

   assign dec_i0_lsu_decode_d = i0_legal_decode_d & i0_dp.lsu;
   
   assign lsu_decode_d = (i0_legal_decode_d & i0_dp.lsu) |
                         (dec_i1_decode_d & i1_dp.lsu);

   assign mul_decode_d = (i0_legal_decode_d & i0_dp.mul) |
                         (dec_i1_decode_d & i1_dp.mul);

   assign div_decode_d = (i0_legal_decode_d & i0_dp.div) |
                         (dec_i1_decode_d & i1_dp.div);

   //---------------------vector & fpu----------------------
   assign fpu_decode_d = (i0_legal_decode_d & i0_dp.fpu) |
                         (dec_i1_decode_d & i1_dp.fpu);
   //---------------------vector & fpu----------------------
   
   
   rvdffs #(2) flushff (.*, .en(~freeze_before_dc3), .clk(free_clk), .din({exu_i0_flush_final,exu_i1_flush_final}), .dout({i0_flush_final_e3, i1_flush_final_e3}));

   
   assign flush_final_e3 = i0_flush_final_e3 | i1_flush_final_e3;

   
   


// scheduling logic for primary and secondary alu's

   // arf : 判断 寄存器文件 是浮点, 还是整数的, 如果 寄存器文件相同, 才会有依赖,  不同,  则没有依赖
   
   assign i0_rs1_depend_i0_e1 = dec_i0_rs1_en_d & e1d.i0v & (i0_rs1_arf == e1d.i0_rd_arf) & (e1d.i0rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i0_e2 = dec_i0_rs1_en_d & e2d.i0v & (i0_rs1_arf == e2d.i0_rd_arf) & (e2d.i0rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i0_e3 = dec_i0_rs1_en_d & e3d.i0v & (i0_rs1_arf == e3d.i0_rd_arf) & (e3d.i0rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i0_e4 = dec_i0_rs1_en_d & e4d.i0v & (i0_rs1_arf == e4d.i0_rd_arf) & (e4d.i0rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i0_wb = dec_i0_rs1_en_d & wbd.i0v & (i0_rs1_arf == wbd.i0_rd_arf) & (wbd.i0rd[4:0] == i0r.rs1[4:0]);   
   
   assign i0_rs1_depend_i1_e1 = dec_i0_rs1_en_d & e1d.i1v & (i0_rs1_arf == e1d.i1_rd_arf) & (e1d.i1rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i1_e2 = dec_i0_rs1_en_d & e2d.i1v & (i0_rs1_arf == e2d.i1_rd_arf) & (e2d.i1rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i1_e3 = dec_i0_rs1_en_d & e3d.i1v & (i0_rs1_arf == e3d.i1_rd_arf) & (e3d.i1rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i1_e4 = dec_i0_rs1_en_d & e4d.i1v & (i0_rs1_arf == e4d.i1_rd_arf) & (e4d.i1rd[4:0] == i0r.rs1[4:0]);
   assign i0_rs1_depend_i1_wb = dec_i0_rs1_en_d & wbd.i1v & (i0_rs1_arf == wbd.i1_rd_arf) & (wbd.i1rd[4:0] == i0r.rs1[4:0]);   

   assign i0_rs2_depend_i0_e1 = dec_i0_rs2_en_d & e1d.i0v & (i0_rs2_arf == e1d.i0_rd_arf) & (e1d.i0rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i0_e2 = dec_i0_rs2_en_d & e2d.i0v & (i0_rs2_arf == e2d.i0_rd_arf) & (e2d.i0rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i0_e3 = dec_i0_rs2_en_d & e3d.i0v & (i0_rs2_arf == e3d.i0_rd_arf) & (e3d.i0rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i0_e4 = dec_i0_rs2_en_d & e4d.i0v & (i0_rs2_arf == e4d.i0_rd_arf) & (e4d.i0rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i0_wb = dec_i0_rs2_en_d & wbd.i0v & (i0_rs2_arf == wbd.i0_rd_arf) & (wbd.i0rd[4:0] == i0r.rs2[4:0]);   
   
   assign i0_rs2_depend_i1_e1 = dec_i0_rs2_en_d & e1d.i1v & (i0_rs2_arf == e1d.i1_rd_arf) & (e1d.i1rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i1_e2 = dec_i0_rs2_en_d & e2d.i1v & (i0_rs2_arf == e2d.i1_rd_arf) & (e2d.i1rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i1_e3 = dec_i0_rs2_en_d & e3d.i1v & (i0_rs2_arf == e3d.i1_rd_arf) & (e3d.i1rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i1_e4 = dec_i0_rs2_en_d & e4d.i1v & (i0_rs2_arf == e4d.i1_rd_arf) & (e4d.i1rd[4:0] == i0r.rs2[4:0]);
   assign i0_rs2_depend_i1_wb = dec_i0_rs2_en_d & wbd.i1v & (i0_rs2_arf == wbd.i1_rd_arf) & (wbd.i1rd[4:0] == i0r.rs2[4:0]);   

   assign i0_rs3_depend_i0_e1 = dec_i0_rs3_en_d & e1d.i0v & (i0_rs3_arf == e1d.i0_rd_arf) & (e1d.i0rd[4:0] == i0r.rs3[4:0]);
   assign i0_rs3_depend_i0_e2 = dec_i0_rs3_en_d & e2d.i0v & (i0_rs3_arf == e2d.i0_rd_arf) & (e2d.i0rd[4:0] == i0r.rs3[4:0]);
   assign i0_rs3_depend_i0_e3 = dec_i0_rs3_en_d & e3d.i0v & (i0_rs3_arf == e3d.i0_rd_arf) & (e3d.i0rd[4:0] == i0r.rs3[4:0]);
   assign i0_rs3_depend_i0_e4 = dec_i0_rs3_en_d & e4d.i0v & (i0_rs3_arf == e4d.i0_rd_arf) & (e4d.i0rd[4:0] == i0r.rs3[4:0]);
   assign i0_rs3_depend_i0_wb = dec_i0_rs3_en_d & wbd.i0v & (i0_rs3_arf == wbd.i0_rd_arf) & (wbd.i0rd[4:0] == i0r.rs3[4:0]);   

   assign i0_rs3_depend_i1_e1 = dec_i0_rs3_en_d & e1d.i1v & (i0_rs3_arf == e1d.i1_rd_arf) & (e1d.i1rd[4:0] == i0r.rs3[4:0]);
   assign i0_rs3_depend_i1_e2 = dec_i0_rs3_en_d & e2d.i1v & (i0_rs3_arf == e2d.i1_rd_arf) & (e2d.i1rd[4:0] == i0r.rs3[4:0]);
   assign i0_rs3_depend_i1_e3 = dec_i0_rs3_en_d & e3d.i1v & (i0_rs3_arf == e3d.i1_rd_arf) & (e3d.i1rd[4:0] == i0r.rs3[4:0]);
   assign i0_rs3_depend_i1_e4 = dec_i0_rs3_en_d & e4d.i1v & (i0_rs3_arf == e4d.i1_rd_arf) & (e4d.i1rd[4:0] == i0r.rs3[4:0]);
   assign i0_rs3_depend_i1_wb = dec_i0_rs3_en_d & wbd.i1v & (i0_rs3_arf == wbd.i1_rd_arf) & (wbd.i1rd[4:0] == i0r.rs3[4:0]);   


   assign i1_rs1_depend_i0_e1 = dec_i1_rs1_en_d & e1d.i0v & (i1_rs1_arf == e1d.i0_rd_arf) & (e1d.i0rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i0_e2 = dec_i1_rs1_en_d & e2d.i0v & (i1_rs1_arf == e2d.i0_rd_arf) & (e2d.i0rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i0_e3 = dec_i1_rs1_en_d & e3d.i0v & (i1_rs1_arf == e3d.i0_rd_arf) & (e3d.i0rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i0_e4 = dec_i1_rs1_en_d & e4d.i0v & (i1_rs1_arf == e4d.i0_rd_arf) & (e4d.i0rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i0_wb = dec_i1_rs1_en_d & wbd.i0v & (i1_rs1_arf == wbd.i0_rd_arf) & (wbd.i0rd[4:0] == i1r.rs1[4:0]);
   
   assign i1_rs1_depend_i1_e1 = dec_i1_rs1_en_d & e1d.i1v & (i1_rs1_arf == e1d.i1_rd_arf) & (e1d.i1rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i1_e2 = dec_i1_rs1_en_d & e2d.i1v & (i1_rs1_arf == e2d.i1_rd_arf) & (e2d.i1rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i1_e3 = dec_i1_rs1_en_d & e3d.i1v & (i1_rs1_arf == e3d.i1_rd_arf) & (e3d.i1rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i1_e4 = dec_i1_rs1_en_d & e4d.i1v & (i1_rs1_arf == e4d.i1_rd_arf) & (e4d.i1rd[4:0] == i1r.rs1[4:0]);
   assign i1_rs1_depend_i1_wb = dec_i1_rs1_en_d & wbd.i1v & (i1_rs1_arf == wbd.i1_rd_arf) & (wbd.i1rd[4:0] == i1r.rs1[4:0]);
   
   assign i1_rs2_depend_i0_e1 = dec_i1_rs2_en_d & e1d.i0v & (i1_rs2_arf == e1d.i0_rd_arf) & (e1d.i0rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i0_e2 = dec_i1_rs2_en_d & e2d.i0v & (i1_rs2_arf == e2d.i0_rd_arf) & (e2d.i0rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i0_e3 = dec_i1_rs2_en_d & e3d.i0v & (i1_rs2_arf == e3d.i0_rd_arf) & (e3d.i0rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i0_e4 = dec_i1_rs2_en_d & e4d.i0v & (i1_rs2_arf == e4d.i0_rd_arf) & (e4d.i0rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i0_wb = dec_i1_rs2_en_d & wbd.i0v & (i1_rs2_arf == wbd.i0_rd_arf) & (wbd.i0rd[4:0] == i1r.rs2[4:0]);
   
   assign i1_rs2_depend_i1_e1 = dec_i1_rs2_en_d & e1d.i1v & (i1_rs2_arf == e1d.i1_rd_arf) & (e1d.i1rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i1_e2 = dec_i1_rs2_en_d & e2d.i1v & (i1_rs2_arf == e2d.i1_rd_arf) & (e2d.i1rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i1_e3 = dec_i1_rs2_en_d & e3d.i1v & (i1_rs2_arf == e3d.i1_rd_arf) & (e3d.i1rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i1_e4 = dec_i1_rs2_en_d & e4d.i1v & (i1_rs2_arf == e4d.i1_rd_arf) & (e4d.i1rd[4:0] == i1r.rs2[4:0]);
   assign i1_rs2_depend_i1_wb = dec_i1_rs2_en_d & wbd.i1v & (i1_rs2_arf == wbd.i1_rd_arf) & (wbd.i1rd[4:0] == i1r.rs2[4:0]);   


   
   // rvdff #(2) freezeff (.*, .clk(active_clk), .din({freeze_before_dc3,freeze_prior1}), .dout({freeze_prior1,freeze_prior2}));

   rvdffs #(2) freeze_dc3_ff (.*, .en(~exu_freeze_dc4), .din({lsu_freeze_dc3,lsu_freeze_dc3_f1}), .dout({lsu_freeze_dc3_f1,lsu_freeze_dc3_f2}));
   rvdffs #(1) freeze_dc4_ff (.*, .en(1'b1), .din(exu_freeze_dc4), .dout(exu_freeze_dc4_f1));

   rvdffe #(3) freeze_dc2_ff (.*, .en(~exu_freeze_dc4), .din({lsu_freeze_dc2,lsu_freeze_dc2_f1,lsu_freeze_dc2_f2}), .dout({lsu_freeze_dc2_f1,lsu_freeze_dc2_f2,lsu_freeze_dc2_f3}));
   rvdffe #(3) freeze_dc1_ff (.*, .en(~exu_freeze_dc4), .din({lsu_freeze_dc1,lsu_freeze_dc1_f1,lsu_freeze_dc1_f2}), .dout({lsu_freeze_dc1_f1,lsu_freeze_dc1_f2,lsu_freeze_dc1_f3}));

   rvdffs #(1) freeze_dc3_posedge_ff1 (.*, .en(~exu_freeze_dc4), .din(freeze_dc3_posedge), .dout(freeze_dc3_posedge_f1));

   rvdffs #(1) freeze_dc2_posedge_ff1 (.*, .en(~exu_freeze_dc4), .din(freeze_dc2_posedge), .dout(freeze_dc2_posedge_f1));
   rvdffs #(1) freeze_dc2_posedge_ff2 (.*, .en(~exu_freeze_dc4), .din(freeze_dc2_posedge_f1), .dout(freeze_dc2_posedge_f2));

   rvdffs #(1) dc2_freeze_after_unfreeze2_ff (.*, .en(~exu_freeze_dc4), .din(dc2_freeze_after_unfreeze2), .dout(dc2_freeze_after_unfreeze2_f1));

   rvdffs #(1) freeze_dc1_posedge_ff1 (.*, .en(~exu_freeze_dc4), .din(freeze_dc1_posedge), .dout(freeze_dc1_posedge_f1));
   rvdffs #(1) freeze_dc1_posedge_ff2 (.*, .en(~exu_freeze_dc4 & ~lsu_freeze_dc3), .din(freeze_dc1_posedge_f1), .dout(freeze_dc1_posedge_f2));
   rvdffs #(1) freeze_dc1_posedge_ff3 (.*, .en(~exu_freeze_dc4), .din(freeze_dc1_posedge_f2), .dout(freeze_dc1_posedge_f3));

   // take snapshot of e4 and wb
   // assign freeze_after_unfreeze1 = freeze_before_dc3 & ~freeze_prior1;
   // assign freeze_after_unfreeze2 = freeze_before_dc3 & ~freeze_prior1 & ~freeze_prior2;   
   assign freeze_dc4_posedge = exu_freeze_dc4 & ~exu_freeze_dc4_f1;
   assign freeze_dc3_posedge = lsu_freeze_dc3 & ~lsu_freeze_dc3_f1;
   assign freeze_dc2_posedge = lsu_freeze_dc2 & ~lsu_freeze_dc2_f1;
   assign freeze_dc1_posedge = lsu_freeze_dc1 & ~lsu_freeze_dc1_f1;

   assign dc3_freeze_after_unfreeze1 = lsu_freeze_dc3 & ~lsu_freeze_dc3_f1 & ~lsu_freeze_dc2_f2;
   assign dc3_freeze_after_unfreeze2 = lsu_freeze_dc3 & ~lsu_freeze_dc3_f1 & ~lsu_freeze_dc3_f2;

   // instruction freeze in dc2 but not freeze in dc1 (~lsu_freeze_dc1_f2)
   assign dc2_freeze_after_unfreeze1 = lsu_freeze_dc2 & ~lsu_freeze_dc2_f1 & ~lsu_freeze_dc1_f2;
   assign dc2_freeze_after_unfreeze2 = lsu_freeze_dc2 & ~lsu_freeze_dc2_f1 & ~lsu_freeze_dc2_f2 & ~lsu_freeze_dc1_f2;
   assign dc2_freeze_after_unfreeze3 = lsu_freeze_dc2 & ~lsu_freeze_dc2_f1 & ~lsu_freeze_dc2_f2 & ~lsu_freeze_dc2_f3 & ~lsu_freeze_dc1_f2;

   // a instrction both freeze in dc1 and dc2
   assign instr_both_freeze_dc1_dc2 = lsu_freeze_dc2 & (instr_both_freeze_dc1_dc2_f1 | (~lsu_freeze_dc2_f1 & lsu_freeze_dc1_f2));
   rvdffs #(2) instr_both_freeze_dc1_dc2_ff (.*, .en(1'b1), .din({instr_both_freeze_dc1_dc2, instr_both_freeze_dc1_dc2_f1}), .dout({instr_both_freeze_dc1_dc2_f1, instr_both_freeze_dc1_dc2_f2}));

   assign dc1_freeze_after_dc2_unfreeze_f2 = lsu_freeze_dc2_f3 & lsu_freeze_dc1_f2;
   assign dc1_freeze_after_dc2_unfreeze_f1 = lsu_freeze_dc2_f2 & lsu_freeze_dc1_f1;
   assign dc3_freeze_after_dc1_unfreeze = lsu_freeze_dc3 & ~lsu_freeze_dc2_f1 & lsu_freeze_dc1_f2;
   assign dc3_freeze_after_dc2_unfreeze = lsu_freeze_dc3 & lsu_freeze_dc2_f1;

   assign dc3_unfreeze_cycle1 = ~lsu_freeze_dc3 & lsu_freeze_dc3_f1;
   assign dc3_unfreeze_cycle2 = ~lsu_freeze_dc3 & ~lsu_freeze_dc3_f1 & lsu_freeze_dc3_f2;   

   assign dc2_unfreeze_cycle1 = ~lsu_freeze_dc2 & lsu_freeze_dc2_f1;   
   assign dc2_unfreeze_cycle2 = ~lsu_freeze_dc2_f1 & lsu_freeze_dc2_f2 & ~lsu_freeze_dc3;   
   assign dc2_unfreeze_cycle3 = ~lsu_freeze_dc2_f1 & ~lsu_freeze_dc2_f2 & lsu_freeze_dc2_f3 & ~lsu_freeze_dc3_f1;   

   assign dc1_unfreeze_cycle1 = ~lsu_freeze_dc1 & lsu_freeze_dc1_f1;   
   assign dc1_unfreeze_cycle2 = ~lsu_freeze_dc1_f1 & lsu_freeze_dc1_f2 & ~lsu_freeze_dc2;   
   assign dc1_unfreeze_cycle3 = ~lsu_freeze_dc1_f2 & lsu_freeze_dc1_f3 & ~lsu_freeze_dc3 & ~lsu_freeze_dc2_f1;
   
   // rvdffe #(64) freeze_i0_e4ff (.*, .en(freeze_after_unfreeze1), .din(i0_result_e4_final[63:0]), .dout(i0_result_e4_freeze[63:0]));
   // rvdffe #(64) freeze_i1_e4ff (.*, .en(freeze_after_unfreeze1), .din(i1_result_e4_final[63:0]), .dout(i1_result_e4_freeze[63:0]));   

   rvdffe #(64) freeze_i0_e4ff (.*, .en((freeze_dc3_posedge || freeze_dc2_posedge_f1 || freeze_dc1_posedge_f2) && e4d.i0v), .din(i0_result_e4_final[63:0]), .dout(i0_result_e4_freeze[63:0]));
   rvdffe #(64) freeze_i1_e4ff (.*, .en((freeze_dc3_posedge || freeze_dc2_posedge_f1 || freeze_dc1_posedge_f2) && e4d.i1v), .din(i1_result_e4_final[63:0]), .dout(i1_result_e4_freeze[63:0]));   

   // rvdffe #(64) freeze_for_lsu_i0_e4ff (.*, .en((freeze_dc3_posedge || freeze_dc2_posedge || freeze_dc1_posedge) && e4d.i0v), .din(i0_result_e4_final[63:0]), .dout(i0_result_e4_freeze_for_lsu[63:0]));
   // rvdffe #(64) freeze_for_lsu_i1_e4ff (.*, .en((freeze_dc3_posedge || freeze_dc2_posedge || freeze_dc1_posedge) && e4d.i1v), .din(i1_result_e4_final[63:0]), .dout(i1_result_e4_freeze_for_lsu[63:0]));   
   
   // rvdffe #(64) freeze_i0_wbff (.*, 
   //                            .en(freeze_after_unfreeze1), 
   //                            .din( (freeze_after_unfreeze2) ? i0_result_wb[63:0] : i0_result_e4_freeze[63:0]), 
   //                            .dout(i0_result_wb_freeze[63:0])
   //                            );
   
   // rvdffe #(64) freeze_i1_wbff (.*, 
   //                            .en(freeze_after_unfreeze1), 
   //                            .din( (freeze_after_unfreeze2) ? i1_result_wb[63:0] : i1_result_e4_freeze[63:0]), 
   //                            .dout(i1_result_wb_freeze[63:0])
   //                            );


   // freeze_dc3_posedge for store wb to e3 bypass
   // freeze_dc2_posedge for store wb to e2 bypass
   // freeze_dc1_posedge_f1 for store wb to e2 bypass (e4 to e1)
   assign i0_result_wb_to_freeze = ((dc3_freeze_after_unfreeze2 || dc2_freeze_after_unfreeze3 || freeze_dc1_posedge_f1 || freeze_dc4_posedge) && wbd.i0v)? i0_result_wb[63:0] :
                                    (dc2_freeze_after_unfreeze2 || ((dc2_freeze_after_unfreeze1 || dc1_freeze_after_dc2_unfreeze_f1) && instr_both_freeze_dc1_dc2_f2))? i0_result_e4_freeze :
                                    (dc3_freeze_after_unfreeze1 || ((dc2_freeze_after_unfreeze1 || dc1_freeze_after_dc2_unfreeze_f1) && !instr_both_freeze_dc1_dc2_f2) || dc3_freeze_after_dc1_unfreeze || dc3_freeze_after_dc2_unfreeze)? i0_result_wb_freeze_2 :
                                    i0_result_wb_freeze;

   assign i1_result_wb_to_freeze = ((dc3_freeze_after_unfreeze2 || dc2_freeze_after_unfreeze3 || freeze_dc1_posedge_f1 || freeze_dc4_posedge) && wbd.i1v)? i1_result_wb[63:0] :
                                    (dc2_freeze_after_unfreeze2 || ((dc2_freeze_after_unfreeze1 || dc1_freeze_after_dc2_unfreeze_f1) && instr_both_freeze_dc1_dc2_f2))? i1_result_e4_freeze :
                                    (dc3_freeze_after_unfreeze1 || ((dc2_freeze_after_unfreeze1 || dc1_freeze_after_dc2_unfreeze_f1) && !instr_both_freeze_dc1_dc2_f2) || dc3_freeze_after_dc1_unfreeze || dc3_freeze_after_dc2_unfreeze)? i1_result_wb_freeze_2 :
                                    i1_result_wb_freeze;

    rvdffe #(64) freeze_i0_wbff (.*, 
                              .en(1'b1), 
                              .din(i0_result_wb_to_freeze), 
                              .dout(i0_result_wb_freeze)
                              );
   
   rvdffe #(64) freeze_i1_wbff (.*, 
                              .en(1'b1), 
                              .din(i1_result_wb_to_freeze), 
                              .dout(i1_result_wb_freeze)
                              );

   // freeze_dc3_posedge_f1 for store wb to e3 bypass (e4 to e2)
   // freeze_dc2_posedge_f1 for store wb to e3 bypass (e4 to e2) and wb to e2 (e4 to e1)
   // freeze_dc1_posedge_f2 for store wb to e3 bypass (e3 to e1)
   assign i0_result_wb_to_freeze_2 = ((dc2_freeze_after_unfreeze2_f1 || freeze_dc3_posedge_f1 || freeze_dc1_posedge_f2) && wbd.i0v) ? i0_result_wb[63:0] : 
                                       (dc2_freeze_after_unfreeze1 || dc1_freeze_after_dc2_unfreeze_f1 || dc3_freeze_after_dc2_unfreeze)? i0_result_e4_freeze[63:0] :
                                       i0_result_wb_freeze_2;

   assign i1_result_wb_to_freeze_2 = ((dc2_freeze_after_unfreeze2_f1 || freeze_dc3_posedge_f1 || freeze_dc1_posedge_f2) && wbd.i1v) ? i1_result_wb[63:0] : 
                                       (dc2_freeze_after_unfreeze1 || dc1_freeze_after_dc2_unfreeze_f1 || dc3_freeze_after_dc2_unfreeze)? i1_result_e4_freeze[63:0] :
                                       i1_result_wb_freeze_2;

    rvdffe #(64) freeze_i0_wbff_2 (.*, 
                              .en(1'b1), 
                              .din(i0_result_wb_to_freeze_2), 
                              .dout(i0_result_wb_freeze_2)
                              );
   
   rvdffe #(64) freeze_i1_wbff_2 (.*, 
                              .en(1'b1), 
                              .din(i1_result_wb_to_freeze_2), 
                              .dout(i1_result_wb_freeze_2)
                              );

   
// define bypasses for e2 stage - 1 is youngest

   assign dd.i0rs1bype2[1:0] = {  (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs1_depth_d[3:0] == 4'd5 & i0_rs1_class_d.sec,
                                  (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs1_depth_d[3:0] == 4'd6 & i0_rs1_class_d.sec };
   
   assign dd.i0rs2bype2[1:0] = {  (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs2_depth_d[3:0] == 4'd5 & i0_rs2_class_d.sec,
                                  (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs2_depth_d[3:0] == 4'd6 & i0_rs2_class_d.sec };
   
   assign dd.i1rs1bype2[1:0] = {  i1_dp.alu & i1_rs1_depth_d[3:0] == 4'd5 & i1_rs1_class_d.sec,
                                  i1_dp.alu & i1_rs1_depth_d[3:0] == 4'd6 & i1_rs1_class_d.sec };
   
   assign dd.i1rs2bype2[1:0] = {  i1_dp.alu & i1_rs2_depth_d[3:0] == 4'd5 & i1_rs2_class_d.sec,
                                  i1_dp.alu & i1_rs2_depth_d[3:0] == 4'd6 & i1_rs2_class_d.sec };

   
   // assign i1_result_wb_eff[63:0] = (unfreeze_cycle1) ? i1_result_wb_freeze[63:0] :
   //                                 (unfreeze_cycle2) ? i1_result_e4_freeze[63:0] :
   //                                 i1_result_wb[63:0];
   
   // assign i0_result_wb_eff[63:0] = (unfreeze_cycle1) ? i0_result_wb_freeze[63:0] :
   //                                 (unfreeze_cycle2) ? i0_result_e4_freeze[63:0] :
   //                                 i0_result_wb[63:0];

   // assign i1_result_wb_eff[63:0] = (dc3_unfreeze_cycle1 || dc2_unfreeze_cycle1 || dc1_unfreeze_cycle2) ? i1_result_wb_freeze[63:0] :
   //                                 (dc3_unfreeze_cycle2 || dc2_unfreeze_cycle2 || dc1_unfreeze_cycle3) ? i1_result_wb_freeze_2[63:0] :
   //                                 (dc2_unfreeze_cycle3)? i1_result_e4_freeze[63:0] :
   //                                 i1_result_wb[63:0];
                                    
   // assign i0_result_wb_eff[63:0] = (dc3_unfreeze_cycle1 || dc2_unfreeze_cycle1 || dc1_unfreeze_cycle2) ? i0_result_wb_freeze[63:0] :
   //                                 (dc3_unfreeze_cycle2 || dc2_unfreeze_cycle2 || dc1_unfreeze_cycle3) ? i0_result_wb_freeze_2[63:0] :
   //                                 (dc2_unfreeze_cycle3)? i0_result_e4_freeze[63:0] :
   //                                 i0_result_wb[63:0];


   // 20220804 fix wb data bypass to e2/e3 bug
   // if dc1 only freeze one cycle: 
   // when dc1_unfreeze_cycle3 is valid, it may be e3(wb) -> e1(e3) and e2(wb) -> d(e2)
   assign i1_result_wb_to_e3[63:0] = (dc2_unfreeze_cycle3)? i1_result_e4_freeze[63:0] :
                                     (dc3_unfreeze_cycle2 || dc2_unfreeze_cycle2 || dc1_unfreeze_cycle3) ? i1_result_wb_freeze_2[63:0] :
                                     (dc3_unfreeze_cycle1 || exu_freeze_dc4_f1)? i1_result_wb_freeze[63:0] :
                                     i1_result_wb[63:0];
                                    
   assign i0_result_wb_to_e3[63:0] = (dc2_unfreeze_cycle3)? i0_result_e4_freeze[63:0] :
                                     (dc3_unfreeze_cycle2 || dc2_unfreeze_cycle2 || dc1_unfreeze_cycle3) ? i0_result_wb_freeze_2[63:0] :
                                     (dc3_unfreeze_cycle1 || exu_freeze_dc4_f1) ? i0_result_wb_freeze[63:0] :
                                     i0_result_wb[63:0];

   assign i1_result_wb_to_e2[63:0] = (dc2_unfreeze_cycle2 && instr_both_freeze_dc1_dc2_f2)? i1_result_e4_freeze[63:0] :
                                     (dc3_unfreeze_cycle2 || (dc2_unfreeze_cycle2 && !instr_both_freeze_dc1_dc2_f2))? i1_result_wb_freeze_2[63:0] :
                                     (dc3_unfreeze_cycle1 || dc2_unfreeze_cycle1 || dc1_unfreeze_cycle2 || exu_freeze_dc4_f1)? i1_result_wb_freeze[63:0] :
                                     i1_result_wb[63:0];
   assign i0_result_wb_to_e2[63:0] = (dc2_unfreeze_cycle2 && instr_both_freeze_dc1_dc2_f2)? i0_result_e4_freeze[63:0] :
                                     (dc3_unfreeze_cycle2 || (dc2_unfreeze_cycle2 && !instr_both_freeze_dc1_dc2_f2))? i0_result_wb_freeze_2[63:0] :
                                     (dc3_unfreeze_cycle1 || dc2_unfreeze_cycle1 || dc1_unfreeze_cycle2 || exu_freeze_dc4_f1)? i0_result_wb_freeze[63:0] :
                                     i0_result_wb[63:0];
   

   assign i0_rs1_bypass_data_e2[63:0] = ({64{e2d.i0rs1bype2[1]}} & i1_result_wb_to_e2[63:0]) |
                                        ({64{e2d.i0rs1bype2[0]}} & i0_result_wb_to_e2[63:0]);

   assign i0_rs2_bypass_data_e2[63:0] = ({64{e2d.i0rs2bype2[1]}} & i1_result_wb_to_e2[63:0]) |
                                        ({64{e2d.i0rs2bype2[0]}} & i0_result_wb_to_e2[63:0]);

   assign i1_rs1_bypass_data_e2[63:0] = ({64{e2d.i1rs1bype2[1]}} & i1_result_wb_to_e2[63:0]) |
                                        ({64{e2d.i1rs1bype2[0]}} & i0_result_wb_to_e2[63:0]);

   assign i1_rs2_bypass_data_e2[63:0] = ({64{e2d.i1rs2bype2[1]}} & i1_result_wb_to_e2[63:0]) |
                                        ({64{e2d.i1rs2bype2[0]}} & i0_result_wb_to_e2[63:0]);


   assign dec_i0_rs1_bypass_en_e2 = |e2d.i0rs1bype2[1:0];
   assign dec_i0_rs2_bypass_en_e2 = |e2d.i0rs2bype2[1:0];   
   assign dec_i1_rs1_bypass_en_e2 = |e2d.i1rs1bype2[1:0];
   assign dec_i1_rs2_bypass_en_e2 = |e2d.i1rs2bype2[1:0];   
   


   
// define bypasses for e3 stage before secondary alu's
   

   assign i1_rs1_depend_i0_d = dec_i1_rs1_en_d & i0_dp.rd & (i1_rs1_arf == dd.i0_rd_arf) & (i1r.rs1[4:0] == i0r.rd[4:0]);
   assign i1_rs2_depend_i0_d = dec_i1_rs2_en_d & i0_dp.rd & (i1_rs2_arf == dd.i0_rd_arf) & (i1r.rs2[4:0] == i0r.rd[4:0]);
   

// i0   
   assign dd.i0rs1bype3[3:0] = { (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs1_depth_d[3:0]==4'd1 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                 (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs1_depth_d[3:0]==4'd2 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                 (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs1_depth_d[3:0]==4'd3 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                 (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs1_depth_d[3:0]==4'd4 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul) };
   
   assign dd.i0rs2bype3[3:0] = { (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs2_depth_d[3:0]==4'd1 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul), 
                                 (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs2_depth_d[3:0]==4'd2 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                 (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs2_depth_d[3:0]==4'd3 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                 (i0_dp.alu | i0_dp.vec | i0_dp.sfence_vma) & i0_rs2_depth_d[3:0]==4'd4 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul) };
   
// i1
   
   assign i1rs1_intra[2:0] = {   i1_dp.alu & i0_dp.alu  & i1_rs1_depend_i0_d,
                                 i1_dp.alu & i0_dp.mul  & i1_rs1_depend_i0_d,
                                 i1_dp.alu & i0_dp.load & i1_rs1_depend_i0_d
                                 };

   assign i1rs2_intra[2:0] = {   i1_dp.alu & i0_dp.alu  & i1_rs2_depend_i0_d,
                                 i1_dp.alu & i0_dp.mul  & i1_rs2_depend_i0_d,
                                 i1_dp.alu & i0_dp.load & i1_rs2_depend_i0_d
                                 };
   
   assign i1_rs1_intra_bypass = |i1rs1_intra[2:0];

   assign i1_rs2_intra_bypass = |i1rs2_intra[2:0];
   

   assign dd.i1rs1bype3[6:0] = { i1rs1_intra[2:0],
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd1 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass, 
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd2 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass,
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd3 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass,
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd4 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass };
   
   assign dd.i1rs2bype3[6:0] = { i1rs2_intra[2:0],
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd1 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass, 
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd2 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass,
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd3 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass,
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd4 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass };
   

   

   assign dec_i0_rs1_bypass_en_e3 = |e3d.i0rs1bype3[3:0];
   assign dec_i0_rs2_bypass_en_e3 = |e3d.i0rs2bype3[3:0];
   assign dec_i1_rs1_bypass_en_e3 = |e3d.i1rs1bype3[6:0];
   assign dec_i1_rs2_bypass_en_e3 = |e3d.i1rs2bype3[6:0];   



   


   
   // assign i1_result_e4_eff[63:0] = (freeze_prior1) ? i1_result_e4_freeze[63:0] :
   //                                 i1_result_e4_final[63:0];
   
   // assign i0_result_e4_eff[63:0] = (freeze_prior1) ? i0_result_e4_freeze[63:0] :
   //                                 i0_result_e4_final[63:0];
   
   assign i1_result_e4_eff[63:0] = (lsu_freeze_dc3_f1 || lsu_freeze_dc2_f2 || lsu_freeze_dc1_f3) ? i1_result_e4_freeze[63:0] :
                                   i1_result_e4_final[63:0];
   
   assign i0_result_e4_eff[63:0] = (lsu_freeze_dc3_f1 || lsu_freeze_dc2_f2 || lsu_freeze_dc1_f3) ? i0_result_e4_freeze[63:0] :
                                   i0_result_e4_final[63:0];

   // assign i1_result_e4_to_lsu = (lsu_freeze_dc3_f1 || lsu_freeze_dc2_f1 || lsu_freeze_dc1_f1)? i1_result_e4_freeze_for_lsu[63:0] : 
   //                               i1_result_e4_final[63:0];

   // assign i0_result_e4_to_lsu = (lsu_freeze_dc3_f1 || lsu_freeze_dc2_f1 || lsu_freeze_dc1_f1)? i0_result_e4_freeze_for_lsu[63:0] : 
   //                               i0_result_e4_final[63:0];

   assign i1_result_e4_to_lsu = i1_result_e4_final;
   assign i0_result_e4_to_lsu = i0_result_e4_final;

   assign i0_rs1_bypass_data_e3[63:0] = ({64{e3d.i0rs1bype3[3]}} & i1_result_e4_eff[63:0]) |
                                        ({64{e3d.i0rs1bype3[2]}} & i0_result_e4_eff[63:0]) |
                                        ({64{e3d.i0rs1bype3[1]}} & i1_result_wb_to_e3[63:0]) |
                                        ({64{e3d.i0rs1bype3[0]}} & i0_result_wb_to_e3[63:0]);
   
   assign i0_rs2_bypass_data_e3[63:0] = ({64{e3d.i0rs2bype3[3]}} & i1_result_e4_eff[63:0]) |
                                        ({64{e3d.i0rs2bype3[2]}} & i0_result_e4_eff[63:0]) |
                                        ({64{e3d.i0rs2bype3[1]}} & i1_result_wb_to_e3[63:0]) |
                                        ({64{e3d.i0rs2bype3[0]}} & i0_result_wb_to_e3[63:0]);

   assign i1_rs1_bypass_data_e3[63:0] = ({64{e3d.i1rs1bype3[6]}} & i0_result_e3[63:0]) |
                                        ({64{e3d.i1rs1bype3[5]}} & exu_mul_result_e3[63:0]) |
                                        ({64{e3d.i1rs1bype3[4]}} & lsu_result_dc3[63:0]) |
                                        ({64{e3d.i1rs1bype3[3]}} & i1_result_e4_eff[63:0]) |
                                        ({64{e3d.i1rs1bype3[2]}} & i0_result_e4_eff[63:0]) |
                                        ({64{e3d.i1rs1bype3[1]}} & i1_result_wb_to_e3[63:0]) |
                                        ({64{e3d.i1rs1bype3[0]}} & i0_result_wb_to_e3[63:0]);
   

   assign i1_rs2_bypass_data_e3[63:0] = ({64{e3d.i1rs2bype3[6]}} & i0_result_e3[63:0]) |
                                        ({64{e3d.i1rs2bype3[5]}} & exu_mul_result_e3[63:0]) |
                                        ({64{e3d.i1rs2bype3[4]}} & lsu_result_dc3[63:0]) |
                                        ({64{e3d.i1rs2bype3[3]}} & i1_result_e4_eff[63:0]) |
                                        ({64{e3d.i1rs2bype3[2]}} & i0_result_e4_eff[63:0]) |
                                        ({64{e3d.i1rs2bype3[1]}} & i1_result_wb_to_e3[63:0]) |
                                        ({64{e3d.i1rs2bype3[0]}} & i0_result_wb_to_e3[63:0]);
   

   

// order the producers as follows:  i1_e1 - 1, i0_e1 - 2, i1_e2 - 3, ..., i1_wb - 9, i0_wb - 10

   
   assign {i0_rs1_class_d, i0_rs1_depth_d[3:0]} = 
                                                  (i0_rs1_depend_i1_e1) ? { i1_e1c, 4'd1 } :
                                                  (i0_rs1_depend_i0_e1) ? { i0_e1c, 4'd2 } :
                                                  (i0_rs1_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i0_rs1_depend_i0_e2) ? { i0_e2c, 4'd4 } :
                                                  (i0_rs1_depend_i1_e3) ? { i1_e3c, 4'd5 } :
                                                  (i0_rs1_depend_i0_e3) ? { i0_e3c, 4'd6 } : 
                                                  (i0_rs1_depend_i1_e4) ? { i1_e4c, 4'd7 } :
                                                  (i0_rs1_depend_i0_e4) ? { i0_e4c, 4'd8 } :  
                                                  (i0_rs1_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i0_rs1_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0;
   
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

   assign {i0_rs3_class_d, i0_rs3_depth_d[3:0]} = 
                                                  (i0_rs3_depend_i1_e1) ? { i1_e1c, 4'd1 } :
                                                  (i0_rs3_depend_i0_e1) ? { i0_e1c, 4'd2 } :
                                                  (i0_rs3_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i0_rs3_depend_i0_e2) ? { i0_e2c, 4'd4 } :
                                                  (i0_rs3_depend_i1_e3) ? { i1_e3c, 4'd5 } :
                                                  (i0_rs3_depend_i0_e3) ? { i0_e3c, 4'd6 } : 
                                                  (i0_rs3_depend_i1_e4) ? { i1_e4c, 4'd7 } :
                                                  (i0_rs3_depend_i0_e4) ? { i0_e4c, 4'd8 } :  
                                                  (i0_rs3_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i0_rs3_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0;
   
   assign {i1_rs1_class_d, i1_rs1_depth_d[3:0]} = 
                                                  (i1_rs1_depend_i1_e1) ? { i1_e1c, 4'd1 } :
                                                  (i1_rs1_depend_i0_e1) ? { i0_e1c, 4'd2 } :
                                                  (i1_rs1_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i1_rs1_depend_i0_e2) ? { i0_e2c, 4'd4 } :
                                                  (i1_rs1_depend_i1_e3) ? { i1_e3c, 4'd5 } :
                                                  (i1_rs1_depend_i0_e3) ? { i0_e3c, 4'd6 } : 
                                                  (i1_rs1_depend_i1_e4) ? { i1_e4c, 4'd7 } :
                                                  (i1_rs1_depend_i0_e4) ? { i0_e4c, 4'd8 } :  
                                                  (i1_rs1_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i1_rs1_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0;
   
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

   
   assign i0_rs1_match_e1 = (i0_rs1_depth_d[3:0] == 4'd1 | 
                             i0_rs1_depth_d[3:0] == 4'd2);
   
   assign i0_rs1_match_e2 = (i0_rs1_depth_d[3:0] == 4'd3 | 
                             i0_rs1_depth_d[3:0] == 4'd4);

   assign i0_rs1_match_e3 = (i0_rs1_depth_d[3:0] == 4'd5 | 
                             i0_rs1_depth_d[3:0] == 4'd6);

   assign i0_rs2_match_e1 = (i0_rs2_depth_d[3:0] == 4'd1 | 
                             i0_rs2_depth_d[3:0] == 4'd2);
   
   assign i0_rs2_match_e2 = (i0_rs2_depth_d[3:0] == 4'd3 | 
                             i0_rs2_depth_d[3:0] == 4'd4);

   assign i0_rs2_match_e3 = (i0_rs2_depth_d[3:0] == 4'd5 | 
                             i0_rs2_depth_d[3:0] == 4'd6);

   assign i0_rs3_match_e1 = (i0_rs3_depth_d[3:0] == 4'd1 | 
                             i0_rs3_depth_d[3:0] == 4'd2);
   
   assign i0_rs3_match_e2 = (i0_rs3_depth_d[3:0] == 4'd3 | 
                             i0_rs3_depth_d[3:0] == 4'd4);

   assign i0_rs3_match_e3 = (i0_rs3_depth_d[3:0] == 4'd5 | 
                             i0_rs3_depth_d[3:0] == 4'd6);

   assign i0_rs1_match_e1_e2 = i0_rs1_match_e1 | i0_rs1_match_e2;
   assign i0_rs1_match_e1_e3 = i0_rs1_match_e1 | i0_rs1_match_e2 | i0_rs1_match_e3;
   
   assign i0_rs2_match_e1_e2 = i0_rs2_match_e1 | i0_rs2_match_e2;
   assign i0_rs2_match_e1_e3 = i0_rs2_match_e1 | i0_rs2_match_e2 | i0_rs2_match_e3;

   assign i0_rs3_match_e1_e2 = i0_rs3_match_e1 | i0_rs3_match_e2;
   assign i0_rs3_match_e1_e3 = i0_rs3_match_e1 | i0_rs3_match_e2 | i0_rs3_match_e3;
   
   
   
   

   assign i0_secondary_d = ((i0_dp.alu & (i0_rs1_class_d.load | i0_rs1_class_d.mul) & i0_rs1_match_e1_e2) |
                            (i0_dp.alu & (i0_rs2_class_d.load | i0_rs2_class_d.mul) & i0_rs2_match_e1_e2) |
                            (i0_dp.alu & i0_rs1_class_d.sec & i0_rs1_match_e1_e3) |                      
                            (i0_dp.alu & i0_rs2_class_d.sec & i0_rs2_match_e1_e3)) & ~disable_secondary;

  // stall i0 until it's not a secondary for performance 
   assign i0_secondary_stall_d = ((i0_dp.alu & i1_rs1_depend_i0_d & ~i1_dp.alu & i0_secondary_d) |
                                  (i0_dp.alu & i1_rs2_depend_i0_d & ~i1_dp.alu & ~i1_dp.store & i0_secondary_d)) & ~disable_secondary;
   


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
   
   

                           
  // cyh: if the depend instruction will product a result in next cycle then we not use secondary but
  // stall pipeline and use E1 ALU in the next cycle
   assign i1_secondary_d = ((i1_dp.alu & (i1_rs1_class_d.load | i1_rs1_class_d.mul) & i1_rs1_match_e1) |
                            (i1_dp.alu & (i1_rs2_class_d.load | i1_rs2_class_d.mul) & i1_rs2_match_e1) |
                            (i1_dp.alu & (i1_rs1_class_d.sec) & i1_rs1_match_e1_e2) |
                            (i1_dp.alu & (i1_rs2_class_d.sec) & i1_rs2_match_e1_e2) |
                            (non_block_case_d & i1_depend_i0_d)) & ~disable_secondary;                     
                           

   assign i1_secondary_stall_d = ((i1_dp.alu & (i1_rs1_class_d.load | i1_rs1_class_d.mul) & i1_rs1_match_e2) |
                                  (i1_dp.alu & (i1_rs2_class_d.load | i1_rs2_class_d.mul) & i1_rs2_match_e2) |
                                  (i1_dp.alu & i1_rs1_class_d.sec & i1_rs1_match_e3) |
                                  (i1_dp.alu & i1_rs2_class_d.sec & i1_rs2_match_e3)) & ~disable_secondary;
   
   
   assign store_data_bypass_i0_e2_c2 = i0_dp.alu & ~i0_secondary_d & i1_rs2_depend_i0_d & ~i1_rs1_depend_i0_d & i1_dp.store;

   assign non_block_case_d = (  // (i1_dp.alu & i0_dp.alu & ~i0_secondary_d) | - not a good idea, bad for performance
                                (i1_dp.alu & i0_dp.load) |
                                (i1_dp.alu & i0_dp.mul)
                                ) & ~disable_secondary;
   

   

   assign store_data_bypass_c2 =  ((             i0_dp.store & i0_rs2_depth_d[3:0] == 4'd1 & i0_rs2_class_d.load) |
                              (             i0_dp.store & i0_rs2_depth_d[3:0] == 4'd2 & i0_rs2_class_d.load) |
                              (~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd1 & i1_rs2_class_d.load) |
                              (~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd2 & i1_rs2_class_d.load));
   
   assign store_data_bypass_c1 =  ((             i0_dp.store & i0_rs2_depth_d[3:0] == 4'd3 & i0_rs2_class_d.load) |
                              (             i0_dp.store & i0_rs2_depth_d[3:0] == 4'd4 & i0_rs2_class_d.load) |
                              (~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd3 & i1_rs2_class_d.load) |
                              (~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd4 & i1_rs2_class_d.load));
   
   assign load_ldst_bypass_c1 =  ((         (i0_dp.load | i0_dp.store) & i0_rs1_depth_d[3:0] == 4'd3 & i0_rs1_class_d.load) |
                              (             (i0_dp.load | i0_dp.store) & i0_rs1_depth_d[3:0] == 4'd4 & i0_rs1_class_d.load) |
                              (~i0_dp.lsu & (i1_dp.load | i1_dp.store) & i1_rs1_depth_d[3:0] == 4'd3 & i1_rs1_class_d.load) |
                              (~i0_dp.lsu & (i1_dp.load | i1_dp.store) & i1_rs1_depth_d[3:0] == 4'd4 & i1_rs1_class_d.load));
   
   assign load_mul_rs1_bypass_e1 =  ((             (i0_dp.mul) & i0_rs1_depth_d[3:0] == 4'd3 & i0_rs1_class_d.load) |
                                     (             (i0_dp.mul) & i0_rs1_depth_d[3:0] == 4'd4 & i0_rs1_class_d.load) |
                                     (~i0_dp.mul & (i1_dp.mul) & i1_rs1_depth_d[3:0] == 4'd3 & i1_rs1_class_d.load) |
                                     (~i0_dp.mul & (i1_dp.mul) & i1_rs1_depth_d[3:0] == 4'd4 & i1_rs1_class_d.load));
   
   assign load_mul_rs2_bypass_e1 =  ((             (i0_dp.mul) & i0_rs2_depth_d[3:0] == 4'd3 & i0_rs2_class_d.load) |
                                     (             (i0_dp.mul) & i0_rs2_depth_d[3:0] == 4'd4 & i0_rs2_class_d.load) |
                                     (~i0_dp.mul & (i1_dp.mul) & i1_rs2_depth_d[3:0] == 4'd3 & i1_rs2_class_d.load) |
                                     (~i0_dp.mul & (i1_dp.mul) & i1_rs2_depth_d[3:0] == 4'd4 & i1_rs2_class_d.load));
   

   assign store_data_bypass_e4_c3[1:0] = {
                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd1 & (i1_rs2_class_d.sec | i1_rs2_class_d.mul) ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd1 & (i0_rs2_class_d.sec | i0_rs2_class_d.mul) ),

                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd2 & (i1_rs2_class_d.sec | i1_rs2_class_d.mul) ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd2 & (i0_rs2_class_d.sec | i0_rs2_class_d.mul) )
                                     };

   assign store_data_bypass_e4_c2[1:0] = {
                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd3 & (i1_rs2_class_d.sec | i1_rs2_class_d.mul) ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd3 & (i0_rs2_class_d.sec | i0_rs2_class_d.mul) ),

                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd4 & (i1_rs2_class_d.sec | i1_rs2_class_d.mul) ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd4 & (i0_rs2_class_d.sec | i0_rs2_class_d.mul) )
                                     };


   assign store_data_bypass_e4_c1[1:0] = {
                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd5 & (i1_rs2_class_d.sec | i1_rs2_class_d.mul) ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd5 & (i0_rs2_class_d.sec | i0_rs2_class_d.mul) ),

                                      ( ~i0_dp.lsu & i1_dp.store & i1_rs2_depth_d[3:0] == 4'd6 & (i1_rs2_class_d.sec | i1_rs2_class_d.mul) ) |
                                      (              i0_dp.store & i0_rs2_depth_d[3:0] == 4'd6 & (i0_rs2_class_d.sec | i0_rs2_class_d.mul) )
                                     };


   
   assign i0_not_alu_eff = (~i0_dp.alu | disable_secondary);
   assign i1_not_alu_eff = (~i1_dp.alu | disable_secondary);


   logic i0_rs1_match_e1_i0, i0_rs1_match_e1_i1;
   logic i0_rs2_match_e1_i0, i0_rs2_match_e1_i1;
   logic i0_rs3_match_e1_i0, i0_rs3_match_e1_i1;

   logic i0_rs1_match_e2_i0, i0_rs1_match_e2_i1;
   logic i0_rs2_match_e2_i0, i0_rs2_match_e2_i1;
   logic i0_rs3_match_e2_i0, i0_rs3_match_e2_i1;


   logic i1_rs1_match_e1_i0, i1_rs1_match_e1_i1;
   logic i1_rs2_match_e1_i0, i1_rs2_match_e1_i1;

   logic i1_rs1_match_e2_i0, i1_rs1_match_e2_i1;
   logic i1_rs2_match_e2_i0, i1_rs2_match_e2_i1;

   // 只有 i0 有 rs3 
   assign i0_rs1_match_e1_i1 = i0_rs1_depth_d[3:0] == 4'd1;      // 1：i1 e1   2: i0 e1
   assign i0_rs2_match_e1_i1 = i0_rs2_depth_d[3:0] == 4'd1;      // 1：i1 e1   2: i0 e1
   assign i0_rs3_match_e1_i1 = i0_rs3_depth_d[3:0] == 4'd1;      

   assign i0_rs1_match_e1_i0 = i0_rs1_depth_d[3:0] == 4'd2;      // 1：i1 e1   2: i0 e1
   assign i0_rs2_match_e1_i0 = i0_rs2_depth_d[3:0] == 4'd2;      // 1：i1 e1   2: i0 e1
   assign i0_rs3_match_e1_i0 = i0_rs3_depth_d[3:0] == 4'd2;

   assign i0_rs1_match_e2_i1 = i0_rs1_depth_d[3:0] == 4'd3;      
   assign i0_rs2_match_e2_i1 = i0_rs2_depth_d[3:0] == 4'd3; 
   assign i0_rs3_match_e2_i1 = i0_rs3_depth_d[3:0] == 4'd3;       

   assign i0_rs1_match_e2_i0 = i0_rs1_depth_d[3:0] == 4'd4;      
   assign i0_rs2_match_e2_i0 = i0_rs2_depth_d[3:0] == 4'd4;   
   assign i0_rs3_match_e2_i0 = i0_rs3_depth_d[3:0] == 4'd4;     
  


   assign i1_rs1_match_e1_i1 = i1_rs1_depth_d[3:0] == 4'd1;      // 1：i1 e1   2: i0 e1
   assign i1_rs2_match_e1_i1 = i1_rs2_depth_d[3:0] == 4'd1;      // 1：i1 e1   2: i0 e1

   assign i1_rs1_match_e1_i0 = i1_rs1_depth_d[3:0] == 4'd2;      // 1：i1 e1   2: i0 e1
   assign i1_rs2_match_e1_i0 = i1_rs2_depth_d[3:0] == 4'd2;      // 1：i1 e1   2: i0 e1
   

   assign i1_rs1_match_e2_i1 = i1_rs1_depth_d[3:0] == 4'd3;     
   assign i1_rs2_match_e2_i1 = i1_rs2_depth_d[3:0] == 4'd3;     

   assign i1_rs1_match_e2_i0 = i1_rs1_depth_d[3:0] == 4'd4;      
   assign i1_rs2_match_e2_i0 = i1_rs2_depth_d[3:0] == 4'd4;      


   logic i0_load_match_e1_i0, i1_load_match_e1_i0;
   logic i0_load_match_e1_i1, i1_load_match_e1_i1;  

   logic i0_load_match_e2_i1, i0_load_match_e2_i0;
   logic i1_load_match_e2_i1, i1_load_match_e2_i0;

   //  dec 阶段的 i0 指令 依赖 e1, e2 阶段的 i0, i1               
   assign i0_load_match_e1_i0 = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e1_i0) |
                                (i0_not_alu_eff & i0_rs2_class_d.load &    i0_rs2_match_e1_i0 & ~i0_dp.store) |

                                (i0_rs3_class_d.load & i0_rs3_match_e1_i0 & i0_dp.fpu);  // dec i0, match 的 e1 包含 i0

   assign i0_load_match_e1_i1 = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e1_i1) |
                                (i0_not_alu_eff & i0_rs2_class_d.load &   i0_rs2_match_e1_i1 & ~i0_dp.store) |

                                (i0_rs3_class_d.load & i0_rs3_match_e1_i1 & i0_dp.fpu);  //dec i0, match 的 e1 包含 i1                         
    

   assign i0_load_match_e2_i0 = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e2_i0 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) |
                                (i0_not_alu_eff & i0_rs2_class_d.load &    i0_rs2_match_e2_i0 & ~i0_dp.store & ~i0_dp.mul) | 

                                (i0_rs3_class_d.load & i0_rs3_match_e2_i0 & i0_dp.fpu);  //dec i0, match 的 e2 包含 i0                          

         //  匹配到 e2 阶段 i1
   assign i0_load_match_e2_i1 = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e2_i1 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) |
                                (i0_not_alu_eff & i0_rs2_class_d.load &   i0_rs2_match_e2_i1 & ~i0_dp.store & ~i0_dp.mul) | 

                                (i0_rs3_class_d.load & i0_rs3_match_e2_i1 & i0_dp.fpu);   //dec i0, match 的 e2 包含 i1


   //  dec 阶段的 i1 指令 依赖 e1, e2 阶段的 i0, i1   
   assign i1_load_match_e1_i0 = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e1_i0) |
                                (i1_not_alu_eff & i1_rs2_class_d.load &      i1_rs2_match_e1_i0 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i0  

            //  匹配到 e1 阶段 i1
   assign i1_load_match_e1_i1 = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e1_i1) |
                                (i1_not_alu_eff & i1_rs2_class_d.load &      i1_rs2_match_e1_i1 & ~i1_dp.store);  //dec i1, match 的 e1 包含 i1


   assign i1_load_match_e2_i0 = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e2_i0 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                                (i1_not_alu_eff & i1_rs2_class_d.load &   i1_rs2_match_e2_i0 & ~i1_dp.store & ~i1_dp.mul);   //dec i1, match 的 e2 包含 i0

   
   assign i1_load_match_e2_i1 = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e2_i1 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                                (i1_not_alu_eff & i1_rs2_class_d.load &   i1_rs2_match_e2_i1 & ~i1_dp.store & ~i1_dp.mul);  //dec i1, match 的 e2 包含 i1


   // 这些信号 等价于  i0_load_block_d, i1_load_block_d



   logic load_match_e1_i0,  load_match_e2_i0;

   logic load_match_e1_i1,  load_match_e2_i1;

   assign load_match_e1_i0 = i0_load_match_e1_i0 | i1_load_match_e1_i0;   // dec 阶段的 i0, i1 指令 依赖于 e1 阶段的 i0 的 load 指令
   assign load_match_e2_i0 = i0_load_match_e2_i0 | i1_load_match_e2_i0;

   assign load_match_e1_i1 = i0_load_match_e1_i1 | i1_load_match_e1_i1;
   assign load_match_e2_i1 = i0_load_match_e2_i1 | i1_load_match_e2_i1;





   logic load_match_i0_e1,  load_match_i0_e2, load_match_i0_e2_final, load_match_i0_e3, load_match_i0_e4;
   logic load_match_i1_e1,  load_match_i1_e2, load_match_i1_e2_final, load_match_i1_e3, load_match_i1_e4;

   assign load_match_i0_e1 = load_match_e1_i0;
   assign load_match_i1_e1 = load_match_e1_i1;

   // load_match_i0_e2: load 在 e1 阶段时, dec 阶段的指令 依赖 load,  流到 e2 阶段
   // load_match_e2_i0: load 在 e2 阶段, dec 阶段的指令 依赖 load
   assign load_match_i0_e2_final = load_match_i0_e2 | load_match_e2_i0;
   assign load_match_i1_e2_final = load_match_i1_e2 | load_match_e2_i1;

   // 没有使用的信号
   logic [8 : 0] i0_load_match_e1_test,  i0_load_match_e2_test, i0_load_match_e2_final_test, i0_load_match_e3_test, i0_load_match_e4_test;
   logic [8 : 0] i1_load_match_e1_test,  i1_load_match_e2_test, i1_load_match_e2_final_test, i1_load_match_e3_test, i1_load_match_e4_test;


      // pipe the results down the pipe
   rvdffe #(10) i0e2matchff (.*, .en(i0_e2_data_en), .din({load_match_i0_e1, i0_load_match_e1_test}), .dout({ load_match_i0_e2, i0_load_match_e2_test} )); 

   rvdffe #(10) i1e2matchff (.*, .en(i1_e2_data_en), .din({load_match_i1_e1, i1_load_match_e1_test}), .dout({ load_match_i1_e2, i1_load_match_e2_test} ));
   

   rvdffe #(10) i0e3matchff (.*, .en(i0_e3_data_en), .din({load_match_i0_e2_final, i0_load_match_e2_test}), .dout({ load_match_i0_e3, i0_load_match_e3_test}));

   rvdffe #(10) i1e3matchff (.*, .en(i1_e3_data_en), .din({load_match_i1_e2_final, i1_load_match_e2_test}), .dout({ load_match_i1_e3, i1_load_match_e3_test}));

   rvdffe #(10) i0e4matchff (.*, .en(i0_e4_data_en), .din({load_match_i0_e3, i0_load_match_e3_test}), .dout({ load_match_i0_e4, i0_load_match_e4_test}));

   rvdffe #(10) i1e4matchff (.*, .en(i1_e4_data_en), .din({load_match_i1_e3, i1_load_match_e3_test}), .dout({ load_match_i1_e4, i1_load_match_e4_test}));

   // load_match_i0_e4, load_match_i1_e4: 依赖 load 的指令 从 e1 阶段 流水线到 e4 阶段


   // i0_load_block_d: dec 阶段的 i0 指令 依赖于 e1, e2 阶段的 load 指令 

// stores will bypass load data in the lsu pipe
   assign i0_load_block_d = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e1) |
                            (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e2 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) | // can bypass load to address of load/store
                            (i0_not_alu_eff & i0_rs2_class_d.load & i0_rs2_match_e1 & ~i0_dp.store) |
                            (i0_not_alu_eff & i0_rs2_class_d.load & i0_rs2_match_e2 & ~i0_dp.store & ~i0_dp.mul) |
                            (i0_rs3_class_d.load & i0_rs3_match_e1_e2 & i0_dp.fpu);
                            
   assign i1_load_block_d = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e1) |
                            (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e2 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                            (i1_not_alu_eff & i1_rs2_class_d.load & i1_rs2_match_e1 & ~i1_dp.store) |
                            (i1_not_alu_eff & i1_rs2_class_d.load & i1_rs2_match_e2 & ~i1_dp.store & ~i1_dp.mul);                           
                            
   assign i0_mul_block_d = (i0_not_alu_eff & i0_rs1_class_d.mul & i0_rs1_match_e1_e2) |
                           (i0_not_alu_eff & i0_rs2_class_d.mul & i0_rs2_match_e1_e2 & ~i0_dp.store);
                            
   assign i1_mul_block_d = (i1_not_alu_eff & i1_rs1_class_d.mul & i1_rs1_match_e1_e2) |
                           (i1_not_alu_eff & i1_rs2_class_d.mul & i1_rs2_match_e1_e2 & ~i1_dp.store);
                            
   assign i0_secondary_block_d = ((~i0_dp.alu & i0_rs1_class_d.sec & i0_rs1_match_e1_e3) |
                                  (~i0_dp.alu & i0_rs2_class_d.sec & i0_rs2_match_e1_e3 & ~i0_dp.store) |
                                  (i0_dp.fpu & i0_rs3_class_d.sec & i0_rs3_match_e1_e3)) & ~disable_secondary;
  
   assign i1_secondary_block_d = ((~i1_dp.alu & i1_rs1_class_d.sec & i1_rs1_match_e1_e3) |
                                  (~i1_dp.alu & i1_rs2_class_d.sec & i1_rs2_match_e1_e3 & ~i1_dp.store) & ~disable_secondary);

   
   // use this to flop the npc address of a divide for externals
   // this logic will change for delay wb of divides
   assign dec_div_decode_e4 = e4d.i0div;
   
   //--------vector & fpu------------
   assign dec_tlu_i0_valid_e4 = ((e4d.i0valid & ~e4d.i0div & ~flush_lower_wb) | exu_div_finish);
   assign dec_tlu_i1_valid_e4 = (e4d.i1valid & ~flush_lower_wb);
   
   assign dt.fetch_page_fault = dec_i0_fetch_page_fault;
   assign dt.legal     =  i0_legal_decode_d | dec_i0_fetch_page_fault;
   assign dt.icaf      =  i0_icaf_d & i0_legal_decode_d;            // dbecc is icaf exception
   assign dt.icaf_f1   =  dec_i0_icaf_f1_d & i0_legal_decode_d;     // this includes icaf and dbecc
   assign dt.perr      =   dec_i0_perr_d & i0_legal_decode_d;
   assign dt.sbecc     =   dec_i0_sbecc_d & i0_legal_decode_d;   
   assign dt.fence_i   = (i0_dp.fence_i | debug_fence_i) & i0_legal_decode_d;
   assign dt.sfence_vma= i0_dp.sfence_vma & i0_legal_decode_d;

// put pmu info into the trap packet   
   assign dt.pmu_i0_itype = i0_itype;
   assign dt.pmu_i1_itype = i1_itype;   
   assign dt.pmu_i0_br_unpred = i0_br_unpred;
   assign dt.pmu_i1_br_unpred = i1_br_unpred;   
   assign dt.pmu_divide = 1'b0;
   assign dt.pmu_lsu_misaligned = 1'b0;

   //--------------------------vector & fpu-----------------------------
   assign dt.i0trigger[3:0] = dec_i0_trigger_match_d[3:0] & {4{dec_i0_decode_d & ~i0_div_decode_d}};  
   assign dt.i1trigger[3:0] = dec_i1_trigger_match_d[3:0] & {4{dec_i1_decode_d}};

   
   
   rvdffe #( $bits(trap_pkt_t) ) trap_e1ff (.*, .en(i0_e1_ctl_en), .din( dt),  .dout(e1t));

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
   end   

   rvdffe #( $bits(trap_pkt_t) ) trap_e4ff (.*, .en(i0_e4_ctl_en), .din(e3t_in),  .dout(e4t));

   assign dec_i0_fence_i_e4_o = e4d.i0valid & e4t.legal & e4t.fence_i & ~flush_lower_wb;
   assign dec_i0_sfence_vma_e4_o = e4d.i0valid & e4t.legal & e4t.sfence_vma & ~flush_lower_wb;

   
   // assign freeze_e3 = freeze_before_dc3 & ~freeze_before;
   
   // rvdff #(1) freeze_before_ff (.*, .clk(active_clk), .din(freeze_before_dc3), .dout(freeze_before));

   // rvdff #(1) freeze_e4_ff     (.*, .clk(active_clk), .din(freeze_e3), .dout(freeze_e4));   

// these signals pipe down in the event of a freeze at dc3, needed by trap to compute triggers for a load   
   // rvdffe #(9) e4_trigger_ff   (.*, .en(freeze_e3), .din({e3d.i0load,e3t.i0trigger[3:0],e3t.i1trigger[3:0]}), .dout({e4d_i0load,e4t_i0trigger[3:0],e4t_i1trigger[3:0]}));
   
   always_comb begin

      if (exu_div_finish)    // wipe data for exu_div_finish - safer 
        dec_tlu_packet_e4 = '0;
      else
        dec_tlu_packet_e4 = e4t;

      dec_tlu_packet_e4.legal = e4t.legal | exu_div_finish;
      dec_tlu_packet_e4.i0trigger[3:0] = (exu_div_finish) ? div_trigger[3:0] : 
                                         e4t.i0trigger[3:0];

      dec_tlu_packet_e4.pmu_divide = exu_div_finish;

      // if (freeze_e4) begin  // in case of freeze, pipe down trigger information
      //    dec_tlu_packet_e4.i0trigger[3:0] = e4t_i0trigger[3:0];
      //    dec_tlu_packet_e4.i1trigger[3:0] = e4t_i1trigger[3:0];
      // end
      
   end   

   // assign dec_i0_load_e4 = e4d_i0load;
   assign dec_i0_load_e4 = 1'b0;
   
   
//   assign dec_tlu_packet_e4 = e4t;

   

// end tlu stuff

   assign i0_dc.mul   = i0_dp.mul & i0_legal_decode_d;
   assign i0_dc.load  = (i0_dp.load | i0_dp.amo) & i0_legal_decode_d;
   //----------------------- vector & fpu----------------------
   assign i0_dc.sec = ((i0_dp.alu &  i0_secondary_d) | i0_dp.vec | i0_dp.fpu) & i0_legal_decode_d; // vector and fpu just like secondary alu
   assign i0_dc.alu = i0_dp.alu & ~i0_secondary_d & i0_legal_decode_d;                                  
                                        
   assign i0_e2c_in = (freeze_before_dc1) ? '0 : i0_e1c;
   assign i0_e3c_in = (freeze_before_dc2) ? '0 : i0_e2c;
   assign i0_e4c_in = (freeze_before_dc3) ? '0 : i0_e3c;
   assign i0_wbc_in = (freeze_before_dc4) ? '0 : i0_e4c;

   rvdffs #( $bits(class_pkt_t) ) i0_e1c_ff (.*, .en(i0_e1_ctl_en), .clk(active_clk), .din(i0_dc),   .dout(i0_e1c));
   rvdffs #( $bits(class_pkt_t) ) i0_e2c_ff (.*, .en(i0_e2_ctl_en), .clk(active_clk), .din(i0_e2c_in),  .dout(i0_e2c));
   rvdffs #( $bits(class_pkt_t) ) i0_e3c_ff (.*, .en(i0_e3_ctl_en), .clk(active_clk), .din(i0_e3c_in),  .dout(i0_e3c));
   rvdffs  #( $bits(class_pkt_t) ) i0_e4c_ff (.*, .en(i0_e4_ctl_en), .clk(active_clk), .din(i0_e4c_in), .dout(i0_e4c));
   rvdffs  #( $bits(class_pkt_t) ) i0_wbc_ff (.*, .en(i0_wb_ctl_en), .clk(active_clk), .din(i0_wbc_in),    .dout(i0_wbc));                                    


   assign i1_dc.mul   = i1_dp.mul & dec_i1_decode_d;
   assign i1_dc.load  = (i1_dp.load | i1_dp.amo) & dec_i1_decode_d;   
   //----------------------- vector & fpu----------------------
   assign i1_dc.sec = ((i1_dp.alu &  i1_secondary_d) | i1_dp.fpu) & dec_i1_decode_d;
   assign i1_dc.alu = i1_dp.alu & ~i1_secondary_d & dec_i1_decode_d;                                    
                                        
   assign i1_e2c_in = (freeze_before_dc1) ? '0 : i1_e1c;
   assign i1_e3c_in = (freeze_before_dc2) ? '0 : i1_e2c;
   assign i1_e4c_in = (freeze_before_dc3) ? '0 : i1_e3c;
   assign i1_wbc_in = (freeze_before_dc4) ? '0 : i1_e4c;

   rvdffs #( $bits(class_pkt_t) ) i1_e1c_ff (.*, .en(i0_e1_ctl_en), .clk(active_clk), .din(i1_dc),   .dout(i1_e1c));
   rvdffs #( $bits(class_pkt_t) ) i1_e2c_ff (.*, .en(i0_e2_ctl_en), .clk(active_clk), .din(i1_e2c_in),  .dout(i1_e2c));
   rvdffs #( $bits(class_pkt_t) ) i1_e3c_ff (.*, .en(i0_e3_ctl_en), .clk(active_clk), .din(i1_e3c_in),  .dout(i1_e3c));
   rvdffs #( $bits(class_pkt_t) ) i1_e4c_ff (.*, .en(i0_e4_ctl_en), .clk(active_clk), .din(i1_e4c_in), .dout(i1_e4c));
   rvdffs #( $bits(class_pkt_t) ) i1_wbc_ff (.*, .en(i0_wb_ctl_en), .clk(active_clk), .din(i1_wbc_in),    .dout(i1_wbc)); 
       

   assign dd.i0rd[4:0] = i0r.rd[4:0];
   
   assign dd.i0v = i0_rd_en_d & i0_legal_decode_d;
   assign dd.i0valid =              dec_i0_decode_d;  // has flush_final_e3

   assign dd.i0mul = i0_dp.mul & i0_legal_decode_d;

   assign dd.i0load = (i0_dp.load /*| i0_dp.amo*/) & i0_legal_decode_d;  

   assign dd.i0secload = (i0_dp.load /*| i0_dp.amo*/) & i0_legal_decode_d;  


   assign dd.i0store = i0_dp.store & i0_legal_decode_d;
   assign dd.i0div = i0_dp.div & i0_legal_decode_d;
   //----------------vector & fpu--------------
   assign dd.i0vec = i0_dp.vec & i0_legal_decode_d;
   assign dd.i0fpu = i0_dp.fpu & i0_legal_decode_d;
   assign dd.i0outine4 = (i0_secondary_d | i0_dp.vec | i0_dp.fpu) & i0_legal_decode_d; // secondary alu and vector instruction is execution in e4

   //正常流水线的目标操作数寄存器文件
   // assign dd.i0_rd_arf = (i0_dp.load & i0_dp.ls_fp) | i0_dp.fpu_wb_f_arf | (i0_dp.vec & i0_dp.vec_fd);
   assign dd.i0_rd_arf = (i0_dp.load & i0_dp.ls_fp) | (i0_dp.vec & i0_dp.vec_fd) | (i0_dp.fpu & i0_dp.fpu_wb_f_arf);
   assign dd.i1_rd_arf = (i1_dp.load & i1_dp.ls_fp) | (i1_dp.fpu & i1_dp.fpu_wb_f_arf)/* | (i1_dp.vec & i1_dp.vec_fd)*/;

   assign dd.i0amo = i0_dp.amo & i0_legal_decode_d;
   assign dd.i1amo = i1_dp.amo;
   

   assign dd.i1rd[4:0] = i1r.rd[4:0];
   assign dd.i1v = i1_rd_en_d & dec_i1_decode_d;
   assign dd.i1valid =              dec_i1_decode_d;   
   assign dd.i1mul = i1_dp.mul;

   assign dd.i1load = i1_dp.load /*| i1_dp.amo*/;

   assign dd.i1secload = i1_dp.load /*| i1_dp.amo*/;


   assign dd.i1store = i1_dp.store;
   //----------------- vector & fpu-----------------
   assign dd.i1outine4 = (i1_secondary_d | i1_dp.fpu) & dec_i1_decode_d;   
   assign dd.i1fpu = i1_dp.fpu & dec_i1_decode_d;

   assign dd.csrwen = dec_csr_wen_unq_d & i0_legal_decode_d;        
   assign dd.csrwonly = i0_csr_write_only_d & dec_i0_decode_d;
   assign dd.csrwaddr[11:0] = i0[31:20];    // csr write address for rd==0 case
   
   
   assign i0_pipe_en[5] = dec_i0_decode_d;
   
   rvdffs #(1) i0cg0ff (.*, .clk(active_clk), .en(~freeze_before_dc1), .din(i0_pipe_en[5]), .dout(i0_pipe_en[4]));
   rvdffs #(1) i0cg1ff (.*, .clk(active_clk), .en(~freeze_before_dc2), .din(i0_pipe_en[4]), .dout(i0_pipe_en[3]));
   rvdffs #(1) i0cg2ff (.*, .clk(active_clk), .en(~freeze_before_dc3), .din(i0_pipe_en[3]), .dout(i0_pipe_en[2]));
   rvdffs #(1) i0cg3ff (.*, .clk(active_clk), .en(~freeze_before_dc4), .din(i0_pipe_en[2]), .dout(i0_pipe_en[1]));
   rvdff  #(1) i0cg4ff (.*, .clk(active_clk),               .din(i0_pipe_en[1]), .dout(i0_pipe_en[0]));   

   
   assign i0_e1_ctl_en = (|i0_pipe_en[5:4] | clk_override) & ~freeze_before_dc1;
   assign i0_e2_ctl_en = (|i0_pipe_en[4:3] | clk_override) & ~freeze_before_dc2;
   assign i0_e3_ctl_en = (|i0_pipe_en[3:2] | clk_override) & ~freeze_before_dc3;
   assign i0_e4_ctl_en = (|i0_pipe_en[2:1] | clk_override) & ~freeze_before_dc4;
   assign i0_wb_ctl_en = (|i0_pipe_en[1:0] | clk_override);   
   
   assign i0_e1_data_en = (i0_pipe_en[5] | clk_override) & ~freeze_before_dc1;
   assign i0_e2_data_en = (i0_pipe_en[4] | clk_override) & ~freeze_before_dc2;
   assign i0_e3_data_en = (i0_pipe_en[3] | clk_override) & ~freeze_before_dc3;
   assign i0_e4_data_en = (i0_pipe_en[2] | clk_override) & ~freeze_before_dc4;
   assign i0_wb_data_en = (i0_pipe_en[1] | clk_override);
   assign i0_wb1_data_en = (i0_pipe_en[0] | clk_override);      

   assign dec_i0_data_en[4:2] = {i0_e1_data_en, i0_e2_data_en, i0_e3_data_en};
   assign dec_i0_ctl_en[4:1]  = {i0_e1_ctl_en, i0_e2_ctl_en, i0_e3_ctl_en, i0_e4_ctl_en};   

   
   assign i1_pipe_en[5] = dec_i1_decode_d;
   
   rvdffs #(1) i1cg0ff (.*, .clk(active_clk), .en(~freeze_before_dc1), .din(i1_pipe_en[5]), .dout(i1_pipe_en[4]));
   rvdffs #(1) i1cg1ff (.*, .clk(active_clk), .en(~freeze_before_dc2), .din(i1_pipe_en[4]), .dout(i1_pipe_en[3]));
   rvdffs #(1) i1cg2ff (.*, .clk(active_clk), .en(~freeze_before_dc3), .din(i1_pipe_en[3]), .dout(i1_pipe_en[2]));
   rvdffs #(1) i1cg3ff (.*, .clk(active_clk), .en(~freeze_before_dc4), .din(i1_pipe_en[2]), .dout(i1_pipe_en[1]));
   rvdff  #(1) i1cg4ff (.*, .clk(free_clk),               .din(i1_pipe_en[1]), .dout(i1_pipe_en[0]));   

   
   assign i1_e1_ctl_en = (|i1_pipe_en[5:4] | clk_override) & ~freeze_before_dc1;
   assign i1_e2_ctl_en = (|i1_pipe_en[4:3] | clk_override) & ~freeze_before_dc2;
   assign i1_e3_ctl_en = (|i1_pipe_en[3:2] | clk_override) & ~freeze_before_dc3;
   assign i1_e4_ctl_en = (|i1_pipe_en[2:1] | clk_override) & ~freeze_before_dc4;
   assign i1_wb_ctl_en = (|i1_pipe_en[1:0] | clk_override);   
   
   assign i1_e1_data_en = (i1_pipe_en[5] | clk_override) & ~freeze_before_dc1;
   assign i1_e2_data_en = (i1_pipe_en[4] | clk_override) & ~freeze_before_dc2;
   assign i1_e3_data_en = (i1_pipe_en[3] | clk_override) & ~freeze_before_dc3;
   assign i1_e4_data_en = (i1_pipe_en[2] | clk_override) & ~freeze_before_dc4;
   assign i1_wb_data_en = (i1_pipe_en[1] | clk_override);
   assign i1_wb1_data_en = (i1_pipe_en[0] | clk_override);      

   assign dec_i1_data_en[4:2] = {i1_e1_data_en, i1_e2_data_en, i1_e3_data_en};
   assign dec_i1_ctl_en[4:1]  = {i1_e1_ctl_en, i1_e2_ctl_en, i1_e3_ctl_en, i1_e4_ctl_en};   
   
   rvdffe #( $bits(dest_pkt_t) ) e1ff (.*, .en(i0_e1_ctl_en), .din(dd),  .dout(e1d));

   always_comb begin
      e1d_in = e1d;
      
      e1d_in.i0v = e1d.i0v & ~flush_final_e3 & ~flush_lower_wb;
      e1d_in.i1v = e1d.i1v & ~flush_final_e3 & ~flush_lower_wb;
      e1d_in.i0valid = e1d.i0valid & ~flush_final_e3 & ~flush_lower_wb;
      e1d_in.i1valid = e1d.i1valid & ~flush_final_e3 & ~flush_lower_wb;
      e1d_in.i0outine4 = e1d.i0outine4 & ~flush_final_e3 & ~flush_lower_wb;
      e1d_in.i1outine4 = e1d.i1outine4 & ~flush_final_e3 & ~flush_lower_wb;

      // e1d_in.i0load = e1d.i0load & ~flush_final_e3 & (i0_load_match_e1_i0 | i1_load_match_e1_i0);
      // e1d_in.i1load = e1d.i1load & ~flush_final_e3 & (i0_load_match_e1_i1 | i1_load_match_e1_i1);

      e1d_in.i0secload =  (i0_load_match_e1_i0 | i1_load_match_e1_i0);
      e1d_in.i1secload =  (i0_load_match_e1_i1 | i1_load_match_e1_i1);

      if (freeze_before_dc1) e1d_in = '0;
   end   
   
   assign dec_i1_valid_e1 = e1d.i1valid;   
   
   
   rvdffe #( $bits(dest_pkt_t) ) e2ff (.*, .en(i0_e2_ctl_en), .din(e1d_in), .dout(e2d));

   always_comb begin
      e2d_in = e2d;
      
      e2d_in.i0v = e2d.i0v &         ~flush_final_e3 & ~flush_lower_wb;
      e2d_in.i1v = e2d.i1v &         ~flush_final_e3 & ~flush_lower_wb;   
      e2d_in.i0valid = e2d.i0valid & ~flush_final_e3 & ~flush_lower_wb;
      e2d_in.i1valid = e2d.i1valid & ~flush_final_e3 & ~flush_lower_wb;   
      e2d_in.i0outine4 = e2d.i0outine4 & ~flush_final_e3 & ~flush_lower_wb;   
      e2d_in.i1outine4 = e2d.i1outine4 & ~flush_final_e3 & ~flush_lower_wb;   

      // e2d_in.i0load = e2d.i0load & ~flush_final_e3 & ~flush_lower_wb;
      // e2d_in.i1load = e2d.i1load & ~flush_final_e3 & ~flush_lower_wb;

      // e2d_in.i0load = e2d.seci0load ;
      // e2d_in.i1load = e2d.seci1load ;

      if (freeze_before_dc2) e2d_in = '0;
   end   
   
   rvdffe #( $bits(dest_pkt_t) ) e3ff (.*, .en(i0_e3_ctl_en), .din(e2d_in), .dout(e3d));

   always_comb begin
      e3d_in = e3d;
      
      e3d_in.i0v = e3d.i0v                              & ~flush_lower_wb;   
      e3d_in.i0valid = e3d.i0valid                      & ~flush_lower_wb;

      e3d_in.i0outine4 = e3d.i0outine4 & ~flush_lower_wb;   
      
      e3d_in.i1v = e3d.i1v         & ~i0_flush_final_e3 & ~flush_lower_wb;   
      e3d_in.i1valid = e3d.i1valid & ~i0_flush_final_e3 & ~flush_lower_wb;   

      e3d_in.i1outine4 = e3d.i1outine4 & ~i0_flush_final_e3 & ~flush_lower_wb;

      // e3d_in.i0load = e3d.i0load & ~flush_lower_wb;  // 
      // e3d_in.i1load = e3d.i1load & ~i0_flush_final_e3 & ~flush_lower_wb;

      if (freeze_before_dc3) e3d_in = '0;
      
   end 


   assign dec_i0_sec_decode_e3 = e3d.i0outine4 & ~e3d.i0vec & ~e3d.i0fpu & ~flush_lower_wb & ~freeze_before_dc3;
   assign dec_i1_sec_decode_e3 = e3d.i1outine4 & ~e3d.i1fpu & ~i0_flush_final_e3 & ~flush_lower_wb & ~freeze_before_dc3; 

   // 检测 是否依赖 流水线中的 load 指令信号
   assign dec_i0_load_decode_e3 = e3d.i0secload & ~e3d.i0vec & ~e3d.i0fpu & ~flush_lower_wb & ~freeze_before_dc3;
   assign dec_i1_load_decode_e3 = e3d.i1secload & ~e3d.i1fpu & ~i0_flush_final_e3 & ~flush_lower_wb & ~freeze_before_dc3; 


   
   assign dec_i0_vec_decode_e4 = e4d.i0outine4 & e4d.i0vec & ~flush_lower_wb;

   assign dec_i0_fpu_decode_e4 = e4d.i0outine4 & e4d.i0fpu & ~flush_lower_wb;
   assign dec_i1_fpu_decode_e4 = e4d.i1outine4 & e4d.i1fpu & ~flush_lower_wb;

   rvdffe #( $bits(dest_pkt_t) ) e4ff (.*, .en(i0_e4_ctl_en), .din(e3d_in), .dout(e4d));

   always_comb begin
      if (exu_div_finish)    // wipe data for exu_div_finish - bug where csr_wen was set for fast divide
        e4d_in = '0;
      else
        e4d_in = e4d;

      e4d_in.i0rd[4:0] = (exu_div_finish) ? div_waddr_wb[4:0] : 
                         e4d.i0rd[4:0];

      e4d_in.i0v = ((e4d.i0v  & ~e4d.i0div) | (exu_div_finish & div_waddr_wb[4:0]!=5'b0)) & ~flush_lower_wb;

      e4d_in.i0valid = (e4d.i0valid | exu_div_finish) & ~flush_lower_wb;
      // qual the following with div finish; necessary for divides with early exit
      e4d_in.i0outine4 = e4d.i0outine4 & ~flush_lower_wb & ~exu_div_finish;
      e4d_in.i0load = e4d.i0load & ~flush_lower_wb & ~exu_div_finish;
      e4d_in.i0store = e4d.i0store & ~flush_lower_wb & ~exu_div_finish;
      e4d_in.i0_rd_arf = e4d.i0_rd_arf & ~exu_div_finish;

      e4d_in.i1v = e4d.i1v         & ~flush_lower_wb;   
      e4d_in.i1valid = e4d.i1valid & ~flush_lower_wb;
      e4d_in.i1outine4 = e3d.i1outine4 & ~flush_lower_wb;

      if(freeze_before_dc4) begin
         e4d_in = '0;
      end

   end

   //------------vector & fpu--------------
   rvdffe #( $bits(dest_pkt_t) ) wbff (.*, .en(i0_wb_ctl_en | exu_div_finish | div_wen_wb), .din(e4d_in), .dout(wbd));
   
   assign dec_i0_waddr_wb[4:0] =  wbd.i0rd[4:0];

   // squash same write, take last write assuming we don't kill the I1 write for some reason.
   // 如果I0和I1有相同的目标寄存器，那么就只写I1
   assign i0_wen_wb = wbd.i0v & ~(~dec_tlu_i1_kill_writeb_wb & ~i1_load_kill_wen & wbd.i0v & wbd.i1v & (wbd.i0_rd_arf == wbd.i1_rd_arf) & (wbd.i0rd[4:0] == wbd.i1rd[4:0])) & ~dec_tlu_i0_kill_writeb_wb & ~i0_load_kill_wen;
   assign dec_i0_wen_wb = i0_wen_wb & ~wbd.i0_rd_arf; // write integer gpr
   assign i0_f_arf_wen_wb = i0_wen_wb & wbd.i0_rd_arf; //write float gpr
   assign dec_i0_wdata_wb[63:0] = i0_result_wb[63:0];

   assign dec_i1_waddr_wb[4:0] = wbd.i1rd[4:0];
   assign i1_wen_wb = wbd.i1v & ~dec_tlu_i1_kill_writeb_wb & ~i1_load_kill_wen;
   assign i1_f_arf_wen_wb = i1_wen_wb & wbd.i1_rd_arf; // write float gpr
   assign dec_i1_wen_wb = i1_wen_wb & ~wbd.i1_rd_arf;
   assign dec_i1_wdata_wb[63:0] = i1_result_wb[63:0];

// divide stuff

   
   assign div_stall = exu_div_stall | div_stall_ff;   // hold for 1 extra cycle so wb can happen before next inst
   rvdff  #(1) divstallff (.*, .clk(active_clk), .din(exu_div_stall), .dout(div_stall_ff));

   assign i0_div_decode_d = i0_legal_decode_d & i0_dp.div;
   
   rvdffe #(63) divpcff (.*, .en(i0_div_decode_d), .din(dec_i0_pc_d[63:1]), .dout(div_pc[63:1]));

   rvdffs #(4) divtriggerff (.*, .en(i0_div_decode_d), .clk(active_clk), .din(dec_i0_trigger_match_d[3:0]), .dout(div_trigger[3:0]));

   rvdffs #(5) divwbaddrff (.*, .en(i0_div_decode_d), .clk(active_clk), .din(i0r.rd[4:0]), .dout(div_waddr_wb[4:0]));

   // active_clk -> used for clockgating for wb stage ctl logic
   rvdff  #(1) divwbff (.*, .clk(active_clk), .din(exu_div_finish), .dout(div_wen_wb));

   //----------------vector & fpu---------------------
   //vector and fpu module output -> wb
   rvdffe  #(5) fpuflagsff1 (.*, .en(i0_wb_ctl_en), .din(fpu_flags_e4_i), .dout(fpu_flags_wb_o));
   assign fpu_flags_wen_wb_o = (wbd.i0valid & wbd.i0fpu & ~dec_tlu_i0_kill_writeb_wb) | (wbd.i1valid & wbd.i1fpu & ~dec_tlu_i0_kill_writeb_wb & ~dec_tlu_i1_kill_writeb_wb);
   //----------------vector & fpu---------------------
   





   // amo 指令默认正确
   assign i0_result_e1[63:0] = exu_i0_result_e1[63:0];
   assign i1_result_e1[63:0] = exu_i1_result_e1[63:0];   

   // pipe the results down the pipe
   rvdffe #(64) i0e2resultff (.*, .en(i0_e2_data_en), .din(i0_result_e1[63:0]), .dout(i0_result_e2[63:0]));
   rvdffe #(64) i1e2resultff (.*, .en(i1_e2_data_en), .din(i1_result_e1[63:0]), .dout(i1_result_e2[63:0]));   

   rvdffe #(64) i0e3resultff (.*, .en(i0_e3_data_en), .din(i0_result_e2[63:0]), .dout(i0_result_e3[63:0]));
   rvdffe #(64) i1e3resultff (.*, .en(i1_e3_data_en), .din(i1_result_e2[63:0]), .dout(i1_result_e3[63:0]));   



   assign i0_result_e3_final[63:0] = (e3d.i0v & e3d.i0load) ? lsu_result_dc3[63:0] : (e3d.i0v & e3d.i0mul) ? exu_mul_result_e3[63:0] : i0_result_e3[63:0];

   assign i1_result_e3_final[63:0] = (e3d.i1v & e3d.i1load) ? lsu_result_dc3[63:0] : (e3d.i1v & e3d.i1mul) ? exu_mul_result_e3[63:0] : i1_result_e3[63:0];   



   rvdffe #(64) i0e4resultff (.*, .en(i0_e4_data_en), .din(i0_result_e3_final[63:0]), .dout(i0_result_e4[63:0]));
   rvdffe #(64) i1e4resultff (.*, .en(i1_e4_data_en), .din(i1_result_e3_final[63:0]), .dout(i1_result_e4[63:0]));   
   
   assign i0_result_e4_final[63:0] =
                                     (          e4d.i0outine4 ) ? exu_i0_result_e4[63:0] : 
                                     (e4d.i0v & e4d.i0amo)? lsu_amo_result_e4[63:0] :
                                    //  (e4d.i0v & e4d.i0load) ? lsu_result_corr_dc4[63:0] : 
                                     i0_result_e4[63:0];                                
   
   assign i1_result_e4_final[63:0] = 
                                     (e4d.i1v & e4d.i1outine4 ) ? exu_i1_result_e4[63:0] : 
                                     (e4d.i1v & e4d.i1amo)? lsu_amo_result_e4 :
                                    //  (e4d.i1v & e4d.i1load) ? lsu_result_corr_dc4[63:0] : 
                                     i1_result_e4[63:0];

   rvdffe #(64) i0wbresultff (.*, .en(i0_wb_data_en), .din(i0_result_e4_final[63:0]), .dout(i0_result_wb_raw[63:0]));
   rvdffe #(64) i1wbresultff (.*, .en(i1_wb_data_en), .din(i1_result_e4_final[63:0]), .dout(i1_result_wb_raw[63:0]));   
   //--------------------vector & fpu---------------
   assign i0_result_wb[63:0] =   (div_wen_wb) ? exu_div_result[63:0] :           //除法单元结果
                                 i0_result_wb_raw[63:0];                         //流水线结果

   assign i1_result_wb[63:0] = i1_result_wb_raw[63:0];


   // Added

   assign i0_result_vp_e1[63:0] = exu_i0_result_e1[63:0];
   assign i1_result_vp_e1[63:0] = exu_i1_result_e1[63:0];   

   // pipe the results down the pipe
   rvdffe #(64) i0e2resultvpff (.*, .en(i0_e2_data_en), .din(i0_result_vp_e1[63:0]), .dout(i0_result_vp_e2[63:0]));
   rvdffe #(64) i1e2resultvpff (.*, .en(i1_e2_data_en), .din(i1_result_vp_e1[63:0]), .dout(i1_result_vp_e2[63:0]));   

   rvdffe #(64) i0e3resultvpff (.*, .en(i0_e3_data_en), .din(i0_result_vp_e2[63:0]), .dout(i0_result_vp_e3[63:0]));
   rvdffe #(64) i1e3resultvpff (.*, .en(i1_e3_data_en), .din(i1_result_vp_e2[63:0]), .dout(i1_result_vp_e3[63:0]));   


   rvdffe #(64) i0e4resultvpff (.*, .en(i0_e4_data_en), .din(i0_result_vp_e3[63:0]), .dout(i0_result_vp_e4_raw[63:0]));
   rvdffe #(64) i1e4resultvpff (.*, .en(i1_e4_data_en), .din(i1_result_vp_e3[63:0]), .dout(i1_result_vp_e4_raw[63:0]));   
   
   

   rvdffe #(64) i0wbresultvpff (.*, .en(i0_wb_data_en), .din(i0_result_vp_e4[63:0]), .dout(i0_result_vp_wb[63:0]));
   rvdffe #(64) i1wbresultvpff (.*, .en(i1_wb_data_en), .din(i1_result_vp_e4[63:0]), .dout(i1_result_vp_wb[63:0]));   



   logic [63 : 1] exu_i0_flush_path_vp_e4, exu_i1_flush_path_vp_e4;
   /*
   // logic exu_i0_flush_vp_e4, exu_i1_flush_vp_e4;

   dec_vp_flush_ctl i0_dec_vp_e4 (.*,
                          .freeze                ( 1'b0                        ),   // I   
                          .enable                ( i0_e4_ctl_en                ),   // I  
          
                          .valid                 ( dec_i0_load_decode_e3             ), 

                          .flush                 ( dec_tlu_flush_lower_wb      ),   // I   

                          .predicted_result      ( i0_result_vp_e4_raw[63:0]     ),   // I

                          // .predicted_result      ( lsu_result_dc3[63:0]     ),   // I 

                          .actual_result         ( i0_result_e4_final[63:0]             ),   // I

                          .pred_path           ( npc_e4[63:1]                ),   // I  // 分支预测的地址, 无论正确与否

                          // .any_jal_flush_path    ( i0_value_flush_e4[63:1]     ),   // I   any_jal
                          // .any_jal               ( i0_is_jal_e4                ),   // I                                 
                          // .inst_type             ( 1'b1                        ),   // I     
                                                                                                  
                          .flush_upper_vp        ( exu_i0_flush_vp_e4          ),   // O 
						                   
                          .flush_path_vp         ( exu_i0_flush_path_vp_e4[63:1]         ),   // O

                          .predicted_result_new  ( i0_result_vp_e4[63:0]          )   // O    

                          );

   dec_vp_flush_ctl i1_dec_vp_e4 (.*,
                          .freeze                ( 1'b0                        ),   // I   
                         //  .freeze                ( freeze                        ),   // I   
                          .enable                ( i1_e4_ctl_en                ),   // I  
               
                          .valid                 ( dec_i1_load_decode_e3            ),  // dec_i1_sec_decode_e3
                   
                         .flush                 ( dec_tlu_flush_lower_wb      ),   // I 

                          .predicted_result      ( i1_result_vp_e4_raw[63:0]     ),   // I   

                          // .predicted_result      ( lsu_result_dc3[63:0]     ),   // I   
                          
                          .actual_result         ( i1_result_e4_final[63:0]             ),   // I

                          .pred_path             ( npc_e4[63:1]                ),   // I  // 分支预测的地址, 无论正确与否

                          // .any_jal_flush_path    ( i1_value_flush_e4[31:1]     ),   // I   any_jal
                          // .any_jal               ( i1_is_jal_e4                ),   // I                              
                          // .inst_type             ( 1'b1                        ),   // I     
                                                                                                                             
                          .flush_upper_vp        ( exu_i1_flush_vp_e4          ),   // O 
						                   
                          .flush_path_vp         ( exu_i1_flush_path_vp_e4[63:1]         ),   // O

                          .predicted_result_new  ( i1_result_vp_e4[63:0]          )   // O    

                          );   





      */



   rvdffe #(12) e1brpcff (.*, .en(i0_e1_data_en), .din(last_br_immed_d[12:1] ), .dout(last_br_immed_e1[12:1]));
   rvdffe #(12) e2brpcff (.*, .en(i0_e2_data_en), .din(last_br_immed_e1[12:1]), .dout(last_br_immed_e2[12:1]));


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

   assign dec_i0_inst_wb1[31:0] = i0_inst_wb1[31:0];
   assign dec_i1_inst_wb1[31:0] = i1_inst_wb1[31:0];   

   rvdffe #(63) i0wbpcff  (.*, .en(i0_wb_data_en | exu_div_finish), .din(dec_tlu_i0_pc_e4[63:1]), .dout(i0_pc_wb[63:1]));
   rvdffe #(63) i0wb1pcff (.*, .en(i0_wb1_data_en | div_wen_wb),    .din(i0_pc_wb[63:1]),         .dout(i0_pc_wb1[63:1]));

   rvdffe #(63) i1wb1pcff (.*, .en(i1_wb1_data_en),.din(i1_pc_wb[63:1]),         .dout(i1_pc_wb1[63:1]));      

   assign dec_i0_pc_wb1[63:1] = i0_pc_wb1[63:1];
   assign dec_i1_pc_wb1[63:1] = i1_pc_wb1[63:1];   
   
   
   // pipe the pc's down the pipe
   assign i0_pc_e1[63:1] = exu_i0_pc_e1[63:1];
   assign i1_pc_e1[63:1] = exu_i1_pc_e1[63:1];   
   
   rvdffe #(63) i0e2pcff (.*, .en(i0_e2_data_en), .din(i0_pc_e1[63:1]), .dout(i0_pc_e2[63:1]));
   rvdffe #(63) i0e3pcff (.*, .en(i0_e3_data_en), .din(i0_pc_e2[63:1]), .dout(i0_pc_e3[63:1]));
   rvdffe #(63) i0e4pcff (.*, .en(i0_e4_data_en), .din(i0_pc_e3[63:1]), .dout(i0_pc_e4[63:1]));
   rvdffe #(63) i1e2pcff (.*, .en(i1_e2_data_en), .din(i1_pc_e1[63:1]), .dout(i1_pc_e2[63:1]));
   rvdffe #(63) i1e3pcff (.*, .en(i1_e3_data_en), .din(i1_pc_e2[63:1]), .dout(i1_pc_e3[63:1]));
   rvdffe #(63) i1e4pcff (.*, .en(i1_e4_data_en), .din(i1_pc_e3[63:1]), .dout(i1_pc_e4[63:1]));

   assign dec_i0_pc_e3[63:1] = i0_pc_e3[63:1];
   assign dec_i1_pc_e3[63:1] = i1_pc_e3[63:1];   
   
   assign dec_tlu_i0_pc_e4[63:1] = (exu_div_finish) ? div_pc[63:1]:
                                   i0_pc_e4[63:1];
   assign dec_tlu_i1_pc_e4[63:1] = i1_pc_e4[63:1];

   // generate the correct npc for correct br predictions
   assign last_pc_e2[63:1] = (e2d.i1valid) ? i1_pc_e2[63:1] : i0_pc_e2[63:1];
   
   rvbradder ibradder_correct (
                     .pc(last_pc_e2[63:1]),
                     .offset(last_br_immed_e2[12:1]),
                     .dout(pred_correct_npc_e2[63:1])
                     );



   // needed for debug triggers 
   rvdffe #(63) i1wbpcff (.*, .en(i1_wb_data_en), .din(dec_tlu_i1_pc_e4[63:1]), .dout(i1_pc_wb[63:1]));   



  
   
   
   // bit 9 is priority match, bit 0 lowest priority, i1_e1, i0_e1, i1_e2, ... i1_wb, i0_wb


   
   assign i0_rs1bypass[9:0] = {   i0_rs1_depth_d[3:0] == 4'd1  &  i0_rs1_class_d.alu,
                                  i0_rs1_depth_d[3:0] == 4'd2  &  i0_rs1_class_d.alu,
                                  i0_rs1_depth_d[3:0] == 4'd3  &  i0_rs1_class_d.alu,
                                  i0_rs1_depth_d[3:0] == 4'd4  &  i0_rs1_class_d.alu,
                                  i0_rs1_depth_d[3:0] == 4'd5  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                  i0_rs1_depth_d[3:0] == 4'd6  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                  i0_rs1_depth_d[3:0] == 4'd7  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec),
                                  i0_rs1_depth_d[3:0] == 4'd8  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec),
                                  i0_rs1_depth_d[3:0] == 4'd9  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec),
                                  i0_rs1_depth_d[3:0] == 4'd10 & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec) };
   

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

   assign i0_rs3bypass[9:0] = {   i0_rs3_depth_d[3:0] == 4'd1  &  i0_rs3_class_d.alu,
                                  i0_rs3_depth_d[3:0] == 4'd2  &  i0_rs3_class_d.alu,
                                  i0_rs3_depth_d[3:0] == 4'd3  &  i0_rs3_class_d.alu,
                                  i0_rs3_depth_d[3:0] == 4'd4  &  i0_rs3_class_d.alu,
                                  i0_rs3_depth_d[3:0] == 4'd5  & (i0_rs3_class_d.alu | i0_rs3_class_d.load | i0_rs3_class_d.mul),
                                  i0_rs3_depth_d[3:0] == 4'd6  & (i0_rs3_class_d.alu | i0_rs3_class_d.load | i0_rs3_class_d.mul),
                                  i0_rs3_depth_d[3:0] == 4'd7  & (i0_rs3_class_d.alu | i0_rs3_class_d.load | i0_rs3_class_d.mul | i0_rs3_class_d.sec),
                                  i0_rs3_depth_d[3:0] == 4'd8  & (i0_rs3_class_d.alu | i0_rs3_class_d.load | i0_rs3_class_d.mul | i0_rs3_class_d.sec),
                                  i0_rs3_depth_d[3:0] == 4'd9  & (i0_rs3_class_d.alu | i0_rs3_class_d.load | i0_rs3_class_d.mul | i0_rs3_class_d.sec),
                                  i0_rs3_depth_d[3:0] == 4'd10 & (i0_rs3_class_d.alu | i0_rs3_class_d.load | i0_rs3_class_d.mul | i0_rs3_class_d.sec) };
   

   assign i1_rs1bypass[9:0] = {   i1_rs1_depth_d[3:0] == 4'd1  &  i1_rs1_class_d.alu,
                                  i1_rs1_depth_d[3:0] == 4'd2  &  i1_rs1_class_d.alu,
                                  i1_rs1_depth_d[3:0] == 4'd3  &  i1_rs1_class_d.alu,
                                  i1_rs1_depth_d[3:0] == 4'd4  &  i1_rs1_class_d.alu,
                                  i1_rs1_depth_d[3:0] == 4'd5  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul),
                                  i1_rs1_depth_d[3:0] == 4'd6  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul),
                                  i1_rs1_depth_d[3:0] == 4'd7  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec),
                                  i1_rs1_depth_d[3:0] == 4'd8  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec),
                                  i1_rs1_depth_d[3:0] == 4'd9  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec),
                                  i1_rs1_depth_d[3:0] == 4'd10 & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec) };
   

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
   



   assign dec_i0_rs1_bypass_en_d = |i0_rs1bypass[9:0];
   assign dec_i0_rs2_bypass_en_d = |i0_rs2bypass[9:0];
   assign dec_i0_rs3_bypass_en_d = |i0_rs3bypass[9:0];
   assign dec_i1_rs1_bypass_en_d = |i1_rs1bypass[9:0];
   assign dec_i1_rs2_bypass_en_d = |i1_rs2bypass[9:0];   

   

   assign i0_rs1_bypass_data_d[63:0] = ({64{i0_rs1bypass[9]}} & i1_result_e1[63:0]) |
                                       ({64{i0_rs1bypass[8]}} & i0_result_e1[63:0]) |
                                       ({64{i0_rs1bypass[7]}} & i1_result_e2[63:0]) |
                                       ({64{i0_rs1bypass[6]}} & i0_result_e2[63:0]) |
                                       ({64{i0_rs1bypass[5]}} & i1_result_e3_final[63:0]) |
                                       ({64{i0_rs1bypass[4]}} & i0_result_e3_final[63:0]) |
                                       ({64{i0_rs1bypass[3]}} & i1_result_e4_final[63:0]) |
                                       ({64{i0_rs1bypass[2]}} & i0_result_e4_final[63:0]) |
                                       ({64{i0_rs1bypass[1]}} & i1_result_wb[63:0]) |
                                       ({64{i0_rs1bypass[0]}} & i0_result_wb[63:0]);
   

   assign i0_rs2_bypass_data_d[63:0] = ({64{i0_rs2bypass[9]}} & i1_result_e1[63:0]) |
                                       ({64{i0_rs2bypass[8]}} & i0_result_e1[63:0]) |
                                       ({64{i0_rs2bypass[7]}} & i1_result_e2[63:0]) |
                                       ({64{i0_rs2bypass[6]}} & i0_result_e2[63:0]) |
                                       ({64{i0_rs2bypass[5]}} & i1_result_e3_final[63:0]) |
                                       ({64{i0_rs2bypass[4]}} & i0_result_e3_final[63:0]) |
                                       ({64{i0_rs2bypass[3]}} & i1_result_e4_final[63:0]) |
                                       ({64{i0_rs2bypass[2]}} & i0_result_e4_final[63:0]) |
                                       ({64{i0_rs2bypass[1]}} & i1_result_wb[63:0]) |
                                       ({64{i0_rs2bypass[0]}} & i0_result_wb[63:0]);

   assign i0_rs3_bypass_data_d[63:0] = ({64{i0_rs3bypass[9]}} & i1_result_e1[63:0]) |
                                       ({64{i0_rs3bypass[8]}} & i0_result_e1[63:0]) |
                                       ({64{i0_rs3bypass[7]}} & i1_result_e2[63:0]) |
                                       ({64{i0_rs3bypass[6]}} & i0_result_e2[63:0]) |
                                       ({64{i0_rs3bypass[5]}} & i1_result_e3_final[63:0]) |
                                       ({64{i0_rs3bypass[4]}} & i0_result_e3_final[63:0]) |
                                       ({64{i0_rs3bypass[3]}} & i1_result_e4_final[63:0]) |
                                       ({64{i0_rs3bypass[2]}} & i0_result_e4_final[63:0]) |
                                       ({64{i0_rs3bypass[1]}} & i1_result_wb[63:0]) |
                                       ({64{i0_rs3bypass[0]}} & i0_result_wb[63:0]);
   
   assign i1_rs1_bypass_data_d[63:0] = ({64{i1_rs1bypass[9]}} & i1_result_e1[63:0]) |
                                       ({64{i1_rs1bypass[8]}} & i0_result_e1[63:0]) |
                                       ({64{i1_rs1bypass[7]}} & i1_result_e2[63:0]) |
                                       ({64{i1_rs1bypass[6]}} & i0_result_e2[63:0]) |
                                       ({64{i1_rs1bypass[5]}} & i1_result_e3_final[63:0]) |
                                       ({64{i1_rs1bypass[4]}} & i0_result_e3_final[63:0]) |
                                       ({64{i1_rs1bypass[3]}} & i1_result_e4_final[63:0]) |
                                       ({64{i1_rs1bypass[2]}} & i0_result_e4_final[63:0]) |
                                       ({64{i1_rs1bypass[1]}} & i1_result_wb[63:0]) |
                                       ({64{i1_rs1bypass[0]}} & i0_result_wb[63:0]);
   

   assign i1_rs2_bypass_data_d[63:0] = ({64{i1_rs2bypass[9]}} & i1_result_e1[63:0]) |
                                       ({64{i1_rs2bypass[8]}} & i0_result_e1[63:0]) |
                                       ({64{i1_rs2bypass[7]}} & i1_result_e2[63:0]) |
                                       ({64{i1_rs2bypass[6]}} & i0_result_e2[63:0]) |
                                       ({64{i1_rs2bypass[5]}} & i1_result_e3_final[63:0]) |
                                       ({64{i1_rs2bypass[4]}} & i0_result_e3_final[63:0]) |
                                       ({64{i1_rs2bypass[3]}} & i1_result_e4_final[63:0]) |
                                       ({64{i1_rs2bypass[2]}} & i0_result_e4_final[63:0]) |
                                       ({64{i1_rs2bypass[1]}} & i1_result_wb[63:0]) |
                                       ({64{i1_rs2bypass[0]}} & i0_result_wb[63:0]);
   
   
   
   
   
   
endmodule

// file "decode" is human readable file that has all of the instruction decodes defined and is part of git repo
// modify this file as needed

// to generate all the equations below from "decode" except legal equation:

// 1) coredecode -in decode > coredecode.e

// 2) espresso -Dso -oeqntott coredecode.e | addassign -pre out.  > equations

// to generate the legal (32b instruction is legal) equation below:

// 1) coredecode -in decode -legal > legal.e

// 2) espresso -Dso -oeqntott legal.e | addassign -pre out. > legal_equation

module dec_dec_ctl
   import swerv_types::*;
(
   input logic [31:0] inst,

   output dec_pkt_t out
   );

   logic [31:0] i;
   
   
   assign i[31:0] = inst[31:0];

   // inst[6:2] opcode
   logic load, load_fp, misc_mem, op_imm, auipc, op_imm_32;
   logic store, store_fp, amo, op, lui, op_32;
   logic madd, msub, nmsub, nmadd, op_fp, op_vec;
   logic branch, jalr, jal, system;
   assign load = !i[6]&!i[5]&!i[4]&!i[3]&!i[2];
   assign load_fp = !i[6]&!i[5]&!i[4]&!i[3]&i[2];
   assign misc_mem = !i[6]&!i[5]&!i[4]&i[3]&i[2];
   assign op_imm = !i[6]&!i[5]&i[4]&!i[3]&!i[2];
   assign auipc = !i[6]&!i[5]&i[4]&!i[3]&i[2];
   assign op_imm_32 = !i[6]&!i[5]&i[4]&i[3]&!i[2];

   assign store = !i[6]&i[5]&!i[4]&!i[3]&!i[2];
   assign store_fp = !i[6]&i[5]&!i[4]&!i[3]&i[2];
   assign amo = !i[6]&i[5]&!i[4]&i[3]&i[2];
   assign op = !i[6]&i[5]&i[4]&!i[3]&!i[2];
   assign lui = !i[6]&i[5]&i[4]&!i[3]&i[2];
   assign op_32 = !i[6]&i[5]&i[4]&i[3]&!i[2];

   assign madd = i[6]&!i[5]&!i[4]&!i[3]&!i[2];
   assign msub = i[6]&!i[5]&!i[4]&!i[3]&i[2];
   assign nmsub = i[6]&!i[5]&!i[4]&i[3]&!i[2];
   assign nmadd = i[6]&!i[5]&!i[4]&i[3]&i[2];
   assign op_fp = i[6]&!i[5]&i[4]&!i[3]&!i[2];
   // assign op_vec = i[6]&!i[5]&i[4]&!i[3]&i[2];

   assign branch = i[6]&i[5]&!i[4]&!i[3]&!i[2];
   assign jalr = i[6]&i[5]&!i[4]&!i[3]&i[2];
   assign jal = i[6]&i[5]&!i[4]&i[3]&i[2];
   assign system = i[6]&i[5]&i[4]&!i[3]&!i[2];
   
   // scalar
   logic sca_op3;

   // vector
   logic vec_ls, vec_amo;
   logic vec_op3;
   logic vec_csr, is_vec_csr;


   assign sca_op3 = (i[14]&!i[13]&!i[12]) | (!i[14]&!i[13]&i[12]) | (!i[14]&i[13]);
   //-----------------------vector & fpu ----------------------------
`ifdef ARA_EH1
   assign op_vec = (i[6]&!i[5]&i[4]&!i[3]&i[2]);   // vector major opcode
   assign vec_op3 = (!i[14]&!i[13]&!i[12]) | (i[14]&i[13]) | (i[14]&i[12]);
   assign vec_ls = (load_fp | store_fp) & vec_op3;
   assign vec_amo = amo & vec_op3;
   assign is_vec_csr =  (i[31:20] == 12'h008) | // vstart
                        (i[31:20] == 12'h009) | // vxsat
                        (i[31:20] == 12'h00a) | // vxrm
                        (i[31:20] == 12'h00f) | // vcsr
                        (i[31:20] == 12'hc20) | // vl
                        (i[31:20] == 12'hc21) | // vtype
                        (i[31:20] == 12'hc22);  // vlenb
   assign vec_csr =  system & (i[12]|i[13]) & is_vec_csr;
   assign out.vec =  op_vec | vec_ls | vec_amo | vec_csr;
`ifdef EH1_RV64D
   assign out.vec_fs1 = op_vec&i[14]&!i[13]&i[12]; //opfvf
   assign out.vec_fs2 = 1'b0;
   assign out.vec_fd = op_vec&!i[31]&i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[14]&!i[13]&i[12]; // opfvv -> vwfunary0(vfmv.f.s) write float gpr
`else
   assign out.vec_fs1 = 1'b0;
   assign out.vec_fs2 = 1'b0;
   assign out.vec_fd = 1'b0;
`endif // EH1_RV64D

`else
   assign op_vec = 1'b0;
   assign vec_op3 = 1'b0;
   assign vec_ls = 1'b0;
   assign vec_amo = 1'b0;
   assign is_vec_csr = 1'b0;
   assign vec_csr = 1'b0;
   assign out.vec = 1'b0;
   assign out.vec_fs1 = 1'b0;
   assign out.vec_fs2 = 1'b0;
   assign out.vec_fd = 1'b0;
`endif //ARA_EH1

`ifdef EH1_RV64D
   assign out.fpu = madd | msub | nmadd | nmsub | op_fp; // fpu
   assign out.rs3 = madd | msub | nmadd | nmsub; // require rs3
   assign out.fmadd = madd;
   assign out.fmsub = msub;
   assign out.fnmsub = nmsub;
   assign out.fnmadd = nmadd;
   assign out.fadd = (op_fp & !i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]);
   assign out.fsub = (op_fp & !i[31]&!i[30]&!i[29]&!i[28]&i[27]&!i[26]);
   assign out.fmul = (op_fp & !i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]);
   assign out.fdiv = (op_fp & !i[31]&!i[30]&!i[29]&i[28]&i[27]&!i[26]);
   assign out.fsqrt = (op_fp & !i[31]&i[30]&!i[29]&i[28]&i[27]&!i[26]);
   assign out.fsgnj = (op_fp & !i[31]&!i[30]&i[29]&!i[28]&!i[27]&!i[26]);
   assign out.fmin_max = (op_fp & !i[31]&!i[30]&i[29]&!i[28]&i[27]&!i[26]);
   assign out.fcvt_fti = (op_fp & i[31]&i[30]&!i[29]&!i[28]&!i[27]&!i[26]);
   assign out.fcvt_itf = (op_fp & i[31]&i[30]&!i[29]&i[28]&!i[27]&!i[26]);
   assign out.mv_fti = (op_fp & i[31]&i[30]&i[29]&!i[28]&!i[27]&!i[26]&!i[12]);
   assign out.mv_itf = (op_fp & i[31]&i[30]&i[29]&i[28]&!i[27]&!i[26]);
   assign out.fcmp = (op_fp & i[31]&!i[30]&i[29]&!i[28]&!i[27]&!i[26]);
   assign out.fclass = (op_fp & i[31]&i[30]&i[29]&!i[28]&!i[27]&!i[26]&i[12]);
   assign out.fcvt_dts = (op_fp & !i[31]&i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]);
   assign out.fcvt_std = (op_fp & !i[31]&i[30]&!i[29]&!i[28]&!i[27]&!i[26]&i[25]);

   assign out.is_double = i[25];
   assign out.ls_fp = (load_fp | store_fp) & sca_op3;

   assign out.fpu_wb_f_arf = out.fmadd | out.fmsub | out.fnmsub | out.fnmadd | out.fadd | out.fsub | out.fmul |
                              out.fdiv | out.fsqrt | out.fsgnj | out.fmin_max | out.fcvt_itf | out.mv_itf |
                              out.fcvt_dts | out.fcvt_std; // write float gpr
`else
	assign out.fpu = 1'b0;
	assign out.rs3 = 1'b0;
	assign out.fmadd = 1'b0;
	assign out.fmsub = 1'b0;
	assign out.fnmsub = 1'b0;
	assign out.fnmadd = 1'b0;
	assign out.fadd = 1'b0;
	assign out.fsub = 1'b0;
	assign out.fmul = 1'b0;
	assign out.fdiv = 1'b0;
	assign out.fsqrt = 1'b0;
	assign out.fsgnj = 1'b0;
	assign out.fmin_max = 1'b0;
	assign out.fcvt_fti = 1'b0;
	assign out.fcvt_itf = 1'b0;
	assign out.mv_fti = 1'b0;
	assign out.mv_itf = 1'b0;
	assign out.fcmp = 1'b0;
	assign out.fclass = 1'b0;
	assign out.fcvt_dts = 1'b0;
	assign out.fcvt_std = 1'b0;
	assign out.is_double = 1'b0;
	assign out.ls_fp = 1'b0;
   assign fpu_wb_f_arf = 1'b0;
`endif


   assign out.opt_32 = op_imm_32 | op_32;

   assign out.alu = misc_mem | auipc | lui | jalr | jal | branch | (system & ~vec_csr) | 
                    (( op | op_32) & !i[25]) | // except mul and div
                    (op_imm | op_imm_32);

   assign out.rs1 =  (system & !i[14]&!i[13]&i[12]) | // CSRRW
                     (system & i[13] & (i[15]|i[16]|i[17]|i[18]|i[19])) | // CSRRS/CSRRC rs1 and uimm donot equal to 0
                     // (system & !i[13] & (i[7]|i[8]|i[9]|i[10]|i[11])) | //
                     (system & !i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]&i[25]&!i[14]&!i[13]&!i[12]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | // SFENCE.VMA
                     op_imm | op | op_imm_32 | op_32 |
                     load | store | load_fp | store_fp | 
                     branch | jalr |
                     madd | msub | nmsub | nmadd |
                     op_fp |   // float
                     amo |     // amo
                     (op_vec & i[14]&!i[13]&!i[12]) | // vector opivx
                     (op_vec & i[14]&!i[13]&i[12]) | // vector opfvf
                     (op_vec & i[14]&i[13]&!i[12]) | // vector opmvx
                     (op_vec & (!i[31]|!i[30]) &i[14]&i[13]&i[12]); // vector vsetvl

   assign out.rs2 =  branch | store | store_fp | op | op_32 |
                     madd | msub | nmsub | nmadd |
                     op_fp |
                     amo |
                     (system & !i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]&i[25]&!i[14]&!i[13]&!i[12]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | // SFENCE.VMA
                    (op_vec & i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&i[14]&i[13]&i[12]) | // vector vsetvl
                    (load_fp & vec_op3 & i[27]&!i[26]); // vector strided load


   assign out.imm12 =   jalr |
                        ((op_imm | op_imm_32) & (i[13]|!i[12])) |
                        // (system & !i[13]&!i[12]) |
                        op_fp; // float control signals in i[31:20] pass by imm

   assign out.rd = load | op_imm | op_imm_32 | madd | nmsub | op_fp |
                   jalr | jal | 
                   op | system | auipc | lui | op_32 |
                   amo |
                   misc_mem | msub | nmadd |
                   (load_fp & sca_op3) |
                   (op_vec & ((!i[31]&i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[14]&i[13]&!i[12]) | // opmvv -> vwxunary0(vmv.x.s) write integer gpr
                              (i[14]&i[13]&i[12]) | //vset instr write integer gpr
                              (!i[31]&i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[14]&!i[13]&i[12])) //  opfvv -> vwfunary0(vfmv.f.s) write float gpr
                   );

   assign out.shimm5 = (op_imm | op_imm_32) & !i[13] & i[12];

   assign out.imm20 = jal | auipc | lui;

   assign out.pc = auipc | jal;

   assign out.amo = amo;

   assign out.amo_lr = (!i[31]&!i[30]&!i[29]&i[28]&!i[27]);
   assign out.amo_sc = (!i[31]&!i[30]&!i[29]&i[28]&i[27]);
   assign out.amo_swap = (!i[31]&!i[30]&!i[29]&!i[28]&i[27]);
   assign out.amo_add = (!i[31]&!i[30]&!i[29]&!i[28]&!i[27]);
   assign out.amo_xor = (!i[31]&!i[30]&i[29]&!i[28]&!i[27]);
   assign out.amo_and = (!i[31]&i[30]&i[29]&!i[28]&!i[27]);
   assign out.amo_or = (!i[31]&i[30]&!i[29]&!i[28]&!i[27]);
   assign out.amo_min = (i[31]&!i[30]&!i[29]&!i[28]&!i[27]);
   assign out.amo_max = (i[31]&!i[30]&i[29]&!i[28]&!i[27]);
   assign out.amo_minu = (i[31]&i[30]&!i[29]&!i[28]&!i[27]);
   assign out.amo_maxu = (i[31]&i[30]&i[29]&!i[28]&!i[27]);


   assign out.load = load | load_fp;
   assign out.store = store | store_fp;

   assign out.lsu =  load | store | //ls
                     ((load_fp | store_fp) & sca_op3) | // fls
                     amo & sca_op3;

   assign out.add =  ((op_imm | op_imm_32) & (!i[14]&!i[13]&!i[12])) | //addi
                     auipc |
                     ((op | op_32) & (!i[30]&!i[25]&!i[14]&!i[13]&!i[12])); // add

   assign out.sub =  ((op | op_32) & i[30] & !i[12]) | //sub
                     ((op | op_32) & !i[25] & !i[14] & i[13]) | //slt
                     ((op_imm | op_imm_32) & !i[14] & i[13]) | //slti
                     branch;

   assign out.land = ((op_imm | op_imm_32) & i[14] & i[13] & i[12]) |
                     ((op | op_32) & !i[25] & i[14] & i[13] & i[12]);

   assign out.lor =  /*misc_mem |*/
                     ((op | op_32) & !i[25] & i[14] & i[13] & !i[12]) |
                     lui |
                     ((op_imm | op_imm_32) & i[14] & i[13] & !i[12]) |
                     (system & (i[11]|i[10]|i[9]|i[8]|i[7]));  // csr read will execute OR


   assign out.lxor = ((op | op_32) & !i[25] & i[14] & !i[13] & !i[12]) |
                     ((op_imm | op_imm_32) & i[14] & !i[13] & !i[12]);



   assign out.sll = (op | op_32 | op_imm | op_imm_32) & !i[14] & !i[13] & i[12];

   assign out.sra = (op | op_32 | op_imm | op_imm_32) & i[30] & i[14] & !i[13] & i[12];

   assign out.srl = (op | op_32 | op_imm | op_imm_32) & !i[30] & i[14] & !i[13] & i[12];

   assign out.slt =  ((op | op_32) & !i[25] & !i[14] & i[13]) |
                     ((op_imm | op_imm_32) & !i[14] & i[13]);

   assign out.unsign =  ((op_imm | op_imm_32) & !i[14] & i[13] & i[12]) | //sltiu
                        (branch & i[13]) | //bltu, bgeu
                        (load & i[14]) | //lbu,lhu
                        ((op | op_32) & !i[25] & !i[14] & i[13] & i[12]) | //sltu
                        ((op | op_32) & i[25] & i[14] & i[12]); //divu, remu



   assign out.condbr = branch;

   assign out.beq = branch & !i[14] & !i[12];

   assign out.bne = branch & !i[14] & i[12];

   assign out.bge = branch & i[14] & i[12];

   assign out.blt = branch & i[14] & !i[12];

   assign out.jal = jalr | jal;


   assign out.by = (load | store) & !i[13] & !i[12];

   assign out.half = (load | store) & !i[13] & i[12];

   assign out.word = (load | store | load_fp | store_fp | amo) & i[13] & !i[12];

   assign out.dword = (load | store | load_fp | store_fp | amo) & i[13] & i[12];



   assign out.csr_read = system & (i[13]|i[7]|i[8]|i[9]|i[10]|i[11]) & (!is_vec_csr);

   assign out.csr_clr = system & i[12]&i[13] & (i[15]|i[16]|i[17]|i[18]|i[19]) & (!is_vec_csr);

   assign out.csr_set = system & !i[12]&i[13] & (i[15]|i[16]|i[17]|i[18]|i[19]) & (!is_vec_csr);

   assign out.csr_write = system & i[12]&!i[13] & (!is_vec_csr);

   assign out.csr_imm = system & i[14] & (!i[13]|i[15]|i[16]|i[17]|i[18]|i[19]) & (!is_vec_csr);

   assign out.presync = out.csr_read | /*out.csr_read | out.csr_clr | out.csr_set |*/
                        misc_mem |   //(!i[5]&!i[4]&i[3]) | //fence
                         (system & !i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]&i[25]&!i[14]&!i[13]&!i[12]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | // SFENCE.VMA
                        ((op | op_32) & i[25]&i[14]) |   //(i[25]&i[14]&!i[6]&i[5]&!i[2]) | //div
                        amo;

   assign out.postsync = out.csr_write | out.csr_clr | out.csr_set | /*out.csr_read | out.csr_clr | out.csr_set |*/
                         (misc_mem & i[12]) |  // (i[12]&!i[5]&!i[4]&i[3]) |   //fence.i
                         (system & !i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]&i[25]&!i[14]&!i[13]&!i[12]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | // SFENCE.VMA
                         (!i[22]&!i[13]&!i[12]&system) | //ebreak/ecall/sret/mret/dret
                         ((op | op_32) & i[25]&i[14]) |   //(i[25]&i[14]&!i[6]&i[5]&!i[2]) | //div
                         (out.fdiv | out.fsqrt) |
                         amo;

   assign out.ebreak = system & (i[31:20] == 12'h001) & !i[14]&!i[13]&!i[12];

   assign out.ecall = system & (i[31:20] == 12'h000) & !i[14]&!i[13]&!i[12];

   assign out.mret = system & (i[31:20] == 12'h302) & !i[14]&!i[13]&!i[12];

   assign out.sret = system & (i[31:20] == 12'h102) & !i[14]&!i[13]&!i[12];

   assign out.dret = system & (i[31:20] == 12'h7b2) & !i[14]&!i[13]&!i[12];

   assign out.wfi = system & (i[31:20] == 12'h105) & !i[14]&!i[13]&!i[12];

   assign out.mul = (op | op_32) & i[25] & !i[14];


   assign out.rs1_sign =   ((op | op_32) & i[25]&!i[14]&i[13]&!i[12]) | // MULHSU
                           ((op | op_32) & i[25]&!i[14]&!i[13]&i[12]); //MULH

   assign out.rs2_sign = ((op | op_32) & i[25]&!i[14]&!i[13]&i[12]); //MULH

   assign out.low = ((op | op_32) & i[25]&!i[14]&!i[13]&!i[12]);

   assign out.is_mulw = op_32;

   assign out.is_div_w = op_32;

   assign out.div = (op | op_32) & i[25] & i[14];

   assign out.rem = (op | op_32) & i[25] & i[13];

   assign out.fence =   misc_mem | 
                        (system & !i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]&i[25]&!i[14]&!i[13]&!i[12]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]); //sfence.vma

   assign out.sfence_vma = system & !i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]&i[25]&!i[14]&!i[13]&!i[12]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7];

   assign out.fence_i = misc_mem & i[12];

   assign out.pm_alu =  auipc | lui |
                        op_imm | op_imm_32 |
                        ((op | op_32) & !i[25]);

   assign out.legal = i[1]&i[0]&(
                        (system & !i[31]&!i[30]&i[28]&!i[27]&!i[26]&!i[25]&!i[24]&
                        !i[23]&!i[22]&i[21]&!i[20]&!i[19]&!i[18]&!i[17]&!i[16]&
                        !i[15]&!i[14]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | // MRET/SRET
                        (system & !i[31]&i[30]&i[29]&i[28]&i[27]&!i[26]&i[25]&i[24]&
                        !i[23]&!i[22]&i[21]&!i[20]&!i[19]&!i[18]&!i[17]&!i[16]&
                        !i[15]&!i[14]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | // DRET
                        (system & !i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]&!i[25]&!i[24]&
                        !i[23]&i[22]&!i[21]&i[20]&!i[19]&!i[18]&!i[17]&!i[16]&
                        !i[15]&!i[14]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | // WFI
                        (system & !i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]&i[25]&
                        !i[14]&!i[13]&!i[12]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | //SFENCE.VMA
                        (system & !i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&!i[24]&
                        !i[23]&!i[22]&!i[21]&!i[19]&!i[18]&!i[17]&!i[16]&
                        !i[15]&!i[14]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | // ECALL/EBREAK
                        (misc_mem & !i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&!i[24]&
                        !i[23]&!i[22]&!i[21]&!i[20]&!i[19]&!i[18]&!i[17]&!i[16]&
                        !i[15]&!i[14]&!i[13]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | // FENCE
                        (misc_mem & !i[31]&!i[30]&!i[29]&!i[28]&
                        !i[19]&!i[18]&!i[17]&!i[16]&
                        !i[15]&!i[14]&!i[13]&!i[12]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]) | 
                        (auipc | lui | jal) | // AUIPC/LUI/JAL
                        (jalr & !i[14]&!i[13]&!i[12]) | // JALR
                        (branch & !i[14]&!i[13]) | // BEQ/BNE
                        (branch & i[14]) | // BLT/BGE/BLTU/BGEU
                        ((load | store) & !i[14]) | // LB/LH/LW/LD/SB/SH/SW/SD
                        (load & i[14]&!i[13]) | // LBU/LHU
                        (load & i[14]&i[13]&!i[12]) | // LWU
                        (op_imm & i[13]) | // SLTI/SLTIU/ORI/ANDI
                        (op_imm & !i[12]) | // ADDI/SLTI/XORI/ORI
                        (op_imm & !i[31]&!i[29]&!i[28]&!i[27]&!i[26]&!i[13]&i[12]) | // SLLI/SRLI/SRAI
                        (op & !i[31]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&!i[14]&!i[13]&!i[12]) |  // ADD/SUB
                        (op & !i[31]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&i[14]&!i[13]&i[12]) | // SRL/SRA
                        (op & !i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]) | // ADD/SLL/SLT/SLTU/XOR/SRL/OR/AND/MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU
                        (op_imm_32 & !i[14]&!i[13]&!i[12]) | // ADDIW
                        ((op_imm_32 | op_32) & !i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&!i[14]&!i[13]&i[12]) | // SLLIW/SLLW
                        ((op_imm_32 | op_32) & !i[31]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&i[14]&!i[13]&i[12]) | // SRLIW/SRAIW/SRLW/SRAW
                        (op_32 & !i[31]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&!i[14]&!i[13]&!i[12]) | // ADDW/SUBW
                        (op_32 & !i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&i[25]&!i[14]&!i[13]&!i[12]) | // MULW
                        (op_32 & !i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&i[25]&i[14]) |  // DIVW/DIVWU/REMW/REMUW
                        (system & i[12]) | // CSRRW/CSRRC
                        (system & i[13]) | // CSRRS/CSRRC
                        (amo & !i[31]&!i[30]&!i[29]&!i[14]&i[13]) | // LR/SC/AMOSWAP
                        (amo & !i[28]&!i[27]&!i[14]&i[13]) | // AMOXXX
                        ((load_fp | store_fp) & !i[14]&i[13]) | //FLW/FSW/FLD/FSD
                        ((madd | msub | nmsub | nmadd) & !i[26]) | //FMADD.S(D)/FMSUB.S(D)/FNMSUB.S(D)/FNMADD.S(D)
                        (op_fp & !i[31]&!i[30]&!i[29]&!i[26]) | //FADD/FSUB/FMUL/FDIV
                        (op_fp & !i[31]&i[30]&!i[29]&i[28]&i[27]&!i[26]&!i[24]&!i[23]&!i[22]&!i[21]&!i[20]) | //FSQRT
                        (op_fp & !i[31]&!i[30]&i[29]&!i[28]&!i[27]&!i[26]&!i[14]&!i[13]) | //FSGNJ/FSGNJN
                        (op_fp & !i[31]&!i[30]&i[29]&!i[28]&!i[27]&!i[26]&!i[14]&i[13]&!i[12]) | //FSGNJX
                        (op_fp & !i[31]&!i[30]&i[29]&!i[28]&i[27]&!i[26]&!i[14]&!i[13]) | //FMIN/FMAX
                        (op_fp & i[31]&i[30]&!i[29]&!i[27]&!i[26]&!i[24]&!i[23]&!i[22]) | //FCVT.W.S(S.W)
                        (op_fp & i[31]&i[30]&i[29]&!i[27]&!i[26]&!i[24]&!i[23]&!i[22]&!i[21]&!i[20]&!i[14]&!i[13]&!i[12]) | //FMV
                        (op_fp & i[31]&!i[30]&i[29]&!i[28]&!i[27]&!i[26]&!i[14]&i[13]&!i[12]) | //FEQ
                        (op_fp & i[31]&!i[30]&i[29]&!i[28]&!i[27]&!i[26]&!i[14]&!i[13]) | //FLT/FLE
                        (op_fp & i[31]&i[30]&i[29]&!i[28]&!i[27]&!i[26]&!i[24]&!i[23]&!i[22]&!i[21]&!i[20]&!i[14]&!i[13]&i[12]) | //CLASS
                        (op_fp & !i[31]&i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&!i[24]&!i[23]&!i[22]&!i[21]&i[20]) | //FCVT.S.D
                        (op_fp & !i[31]&i[30]&!i[29]&!i[28]&!i[27]&!i[26]&i[25]&!i[24]&!i[23]&!i[22]&!i[21]&!i[20]) | //FCVT.D.S
                        (op_vec | vec_ls | vec_amo) //VEC
                     );
   
endmodule
