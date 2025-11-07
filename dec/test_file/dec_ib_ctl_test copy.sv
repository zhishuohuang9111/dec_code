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

module dec_ib_ctl
   import swerv_types::*;
(
   input logic   free_clk,                    // free clk
   input logic   active_clk,                  // active clk if not halt / pause

   input logic                 dbg_cmd_valid,  // valid dbg cmd

   input logic                 dbg_cmd_write,  // dbg cmd is write
   input logic [1:0]           dbg_cmd_type,   // dbg type
   input logic [1:0]           dbg_cmd_size,   // 00 - 1B, 01 - 2B, 10 - 4B, 11 - reserved
   input logic [63:0]          dbg_cmd_addr,   // expand to 63:0
   
   input logic exu_flush_final,                // all flush sources: primary/secondary alu's, trap

   input logic          dec_ib0_valid_eff_d,   // effective valid taking decode into account 
   input logic          dec_ib1_valid_eff_d,
   
   input br_pkt_t i0_brp,                      // i0 branch packet from aligner
   input br_pkt_t i1_brp,
   
   input logic   ifu_i0_pc4,                   // i0 is 4B inst else 2B
   input logic   ifu_i1_pc4,
   
   input logic   ifu_i0_valid,                 // i0 valid from ifu
   input logic   ifu_i1_valid,

   input logic ifu_i0_fetch_page_fault,
   input logic ifu_i1_fetch_page_fault,

   input logic   ifu_i0_icaf,                  // i0 instruction access fault
   input logic   ifu_i1_icaf,
   input logic   ifu_i0_icaf_f1,               // i0 has access fault on second fetch group
   input logic   ifu_i1_icaf_f1,               
   input logic   ifu_i0_perr,                  // i0 instruction parity error
   input logic   ifu_i1_perr,
   input logic   ifu_i0_sbecc,                 // i0 single-bit error
   input logic   ifu_i1_sbecc,
   input logic   ifu_i0_dbecc,                 // i0 double-bit error
   input logic   ifu_i1_dbecc,

   input logic [31:0]  ifu_i0_instr,           // i0 instruction from the aligner
   input logic [31:0]  ifu_i1_instr,

   input logic [63:1]  ifu_i0_pc,              // i0 pc from the aligner
   input logic [63:1] ifu_i1_pc,

   input logic   dec_i0_decode_d,              // i0 decode
   input logic   dec_i1_decode_d,
   
   
   input logic   rst_l,                        // test stuff
   input logic   clk,
   
   
   output logic dec_ib3_valid_d,               // ib3 valid
   output logic dec_ib2_valid_d,               // ib2 valid
   output logic dec_ib1_valid_d,               // ib1 valid
   output logic dec_ib0_valid_d,               // ib0 valid
   

   output logic [31:0] dec_i0_instr_d,         // i0 inst at decode
   output logic [31:0] dec_i1_instr_d,         // i1 inst at decode

   output logic [63:1] dec_i0_pc_d,            // i0 pc at decode
   output logic [63:1] dec_i1_pc_d,

   output logic dec_i0_pc4_d,                  // i0 is 4B inst else 2B
   output logic dec_i1_pc4_d,

   output br_pkt_t dec_i0_brp,                 // i0 branch packet at decode
   output br_pkt_t dec_i1_brp,

   output logic dec_i0_fetch_page_fault,
   output logic dec_i1_fetch_page_fault,
   output logic dec_i0_icaf_d,                 // i0 instruction access fault at decode
   output logic dec_i1_icaf_d,
   output logic dec_i0_icaf_f1_d,              // i0 instruction access fault at decode for f1 fetch group
   output logic dec_i0_perr_d,                 // i0 instruction parity error at decode
   output logic dec_i1_perr_d,
   output logic dec_i0_sbecc_d,                // i0 single-bit error at decode
   output logic dec_i1_sbecc_d,
   output logic dec_i0_dbecc_d,                // i0 double-bit error at decode
   output logic dec_i1_dbecc_d,
   output logic dec_debug_wdata_rs1_d,         // put debug write data onto rs1 source: machine is halted

   output logic dec_debug_fence_d,             // debug fence inst

   input logic [15:0] ifu_i0_cinst,            // 16b compressed inst from aligner
   input logic [15:0] ifu_i1_cinst,

   output logic [15:0] dec_i0_cinst_d,         // 16b compress inst at decode
   output logic [15:0] dec_i1_cinst_d,

   input  logic scan_mode

   );

`include "../include/global_param.sv"
   
   logic         flush_final;
   
   logic [3:0]   ibval_in, ibval;

   logic [31:0]  ib3_in, ib2_in, ib1_in, ib0_in;
   logic [31:0]  ib3, ib2, ib1, ib0;

   logic [69:0]  pc3_in, pc2_in, pc1_in, pc0_in;
   logic [69:0]  pc3, pc2, pc1, pc0;

   logic [15:0]  cinst3_in, cinst2_in, cinst1_in, cinst0_in;
   logic [15:0]  cinst3, cinst2, cinst1, cinst0;

   logic         write_i1_ib3, write_i0_ib3;
   logic         write_i1_ib2, write_i0_ib2;
   logic         write_i1_ib1, write_i0_ib1;
   logic         write_i0_ib0;

   logic         shift2, shift1, shift0;

   logic         shift_ib1_ib0, shift_ib2_ib1, shift_ib3_ib2;
   logic         shift_ib2_ib0;
   logic         shift_ib3_ib1;
   
   
   logic         ifu_i0_val, ifu_i1_val;
   logic         debug_valid;
   logic [4:0]   dreg;
   logic [11:0]  dcsr;
   logic [31:0]  ib0_debug_in;
   
//   logic                     debug_read_mem;
//   logic                     debug_write_mem;
   logic         debug_read;
   logic         debug_write;
   logic         debug_read_gpr;
   logic         debug_write_gpr;
   logic         debug_read_csr;
   logic         debug_write_csr;
   

   
   rvdff #(1) flush_upperff (.*, .clk(free_clk), .din(exu_flush_final), .dout(flush_final)); 

   logic [3:0]   ibvalid;
   // logic [7:0]   ibvalid;

   logic [3:0]   i0_wen;
   logic [3:1]   i1_wen;

   // logic [7:0]   i0_wen;  7 个位置的写使能信号
   // logic [7:1]   i1_wen;



   logic [3:0]   shift_ibval;

   logic [3:0]   ibwrite;

   // logic [7:0]   ibwrite;
   
   // ibvalid 表示本周期在各个槽上是否有指令（更新前的视角）
   assign ibvalid[3:0] = ibval[3:0] | i0_wen[3:0] | {i1_wen[3:1],1'b0};
   
   // 如果进行了刷新, 则 ibval_in[3:0] = 4'b0000,  下一周期的 ibval[3:0] 也为 0 了, 代表为 空, 可以写入 指令
   assign ibval_in[3:0] = (({4{shift0}} & ibvalid[3:0]) |
                           ({4{shift1}} & {1'b0, ibvalid[3:1]}) |  // 由 4'b1110  -->  4'b0111
                           ({4{shift2}} & {2'b0, ibvalid[3:2]})) & ~{4{flush_final}};  // 由 4'b1100  -->  4'b0011  

   rvdff #(4) ibvalff (.*, .clk(active_clk), .din(ibval_in[3:0]), .dout(ibval[3:0]));


   // 修改的
   assign ibvalid[7:0] = ibval[7:0] | i0_wen[7:0] | i1_wen[7:0];

   assign ibval_in[7:0] = (({4{shift0}} & ibvalid[7:0]) |
                           ({4{shift1}} & {1'b0, ibvalid[7:1]}) | 
                           ({4{shift2}} & {2'b0, ibvalid[7:2]})) ;  

   rvdff #(8) ibvalff (.*, .clk(active_clk), .din(ibval_in[7:0]), .dout(ibval[7:0]));


   
// only valid if there is room
   if (DEC_INSTBUF_DEPTH==4) begin
      assign ifu_i0_val = ifu_i0_valid & ~ibval[3] & ~flush_final;  // 写入 i0 至少要保证有 1 个空位
      assign ifu_i1_val = ifu_i1_valid & ~ibval[2] & ~flush_final;  // 写入 i1 至少要保证有 2 个空位,  所以检查  ~ibval[2]  
   end
   else begin
      // ~dec_ib0_valid_eff_d | ~dec_ib1_valid_eff_d: i0, i1 至少消耗了一条指令
      assign ifu_i0_val = ifu_i0_valid & (~dec_ib0_valid_eff_d | ~dec_ib1_valid_eff_d) & ~flush_final; // 
      // ~dec_ib0_valid_eff_d & ~dec_ib1_valid_eff_d: i0, i1 两条指令都消耗了
      assign ifu_i1_val = ifu_i1_valid & (~dec_ib0_valid_eff_d & ~dec_ib1_valid_eff_d) & ~flush_final;
   end
   
   
   assign i0_wen[0] = ~ibval[0]             & (ifu_i0_val | debug_valid);
   assign i0_wen[1] =  ibval[0] & ~ibval[1] & ifu_i0_val;
   assign i0_wen[2] =  ibval[1] & ~ibval[2] & ifu_i0_val;

   assign i0_wen[3] =  ibval[2] & ~ibval[3] & ifu_i0_val;  // 第二个曹不为空,  第三个曹 为空
   
   assign i1_wen[1] = ~ibval[0]             & ifu_i1_val;
   assign i1_wen[2] =  ibval[0] & ~ibval[1] & ifu_i1_val;
   assign i1_wen[3] =  ibval[1] & ~ibval[2] & ifu_i1_val;


   // 原本设计是   ib0  只能由  i0 指令写入

   // only valid if there is room
   if (DEC_INSTBUF_DEPTH==8) begin  // ifu_i0_valid: 写入的值 是有效的
      assign ifu_i0_val = ifu_i0_valid & ~ibval[7] & ~flush_final;  // 写入 i0 至少要保证有 1 个空位
      assign ifu_i1_val = ifu_i1_valid & ~ibval[6] & ~flush_final;  // 写入 i1 至少要保证有 2 个空位,  所以检查  ~ibval[2]  
   end
   else begin
      // ~dec_ib0_valid_eff_d | ~dec_ib1_valid_eff_d: i0, i1 至少消耗了一条指令
      assign ifu_i0_val = ifu_i0_valid & (~dec_ib0_valid_eff_d | ~dec_ib1_valid_eff_d) & ~flush_final; // 
      // ~dec_ib0_valid_eff_d & ~dec_ib1_valid_eff_d: i0, i1 两条指令都消耗了
      assign ifu_i1_val = ifu_i1_valid & (~dec_ib0_valid_eff_d & ~dec_ib1_valid_eff_d) & ~flush_final;
   end
   
   // 写入,  一次性写入 两条指令   ibval[0] = 1: 表示 0 号位置 有指令, 被占据了,  ifu_i0_val = 1: 表所当前的 i0 指令是有效的
   assign i0_wen[0] = ~ibval[0]             & ifu_i0_val ;
   assign i0_wen[1] =  ibval[0] & ~ibval[1] & ifu_i0_val;  // 0 号占据了,  1 号为空
   assign i0_wen[2] =  ibval[1] & ~ibval[2] & ifu_i0_val;
   assign i0_wen[3] =  ibval[2] & ~ibval[3] & ifu_i0_val;

   assign i0_wen[4] =  ibval[3] & ~ibval[4] & ifu_i0_val;
   assign i0_wen[5] =  ibval[4] & ~ibval[5] & ifu_i0_val;
   assign i0_wen[6] =  ibval[5] & ~ibval[6] & ifu_i0_val;
   assign i0_wen[7] =  ibval[6] & ~ibval[7] & ifu_i0_val;  // 第 6 个曹不为空,  第 7 个曹 为空
   

   assign i1_wen[1] = ~ibval[0]             & ifu_i1_val;  //  0 个曹 开始为空, i1 写入第 1 个曹
   assign i1_wen[2] =  ibval[0] & ~ibval[1] & ifu_i1_val;
   assign i1_wen[3] =  ibval[1] & ~ibval[2] & ifu_i1_val;

   assign i1_wen[4] =  ibval[2] & ~ibval[3] & ifu_i1_val;  // 0 - 2 曹不为空,  第 3 个曹 开始为空, i1 写入第 4 个曹
   assign i1_wen[5] =  ibval[3] & ~ibval[4] & ifu_i1_val;
   assign i1_wen[6] =  ibval[4] & ~ibval[5] & ifu_i1_val;
   assign i1_wen[7] =  ibval[5] & ~ibval[6] & ifu_i1_val;  // 0 - 5 曹不为空,  第 6 个曹 开始为空, i1 写入第 7 个曹

   

   /*
   
   // start trace

   if (DEC_INSTBUF_DEPTH==4) begin
      assign cinst3_in[15:0] = ({16{write_i0_ib3}} & ifu_i0_cinst[15:0]) |
                               ({16{write_i1_ib3}} & ifu_i1_cinst[15:0]);
      
      rvdffe #(16) cinst3ff (.*, .en(ibwrite[3]), .din(cinst3_in[15:0]), .dout(cinst3[15:0]));
      
      assign cinst2_in[15:0] = ({16{write_i0_rtl/dec/dec_ib_ctl_test.svib2}} & ifu_i0_cinst[15:0]) |
                               ({16{write_i1_ib2}} & ifu_i1_cinst[15:0]) |
                               ({16{shift_ib3_ib2}} & cinst3[15:0]);
      
      rvdffe #(16) cinst2ff (.*, .en(ibwrite[2]), .din(cinst2_in[15:0]), .dout(cinst2[15:0]));
   end // if (DEC_INSTBUF_DEPTH==4)
   else begin
      assign cinst3 = '0;
      assign cinst2 = '0;
   end
   
   assign cinst1_in[15:0] = ({16{write_i0_ib1}} & ifu_i0_cinst[15:0]) |
                            ({16{write_i1_ib1}} & ifu_i1_cinst[15:0]) |
                            ({16{shift_ib2_ib1}} & cinst2[15:0]) |
                            ({16{shift_ib3_ib1}} & cinst3[15:0]);
   
   rvdffe #(16) cinst1ff (.*, .en(ibwrite[1]), .din(cinst1_in[15:0]), .dout(cinst1[15:0]));


   assign cinst0_in[15:0] = ({16{write_i0_ib0}} & ifu_i0_cinst[15:0]) |
                            ({16{shift_ib1_ib0}} & cinst1[15:0]) |
                            ({16{shift_ib2_ib0}} & cinst2[15:0]);
   
   rvdffe #(16) cinst0ff (.*, .en(ibwrite[0]), .din(cinst0_in[15:0]), .dout(cinst0[15:0]));

   assign dec_i0_cinst_d[15:0] = cinst0[15:0];

   assign dec_i1_cinst_d[15:0] = cinst1[15:0];   
   
   // end trace

   */
   
   
   // pc tracking


   assign ibwrite[3:0] = {  write_i0_ib3 | write_i1_ib3,   // 写入第 3 个 空位
                            write_i0_ib2 | write_i1_ib2 | shift_ib3_ib2,   // 写入第 2 个 空位
                            write_i0_ib1 | write_i1_ib1 | shift_ib2_ib1 | shift_ib3_ib1,  // 写入第 1 个 空位
                            write_i0_ib0 | shift_ib1_ib0 | shift_ib2_ib0  // 写入第 0 个 空位
                            };

   // 修改的
   assign ibwrite[7:0] = {  write_i0_ib7 | write_i1_ib7,   
                            write_i0_ib6 | write_i1_ib6 | shift_ib7_ib6, 

                            write_i0_ib5 | write_i1_ib5 | shift_ib6_ib5 | shift_ib7_ib5, 
                            write_i0_ib4 | write_i1_ib4 | shift_ib5_ib4 | shift_ib6_ib4,  

                            write_i0_ib3 | write_i1_ib3 | shift_ib4_ib3 | shift_ib5_ib3, 
                            write_i0_ib2 | write_i1_ib2 | shift_ib3_ib2 | shift_ib4_ib2,  

                            write_i0_ib1 | write_i1_ib1 | shift_ib2_ib1 | shift_ib3_ib1,  

                            write_i0_ib0 | write_i1_ib0 | shift_ib1_ib0 | shift_ib2_ib0  // i0, i1 指令 都可以写入  0 槽
                            };                        

   /*
   logic [69:0]  ifu_i1_pcdata, ifu_i0_pcdata;
   
   assign ifu_i1_pcdata[69:0] = {ifu_i1_fetch_page_fault, ifu_i1_icaf_f1, ifu_i1_dbecc, ifu_i1_sbecc, ifu_i1_perr, ifu_i1_icaf, 
                                  ifu_i1_pc[63:1], ifu_i1_pc4 };   
   assign ifu_i0_pcdata[69:0] = {ifu_i0_fetch_page_fault, ifu_i0_icaf_f1, ifu_i0_dbecc, ifu_i0_sbecc, ifu_i0_perr, ifu_i0_icaf, 
                                  ifu_i0_pc[63:1], ifu_i0_pc4 };
   
   if (DEC_INSTBUF_DEPTH==4) begin
      assign pc3_in[69:0] = ({70{write_i0_ib3}} & ifu_i0_pcdata[69:0]) |
                            ({70{write_i1_ib3}} & ifu_i1_pcdata[69:0]);
      
      rvdffe #(70) pc3ff (.*, .en(ibwrite[3]), .din(pc3_in[69:0]), .dout(pc3[69:0]));
      
      assign pc2_in[69:0] = ({70{write_i0_ib2}} & ifu_i0_pcdata[69:0]) |
                            ({70{write_i1_ib2}} & ifu_i1_pcdata[69:0]) |
                            ({70{shift_ib3_ib2}} & pc3[69:0]);
      
      rvdffe #(70) pc2ff (.*, .en(ibwrite[2]), .din(pc2_in[69:0]), .dout(pc2[69:0]));
   end // if (DEC_INSTBUF_DEPTH==4)
   else begin
      assign pc3 = '0;
      assign pc2 = '0;
   end
   
   assign pc1_in[69:0] = ({70{write_i0_ib1}} & ifu_i0_pcdata[69:0]) |
                         ({70{write_i1_ib1}} & ifu_i1_pcdata[69:0]) |
                         ({70{shift_ib2_ib1}} & pc2[69:0]) |
                         ({70{shift_ib3_ib1}} & pc3[69:0]);
   
   rvdffe #(70) pc1ff (.*, .en(ibwrite[1]), .din(pc1_in[69:0]), .dout(pc1[69:0]));


   assign pc0_in[69:0] = ({70{write_i0_ib0}} & ifu_i0_pcdata[69:0]) |
                         ({70{shift_ib1_ib0}} & pc1[69:0]) |
                         ({70{shift_ib2_ib0}} & pc2[69:0]);
   
   rvdffe #(70) pc0ff (.*, .en(ibwrite[0]), .din(pc0_in[69:0]), .dout(pc0[69:0]));

   assign dec_i0_fetch_page_fault = pc0[69];
   assign dec_i1_fetch_page_fault = pc1[69];

   assign dec_i0_icaf_f1_d = pc0[68];   // icaf's can only decode as i0
   
   assign dec_i1_dbecc_d = pc1[67];
   assign dec_i0_dbecc_d = pc0[67];   

   assign dec_i1_sbecc_d = pc1[66];
   assign dec_i0_sbecc_d = pc0[66];   

   assign dec_i1_perr_d = pc1[65];
   assign dec_i0_perr_d = pc0[65];   

   assign dec_i1_icaf_d = pc1[64];
   assign dec_i0_icaf_d = pc0[64];   
   
   assign dec_i1_pc_d[63:1] = pc1[63:1];   
   assign dec_i0_pc_d[63:1] = pc0[63:1];

   assign dec_i1_pc4_d = pc1[0];   
   assign dec_i0_pc4_d = pc0[0];

   */

   /*
   // branch prediction

   logic [$bits(br_pkt_t)-1:0] bp3_in,bp3,bp2_in,bp2,bp1_in,bp1,bp0_in,bp0;

   if (DEC_INSTBUF_DEPTH==4) begin   
      assign bp3_in = ({$bits(br_pkt_t){write_i0_ib3}} & i0_brp) |
                      ({$bits(br_pkt_t){write_i1_ib3}} & i1_brp);
      
      rvdffe #($bits(br_pkt_t)) bp3ff (.*, .en(ibwrite[3]), .din(bp3_in), .dout(bp3));
      
      assign bp2_in = ({$bits(br_pkt_t){write_i0_ib2}} & i0_brp) |
                      ({$bits(br_pkt_t){write_i1_ib2}} & i1_brp) |
                      ({$bits(br_pkt_t){shift_ib3_ib2}} & bp3);
      
      rvdffe #($bits(br_pkt_t)) bp2ff (.*, .en(ibwrite[2]), .din(bp2_in), .dout(bp2));
   end // if (DEC_INSTBUF_DEPTH==4)
   else begin
      assign bp3 = '0;
      assign bp2 = '0;
   end
   
   assign bp1_in = ({$bits(br_pkt_t){write_i0_ib1}} & i0_brp) |
                   ({$bits(br_pkt_t){write_i1_ib1}} & i1_brp) |
                   ({$bits(br_pkt_t){shift_ib2_ib1}} & bp2) |
                   ({$bits(br_pkt_t){shift_ib3_ib1}} & bp3);
   
   rvdffe #($bits(br_pkt_t)) bp1ff (.*, .en(ibwrite[1]), .din(bp1_in), .dout(bp1));
   


   assign bp0_in = ({$bits(br_pkt_t){write_i0_ib0}} & i0_brp) |
                   ({$bits(br_pkt_t){shift_ib1_ib0}} & bp1) |
                   ({$bits(br_pkt_t){shift_ib2_ib0}} & bp2);
   
   rvdffe #($bits(br_pkt_t)) bp0ff (.*, .en(ibwrite[0]), .din(bp0_in), .dout(bp0));

   */

   // instruction buffers

   if (DEC_INSTBUF_DEPTH==4) begin
      assign ib3_in[31:0] = ({32{write_i0_ib3}} & ifu_i0_instr[31:0]) |
                            ({32{write_i1_ib3}} & ifu_i1_instr[31:0]);
      
      rvdffe #(32) ib3ff (.*, .en(ibwrite[3]), .din(ib3_in[31:0]), .dout(ib3[31:0]));
      
      assign ib2_in[31:0] = ({32{write_i0_ib2}} & ifu_i0_instr[31:0]) |
                            ({32{write_i1_ib2}} & ifu_i1_instr[31:0]) |
                            ({32{shift_ib3_ib2}} & ib3[31:0]);
      
      rvdffe #(32) ib2ff (.*, .en(ibwrite[2]), .din(ib2_in[31:0]), .dout(ib2[31:0]));
   end // if (DEC_INSTBUF_DEPTH==4)
   else begin
      assign ib3 = '0;
      assign ib2 = '0;
   end
   
   assign ib1_in[31:0] = ({32{write_i0_ib1}} & ifu_i0_instr[31:0]) |
                         ({32{write_i1_ib1}} & ifu_i1_instr[31:0]) |
                         ({32{shift_ib2_ib1}} & ib2[31:0]) |
                         ({32{shift_ib3_ib1}} & ib3[31:0]);
   
   rvdffe #(32) ib1ff (.*, .en(ibwrite[1]), .din(ib1_in[31:0]), .dout(ib1[31:0]));


// GPR accesses

// put reg to read on rs1
// read ->   or %x0,  %reg,%x0      {000000000000,reg[4:0],110000000110011}

// put write date on rs1
// write ->  or %reg, %x0, %x0      {00000000000000000110,reg[4:0],0110011}


// CSR accesses
// csr is of form rd, csr, rs1

// read  -> csrrs %x0, %csr, %x0     {csr[11:0],00000010000001110011}

// put write data on rs1
// write -> csrrw %x0, %csr, %x0     {csr[11:0],00000001000001110011}

// abstract memory command not done here   
   assign debug_valid = dbg_cmd_valid & (dbg_cmd_type[1:0] != 2'h2);   
   

   assign debug_read  = debug_valid & ~dbg_cmd_write;
   assign debug_write = debug_valid &  dbg_cmd_write;   

   assign debug_read_gpr  = debug_read  & (dbg_cmd_type[1:0]==2'h0);
   assign debug_write_gpr = debug_write & (dbg_cmd_type[1:0]==2'h0);   
   assign debug_read_csr  = debug_read  & (dbg_cmd_type[1:0]==2'h1);
   assign debug_write_csr = debug_write & (dbg_cmd_type[1:0]==2'h1);

   assign dreg[4:0]  = dbg_cmd_addr[4:0];   
   assign dcsr[11:0] = dbg_cmd_addr[11:0];   
   

   assign ib0_debug_in[31:0] = ({32{debug_read_gpr}}  & {12'b000000000000,dreg[4:0],15'b110000000110011}) |
                               ({32{debug_write_gpr}} & {20'b00000000000000000110,dreg[4:0],7'b0110011}) |
                               ({32{debug_read_csr}}  & {dcsr[11:0],20'b00000010000001110011}) |
                               ({32{debug_write_csr}} & {dcsr[11:0],20'b00000001000001110011});


   // machine is in halted state, pipe empty, write will always happen next cycle
   rvdff #(1) debug_wdata_rs1ff (.*, .clk(free_clk), .din(debug_write_gpr | debug_write_csr), .dout(dec_debug_wdata_rs1_d));


   // special fence csr for use only in debug mode

   logic                       debug_fence_in;
   
   assign debug_fence_in = debug_write_csr & (dcsr[11:0] == 12'h7c4);
   
   rvdff #(1) debug_fence_ff (.*,  .clk(free_clk), .din(debug_fence_in),  .dout(dec_debug_fence_d));      
   
   
   assign ib0_in[31:0] = ({32{write_i0_ib0}} & ((debug_valid) ? ib0_debug_in[31:0] : ifu_i0_instr[31:0])) |
                         ({32{shift_ib1_ib0}} & ib1[31:0]) |
                         ({32{shift_ib2_ib0}} & ib2[31:0]);
   
   rvdffe #(32) ib0ff (.*, .en(ibwrite[0]), .din(ib0_in[31:0]), .dout(ib0[31:0]));




      // instruction buffers    修改的  

   assign ib7_in[31:0] = ({32{write_i0_ib7}} & ifu_i0_instr[31:0]) |  // ifu_i0_instr[31:0]:  需要改为 入队 的指令
                         ({32{write_i1_ib7}} & ifu_i1_instr[31:0]);
      
   rvdffe #(32) ib7ff (.*, .en(ibwrite[7]), .din(ib7_in[31:0]), .dout(ib7[31:0]));
      
   assign ib6_in[31:0] = ({32{write_i0_ib7}} & ifu_i0_instr[31:0]) |
                         ({32{write_i1_ib7}} & ifu_i1_instr[31:0]) |

                         ({32{shift_ib7_ib6}} & ib7[31:0]);

   rvdffe #(32) ib6ff (.*, .en(ibwrite[6]), .din(ib6_in[31:0]), .dout(ib6[31:0]));

   assign ib5_in[31:0] = ({32{write_i0_ib5}} & ifu_i0_instr[31:0]) |
                         ({32{write_i1_ib5}} & ifu_i1_instr[31:0]);

                         ({32{shift_ib6_ib5}} & ib6[31:0]);
                         ({32{shift_ib7_ib5}} & ib7[31:0]);
      
   rvdffe #(32) ib5ff (.*, .en(ibwrite[5]), .din(ib5_in[31:0]), .dout(ib5[31:0]));
      
   assign ib4_in[31:0] = ({32{write_i0_ib4}} & ifu_i0_instr[31:0]) |
                         ({32{write_i1_ib4}} & ifu_i1_instr[31:0]) |

                         ({32{shift_ib5_ib4}} & ib5[31:0]);
                         ({32{shift_ib6_ib4}} & ib6[31:0]);
      
   rvdffe #(32) ib4ff (.*, .en(ibwrite[4]), .din(ib4_in[31:0]), .dout(ib4[31:0]));

   assign ib3_in[31:0] = ({32{write_i0_ib3}} & ifu_i0_instr[31:0]) |
                         ({32{write_i1_ib3}} & ifu_i1_instr[31:0]);

                         ({32{shift_ib4_ib3}} & ib4[31:0]);
                         ({32{shift_ib5_ib3}} & ib5[31:0]);
      
   rvdffe #(32) ib3ff (.*, .en(ibwrite[3]), .din(ib3_in[31:0]), .dout(ib3[31:0]));
      
   assign ib2_in[31:0] = ({32{write_i0_ib2}} & ifu_i0_instr[31:0]) |
                         ({32{write_i1_ib2}} & ifu_i1_instr[31:0]) |

                         ({32{shift_ib3_ib2}} & ib3[31:0]);
                         ({32{shift_ib4_ib2}} & ib2[31:0]);
      
   rvdffe #(32) ib2ff (.*, .en(ibwrite[2]), .din(ib2_in[31:0]), .dout(ib2[31:0]));

   assign ib1_in[31:0] = ({32{write_i0_ib1}} & ifu_i0_instr[31:0]) |
                         ({32{write_i1_ib1}} & ifu_i1_instr[31:0]) |

                         ({32{shift_ib2_ib1}} & ib2[31:0]) |
                         ({32{shift_ib3_ib1}} & ib3[31:0]);
   
   rvdffe #(32) ib1ff (.*, .en(ibwrite[1]), .din(ib1_in[31:0]), .dout(ib1[31:0]));

   assign ib0_in[31:0] = ({32{write_i0_ib0}} & ( ifu_i0_instr[31:0])) |  // i0 指令写入 0 槽
                         ({32{write_i1_ib0}} & ( ifu_i0_instr[31:0])) |  // i1 指令写入 0 槽

                         ({32{shift_ib1_ib0}} & ib1[31:0]) |
                         ({32{shift_ib2_ib0}} & ib2[31:0]);
   
   rvdffe #(32) ib0ff (.*, .en(ibwrite[0]), .din(ib0_in[31:0]), .dout(ib0[31:0]));


   assign dec_i0_instr_d[31:0] = ib0[31:0];
   assign dec_i1_instr_d[31:0] = ib1[31:0]; 

   // 还要实现一个读取的信号,  读取  ib0, ib1 的值




   assign dec_ib3_valid_d = ibval[3];
   assign dec_ib2_valid_d = ibval[2];
   assign dec_ib1_valid_d = ibval[1];
   assign dec_ib0_valid_d = ibval[0];   

   // 修改的
   assign dec_ib7_valid_d = ibval[7];
   assign dec_ib6_valid_d = ibval[6];
   assign dec_ib5_valid_d = ibval[5];
   assign dec_ib4_valid_d = ibval[4];

   assign dec_ib3_valid_d = ibval[3];
   assign dec_ib2_valid_d = ibval[2];
   assign dec_ib1_valid_d = ibval[1];
   assign dec_ib0_valid_d = ibval[0];
   


  

   assign dec_i0_brp = bp0;
   assign dec_i1_brp = bp1;   
   
   
   assign shift1 = dec_i0_decode_d & ~dec_i1_decode_d;  // 从队列读取的, 消耗了 i0 一条指令

   assign shift2 = dec_i0_decode_d & dec_i1_decode_d;  // 从队列读取的, 消耗了 i0, i1 两条指令

   assign shift0 = ~dec_i0_decode_d;  // 从队列读取的, 没有 消耗指令
   
   
   // compute shifted ib valids to determine where to write
   assign shift_ibval[3:0] = ({4{shift1}} & {1'b0, ibval[3:1] }) |
                             ({4{shift2}} & {2'b0, ibval[3:2]}) |
                             ({4{shift0}} & ibval[3:0]);

   assign write_i0_ib0 = ~shift_ibval[0]                & (ifu_i0_val | debug_valid);
   assign write_i0_ib1 =  shift_ibval[0] & ~shift_ibval[1] & ifu_i0_val;
   assign write_i0_ib2 =  shift_ibval[1] & ~shift_ibval[2] & ifu_i0_val;
   assign write_i0_ib3 =  shift_ibval[2] & ~shift_ibval[3] & ifu_i0_val;
   
   assign write_i1_ib1 = ~shift_ibval[0]                & ifu_i1_val;
   assign write_i1_ib2 =  shift_ibval[0] & ~shift_ibval[1] & ifu_i1_val;
   assign write_i1_ib3 =  shift_ibval[1] & ~shift_ibval[2] & ifu_i1_val;


   // 修改的    写使能
   // compute shifted ib valids to determine where to write
   assign shift_ibval[7:0] = ({4{shift1}} & {1'b0, ibval[7:1] }) |
                             ({4{shift2}} & {2'b0, ibval[7:2]}) |
                             ({4{shift0}} & ibval[7:0]);


   assign write_i0_ib0 = ~shift_ibval[0]                & ifu_i0_val ;
   assign write_i0_ib1 =  shift_ibval[0] & ~shift_ibval[1] & ifu_i0_val;
   assign write_i0_ib2 =  shift_ibval[1] & ~shift_ibval[2] & ifu_i0_val;
   assign write_i0_ib3 =  shift_ibval[2] & ~shift_ibval[3] & ifu_i0_val;

   assign write_i0_ib4 =  shift_ibval[3] & ~shift_ibval[4] & ifu_i0_val; // 槽0~3占用、槽4空，则写入 i0
   assign write_i0_ib5 =  shift_ibval[4] & ~shift_ibval[5] & ifu_i0_val;
   assign write_i0_ib6 =  shift_ibval[5] & ~shift_ibval[6] & ifu_i0_val;
   assign write_i0_ib7 =  shift_ibval[6] & ~shift_ibval[7] & ifu_i0_val; // 槽0~6占用、槽7空，则 槽7 写入 i0
   
   assign write_i1_ib0 = ;  // 队列中,  i1 指令也可以写入  第 0 个    ？？？？
   assign write_i1_ib1 = ~shift_ibval[0]                & ifu_i1_val;  // 槽 0 空，则 槽1 写入 i1
   assign write_i1_ib2 =  shift_ibval[0] & ~shift_ibval[1] & ifu_i1_val;
   assign write_i1_ib3 =  shift_ibval[1] & ~shift_ibval[2] & ifu_i1_val;  // 槽0~1占用、槽2空，则 槽3 写入 i1

   assign write_i1_ib4 =  shift_ibval[2] & ~shift_ibval[3] & ifu_i1_val;
   assign write_i1_ib5 =  shift_ibval[3] & ~shift_ibval[4] & ifu_i1_val;
   assign write_i1_ib6 =  shift_ibval[4] & ~shift_ibval[5] & ifu_i1_val;
   assign write_i1_ib7 =  shift_ibval[5] & ~shift_ibval[6] & ifu_i1_val; // 槽0~5占用、槽6空，则 槽7 写入 i1





   assign shift_ib1_ib0 = shift1 & ibval[1];
   assign shift_ib2_ib1 = shift1 & ibval[2];
   assign shift_ib3_ib2 = shift1 & ibval[3];   
   
   assign shift_ib2_ib0 = shift2 & ibval[2];
   assign shift_ib3_ib1 = shift2 & ibval[3];



   // 修改的     移位使能
   // 消耗 1 条指令的 移位
   assign shift_ib1_ib0 = shift1 & ibval[1];
   assign shift_ib2_ib1 = shift1 & ibval[2];
   assign shift_ib3_ib2 = shift1 & ibval[3];

   assign shift_ib4_ib3 = shift1 & ibval[4];
   assign shift_ib5_ib4 = shift1 & ibval[5];  
   assign shift_ib6_ib5 = shift1 & ibval[6];
   assign shift_ib7_ib6 = shift1 & ibval[7];    // 将 第 7 个值 写入 第 6 个  
   
   // 消耗 2 条指令的 移位
   assign shift_ib2_ib0 = shift2 & ibval[2];  // 将 FIFO 中的 2 槽 移入到 0 槽的位置中
   assign shift_ib3_ib1 = shift2 & ibval[3];

   assign shift_ib4_ib2 = shift2 & ibval[4];
   assign shift_ib5_ib3 = shift2 & ibval[5];
   assign shift_ib6_ib4 = shift2 & ibval[6];
   assign shift_ib7_ib5 = shift2 & ibval[7];
   

   
endmodule


/*
      添加的队列需要实现的是,  向 队列 里面进行写指令,   读取指令的时候, 将 后面的指令 移动 到队头 


      在检测到依赖之后, 每周期 入队的指令可能为 0, 1, 2 条


      队列 里面 只存储 发射出来的实际的指令

      有 0 -7  总共 8 个存储空间,  i0, i1 都可以向这 8 个空间里面写入
      第 0 个 和 第 1 个进行 写入给 i0, i1 指令,  2 - 7 个依次往前移     刷新是, 每次前移动 2 条


      dec 阶段的 i0 有依赖前面的指令,  当前 dec 的 i0 i1 都入队,  后续的也进行入队
      dec 阶段的 i1 有依赖前面的指令,  当前 dec 的 i1 入队,  后续的也进行入队

      dec_i0_decode_d = 1 表示 i0 有效, 可以写入队列


      读 队列 的时候, 也需要 旁段 当前 从队列读取的指令, 是否进行了消耗 指令


      队列不能同时进行 写指令 和 读指令,   指令读取,  直到 队列 为空


      什么时候会 shift, 消耗掉指令的时候,  也就是 开始读取的时候


      




*/
ifu_i0_valid