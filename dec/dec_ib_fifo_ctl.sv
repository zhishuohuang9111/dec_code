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

module dec_ib_fifo_ctl
   import swerv_types::*;
(
   input logic   free_clk,                    // free clk
   input logic   active_clk,                  // active clk if not halt / pause


   input logic exu_flush_final,                // all flush sources: primary/secondary alu's, trap

   // 上一周期从队列读取的两条指令是否消耗掉了, 0 条, 1 条 或者两条, 通过移位 shift 来同时    
   // input logic          dec_ib0_valid_eff_fifo,   // effective valid taking decode into account 
   // input logic          dec_ib1_valid_eff_fifo,

   input logic          dec_ib0_valid_eff_d,   // effective valid taking decode into account 
   input logic          dec_ib1_valid_eff_d,

   
   // 修改为 判断 当前指令 是否为 依赖指令 的信号
   // input logic   ifu_i0_valid,                 // i0 valid from ifu
   // input logic   ifu_i1_valid,

   
   // 传入, 表示 消耗了 几条指令
   input logic   dec_i0_decode_d,              // i0 decode
   input logic   dec_i1_decode_d,

   // Added
   input logic   i0_load_block_d,              // 是否进入 写队列
   input logic   i1_load_block_d,   

      //  值预测正确,  清空队列    所依赖的 load 指令到了 e4 阶段, 并且 值预测信号正确
   
   // exu_i0_flush_vp_e4, exu_i1_flush_vp_e4 = 0: 值预测信号正确
   input logic        exu_i0_flush_vp_e4,    // 是否需要进行刷新队列, 或者 从队列里去读
   input logic        exu_i1_flush_vp_e4,

   input logic          load_match_i0_e4,  // 依赖的 load 指令 流水到 e4 阶段了  
   input logic          load_match_i1_e4,
   
   
   input logic   rst_l,                        // test stuff
   input logic   clk,

   
   input  logic scan_mode,

   // 从 buffer 传入的指令
   input logic dec_ib3_valid_buffer,               // ib3 valid
   input logic dec_ib2_valid_buffer,               // ib2 valid
   input logic dec_ib1_valid_buffer,               // ib1 valid
   input logic dec_ib0_valid_buffer,               // ib0 valid
   

   input logic [31:0] dec_i0_instr_buffer,         // i0 inst at decode
   input logic [31:0] dec_i1_instr_buffer,         // i1 inst at decode


   input br_pkt_t dec_i0_brp_buffer,                 // i0 branch packet at decode
   input br_pkt_t dec_i1_brp_buffer,

   input logic [15:0] dec_i0_cinst_buffer,         // 16b compress inst at decode
   input logic [15:0] dec_i1_cinst_buffer,

   input logic dec_debug_wdata_rs1_buffer,         // put debug write data onto rs1 source: machine is halted

   input logic dec_debug_fence_buffer,             // debug fence inst

   input logic [69:0] pc0_buffer,
   input logic [69:0] pc1_buffer,


   // 从 FIFO 里面读取的指令
   output logic dec_ib3_valid_fifo,               // ib3 valid
   output logic dec_ib2_valid_fifo,               // ib2 valid
   output logic dec_ib1_valid_fifo,               // ib1 valid
   output logic dec_ib0_valid_fifo,               // ib0 valid
   

   output logic [31:0] dec_i0_instr_fifo,         // i0 inst at decode
   output logic [31:0] dec_i1_instr_fifo,         // i1 inst at decode



   output br_pkt_t dec_i0_brp_fifo,                 // i0 branch packet at decode
   output br_pkt_t dec_i1_brp_fifo,

   output logic [15:0] dec_i0_cinst_fifo,         // 16b compress inst at decode
   output logic [15:0] dec_i1_cinst_fifo,

   output logic dec_debug_wdata_rs1_fifo,         // put debug write data onto rs1 source: machine is halted

   output logic dec_debug_fence_fifo,             // debug fence inst

   output logic [69:0] pc0_fifo,
   output logic [69:0] pc1_fifo

   );

`include "../include/global_param.sv"
   
   logic         flush_final, flush_fifo;   // 有依赖, 但是值预测正确, 清空 队列
   
   logic [7:0]   ibval_in, ibval;

   logic [31:0]  ib7_in, ib6_in, ib5_in, ib4_in, ib3_in, ib2_in, ib1_in, ib0_in;
   logic [31:0]  ib7, ib6, ib5, ib4, ib3, ib2, ib1, ib0;

   
   logic [69:0]  pc7_in, pc6_in, pc5_in, pc4_in, pc3_in, pc2_in, pc1_in, pc0_in;
   logic [69:0]  pc7, pc6, pc5, pc4, pc3, pc2, pc1, pc0;

   logic [15:0]  cinst7_in, cinst6_in, cinst5_in, cinst4_in, cinst3_in, cinst2_in, cinst1_in, cinst0_in;
   logic [15:0]  cinst7, cinst6, cinst5, cinst4, cinst3, cinst2, cinst1, cinst0;

   

   logic [$bits(br_pkt_t)-1:0] bp7_in, bp6_in, bp5_in, bp4_in, bp3_in, bp2_in, bp1_in, bp0_in;
   logic [$bits(br_pkt_t)-1:0] bp7, bp6, bp5, bp4, bp3, bp2, bp1, bp0;

   

   // 输出的 结果
   // logic [31:0] dec_i0_instr_fifo, dec_i1_instr_fifo;


   logic         write_i1_ib7, write_i0_ib7;
   logic         write_i1_ib6, write_i0_ib6;
   logic         write_i1_ib5, write_i0_ib5;
   logic         write_i1_ib4, write_i0_ib4;
   logic         write_i1_ib3, write_i0_ib3;
   logic         write_i1_ib2, write_i0_ib2;
   logic         write_i1_ib1, write_i0_ib1;
   logic         write_i1_ib0, write_i0_ib0;

   logic         shift2, shift1, shift0;

   // 消耗 1 条指令, 移动 1 位的
   logic         shift_ib1_ib0, shift_ib2_ib1, shift_ib3_ib2, shift_ib4_ib3, shift_ib5_ib4, shift_ib6_ib5, shift_ib7_ib6;
   // 消耗 2 条指令, 移动 2 位的
   logic         shift_ib2_ib0, shift_ib3_ib1, shift_ib4_ib2, shift_ib5_ib3, shift_ib6_ib4, shift_ib7_ib5;


   logic         dec_i0_val, dec_i1_val;  // i0, i1 指令 是否有效

   logic        dec_i0_valid, dec_i1_valid;  // 当前写入的指令 要是 有效的




   logic [7:0]   ibvalid;

   logic [7:0]   i0_wen;  // 7 个位置的写使能信号
   logic [7:0]   i1_wen;  // or  logic [7:0]   i1_wen;


   logic [7:0]   shift_ibval;
   logic [7:0]   ibwrite;



   logic fifo_empty, fifo_full;

   logic write_enable_in, read_enable_in;

   logic write_enable, read_enable;

   logic load_block_d_in, load_block_d;

   logic load_block_en;

   logic read_fifo_enable, read_fifo_enable_q;


   assign dec_i0_valid = dec_i0_decode_d ; 
   assign dec_i1_valid = dec_i1_decode_d ;

      // 队列里面只有 一条指令的 时候, 加一个信号 通知从 指令buffer 里面取一条指令, 发射给 i1

   //  一但值预测错误, ibval[7 : 0] 就会为全 0, 队列就 空了
   assign fifo_empty = ~(|ibval[7 : 0]);  //  fifo_empty = 1表示 fifo 是 空的  

   assign fifo_full = & ibval[7 : 0];  //  队列满, 不能进行 写入, 直到 队列 为 空, 再次激活 写 队列

   

   //  flush_fifo :有依赖, 但是值预测正确, 清空 队列   还需要考虑 e4 阶段的 load 是否是 有效的 load 指令
   // rvdff #(1) flush_fifoff (.*, .clk(free_clk), .din( (~exu_i0_flush_vp_e4 & load_match_i0_e4) | (  ~exu_i1_flush_vp_e4 & load_match_i1_e4) ), .dout(flush_fifo));
   assign flush_fifo = (load_match_i0_e4 & ~exu_i0_flush_vp_e4) | (load_match_i1_e4 & ~exu_i1_flush_vp_e4);

   // 值预测错误, 从队列读
   assign read_fifo_enable = ((exu_i0_flush_vp_e4 | exu_i1_flush_vp_e4) | read_fifo_enable_q) & ~fifo_empty;
   rvdff #(1) flush_fifoff (.*, .clk(free_clk), .din(read_fifo_enable) , .dout(read_fifo_enable_q)); 


   assign ibvalid[7:0] = ibval[7:0] | i0_wen[7:0] | i1_wen[7:0]; 

   assign ibval_in[7:0] = (({8{shift0}} & ibvalid[7:0]) |
                           ({8{shift1}} & {1'b0, ibvalid[7:1]}) | 
                           ({8{shift2}} & {2'b0, ibvalid[7:2]})) & ~{8{flush_fifo}}; // flush_fifo = 1, 清空队列

   rvdff #(8) ibvalff (.*, .clk(active_clk), .din(ibval_in[7:0]), .dout(ibval[7:0]));




   logic load_block_q;
   assign load_block_d_in = ((i0_load_block_d  | i1_load_block_d) | load_block_q) & ~flush_fifo;

   // 从依赖开始, 到值预测结束, 所有指令 都进行入队
   rvdffe #(1) loadblockff (.*, .en(1'b1), .din(load_block_d_in ), .dout(load_block_q));




   
   // assign write_enable_in = fifo_empty & (load_block_d | i0_load_block_d  | i1_load_block_d) & ~flush_final & ~flush_fifo;  // 写的话, 队列为 空, 并且 有依赖

   // assign read_enable_in = flush_final & ~fifo_empty; // 值预测错误, 就开始读取, 读取到队列为空
   assign write_enable = load_block_d_in;
   assign read_enable = read_fifo_enable;


   /*
   写 的时候, 一定不会 读
   读 的时候, 一定不会 写
   write_enable: 每次开始写,  队列 一定要为 空,

   值预测正确, 清空队列,

   值预测错误, 从队列里面开始读取指令


   */







   // 原本设计是   ib0  只能由  i0 指令写入

   // 定义了 FIFO 的大小为 8 了   dec_i0_decode_d  当前的解码指令是有效的    如果刷新了, 当前的 写入指令 也要无效
   // only valid if there is room
   if (DEC_INSTFIFO_DEPTH==8) begin  // ifu_i0_valid: 写入的值 是有效的
      assign dec_i0_val = dec_i0_valid & ~ibval[7] & write_enable;  // 写入 i0 至少要保证有 1 个空位
      // 只有 一个 空位 时, 如果 i0 无效, i1 有效, 则 i1, 也能写入
      assign dec_i1_val = dec_i1_valid & ~ibval[6] & write_enable;  // 写入 i1 至少要保证有 2 个空位,  所以检查  ~ibval[2]  

      
   end
   else begin   // 这个根本就不会执行

      assign dec_i0_val = dec_i0_valid & (~dec_ib0_valid_eff_d | ~dec_ib1_valid_eff_d) & write_enable; // 写使能有效, 并且 没有队列刷新的信号
      assign dec_i1_val = dec_i1_valid & (~dec_ib0_valid_eff_d & ~dec_ib1_valid_eff_d) & write_enable;

      
   end
   
   // 写入,  一次性写入 两条指令   ibval[0] = 1: 表示 0 号位置 有指令, 被占据了,  dec_i0_val = 1: 表所当前的 i0 指令是有效的
   // assign i0_wen[0] = 1'b0 ;

   assign i0_wen[0] = ~ibval[0]             & dec_i0_val ;

   assign i0_wen[1] =  ibval[0] & ~ibval[1] & dec_i0_val;  // 0 号占据了,  1 号为空
   assign i0_wen[2] =  ibval[1] & ~ibval[2] & dec_i0_val;
   assign i0_wen[3] =  ibval[2] & ~ibval[3] & dec_i0_val;
   assign i0_wen[4] =  ibval[3] & ~ibval[4] & dec_i0_val;
   assign i0_wen[5] =  ibval[4] & ~ibval[5] & dec_i0_val;
   assign i0_wen[6] =  ibval[5] & ~ibval[6] & dec_i0_val;
   assign i0_wen[7] =  ibval[6] & ~ibval[7] & dec_i0_val;  // 第 6 个曹不为空,  第 7 个曹 为空
   

   // 若 dec_i0_decode_d = 0, 则 dec_i1_decode_d 一定为 0

   
   // i0 指令无效的情况下, i1 有效, 将 i1 写入 0 号位   第一条 依赖的是 i1 指令, 则 i1 进入 0 号 曹
   assign i1_wen[0] = ~ibval[0] & ~dec_i0_val & dec_i1_val;  
  
   assign i1_wen[1] = ~ibval[0]  &  dec_i0_val & dec_i1_val;

   assign i1_wen[2] =  ibval[0] & ~ibval[1] & dec_i1_val;
   assign i1_wen[3] =  ibval[1] & ~ibval[2] & dec_i1_val;
   assign i1_wen[4] =  ibval[2] & ~ibval[3] & dec_i1_val;  // 0 - 2 曹不为空,  第 3 个曹 开始为空, i1 写入第 4 个曹
   assign i1_wen[5] =  ibval[3] & ~ibval[4] & dec_i1_val;
   assign i1_wen[6] =  ibval[4] & ~ibval[5] & dec_i1_val;
   assign i1_wen[7] =  ibval[5] & ~ibval[6] & dec_i1_val;  // 0 - 5 曹不为空,  第 6 个曹 开始为空, i1 写入第 7 个曹




   /*
   // 修改的


   assign dec_ib1_valid_fifo = ibval[1];   // 用来表示 当前读出的指令 是否有效, 即读取到的是否是空值
   assign dec_ib0_valid_fifo = ibval[0];

   */
   


   assign shift1 = dec_i0_decode_d & ~dec_i1_decode_d & read_enable;  // 从队列读取的, 消耗了 i0 一条指令

   assign shift2 = dec_i0_decode_d & dec_i1_decode_d  & read_enable; 

   //  写的时候, shift0 一定为 1,  读取的时候, shift0 也可能为 1
   assign shift0 = write_enable | (~dec_i0_decode_d & read_enable);  // 从队列读取的, 没有 消耗指令  如果是 奇数 条指令, 一次消耗 2 条指令, 最后一次, 只有 1 条指令


   
   
   // 修改的    写使能
   // compute shifted ib valids to determine where to write   进行 FIFO 写指令的时候,  不会消耗 指令
   assign shift_ibval[7:0] = ({8{shift0}} & ibval[7:0]) |
                             ({8{shift1}} & {1'b0, ibval[7:1] }) | 
                             ({8{shift2}} & {2'b0, ibval[7:2] });

   

   assign write_i0_ib0 = ~shift_ibval[0]                & dec_i0_val ;

   assign write_i0_ib1 =  shift_ibval[0] & ~shift_ibval[1] & dec_i0_val;
   assign write_i0_ib2 =  shift_ibval[1] & ~shift_ibval[2] & dec_i0_val;
   assign write_i0_ib3 =  shift_ibval[2] & ~shift_ibval[3] & dec_i0_val;

   assign write_i0_ib4 =  shift_ibval[3] & ~shift_ibval[4] & dec_i0_val; // 槽0~3占用、槽4空，则写入 i0
   assign write_i0_ib5 =  shift_ibval[4] & ~shift_ibval[5] & dec_i0_val;
   assign write_i0_ib6 =  shift_ibval[5] & ~shift_ibval[6] & dec_i0_val;
   assign write_i0_ib7 =  shift_ibval[6] & ~shift_ibval[7] & dec_i0_val; // 槽0~6占用、槽7空，则 槽7 写入 i0
   
   
   assign write_i1_ib0 = ~shift_ibval[0] & ~dec_i0_val    & dec_i1_val ;  // 队列中,  i1 指令也可以写入  第 0 个    ？？？？

   assign write_i1_ib1 = ~shift_ibval[0] &  dec_i0_val    & dec_i1_val;  // 槽 0 空, i0 指令有效, 则 槽1 写入 i1

   assign write_i1_ib2 =  shift_ibval[0] & ~shift_ibval[1] & dec_i1_val;
   assign write_i1_ib3 =  shift_ibval[1] & ~shift_ibval[2] & dec_i1_val;  // 槽0~1占用、槽2空，则 槽3 写入 i1
   assign write_i1_ib4 =  shift_ibval[2] & ~shift_ibval[3] & dec_i1_val;
   assign write_i1_ib5 =  shift_ibval[3] & ~shift_ibval[4] & dec_i1_val;
   assign write_i1_ib6 =  shift_ibval[4] & ~shift_ibval[5] & dec_i1_val;
   assign write_i1_ib7 =  shift_ibval[5] & ~shift_ibval[6] & dec_i1_val; // 槽0~5占用、槽6空，则 槽7 写入 i1



   // 修改的     移位使能shift1
   // 消耗 1 条指令的 移位
   assign shift_ib1_ib0 = shift1 & ibval[1];  // 消耗 1 条指令, 并且 1 曹有指令, 将 曹 1 的指令移入到 曹 0
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


      // pc tracking

   // 修改的  写的时候, write_i0_ib7 等信号 有效, 读的时候, shift_ib7_ib6 等信号 有效
   assign ibwrite[7:0] = {  write_i0_ib7 | write_i1_ib7,   
                            write_i0_ib6 | write_i1_ib6 | shift_ib7_ib6, 

                            write_i0_ib5 | write_i1_ib5 | shift_ib6_ib5 | shift_ib7_ib5, 
                            write_i0_ib4 | write_i1_ib4 | shift_ib5_ib4 | shift_ib6_ib4,  

                            write_i0_ib3 | write_i1_ib3 | shift_ib4_ib3 | shift_ib5_ib3, 
                            write_i0_ib2 | write_i1_ib2 | shift_ib3_ib2 | shift_ib4_ib2,  

                            write_i0_ib1 | write_i1_ib1 | shift_ib2_ib1 | shift_ib3_ib1,  

                            write_i0_ib0 | write_i1_ib0 | shift_ib1_ib0 | shift_ib2_ib0  //      i0, i1 指令 都可以写入  0 槽

                            }; // 右边的每个信号中,  都 由  dec_i0_val, dec_i1_val 进行决定   
                    

      // 从 buffer 出来的 依赖指令 进行入队 

   assign ib7_in[31:0] = ({32{write_i0_ib7}} & dec_i0_instr_buffer[31:0]) |  
                         ({32{write_i1_ib7}} & dec_i1_instr_buffer[31:0]);
      
   rvdffe #(32) ib7ff (.*, .en(ibwrite[7]), .din(ib7_in[31:0]), .dout(ib7[31:0]));
      

   assign ib6_in[31:0] = ({32{write_i0_ib6}} & dec_i0_instr_buffer[31:0]) |
                         ({32{write_i1_ib6}} & dec_i1_instr_buffer[31:0]) |

                         ({32{shift_ib7_ib6}} & ib7[31:0]);

   rvdffe #(32) ib6ff (.*, .en(ibwrite[6]), .din(ib6_in[31:0]), .dout(ib6[31:0]));


   assign ib5_in[31:0] = ({32{write_i0_ib5}} & dec_i0_instr_buffer[31:0]) |
                         ({32{write_i1_ib5}} & dec_i1_instr_buffer[31:0]) |

                         ({32{shift_ib6_ib5}} & ib6[31:0]) |
                         ({32{shift_ib7_ib5}} & ib7[31:0]);
      
   rvdffe #(32) ib5ff (.*, .en(ibwrite[5]), .din(ib5_in[31:0]), .dout(ib5[31:0]));
      

   assign ib4_in[31:0] = ({32{write_i0_ib4}} & dec_i0_instr_buffer[31:0]) |
                         ({32{write_i1_ib4}} & dec_i1_instr_buffer[31:0]) |

                         ({32{shift_ib5_ib4}} & ib5[31:0]) |
                         ({32{shift_ib6_ib4}} & ib6[31:0]);
      
   rvdffe #(32) ib4ff (.*, .en(ibwrite[4]), .din(ib4_in[31:0]), .dout(ib4[31:0]));


   assign ib3_in[31:0] = ({32{write_i0_ib3}} & dec_i0_instr_buffer[31:0]) |
                         ({32{write_i1_ib3}} & dec_i1_instr_buffer[31:0]) |

                         ({32{shift_ib4_ib3}} & ib4[31:0]) |
                         ({32{shift_ib5_ib3}} & ib5[31:0]);
      
   rvdffe #(32) ib3ff (.*, .en(ibwrite[3]), .din(ib3_in[31:0]), .dout(ib3[31:0]));
      

   assign ib2_in[31:0] = ({32{write_i0_ib2}} & dec_i0_instr_buffer[31:0]) |
                         ({32{write_i1_ib2}} & dec_i1_instr_buffer[31:0]) |

                         ({32{shift_ib3_ib2}} & ib3[31:0]) | 
                         ({32{shift_ib4_ib2}} & ib4[31:0]);
      
   rvdffe #(32) ib2ff (.*, .en(ibwrite[2]), .din(ib2_in[31:0]), .dout(ib2[31:0]));


   assign ib1_in[31:0] = ({32{write_i0_ib1}} & dec_i0_instr_buffer[31:0]) |
                         ({32{write_i1_ib1}} & dec_i1_instr_buffer[31:0]) |

                         ({32{shift_ib2_ib1}} & ib2[31:0]) |
                         ({32{shift_ib3_ib1}} & ib3[31:0]);
   
   rvdffe #(32) ib1ff (.*, .en(ibwrite[1]), .din(ib1_in[31:0]), .dout(ib1[31:0]));


   assign ib0_in[31:0] = ({32{write_i0_ib0}} & ( dec_i0_instr_buffer[31:0])) |  // i0 指令写入 0 槽
                         ({32{write_i1_ib0}} & ( dec_i1_instr_buffer[31:0])) |  // i1 指令写入 0 槽

                         ({32{shift_ib1_ib0}} & ib1[31:0]) |
                         ({32{shift_ib2_ib0}} & ib2[31:0]);
   
   rvdffe #(32) ib0ff (.*, .en(ibwrite[0]), .din(ib0_in[31:0]), .dout(ib0[31:0]));


   // 从 fifo 里面读出的指令


   // 其它控制信号会保证 从队列读取的时候, 才使用 dec_i0_instr_fifo[31:0] 的值

   assign dec_i0_instr_fifo[31:0] = ib0[31:0] ;  // 保证当前读取的 是 有指令
   assign dec_i1_instr_fifo[31:0] = ib1[31:0] ; 




   

   assign pc7_in[69:0] = ({70{write_i0_ib7}} & pc0_buffer[69:0]) |    // pc0_buffer  pc1_buffer
                         ({70{write_i1_ib7}} & pc1_buffer[69:0]); 
      
   rvdffe #(70) pc7ff (.*, .en(ibwrite[7]), .din(pc7_in[69:0]), .dout(pc7[69:0]));
      

   assign pc6_in[69:0] = ({70{write_i0_ib7}} & pc0_buffer[69:0]) |
                         ({70{write_i1_ib7}} & pc1_buffer[69:0]) |

                         ({70{shift_ib7_ib6}} & pc7[69:0]);

   rvdffe #(70) pc6ff (.*, .en(ibwrite[6]), .din(pc6_in[69:0]), .dout(pc6[69:0]));


   assign pc5_in[69:0] = ({70{write_i0_ib5}} & pc0_buffer[69:0]) |
                         ({70{write_i1_ib5}} & pc1_buffer[69:0]) |

                         ({70{shift_ib6_ib5}} & pc6[69:0]) |
                         ({70{shift_ib7_ib5}} & pc7[69:0]);
      
   rvdffe #(70) pc5ff (.*, .en(ibwrite[5]), .din(pc5_in[69:0]), .dout(pc5[69:0]));
      

   assign pc4_in[69:0] = ({70{write_i0_ib4}} & pc0_buffer[69:0]) |
                         ({70{write_i1_ib4}} & pc1_buffer[69:0]) |

                         ({70{shift_ib5_ib4}} & pc5[69:0]) |
                         ({70{shift_ib6_ib4}} & pc6[69:0]);
      
   rvdffe #(70) pc4ff (.*, .en(ibwrite[4]), .din(pc4_in[69:0]), .dout(pc4[69:0]));


   assign pc3_in[69:0] = ({70{write_i0_ib3}} & pc0_buffer[69:0]) |
                         ({70{write_i1_ib3}} & pc1_buffer[69:0]) |

                         ({70{shift_ib4_ib3}} & pc4[69:0]) |
                         ({70{shift_ib5_ib3}} & pc5[69:0]);
      
   rvdffe #(70) pc3ff (.*, .en(ibwrite[3]), .din(pc3_in[69:0]), .dout(pc3[69:0]));
      

   assign pc2_in[69:0] = ({70{write_i0_ib2}} & pc0_buffer[69:0]) |
                         ({70{write_i1_ib2}} & pc1_buffer[69:0]) |

                         ({70{shift_ib3_ib2}} & pc3[69:0]) | 
                         ({70{shift_ib4_ib2}} & pc4[69:0]);
      
   rvdffe #(70) pc2ff (.*, .en(ibwrite[2]), .din(pc2_in[69:0]), .dout(pc2[69:0]));


   assign pc1_in[69:0] = ({70{write_i0_ib1}} & pc0_buffer[69:0]) |
                         ({70{write_i1_ib1}} & pc1_buffer[69:0]) |

                         ({70{shift_ib2_ib1}} & pc2[69:0]) |
                         ({70{shift_ib3_ib1}} & pc3[69:0]);
   
   rvdffe #(70) pc1ff (.*, .en(ibwrite[1]), .din(pc1_in[69:0]), .dout(pc1[69:0]));


   assign pc0_in[69:0] = ({70{write_i0_ib0}} & ( pc0_buffer[69:0])) |  // i0 指令写入 0 槽
                         ({70{write_i1_ib0}} & ( pc1_buffer[69:0])) |  // i1 指令写入 0 槽

                         ({70{shift_ib1_ib0}} & pc1[69:0]) |
                         ({70{shift_ib2_ib0}} & pc2[69:0]);
   
   rvdffe #(70) pc0ff (.*, .en(ibwrite[0]), .din(pc0_in[69:0]), .dout(pc0[69:0]));


   assign pc0_fifo[69:0] = pc0[69:0];
   assign pc1_fifo[69:0] = pc1[69:0]; 


   assign cinst7_in[15:0] = ({16{write_i0_ib7}} & dec_i0_cinst_buffer[15:0]) |    // cinst3_in  dec_i1_cinst_buffer
                         ({16{write_i1_ib7}} & dec_i1_cinst_buffer[15:0]);   // 
      
   rvdffe #(16) cinst7ff (.*, .en(ibwrite[7]), .din(cinst7_in[15:0]), .dout(cinst7[15:0]));
      

   assign cinst6_in[15:0] = ({16{write_i0_ib7}} & dec_i0_cinst_buffer[15:0]) |
                         ({16{write_i1_ib7}} & dec_i1_cinst_buffer[15:0]) |

                         ({16{shift_ib7_ib6}} & cinst7[15:0]);

   rvdffe #(16) cinst6ff (.*, .en(ibwrite[6]), .din(cinst6_in[15:0]), .dout(cinst6[15:0]));  // 


   assign cinst5_in[15:0] = ({16{write_i0_ib5}} & dec_i0_cinst_buffer[15:0]) |
                         ({16{write_i1_ib5}} & dec_i1_cinst_buffer[15:0]) |

                         ({16{shift_ib6_ib5}} & cinst6[15:0]) |
                         ({16{shift_ib7_ib5}} & cinst7[15:0]);
      
   rvdffe #(16) cinst5ff (.*, .en(ibwrite[5]), .din(cinst5_in[15:0]), .dout(cinst5[15:0]));  // 
      

   assign cinst4_in[15:0] = ({16{write_i0_ib4}} & dec_i0_cinst_buffer[15:0]) |
                         ({16{write_i1_ib4}} & dec_i1_cinst_buffer[15:0]) |

                         ({16{shift_ib5_ib4}} & cinst5[15:0]) |
                         ({16{shift_ib6_ib4}} & cinst6[15:0]);
      
   rvdffe #(16) cinst4ff (.*, .en(ibwrite[4]), .din(cinst4_in[15:0]), .dout(cinst4[15:0]));  // 


   assign cinst3_in[15:0] = ({16{write_i0_ib3}} & dec_i0_cinst_buffer[15:0]) |   
                         ({16{write_i1_ib3}} & dec_i1_cinst_buffer[15:0]) |

                         ({16{shift_ib4_ib3}} & cinst4[15:0]) |
                         ({16{shift_ib5_ib3}} & cinst5[15:0]);
      
   rvdffe #(16) cinst3ff (.*, .en(ibwrite[3]), .din(cinst3_in[15:0]), .dout(cinst3[15:0]));  // 
      

   assign cinst2_in[15:0] = ({16{write_i0_ib2}} & dec_i0_cinst_buffer[15:0]) |
                         ({16{write_i1_ib2}} & dec_i1_cinst_buffer[15:0]) |

                         ({16{shift_ib3_ib2}} & cinst3[15:0]) | 
                         ({16{shift_ib4_ib2}} & cinst4[15:0]);
      
   rvdffe #(16) cinst2ff (.*, .en(ibwrite[2]), .din(cinst2_in[15:0]), .dout(cinst2[15:0]));  // 


   assign cinst1_in[15:0] = ({16{write_i0_ib1}} & dec_i0_cinst_buffer[15:0]) |
                         ({16{write_i1_ib1}} & dec_i1_cinst_buffer[15:0]) |

                         ({16{shift_ib2_ib1}} & cinst2[15:0]) |
                         ({16{shift_ib3_ib1}} & cinst3[15:0]);
   
   rvdffe #(16) cinst1ff (.*, .en(ibwrite[1]), .din(cinst1_in[15:0]), .dout(cinst1[15:0]));  // 


   assign cinst0_in[15:0] = ({16{write_i0_ib0}} & ( dec_i0_cinst_buffer[15:0])) |  // i0 指令写入 0 槽
                         ({16{write_i1_ib0}} & ( dec_i1_cinst_buffer[15:0])) |  // i1 指令写入 0 槽

                         ({16{shift_ib1_ib0}} & cinst1[15:0]) |
                         ({16{shift_ib2_ib0}} & cinst2[15:0]);
   
   rvdffe #(16) cinst0ff (.*, .en(ibwrite[0]), .din(cinst0_in[15:0]), .dout(cinst0[15:0]));  // 

   assign dec_i0_cinst_fifo[15:0] = cinst0[15:0];
   assign dec_i1_cinst_fifo[15:0] = cinst1[15:0]; 




   assign bp7_in[$bits(br_pkt_t)-1:0] = ({$bits(br_pkt_t){write_i0_ib7}} & dec_i0_brp_buffer[$bits(br_pkt_t)-1:0]) |    // bp3_in  dec_i1_brp_buffer
                                       ({$bits(br_pkt_t){write_i1_ib7}} & dec_i1_brp_buffer[$bits(br_pkt_t)-1:0]);   //  
      
   rvdffe #($bits(br_pkt_t)) bp7ff (.*, .en(ibwrite[7]), .din(bp7_in[$bits(br_pkt_t)-1:0]), .dout(bp7[$bits(br_pkt_t)-1:0]));
      
   // 
   assign bp6_in[$bits(br_pkt_t)-1:0] = ({$bits(br_pkt_t){write_i0_ib7}} & dec_i0_brp_buffer[$bits(br_pkt_t)-1:0]) |
                                       ({$bits(br_pkt_t){write_i1_ib7}} & dec_i1_brp_buffer[$bits(br_pkt_t)-1:0]) |

                                       ({$bits(br_pkt_t){shift_ib7_ib6}} & bp7[$bits(br_pkt_t)-1:0]);  // 

   rvdffe #($bits(br_pkt_t)) bp6ff (.*, .en(ibwrite[6]), .din(bp6_in[$bits(br_pkt_t)-1:0]), .dout(bp6[$bits(br_pkt_t)-1:0]));  // 

   // 
   assign bp5_in[$bits(br_pkt_t)-1:0] = ({$bits(br_pkt_t){write_i0_ib5}} & dec_i0_brp_buffer[$bits(br_pkt_t)-1:0]) |
                                       ({$bits(br_pkt_t){write_i1_ib5}} & dec_i1_brp_buffer[$bits(br_pkt_t)-1:0]) |

                                       ({$bits(br_pkt_t){shift_ib6_ib5}} & bp6[$bits(br_pkt_t)-1:0]) |
                                       ({$bits(br_pkt_t){shift_ib7_ib5}} & bp7[$bits(br_pkt_t)-1:0]);  
      
   rvdffe #($bits(br_pkt_t)) bp5ff (.*, .en(ibwrite[5]), .din(bp5_in[$bits(br_pkt_t)-1:0]), .dout(bp5[$bits(br_pkt_t)-1:0]));  // 
      
   // 
   assign bp4_in[$bits(br_pkt_t)-1:0] = ({$bits(br_pkt_t){write_i0_ib4}} & dec_i0_brp_buffer[$bits(br_pkt_t)-1:0]) |
                                       ({$bits(br_pkt_t){write_i1_ib4}} & dec_i1_brp_buffer[$bits(br_pkt_t)-1:0]) |

                                       ({$bits(br_pkt_t){shift_ib5_ib4}} & bp5[$bits(br_pkt_t)-1:0]) |
                                       ({$bits(br_pkt_t){shift_ib6_ib4}} & bp6[$bits(br_pkt_t)-1:0]);  
                     
   rvdffe #($bits(br_pkt_t)) bp4ff (.*, .en(ibwrite[4]), .din(bp4_in[$bits(br_pkt_t)-1:0]), .dout(bp4[$bits(br_pkt_t)-1:0]));  // 

   // 
   assign bp3_in[$bits(br_pkt_t)-1:0] = ({$bits(br_pkt_t){write_i0_ib3}} & dec_i0_brp_buffer[$bits(br_pkt_t)-1:0]) |   
                                       ({$bits(br_pkt_t){write_i1_ib3}} & dec_i1_brp_buffer[$bits(br_pkt_t)-1:0]) |

                                       ({$bits(br_pkt_t){shift_ib4_ib3}} & bp4[$bits(br_pkt_t)-1:0]) |
                                       ({$bits(br_pkt_t){shift_ib5_ib3}} & bp5[$bits(br_pkt_t)-1:0]);  
      
   rvdffe #($bits(br_pkt_t)) bp3ff (.*, .en(ibwrite[3]), .din(bp3_in[$bits(br_pkt_t)-1:0]), .dout(bp3[$bits(br_pkt_t)-1:0]));  // 
      
   // 
   assign bp2_in[$bits(br_pkt_t)-1:0] = ({$bits(br_pkt_t){write_i0_ib2}} & dec_i0_brp_buffer[$bits(br_pkt_t)-1:0]) |
                                       ({$bits(br_pkt_t){write_i1_ib2}} & dec_i1_brp_buffer[$bits(br_pkt_t)-1:0]) |

                                       ({$bits(br_pkt_t){shift_ib3_ib2}} & bp3[$bits(br_pkt_t)-1:0]) | 
                                       ({$bits(br_pkt_t){shift_ib4_ib2}} & bp4[$bits(br_pkt_t)-1:0]);   
      
   rvdffe #($bits(br_pkt_t)) bp2ff (.*, .en(ibwrite[2]), .din(bp2_in[$bits(br_pkt_t)-1:0]), .dout(bp2[$bits(br_pkt_t)-1:0]));  // 

   // 
   assign bp1_in[$bits(br_pkt_t)-1:0] = ({$bits(br_pkt_t){write_i0_ib1}} & dec_i0_brp_buffer[$bits(br_pkt_t)-1:0]) |
                                       ({$bits(br_pkt_t){write_i1_ib1}} & dec_i1_brp_buffer[$bits(br_pkt_t)-1:0]) |

                                       ({$bits(br_pkt_t){shift_ib2_ib1}} & bp2[$bits(br_pkt_t)-1:0]) |
                                       ({$bits(br_pkt_t){shift_ib3_ib1}} & bp3[$bits(br_pkt_t)-1:0]);   
                  
   rvdffe #($bits(br_pkt_t)) bp1ff (.*, .en(ibwrite[1]), .din(bp1_in[$bits(br_pkt_t)-1:0]), .dout(bp1[$bits(br_pkt_t)-1:0]));  // 

   // 
   assign bp0_in[$bits(br_pkt_t)-1:0] = ({$bits(br_pkt_t){write_i0_ib0}} & ( dec_i0_brp_buffer[$bits(br_pkt_t)-1:0])) |  // i0 指令写入 0 槽
                                       ({$bits(br_pkt_t){write_i1_ib0}} & ( dec_i1_brp_buffer[$bits(br_pkt_t)-1:0])) |  // i1 指令写入 0 槽

                                       ({$bits(br_pkt_t){shift_ib1_ib0}} & bp1[$bits(br_pkt_t)-1:0]) |
                                       ({$bits(br_pkt_t){shift_ib2_ib0}} & bp2[$bits(br_pkt_t)-1:0]);   
   
   rvdffe #($bits(br_pkt_t)) bp0ff (.*, .en(ibwrite[0]), .din(bp0_in[$bits(br_pkt_t)-1:0]), .dout(bp0[$bits(br_pkt_t)-1:0]));  // 


   // 从 fifo 里面读出的指令
   assign dec_i0_brp_fifo[$bits(br_pkt_t)-1:0] = bp0[$bits(br_pkt_t)-1:0];
   assign dec_i1_brp_fifo[$bits(br_pkt_t)-1:0] = bp1[$bits(br_pkt_t)-1:0]; 




   


   
endmodule


