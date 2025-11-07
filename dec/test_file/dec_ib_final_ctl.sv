
//  对 buffer 和  fifo  的输入进行选择,  最终输出到   dec_decode_d 模块中

module dec_ib_final_ctl
   import veer_types::*;
(



   input logic i0_rd_enable, i1_rd_enable, 

   input logic i0_rd_enable_next, i1_rd_enable_next,


      // 还需要传入一些  写入的使能信号
   

   //  从 buffer 里面的 输入
   input logic dec_ib3_valid_d_buffer,               // ib3 valid
   input logic dec_ib2_valid_d_buffer,               // ib2 valid
   input logic dec_ib1_valid_d_buffer,               // ib1 valid
   input logic dec_ib0_valid_d_buffer,               // ib0 valid    to  ifu_aln_ctl.sv


   input logic [31:0] dec_i0_instr_d_buffer,         // i0 inst at decode
   input logic [31:0] dec_i1_instr_d_buffer,         // i1 inst at decode   to  dec_decode_ctl.sv


   input br_pkt_t dec_i0_brp_buffer,                 // i0 branch packet at decode
   input br_pkt_t dec_i1_brp_buffer,

   input logic [36:0] pc0_buffer,            // i0 pc at decode     to  dec_decode_ctl.sv  dec_trigger.sv   exu.sv
   input logic [36:0] pc1_buffer,





   input logic dec_debug_wdata_rs1_d_buffer,         // put debug write data onto rs1 source: machine is halted

   input logic dec_debug_fence_d_buffer,             // debug fence inst


   input logic [15:0] dec_i0_cinst_d_buffer,         // 16b compress inst at decode
   input logic [15:0] dec_i1_cinst_d_buffer,





   input logic dec_ib3_valid_d_fifo,               // ib3 valid
   input logic dec_ib2_valid_d_fifo,               // ib2 valid
   input logic dec_ib1_valid_d_fifo,               // ib1 valid
   input logic dec_ib0_valid_d_fifo,               // ib0 valid    to  ifu_aln_ctl.sv


   //    从 fifo 里面的 输入
   input logic [31:0] dec_i0_instr_d_fifo,         // i0 inst at decode
   input logic [31:0] dec_i1_instr_d_fifo,         // i1 inst at decode   to  dec_decode_ctl.sv

   input br_pkt_t dec_i0_brp_fifo,                 // i0 branch packet at decode
   input br_pkt_t dec_i1_brp_fifo,


   //  这 7 组 用 1 个 整体信号输出就行  pc0[36:0],  pc1[36:0]
   input logic [36:0] pc0_fifo,            // i0 pc at decode     to  dec_decode_ctl.sv  dec_trigger.sv   exu.sv
   input logic [36:0] pc1_fifo,


   //  这 7 组 用 1 个 整体信号输出就行


   input logic dec_debug_wdata_rs1_d_fifo,         // put debug write data onto rs1 source: machine is halted

   input logic dec_debug_fence_d_fifo,             // debug fence inst



   input logic [15:0] dec_i0_cinst_d_fifo,         // 16b compress inst at decode
   input logic [15:0] dec_i1_cinst_d_fifo,





   //  最终的 输出结果
   output logic dec_ib3_valid_d,               // ib3 valid
   output logic dec_ib2_valid_d,               // ib2 valid
   output logic dec_ib1_valid_d,               // ib1 valid
   output logic dec_ib0_valid_d,               // ib0 valid    to  ifu_aln_ctl.sv


   output logic [31:0] dec_i0_instr_d,         // i0 inst at decode
   output logic [31:0] dec_i1_instr_d,         // i1 inst at decode   to  dec_decode_ctl.sv

   output br_pkt_t dec_i0_brp,                 // i0 branch packet at decode
   output br_pkt_t dec_i1_brp,


   output logic [36:0] pc0,            // i0 pc at decode     to  dec_decode_ctl.sv  dec_trigger.sv   exu.sv
   output logic [36:0] pc1,



   output logic dec_debug_wdata_rs1_d,         // put debug write data onto rs1 source: machine is halted

   output logic dec_debug_fence_d,             // debug fence inst



   output logic [15:0] dec_i0_cinst_d,         // 16b compress inst at decode
   output logic [15:0] dec_i1_cinst_d,

   output logic [31:1] dec_i0_pc_d,            // i0 pc at decode     to  dec_decode_ctl.sv  dec_trigger.sv   exu.sv
   output logic [31:1] dec_i1_pc_d,

   output logic dec_i0_pc4_d,                  // i0 is 4B inst else 2B
   output logic dec_i1_pc4_d,

   
   output logic dec_i0_icaf_d,                 // i0 instruction access fault at decode
   output logic dec_i1_icaf_d,

   output logic dec_i0_icaf_second_d,              // i0 instruction access fault on second 2B of 4B inst

   output logic dec_i0_perr_d,                 // i0 instruction parity error at decode
   output logic dec_i1_perr_d,

   output logic dec_i0_sbecc_d,                // i0 single-bit error at decode
   output logic dec_i1_sbecc_d,

   output logic dec_i0_dbecc_d,                // i0 double-bit error at decode
   output logic dec_i1_dbecc_d

   

   );


   logic [31 : 1] dec_i0_pc_d_buffer, dec_i1_pc_d_buffer;

   logic [31 : 1] dec_i0_pc_d_fifo, dec_i1_pc_d_fifo;

   assign dec_i0_pc_d_buffer[31 : 1] = pc0_buffer[31 : 1];
   assign dec_i1_pc_d_buffer[31 : 1] = pc1_buffer[31 : 1];

   assign dec_i0_pc_d_fifo[31 : 1] = pc0_fifo[31 : 1];
   assign dec_i1_pc_d_fifo[31 : 1] = pc1_fifo[31 : 1];


   
   
   always_comb begin
      // if(i0_rd_enable_next | i1_rd_enable_next) begin
      if(i0_rd_enable_next ) begin


         dec_ib3_valid_d = dec_ib3_valid_d_fifo;
         dec_ib2_valid_d = dec_ib2_valid_d_fifo;
         dec_ib1_valid_d = dec_ib1_valid_d_fifo;
         dec_ib0_valid_d = dec_ib0_valid_d_fifo;

         dec_i0_brp = dec_i0_brp_fifo;
         dec_i1_brp = dec_i1_brp_fifo;

         dec_i0_instr_d[31 : 0] = dec_i0_instr_d_fifo[31 : 0];
         dec_i1_instr_d[31 : 0] = dec_i1_instr_d_fifo[31 : 0];

         dec_debug_wdata_rs1_d = dec_debug_wdata_rs1_d_fifo; 
         dec_debug_fence_d = dec_debug_fence_d_fifo; 
         
         dec_i0_cinst_d[15 : 0] = dec_i0_cinst_d_fifo[15 : 0];
         dec_i1_cinst_d[15 : 0] = dec_i1_cinst_d_fifo[15 : 0];

         dec_i0_icaf_second_d = pc0_fifo[36]; 

         dec_i1_dbecc_d = pc1_fifo[35];
         dec_i0_dbecc_d = pc0_fifo[35];

         dec_i1_sbecc_d = pc1_fifo[34];
         dec_i0_sbecc_d = pc0_fifo[34];

         dec_i1_perr_d = pc1_fifo[33];
         dec_i0_perr_d = pc0_fifo[33];
      
         dec_i1_icaf_d = pc1_fifo[32];
         dec_i0_icaf_d = pc0_fifo[32];
      
         dec_i1_pc_d[31:1] = pc1_fifo[31:1];
         dec_i0_pc_d[31:1] = pc0_fifo[31:1];

         dec_i1_pc4_d = pc1_fifo[0];
         dec_i0_pc4_d = pc0_fifo[0];
      
         pc1[36 : 0] = pc1_fifo[36 : 0];
         pc0[36 : 0] = pc0_fifo[36 : 0];
         


      end

      else begin



         dec_ib3_valid_d = dec_ib3_valid_d_buffer;
         dec_ib2_valid_d = dec_ib2_valid_d_buffer;
         dec_ib1_valid_d = dec_ib1_valid_d_buffer;
         dec_ib0_valid_d = dec_ib0_valid_d_buffer;

         dec_i0_brp = dec_i0_brp_buffer;
         dec_i1_brp = dec_i1_brp_buffer;

         dec_i0_instr_d[31 : 0] = dec_i0_instr_d_buffer[31 : 0];
         dec_i1_instr_d[31 : 0] = dec_i1_instr_d_buffer[31 : 0];

         dec_debug_wdata_rs1_d = dec_debug_wdata_rs1_d_buffer; 
         dec_debug_fence_d = dec_debug_fence_d_buffer; 
         
         dec_i0_cinst_d[15 : 0] = dec_i0_cinst_d_buffer[15 : 0];
         dec_i1_cinst_d[15 : 0] = dec_i1_cinst_d_buffer[15 : 0];

         dec_i0_icaf_second_d = pc0_buffer[36]; 

         dec_i1_dbecc_d = pc1_buffer[35];
         dec_i0_dbecc_d = pc0_buffer[35];

         dec_i1_sbecc_d = pc1_buffer[34];
         dec_i0_sbecc_d = pc0_buffer[34];

         dec_i1_perr_d = pc1_buffer[33];
         dec_i0_perr_d = pc0_buffer[33];
      
         dec_i1_icaf_d = pc1_buffer[32];
         dec_i0_icaf_d = pc0_buffer[32];
      
         dec_i1_pc_d[31:1] = pc1_buffer[31:1];
         dec_i0_pc_d[31:1] = pc0_buffer[31:1];

         dec_i1_pc4_d = pc1_buffer[0];
         dec_i0_pc4_d = pc0_buffer[0];
      
         pc0[36 : 0] = pc0_buffer[36 : 0];
         pc1[36 : 0] = pc1_buffer[36 : 0];



      end
      
   end

   





   /*
   
   assign dec_ib3_valid_d = dec_ib3_valid_d_buffer;
   assign dec_ib2_valid_d = dec_ib2_valid_d_buffer;
   assign dec_ib1_valid_d = dec_ib1_valid_d_buffer;
   assign dec_ib0_valid_d = dec_ib0_valid_d_buffer;

   // 队列 输出结果赋值     能够正确将 写入的 给重新读 出来了
   assign dec_i0_brp = dec_i0_brp_buffer;
   assign dec_i1_brp = dec_i1_brp_buffer;

   assign dec_i0_instr_d[31 : 0] = dec_i0_instr_d_buffer[31 : 0];
   assign dec_i1_instr_d[31 : 0] = dec_i1_instr_d_buffer[31 : 0];

   

   assign dec_debug_wdata_rs1_d = dec_debug_wdata_rs1_d_buffer;   

   assign dec_debug_fence_d = dec_debug_fence_d_buffer;   

   assign dec_i0_cinst_d[15 : 0] = dec_i0_cinst_d_buffer[15 : 0];
   assign dec_i1_cinst_d[15 : 0] = dec_i1_cinst_d_buffer[15 : 0];


   assign dec_i0_icaf_second_d = pc0_buffer[36];   // icaf's can only decode as i0

   assign dec_i1_dbecc_d = pc1_buffer[35];
   assign dec_i0_dbecc_d = pc0_buffer[35];

   assign dec_i1_sbecc_d = pc1_buffer[34];
   assign dec_i0_sbecc_d = pc0_buffer[34];

   assign dec_i1_perr_d = pc1_buffer[33];
   assign dec_i0_perr_d = pc0_buffer[33];

   assign dec_i1_icaf_d = pc1_buffer[32];
   assign dec_i0_icaf_d = pc0_buffer[32];

   assign dec_i1_pc_d[31:1] = pc1_buffer[31:1];
   assign dec_i0_pc_d[31:1] = pc0_buffer[31:1];

   assign dec_i1_pc4_d = pc1_buffer[0];
   assign dec_i0_pc4_d = pc0_buffer[0];



   assign pc0[36 : 0] = pc0_buffer[36 : 0];
   assign pc1[36 : 0] = pc1_buffer[36 : 0];

*/






endmodule