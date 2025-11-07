
//  对 buffer 和  fifo  的输入进行选择,  最终输出到   dec_decode_d 模块中

module dec_ib_final_ctl
   import swerv_types::*;
(



   // input logic i0_rd_enable, i1_rd_enable, 

   // input logic i0_rd_enable_next, i1_rd_enable_next,


      // 还需要传入一些  写入的使能信号
   

   //  从 buffer 里面的 输入
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
   input logic dec_ib3_valid_fifo,               // ib3 valid
   input logic dec_ib2_valid_fifo,               // ib2 valid
   input logic dec_ib1_valid_fifo,               // ib1 valid
   input logic dec_ib0_valid_fifo,               // ib0 valid
   

   input logic [31:0] dec_i0_instr_fifo,         // i0 inst at decode
   input logic [31:0] dec_i1_instr_fifo,         // i1 inst at decode



   input br_pkt_t dec_i0_brp_fifo,                 // i0 branch packet at decode
   input br_pkt_t dec_i1_brp_fifo,

   input logic [15:0] dec_i0_cinst_fifo,         // 16b compress inst at decode
   input logic [15:0] dec_i1_cinst_fifo,

   input logic dec_debug_wdata_rs1_fifo,         // put debug write data onto rs1 source: machine is halted

   input logic dec_debug_fence_fifo,             // debug fence inst

   input logic [69:0] pc0_fifo,
   input logic [69:0] pc1_fifo,





   //  buffer, fifo 最终选择的 输出结果
   
   output logic dec_ib3_valid_d,               // ib3 valid
   output logic dec_ib2_valid_d,               // ib2 valid
   output logic dec_ib1_valid_d,               // ib1 valid
   output logic dec_ib0_valid_d,               // ib0 valid
   

   output logic [31:0] dec_i0_instr_d,         // i0 inst at decode
   output logic [31:0] dec_i1_instr_d,         // i1 inst at decode

   output br_pkt_t dec_i0_brp,                 // i0 branch packet at decode
   output br_pkt_t dec_i1_brp,

   output logic [15:0] dec_i0_cinst_d,         // 16b compress inst at decode
   output logic [15:0] dec_i1_cinst_d,

   output logic dec_debug_wdata_rs1_d,         // put debug write data onto rs1 source: machine is halted

   output logic dec_debug_fence_d,             // debug fence inst

   // 以下信号由  dec_i0_pcdata_buffer[69:0] 或 dec_i0_pcdata_fifo[69:0] 进行赋值
   output logic dec_i0_fetch_page_fault,
   output logic dec_i1_fetch_page_fault,

   output logic dec_i0_icaf_d,                 // i0 instruction access fault at decode
   output logic dec_i1_icaf_d,

   output logic dec_i0_perr_d,                 // i0 instruction parity error at decode
   output logic dec_i1_perr_d,

   output logic dec_i0_sbecc_d,                // i0 single-bit error at decode
   output logic dec_i1_sbecc_d,

   output logic dec_i0_dbecc_d,                // i0 double-bit error at decode
   output logic dec_i1_dbecc_d,

   output logic dec_i0_pc4_d,                  // i0 is 4B inst else 2B
   output logic dec_i1_pc4_d,

   output logic [63:1] dec_i0_pc_d,            // i0 pc at decode
   output logic [63:1] dec_i1_pc_d,

   output logic dec_i0_icaf_f1_d              // i0 instruction access fault at decode for f1 fetch group


   );







   assign dec_ib3_valid_d = dec_ib3_valid_buffer;
   assign dec_ib2_valid_d = dec_ib2_valid_buffer;
   assign dec_ib1_valid_d = dec_ib1_valid_buffer;
   assign dec_ib0_valid_d = dec_ib0_valid_buffer;

   // 队列 输出结果赋值     能够正确将 写入的 给重新读 出来了
   assign dec_i0_brp = dec_i0_brp_buffer;
   assign dec_i1_brp = dec_i1_brp_buffer;

   assign dec_i0_instr_d[31 : 0] = dec_i0_instr_buffer[31 : 0];
   assign dec_i1_instr_d[31 : 0] = dec_i1_instr_buffer[31 : 0];

   assign dec_i0_cinst_d[15 : 0] = dec_i0_cinst_buffer[15 : 0];
   assign dec_i1_cinst_d[15 : 0] = dec_i1_cinst_buffer[15 : 0];

   assign dec_debug_wdata_rs1_d = dec_debug_wdata_rs1_buffer;   

   assign dec_debug_fence_d = dec_debug_fence_buffer;  


   // 由 pc0_buffer, pc1_buffer  赋值
   assign dec_i0_fetch_page_fault = pc0_buffer[69];
   assign dec_i1_fetch_page_fault = pc1_buffer[69];  // 2

   assign dec_i0_icaf_f1_d = pc0_buffer[68];   // icaf's can only decode as i0   // 2
   
   assign dec_i1_dbecc_d = pc1_buffer[67];
   assign dec_i0_dbecc_d = pc0_buffer[67];   // 2  

   assign dec_i1_sbecc_d = pc1_buffer[66];
   assign dec_i0_sbecc_d = pc0_buffer[66];    // 2

   assign dec_i1_perr_d = pc1_buffer[65];
   assign dec_i0_perr_d = pc0_buffer[65];   // 2

   assign dec_i1_icaf_d = pc1_buffer[64];
   assign dec_i0_icaf_d = pc0_buffer[64];    // 2
   
   assign dec_i1_pc_d[63:1] = pc1_buffer[63:1];   
   assign dec_i0_pc_d[63:1] = pc0_buffer[63:1]; // 2

   assign dec_i1_pc4_d = pc1_buffer[0];   
   assign dec_i0_pc4_d = pc0_buffer[0];   // 总共 15 个


   /*
   always_comb begin
      // if(i0_rd_enable_next | i1_rd_enable_next) begin
      if(i0_rd_enable_next ) begin

         assign dec_ib3_valid_d = dec_ib3_valid_fifo;
         assign dec_ib2_valid_d = dec_ib2_valid_fifo;
         assign dec_ib1_valid_d = dec_ib1_valid_fifo;
         assign dec_ib0_valid_d = dec_ib0_valid_fifo;

         // 队列 输出结果赋值     能够正确将 写入的 给重新读 出来了
         assign dec_i0_brp = dec_i0_brp_fifo;
         assign dec_i1_brp = dec_i1_brp_fifo;

         assign dec_i0_instr_d[31 : 0] = dec_i0_instr_fifo[31 : 0];
         assign dec_i1_instr_d[31 : 0] = dec_i1_instr_fifo[31 : 0];

         assign dec_i0_cinst_d[15 : 0] = dec_i0_cinst_fifo[15 : 0];
         assign dec_i1_cinst_d[15 : 0] = dec_i1_cinst_fifo[15 : 0];

         assign dec_debug_wdata_rs1_d = dec_debug_wdata_rs1_fifo;   

         assign dec_debug_fence_d = dec_debug_fence_fifo;  


         // 由 pc0_buffer, pc1_buffer  赋值
         assign dec_i0_fetch_page_fault = pc0_fifo[69];
         assign dec_i1_fetch_page_fault = pc1_fifo[69];  // 2

         assign dec_i0_icaf_f1_d = pc0_fifo[68];   // icaf's can only decode as i0   // 2
         
         assign dec_i1_dbecc_d = pc1_fifo[67];
         assign dec_i0_dbecc_d = pc0_fifo[67];   // 2  

         assign dec_i1_sbecc_d = pc1_fifo[66];
         assign dec_i0_sbecc_d = pc0_fifo[66];    // 2

         assign dec_i1_perr_d = pc1_fifo[65];
         assign dec_i0_perr_d = pc0_fifo[65];   // 2

         assign dec_i1_icaf_d = pc1_fifo[64];
         assign dec_i0_icaf_d = pc0_fifo[64];    // 2
         
         assign dec_i1_pc_d[63:1] = pc1_fifo[63:1];   
         assign dec_i0_pc_d[63:1] = pc0_fifo[63:1]; // 2

         assign dec_i1_pc4_d = pc1_fifo[0];   
         assign dec_i0_pc4_d = pc0_fifo[0];   // 总共 15 个
         


      end

      else begin

         assign dec_ib3_valid_d = dec_ib3_valid_buffer;
         assign dec_ib2_valid_d = dec_ib2_valid_buffer;
         assign dec_ib1_valid_d = dec_ib1_valid_buffer;
         assign dec_ib0_valid_d = dec_ib0_valid_buffer;

         // 队列 输出结果赋值     能够正确将 写入的 给重新读 出来了
         assign dec_i0_brp = dec_i0_brp_buffer;
         assign dec_i1_brp = dec_i1_brp_buffer;

         assign dec_i0_instr_d[31 : 0] = dec_i0_instr_buffer[31 : 0];
         assign dec_i1_instr_d[31 : 0] = dec_i1_instr_buffer[31 : 0];

         assign dec_i0_cinst_d[15 : 0] = dec_i0_cinst_buffer[15 : 0];
         assign dec_i1_cinst_d[15 : 0] = dec_i1_cinst_buffer[15 : 0];

         assign dec_debug_wdata_rs1_d = dec_debug_wdata_rs1_buffer;   

         assign dec_debug_fence_d = dec_debug_fence_buffer;  


         // 由 pc0_buffer, pc1_buffer  赋值
         assign dec_i0_fetch_page_fault = pc0_buffer[69];
         assign dec_i1_fetch_page_fault = pc1_buffer[69];  // 2

         assign dec_i0_icaf_f1_d = pc0_buffer[68];   // icaf's can only decode as i0   // 2
         
         assign dec_i1_dbecc_d = pc1_buffer[67];
         assign dec_i0_dbecc_d = pc0_buffer[67];   // 2  

         assign dec_i1_sbecc_d = pc1_buffer[66];
         assign dec_i0_sbecc_d = pc0_buffer[66];    // 2

         assign dec_i1_perr_d = pc1_buffer[65];
         assign dec_i0_perr_d = pc0_buffer[65];   // 2

         assign dec_i1_icaf_d = pc1_buffer[64];
         assign dec_i0_icaf_d = pc0_buffer[64];    // 2
         
         assign dec_i1_pc_d[63:1] = pc1_buffer[63:1];   
         assign dec_i0_pc_d[63:1] = pc0_buffer[63:1]; // 2

         assign dec_i1_pc4_d = pc1_buffer[0];   
         assign dec_i0_pc4_d = pc0_buffer[0];   // 总共 15 个



      end
      
   end

  */ 






   
   







endmodule