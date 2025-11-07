module dec_vp_flush_ctl  
   import swerv_types::*;
(
   input logic  clk,                      // Top level clock
   input logic  active_clk,               // Level 1 free clock
   input logic  rst_l,                    // Reset
   input logic  scan_mode,                // Scan control

   input logic freeze,                    // Clock enable for valid

   input logic  enable,                   // Clock enable

   input logic valid,                     // Valid
   input logic flush,                     // Flush pipeline

   input logic [63:0] predicted_result, 
   input logic [63:0] actual_result, 


   input logic [63:1] pred_path,        // 预测跳转的地址      
   
   // input logic [31:1] any_jal_flush_path, // any_jal 的地址
   // input logic any_jal,  

   // input logic inst_type,                 // 预测的指令类型


	output logic        flush_upper_vp,       // 值预测 刷新信号

   output logic [63:1] flush_path_vp,   // 值预测错误时, 跳转的地址

   output logic [63:0] predicted_result_new  // 更新预测值

  );

   logic                valid_vp_ff;

   logic                 mispredict;

   logic                 vp_flush;


   rvdffs #(1)  validff (.*, .clk(active_clk), .en(~freeze), .din(valid & ~flush), .dout(valid_vp_ff)); 

   // any_jal_flush_path: any_jal 的地址,  pred_path: 分支的地址, 无论预测正确与否
   // assign flush_path_vp[31:1] =   any_jal ? any_jal_flush_path[31:1] : pred_path[31:1];  
   // 对于 any_jal 有效, 要不是分支预测有效, 要不是 flush_upper_vp 无效, 所以不会影响跳转地址

   assign flush_path_vp[63:1] =   pred_path[63:1];  // pred_path: 分支的地址, 无论预测正确与否

    // 预测错误, 为 1， 预测正确，为 0
  assign mispredict = predicted_result[63:0] != actual_result[63:0];  // 预测正确了, 但是没有把预测值传递给依赖指令


   // assign mispredict = 1;
   // assign mispredict = 0;
   

   // assign vp_flush =  mispredict & inst_type;  // 一直预测错误, 每一条都刷新
   assign vp_flush =  mispredict ;
    
   // i0_ap_e4, i1_ap_e4

   assign flush_upper_vp = vp_flush & valid_vp_ff & ~flush & ~freeze;  // 刷新信号

  
   assign predicted_result_new[63:0] = flush_upper_vp ? actual_result[63:0] : predicted_result[63:0];  // 更新预测值


endmodule


