rtl/dec/dec_ib_final_ctl.sv

	"$PRO_ROOT/rtl/dec/dec_ib_final_ctl.sv" \ 

dec/dec_ib_final_ctl.sv

trace 文件: diag.dump


   只要 i0_load_block_d, i1_load_block_d 这个信号为 1, 则 i0, i1 中的 源操作数  一定会有一个为 1

   
   i0_load_block_d 这个信号只能知道 当前 dec 的指令 依赖 e1 或 e2 阶段的 load, 但是不直到具体依赖 哪一条


   dec_i0_decode_d 这个信号也要进行添加


   i0_rs1_bypass_data_d 进行了修改,   i0_load_block_d 注释了

       
   // 判断的是 当前 dec 阶段的 rs1 依赖于 当前周期的 e1, e2, e3, e4, 还是 wb 阶段 的结果
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









// arf : 判断 寄存器文件 是浮点, 还是整数的, 如果 寄存器文件相同, 才会有依赖,  不同,  则没有依赖


         *****   实际是 队列 不一定存满,  只要读 队列 为空就好


localparam DEC_INSTBUF_DEPTH = `RV_DEC_INSTBUF_DEPTH;

`define RV_DEC_INSTBUF_DEPTH 4

   /*

   原本的设计能否实现: i0 写了 0 号曹,  i0 再写 1 号曹, i1 写 2 号曹

   assign i0_wen[1] =  ibval[0] & ~ibval[1] & ifu_i0_val;  // 0 号占据了,  1 号为空
   assign i1_wen[2] =  ibval[0] & ~ibval[1] & ifu_i1_val;

   */

   队列里面只有 一条指令的 时候, 加一个信号 通知从 指令buffer 里面取一条指令, 发射给 i1


   DEC_INSTFIFO_DEPTH = 8: 设置 FIFO 的大小

   进行 FIFO 写指令的时候,  不会消耗 指令


   如果是 奇数 条指令, 一次消耗 2 条指令, 最后一次, 只有 1 条指令


   只有在 读取指令的时候, shift_ib1_ib0  等信号 才会有效

   队列为空: ibval[7:0] = 7'd0 的时候



   assign i1_wen[0] = ~ibval[0] & ~ifu_i0_val & ifu_i1_val;  // 如果 两条都有效, 则都会写道  0 号

   assign i1_wen[1] = ~ibval[0]             & ifu_i1_val;   这样会导致  i1_wen[0], i1_wen[1] 都为 1


   // 加了 cache 的, 也就是这两个的延迟 时间会长 一些 ?    load 的结果 不是在 e4 出结果, 如何将 预测值 与 实际值 进行比较
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


   exu 执行单元的 freeze 信号改变了
    .freeze        ( freeze_before_dc4           ),   // I


   /*
      在队列有空 空间的情况下, 由 ifu_i0_val, ifu_i1_val 来决定写入

      ifu_i0_val: 第 1 条指令能否写入,  ifu_i1_val: 第 2 条指令能否写入

      ifu_i0_valid:  表所传入的指令是有效的,    在队列中可以修改为   开始 进行存储依赖 的信号

      添加的队列需要实现的是,  向 队列 里面进行写指令,   读取指令的时候, 将 后面的指令 移动 到队头 


      * * * 在检测到依赖之后, 每周期 入队的指令可能为 0, 1, 2 条    所以 可能 i1 写入 1 曹, 然后 i1 写入 2 曹


      队列 里面 只存储 发射出来的实际的指令

      有 0 -7  总共 8 个存储空间,  i0, i1 都可以向这 8 个空间里面写入
      第 0 个 和 第 1 个进行 写入给 i0, i1 指令,  2 - 7 个依次往前移     刷新是, 每次前移动 2 条


      dec 阶段的 i0 有依赖前面的指令,  当前 dec 的 i0 i1 都入队,  后续的也进行入队
      dec 阶段的 i1 有依赖前面的指令,  当前 dec 的 i1 入队,  后续的也进行入队

      dec_i0_decode_d = 1 表示 i0 有效, 可以写入队列


      读 队列 的时候, 也需要 旁段 当前 从队列读取的指令, 是否进行了消耗 指令


      队列不能同时进行 写指令 和 读指令,   指令读取,  直到 队列 为空


      什么时候会 shift, 消耗掉指令的时候,  也就是 开始读取的时候


   // ~{4{flush_final}}: 预取正确的时候, 把这个信号 置为 1
         // 如果进行了刷新, 则 ibval_in[3:0] = 4'b0000,  下一周期的 ibval[3:0] 也为 0 了, 代表为 空, 可以写入 指令
   assign ibval_in[3:0] = (({4{shift0}} & ibvalid[3:0]) |
                           ({4{shift1}} & {1'b0, ibvalid[3:1]}) |  // 由 4'b1110  -->  4'b0111
                           ({4{shift2}} & {2'b0, ibvalid[3:2]})) & ~{4{flush_final}}; 


      




*/




/*
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

   assign ib0_in[31:0] = ({32{write_i0_ib0}} & ((debug_valid) ? ib0_debug_in[31:0] : ifu_i0_instr[31:0])) |
                         ({32{shift_ib1_ib0}} & ib1[31:0]) |
                         ({32{shift_ib2_ib0}} & ib2[31:0]);
   
   rvdffe #(32) ib0ff (.*, .en(ibwrite[0]), .din(ib0_in[31:0]), .dout(ib0[31:0]));

*/



   // pc tracking


   // 修改的  写的时候, write_i0_ib7 等信号 有效, 读的时候, shift_ib7_ib6 等信号 有效
   assign ibwrite[7:0] = {  write_i0_ib7 | write_i1_ib7,   
                            write_i0_ib6 | write_i1_ib6 | shift_ib7_ib6, 

                            write_i0_ib5 | write_i1_ib5 | shift_ib6_ib5 | shift_ib7_ib5, 
                            write_i0_ib4 | write_i1_ib4 | shift_ib5_ib4 | shift_ib6_ib4,  

                            write_i0_ib3 | write_i1_ib3 | shift_ib4_ib3 | shift_ib5_ib3, 
                            write_i0_ib2 | write_i1_ib2 | shift_ib3_ib2 | shift_ib4_ib2,  

                            write_i0_ib1 | write_i1_ib1 | shift_ib2_ib1 | shift_ib3_ib1,  

                            write_i0_ib0 | shift_ib1_ib0 | shift_ib2_ib0  //   write_i1_ib0 |    i0, i1 指令 都可以写入  0 槽

                            }; // 右边的每个信号中,  都 由  ifu_i0_val, ifu_i1_val 进行决定   
                    











