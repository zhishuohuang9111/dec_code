            // 检测 是否依赖 流水线中的 load 指令信号   
   assign dec_i0_load_decode_e3 = load_match_i0_e3;
   assign dec_i1_load_decode_e3 = load_match_i1_e3;   // 导致这个信号也存在问题了
      
      
      // 这些 值的传递存在问题    如何将 依赖指令的信号 i0_load_block_d 传递到 e4 阶段
         // pipe the results down the pipe
   rvdffe #(1) i0e2matchff (.*, .en(i0_e2_data_en), .din({load_match_i0_e1}), .dout({ load_match_i0_e2} )); 

   rvdffe #(1) i1e2matchff (.*, .en(i1_e2_data_en), .din({load_match_i1_e1}), .dout({ load_match_i1_e2} ));
   

   rvdffe #(1) i0e3matchff (.*, .en(i0_e3_data_en), .din({load_match_i0_e2_final}), .dout({ load_match_i0_e3}));

   rvdffe #(1) i1e3matchff (.*, .en(i1_e3_data_en), .din({load_match_i1_e2_final}), .dout({ load_match_i1_e3}));

   rvdffe #(1) i0e4matchff (.*, .en(i0_e4_data_en), .din({load_match_i0_e3}), .dout({ load_match_i0_e4 }));

   rvdffe #(1) i1e4matchff (.*, .en(i1_e4_data_en), .din({load_match_i1_e3}), .dout({ load_match_i1_e4 }));
