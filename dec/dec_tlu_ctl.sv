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


//********************************************************************************
// dec_tlu_ctl.sv
//
// 
// Function: CSRs, Commit/WB, flushing, exceptions, interrupts
// Comments: 
//
//********************************************************************************

/* ---------------------------------------------- */
// 2022.04.01 lsu_error_pkt_dc3 used to send load and store page fault. 
//             load_page_fault and store_page_fault are in lsu_exc_valid_e4

/* ---------------------------------------------- */

import swerv_types::*;
import riscv::*;
import ariane_pkg::*;

module dec_tlu_ctl #(
   parameter logic [63:0] DmBaseAddress = 64'b0 // Debug Module base address
)(
   input logic clk,
   input logic active_clk,
   input logic free_clk,
   input logic rst_l,
   input logic scan_mode,

   input logic [63:0] hart_id_i,
   
   input logic [63:1] rst_vec, // reset vector, from core pins
   input logic        nmi_int, // nmi pin
   input logic [63:1] nmi_vec, // nmi vector        
   input logic  i_cpu_halt_req,    // Asynchronous Halt request to CPU
   input logic  i_cpu_run_req,     // Asynchronous Restart request to CPU

   input logic mpc_debug_halt_req, // Async halt request
   input logic mpc_debug_run_req, // Async run request
   input logic mpc_reset_run_req, // Run/halt after reset
   
   // perf counter inputs
   input logic [1:0] ifu_pmu_instr_aligned,   // aligned instructions
   input logic       ifu_pmu_align_stall,  // aligner stalled
   input logic       ifu_pmu_fetch_stall, // fetch unit stalled
   input logic       ifu_pmu_ic_miss, // icache miss
   input logic       ifu_pmu_ic_hit, // icache hit
   input logic       ifu_pmu_bus_error, // Instruction side bus error
   input logic       ifu_pmu_bus_busy, // Instruction side bus busy 
   input logic       ifu_pmu_bus_trxn, // Instruction side bus transaction
   input logic [1:0] dec_pmu_instr_decoded, // decoded instructions
   input logic       dec_pmu_decode_stall, // decode stall
   input logic       dec_pmu_presync_stall, // decode stall due to presync'd inst
   input logic       dec_pmu_postsync_stall,// decode stall due to postsync'd inst
   input logic       lsu_freeze_dc3,         // lsu freeze stall
   input logic       lsu_store_stall_any,    // SB or WB is full, stall decode
   //input logic       dma_dccm_stall_any,     // DMA stall of lsu
   //input logic       dma_iccm_stall_any,     // DMA stall of ifu   
   input logic       exu_pmu_i0_br_misp,     // pipe 0 branch misp
   input logic       exu_pmu_i0_br_ataken,   // pipe 0 branch actual taken
   input logic       exu_pmu_i0_pc4,         // pipe 0 4 byte branch      
   input logic       exu_pmu_i1_br_misp,     // pipe 1 branch misp        
   input logic       exu_pmu_i1_br_ataken,   // pipe 1 branch actual taken
   input logic       exu_pmu_i1_pc4,         // pipe 1 4 byte branch      
   input logic       lsu_pmu_bus_trxn,       // D side bus transaction
   input logic       lsu_pmu_bus_misaligned, // D side bus misaligned
   input logic       lsu_pmu_bus_error,      // D side bus error
   input logic       lsu_pmu_bus_busy,       // D side bus busy
   input logic       pmu_freeze_dc1,
   input logic       pmu_freeze_dc2,
   input logic       pmu_icache_miss_i,      // ICache MISS counter
   input logic       pmu_icache_access_i,    // ICache ACCESS counter
   input logic       pmu_dcache_miss_i,      // DCache MISS counter
   input logic       pmu_dcache_access_i,    // DCache ACCESS counter


   input logic       iccm_dma_sb_error,      // I side dma single bit error
   
   input lsu_error_pkt_t lsu_error_pkt_dc3, // lsu precise exception/error packet

   input logic dec_pause_state, // Pause counter not zero
   input logic         lsu_imprecise_error_store_any,      // store bus error
   input logic         lsu_imprecise_error_load_any,      // store bus error
   input logic [63:0]  lsu_imprecise_error_addr_any, // store bus error address
   input logic         lsu_block_interrupts_i,       // load to side effect region

   input logic        dec_csr_wen_unq_d,       // valid csr with write - for csr legal
   input logic        dec_csr_any_unq_d,       // valid csr - for csr legal    
   input logic        dec_csr_wen_wb,      // csr write enable at wb 
   input logic [11:0] dec_csr_rdaddr_d,      // read address for csr 
   input logic [11:0] dec_csr_wraddr_wb,      // write address for csr
   input logic [63:0] dec_csr_wrdata_wb,   // csr write data at wb 
   input logic        dec_csr_stall_int_ff, // csr is mie/mstatus

   input logic dec_tlu_i0_valid_e4, // pipe 0 op at e4 is valid
   input logic dec_tlu_i1_valid_e4, // pipe 1 op at e4 is valid

   input logic dec_i0_load_e4, // during cycle after freeze asserts, load is in i0

   input logic dec_fence_pending, // tell TLU to stall DMA

   input logic [63:1] exu_npc_e4, // for NPC tracking
   input logic exu_i0_flush_lower_e4,       // pipe 0 branch mp flush
   input logic exu_i1_flush_lower_e4,       // pipe 1 branch mp flush
   input logic [63:1] exu_i0_flush_path_e4, // pipe 0 correct path for mp, merge with lower path
   input logic [63:1] exu_i1_flush_path_e4, // pipe 1 correct path for mp, merge with lower path

   input logic [63:1] dec_tlu_i0_pc_e4, // for PC/NPC tracking
   input logic [63:1] dec_tlu_i1_pc_e4, // for PC/NPC tracking

   input trap_pkt_t dec_tlu_packet_e4, // exceptions known at decode

   input logic [31:0] dec_illegal_inst, // For mtval
   input logic        dec_i0_decode_d,  // decode valid, used for clean icache diagnostics

   // branch info from pipe0 for errors or counter updates
   input logic [`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO] exu_i0_br_index_e4, // index
   input logic [1:0]  exu_i0_br_hist_e4, // history
   input logic [1:0]  exu_i0_br_bank_e4, // bank
   input logic        exu_i0_br_error_e4, // error
   input logic        exu_i0_br_start_error_e4, // start error
   input logic        exu_i0_br_valid_e4, // valid
   input logic        exu_i0_br_mp_e4, // mispredict
   input logic        exu_i0_br_middle_e4, // middle of bank
   input logic [`RV_BHT_GHR_RANGE]  exu_i0_br_fghr_e4, // FGHR when predicted 
   
   // branch info from pipe1 for errors or counter updates
   input logic [`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO] exu_i1_br_index_e4, // index
   input logic [1:0]  exu_i1_br_hist_e4, // history
   input logic [1:0]  exu_i1_br_bank_e4, // bank
   input logic        exu_i1_br_error_e4, // error
   input logic        exu_i1_br_start_error_e4, // start error
   input logic        exu_i1_br_valid_e4, // valid
   input logic        exu_i1_br_mp_e4, // mispredict
   input logic        exu_i1_br_middle_e4, // middle of bank
   input logic [`RV_BHT_GHR_RANGE]  exu_i1_br_fghr_e4, // FGHR when predicted

`ifdef RV_BTB_48
   input logic [1:0]       exu_i1_br_way_e4, // way hit or repl
   input logic [1:0]       exu_i0_br_way_e4, // way hit or repl
`else
   input logic        exu_i1_br_way_e4, // way hit or repl
   input logic        exu_i0_br_way_e4, // way hit or repl
`endif

   // Debug start
   output logic dec_dbg_cmd_done, // abstract command done
   output logic dec_dbg_cmd_fail, // abstract command failed
   output logic dec_tlu_flush_noredir_wb , // Tell fetch to idle on this flush
   output logic dec_tlu_mpc_halted_only, // Core is halted only due to MPC
   output logic dec_tlu_dbg_halted, // Core is halted and ready for debug command
   output logic dec_tlu_pmu_fw_halted, // Core is halted due to Power management unit or firmware halt
   output logic dec_tlu_debug_mode, // Core is in debug mode
   output logic dec_tlu_resume_ack, // Resume acknowledge
   output logic dec_tlu_debug_stall, // stall decode while waiting on core to empty
   output logic dec_tlu_flush_leak_one_wb, // single step
   output logic dec_tlu_flush_err_wb, // iside perr/ecc rfpc
   output logic dec_tlu_stall_dma, // stall dma access when there's a halt request

   input logic dmi_debug_req_i,
   input  logic dbg_halt_req, // DM requests a halt
   input  logic dbg_resume_req, // DM requests a resume
   input  logic ifu_miss_state_idle, // I-side miss buffer empty
   input  logic lsu_halt_idle_any, // lsu is idle
   output trigger_pkt_t  [3:0] trigger_pkt_any, // trigger info for trigger blocks

`ifdef RV_ICACHE_ECC
   input logic [63:0] ifu_ic_debug_rd_data, // diagnostic icache read data
`else
   input logic [63:0] ifu_ic_debug_rd_data, // diagnostic icache read data
`endif
   input logic ifu_ic_debug_rd_data_valid, // diagnostic icache read data valid
   output cache_debug_pkt_t dec_tlu_ic_diag_pkt, // packet of DICAWICS, DICAD0/1, DICAGO info for icache diagnostics
   // Debug end

   input logic [7:0] pic_claimid, // pic claimid for csr
   input logic [3:0] pic_pl, // pic priv level for csr
   input logic       mhwakeup, // high priority external int, wakeup if halted

   // input logic mexintpend, // external interrupt pending
   input logic [1:0] ext_int, // cyh: external interrupt pending, irq[1]: S mode irq, irq[0]: M mode irq
   input logic timer_int, // timer interrupt pending
   input logic software_int, // software interrupt pending

   output logic o_cpu_halt_status, // PMU interface, halted
   output logic o_cpu_halt_ack, // halt req ack
   output logic o_cpu_run_ack, // run req ack
   output logic o_debug_mode_status, // Core to the PMU that core is in debug mode. When core is in debug mode, the PMU should refrain from sendng a halt or run request

   output logic mpc_debug_halt_ack, // Halt ack
   output logic mpc_debug_run_ack, // Run ack
   output logic debug_brkpt_status, // debug breakpoint

   output logic [3:0] dec_tlu_meicurpl, // to PIC
   output logic [3:0] dec_tlu_meipt, // to PIC
   
   output br_tlu_pkt_t dec_tlu_br0_wb_pkt, // branch pkt to bp
   output br_tlu_pkt_t dec_tlu_br1_wb_pkt, // branch pkt to bp

   output logic [63:0] dec_csr_rddata_d,      // csr read data at wb
   output logic dec_csr_legal_d,              // csr indicates legal operation

   output logic dec_tlu_i0_kill_writeb_wb,    // I0 is flushed, don't writeback any results to arch state 
   output logic dec_tlu_i1_kill_writeb_wb,    // I1 is flushed, don't writeback any results to arch state 

   output logic dec_tlu_flush_lower_wb,       // commit has a flush (exception, int, mispredict at e4)
   output logic [63:1] dec_tlu_flush_path_wb, // flush pc
   // output logic dec_tlu_flush_icache_e4,      // flush is a fence_i rfnpc, flush icache
   // output logic dec_tlu_flush_tlb_e4,
   
   output logic dec_tlu_presync_d,            // CSR read needs to be presync'd
   output logic dec_tlu_postsync_d,           // CSR needs to be presync'd

   output logic [63:0] dec_tlu_mrac_ff,        // CSR for memory region control

   output logic dec_tlu_cancel_e4,             // Cancel lsu op at DC4 due to future trigger hit
   
   output logic dec_tlu_wr_pause_wb,           // CSR write to pause reg is at WB.
   output logic dec_tlu_flush_pause_wb,        // Flush is due to pause

   output logic [1:0] dec_tlu_perfcnt0, // toggles when pipe0 perf counter 0 has an event inc
   output logic [1:0] dec_tlu_perfcnt1, // toggles when pipe0 perf counter 1 has an event inc
   output logic [1:0] dec_tlu_perfcnt2, // toggles when pipe0 perf counter 2 has an event inc
   output logic [1:0] dec_tlu_perfcnt3, // toggles when pipe0 perf counter 3 has an event inc


   output logic dec_tlu_i0_valid_wb1,  // pipe 0 valid
   output logic dec_tlu_i1_valid_wb1,  // pipe 1 valid
   output logic dec_tlu_i0_exc_valid_wb1, // pipe 0 exception valid
   output logic dec_tlu_i1_exc_valid_wb1, // pipe 1 exception valid
   output logic dec_tlu_int_valid_wb1, // pipe 2 int valid
   output logic [4:0] dec_tlu_exc_cause_wb1, // exception or int cause
   output logic [63:0] dec_tlu_mtval_wb1, // MTVAL value
   
   // feature disable from mfdc
   output logic  dec_tlu_sideeffect_posted_disable,    // disable posted writes to side-effect address
   output logic  dec_tlu_dual_issue_disable, // disable dual issue
   output logic  dec_tlu_core_ecc_disable, // disable core ECC
   output logic  dec_tlu_sec_alu_disable, // disable secondary ALU
   output logic  dec_tlu_non_blocking_disable,    // disable non blocking loads
   output logic  dec_tlu_fast_div_disable,        // disable fast divider
   output logic  dec_tlu_bpred_disable,           // disable branch prediction
   output logic  dec_tlu_wb_coalescing_disable,   // disable writebuffer coalescing
   output logic  dec_tlu_ld_miss_byp_wb_disable,  // disable loads miss bypass write buffer
   output logic  dec_tlu_pipelining_disable,      // disable pipelining
   output logic [2:0]  dec_tlu_dma_qos_prty,    // DMA QoS priority coming from MFDC [18:16]
   
   // clock gating overrides from mcgc
   output logic  dec_tlu_misc_clk_override, // override misc clock domain gating
   output logic  dec_tlu_dec_clk_override,  // override decode clock domain gating
   output logic  dec_tlu_exu_clk_override,  // override exu clock domain gating
   output logic  dec_tlu_ifu_clk_override,  // override fetch clock domain gating
   output logic  dec_tlu_lsu_clk_override,  // override load/store clock domain gating
   output logic  dec_tlu_bus_clk_override,  // override bus clock domain gating
   output logic  dec_tlu_pic_clk_override,  // override PIC clock domain gating
   output logic  dec_tlu_dccm_clk_override, // override DCCM clock domain gating
   output logic  dec_tlu_icm_clk_override,   // override ICCM clock domain gating

   input  logic freeze_dc4,   // pipeline freeze in dc4 block flush_e4 -> flush_wb
   input  logic vec_illegal_instr_dc4_i,
   input  logic vec_load_page_fault_dc4_i,
   input  logic vec_store_page_fault_dc4_i,
   input  logic [63:0] vec_tval_dc4_i,

   // to mmu
   output logic enable_fetch_translation_o,
   output logic enable_ls_translation_o,
   output priv_lvl_t priv_lvl_o,
   output priv_lvl_t ls_priv_lvl_o,
   output logic sum_o,
   output logic mxr_o,
   output logic [PPNW-1:0] satp_ppn_o,
   output logic [ASID_WIDTH-1:0] asid_o,

   output pmpcfg_t [15:0] csr_pmpcfg_o,
   output logic [15:0][PLEN-3:0] csr_pmpaddr_o,

   output logic mstatus_tvm_o,
   output logic mstatus_tsr_o,

   // vector
   input  logic vector_state_is_dirty_i,
   output logic vector_is_off_o,


   //浮点扩展添加
   input logic fp_state_is_dirty_i,
   input logic [4:0] fpu_flags_wb_i,
   input logic fpu_flags_wen_wb_i,
   output logic fp_is_off_o,
   output logic [2:0] fpu_frm_csr_o,
   output logic [4:0] fpu_fflags_csr_o
   // output logic [6:0] fpu_prec_csr
   );

   //------------------fpu logic-----------------

   fcsr_t fcsr, fcsr_n;  //7位prec, 3位frm, 5位fflags, just use frm and fflags
   logic csr_fcsr, csr_frm, csr_fflags;
   logic fcsr_is_dirty;
   assign fpu_frm_csr_o = fcsr.frm;
   assign fpu_fflags_csr_o = fcsr.fflags;
   assign fp_is_off_o = (mstatus.fs == riscv::Off);
   assign vector_is_off_o = (mstatus.vs == riscv::Off);
   //  assign fpu_prec_csr = fcsr[14:8];

   //------------------fpu logic-----------------


   logic dmi_debug_req_f1, dmi_debug_req_f2, dmi_debug_req_f3;
   logic dmi_debug_req_valid_e4, dmi_debug_req_valid_e5;
   // logic dmi_debug_req_posedge, dmi_debug_req_posedge_f1, dmi_debug_req_posedge_f2;

   //------------------添加结束-------------------
   
   /* ------------------------ CSR -------------------------- */
   localparam SSTATUS_WRITE_MASK = SSTATUS_SIE | SSTATUS_SPIE | SSTATUS_SPP | SSTATUS_VS | SSTATUS_FS | SSTATUS_SUM | SSTATUS_MXR;
   localparam SSTATUS_READ_MASK = SSTATUS_UIE | SSTATUS_SIE | SSTATUS_SPIE | SSTATUS_SPP | SSTATUS_VS | SSTATUS_FS | SSTATUS_XS | SSTATUS_SUM | SSTATUS_MXR | SSTATUS_UPIE | SSTATUS_SPIE | SSTATUS_UXL | SSTATUS_SD; 
   localparam MIE_WRITE_MASK = 64'h0000_0000_4000_0000 | MIP_SSIP | MIP_STIP | MIP_SEIP | MIP_MSIP | MIP_MTIP | MIP_SEIP; //mie[30] is M-Mode Correctable Error interrupt enable
   localparam MIP_WRITE_MASK = MIP_SSIP | MIP_STIP | MIP_SEIP;
   // localparam SIE_WRITE_MASK = MIP_SSIP | MIP_STIP | MIP_SEIP;
   localparam SIP_WRITE_MASK = MIP_SSIP;
   localparam MHPMCOUNTER_NUM = 8;  // the number of MHPMCOUNTER, 1 ~ 29
   localparam MHPMCOUNTER_INDEX_SIZE = $clog2(MHPMCOUNTER_NUM);

   // logic [63:0] cycle, cycle_n;
   /* ------------------------ CSR -------------------------- */

   /* ----------------- privilege level ----------------- */
   priv_lvl_t privilege_level, privilege_level_n;
   logic debug_mode, debug_mode_n;
   /* ----------------- privilege level ----------------- */

   /* ----------------- physical memory protection ----------------- */
   pmpcfg_t [15:0] pmpcfg_n, pmpcfg;
   logic [15:0][PLEN-3:0] pmpaddr_n, pmpaddr;
   /* ----------------- physical memory protection ----------------- */

   /* ------------------ mmu -----------------------------*/
   logic load_page_fault_e4, store_page_fault_e4;
   /* ------------------ mmu -----------------------------*/

   /* -------------------------temperary-----------------*/
   logic wr_sepc_wb;
   // logic wr_sscratch_wb;
   /* -------------------------temperary-----------------*/


   logic         dec_csr_wen_wb_mod, clk_override, e4e5_int_clk, nmi_lsu_load_type, nmi_lsu_store_type, nmi_int_detected_f, nmi_lsu_load_type_f, 
                 nmi_lsu_store_type_f, allow_dbg_halt_csr_write, dbg_cmd_done_ns, i_cpu_run_req_d1_raw, debug_mode_status, lsu_single_ecc_error_wb, 
                 i0_mp_e4, i1_mp_e4, sel_npc_e4, sel_npc_wb, ce_int, mtval_capture_lsu_wb, wr_mdeau_wb, micect_cout_nc, miccmect_cout_nc, 
                 mdccmect_cout_nc, nmi_in_debug_mode, dpc_capture_npc, dpc_capture_pc, tdata_load, tdata_opcode, tdata_action, perfcnt_halted;
           

   logic reset_delayed, reset_detect, reset_detected;
   logic wr_mstatus_wb, wr_mtvec_wb, wr_mie_wb, wr_mcyclel_wb, wr_mcycleh_wb, 
         wr_minstretl_wb, wr_minstreth_wb, wr_mscratch_wb, wr_mepc_wb, wr_mcause_wb, wr_mtval_wb,
         wr_mrac_wb, wr_meihap_wb, wr_meicurpl_wb, wr_meipt_wb, wr_dcsr_wb,
         wr_dpc_wb, wr_meicidpl_wb, wr_meivt_wb, wr_meicpct_wb, wr_micect_wb, wr_miccmect_wb,
         wr_mdccmect_wb,wr_mhpme3_wb, wr_mhpme4_wb, wr_mhpme5_wb, wr_mhpme6_wb;
   logic wr_mgpmc_wb, mgpmc_b, mgpmc;
   logic wr_mtsel_wb, wr_mtdata1_t0_wb, wr_mtdata1_t1_wb, wr_mtdata1_t2_wb, wr_mtdata1_t3_wb, wr_mtdata2_t0_wb, wr_mtdata2_t1_wb, wr_mtdata2_t2_wb, wr_mtdata2_t3_wb;
   logic [63:0] mtdata2_t0, mtdata2_t1, mtdata2_t2, mtdata2_t3, mtdata2_tsel_out, mtdata1_tsel_out;
   logic [9:0]  mtdata1_t0_ns, mtdata1_t0, mtdata1_t1_ns, mtdata1_t1, mtdata1_t2_ns, mtdata1_t2, mtdata1_t3_ns, mtdata1_t3;
   logic [27:0] tdata_wrdata_wb;
   logic [1:0] mtsel_ns, mtsel;
   logic tlu_i0_kill_writeb_e4, tlu_i1_kill_writeb_e4;
   // logic [1:0]  mstatus_ns, mstatus;
   status_rv_t mstatus, mstatus_n;
   logic [63:0] medeleg, medeleg_n;
   logic [63:0] mideleg, mideleg_n;
   // logic [1:0]  sstatus_ns, sstatus;
   // logic [63:0] medeleg_ns, medeleg;
   // logic [63:0] mideleg_ns, mideleg;
   logic [63:0] dscratch0_ns, dscratch0;
   logic [63:0] dscratch1_ns, dscratch1;
   // logic mstatus_mie_ns;
   // logic [62:0] mtvec_ns, mtvec;
   logic [63:0] mtvec_n, mtvec;
   // logic [15:2] dcsr_ns, dcsr; 
   dcsr_t dcsr, dcsr_n;
   logic [63:0] mip_n, mip;
   logic [63:0] mie_n, mie;

   // logic [63:0] menvcfg_n, menvcfg;

   logic [63:0] mcycle_n, mcycle;
   // logic [63:0] mcycleh_ns, mcycleh;
   logic [63:0] minstret_n, minstret;
   // logic [63:0] minstreth_ns, minstreth;
   logic [MHPMCOUNTER_NUM-1:0] [63:0] mhpmcounter, mhpmcounter_n;
   logic [MHPMCOUNTER_NUM-1:0] [63:0] mhpmevent, mhpmevent_n;
   logic [31:0] mcounteren_n, mcounteren;
   logic [31:0] mcountinhibit_n, mcountinhibit;

   logic [63:0] micect_ns, micect, miccmect_ns, miccmect, mdccmect_ns, mdccmect; 
   logic [26:0] micect_inc, miccmect_inc, mdccmect_inc;
   logic [63:0] mscratch;
   logic [63:0] mrac, mrac_n;
   logic [9:2] meihap;
   logic [63:10] meivt;
   logic [3:0] meicurpl_ns, meicurpl;
   logic [3:0] meicidpl_ns, meicidpl;
   logic [3:0] meipt_ns, meipt;
   logic [63:0] mdseac;
   logic mdseac_locked_ns, mdseac_locked_f, mdseac_en, nmi_lsu_detected;

   logic [63:0] sstatus_n, sstatus; // this is a shadow reg of mstatus
   logic [63:0] sie_n, sie; // this is a shadow reg of mie
   logic [63:0] stvec_n, stvec;
   logic [31:0] scounteren_n, scounteren;
   logic [63:0] sscratch_n, sscratch;
   logic [63:0] sepc_n, sepc;
   logic [63:0] scause_n, scause;
   logic [63:0] stval_n, stval;
   logic [63:0] sip_n, sip; // this is a shadow reg of mip
   satp_t satp_n, satp;

   logic [63:0] mepc_n, mepc;
   // logic [63:1] dpc_ns, dpc;
   logic [63:0] dpc_n, dpc;
   // logic [63:0] mcause_ns, mcause;
   logic [63:0] mcause_n, mcause;
   // logic [63:0] scause_ns, scause;
   // logic [63:0] mtval_ns, mtval;
   // logic [63:0] stval_ns, stval;
   logic [63:0] tval_ns;
   logic [63:0] mtval_n, mtval;

   logic       mret_wb, sret_wb;
   logic dec_pause_state_f, dec_tlu_wr_pause_wb_f, pause_expired_e4, pause_expired_wb;
   logic       tlu_flush_lower_e4, tlu_flush_lower_wb;
   logic [63:1] tlu_flush_path_e4, tlu_flush_path_wb;
   logic i0_valid_wb, i1_valid_wb;

   priv_lvl_t trap_into_priv_lvl_e4, trap_into_priv_lvl_wb;
   logic [63:0] trap_vec;
   logic [5:0] vectored_cause;
   logic       vpath_overflow_nc;
   logic [63:1] vectored_path, interrupt_path;
   logic [18:2] dicawics_ns, dicawics;
   logic        wr_dicawics_wb, wr_dicad0_wb, wr_dicad1_wb;
   logic [63:0] dicad0_ns, dicad0;
`ifdef RV_ICACHE_ECC
   logic [9:0]  dicad1_ns, dicad1;
 `else
   logic [1:0]  dicad1_ns, dicad1;   
 `endif
   logic dret_e4;
   logic dret_wb;
   logic ebreak_to_debug_en;
   logic ecall_from_u_mode_e4, ecall_from_s_mode_e4, ecall_from_m_mode_e4;
   logic        ebreak_e4, ebreak_to_debug_mode_e4, ecall_e4, illegal_e4, illegal_e4_qual, mret_e4, sret_e4, inst_acc_e4, fence_i_e4, sfence_vma_e4, 
                ic_perr_e4, iccm_sbecc_e4, fetch_page_fault_e4, ebreak_to_debug_mode_wb, kill_ebreak_count_wb, inst_acc_second_e4;
   logic        ebreak_wb, ecall_wb, illegal_wb,  illegal_raw_wb, inst_acc_wb, inst_acc_second_wb, fence_i_wb, sfence_vma_wb, ic_perr_wb, iccm_sbecc_wb, fetch_page_fault_wb;
   logic mce_int_ready, mext_int_ready, mtimer_int_ready, msoftware_int_ready, mhwakeup_ready, 
         take_mext_int, take_mce_int, take_mtimer_int, take_msoft_int, take_nmi, take_nmi_wb;
   logic sext_int_ready, stimer_int_ready, ssoftware_int_ready;
   logic take_sext_int, take_stimer_int, take_ssoft_int;
   logic global_interrupt_enable;
   logic i0_exception_valid_e4, interrupt_valid, i0_exception_valid_wb, interrupt_valid_wb, exc_or_int_valid, exc_or_int_valid_wb, mdccme_ce_req, miccme_ce_req, mice_ce_req;
   logic synchronous_flush_e4;
   logic [4:0] exc_cause_e4, exc_cause_wb;
   logic        mcyclel_cout, mcyclel_cout_f;
   logic [63:0] mcyclel_inc;
   logic        mcycleh_cout_nc;
   logic [63:0] mcycleh_inc;
   logic        minstretl_cout, minstretl_cout_f, minstret_enable;
   logic [63:0] minstretl_inc, minstretl_read;
   logic        minstreth_cout_nc;
   logic [63:0] minstreth_inc, minstreth_read;
   logic [63:1] pc_e4, pc_wb, npc_e4, npc_wb;
   logic        mtval_capture_pc_wb, mtval_capture_inst_wb, mtval_clear_wb, mtval_capture_pc_plus2_wb;
   logic valid_csr;
   logic [`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO] dec_tlu_br0_addr_e4, dec_tlu_br1_addr_e4;
   logic [1:0]  dec_tlu_br0_bank_e4, dec_tlu_br1_bank_e4;
   logic rfpc_i0_e4, rfpc_i1_e4;
   logic lsu_i0_rfnpc_dc4, lsu_i1_rfnpc_dc4;
   logic dec_tlu_br0_error_e4, dec_tlu_br0_start_error_e4, dec_tlu_br0_v_e4;
   logic dec_tlu_br1_error_e4, dec_tlu_br1_start_error_e4, dec_tlu_br1_v_e4;
   logic lsu_i0_exc_dc4, lsu_i1_exc_dc4, lsu_i0_exc_dc4_raw, lsu_i1_exc_dc4_raw, lsu_exc_ma_dc4, lsu_exc_acc_dc4, lsu_exc_st_dc4, 
         lsu_exc_valid_e4, lsu_exc_valid_e4_raw, lsu_exc_valid_wb, lsu_i0_exc_wb, 
         block_interrupts, lsu_block_interrupts_dc3, lsu_block_interrupts_e4;
   logic tlu_i0_commit_cmt, tlu_i1_commit_cmt;
   logic i0_trigger_eval_e4, i1_trigger_eval_e4, lsu_freeze_e4, lsu_freeze_pulse_e3, lsu_freeze_pulse_e4;
   logic vec_ls_exc_valid_e4, vec_ls_exc_valid_wb;
   logic vec_illegal_instr_wb;

   logic request_debug_mode_e4, request_debug_mode_wb, request_debug_mode_done, request_debug_mode_done_f;
   logic take_halt, take_halt_f, halt_taken, halt_taken_f, internal_dbg_halt_mode, dbg_tlu_halted_f, take_reset,
         dbg_tlu_halted, core_empty, lsu_halt_idle_any_f, ifu_miss_state_idle_f, resume_ack_ns,
          debug_halt_req_f, debug_resume_req_f, enter_debug_halt_req, dcsr_single_step_done, dcsr_single_step_done_f,
          debug_halt_req_d1, debug_halt_req_ns, dcsr_single_step_running, dcsr_single_step_running_f, internal_dbg_halt_timers;
         
   logic [3:0] i0_trigger_e4, i1_trigger_e4, trigger_action, trigger_enabled, 
               i0_trigger_chain_masked_e4, i1_trigger_chain_masked_e4;
   logic [2:0] trigger_chain;
   logic       i0_trigger_hit_e4, i0_trigger_hit_raw_e4, i0_trigger_action_e4,
               trigger_hit_e4, trigger_hit_wb, i0_trigger_hit_wb,
               mepc_trigger_hit_sel_pc_e4,
               mepc_trigger_hit_sel_pc_wb;
   logic       i1_trigger_hit_e4, i1_trigger_hit_raw_e4, i1_trigger_action_e4;
   logic [3:0] update_hit_bit_e4, update_hit_bit_wb, i0_iside_trigger_has_pri_e4, i1_iside_trigger_has_pri_e4, 
               i0_lsu_trigger_has_pri_e4, i1_lsu_trigger_has_pri_e4;
   logic cpu_halt_status, cpu_halt_ack, cpu_run_ack, ext_halt_pulse, i_cpu_halt_req_d1, i_cpu_run_req_d1;

   logic inst_acc_e4_raw, trigger_hit_dmode_e4, trigger_hit_dmode_wb, trigger_hit_for_dscr_cause_wb;
   logic wr_mcgc_wb, wr_mfdc_wb;
   logic [8:0] mcgc;
   logic [18:0] mfdc; 
   logic [13:0] mfdc_int, mfdc_ns;
   logic i_cpu_halt_req_sync_qual, i_cpu_run_req_sync_qual, pmu_fw_halt_req_ns, pmu_fw_halt_req_f,
         fw_halt_req, enter_pmu_fw_halt_req, pmu_fw_tlu_halted, pmu_fw_tlu_halted_f, internal_pmu_fw_halt_mode, 
         internal_pmu_fw_halt_mode_f;
   logic dcsr_single_step_running_ff;
   logic nmi_int_delayed, nmi_int_detected;
   logic [3:0] trigger_execute, trigger_data, trigger_store;
   logic mpc_run_state_ns, debug_brkpt_status_ns, mpc_debug_halt_ack_ns, mpc_debug_run_ack_ns, dbg_halt_state_ns, dbg_run_state_ns, 
         dbg_halt_state_f, mpc_debug_halt_req_sync_f, mpc_debug_run_req_sync_f, mpc_halt_state_f, mpc_halt_state_ns, mpc_run_state_f, debug_brkpt_status_f, 
         mpc_debug_halt_ack_f, mpc_debug_run_ack_f, dbg_run_state_f, dbg_halt_state_ff, mpc_debug_halt_req_sync_pulse, 
         mpc_debug_run_req_sync_pulse, debug_brkpt_valid, debug_halt_req, debug_resume_req, dec_tlu_mpc_halted_only_ns;

   logic csr_access_privilege_violation, csr_access_write_violation, csr_access_read_violation;
   logic write_csr_flush_pipeline, write_csr_flush_pipeline_f, write_csr_flush_e4;
   
   assign clk_override = dec_tlu_dec_clk_override;
   
   // Async inputs to the core have to be sync'd to the core clock.
   logic nmi_int_sync, timer_int_sync, software_int_sync, i_cpu_halt_req_sync, i_cpu_run_req_sync, mpc_debug_halt_req_sync, mpc_debug_run_req_sync;
   logic [1:0] ext_int_sync;
   rvsyncss #(9) syncro_ff(.*, 
                           .clk(free_clk),  
                           .din ({nmi_int, ext_int[1:0], timer_int, software_int, i_cpu_halt_req, i_cpu_run_req, mpc_debug_halt_req, mpc_debug_run_req}), 
                           .dout({nmi_int_sync, ext_int_sync[1:0], timer_int_sync, software_int_sync, i_cpu_halt_req_sync, i_cpu_run_req_sync, mpc_debug_halt_req_sync, mpc_debug_run_req_sync}));

   // for CSRs that have inpipe writes only

   // logic csr_wr_clk;
   lsu_error_pkt_t lsu_error_pkt_dc4;

   // rvoclkhdr csrwr_wb_cgc ( .en(dec_csr_wen_wb_mod | clk_override), .l1clk(csr_wr_clk), .* );
   // logic lsu_e3_e4_clk, lsu_e4_e5_clk;
   // rvoclkhdr lsu_e3_e4_cgc ( .en(lsu_error_pkt_dc3.exc_valid | lsu_error_pkt_dc4.exc_valid | lsu_error_pkt_dc3.single_ecc_error | lsu_error_pkt_dc4.single_ecc_error | clk_override), .l1clk(lsu_e3_e4_clk), .* );
   // rvoclkhdr lsu_e4_e5_cgc ( .en(lsu_error_pkt_dc4.exc_valid | lsu_exc_valid_wb | clk_override), .l1clk(lsu_e4_e5_clk), .* );

   logic e4e5_clk, e4_valid, e5_valid, e4e5_valid, internal_dbg_halt_mode_f;
   assign e4_valid = dec_tlu_i0_valid_e4 | dec_tlu_i1_valid_e4;
   assign e4e5_valid = e4_valid | e5_valid;
   // rvoclkhdr e4e5_cgc ( .en(e4e5_valid | clk_override), .l1clk(e4e5_clk), .* );
   // rvoclkhdr e4e5_int_cgc ( .en(e4e5_valid | internal_dbg_halt_mode_f | i_cpu_run_req_d1 | interrupt_valid | interrupt_valid_wb | reset_delayed | pause_expired_e4 | pause_expired_wb | clk_override | dmi_debug_req_posedge_f1 | lsu_exc_valid_e4 | lsu_exc_valid_wb), .l1clk(e4e5_int_clk), .* );


   assign lsu_freeze_pulse_e3 = lsu_freeze_dc3 & ~lsu_freeze_e4;
   rvdffe #(4)  freeff1 (.*, .en(~freeze_dc4), .clk(free_clk),
                              .din({internal_dbg_halt_mode, lsu_freeze_dc3, lsu_freeze_pulse_e3, lsu_block_interrupts_dc3}), 
                              .dout({internal_dbg_halt_mode_f, lsu_freeze_e4, lsu_freeze_pulse_e4, lsu_block_interrupts_e4}));
   rvdffsc #(4)  freeff2 (.*, .en(1'b1), .clear(freeze_dc4), .clk(free_clk), 
                              .din({e4_valid, tlu_flush_lower_e4, tlu_i0_kill_writeb_e4, tlu_i1_kill_writeb_e4 }), 
                              .dout({e5_valid, tlu_flush_lower_wb, dec_tlu_i0_kill_writeb_wb, dec_tlu_i1_kill_writeb_wb}));


   rvdff #(2) reset_ff (.*, .clk(free_clk), .din({1'b1, reset_detect}), .dout({reset_detect, reset_detected}));
   assign reset_delayed = reset_detect ^ reset_detected;

   rvdff #(4) nmi_ff (.*, .clk(free_clk), .din({nmi_int_sync, nmi_int_detected, nmi_lsu_load_type, nmi_lsu_store_type}), .dout({nmi_int_delayed, nmi_int_detected_f, nmi_lsu_load_type_f, nmi_lsu_store_type_f}));

   // Filter subsequent bus errors after the first, until the lock on MDSEAC is cleared
   assign nmi_lsu_detected = ~mdseac_locked_f & (lsu_imprecise_error_load_any | lsu_imprecise_error_store_any);
   
   assign nmi_int_detected = (nmi_int_sync & ~nmi_int_delayed) | nmi_lsu_detected | (nmi_int_detected_f & ~take_nmi_wb);
   // if the first nmi is a lsu type, note it. If there's already an nmi pending, ignore
   assign nmi_lsu_load_type = (nmi_lsu_detected & lsu_imprecise_error_load_any & ~(nmi_int_detected_f & ~take_nmi_wb)) | (nmi_lsu_load_type_f & ~take_nmi_wb);
   assign nmi_lsu_store_type = (nmi_lsu_detected & lsu_imprecise_error_store_any & ~(nmi_int_detected_f & ~take_nmi_wb)) | (nmi_lsu_store_type_f & ~take_nmi_wb);

`define MSTATUS_MIE 0
`define MIP_MCEIP 3
`define MIP_MEIP 2
`define MIP_MTIP 1
`define MIP_MSIP 0

`define MIE_MCEIE 3
`define MIE_MEIE 2
`define MIE_MTIE 1
`define MIE_MSIE 0

`define DCSR_EBREAKM 15    
`define DCSR_STEPIE 11    
`define DCSR_STOPC 10    
//`define DCSR_STOPT 9    
`define DCSR_STEP 2    

   // ----------------------------------------------------------------------
   // MPC halt
   // - can interact with debugger halt and v-v

    rvdff #(11)  mpvhalt_ff (.*, .clk(free_clk), 
                                 .din({mpc_debug_halt_req_sync, mpc_debug_run_req_sync,
                                       mpc_halt_state_ns, mpc_run_state_ns, debug_brkpt_status_ns, 
                                       mpc_debug_halt_ack_ns, mpc_debug_run_ack_ns,
                                       dbg_halt_state_ns, dbg_run_state_ns, dbg_halt_state_f,
                                       dec_tlu_mpc_halted_only_ns}), 
                                .dout({mpc_debug_halt_req_sync_f, mpc_debug_run_req_sync_f,
                                       mpc_halt_state_f, mpc_run_state_f, debug_brkpt_status_f, 
                                       mpc_debug_halt_ack_f, mpc_debug_run_ack_f,
                                       dbg_halt_state_f, dbg_run_state_f, dbg_halt_state_ff,
                                       dec_tlu_mpc_halted_only}));

   // turn level sensitive requests into pulses
   assign mpc_debug_halt_req_sync_pulse = mpc_debug_halt_req_sync & ~mpc_debug_halt_req_sync_f;
   assign mpc_debug_run_req_sync_pulse = mpc_debug_run_req_sync & ~mpc_debug_run_req_sync_f;

   // states
   assign mpc_halt_state_ns = (mpc_halt_state_f | mpc_debug_halt_req_sync_pulse) & ~mpc_debug_run_req_sync;
   assign mpc_run_state_ns = (mpc_run_state_f | (mpc_debug_run_req_sync_pulse & ~mpc_debug_run_ack_f)) & (internal_dbg_halt_mode_f & ~dcsr_single_step_running_f);

   // note, MPC halt can allow the jtag debugger to just start sending commands. When that happens, set the interal debugger halt state to prevent
   // MPC run from starting the core.
   assign dbg_halt_state_ns = (dbg_halt_state_f | (dbg_halt_req /*| dcsr_single_step_done_f*/ | trigger_hit_dmode_wb /*| ebreak_to_debug_mode_wb*/)) & ~dbg_resume_req;
   assign dbg_run_state_ns = (dbg_run_state_f | dbg_resume_req) & (internal_dbg_halt_mode_f & ~dcsr_single_step_running_f);

   // tell dbg we are only MPC halted 
   assign dec_tlu_mpc_halted_only_ns = ~dbg_halt_state_f & mpc_halt_state_f;
   
   // this asserts from detection of bkpt until after we leave debug mode
   assign debug_brkpt_valid = ebreak_to_debug_mode_wb | trigger_hit_dmode_wb;
   assign debug_brkpt_status_ns = (debug_brkpt_valid | debug_brkpt_status_f) & (internal_dbg_halt_mode & ~dcsr_single_step_running_f);

   // acks back to interface
   assign mpc_debug_halt_ack_ns = mpc_halt_state_f & internal_dbg_halt_mode_f & mpc_debug_halt_req_sync & core_empty;
   assign mpc_debug_run_ack_ns = (mpc_debug_run_req_sync & ~dbg_halt_state_ns & ~mpc_debug_halt_req_sync) | (mpc_debug_run_ack_f & mpc_debug_run_req_sync) ;

   // Pins
   assign mpc_debug_halt_ack = mpc_debug_halt_ack_f;
   assign mpc_debug_run_ack = mpc_debug_run_ack_f;
   assign debug_brkpt_status = debug_brkpt_status_f;


   // combine MPC and DBG halt requests 
   assign debug_halt_req = (dbg_halt_req | mpc_debug_halt_req_sync | (reset_delayed & ~mpc_reset_run_req)) & ~internal_dbg_halt_mode_f;
   
   assign debug_resume_req = ~debug_resume_req_f &  // squash back to back resumes
                             ((mpc_run_state_ns & ~dbg_halt_state_ns) |  // MPC run req
                              (dbg_run_state_ns & ~mpc_halt_state_ns)); // dbg request is a pulse
   
   // HALT 
   
   // dbg/pmu/fw requests halt, service as soon as lsu is not blocking interrupts
   assign take_halt = (debug_halt_req_f | pmu_fw_halt_req_f) & ~lsu_block_interrupts_i & ~synchronous_flush_e4 & ~mret_e4 & ~sret_e4 & ~halt_taken_f & ~dec_tlu_flush_noredir_wb & ~take_reset & ~dret_e4;
   
   // hold after we take a halt, so we don't keep taking halts
   assign halt_taken = (dec_tlu_flush_noredir_wb & ~dec_tlu_flush_pause_wb) | (halt_taken_f & ~dbg_tlu_halted_f & ~pmu_fw_tlu_halted_f & ~interrupt_valid_wb);

   // After doing halt flush (RFNPC) wait until core is idle before asserting a particular halt mode
   // It takes a cycle for mb_empty to assert after a fetch, take_halt covers that cycle
   assign core_empty = lsu_halt_idle_any & lsu_halt_idle_any_f & ifu_miss_state_idle & ifu_miss_state_idle_f & ~debug_halt_req & ~debug_halt_req_d1;

//--------------------------------------------------------------------------------
// Debug start
//

   assign enter_debug_halt_req = (~internal_dbg_halt_mode_f & debug_halt_req) /*| dcsr_single_step_done_f*/ | trigger_hit_dmode_wb /*| ebreak_to_debug_mode_wb*/;
   
   // dbg halt state active from request until non-step resume
   assign internal_dbg_halt_mode = debug_halt_req_ns | (internal_dbg_halt_mode_f & ~(debug_resume_req_f & /*~dcsr[`DCSR_STEP]*/~dcsr.step));
   // dbg halt can access csrs as long as we are not stepping
   assign allow_dbg_halt_csr_write = internal_dbg_halt_mode_f & ~dcsr_single_step_running_f;
   

   // hold debug_halt_req_ns high until we enter debug halt
   assign debug_halt_req_ns = enter_debug_halt_req | (debug_halt_req_f & ~dbg_tlu_halted);
   
   assign dbg_tlu_halted = (debug_halt_req_f & core_empty & halt_taken) | (dbg_tlu_halted_f & ~debug_resume_req_f);

   assign resume_ack_ns = (debug_resume_req_f & dbg_tlu_halted_f & dbg_run_state_ns);

   assign dcsr_single_step_done = dec_tlu_i0_valid_e4 & ~dec_tlu_dbg_halted & /*dcsr[`DCSR_STEP]*/dcsr.step & ~rfpc_i0_e4;

   assign dcsr_single_step_running = (debug_resume_req_f & /*dcsr[`DCSR_STEP]*/dcsr.step) | (dcsr_single_step_running_f & ~dcsr_single_step_done_f);

   assign dbg_cmd_done_ns = dec_tlu_i0_valid_e4 & dec_tlu_dbg_halted;
 
   // used to hold off commits after an in-pipe debug mode request (triggers, DCSR)
   assign request_debug_mode_e4 = (trigger_hit_dmode_e4 | ebreak_to_debug_mode_e4) | (request_debug_mode_wb & ~dec_tlu_flush_lower_wb);

   assign request_debug_mode_done = (request_debug_mode_wb | request_debug_mode_done_f) & ~dbg_tlu_halted_f;

    rvdff #(22)  halt_ff (.*, .clk(free_clk), .din({halt_taken, take_halt, lsu_halt_idle_any, ifu_miss_state_idle, dbg_tlu_halted, 
                                  resume_ack_ns, dbg_cmd_done_ns, debug_halt_req_ns, debug_resume_req, trigger_hit_dmode_e4, 
                                  dcsr_single_step_done, debug_halt_req,  update_hit_bit_e4[3:0], dec_tlu_wr_pause_wb, dec_pause_state,
                                  request_debug_mode_e4, request_debug_mode_done, dcsr_single_step_running, dcsr_single_step_running_f}), 
                           .dout({halt_taken_f, take_halt_f, lsu_halt_idle_any_f, ifu_miss_state_idle_f, dbg_tlu_halted_f, 
                                  dec_tlu_resume_ack, dec_dbg_cmd_done, debug_halt_req_f, debug_resume_req_f, trigger_hit_dmode_wb,
                                  dcsr_single_step_done_f, debug_halt_req_d1, update_hit_bit_wb[3:0], dec_tlu_wr_pause_wb_f, dec_pause_state_f,
                                  request_debug_mode_wb, request_debug_mode_done_f, dcsr_single_step_running_f, dcsr_single_step_running_ff}));

   assign dec_tlu_debug_stall = debug_halt_req_f;
   assign dec_tlu_dbg_halted = dbg_tlu_halted_f;
   assign dec_tlu_debug_mode = internal_dbg_halt_mode_f;
   assign dec_tlu_pmu_fw_halted = pmu_fw_tlu_halted_f;

   // kill fetch redirection on flush if going to halt, or if there's a fence during db-halt
   assign dec_tlu_flush_noredir_wb = take_halt_f | ((fence_i_wb | sfence_vma_wb) & internal_dbg_halt_mode_f) | dec_tlu_flush_pause_wb | (trigger_hit_wb & trigger_hit_dmode_wb);

   // 1 cycle after writing the PAUSE counter, flush with noredir to idle F1-D.
   assign dec_tlu_flush_pause_wb = dec_tlu_wr_pause_wb_f & ~interrupt_valid_wb;

   // detect end of pause counter and rfpc
   assign pause_expired_e4 = ~dec_pause_state & dec_pause_state_f & ~(mext_int_ready | mce_int_ready | mtimer_int_ready | msoftware_int_ready | sext_int_ready | stimer_int_ready | ssoftware_int_ready | nmi_int_detected) & ~interrupt_valid_wb & ~debug_halt_req_f & ~pmu_fw_halt_req_f & ~halt_taken_f;
   
   // stall dma fifo if a fence is pending, decode is waiting for lsu to idle before decoding the fence inst.
   assign dec_tlu_stall_dma = dec_fence_pending;
   assign dec_tlu_flush_leak_one_wb = dec_tlu_flush_lower_wb  & /*dcsr[`DCSR_STEP]*/dcsr.step & (dec_tlu_resume_ack | dcsr_single_step_running);
   assign dec_tlu_flush_err_wb = dec_tlu_flush_lower_wb & (ic_perr_wb | iccm_sbecc_wb);

   // If DM attempts to access an illegal CSR, send cmd_fail back
   assign dec_dbg_cmd_fail = illegal_raw_wb & dec_dbg_cmd_done;

   
   //--------------------------------------------------------------------------------
   //--------------------------------------------------------------------------------
   // Triggers
   //
`define MTDATA1_DMODE 9
`define MTDATA1_SEL 7
`define MTDATA1_ACTION 6
`define MTDATA1_CHAIN 5
`define MTDATA1_MATCH 4
`define MTDATA1_M_ENABLED 3
`define MTDATA1_EXE 2
`define MTDATA1_ST 1
`define MTDATA1_LD 0

   // Prioritize trigger hits with other exceptions.
   //
   // Trigger should have highest priority except:
   // - trigger is an execute-data and there is an inst_access exception (lsu triggers won't fire, inst. is nop'd by decode)
   // - trigger is a store-data and there is a lsu_acc_exc or lsu_ma_exc.
   assign trigger_execute[3:0] = {mtdata1_t3[`MTDATA1_EXE], mtdata1_t2[`MTDATA1_EXE], mtdata1_t1[`MTDATA1_EXE], mtdata1_t0[`MTDATA1_EXE]};
   assign trigger_data[3:0] = {mtdata1_t3[`MTDATA1_SEL], mtdata1_t2[`MTDATA1_SEL], mtdata1_t1[`MTDATA1_SEL], mtdata1_t0[`MTDATA1_SEL]};
   assign trigger_store[3:0] = {mtdata1_t3[`MTDATA1_ST], mtdata1_t2[`MTDATA1_ST], mtdata1_t1[`MTDATA1_ST], mtdata1_t0[`MTDATA1_ST]};


   // MSTATUS[MIE] needs to be on to take triggers unless the action is trigger to debug mode.
   assign trigger_enabled[3:0] = {(mtdata1_t3[`MTDATA1_ACTION] | mstatus.mie) & mtdata1_t3[`MTDATA1_M_ENABLED], 
                                  (mtdata1_t2[`MTDATA1_ACTION] | mstatus.mie) & mtdata1_t2[`MTDATA1_M_ENABLED], 
                                  (mtdata1_t1[`MTDATA1_ACTION] | mstatus.mie) & mtdata1_t1[`MTDATA1_M_ENABLED], 
                                  (mtdata1_t0[`MTDATA1_ACTION] | mstatus.mie) & mtdata1_t0[`MTDATA1_M_ENABLED]};

   // iside exceptions are always in i0
   assign i0_iside_trigger_has_pri_e4[3:0] = ~( (trigger_execute[3:0] & trigger_data[3:0] & {4{inst_acc_e4_raw}}) | // exe-data with inst_acc
                                                ({4{exu_i0_br_error_e4 | exu_i0_br_start_error_e4}}));              // branch error in i0

   assign i1_iside_trigger_has_pri_e4[3:0] = ~( ({4{exu_i1_br_error_e4 | exu_i1_br_start_error_e4}}) ); // branch error in i1

   // lsu excs have to line up with their respective triggers since the lsu op can be in either i0 or i1 but not both
   assign i0_lsu_trigger_has_pri_e4[3:0] = ~(trigger_store[3:0] & trigger_data[3:0] & {4{lsu_i0_exc_dc4_raw}});
   assign i1_lsu_trigger_has_pri_e4[3:0] = ~(trigger_store[3:0] & trigger_data[3:0] & {4{lsu_i1_exc_dc4_raw}});

   // Qual trigger hits
   assign i0_trigger_eval_e4 = dec_tlu_i0_valid_e4 | ( dec_i0_load_e4 & lsu_freeze_pulse_e4);
   assign i1_trigger_eval_e4 = dec_tlu_i1_valid_e4 | (~dec_i0_load_e4 & lsu_freeze_pulse_e4);

   assign i0_trigger_e4[3:0] = {4{i0_trigger_eval_e4}} & dec_tlu_packet_e4.i0trigger[3:0] & i0_iside_trigger_has_pri_e4[3:0] & i0_lsu_trigger_has_pri_e4[3:0] & trigger_enabled[3:0];
   assign i1_trigger_e4[3:0] = {4{i1_trigger_eval_e4}} & dec_tlu_packet_e4.i1trigger[3:0] & i1_iside_trigger_has_pri_e4[3:0] & i1_lsu_trigger_has_pri_e4[3:0] & trigger_enabled[3:0];

   assign trigger_chain[2:0] = {mtdata1_t2[`MTDATA1_CHAIN], mtdata1_t1[`MTDATA1_CHAIN], mtdata1_t0[`MTDATA1_CHAIN]};

   // chaining can mask raw trigger info
   assign i0_trigger_chain_masked_e4[3:0] = {i0_trigger_e4[3] & (~trigger_chain[2] | i0_trigger_e4[2]),
                                             i0_trigger_e4[2] & (~trigger_chain[2] | i0_trigger_e4[3]),
                                             i0_trigger_e4[1] & (~trigger_chain[0] | i0_trigger_e4[0]),
                                             i0_trigger_e4[0] & (~trigger_chain[0] | i0_trigger_e4[1])}; 

   assign i1_trigger_chain_masked_e4[3:0] = {i1_trigger_e4[3] & (~trigger_chain[2] | i1_trigger_e4[2]),
                                             i1_trigger_e4[2] & (~trigger_chain[2] | i1_trigger_e4[3]),
                                             i1_trigger_e4[1] & (~trigger_chain[0] | i1_trigger_e4[0]),
                                             i1_trigger_e4[0] & (~trigger_chain[0] | i1_trigger_e4[1])}; 

   // This is the highest priority by this point.
   assign i0_trigger_hit_raw_e4 = |i0_trigger_chain_masked_e4[3:0];
   assign i1_trigger_hit_raw_e4 = |i1_trigger_chain_masked_e4[3:0];

   // Qual trigger hits
   assign i0_trigger_hit_e4 = ~(dec_tlu_flush_lower_wb | dec_tlu_dbg_halted | lsu_freeze_pulse_e4) & i0_trigger_hit_raw_e4;
   assign i1_trigger_hit_e4 = ~(dec_tlu_flush_lower_wb | ~tlu_i0_commit_cmt | exu_i0_br_mp_e4 | dec_tlu_dbg_halted | lsu_freeze_pulse_e4 | lsu_i0_rfnpc_dc4) & i1_trigger_hit_raw_e4;

   assign dec_tlu_cancel_e4 = (i0_trigger_hit_raw_e4 | i1_trigger_hit_raw_e4) & lsu_freeze_pulse_e4;
   
   // Actions include breakpoint, or dmode. Dmode is only possible if the DMODE bit is set.
   // Otherwise, take a breakpoint.
   assign trigger_action[3:0] = {mtdata1_t3[`MTDATA1_ACTION] & mtdata1_t3[`MTDATA1_DMODE], 
                                 mtdata1_t2[`MTDATA1_ACTION] & mtdata1_t2[`MTDATA1_DMODE], 
                                 mtdata1_t1[`MTDATA1_ACTION] & mtdata1_t1[`MTDATA1_DMODE], 
                                 mtdata1_t0[`MTDATA1_ACTION] & mtdata1_t0[`MTDATA1_DMODE]};

   // this is needed to set the HIT bit in the triggers
   assign update_hit_bit_e4[3:0] = ({4{i0_trigger_hit_e4                     }} & i0_trigger_chain_masked_e4[3:0]) | 
                                   ({4{i1_trigger_hit_e4 & ~i0_trigger_hit_e4}} & i1_trigger_chain_masked_e4[3:0]);

   // action, 1 means dmode. Simultaneous triggers with at least 1 set for dmode force entire action to dmode.
   assign i0_trigger_action_e4 = |(i0_trigger_chain_masked_e4[3:0] & trigger_action[3:0]);
   assign i1_trigger_action_e4 = |(i1_trigger_chain_masked_e4[3:0] & trigger_action[3:0]);

   assign trigger_hit_e4 = i0_trigger_hit_e4 | i1_trigger_hit_e4;
   assign trigger_hit_dmode_e4 = (i0_trigger_hit_e4 & i0_trigger_action_e4) | (i1_trigger_hit_e4 & ~i0_trigger_hit_e4 & i1_trigger_action_e4);
   
   assign mepc_trigger_hit_sel_pc_e4 = trigger_hit_e4 & ~trigger_hit_dmode_e4;
   
   
//   
// Debug end
//--------------------------------------------------------------------------------
   
   //----------------------------------------------------------------------
   //
   // Commit
   //
   //----------------------------------------------------------------------



   //--------------------------------------------------------------------------------
   // External halt (not debug halt) 
   // - Fully interlocked handshake
   // i_cpu_halt_req  ____|--------------|_______________
   // core_empty      ---------------|___________
   // o_cpu_halt_ack  _________________|----|__________
   // o_cpu_halt_status _______________|---------------------|_________
   // i_cpu_run_req                              ______|----------|____
   // o_cpu_run_ack                              ____________|------|________
   //


   // debug mode has priority, ignore PMU/FW halt/run while in debug mode
   assign i_cpu_halt_req_sync_qual = i_cpu_halt_req_sync & ~dec_tlu_debug_mode;
   assign i_cpu_run_req_sync_qual = i_cpu_run_req_sync & ~dec_tlu_debug_mode & pmu_fw_tlu_halted_f;
   
   rvdff #(8) exthaltff (.*, .clk(free_clk), .din({i_cpu_halt_req_sync_qual, i_cpu_run_req_sync_qual,   cpu_halt_status,   
                                                   cpu_halt_ack,   cpu_run_ack, internal_pmu_fw_halt_mode,
                                                   pmu_fw_halt_req_ns, pmu_fw_tlu_halted}),  
                                            .dout({i_cpu_halt_req_d1,        i_cpu_run_req_d1_raw,      o_cpu_halt_status, 
                                                   o_cpu_halt_ack, o_cpu_run_ack, internal_pmu_fw_halt_mode_f,
                                                   pmu_fw_halt_req_f, pmu_fw_tlu_halted_f}));

   // only happens if we aren't in dgb_halt
   assign ext_halt_pulse = i_cpu_halt_req_sync_qual & ~i_cpu_halt_req_d1;
 
   assign enter_pmu_fw_halt_req =  ext_halt_pulse | fw_halt_req;

   assign pmu_fw_halt_req_ns = (enter_pmu_fw_halt_req | (pmu_fw_halt_req_f & ~pmu_fw_tlu_halted)) & ~debug_halt_req_f;

   assign internal_pmu_fw_halt_mode = pmu_fw_halt_req_ns | (internal_pmu_fw_halt_mode_f & ~i_cpu_run_req_d1 & ~debug_halt_req_f);

   // debug halt has priority
   assign pmu_fw_tlu_halted = ((pmu_fw_halt_req_f & core_empty & halt_taken & ~enter_debug_halt_req) | (pmu_fw_tlu_halted_f & ~i_cpu_run_req_d1)) & ~debug_halt_req_f;

   assign cpu_halt_ack = i_cpu_halt_req_d1 & pmu_fw_tlu_halted_f;
   assign cpu_halt_status = (pmu_fw_tlu_halted_f & ~i_cpu_run_req_d1) | (o_cpu_halt_status & ~i_cpu_run_req_d1 & ~internal_dbg_halt_mode_f);
   assign cpu_run_ack = (o_cpu_halt_status & i_cpu_run_req_sync_qual) | (o_cpu_run_ack & i_cpu_run_req_sync_qual);
   assign debug_mode_status = internal_dbg_halt_mode_f;
   assign o_debug_mode_status = debug_mode_status;// & ~mpc_debug_run_ack_f;

`ifdef ASSERT_ON  
  assert_commit_while_halted: assert #0 (~((tlu_i0_commit_cmt | tlu_i1_commit_cmt) & o_cpu_halt_status)) else $display("ERROR: Commiting while cpu_halt_status asserted!");
`endif
   
   // high priority interrupts can wakeup from external halt, so can unmasked timer interrupts
   assign i_cpu_run_req_d1 = i_cpu_run_req_d1_raw | ((nmi_int_detected | mtimer_int_ready | (mhwakeup & mhwakeup_ready)) & o_cpu_halt_status);

   //--------------------------------------------------------------------------------
   //--------------------------------------------------------------------------------
   
   // LSU exceptions (LSU responsible for prioritizing simultaneous cases)

   rvdffe #( $bits(lsu_error_pkt_t) ) lsu_error_dc4ff (.*, .en(/*lsu_error_pkt_dc3.exc_valid | lsu_error_pkt_dc4.exc_valid | lsu_error_pkt_dc3.single_ecc_error | lsu_error_pkt_dc4.single_ecc_error | clk_override*/~freeze_dc4), .clk(clk), .din(lsu_error_pkt_dc3),  .dout(lsu_error_pkt_dc4));

   logic lsu_single_ecc_error_wb_ns;
   assign lsu_single_ecc_error_wb_ns = lsu_error_pkt_dc4.single_ecc_error;// & ((~lsu_error_pkt_dc4.inst_pipe & tlu_i0_commit_cmt) | (lsu_error_pkt_dc4.inst_pipe & tlu_i1_commit_cmt));
   rvdff #(2) lsu_dccm_errorff (.*, .clk(free_clk), .din({mdseac_locked_ns, lsu_single_ecc_error_wb_ns}), .dout({mdseac_locked_f, lsu_single_ecc_error_wb}));
   
   logic [63:0] lsu_error_pkt_addr_dc4, lsu_error_pkt_addr_wb;
   assign lsu_error_pkt_addr_dc4[63:0] = lsu_error_pkt_dc4.addr[63:0];
   rvdffsc #(66) lsu_error_wbff (.*, .en(/*lsu_error_pkt_dc4.exc_valid | lsu_exc_valid_wb | clk_override*/1'b1), .clear(freeze_dc4), .clk(clk), .din({lsu_error_pkt_addr_dc4[63:0], lsu_exc_valid_e4, lsu_i0_exc_dc4}),  .dout({lsu_error_pkt_addr_wb[63:0], lsu_exc_valid_wb, lsu_i0_exc_wb}));

   logic [63:0] vec_tval_wb;
   rvdffsc #(66) vec_ls_error_wbff (.*, .en(1'b1), .clear(freeze_dc4), .clk(clk), .din({vec_tval_dc4_i[63:0], vec_ls_exc_valid_e4, vec_illegal_instr_dc4_i}),  .dout({vec_tval_wb[63:0], vec_ls_exc_valid_wb, vec_illegal_instr_wb}));


   // lsu exception is valid unless it's in pipe1 and there was a rfpc_i0_e4, brmp, or an iside exception in pipe0.
   assign lsu_exc_valid_e4_raw = lsu_error_pkt_dc4.exc_valid & ~(lsu_error_pkt_dc4.inst_pipe & (rfpc_i0_e4 | i0_exception_valid_e4 | exu_i0_br_mp_e4)) & ~dec_tlu_flush_lower_wb;
   
   assign lsu_i0_exc_dc4_raw =  lsu_error_pkt_dc4.exc_valid & ~lsu_error_pkt_dc4.inst_pipe;
   assign lsu_i1_exc_dc4_raw = lsu_error_pkt_dc4.exc_valid &  lsu_error_pkt_dc4.inst_pipe;
   assign lsu_i0_exc_dc4 = lsu_i0_exc_dc4_raw & lsu_exc_valid_e4_raw & ~i0_trigger_hit_e4;
   assign lsu_i1_exc_dc4 = lsu_i1_exc_dc4_raw & lsu_exc_valid_e4_raw & ~trigger_hit_e4;
   assign lsu_exc_valid_e4 = lsu_i0_exc_dc4 | lsu_i1_exc_dc4;
   
   assign lsu_exc_ma_dc4 = (lsu_i0_exc_dc4 | lsu_i1_exc_dc4) & ~lsu_error_pkt_dc4.exc_type;
   assign lsu_exc_acc_dc4 = (lsu_i0_exc_dc4 | lsu_i1_exc_dc4) & lsu_error_pkt_dc4.exc_type;
   assign lsu_exc_st_dc4 = (lsu_i0_exc_dc4 | lsu_i1_exc_dc4) & lsu_error_pkt_dc4.inst_type;
   
   assign load_page_fault_e4 = ((lsu_i0_exc_dc4 | lsu_i1_exc_dc4) & lsu_error_pkt_dc4.load_page_fault) | vec_load_page_fault_dc4_i;
   assign store_page_fault_e4 = ((lsu_i0_exc_dc4 | lsu_i1_exc_dc4) & lsu_error_pkt_dc4.store_page_fault) | vec_store_page_fault_dc4_i;
   assign vec_ls_exc_valid_e4 = vec_load_page_fault_dc4_i | vec_store_page_fault_dc4_i;

   // Single bit ECC errors on loads are RFNPC corrected, with the corrected data written to the GPR. 
   // LSU turns the load into a store and patches the data in the DCCM
   assign lsu_i0_rfnpc_dc4 = dec_tlu_i0_valid_e4 & ~lsu_error_pkt_dc4.inst_pipe & ~lsu_error_pkt_dc4.inst_type & 
                             lsu_error_pkt_dc4.single_ecc_error & ~lsu_error_pkt_dc4.dma_valid & ~i0_trigger_hit_e4;
   assign lsu_i1_rfnpc_dc4 = dec_tlu_i1_valid_e4 &  lsu_error_pkt_dc4.inst_pipe & ~lsu_error_pkt_dc4.inst_type & 
                             lsu_error_pkt_dc4.single_ecc_error & ~lsu_error_pkt_dc4.dma_valid & ~i0_trigger_hit_e4 & ~i1_trigger_hit_e4;

   // Branch prediction updating
   assign dec_tlu_br0_addr_e4[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO] = exu_i0_br_index_e4[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO];
   assign dec_tlu_br0_bank_e4[1:0] = exu_i0_br_bank_e4[1:0];
   assign dec_tlu_br1_addr_e4[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO] = exu_i1_br_index_e4[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO];
   assign dec_tlu_br1_bank_e4[1:0] = exu_i1_br_bank_e4[1:0];


   //  Final commit valids
   assign tlu_i0_commit_cmt = dec_tlu_i0_valid_e4 &
                              ~rfpc_i0_e4 & 
                              ~lsu_i0_exc_dc4 & 
                              ~vec_ls_exc_valid_e4 &
                              ~vec_illegal_instr_dc4_i &
                              ~inst_acc_e4 & 
                              ~dec_tlu_dbg_halted & 
                              ~request_debug_mode_wb &
                              ~i0_trigger_hit_e4;
   
   assign tlu_i1_commit_cmt = dec_tlu_i1_valid_e4 & 
                              ~rfpc_i0_e4 & ~rfpc_i1_e4 & 
                              ~exu_i0_br_mp_e4 & 
                              ~lsu_i0_exc_dc4 & ~lsu_i1_exc_dc4 &
                              ~vec_ls_exc_valid_e4 &
                              ~vec_illegal_instr_dc4_i &
                              ~lsu_i0_rfnpc_dc4 & 
                              ~inst_acc_e4 &
                              ~request_debug_mode_wb &
                              ~trigger_hit_e4;
                              
   // unified place to manage the killing of arch state writebacks
   assign tlu_i0_kill_writeb_e4 = rfpc_i0_e4 | lsu_i0_exc_dc4 | vec_ls_exc_valid_e4 | inst_acc_e4 | (illegal_e4 & dec_tlu_dbg_halted) | i0_trigger_hit_e4 | fetch_page_fault_e4;
   assign tlu_i1_kill_writeb_e4 = rfpc_i0_e4 | rfpc_i1_e4 | lsu_exc_valid_e4 | vec_ls_exc_valid_e4 | exu_i0_br_mp_e4 | inst_acc_e4 | (illegal_e4 & dec_tlu_dbg_halted) | trigger_hit_e4 | lsu_i0_rfnpc_dc4 | fetch_page_fault_e4;

   // refetch PC, microarch flush
   // ic errors only in pipe0
   assign rfpc_i0_e4 = dec_tlu_i0_valid_e4 & ~tlu_flush_lower_wb & (exu_i0_br_error_e4 | exu_i0_br_start_error_e4 | ic_perr_e4 | iccm_sbecc_e4) & ~i0_trigger_hit_e4;
   assign rfpc_i1_e4 = dec_tlu_i1_valid_e4 & ~tlu_flush_lower_wb & ~i0_exception_valid_e4 & ~exu_i0_br_mp_e4 & ~lsu_i0_exc_dc4 & ~vec_ls_exc_valid_e4 & ~vec_illegal_instr_dc4_i & ~lsu_i0_rfnpc_dc4 & 
                       ~(exu_i0_br_error_e4 | exu_i0_br_start_error_e4 | ic_perr_e4 | iccm_sbecc_e4) & 
                       (exu_i1_br_error_e4 | exu_i1_br_start_error_e4) &
                       ~trigger_hit_e4;

   // go ahead and repair the branch error on other flushes, doesn't have to be the rfpc flush
   assign dec_tlu_br0_error_e4 = exu_i0_br_error_e4 & dec_tlu_i0_valid_e4 & ~tlu_flush_lower_wb;
   assign dec_tlu_br0_start_error_e4 = exu_i0_br_start_error_e4 & dec_tlu_i0_valid_e4 & ~tlu_flush_lower_wb;
   assign dec_tlu_br0_v_e4 = exu_i0_br_valid_e4 & dec_tlu_i0_valid_e4 & ~tlu_flush_lower_wb & ~exu_i0_br_mp_e4;

   assign dec_tlu_br1_error_e4 = exu_i1_br_error_e4 & dec_tlu_i1_valid_e4 & ~tlu_flush_lower_wb & ~exu_i0_br_mp_e4;
   assign dec_tlu_br1_start_error_e4 = exu_i1_br_start_error_e4 & dec_tlu_i1_valid_e4 & ~tlu_flush_lower_wb & ~exu_i0_br_mp_e4;
   assign dec_tlu_br1_v_e4 = exu_i1_br_valid_e4 & ~tlu_flush_lower_wb & dec_tlu_i1_valid_e4 & ~exu_i0_br_mp_e4 & ~exu_i1_br_mp_e4;
            
`ifdef RV_BTB_48
   rvdffsc #(20)  
`else
   rvdffsc #(18)  
`endif
   bp_wb_ff (.*, .en(/*e4e5_valid | clk_override*/1'b1), .clear(freeze_dc4), .clk(clk),
                            .din({exu_i0_br_hist_e4[1:0], 
                                  dec_tlu_br0_error_e4,
                                  dec_tlu_br0_start_error_e4,
                                  dec_tlu_br0_v_e4,
                                  exu_i1_br_hist_e4[1:0], 
                                  dec_tlu_br1_error_e4,
                                  dec_tlu_br1_start_error_e4,
                                  dec_tlu_br1_v_e4,
                                  dec_tlu_br0_bank_e4[1:0],
                                  dec_tlu_br1_bank_e4[1:0],
                                  exu_i0_br_way_e4,
                                  exu_i1_br_way_e4,
                                  exu_i0_br_middle_e4,
                                  exu_i1_br_middle_e4
                                  }), 
                           .dout({dec_tlu_br0_wb_pkt.hist[1:0], 
                                  dec_tlu_br0_wb_pkt.br_error, 
                                  dec_tlu_br0_wb_pkt.br_start_error, 
                                  dec_tlu_br0_wb_pkt.valid,
                                  dec_tlu_br1_wb_pkt.hist[1:0], 
                                  dec_tlu_br1_wb_pkt.br_error, 
                                  dec_tlu_br1_wb_pkt.br_start_error, 
                                  dec_tlu_br1_wb_pkt.valid,
                                  dec_tlu_br0_wb_pkt.bank[1:0],
                                  dec_tlu_br1_wb_pkt.bank[1:0],
                                  dec_tlu_br0_wb_pkt.way,
                                  dec_tlu_br1_wb_pkt.way,
                                  dec_tlu_br0_wb_pkt.middle,
                                  dec_tlu_br1_wb_pkt.middle
                                  }));

   rvdffsc #(`RV_BHT_GHR_SIZE*2)  bp_wb_ghrff (.*, .en(/*e4e5_valid | clk_override*/1'b1), .clear(freeze_dc4), .clk(clk), 
                                               .din({exu_i0_br_fghr_e4[`RV_BHT_GHR_RANGE],
                                                     exu_i1_br_fghr_e4[`RV_BHT_GHR_RANGE]
                                                     }), 
                                              .dout({dec_tlu_br0_wb_pkt.fghr[`RV_BHT_GHR_RANGE],
                                                     dec_tlu_br1_wb_pkt.fghr[`RV_BHT_GHR_RANGE]
                                                     }));

   rvdffsc #(2*$bits(dec_tlu_br0_addr_e4[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO]))  
        bp_wb_index_ff (.*, .en(/*e4e5_valid | clk_override*/1'b1), .clear(freeze_dc4), .clk(clk),
                            .din({dec_tlu_br0_addr_e4[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO],
                                  dec_tlu_br1_addr_e4[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO]}), 
                           .dout({dec_tlu_br0_wb_pkt.index[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO],
                                  dec_tlu_br1_wb_pkt.index[`RV_BTB_ADDR_HI:`RV_BTB_ADDR_LO]}));

   // only expect these in pipe 0
   assign       ebreak_to_debug_en = (priv_lvl_o == PRIV_LVL_M & dcsr.ebreakm) | (priv_lvl_o == PRIV_LVL_S & dcsr.ebreaks) | (priv_lvl_o == PRIV_LVL_U & dcsr.ebreaku);
   assign       ebreak_e4    =  (dec_tlu_packet_e4.pmu_i0_itype == EBREAK)  & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4 & ~ebreak_to_debug_en & ~debug_mode;
   assign       ecall_from_m_mode_e4 = ecall_e4 & (priv_lvl_o == PRIV_LVL_M);
   assign       ecall_from_s_mode_e4 = ecall_e4 & (priv_lvl_o == PRIV_LVL_S);
   assign       ecall_from_u_mode_e4 = ecall_e4 & (priv_lvl_o == PRIV_LVL_U);
   assign       ecall_e4     =  (dec_tlu_packet_e4.pmu_i0_itype == ECALL)   & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4;
   assign       illegal_e4   =  (~dec_tlu_packet_e4.legal | vec_illegal_instr_dc4_i) & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4;
   assign       mret_e4      =  (dec_tlu_packet_e4.pmu_i0_itype == MRET)    & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4;
   assign       sret_e4      =  (dec_tlu_packet_e4.pmu_i0_itype == SRET)    & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4;
   assign       dret_e4      =  (dec_tlu_packet_e4.pmu_i0_itype == DRET) & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4;
   // fence_i includes debug only fence_i's
   assign       fence_i_e4   =  (dec_tlu_packet_e4.fence_i & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4); //| csr_fence_i_wb;
   assign       sfence_vma_e4   =  (dec_tlu_packet_e4.sfence_vma & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4); //| csr_fence_i_wb;
   assign       ic_perr_e4    =  dec_tlu_packet_e4.perr    & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4;
   assign       iccm_sbecc_e4 =  dec_tlu_packet_e4.sbecc   & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4;
   assign       fetch_page_fault_e4 =  dec_tlu_packet_e4.fetch_page_fault & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4;
   assign       inst_acc_e4_raw  =  dec_tlu_packet_e4.icaf & dec_tlu_i0_valid_e4;
   assign       inst_acc_e4 = inst_acc_e4_raw & ~rfpc_i0_e4 & ~i0_trigger_hit_e4;
   assign       inst_acc_second_e4 = dec_tlu_packet_e4.icaf_f1;
   
   assign       ebreak_to_debug_mode_e4 = (dec_tlu_packet_e4.pmu_i0_itype == EBREAK)  & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4 & (ebreak_to_debug_en | debug_mode);

   assign illegal_e4_qual = illegal_e4 & ~dec_tlu_dbg_halted;

   rvdffsc #(15)  exctype_wb_ff (.*, .en(/*e4e5_valid | clk_override*/1'b1), .clear(freeze_dc4), .clk(clk), 
                                .din({ic_perr_e4, iccm_sbecc_e4, fetch_page_fault_e4, ebreak_e4, ebreak_to_debug_mode_e4, ecall_e4, illegal_e4,
                                 illegal_e4_qual,  inst_acc_e4, inst_acc_second_e4, fence_i_e4, sfence_vma_e4, mret_e4, sret_e4, dret_e4}), 
                               .dout({ic_perr_wb, iccm_sbecc_wb, fetch_page_fault_wb, ebreak_wb, ebreak_to_debug_mode_wb, ecall_wb, illegal_raw_wb,
                                 illegal_wb, inst_acc_wb, inst_acc_second_wb, fence_i_wb, sfence_vma_wb, mret_wb, sret_wb, dret_wb}));

   // icache and tlb flush wait for finishing dcache flush
   // assign dec_tlu_flush_icache_e4 = fence_i_e4;
   // assign dec_tlu_flush_tlb_e4 = sfence_vma_e4;
   //
   // Exceptions
   //
   // - MEPC <- PC
   // - PC <- MTVEC, assert flush_lower
   // - MCAUSE <- cause
   // - MTVAL <-
   // - MPIE <- MIE
   // - MIE <- 0
   //
   assign i0_exception_valid_e4 = (ebreak_e4 | ecall_e4 | illegal_e4 | inst_acc_e4 | fetch_page_fault_e4) & ~rfpc_i0_e4 & ~dec_tlu_dbg_halted;

   // Cause:
   // 
   // 0x2 : illegal
   // 0x3 : breakpoint
   // 0xb : Environment call M-mode

   // 20220321: illegal exception as page fault exception
   assign exc_cause_e4[4:0] = ( ({5{take_mext_int}}                     & 5'h0b) | // Machine ext int
                                ({5{take_msoft_int}}                    & 5'h03) | // Machine software int
                                ({5{take_mtimer_int}}                   & 5'h07) | // Machine timer int
                                ({5{take_mce_int}}                      & 5'h1e) |
                                ({5{take_sext_int}}                     & 5'h09) | // Supervisor ext int
                                ({5{take_ssoft_int}}                    & 5'h01) | // Supervisor software int
                                ({5{take_stimer_int}}                   & 5'h05) | // Supervisor timer int
                                ({5{inst_acc_e4}}                       & 5'h01) | // instruction access fault
                                ({5{illegal_e4}}                        & 5'h02) | // illegal instruction
                                ({5{ebreak_e4 | trigger_hit_e4}}        & 5'h03) | // breakpoint
                                ({5{lsu_exc_ma_dc4 & ~lsu_exc_st_dc4}}  & 5'h04) | // load address misaligned
                                ({5{lsu_exc_acc_dc4 & ~lsu_exc_st_dc4}} & 5'h05) | // load access fault
                                ({5{lsu_exc_ma_dc4 & lsu_exc_st_dc4}}   & 5'h06) | // store/amo address misaligned
                                ({5{lsu_exc_acc_dc4 & lsu_exc_st_dc4}}  & 5'h07) | // store/amo access fault
                                ({5{ecall_from_u_mode_e4}}              & 5'h08) | // ecall from u mode
                                ({5{ecall_from_s_mode_e4}}              & 5'h09) | // ecall from s mode
                                ({5{ecall_from_m_mode_e4}}              & 5'h0b) | // ecall from m mode
                                ({5{fetch_page_fault_e4}}               & 5'h0c) | // instr_page_fault
                                ({5{load_page_fault_e4}}                & 5'h0d) | // load page fault, e4
                                ({5{store_page_fault_e4}}               & 5'h0f)   // store/amo page fault, e4
                                ) & ~{5{take_nmi}};

   //
   // Interrupts
   //
   // Priv spec 1.10, 3.1.14 
   // "Multiple simultaneous interrupts and traps at the same privilege level are handled in the following
   // decreasing priority order: external interrupts, software interrupts, timer interrupts, then finally any
   // synchronous traps."
   //
   // For above purposes, exceptions that are committed have already happened and will cause an int at E4 to wait a cycle 
   // or more if MSTATUS[MIE] is cleared.
   // 
   // -in priority order, highest to lowest
   // -single cycle window where a csr write to MIE/MSTATUS is at E4 when the other conditions for externals are met.
   //  Hold off externals for a cycle to make sure we are consistent with what was just written
   assign mhwakeup_ready =  ~dec_csr_stall_int_ff & global_interrupt_enable & mip[IRQ_M_EXT]   & mie_n[IRQ_M_EXT];
   assign mext_int_ready   = ~dec_csr_stall_int_ff & global_interrupt_enable & mip[IRQ_M_EXT]   & mie_n[IRQ_M_EXT];
   assign mce_int_ready    = ~dec_csr_stall_int_ff & global_interrupt_enable & mip[30]  & mie_n[30];
   assign mtimer_int_ready = ~dec_csr_stall_int_ff & global_interrupt_enable & mip[IRQ_M_TIMER]   & mie_n[IRQ_M_TIMER];
   assign msoftware_int_ready = ~dec_csr_stall_int_ff & global_interrupt_enable & mip[IRQ_M_SOFT]   & mie_n[IRQ_M_SOFT];

   // for S mode interrupt request: if hart in M mode it will not handle S mode irq if the bit in mideleg is set
   assign sext_int_ready = ~dec_csr_stall_int_ff & global_interrupt_enable & mip[IRQ_S_EXT] & mie_n[IRQ_S_EXT] & (~mideleg[IRQ_S_EXT] | privilege_level != PRIV_LVL_M);
   assign stimer_int_ready = ~dec_csr_stall_int_ff & global_interrupt_enable & mip[IRQ_S_TIMER] & mie_n[IRQ_S_TIMER] & (~mideleg[IRQ_S_TIMER] | privilege_level != PRIV_LVL_M);
   assign ssoftware_int_ready = ~dec_csr_stall_int_ff & global_interrupt_enable & mip[IRQ_S_SOFT] & mie_n[IRQ_S_SOFT] & (~mideleg[IRQ_S_SOFT] | privilege_level != PRIV_LVL_M);

   // mispredicts
   assign i0_mp_e4 = exu_i0_flush_lower_e4 & ~i0_trigger_hit_e4;
   assign i1_mp_e4 = exu_i1_flush_lower_e4 & ~trigger_hit_e4 & ~lsu_i0_rfnpc_dc4;
   
   assign internal_dbg_halt_timers = internal_dbg_halt_mode_f & ~dcsr_single_step_running;

   // Prioritize externals
   assign block_interrupts = ( (lsu_block_interrupts_i & ~dec_tlu_flush_lower_wb) | // I/O transaction on the bus pending
                               (internal_dbg_halt_mode & (~dcsr_single_step_running | dec_tlu_i0_valid_e4)) | // No ints in db-halt unless we are single stepping
                               internal_pmu_fw_halt_mode | i_cpu_halt_req_d1 |// No ints in PMU/FW halt. First we exit halt
                               take_nmi | // NMI is top priority
                               dmi_debug_req_valid_e4 | dmi_debug_req_valid_e5 |// Heading to debug mode, hold off ints
                               synchronous_flush_e4 | // exception flush this cycle
                               exc_or_int_valid_wb | // ext/int past cycle (need time for MIE to update)
                               mret_wb | // xret (need time for MIE to update)
                               sret_wb |
                               dret_wb
                               );
   
   assign take_mext_int = mext_int_ready & ~block_interrupts;
   assign take_mce_int  = mce_int_ready & ~mext_int_ready & ~block_interrupts;
   assign take_msoft_int = msoftware_int_ready & ~mext_int_ready & ~mce_int_ready & ~block_interrupts;
   assign take_mtimer_int = mtimer_int_ready & ~mext_int_ready & ~mce_int_ready & ~msoftware_int_ready & ~block_interrupts;

   assign take_sext_int = sext_int_ready & ~mext_int_ready & ~msoftware_int_ready & ~mtimer_int_ready & ~block_interrupts;
   assign take_ssoft_int = ssoftware_int_ready & ~mext_int_ready & ~msoftware_int_ready & ~mtimer_int_ready & ~sext_int_ready & ~block_interrupts;
   assign take_stimer_int = stimer_int_ready & ~mext_int_ready & ~msoftware_int_ready & ~mtimer_int_ready & ~sext_int_ready & ~ssoftware_int_ready & ~block_interrupts;
   
   assign take_reset = reset_delayed & mpc_reset_run_req;
   assign take_nmi = nmi_int_detected & ~internal_pmu_fw_halt_mode & (~internal_dbg_halt_mode | (dcsr_single_step_running_f & dcsr[`DCSR_STEPIE] & ~dec_tlu_i0_valid_e4 & ~dcsr_single_step_done_f)) & ~synchronous_flush_e4 & ~take_reset;

   assign interrupt_valid = take_mext_int | take_mtimer_int | take_nmi | take_mce_int | take_msoft_int | take_sext_int | take_ssoft_int | take_stimer_int;
   

   // Compute interrupt path:
   // If vectored async is set in mtvec, flush path for interrupts is MTVEC + (4 * CAUSE);
   // When support S-mode, first decide trap into mode
   assign trap_into_priv_lvl_e4 = (priv_lvl_o == PRIV_LVL_M)? PRIV_LVL_M :
                               ((interrupt_valid && mideleg[exc_cause_e4[4:0]]) || (!interrupt_valid && medeleg[exc_cause_e4[4:0]]))? PRIV_LVL_S :
                               PRIV_LVL_M; 
   assign trap_vec[63:0] = (trap_into_priv_lvl_e4 == PRIV_LVL_S)? stvec[63:0] : mtvec[63:0];
   assign vectored_cause[5:0] = ({1'b0, exc_cause_e4[4:0]} << 1);
   assign {vpath_overflow_nc, vectored_path[63:1]} = {trap_vec[63:2],1'b0} + {57'b0, vectored_cause[5:0]};
   assign interrupt_path[63:1] = take_nmi ? nmi_vec[63:1] : ((trap_vec[0] == 1'b1) ? vectored_path[63:1] : {trap_vec[63:2], 1'b0});

   assign sel_npc_e4 = lsu_i0_rfnpc_dc4 | (lsu_i1_rfnpc_dc4 & tlu_i1_commit_cmt) | fence_i_e4 | sfence_vma_e4 | (i_cpu_run_req_d1 & ~interrupt_valid) | write_csr_flush_e4;
   assign sel_npc_wb = (i_cpu_run_req_d1 & pmu_fw_tlu_halted_f) | pause_expired_e4;

   
   assign synchronous_flush_e4 = i0_exception_valid_e4 | // exception
                                 i0_mp_e4 | i1_mp_e4 |  // mispredict
                                 rfpc_i0_e4 | rfpc_i1_e4 | // rfpc
                                 lsu_exc_valid_e4 |  // lsu exception in either pipe 0 or pipe 1
                                 vec_ls_exc_valid_e4 | // vec load/store exception
                                 fence_i_e4 | sfence_vma_e4 | // fence, a rfnpc
                                 lsu_i0_rfnpc_dc4 | lsu_i1_rfnpc_dc4 |
                                 debug_resume_req_f | // resume from debug halt, fetch the dpc
                                 sel_npc_wb |  // resume from pmu/fw halt, or from pause and fetch the NPC
                                 dec_tlu_wr_pause_wb | // flush at start of pause
                                 trigger_hit_e4 | // trigger hit, ebreak or goto debug mode
                                 mret_e4 | sret_e4 | dret_e4 | // trap return
                                 ebreak_e4 | ebreak_to_debug_mode_e4 | // ebreak instrcution
                                 dcsr_single_step_done | // single step debug
                                 write_csr_flush_e4; // write some csrs will flush pipe

   assign tlu_flush_lower_e4 = interrupt_valid | synchronous_flush_e4 | take_halt | take_reset | dmi_debug_req_valid_e4;

   assign tlu_flush_path_e4[63:1] = take_reset ? rst_vec[63:1] :
                                    (dmi_debug_req_valid_e4 | ebreak_to_debug_mode_e4 | dcsr_single_step_done)? (DmBaseAddress[63:1] + dm::HaltAddress[63:1]) :
                                    ( ({63{~take_nmi & i0_mp_e4}} & exu_i0_flush_path_e4[63:1]) |
                                      ({63{~take_nmi & ~i0_mp_e4 & i1_mp_e4 & ~rfpc_i0_e4 & ~lsu_i0_exc_dc4 & ~vec_ls_exc_valid_e4 & ~vec_illegal_instr_dc4_i}} & exu_i1_flush_path_e4[63:1]) |
                                      ({63{~take_nmi & sel_npc_e4}} & npc_e4[63:1]) | 
                                      ({63{~take_nmi & rfpc_i0_e4}} & dec_tlu_i0_pc_e4[63:1]) | 
                                      ({63{~take_nmi & rfpc_i1_e4}} & dec_tlu_i1_pc_e4[63:1]) | 
                                      ({63{interrupt_valid}} & interrupt_path[63:1]) | 
                                      ({63{(i0_exception_valid_e4 | lsu_exc_valid_e4 | vec_ls_exc_valid_e4 | (trigger_hit_e4 & ~trigger_hit_dmode_e4)) & ~interrupt_valid & ~debug_mode}} & {trap_vec[63:2], 1'b0}) |
                                      ({63{(i0_exception_valid_e4 | lsu_exc_valid_e4 | vec_ls_exc_valid_e4) & debug_mode}} & (DmBaseAddress[63:1] + dm::ExceptionAddress[63:1])) |
                                      ({63{~take_nmi & mret_e4 & ~wr_mepc_wb}} & mepc[63:1]) |
                                      ({63{~take_nmi & sret_e4 & ~wr_sepc_wb}} & sepc[63:1]) |
                                      ({63{~take_nmi & (debug_resume_req_f | dret_e4)}} & dpc[63:1]) |
                                      ({63{~take_nmi & sel_npc_wb}} & npc_wb[63:1]) |
                                      ({63{~take_nmi & mret_e4 & wr_mepc_wb}} & dec_csr_wrdata_wb[63:1]) | 
                                      ({63{~take_nmi & sret_e4 & wr_sepc_wb}} & dec_csr_wrdata_wb[63:1]));

   rvdffsc #(63)  flush_lower_ff (.*, .en(/*e4e5_valid | internal_dbg_halt_mode_f | i_cpu_run_req_d1 | interrupt_valid | interrupt_valid_wb | reset_delayed | pause_expired_e4 | pause_expired_wb | clk_override | dmi_debug_req_posedge_f1 | lsu_exc_valid_e4 | lsu_exc_valid_wb*/1'b1), .clear(freeze_dc4), .clk(clk), 
                                  .din({tlu_flush_path_e4[63:1]}), 
                                 .dout({tlu_flush_path_wb[63:1]}));

   assign dec_tlu_flush_lower_wb = tlu_flush_lower_wb;
   assign dec_tlu_flush_path_wb[63:1] = tlu_flush_path_wb[63:1];

   
   // this is used to capture mepc, etc.
   assign exc_or_int_valid = lsu_exc_valid_e4 | vec_ls_exc_valid_e4 | i0_exception_valid_e4 | interrupt_valid | (trigger_hit_e4 & ~trigger_hit_dmode_e4);

   // assign lsu_block_interrupts_dc3 = lsu_freeze_external_ints_dc3 & ~dec_tlu_flush_lower_wb;
   assign lsu_block_interrupts_dc3 = lsu_block_interrupts_i;
   
   rvdffsc #(17)  excinfo_wb_ff (.*, .en(/*e4e5_valid | internal_dbg_halt_mode_f | i_cpu_run_req_d1 | interrupt_valid | interrupt_valid_wb | reset_delayed | pause_expired_e4 | pause_expired_wb | clk_override | dmi_debug_req_posedge_f1 | lsu_exc_valid_e4 | lsu_exc_valid_wb*/1'b1), .clear(freeze_dc4), .clk(clk), 
                                .din({interrupt_valid, i0_exception_valid_e4, exc_or_int_valid, 
                                      exc_cause_e4[4:0], tlu_i0_commit_cmt & ~illegal_e4, tlu_i1_commit_cmt, 
                                       mepc_trigger_hit_sel_pc_e4, trigger_hit_e4, i0_trigger_hit_e4,
                                      take_nmi, pause_expired_e4, trap_into_priv_lvl_e4[1:0]}), 
                               .dout({interrupt_valid_wb, i0_exception_valid_wb, exc_or_int_valid_wb, 
                                      exc_cause_wb[4:0], i0_valid_wb, i1_valid_wb, 
                                       mepc_trigger_hit_sel_pc_wb, trigger_hit_wb, i0_trigger_hit_wb,
                                      take_nmi_wb, pause_expired_wb, trap_into_priv_lvl_wb[1:0]}));

   //----------------------------------------------------------------------
   //
   // CSRs
   //
   //----------------------------------------------------------------------


   // ----------------------------------------------------------------------
   // MISA (RO)
   //  [31:30] XLEN - implementation width, 2'b01 - 32 bits
   //  [12]    M    - integer mul/div
   //  [8]     I    - RV32I
   //  [2]     C    - Compressed extension
   `define MISA 12'h301

   // MVENDORID, MARCHID, MIMPID, MHARTID
   `define MVENDORID 12'hf11
   `define MARCHID 12'hf12
   `define MIMPID 12'hf13
   `define MHARTID 12'hf14

   
   // ----------------------------------------------------------------------
   // MSTATUS (RW)
   // [12:11] MPP  : Prior priv level, always 2'b11, not flopped
   // [7]     MPIE : Int enable previous [1]
   // [3]     MIE  : Int enable          [0]
   // `define MSTATUS 12'h300


   //When executing a MRET instruction, supposing MPP holds the value 3, MIE
   //is set to MPIE; the privilege mode is changed to 3; MPIE is set to 1; and MPP is set to 3

   assign dec_csr_wen_wb_mod = dec_csr_wen_wb & ~trigger_hit_wb;
   // assign wr_mstatus_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MSTATUS);

   // assign mstatus_ns[1:0] = ( ({2{exc_or_int_valid_wb}} & {mstatus[`MSTATUS_MIE], 1'b0}) |
   //                            ({2{mret_wb & ~exc_or_int_valid_wb}} & {1'b1, mstatus[1]}) |
   //                            ({2{wr_mstatus_wb & ~exc_or_int_valid_wb}} & {dec_csr_wrdata_wb[7], dec_csr_wrdata_wb[3]}) |
   //                            ({2{~wr_mstatus_wb & ~exc_or_int_valid_wb & ~mret_wb}} & mstatus[1:0]) );

   // gate MIE if we are single stepping and DCSR[STEPIE] is off
   // assign mstatus_mie_ns = mstatus_ns[`MSTATUS_MIE] & (~dcsr_single_step_running_f | dcsr[`DCSR_STEPIE]);
   // assign mstatus_mie_ns = mstatus_n.mie & (~dcsr_single_step_running_f | dcsr[`DCSR_STEPIE]);
   assign global_interrupt_enable = ~debug_mode &
                                    (~dcsr_single_step_running_f | dcsr[`DCSR_STEPIE]) &
                                    ((mstatus_n.mie & privilege_level == PRIV_LVL_M) | (mstatus_n.sie & privilege_level == PRIV_LVL_S) | privilege_level == PRIV_LVL_U);

   // rvdff #(2)  mstatus_ff (.*, .clk(free_clk), .din(mstatus_ns[1:0]), .dout(mstatus[1:0]));

   //  `define SSTATUS 12'h100
   //  logic wr_sstatus_wb;
   // assign wr_sstatus_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `SSTATUS);

   // assign sstatus_ns[1:0] = ( ({2{exc_or_int_valid_wb}} & {sstatus[`MSTATUS_MIE], 1'b0}) |
   //                            ({2{mret_wb & ~exc_or_int_valid_wb}} & {1'b1, sstatus[1]}) |
   //                            ({2{wr_sstatus_wb & ~exc_or_int_valid_wb}} & {dec_csr_wrdata_wb[7], dec_csr_wrdata_wb[3]}) |
   //                            ({2{~wr_sstatus_wb & ~exc_or_int_valid_wb & ~mret_wb}} & sstatus[1:0]) );
   // rvdff #(2)  sstatus_ff (.*, .clk(free_clk), .din(sstatus_ns[1:0]), .dout(sstatus[1:0]));

   //  logic wr_medeleg_wb, wr_mideleg_wb;
   // `define MEDELEG 12'h302
   // assign wr_medeleg_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MEDELEG);
   // assign medeleg_ns[63:0] = dec_csr_wrdata_wb[63:0]; 
   // rvdffe #(64)  medeleg_ff (.*, .en(wr_medeleg_wb), .din(medeleg_ns[63:0]), .dout(medeleg[63:0]));

   // `define MIDELEG 12'h303
   // assign wr_mideleg_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MIDELEG);
   // assign mideleg_ns[63:0] = dec_csr_wrdata_wb[63:0]; 
   // rvdffe #(64)  mideleg_ff (.*, .en(wr_mideleg_wb), .din(mideleg_ns[63:0]), .dout(mideleg[63:0]));

    logic wr_dscratch0_wb, wr_dscratch1_wb;
   `define DSCRATCH0 12'h7b2
   assign wr_dscratch0_wb= dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `DSCRATCH0);
   assign dscratch0_ns[63:0] = dec_csr_wrdata_wb[63:0]; 
   rvdffe #(64)  dscratch0_ff (.*, .en(wr_dscratch0_wb), .din(dscratch0_ns[63:0]), .dout(dscratch0[63:0]));
   `define DSCRATCH1 12'h7b3
   assign wr_dscratch1_wb= dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `DSCRATCH1);
   assign dscratch1_ns[63:0] = dec_csr_wrdata_wb[63:0]; 
   rvdffe #(64)  dscratch1_ff (.*, .en(wr_dscratch1_wb), .din(dscratch1_ns[63:0]), .dout(dscratch1[63:0]));
   // ----------------------------------------------------------------------
   // MTVEC (RW)
   // [31:2] BASE : Trap vector base address
   // [1] - Reserved, not implemented, reads zero
   // [0]  MODE : 0 = Direct, 1 = Asyncs are vectored to BASE + (4 * CAUSE)
   // `define MTVEC 12'h305
   
   // assign wr_mtvec_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTVEC);
   // assign mtvec_ns[62:0] = {dec_csr_wrdata_wb[63:2], dec_csr_wrdata_wb[0]} ; 
   // rvdffe #(63)  mtvec_ff (.*, .en(wr_mtvec_wb), .din(mtvec_ns[62:0]), .dout(mtvec[62:0]));

   // ----------------------------------------------------------------------
   // MIP (RW)
   // 
   // [30] MCEIP  : (RO) M-Mode Correctable Error interrupt pending
   // [11] MEIP   : (RO) M-Mode external interrupt pending
   // [7]  MTIP   : (RO) M-Mode timer interrupt pending
   // [3]  MSIP   : (RO) M-Mode software interrupt pending
   // `define MIP 12'h344

   assign ce_int = (mdccme_ce_req | miccme_ce_req | mice_ce_req);

   // assign mip_n[63:0] = {ce_int, mexintpend, timer_int_sync, mip[0]}; 
   // rvdff #(4)  mip_ff (.*, .clk(free_clk), .din(mip_ns[3:0]), .dout(mip[3:0]));

   // ----------------------------------------------------------------------
   // MIE (RW)
   // [30] MCEIE  : (RO) M-Mode Correctable Error interrupt enable
   // [11] MEIE   : (RW) M-Mode external interrupt enable
   // [7]  MTIE   : (RW) M-Mode timer interrupt enable
   // [3]  MSIE   : (RW) M-Mode software interrupt enable
   // `define MIE 12'h304
   
   // assign wr_mie_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MIE);
   // assign mie_ns[3:0] = wr_mie_wb ? {dec_csr_wrdata_wb[30], dec_csr_wrdata_wb[11], dec_csr_wrdata_wb[7], dec_csr_wrdata_wb[3]} : mie[3:0]; 
   // rvdffe #(4)  mie_ff (.*, .en(dec_csr_wen_wb_mod | clk_override), .clk(clk), .din(mie_ns[3:0]), .dout(mie[3:0]));


   // ----------------------------------------------------------------------
   // MCYCLEL (RW)
   // [31:0] : Lower Cycle count
   
   // `define MCYCLEL 12'hb00


   // assign wr_mcyclel_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MCYCLEL);

   // logic mcyclel_cout_in;

   // assign kill_ebreak_count_wb = ebreak_to_debug_mode_wb & dcsr[`DCSR_STOPC];

   // assign mcyclel_cout_in = ~(kill_ebreak_count_wb | (dec_tlu_dbg_halted & dcsr[`DCSR_STOPC]) | dec_tlu_pmu_fw_halted);
   
   // assign {mcyclel_cout, mcyclel_inc[63:0]} = mcyclel[63:0] + {63'b0, mcyclel_cout_in}; 
   // assign mcyclel_ns[63:0] = wr_mcyclel_wb ? dec_csr_wrdata_wb[63:0] : mcyclel_inc[63:0]; 

   // rvdffe #(64) mcyclel_ff      (.*, .en(wr_mcyclel_wb | mcyclel_cout_in), .din(mcyclel_ns[63:0]), .dout(mcyclel[63:0]));
   // rvdff   #(1) mcyclef_cout_ff (.*, .clk(free_clk), .din(mcyclel_cout & ~wr_mcycleh_wb), .dout(mcyclel_cout_f));
   // ----------------------------------------------------------------------
   // MCYCLEH (RW)
   // [63:32] : Higher Cycle count
   // Chained with mcyclel. Note: mcyclel overflow due to a mcycleh write gets ignored.

   // `define MCYCLEH 12'hb80
   
   // assign wr_mcycleh_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MCYCLEH);

   // assign {mcycleh_cout_nc, mcycleh_inc[63:0]} = mcycleh[63:0] + {63'b0, mcyclel_cout_f}; 
   // assign mcycleh_ns[63:0] = wr_mcycleh_wb ? dec_csr_wrdata_wb[63:0] : mcycleh_inc[63:0]; 

   // rvdffe #(64)  mcycleh_ff (.*, .en(wr_mcycleh_wb | mcyclel_cout_f), .din(mcycleh_ns[63:0]), .dout(mcycleh[63:0]));
   
   // ----------------------------------------------------------------------
   // MINSTRETL (RW)
   // [31:0] : Lower Instruction retired count
   // From the spec "Some CSRs, such as the instructions retired counter, instret, may be modified as side effects
   // of instruction execution. In these cases, if a CSR access instruction reads a CSR, it reads the
   // value prior to the execution of the instruction. If a CSR access instruction writes a CSR, the
   // update occurs after the execution of the instruction. In particular, a value written to instret by
   // one instruction will be the value read by the following instruction (i.e., the increment of instret
   // caused by the first instruction retiring happens before the write of the new value)."
   // `define MINSTRETL 12'hb02
   // logic i0_valid_no_ebreak_ecall_wb;
   // assign i0_valid_no_ebreak_ecall_wb = i0_valid_wb & ~(ebreak_wb | ecall_wb | ebreak_to_debug_mode_wb);

   // assign wr_minstretl_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MINSTRETL);

   // assign {minstretl_cout, minstretl_inc[63:0]} = minstretl[63:0] + {63'b0,i0_valid_no_ebreak_ecall_wb} + {63'b0,i1_valid_wb}; 

   // assign minstret_enable = i0_valid_no_ebreak_ecall_wb | i1_valid_wb | wr_minstretl_wb;
   
   // assign minstretl_ns[63:0] = wr_minstretl_wb ? dec_csr_wrdata_wb[63:0] : minstretl_inc[63:0]; 
   // rvdffe #(64)  minstretl_ff (.*, .en(minstret_enable), .din(minstretl_ns[63:0]), .dout(minstretl[63:0]));
   // logic minstret_enable_f;
   // rvdff #(2) minstretf_cout_ff (.*, .clk(free_clk), .din({minstret_enable, minstretl_cout & ~wr_minstreth_wb}), .dout({minstret_enable_f, minstretl_cout_f}));

   // assign minstretl_read[63:0] = minstretl[63:0];
   // ----------------------------------------------------------------------
   // MINSTRETH (RW)
   // [63:32] : Higher Instret count
   // Chained with minstretl. Note: minstretl overflow due to a minstreth write gets ignored.

   // `define MINSTRETH 12'hb82
   
   // assign wr_minstreth_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MINSTRETH);

   // assign {minstreth_cout_nc, minstreth_inc[63:0]} = minstreth[63:0] + {63'b0, minstretl_cout_f}; 
   // assign minstreth_ns[63:0] = wr_minstreth_wb ? dec_csr_wrdata_wb[63:0] : minstreth_inc[63:0]; 
   // rvdffe #(64)  minstreth_ff (.*, .en(minstret_enable_f | wr_minstreth_wb), .din(minstreth_ns[63:0]), .dout(minstreth[63:0]));
  
   // assign minstreth_read[63:0] = minstreth_inc[63:0];

   // ----------------------------------------------------------------------
   // MSCRATCH (RW)
   // [31:0] : Scratch register
   `define MSCRATCH 12'h340
   
   assign wr_mscratch_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MSCRATCH);

   rvdffe #(64)  mscratch_ff (.*, .en(wr_mscratch_wb), .din(dec_csr_wrdata_wb[63:0]), .dout(mscratch[63:0]));

   // ----------------------------------------------------------------------
   // MEPC (RW)
   // [31:1] : Exception PC
   `define MEPC 12'h341

   // NPC
   logic sel_exu_npc_e4, sel_flush_npc_e4, sel_i0_npc_e4, sel_hold_npc_e4;

   // commit all ops
   assign sel_exu_npc_e4 = ~dec_tlu_dbg_halted & ~tlu_flush_lower_wb & (dec_tlu_i0_valid_e4 | dec_tlu_i1_valid_e4) & ~(dec_tlu_i1_valid_e4 & lsu_i0_rfnpc_dc4);
   // commit just i0 when there's a valid i1 that should be flushed
   assign sel_i0_npc_e4 = ~dec_tlu_dbg_halted & ~tlu_flush_lower_wb & dec_tlu_i0_valid_e4 & lsu_i0_rfnpc_dc4 & dec_tlu_i1_valid_e4;
   // flush, update npc
   assign sel_flush_npc_e4 = ~dec_tlu_dbg_halted & tlu_flush_lower_wb & ~dec_tlu_flush_noredir_wb;
   // hold prior npc
   assign sel_hold_npc_e4 = ~sel_exu_npc_e4 & ~sel_flush_npc_e4 & ~sel_i0_npc_e4;

   assign npc_e4[63:1] = ( ({63{sel_exu_npc_e4}} & exu_npc_e4[63:1]) |
                           ({63{sel_i0_npc_e4}} & dec_tlu_i1_pc_e4[63:1]) |
                           ({63{~mpc_reset_run_req & reset_delayed}} & rst_vec[63:1]) | // init to reset vector for mpc halt on reset case
                           ({63{(sel_flush_npc_e4)}} & tlu_flush_path_wb[63:1]) |
                           ({63{(sel_hold_npc_e4)}} & npc_wb[63:1]) );

   rvdffe #(63)  npwbc_ff (.*, .en(/*sel_i0_npc_e4 | sel_exu_npc_e4 | sel_flush_npc_e4 | reset_delayed*/~freeze_dc4), .din(npc_e4[63:1]), .dout(npc_wb[63:1]));
   
   // PC has to be captured for exceptions and interrupts. For MRET, we could execute it and then take an
   // interrupt before the next instruction.
   logic pc0_valid_e4, pc1_valid_e4;
   assign pc0_valid_e4 = ~dec_tlu_dbg_halted & dec_tlu_i0_valid_e4;
   assign pc1_valid_e4 = ~dec_tlu_dbg_halted & dec_tlu_i0_valid_e4 & dec_tlu_i1_valid_e4 & ~lsu_i0_exc_dc4 & ~vec_ls_exc_valid_e4 & ~vec_illegal_instr_dc4_i & ~rfpc_i0_e4 & ~inst_acc_e4 & ~i0_trigger_hit_e4;
   
   assign pc_e4[63:1] = ( ({63{ pc0_valid_e4 & ~pc1_valid_e4}} & dec_tlu_i0_pc_e4[63:1]) |
                          ({63{ pc1_valid_e4}} & dec_tlu_i1_pc_e4[63:1]) |
                          ({63{~pc0_valid_e4 & ~pc1_valid_e4}} & pc_wb[63:1])
                          );

   rvdffe #(63)  pwbc_ff (.*, .en(/*pc0_valid_e4 | pc1_valid_e4*/~freeze_dc4), .din(pc_e4[63:1]), .dout(pc_wb[63:1]));
   
   assign wr_mepc_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MEPC);

   // assign mepc_ns[63:1] = ( ({63{i0_exception_valid_wb | lsu_exc_valid_wb | mepc_trigger_hit_sel_pc_wb}} & pc_wb[63:1]) |
   //                          ({63{interrupt_valid_wb}} & npc_wb[63:1]) |
   //                          ({63{wr_mepc_wb & ~exc_or_int_valid_wb}} & dec_csr_wrdata_wb[63:1]) |
   //                          ({63{~wr_mepc_wb & ~exc_or_int_valid_wb}} & mepc[63:1]) ); 


   // rvdff #(63)  mepc_ff (.*, .clk(e4e5_int_clk), .din(mepc_ns[63:1]), .dout(mepc[63:1]));

   // `define SSCRATCH 12'h140
   // assign wr_sscratch_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `SSCRATCH);
   // assign sscratch_ns[63:0] = ( ({63{wr_sscratch_wb}} & dec_csr_wrdata_wb[63:0]) |
   //                          ({63{~wr_sscratch_wb}} & sscratch[63:0]) ); 
   // rvdffe #(64)  sscratch_ff (.*, .en(e4e5_valid | internal_dbg_halt_mode_f | i_cpu_run_req_d1 | interrupt_valid | interrupt_valid_wb | reset_delayed | pause_expired_e4 | pause_expired_wb | clk_override | dmi_debug_req_posedge_f1 | lsu_exc_valid_e4 | lsu_exc_valid_wb), .clk(clk), .din(sscratch_ns[63:0]), .dout(sscratch[63:0]));


   `define SEPC 12'h141
   assign wr_sepc_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `SEPC);
   // // sepc uses mepc update logic 
   // assign sepc_ns[63:1] = ( ({63{i0_exception_valid_wb | lsu_exc_valid_wb | mepc_trigger_hit_sel_pc_wb}} & pc_wb[63:1]) |
   //                          ({63{interrupt_valid_wb}} & npc_wb[63:1]) |
   //                          ({63{wr_sepc_wb & ~exc_or_int_valid_wb}} & dec_csr_wrdata_wb[63:1]) |
   //                          ({63{~wr_sepc_wb & ~exc_or_int_valid_wb}} & sepc[63:1]) ); 
   // rvdff #(63)  sepc_ff (.*, .clk(e4e5_int_clk), .din(sepc_ns[63:1]), .dout(sepc[63:1]));


   // ----------------------------------------------------------------------
   // MCAUSE (RW)
   // [31:0] : Exception Cause 
   `define MCAUSE 12'h342
   
   // assign wr_mcause_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MCAUSE);
    //64位添加　32'b0
   // assign mcause_ns[63:0] = ( ({64{exc_or_int_valid_wb & take_nmi_wb & nmi_lsu_store_type_f}} & {64'hf000_0000_0000_0000}) |
   //                            ({64{exc_or_int_valid_wb & take_nmi_wb & nmi_lsu_load_type_f}} & {64'hf000_0000_0000_0001}) |
   //                            ({64{exc_or_int_valid_wb & ~take_nmi_wb}} & {interrupt_valid_wb, 58'b0, exc_cause_wb[4:0]}) |
   //                            ({64{wr_mcause_wb & ~exc_or_int_valid_wb}} & dec_csr_wrdata_wb[63:0]) |
   //                            ({64{~wr_mcause_wb & ~exc_or_int_valid_wb}} & mcause[63:0]) ); 

   // rvdff #(64)  mcause_ff (.*, .clk(e4e5_int_clk), .din(mcause_ns[63:0]), .dout(mcause[63:0]));

   // scause uses mcause update logic
   `define SCAUSE 12'h142
   // assign wr_scause_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `SCAUSE);
   //  //64位添加　32'b0
   // assign scause_ns[63:0] = ( ({64{exc_or_int_valid_wb & take_nmi_wb & nmi_lsu_store_type_f}} & {64'hf000_0000_0000_0000}) |
   //                            ({64{exc_or_int_valid_wb & take_nmi_wb & nmi_lsu_load_type_f}} & {64'hf000_0000_0000_0001}) |
   //                            ({64{exc_or_int_valid_wb & ~take_nmi_wb}} & {interrupt_valid_wb, 58'b0, exc_cause_wb[4:0]}) |
   //                            ({64{wr_scause_wb & ~exc_or_int_valid_wb}} & dec_csr_wrdata_wb[63:0]) |
   //                            ({64{~wr_scause_wb & ~exc_or_int_valid_wb}} & scause[63:0]) ); 

   // rvdff #(64)  scause_ff (.*, .clk(e4e5_int_clk), .din(scause_ns[63:0]), .dout(scause[63:0]));

   // ----------------------------------------------------------------------
   // MTVAL (RW)
   // [63:0] : Exception address if relevant
   //   /* debug for hello world */
   // logic [2:0] illegal_counter;
   
   // always @(posedge clk) begin
   //    if(!rst_l) begin
   //       illegal_counter <= 3; 
   //    end else if(illegal_wb) begin
   //       if(illegal_counter >0) begin
   //          illegal_counter <= illegal_counter - 1;
   //       end
   //    end
   // end

   // /* debug for hello world */
  `define MTVAL 12'h343
   
   // assign wr_mtval_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTVAL);
   assign mtval_capture_pc_wb = exc_or_int_valid_wb & (ebreak_wb | ((inst_acc_wb | fetch_page_fault_wb) & ~inst_acc_second_wb) | mepc_trigger_hit_sel_pc_wb) & ~take_nmi_wb;
   assign mtval_capture_pc_plus2_wb = exc_or_int_valid_wb & ((inst_acc_wb | fetch_page_fault_wb) & inst_acc_second_wb) & ~take_nmi_wb;
   assign mtval_capture_inst_wb = exc_or_int_valid_wb & illegal_wb & ~take_nmi_wb;
   assign mtval_capture_lsu_wb = exc_or_int_valid_wb & lsu_exc_valid_wb & ~take_nmi_wb;
   assign mtval_clear_wb = exc_or_int_valid_wb & ~mtval_capture_pc_wb & ~mtval_capture_inst_wb & ~mtval_capture_lsu_wb & ~mepc_trigger_hit_sel_pc_wb;
   assign mtval_capture_vec_wb = exc_or_int_valid_wb & (vec_ls_exc_valid_wb | vec_illegal_instr_wb) & ~take_nmi_wb;
   
   
   assign tval_ns[63:0] = (({64{mtval_capture_pc_wb}} & {pc_wb[63:1], 1'b0}) |
                            ({64{mtval_capture_pc_plus2_wb}} & {pc_wb[63:1] + 63'b1, 1'b0}) |
                            ({64{mtval_capture_inst_wb & ~vec_illegal_instr_wb}} & {32'd0, dec_illegal_inst[31:0]}) |
                            ({64{mtval_capture_lsu_wb}} & lsu_error_pkt_addr_wb[63:0])) |
                            ({64{mtval_capture_vec_wb}} & vec_tval_wb[63:0]);
                           //  ({64{wr_mtval_wb & ~interrupt_valid_wb}} & dec_csr_wrdata_wb[63:0]) |
                           //  ({64{~take_nmi_wb & ~wr_mtval_wb & ~mtval_capture_pc_wb & ~mtval_capture_inst_wb & ~mtval_clear_wb & ~mtval_capture_lsu_wb}} & mtval[63:0]) ); 


   // rvdff #(64)  mtval_ff (.*, .clk(e4e5_int_clk), .din(mtval_ns[63:0]), .dout(mtval[63:0]));
   
   // stval uses mtval update logic
   // `define STVAL 12'h143
   // assign wr_stval_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `STVAL);
   // assign stval_capture_pc_wb = exc_or_int_valid_wb & ( illegal_wb | ebreak_wb | (inst_acc_wb & ~inst_acc_second_wb) | mepc_trigger_hit_sel_pc_wb) & ~take_nmi_wb;
   // assign stval_capture_pc_plus2_wb = exc_or_int_valid_wb & (inst_acc_wb & inst_acc_second_wb) & ~take_nmi_wb;
   // assign stval_capture_inst_wb = exc_or_int_valid_wb /*& illegal_wb*/ & ~take_nmi_wb;
   // assign stval_capture_lsu_wb = exc_or_int_valid_wb & lsu_exc_valid_wb & ~take_nmi_wb;
   // assign stval_clear_wb = exc_or_int_valid_wb & ~stval_capture_pc_wb & ~stval_capture_inst_wb & ~stval_capture_lsu_wb & ~mepc_trigger_hit_sel_pc_wb;
  
   // assign stval_ns[63:0] = (({64{stval_capture_pc_wb}} & {pc_wb[63:1], 1'b0}) |
   //                          ({64{stval_capture_pc_plus2_wb}} & {pc_wb[63:1] + 63'b1, 1'b0}) |
   //                          ({64{stval_capture_inst_wb}} & {32'd0, dec_illegal_inst[31:0]}) |
   //                          ({64{stval_capture_lsu_wb}} & lsu_error_pkt_addr_wb[63:0]) |                            
   //                          ({64{wr_stval_wb & ~interrupt_valid_wb}} & dec_csr_wrdata_wb[63:0]) |
   //                          ({64{~take_nmi_wb & ~wr_stval_wb & ~stval_capture_pc_wb & ~stval_capture_inst_wb & ~stval_clear_wb & ~stval_capture_lsu_wb}} & stval[63:0]) ); 
   // logic [63:0] stval_ns_final;
   // assign stval_ns_final = (illegal_wb && stval_ns < 32'h8000_0000 && stval_ns >= 32'h0001_0000)? (stval_ns + (illegal_counter << 12)) : stval_ns;

   // rvdff #(64)  stval_ff (.*, .clk(e4e5_int_clk), .din(stval_ns[63:0]), .dout(stval[63:0]));
   
   // ----------------------------------------------------------------------
   // MCGC (RW) Clock gating control 
   // [31:9] : Reserved, reads 0x0
   // [8]    : misc_clk_override
   // [7]    : dec_clk_override
   // [6]    : exu_clk_override
   // [5]    : ifu_clk_override
   // [4]    : lsu_clk_override
   // [3]    : bus_clk_override
   // [2]    : pic_clk_override
   // [1]    : dccm_clk_override
   // [0]    : icm_clk_override
   // 
   `define MCGC 12'h7f8
   assign wr_mcgc_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MCGC);

   rvdffe #(9)  mcgc_ff (.*, .en(wr_mcgc_wb), .din(dec_csr_wrdata_wb[8:0]), .dout(mcgc[8:0]));

   assign dec_tlu_misc_clk_override = mcgc[8];
   assign dec_tlu_dec_clk_override  = mcgc[7];
   assign dec_tlu_exu_clk_override  = mcgc[6];
   assign dec_tlu_ifu_clk_override  = mcgc[5];
   assign dec_tlu_lsu_clk_override  = mcgc[4];
   assign dec_tlu_bus_clk_override  = mcgc[3];
   assign dec_tlu_pic_clk_override  = mcgc[2];
   assign dec_tlu_dccm_clk_override = mcgc[1];
   assign dec_tlu_icm_clk_override  = mcgc[0];

   // ----------------------------------------------------------------------
   // MFDC (RW) Feature Disable Control
   // [31:19] : Reserved, reads 0x0
   // [18:16] : DMA QoS Prty
   // [15:11] : Reserved, reads 0x0
   // [10]   : Disable dual issue
   // [9]    : Disable pic multiple ints
   // [8]    : Disable core ecc
   // [7]    : Disable secondary alu?s                                  
   // [6]    : Disable multiple outstanding sideeffect accesses to bus
   // [5]    : Disable non-blocking loads/divides                       
   // [4]    : Disable fast divide                                      
   // [3]    : Disable branch prediction and return stack               
   // [2]    : Disable write buffer coalescing                          
   // [1]    : Disable load misses that bypass the write buffer         
   // [0]    : Disable pipelining - Enable single instruction execution 
   // 
   `define MFDC 12'h7f9
   
   assign wr_mfdc_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MFDC);

   rvdffe #(14)  mfdc_ff (.*, .en(wr_mfdc_wb), .din(mfdc_ns[13:0]), .dout(mfdc_int[13:0]));

   `ifdef RV_BUILD_AXI4
   // flip poweron value of bit 6 for AXI build
   assign mfdc_ns[13:0] = {~dec_csr_wrdata_wb[18:16],dec_csr_wrdata_wb[10:7], ~dec_csr_wrdata_wb[6], dec_csr_wrdata_wb[5:0]};
   assign mfdc[18:0] = {~mfdc_int[13:11], 5'b0, mfdc_int[10:7], ~mfdc_int[6], mfdc_int[5:0]};
   `else
   assign mfdc_ns[13:0] = {~dec_csr_wrdata_wb[18:16],dec_csr_wrdata_wb[10:0]};
   assign mfdc[18:0] = {~mfdc_int[13:11], 5'b0, mfdc_int[10:0]};
   `endif

   assign dec_tlu_dma_qos_prty[2:0] = mfdc[18:16];
   assign dec_tlu_dual_issue_disable = mfdc[10];
   assign dec_tlu_core_ecc_disable = mfdc[8];
   assign dec_tlu_sec_alu_disable = mfdc[7];
   assign dec_tlu_sideeffect_posted_disable = mfdc[6];
   assign dec_tlu_non_blocking_disable = mfdc[5];
   assign dec_tlu_fast_div_disable = mfdc[4];
   assign dec_tlu_bpred_disable = mfdc[3];
   assign dec_tlu_wb_coalescing_disable = mfdc[2];
   assign dec_tlu_ld_miss_byp_wb_disable = mfdc[1];
   assign dec_tlu_pipelining_disable = mfdc[0];
   
   // ----------------------------------------------------------------------
   // MCPC (RW) Pause counter
   // [31:0] : Reads 0x0, decs in the wb register in decode_ctl

   `define MCPC 12'h7c2
   assign dec_tlu_wr_pause_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MCPC) & ~interrupt_valid_wb;

   // ----------------------------------------------------------------------
   // MRAC (RW)
   // [31:0] : Region Access Control Register, 16 regions, {side_effect, cachable} pairs
   `define MRAC 12'h7c0
   
   // assign wr_mrac_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MRAC);

   // prevent pairs of 0x11, side_effect and cacheable
   logic [63:0] mrac_in;

   //64位修改　添加32'b0
   assign mrac_in[63:0] = {dec_csr_wrdata_wb[63], dec_csr_wrdata_wb[62] & ~dec_csr_wrdata_wb[63],
                           dec_csr_wrdata_wb[61], dec_csr_wrdata_wb[60] & ~dec_csr_wrdata_wb[61],
                           dec_csr_wrdata_wb[59], dec_csr_wrdata_wb[58] & ~dec_csr_wrdata_wb[59],
                           dec_csr_wrdata_wb[57], dec_csr_wrdata_wb[56] & ~dec_csr_wrdata_wb[57],
                           dec_csr_wrdata_wb[55], dec_csr_wrdata_wb[54] & ~dec_csr_wrdata_wb[55],
                           dec_csr_wrdata_wb[53], dec_csr_wrdata_wb[52] & ~dec_csr_wrdata_wb[53],
                           dec_csr_wrdata_wb[51], dec_csr_wrdata_wb[50] & ~dec_csr_wrdata_wb[51],
                           dec_csr_wrdata_wb[49], dec_csr_wrdata_wb[48] & ~dec_csr_wrdata_wb[49],
                           dec_csr_wrdata_wb[47], dec_csr_wrdata_wb[46] & ~dec_csr_wrdata_wb[47],
                           dec_csr_wrdata_wb[45], dec_csr_wrdata_wb[44] & ~dec_csr_wrdata_wb[45],
                           dec_csr_wrdata_wb[43], dec_csr_wrdata_wb[42] & ~dec_csr_wrdata_wb[43],
                           dec_csr_wrdata_wb[41], dec_csr_wrdata_wb[40] & ~dec_csr_wrdata_wb[41],
                           dec_csr_wrdata_wb[39], dec_csr_wrdata_wb[38] & ~dec_csr_wrdata_wb[39],
                           dec_csr_wrdata_wb[37], dec_csr_wrdata_wb[36] & ~dec_csr_wrdata_wb[37],
                           dec_csr_wrdata_wb[35], dec_csr_wrdata_wb[34] & ~dec_csr_wrdata_wb[35],
                           dec_csr_wrdata_wb[33], dec_csr_wrdata_wb[32] & ~dec_csr_wrdata_wb[33],
                           dec_csr_wrdata_wb[31], dec_csr_wrdata_wb[30] & ~dec_csr_wrdata_wb[31],
                           dec_csr_wrdata_wb[29], dec_csr_wrdata_wb[28] & ~dec_csr_wrdata_wb[29],
                           dec_csr_wrdata_wb[27], dec_csr_wrdata_wb[26] & ~dec_csr_wrdata_wb[27],
                           dec_csr_wrdata_wb[25], dec_csr_wrdata_wb[24] & ~dec_csr_wrdata_wb[25],
                           dec_csr_wrdata_wb[23], dec_csr_wrdata_wb[22] & ~dec_csr_wrdata_wb[23],
                           dec_csr_wrdata_wb[21], dec_csr_wrdata_wb[20] & ~dec_csr_wrdata_wb[21],
                           dec_csr_wrdata_wb[19], dec_csr_wrdata_wb[18] & ~dec_csr_wrdata_wb[19],
                           dec_csr_wrdata_wb[17], dec_csr_wrdata_wb[16] & ~dec_csr_wrdata_wb[17],
                           dec_csr_wrdata_wb[15], dec_csr_wrdata_wb[14] & ~dec_csr_wrdata_wb[15],
                           dec_csr_wrdata_wb[13], dec_csr_wrdata_wb[12] & ~dec_csr_wrdata_wb[13],
                           dec_csr_wrdata_wb[11], dec_csr_wrdata_wb[10] & ~dec_csr_wrdata_wb[11],
                           dec_csr_wrdata_wb[9], dec_csr_wrdata_wb[8] & ~dec_csr_wrdata_wb[9],
                           dec_csr_wrdata_wb[7], dec_csr_wrdata_wb[6] & ~dec_csr_wrdata_wb[7],
                           dec_csr_wrdata_wb[5], dec_csr_wrdata_wb[4] & ~dec_csr_wrdata_wb[5],
                           dec_csr_wrdata_wb[3], dec_csr_wrdata_wb[2] & ~dec_csr_wrdata_wb[3],
                           dec_csr_wrdata_wb[1], dec_csr_wrdata_wb[0] & ~dec_csr_wrdata_wb[1]};
   
   // rvdffe #(64)  mrac_ff (.*, .en(wr_mrac_wb), .din(mrac_in[63:0]), .dout(mrac[63:0]));

   // drive to LSU/IFU
   assign dec_tlu_mrac_ff[63:0] = mrac[63:0];
   
   // ----------------------------------------------------------------------
   // MDEAU (WAR0)
   // [31:0] : Dbus Error Address Unlock register
   // 
   `define MDEAU 12'hbc0
   
   assign wr_mdeau_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MDEAU);

   
   // ----------------------------------------------------------------------
   // MDSEAC (R)
   // [31:0] : Dbus Store Error Address Capture register
   // 
   `define MDSEAC 12'hfc0
   
   // only capture error bus if the MDSEAC reg is not locked
   assign mdseac_locked_ns = mdseac_en | (mdseac_locked_f & ~wr_mdeau_wb);
   
   assign mdseac_en = (lsu_imprecise_error_store_any | lsu_imprecise_error_load_any) & ~mdseac_locked_f;
   
   rvdffe #(64)  mdseac_ff (.*, .en(mdseac_en), .din(lsu_imprecise_error_addr_any[63:0]), .dout(mdseac[63:0]));
   
   // ----------------------------------------------------------------------
   // MPMC (R0W1)
   // [0:0] : FW halt
   // 
   `define MPMC 12'h7c6
   logic wr_mpmc_wb;
   assign wr_mpmc_wb = dec_csr_wrdata_wb[0] & dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MPMC);
   assign fw_halt_req = wr_mpmc_wb & ~internal_dbg_halt_mode_f;
   
   // ----------------------------------------------------------------------
   // MICECT (I-Cache error counter/threshold)
   // [31:27] : Icache parity error threshold
   // [26:0]  : Icache parity error count
   `define MICECT 12'h7f0

   logic [31:27] csr_sat;
   assign csr_sat[31:27] = (dec_csr_wrdata_wb[31:27] > 5'd26) ? 5'd26 : dec_csr_wrdata_wb[31:27];
   
   assign wr_micect_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MICECT);
   assign {micect_cout_nc, micect_inc[26:0]} = micect[26:0] + {26'b0, ic_perr_wb};

    //64位修改 添加32'b0
   assign micect_ns = {32'b0, wr_micect_wb ? {csr_sat[31:27], dec_csr_wrdata_wb[26:0]} : {micect[31:27], micect_inc[26:0]}};
   //assign micect_ns =  wr_micect_wb ? {csr_sat[31:27], dec_csr_wrdata_wb[26:0]} : {micect[31:27], micect_inc[26:0]};

   rvdffe #(64)  micect_ff (.*, .en(wr_micect_wb | ic_perr_wb), .din(micect_ns[63:0]), .dout(micect[63:0]));

   assign mice_ce_req = |({32'b1 << micect[31:27]} & {5'b0, micect[26:0]});
   
   // ----------------------------------------------------------------------
   // MICCMECT (ICCM error counter/threshold)
   // [31:27] : ICCM parity error threshold
   // [26:0]  : ICCM parity error count
   `define MICCMECT 12'h7f1
   
   assign wr_miccmect_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MICCMECT);
   assign {miccmect_cout_nc, miccmect_inc[26:0]} = miccmect[26:0] + {26'b0, iccm_sbecc_wb | iccm_dma_sb_error};

   //64位修改　添加32'b0
   assign miccmect_ns = {32'b0, wr_miccmect_wb ? {csr_sat[31:27], dec_csr_wrdata_wb[26:0]} : {miccmect[31:27], miccmect_inc[26:0]}};

   rvdffe #(64)  miccmect_ff (.*, .en(wr_miccmect_wb | iccm_sbecc_wb | iccm_dma_sb_error), .din(miccmect_ns[63:0]), .dout(miccmect[63:0]));

   assign miccme_ce_req = |({32'b1 << miccmect[31:27]} & {5'b0, miccmect[26:0]});
   
   // ----------------------------------------------------------------------
   // MDCCMECT (DCCM error counter/threshold) 
   // [31:27] : DCCM parity error threshold
   // [26:0]  : DCCM parity error count
   `define MDCCMECT 12'h7f2
   
   assign wr_mdccmect_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MDCCMECT);
   assign {mdccmect_cout_nc, mdccmect_inc[26:0]} = mdccmect[26:0] + {26'b0, lsu_single_ecc_error_wb};

   //64位修改　添加32'b0
   assign mdccmect_ns = {32'b0, wr_mdccmect_wb ? {csr_sat[31:27], dec_csr_wrdata_wb[26:0]} : {mdccmect[31:27], mdccmect_inc[26:0]}};

   rvdffe #(64)  mdccmect_ff (.*, .en(wr_mdccmect_wb | lsu_single_ecc_error_wb), .din(mdccmect_ns[63:0]), .dout(mdccmect[63:0]));

   assign mdccme_ce_req = |({32'b1 << mdccmect[31:27]} & {5'b0, mdccmect[26:0]});
   
   // ----------------------------------------------------------------------
   // MEIVT (External Interrupt Vector Table (R/W))
   // [31:10]: Base address (R/W)
   // [9:0]  : Reserved, reads 0x0
   `define MEIVT 12'hbc8
   
   assign wr_meivt_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MEIVT);

   rvdffe #(22)  meivt_ff (.*, .en(wr_meivt_wb), .din(dec_csr_wrdata_wb[31:10]), .dout(meivt[31:10]));

   // ----------------------------------------------------------------------
   // MEIHAP (External Interrupt Handler Access Pointer (R))
   // [31:10]: Base address (R/W)
   // [9:2]  : ClaimID (R)
   // [1:0]  : Reserved, 0x0
   `define MEIHAP 12'hfc8
   
   assign wr_meihap_wb = wr_meicpct_wb;

   rvdffe #(8)  meihap_ff (.*, .en(wr_meihap_wb), .din(pic_claimid[7:0]), .dout(meihap[9:2]));
   
   // ----------------------------------------------------------------------
   // MEICURPL (R/W)
   // [31:4] : Reserved (read 0x0)
   // [3:0]  : CURRPRI - Priority level of current interrupt service routine (R/W)
   `define MEICURPL 12'hbcc
   
   assign wr_meicurpl_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MEICURPL);
   assign meicurpl_ns[3:0] = wr_meicurpl_wb ? dec_csr_wrdata_wb[3:0] : meicurpl[3:0]; 
   
   rvdffe #(4)  meicurpl_ff (.*, .en(dec_csr_wen_wb_mod | clk_override), .clk(clk), .din(meicurpl_ns[3:0]), .dout(meicurpl[3:0]));

   // PIC needs this reg
   assign dec_tlu_meicurpl[3:0] = meicurpl[3:0];
   
   
   // ----------------------------------------------------------------------
   // MEICIDPL (R/W)
   // [31:4] : Reserved (read 0x0)
   // [3:0]  : External Interrupt Claim ID's Priority Level Register
   `define MEICIDPL 12'hbcb
   
   assign wr_meicidpl_wb = (dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MEICIDPL));
   
   assign meicidpl_ns[3:0] = wr_meicpct_wb ? pic_pl[3:0] : (wr_meicidpl_wb ? dec_csr_wrdata_wb[3:0] : meicidpl[3:0]); 
   
   rvdffe #(4)  meicidpl_ff (.*, .en(dec_csr_wen_wb_mod | clk_override), .clk(clk), .din(meicidpl_ns[3:0]), .dout(meicidpl[3:0]));
   
   // ----------------------------------------------------------------------
   // MEICPCT (Capture CLAIMID in MEIHAP and PL in MEICIDPL
   // [31:1] : Reserved (read 0x0)
   // [0]    : Capture (W1, Read 0)
   `define MEICPCT 12'hbca

   assign wr_meicpct_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MEICPCT);

   // ----------------------------------------------------------------------
   // MEIPT (External Interrupt Priority Threshold)
   // [31:4] : Reserved (read 0x0)
   // [3:0]  : PRITHRESH 
   `define MEIPT 12'hbc9
   
   assign wr_meipt_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MEIPT);
   assign meipt_ns[3:0] = wr_meipt_wb ? dec_csr_wrdata_wb[3:0] : meipt[3:0]; 
   
   rvdff #(4)  meipt_ff (.*, .clk(active_clk), .din(meipt_ns[3:0]), .dout(meipt[3:0]));

   // to PIC
   assign dec_tlu_meipt[3:0] = meipt[3:0];
   // ----------------------------------------------------------------------
   // DCSR (R/W) (Only accessible in debug mode)
   // [31:28] : xdebugver (hard coded to 0x4) RO
   // [27:16] : 0x0, reserved
   // [15]    : ebreakm
   // [14]    : 0x0, reserved
   // [13]    : ebreaks (0x0 for this core)
   // [12]    : ebreaku (0x0 for this core)
   // [11]    : stepie
   // [10]    : stopcount
   // [9]     : 0x0 //stoptime
   // [8:6]   : cause (RO)
   // [5:4]   : 0x0, reserved
   // [3]     : nmip
   // [2]     : step
   // [1:0]   : prv (0x3 for this core)
   //
   // `define DCSR 12'h7b0
   // logic [8:6] dcsr_cause;

   // RV has clarified that 'priority 4' in the spec means top priority. 
   // 4. single step. 3. Debugger request. 2. Ebreak. 1. Trigger.

   // RV debug spec indicates a cause priority change for trigger hits during single step.
   // assign trigger_hit_for_dscr_cause_wb = trigger_hit_dmode_wb | (trigger_hit_wb & dcsr_single_step_done_f);

   // assign dcsr_cause[8:6] = ( ({3{dcsr_single_step_done_f & ~ebreak_to_debug_mode_wb & ~trigger_hit_for_dscr_cause_wb & ~debug_halt_req}} & 3'b100) |
   //                            ({3{debug_halt_req & ~ebreak_to_debug_mode_wb & ~trigger_hit_for_dscr_cause_wb}} &  3'b011) |
   //                            ({3{ebreak_to_debug_mode_wb & ~trigger_hit_for_dscr_cause_wb}} &  3'b001) | 
   //                            ({3{trigger_hit_for_dscr_cause_wb}} & 3'b010));

   // assign wr_dcsr_wb = allow_dbg_halt_csr_write & dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `DCSR);

   

  // Multiple halt enter requests can happen before we are halted. 
  // We have to continue to upgrade based on dcsr_cause priority but we can't downgrade.
   logic enter_debug_halt_req_le, dcsr_cause_upgradeable;
   assign dcsr_cause_upgradeable = internal_dbg_halt_mode_f & (dcsr[8:6] == 3'b011);
   assign enter_debug_halt_req_le = enter_debug_halt_req & (~dbg_tlu_halted | dcsr_cause_upgradeable);

   assign nmi_in_debug_mode = nmi_int_detected_f & internal_dbg_halt_mode_f;
   // assign dcsr_ns[15:2] = enter_debug_halt_req_le ? {dcsr[15:9], dcsr_cause[8:6], dcsr[5:2]} : 
   //                        (wr_dcsr_wb ? {dec_csr_wrdata_wb[15], 3'b0, dec_csr_wrdata_wb[11:10], 1'b0, dcsr[8:6], 2'b00, nmi_in_debug_mode | dcsr[3], dec_csr_wrdata_wb[2]} :
   //                         {dcsr[15:4], nmi_in_debug_mode, dcsr[2]});
   
   // rvdffe #(14)  dcsr_ff (.*, .en(enter_debug_halt_req_le | wr_dcsr_wb | internal_dbg_halt_mode | take_nmi_wb), .din(dcsr_ns[15:2]), .dout(dcsr[15:2]));

   // ----------------------------------------------------------------------
   // DPC (R/W) (Only accessible in debug mode)
   // [31:0] : Debug PC
   // 2022.03.08 dmi debug store pc
   `define DPC 12'h7b1
   // assign wr_dpc_wb = allow_dbg_halt_csr_write & dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `DPC);
   // assign dpc_capture_npc = (dbg_tlu_halted & ~dbg_tlu_halted_f & ~request_debug_mode_done_f) | dmi_debug_req_posedge_f1;
   // assign dpc_capture_pc = request_debug_mode_wb;

   // assign dpc_ns[63:1] = ( ({63{~dpc_capture_pc & ~dpc_capture_npc & wr_dpc_wb}} & dec_csr_wrdata_wb[63:1]) |
   //                         ({63{dpc_capture_pc}} & pc_wb[63:1]) |
   //                         ({63{~dpc_capture_pc & dpc_capture_npc}} & npc_wb[63:1]) );
   
   // rvdffe #(63)  dpc_ff (.*, .en(wr_dpc_wb | dpc_capture_pc | dpc_capture_npc), .din(dpc_ns[63:1]), .dout(dpc[63:1]));

   // ----------------------------------------------------------------------
   // DICAWICS (R/W) (Only accessible in debug mode)
   // [31:25] : Reserved
   // [24]    : Array select, 0 is data, 1 is tag
   // [23:22] : Reserved
   // [21:20] : Way select
   // [19:16] : Reserved
   // [15:2]  : Index
   // [1:0]   : Reserved
   `define DICAWICS 12'h7c8

   assign dicawics_ns[18:2] = {dec_csr_wrdata_wb[24], dec_csr_wrdata_wb[21:20], dec_csr_wrdata_wb[15:2]};
   assign wr_dicawics_wb = allow_dbg_halt_csr_write & dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `DICAWICS);
                           
   rvdffe #(17)  dicawics_ff (.*, .en(wr_dicawics_wb), .din(dicawics_ns[18:2]), .dout(dicawics[18:2]));

   // ----------------------------------------------------------------------
   // DICAD0 (R/W) (Only accessible in debug mode)
   //
   // If dicawics[array] is 0
   // [31:0]  : inst data
   //
   // If dicawics[array] is 1
   // [31:16] : Tag
   // [15:7]  : Reserved
   // [6:4]   : LRU
   // [3:1]   : Reserved
   // [0]     : Valid
   `define DICAD0 12'h7c9

   assign dicad0_ns[63:0] = wr_dicad0_wb ? dec_csr_wrdata_wb[63:0] : ifu_ic_debug_rd_data[63:0];

   assign wr_dicad0_wb = allow_dbg_halt_csr_write & dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `DICAD0);
                           
   rvdffe #(64)  dicad0_ff (.*, .en(wr_dicad0_wb | ifu_ic_debug_rd_data_valid), .din(dicad0_ns[63:0]), .dout(dicad0[63:0]));


`ifdef RV_ICACHE_ECC
   // ----------------------------------------------------------------------
   // DICAD1 (R/W) (Only accessible in debug mode)
   // [9:0]     : ECC
   `define DICAD1 12'h7ca

   assign dicad1_ns[9:0] = wr_dicad1_wb ? dec_csr_wrdata_wb[9:0] : ifu_ic_debug_rd_data[41:32];

   assign wr_dicad1_wb = allow_dbg_halt_csr_write & dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `DICAD1);
                           
   rvdffs #(10)  dicad1_ff (.*, .clk(active_clk), .en(wr_dicad1_wb | ifu_ic_debug_rd_data_valid), .din(dicad1_ns[9:0]), .dout(dicad1[9:0]));

`else
   // ----------------------------------------------------------------------
   // DICAD1 (R/W) (Only accessible in debug mode)
   // [1:0]     : Parity
   `define DICAD1 12'h7ca

   assign dicad1_ns[1:0] = wr_dicad1_wb ? dec_csr_wrdata_wb[1:0] : ifu_ic_debug_rd_data[33:32];

   assign wr_dicad1_wb = allow_dbg_halt_csr_write & dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `DICAD1);
                           
   rvdffs #(2)  dicad1_ff (.*, .clk(active_clk), .en(wr_dicad1_wb | ifu_ic_debug_rd_data_valid), .din(dicad1_ns[1:0]), .dout(dicad1[1:0]));
`endif
   // ----------------------------------------------------------------------
   // DICAGO (R/W) (Only accessible in debug mode)
   // [0]     : Go
   `define DICAGO 12'h7cb
   
`ifdef RV_ICACHE_ECC
   assign dec_tlu_ic_diag_pkt.icache_wrdata[41:0] = {dicad1[9:0], dicad0[31:0]}; 
`else
   assign dec_tlu_ic_diag_pkt.icache_wrdata[33:0] = {dicad1[1:0], dicad0[31:0]};
`endif
   assign dec_tlu_ic_diag_pkt.icache_dicawics[18:2] = dicawics[18:2];

   logic icache_rd_valid, icache_wr_valid, icache_rd_valid_f, icache_wr_valid_f;
   assign icache_rd_valid = allow_dbg_halt_csr_write & dec_csr_any_unq_d & dec_i0_decode_d & ~dec_csr_wen_unq_d & (dec_csr_rdaddr_d[11:0] == `DICAGO);
   assign icache_wr_valid = allow_dbg_halt_csr_write & dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `DICAGO);

   rvdff #(2)  dicgo_ff (.*, .clk(active_clk), .din({icache_rd_valid, icache_wr_valid}), .dout({icache_rd_valid_f, icache_wr_valid_f}));

   assign dec_tlu_ic_diag_pkt.icache_rd_valid = icache_rd_valid_f;
   assign dec_tlu_ic_diag_pkt.icache_wr_valid = icache_wr_valid_f;

   // ----------------------------------------------------------------------
   // MTSEL (R/W)
   // [1:0] : Trigger select : 00, 01, 10 are data/address triggers. 11 is inst count
   `define MTSEL 12'h7a0
   
   assign wr_mtsel_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTSEL);
   assign mtsel_ns[1:0] = wr_mtsel_wb ? {dec_csr_wrdata_wb[1:0]} : mtsel[1:0]; 

   rvdffe #(2)  mtsel_ff (.*, .en(dec_csr_wen_wb_mod | clk_override), .clk(clk), .din(mtsel_ns[1:0]), .dout(mtsel[1:0]));
   
   // ----------------------------------------------------------------------
   // MTDATA1 (R/W)
   // [31:0] : Trigger Data 1
   `define MTDATA1 12'h7a1

   // for triggers 0, 1, 2 and 3 aka Match Control
   // [31:28] : type, hard coded to 0x2
   // [27]    : dmode
   // [26:21] : hard coded to 0x1f
   // [20]    : hit
   // [19]    : select (0 - address, 1 - data)
   // [18]    : timing, always 'before', reads 0x0
   // [17:12] : action, bits  [17:13] not implemented and reads 0x0
   // [11]    : chain
   // [10:7]  : match, bits [10:8] not implemented and reads 0x0
   // [6]     : M
   // [5:3]   : not implemented, reads 0x0
   // [2]     : execute
   // [1]     : store
   // [0]     : load
   //
   // decoder ring
   // [27]    : => 9
   // [20]    : => 8
   // [19]    : => 7
   // [12]    : => 6
   // [11]    : => 5
   // [7]     : => 4
   // [6]     : => 3
   // [2]     : => 2
   // [1]     : => 1
   // [0]     : => 0


   // don't allow setting load-data.
   assign tdata_load = dec_csr_wrdata_wb[0] & ~dec_csr_wrdata_wb[19];
   // don't allow setting execute-data.
   assign tdata_opcode = dec_csr_wrdata_wb[2] & ~dec_csr_wrdata_wb[19];
   // don't allow clearing DMODE and action=1
   assign tdata_action = (dec_csr_wrdata_wb[27] & dbg_tlu_halted_f) & dec_csr_wrdata_wb[12];
   
   assign tdata_wrdata_wb[9:0]  = {dec_csr_wrdata_wb[27] & dbg_tlu_halted_f, 
                                   dec_csr_wrdata_wb[20:19],
                                   tdata_action, 
                                   dec_csr_wrdata_wb[11], 
                                   dec_csr_wrdata_wb[7:6], 
                                   tdata_opcode, 
                                   dec_csr_wrdata_wb[1], 
                                   tdata_load};

   // If the DMODE bit is set, tdata1 can only be updated in debug_mode
   assign wr_mtdata1_t0_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTDATA1) & (mtsel[1:0] == 2'b0) & (~mtdata1_t0[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign mtdata1_t0_ns[9:0] = wr_mtdata1_t0_wb ? tdata_wrdata_wb[9:0] : 
                                {mtdata1_t0[9], update_hit_bit_wb[0] | mtdata1_t0[8], mtdata1_t0[7:0]}; 

   assign wr_mtdata1_t1_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTDATA1) & (mtsel[1:0] == 2'b01) & (~mtdata1_t1[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign mtdata1_t1_ns[9:0] = wr_mtdata1_t1_wb ? tdata_wrdata_wb[9:0] : 
                                {mtdata1_t1[9], update_hit_bit_wb[1] | mtdata1_t1[8], mtdata1_t1[7:0]}; 

   assign wr_mtdata1_t2_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTDATA1) & (mtsel[1:0] == 2'b10) & (~mtdata1_t2[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign mtdata1_t2_ns[9:0] = wr_mtdata1_t2_wb ? tdata_wrdata_wb[9:0] : 
                                {mtdata1_t2[9], update_hit_bit_wb[2] | mtdata1_t2[8], mtdata1_t2[7:0]}; 

   assign wr_mtdata1_t3_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTDATA1) & (mtsel[1:0] == 2'b11) & (~mtdata1_t3[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign mtdata1_t3_ns[9:0] = wr_mtdata1_t3_wb ? tdata_wrdata_wb[9:0] : 
                                {mtdata1_t3[9], update_hit_bit_wb[3] | mtdata1_t3[8], mtdata1_t3[7:0]}; 

   
   rvdff #(10)  mtdata1_t0_ff (.*, .clk(active_clk), .din(mtdata1_t0_ns[9:0]), .dout(mtdata1_t0[9:0]));
   rvdff #(10)  mtdata1_t1_ff (.*, .clk(active_clk), .din(mtdata1_t1_ns[9:0]), .dout(mtdata1_t1[9:0]));
   rvdff #(10)  mtdata1_t2_ff (.*, .clk(active_clk), .din(mtdata1_t2_ns[9:0]), .dout(mtdata1_t2[9:0]));
   rvdff #(10)  mtdata1_t3_ff (.*, .clk(active_clk), .din(mtdata1_t3_ns[9:0]), .dout(mtdata1_t3[9:0]));
   
   //64位修改　添加32'b0
   assign mtdata1_tsel_out[63:0] ={32'b0, ( ({32{(mtsel[1:0] == 2'b00)}} & {4'h2, mtdata1_t0[9], 6'b011111, mtdata1_t0[8:7], 6'b0, mtdata1_t0[6:5], 3'b0, mtdata1_t0[4:3], 3'b0, mtdata1_t0[2:0]}) |
                                     ({32{(mtsel[1:0] == 2'b01)}} & {4'h2, mtdata1_t1[9], 6'b011111, mtdata1_t1[8:7], 6'b0, mtdata1_t1[6:5], 3'b0, mtdata1_t1[4:3], 3'b0, mtdata1_t1[2:0]}) |
                                     ({32{(mtsel[1:0] == 2'b10)}} & {4'h2, mtdata1_t2[9], 6'b011111, mtdata1_t2[8:7], 6'b0, mtdata1_t2[6:5], 3'b0, mtdata1_t2[4:3], 3'b0, mtdata1_t2[2:0]}) |
                                     ({32{(mtsel[1:0] == 2'b11)}} & {4'h2, mtdata1_t3[9], 6'b011111, mtdata1_t3[8:7], 6'b0, mtdata1_t3[6:5], 3'b0, mtdata1_t3[4:3], 3'b0, mtdata1_t3[2:0]}))};
   
   assign trigger_pkt_any[0].select = mtdata1_t0[`MTDATA1_SEL];
   assign trigger_pkt_any[0].match = mtdata1_t0[`MTDATA1_MATCH];
   assign trigger_pkt_any[0].store = mtdata1_t0[`MTDATA1_ST];
   assign trigger_pkt_any[0].load = mtdata1_t0[`MTDATA1_LD]; 
   assign trigger_pkt_any[0].execute = mtdata1_t0[`MTDATA1_EXE];
   assign trigger_pkt_any[0].m = mtdata1_t0[`MTDATA1_M_ENABLED];
   
   assign trigger_pkt_any[1].select = mtdata1_t1[`MTDATA1_SEL];
   assign trigger_pkt_any[1].match = mtdata1_t1[`MTDATA1_MATCH];
   assign trigger_pkt_any[1].store = mtdata1_t1[`MTDATA1_ST];
   assign trigger_pkt_any[1].load = mtdata1_t1[`MTDATA1_LD];
   assign trigger_pkt_any[1].execute = mtdata1_t1[`MTDATA1_EXE];
   assign trigger_pkt_any[1].m = mtdata1_t1[`MTDATA1_M_ENABLED];
   
   assign trigger_pkt_any[2].select = mtdata1_t2[`MTDATA1_SEL];
   assign trigger_pkt_any[2].match = mtdata1_t2[`MTDATA1_MATCH];
   assign trigger_pkt_any[2].store = mtdata1_t2[`MTDATA1_ST];
   assign trigger_pkt_any[2].load = mtdata1_t2[`MTDATA1_LD];
   assign trigger_pkt_any[2].execute = mtdata1_t2[`MTDATA1_EXE];
   assign trigger_pkt_any[2].m = mtdata1_t2[`MTDATA1_M_ENABLED];
    
   assign trigger_pkt_any[3].select = mtdata1_t3[`MTDATA1_SEL];
   assign trigger_pkt_any[3].match = mtdata1_t3[`MTDATA1_MATCH];
   assign trigger_pkt_any[3].store = mtdata1_t3[`MTDATA1_ST];
   assign trigger_pkt_any[3].load = mtdata1_t3[`MTDATA1_LD];
   assign trigger_pkt_any[3].execute = mtdata1_t3[`MTDATA1_EXE];
   assign trigger_pkt_any[3].m = mtdata1_t3[`MTDATA1_M_ENABLED];
    


   
   
   // ----------------------------------------------------------------------
   // MTDATA2 (R/W)
   // [31:0] : Trigger Data 2
   `define MTDATA2 12'h7a2

   // If the DMODE bit is set, tdata2 can only be updated in debug_mode
   assign wr_mtdata2_t0_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTDATA2) & (mtsel[1:0] == 2'b0)  & (~mtdata1_t0[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign wr_mtdata2_t1_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTDATA2) & (mtsel[1:0] == 2'b01) & (~mtdata1_t1[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign wr_mtdata2_t2_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTDATA2) & (mtsel[1:0] == 2'b10) & (~mtdata1_t2[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign wr_mtdata2_t3_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MTDATA2) & (mtsel[1:0] == 2'b11) & (~mtdata1_t3[`MTDATA1_DMODE] | dbg_tlu_halted_f);

   rvdffe #(64)  mtdata2_t0_ff (.*, .en(wr_mtdata2_t0_wb), .din(dec_csr_wrdata_wb[63:0]), .dout(mtdata2_t0[63:0]));
   rvdffe #(64)  mtdata2_t1_ff (.*, .en(wr_mtdata2_t1_wb), .din(dec_csr_wrdata_wb[63:0]), .dout(mtdata2_t1[63:0]));
   rvdffe #(64)  mtdata2_t2_ff (.*, .en(wr_mtdata2_t2_wb), .din(dec_csr_wrdata_wb[63:0]), .dout(mtdata2_t2[63:0]));
   rvdffe #(64)  mtdata2_t3_ff (.*, .en(wr_mtdata2_t3_wb), .din(dec_csr_wrdata_wb[63:0]), .dout(mtdata2_t3[63:0]));
   
   assign mtdata2_tsel_out[63:0] = ( ({64{(mtsel[1:0] == 2'b00)}} & mtdata2_t0[63:0]) |
                                     ({64{(mtsel[1:0] == 2'b01)}} & mtdata2_t1[63:0]) |
                                     ({64{(mtsel[1:0] == 2'b10)}} & mtdata2_t2[63:0]) |
                                     ({64{(mtsel[1:0] == 2'b11)}} & mtdata2_t3[63:0]));
   
   assign trigger_pkt_any[0].tdata2[63:0] = mtdata2_t0[63:0];
   assign trigger_pkt_any[1].tdata2[63:0] = mtdata2_t1[63:0];
   assign trigger_pkt_any[2].tdata2[63:0] = mtdata2_t2[63:0];
   assign trigger_pkt_any[3].tdata2[63:0] = mtdata2_t3[63:0];


   //----------------------------------------------------------------------
   // Performance Monitor Counters section starts
   //----------------------------------------------------------------------
   `define MHPME_NOEVENT         6'd0
   `define MHPME_CLK_ACTIVE      6'd1 // OOP - out of pipe
   // `define MHPME_ICACHE_HIT      6'd2 // OOP
   `define MHPME_ICACHE_ACCESS   6'd2 // OOP
   `define MHPME_ICACHE_MISS     6'd3 // OOP
   `define MHPME_INST_COMMIT     6'd4
   `define MHPME_INST_COMMIT_16B 6'd5
   `define MHPME_INST_COMMIT_32B 6'd6
   `define MHPME_INST_ALIGNED    6'd7 // OOP
   `define MHPME_INST_DECODED    6'd8 // OOP
   `define MHPME_INST_MUL        6'd9  
   `define MHPME_INST_DIV        6'd10
   `define MHPME_INST_LOAD       6'd11  
   `define MHPME_INST_STORE      6'd12 
   `define MHPME_INST_MALOAD     6'd13  
   `define MHPME_INST_MASTORE    6'd14 
   `define MHPME_INST_ALU        6'd15 
   `define MHPME_INST_CSRREAD    6'd16 
   `define MHPME_INST_CSRRW      6'd17 
   `define MHPME_INST_CSRWRITE   6'd18 
   `define MHPME_INST_EBREAK     6'd19 
   `define MHPME_INST_ECALL      6'd20 
   `define MHPME_INST_FENCE      6'd21 
   `define MHPME_INST_FENCEI     6'd22 
   `define MHPME_INST_MRET       6'd23 
   `define MHPME_INST_BRANCH     6'd24
   `define MHPME_BRANCH_MP       6'd25
   `define MHPME_BRANCH_TAKEN    6'd26
   `define MHPME_BRANCH_NOTP     6'd27
   `define MHPME_FETCH_STALL     6'd28 // OOP
   `define MHPME_ALGNR_STALL     6'd29 // OOP
   `define MHPME_DECODE_STALL    6'd30 // OOP
   `define MHPME_POSTSYNC_STALL  6'd31 // OOP
   `define MHPME_PRESYNC_STALL   6'd32 // OOP
   `define MHPME_LSU_FREEZE      6'd33 // OOP
   `define MHPME_LSU_SB_WB_STALL 6'd34 // OOP
   // `define MHPME_DMA_DCCM_STALL  6'd35 // OOP
   // `define MHPME_DMA_ICCM_STALL  6'd36 // OOP
   `define MHPME_DCACHE_ACCESS   6'd35 // OOP
   `define MHPME_DCACHE_MISS     6'd36 // OOP
   `define MHPME_EXC_TAKEN       6'd37
   `define MHPME_TIMER_INT_TAKEN 6'd38
   `define MHPME_EXT_INT_TAKEN   6'd39
   `define MHPME_FLUSH_LOWER     6'd40
   `define MHPME_BR_ERROR        6'd41
   // `define MHPME_IBUS_TRANS      6'd42 // OOP
   // `define MHPME_DBUS_TRANS      6'd43 // OOP
   // `define MHPME_DBUS_MA_TRANS   6'd44 // OOP
   // `define MHPME_IBUS_ERROR      6'd45 // OOP
   // `define MHPME_DBUS_ERROR      6'd46 // OOP
   // `define MHPME_IBUS_STALL      6'd47 // OOP
   // `define MHPME_DBUS_STALL      6'd48 // OOP
   `define MHPME_FREEZE_DC1      6'd42
   `define MHPME_FREEZE_DC2      6'd43
   `define MHPME_FREEZE_DC3      6'd44
   `define MHPME_FREEZE_DC4      6'd45
   `define MHPME_INT_DISABLED    6'd49 // OOP
   `define MHPME_INT_STALLED     6'd50 // OOP
   `define MHPME_DEBUG_MODE      6'd51

   // logic            mhpmc3_wr_en0, mhpmc3_wr_en1, mhpmc3_wr_en;
   // logic            mhpmc4_wr_en0, mhpmc4_wr_en1, mhpmc4_wr_en;
   // logic            mhpmc5_wr_en0, mhpmc5_wr_en1, mhpmc5_wr_en;
   // logic            mhpmc6_wr_en0, mhpmc6_wr_en1, mhpmc6_wr_en;
   // logic            mhpmc3h_wr_en0, mhpmc3h_wr_en;
   // logic            mhpmc4h_wr_en0, mhpmc4h_wr_en;
   // logic            mhpmc5h_wr_en0, mhpmc5h_wr_en;
   // logic            mhpmc6h_wr_en0, mhpmc6h_wr_en;
   // logic [127:0]    mhpmc3_incr, mhpmc4_incr, mhpmc5_incr, mhpmc6_incr;
   logic [MHPMCOUNTER_NUM-1:0][1:0] mhpmc_inc_e4, mhpmc_inc_wb;
   logic [MHPMCOUNTER_NUM-1:0][5:0] mhpme_vec;

   // only consider committed itypes
   logic [3:0] pmu_i0_itype_qual, pmu_i1_itype_qual;
   assign pmu_i0_itype_qual[3:0] = dec_tlu_packet_e4.pmu_i0_itype[3:0] & {4{tlu_i0_commit_cmt}};
   assign pmu_i1_itype_qual[3:0] = dec_tlu_packet_e4.pmu_i1_itype[3:0] & {4{tlu_i1_commit_cmt}};
   
   // Generate the muxed incs for all counters based on event type
   for (genvar i=0 ; i < MHPMCOUNTER_NUM; i++) begin : hpm_counter_event
      // Pack the event selects into a vector for genvar
      assign mhpme_vec[i][5:0] = mhpmevent[i][5:0];
      assign mhpmc_inc_e4[i][1:0] =  {2{mgpmc}} & 
           ( 
             ({2{(mhpme_vec[i][5:0] == `MHPME_CLK_ACTIVE      )}} & 2'b01) |
            //  ({2{(mhpme_vec[i][5:0] == `MHPME_ICACHE_HIT      )}} & {1'b0, ifu_pmu_ic_hit}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_ICACHE_ACCESS   )}} & {1'b0, pmu_icache_access_i}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_ICACHE_MISS     )}} & {1'b0, pmu_icache_miss_i}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_COMMIT     )}} & {tlu_i1_commit_cmt, tlu_i0_commit_cmt & ~illegal_e4}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_COMMIT_16B )}} & {tlu_i1_commit_cmt & ~exu_pmu_i1_pc4, tlu_i0_commit_cmt & ~exu_pmu_i0_pc4 & ~illegal_e4}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_COMMIT_32B )}} & {tlu_i1_commit_cmt &  exu_pmu_i1_pc4, tlu_i0_commit_cmt &  exu_pmu_i0_pc4 & ~illegal_e4}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_ALIGNED    )}} & ifu_pmu_instr_aligned[1:0])  |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_DECODED    )}} & dec_pmu_instr_decoded[1:0])  |
             ({2{(mhpme_vec[i][5:0] == `MHPME_ALGNR_STALL     )}} & {1'b0,ifu_pmu_align_stall})  |
             ({2{(mhpme_vec[i][5:0] == `MHPME_DECODE_STALL    )}} & {1'b0,dec_pmu_decode_stall}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_MUL        )}} & {(pmu_i1_itype_qual[3:0] == MUL),     (pmu_i0_itype_qual[3:0] == MUL)})     |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_DIV        )}} & {1'b0, dec_tlu_packet_e4.pmu_divide & tlu_i0_commit_cmt})     |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_LOAD       )}} & {(pmu_i1_itype_qual[3:0] == LOAD),    (pmu_i0_itype_qual[3:0] == LOAD)})    |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_STORE      )}} & {(pmu_i1_itype_qual[3:0] == STORE),   (pmu_i0_itype_qual[3:0] == STORE)})   |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_MALOAD     )}} & {(pmu_i1_itype_qual[3:0] == LOAD),    (pmu_i0_itype_qual[3:0] == LOAD)} & 
                                                                      {2{dec_tlu_packet_e4.pmu_lsu_misaligned}})    |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_MASTORE    )}} & {(pmu_i1_itype_qual[3:0] == STORE),   (pmu_i0_itype_qual[3:0] == STORE)} & 
                                                                      {2{dec_tlu_packet_e4.pmu_lsu_misaligned}})    |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_ALU        )}} & {(pmu_i1_itype_qual[3:0] == ALU),     (pmu_i0_itype_qual[3:0] == ALU)})     |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_CSRREAD    )}} & {1'b0, (pmu_i0_itype_qual[3:0] == CSRREAD)}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_CSRWRITE   )}} & {1'b0, (pmu_i0_itype_qual[3:0] == CSRWRITE)})|
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_CSRRW      )}} & {1'b0, (pmu_i0_itype_qual[3:0] == CSRRDWR)})   |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_EBREAK     )}} & {1'b0, (pmu_i0_itype_qual[3:0] == EBREAK)})  |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_ECALL      )}} & {1'b0, (pmu_i0_itype_qual[3:0] == ECALL)})   |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_FENCE      )}} & {1'b0, (pmu_i0_itype_qual[3:0] == FENCE)})   |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_FENCEI     )}} & {1'b0, (pmu_i0_itype_qual[3:0] == FENCEI)})  |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_MRET       )}} & {1'b0, (pmu_i0_itype_qual[3:0] == MRET)})    |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INST_BRANCH     )}} & {((pmu_i1_itype_qual[3:0] == CONDBR) | (pmu_i1_itype_qual[3:0] == JAL)),
                                                                     ((pmu_i0_itype_qual[3:0] == CONDBR) | (pmu_i0_itype_qual[3:0] == JAL))})   |
             ({2{(mhpme_vec[i][5:0] == `MHPME_BRANCH_MP       )}} & {exu_pmu_i1_br_misp & tlu_i1_commit_cmt, exu_pmu_i0_br_misp & tlu_i0_commit_cmt}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_BRANCH_TAKEN    )}} & {exu_pmu_i1_br_ataken & tlu_i1_commit_cmt, exu_pmu_i0_br_ataken & tlu_i0_commit_cmt}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_BRANCH_NOTP     )}} & {dec_tlu_packet_e4.pmu_i1_br_unpred & tlu_i1_commit_cmt, dec_tlu_packet_e4.pmu_i0_br_unpred & tlu_i0_commit_cmt}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_FETCH_STALL     )}} & {1'b0, ifu_pmu_fetch_stall}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_ALGNR_STALL     )}} & {1'b0, ifu_pmu_align_stall}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_DECODE_STALL    )}} & {1'b0, dec_pmu_decode_stall}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_POSTSYNC_STALL  )}} & {1'b0,dec_pmu_postsync_stall}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_PRESYNC_STALL   )}} & {1'b0,dec_pmu_presync_stall}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_LSU_FREEZE      )}} & {1'b0, lsu_freeze_dc3}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_LSU_SB_WB_STALL )}} & {1'b0, lsu_store_stall_any}) |
            //  ({2{(mhpme_vec[i][5:0] == `MHPME_DMA_DCCM_STALL  )}} & {2'b0/*, dma_dccm_stall_any*/}) |
            //  ({2{(mhpme_vec[i][5:0] == `MHPME_DMA_ICCM_STALL  )}} & {2'b0/*, dma_iccm_stall_any*/}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_DCACHE_ACCESS   )}} & {1'b0, pmu_dcache_access_i}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_DCACHE_MISS     )}} & {1'b0, pmu_dcache_miss_i}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_EXC_TAKEN       )}} & {1'b0, (i0_exception_valid_e4 | trigger_hit_e4 | lsu_exc_valid_e4 | vec_ls_exc_valid_e4)}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_TIMER_INT_TAKEN )}} & {1'b0, take_mtimer_int}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_EXT_INT_TAKEN   )}} & {1'b0, take_mext_int}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_FLUSH_LOWER     )}} & {1'b0, tlu_flush_lower_e4}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_BR_ERROR        )}} & {(dec_tlu_br1_error_e4 | dec_tlu_br1_start_error_e4) & rfpc_i1_e4, (dec_tlu_br0_error_e4 | dec_tlu_br0_start_error_e4) & rfpc_i0_e4}) |
            //  ({2{(mhpme_vec[i][5:0] == `MHPME_IBUS_TRANS      )}} & {1'b0, ifu_pmu_bus_trxn}) |
            //  ({2{(mhpme_vec[i][5:0] == `MHPME_DBUS_TRANS      )}} & {1'b0, lsu_pmu_bus_trxn}) |
            //  ({2{(mhpme_vec[i][5:0] == `MHPME_DBUS_MA_TRANS   )}} & {1'b0, lsu_pmu_bus_misaligned}) |
            //  ({2{(mhpme_vec[i][5:0] == `MHPME_IBUS_ERROR      )}} & {1'b0, ifu_pmu_bus_error}) |
            //  ({2{(mhpme_vec[i][5:0] == `MHPME_DBUS_ERROR      )}} & {1'b0, lsu_pmu_bus_error}) |
            //  ({2{(mhpme_vec[i][5:0] == `MHPME_IBUS_STALL      )}} & {1'b0, ifu_pmu_bus_busy}) |
            //  ({2{(mhpme_vec[i][5:0] == `MHPME_DBUS_STALL      )}} & {1'b0, lsu_pmu_bus_busy}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_FREEZE_DC1      )}} & {1'b0, pmu_freeze_dc1}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_FREEZE_DC2      )}} & {1'b0, pmu_freeze_dc2}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_FREEZE_DC3      )}} & {1'b0, lsu_freeze_dc3}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_FREEZE_DC4      )}} & {1'b0, freeze_dc4}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INT_DISABLED    )}} & {1'b0, ~mstatus[`MSTATUS_MIE]}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_INT_STALLED     )}} & {1'b0, ~mstatus[`MSTATUS_MIE] & |(mip[3:0] & mie[3:0])}) |
             ({2{(mhpme_vec[i][5:0] == `MHPME_DEBUG_MODE      )}} & {1'b0, debug_mode})
             );
   
      rvdffsc #(2) pmu0inc_ff (.*, .en(1'b1), .clear(freeze_dc4), .clk(free_clk), .din(mhpmc_inc_e4[i][1:0]), .dout(mhpmc_inc_wb[i][1:0]));
   end

   assign perfcnt_halted = ((dec_tlu_dbg_halted & dcsr[`DCSR_STOPC]) | dec_tlu_pmu_fw_halted);
               
   assign dec_tlu_perfcnt0[1:0] = mhpmc_inc_wb[0][1:0] & ~{2{perfcnt_halted}};
   assign dec_tlu_perfcnt1[1:0] = mhpmc_inc_wb[1][1:0] & ~{2{perfcnt_halted}};
   assign dec_tlu_perfcnt2[1:0] = mhpmc_inc_wb[2][1:0] & ~{2{perfcnt_halted}};
   assign dec_tlu_perfcnt3[1:0] = mhpmc_inc_wb[3][1:0] & ~{2{perfcnt_halted}};

   // ----------------------------------------------------------------------
   // MHPMC3H(RW), MHPMC3(RW)
   // [63:32][31:0] : Hardware Performance Monitor Counter 3
   // `define MHPMC3 12'hB03
   // `define MHPMC3H 12'hB83
   
   // assign mhpmc3_wr_en0 = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPMC3);
   // assign mhpmc3_wr_en1 = ~perfcnt_halted & (|(mhpmc_inc_wb[0][1:0]));
   // assign mhpmc3_wr_en  = mhpmc3_wr_en0 | mhpmc3_wr_en1;
   // assign mhpmc3_incr[127:0] = {mhpmc3h[63:0],mhpmc3[63:0]} + {127'b0,mhpmc_inc_wb[0][1]} + {127'b0,mhpmc_inc_wb[0][0]}; 
   // assign mhpmc3_ns[63:0] = mhpmc3_wr_en0 ? dec_csr_wrdata_wb[63:0] : mhpmc3_incr[63:0];
   // rvdffe #(64)  mhpmc3_ff (.*, .en(mhpmc3_wr_en), .din(mhpmc3_ns[63:0]), .dout(mhpmc3[63:0]));

   // assign mhpmc3h_wr_en0 = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPMC3H);
   // assign mhpmc3h_wr_en  = mhpmc3h_wr_en0 | mhpmc3_wr_en1;
   // assign mhpmc3h_ns[63:0] = mhpmc3h_wr_en0 ? dec_csr_wrdata_wb[63:0] : mhpmc3_incr[127:64];
   // rvdffe #(64)  mhpmc3h_ff (.*, .en(mhpmc3h_wr_en), .din(mhpmc3h_ns[63:0]), .dout(mhpmc3h[63:0]));

   // ----------------------------------------------------------------------
   // MHPMC4H(RW), MHPMC4(RW)
   // [63:32][31:0] : Hardware Performance Monitor Counter 4
   // `define MHPMC4 12'hB04
   // `define MHPMC4H 12'hB84
   
   // assign mhpmc4_wr_en0 = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPMC4);
   // assign mhpmc4_wr_en1 = ~perfcnt_halted & (|(mhpmc_inc_wb[1][1:0]));
   // assign mhpmc4_wr_en  = mhpmc4_wr_en0 | mhpmc4_wr_en1;
   // assign mhpmc4_incr[127:0] = {mhpmc4h[63:0],mhpmc4[63:0]} + {127'b0,mhpmc_inc_wb[1][1]} + {127'b0,mhpmc_inc_wb[1][0]}; 
   // assign mhpmc4_ns[63:0] = mhpmc4_wr_en0 ? dec_csr_wrdata_wb[63:0] : mhpmc4_incr[63:0];
   // rvdffe #(64)  mhpmc4_ff (.*, .en(mhpmc4_wr_en), .din(mhpmc4_ns[63:0]), .dout(mhpmc4[63:0]));

   // assign mhpmc4h_wr_en0 = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPMC4H);
   // assign mhpmc4h_wr_en  = mhpmc4h_wr_en0 | mhpmc4_wr_en1;
   // assign mhpmc4h_ns[63:0] = mhpmc4h_wr_en0 ? dec_csr_wrdata_wb[63:0] : mhpmc4_incr[127:64];
   // rvdffe #(64)  mhpmc4h_ff (.*, .en(mhpmc4h_wr_en), .din(mhpmc4h_ns[63:0]), .dout(mhpmc4h[63:0]));

   // ----------------------------------------------------------------------
   // MHPMC5H(RW), MHPMC5(RW)
   // [63:32][31:0] : Hardware Performance Monitor Counter 5
   // `define MHPMC5 12'hB05
   // `define MHPMC5H 12'hB85
   
   // assign mhpmc5_wr_en0 = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPMC5);
   // assign mhpmc5_wr_en1 = ~perfcnt_halted & (|(mhpmc_inc_wb[2][1:0]));
   // assign mhpmc5_wr_en  = mhpmc5_wr_en0 | mhpmc5_wr_en1;
   // assign mhpmc5_incr[127:0] = {mhpmc5h[63:0],mhpmc5[63:0]} + {127'b0,mhpmc_inc_wb[2][1]} + {127'b0,mhpmc_inc_wb[2][0]}; 
   // assign mhpmc5_ns[63:0] = mhpmc5_wr_en0 ? dec_csr_wrdata_wb[63:0] : mhpmc5_incr[63:0];
   // rvdffe #(64)  mhpmc5_ff (.*, .en(mhpmc5_wr_en), .din(mhpmc5_ns[63:0]), .dout(mhpmc5[63:0]));

   // assign mhpmc5h_wr_en0 = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPMC5H);
   // assign mhpmc5h_wr_en  = mhpmc5h_wr_en0 | mhpmc5_wr_en1;
   // assign mhpmc5h_ns[63:0] = mhpmc5h_wr_en0 ? dec_csr_wrdata_wb[63:0] : mhpmc5_incr[127:64];
   // rvdffe #(64)  mhpmc5h_ff (.*, .en(mhpmc5h_wr_en), .din(mhpmc5h_ns[63:0]), .dout(mhpmc5h[63:0]));

   // ----------------------------------------------------------------------
   // MHPMC6H(RW), MHPMC6(RW)
   // [63:32][31:0] : Hardware Performance Monitor Counter 6
   // `define MHPMC6 12'hB06
   // `define MHPMC6H 12'hB86
   
   // assign mhpmc6_wr_en0 = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPMC6);
   // assign mhpmc6_wr_en1 = ~perfcnt_halted & (|(mhpmc_inc_wb[3][1:0]));
   // assign mhpmc6_wr_en  = mhpmc6_wr_en0 | mhpmc6_wr_en1;
   // assign mhpmc6_incr[127:0] = {mhpmc6h[63:0],mhpmc6[63:0]} + {127'b0,mhpmc_inc_wb[3][1]} + {127'b0,mhpmc_inc_wb[3][0]}; 
   // assign mhpmc6_ns[63:0] = mhpmc6_wr_en0 ? dec_csr_wrdata_wb[63:0] : mhpmc6_incr[63:0];
   // rvdffe #(64)  mhpmc6_ff (.*, .en(mhpmc6_wr_en), .din(mhpmc6_ns[63:0]), .dout(mhpmc6[63:0]));

   // assign mhpmc6h_wr_en0 = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPMC6H);
   // assign mhpmc6h_wr_en  = mhpmc6h_wr_en0 | mhpmc6_wr_en1;
   // assign mhpmc6h_ns[63:0] = mhpmc6h_wr_en0 ? dec_csr_wrdata_wb[63:0] : mhpmc6_incr[127:64];
   // rvdffe #(64)  mhpmc6h_ff (.*, .en(mhpmc6h_wr_en), .din(mhpmc6h_ns[63:0]), .dout(mhpmc6h[63:0]));

   // ----------------------------------------------------------------------
   // MHPME3(RW)
   // [5:0] : Hardware Performance Monitor Event 3
   // `define MHPME3 12'h323

   // // we only have 50 events, HPME* are WARL so saturate at 50
   logic [5:0] event_saturate_wb;
   assign event_saturate_wb[5:0] = ((dec_csr_wrdata_wb[5:0] > 6'd51) | (|dec_csr_wrdata_wb[31:6])) ? 6'd51 : dec_csr_wrdata_wb[5:0];
   
   // assign wr_mhpme3_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPME3);
   // rvdffs #(6)  mhpme3_ff (.*, .clk(active_clk), .en(wr_mhpme3_wb), .din(event_saturate_wb[5:0]), .dout(mhpme3[5:0]));
   // ----------------------------------------------------------------------
   // MHPME4(RW)
   // [5:0] : Hardware Performance Monitor Event 4
   // `define MHPME4 12'h324
   
   // assign wr_mhpme4_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPME4);
   // rvdffs #(6)  mhpme4_ff (.*, .clk(active_clk), .en(wr_mhpme4_wb), .din(event_saturate_wb[5:0]), .dout(mhpme4[5:0]));
   // ----------------------------------------------------------------------
   // MHPME5(RW)
   // [5:0] : Hardware Performance Monitor Event 5
   // `define MHPME5 12'h325
   
   // assign wr_mhpme5_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPME5);
   // rvdffs #(6)  mhpme5_ff (.*, .clk(active_clk), .en(wr_mhpme5_wb), .din(event_saturate_wb[5:0]), .dout(mhpme5[5:0]));
   // ----------------------------------------------------------------------
   // MHPME6(RW)
   // [5:0] : Hardware Performance Monitor Event 6
   // `define MHPME6 12'h326
   
   // assign wr_mhpme6_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MHPME6);
   // rvdffs #(6)  mhpme6_ff (.*, .clk(active_clk), .en(wr_mhpme6_wb), .din(event_saturate_wb[5:0]), .dout(mhpme6[5:0]));

   //----------------------------------------------------------------------
   // Performance Monitor Counters section ends
   //----------------------------------------------------------------------
   // ----------------------------------------------------------------------

   // MGPMC(RW)
   // [31:1] : Reserved, read 0x0
   // [0]    : Perfmon controls 0: disable perf counters 1: enable.
   //
   // Resets to 1'b1, counters enabled
   `define MGPMC 12'h7d0

   assign wr_mgpmc_wb = dec_csr_wen_wb_mod & (dec_csr_wraddr_wb[11:0] == `MGPMC);
   rvdffs #(1)  mgpmc_ff (.*, .clk(active_clk), .en(wr_mgpmc_wb), .din(~dec_csr_wrdata_wb[0]), .dout(mgpmc_b));
   assign mgpmc = ~mgpmc_b;
   

   //--------------------------------------------------------------------------------
   // trace
   //--------------------------------------------------------------------------------
   // logic usoc_tclk;
   
   // rvoclkhdr usoctrace_cgc ( .en(i0_valid_wb | exc_or_int_valid_wb | interrupt_valid_wb | dec_tlu_i0_valid_wb1 | dec_tlu_i0_exc_valid_wb1 | dec_tlu_i1_exc_valid_wb1 | dec_tlu_int_valid_wb1 | clk_override), .l1clk(usoc_tclk), .* );
   rvdffe #(10)  traceff (.*, .en(i0_valid_wb | exc_or_int_valid_wb | interrupt_valid_wb | dec_tlu_i0_valid_wb1 | dec_tlu_i0_exc_valid_wb1 | dec_tlu_i1_exc_valid_wb1 | dec_tlu_int_valid_wb1 | clk_override),  .clk(clk), 
                        .din ({i0_valid_wb, i1_valid_wb, 
                               i0_exception_valid_wb | lsu_i0_exc_wb | (i0_trigger_hit_wb & ~trigger_hit_dmode_wb), 
                               ~(i0_exception_valid_wb | lsu_i0_exc_wb | i0_trigger_hit_wb) & exc_or_int_valid_wb & ~interrupt_valid_wb,
                               exc_cause_wb[4:0],
                               interrupt_valid_wb}), 
                        .dout({dec_tlu_i0_valid_wb1, dec_tlu_i1_valid_wb1,
                               dec_tlu_i0_exc_valid_wb1, dec_tlu_i1_exc_valid_wb1,
                               dec_tlu_exc_cause_wb1[4:0],
                               dec_tlu_int_valid_wb1}));

   assign dec_tlu_mtval_wb1  = mtval[63:0];

   // end trace
   //--------------------------------------------------------------------------------

   
   // ----------------------------------------------------------------------
   // CSR read mux
   // ----------------------------------------------------------------------

   // file "csrdecode" is human readable file that has all of the CSR decodes defined and is part of git repo
   // modify this file as needed

   // to generate all the equations below from "csrdecode" except legal equation:

   // 1) coredecode -in csrdecode > corecsrdecode.e

   // 2) espresso -Dso -oeqntott corecsrdecode.e | addassign -pre out.  > csrequations

   // to generate the legal CSR equation below:

   // 1) coredecode -in csrdecode -legal > csrlegal.e

   // 2) espresso -Dso -oeqntott csrlegal.e | addassign -pre out. > csrlegal_equation


   logic csr_misa;
   logic csr_mvendorid;
   logic csr_marchid;
   logic csr_mimpid;
   logic csr_mhartid;
   logic csr_mstatus;
   logic csr_mtvec;
   logic csr_mip;
   logic csr_mie;
   // logic csr_menvcfg;
   logic csr_mcycle;
   // logic csr_mcycleh;
   logic csr_minstret;
   // logic csr_minstreth;
   logic csr_mcounteren;
   logic csr_mscratch;
   logic csr_mepc;
   logic csr_mcause;
   logic csr_mtval;
   logic csr_mrac;
   logic csr_dmst;
   logic csr_mdseac;
   logic csr_meihap;
   logic csr_meivt;
   logic csr_meipt;
   logic csr_meicurpl;
   logic csr_meicidpl;
   logic csr_dcsr;
   logic csr_mpmc;
   logic csr_mcgc;
   logic csr_mcpc;
   logic csr_mfdc;
   logic csr_dpc;
   logic csr_mtsel;
   logic csr_mtdata1;
   logic csr_mtdata2;
   logic csr_mhpmcounter;
   logic csr_mcinhibit;
   logic csr_mhpmevent;
   logic csr_mgpmc;
   logic csr_micect;
   logic csr_miccmect;
   logic csr_mdccmect;
   logic csr_dicawics;
   logic csr_dicad0;
   logic csr_dicad1;
   logic csr_dicago;
   logic presync;
   logic postsync;

   logic csr_medeleg;
   logic csr_mideleg;

   logic csr_pmpcfg0;
   logic csr_pmpcfg2;
   logic csr_pmpaddr0;
   logic csr_pmpaddr1;
   logic csr_pmpaddr2;
   logic csr_pmpaddr3;
   logic csr_pmpaddr4;
   logic csr_pmpaddr5;
   logic csr_pmpaddr6;
   logic csr_pmpaddr7;
   logic csr_pmpaddr8;
   logic csr_pmpaddr9;
   logic csr_pmpaddr10;
   logic csr_pmpaddr11;
   logic csr_pmpaddr12;
   logic csr_pmpaddr13;
   logic csr_pmpaddr14;
   logic csr_pmpaddr15;

   logic csr_pmpcfg_region;
   logic csr_pmpaddr_region;
   assign csr_pmpcfg_region = {dec_csr_rdaddr_d[11:4], dec_csr_rdaddr_d[0]} == 9'b0011_1010_0;
   assign csr_pmpaddr_region = (dec_csr_rdaddr_d[11:4] == 8'h3b) || (dec_csr_rdaddr_d[11:4] == 8'h3c) || (dec_csr_rdaddr_d[11:4] == 8'h3d) || (dec_csr_rdaddr_d[11:4] == 8'h3e);
   logic csr_priv_counter_region;
   logic csr_priv_event_region;
   assign csr_priv_counter_region = dec_csr_rdaddr_d[11:5] == 7'b1011_000 && (dec_csr_rdaddr_d[4:0] != 5'b0_0001); // except mtime
   assign csr_priv_event_region = (dec_csr_rdaddr_d[11:5] == 7'b0011_001) && (dec_csr_rdaddr_d[4:1] != 4'b0000) && (dec_csr_rdaddr_d[4:0] != 5'b0001_0); // 0x323 ~ 0x33f

   /* supervisor level csr */
   logic csr_sstatus;
   logic csr_sepc;
   logic csr_sscratch;
   logic csr_stval;
   logic csr_scause;

   /* supervisor level csr */

   
   logic csr_cycle;
   // logic csr_time;
   logic csr_instret;
   logic csr_hpmcounter;
   logic csr_counter_region;
   assign csr_unpriv_counter_region = (dec_csr_rdaddr_d[11:5] == 7'b1100_000) && (dec_csr_rdaddr_d[4:0] != 5'b0_0001); // except time

   logic csr_dscratch0, csr_dscratch1;

   // assign csr_misa = (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[6]
   //     &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[0]);


   //=========================== Machine Level CSR ==============================//
   // machine level read/write csr
   assign csr_mstatus = dec_csr_rdaddr_d[11:0] == CSR_MSTATUS;          // 0x300
   assign csr_misa = dec_csr_rdaddr_d[11:0] == CSR_MISA;                // 0x301
   assign csr_medeleg = dec_csr_rdaddr_d[11:0] == CSR_MEDELEG;          // 0x302
   assign csr_mideleg = dec_csr_rdaddr_d[11:0] == CSR_MIDELEG;          // 0x303
   assign csr_mie = dec_csr_rdaddr_d[11:0] == CSR_MIE;                  // 0x304
   assign csr_mtvec = dec_csr_rdaddr_d[11:0] == CSR_MTVEC;              // 0x305
   assign csr_mcounteren = dec_csr_rdaddr_d[11:0] == CSR_MCOUNTEREN;    // 0x306
   assign csr_mscratch = dec_csr_rdaddr_d[11:0] == CSR_MSCRATCH;        // 0x340
   assign csr_mepc = dec_csr_rdaddr_d[11:0] == CSR_MEPC;                // 0x341
   assign csr_mcause = dec_csr_rdaddr_d[11:0] == CSR_MCAUSE;            // 0x342
   assign csr_mtval = dec_csr_rdaddr_d[11:0] == CSR_MTVAL;              // 0x343
   assign csr_mip = dec_csr_rdaddr_d[11:0] == CSR_MIP;                  // 0x344

   // assign csr_menvcfg = dec_csr_rdaddr_d[11:0] == CSR_MENVCFG;          // 0x30a

   assign csr_pmpcfg0 = dec_csr_rdaddr_d[11:0] == CSR_PMPCFG0;          // 0x3a0
   // assign csr_pmpcfg1 = dec_csr_rdaddr_d[11:0] == CSR_PMPCFG1;          // 0x3a1, rv32 only
   assign csr_pmpcfg2 = dec_csr_rdaddr_d[11:0] == CSR_PMPCFG2;          // 0x3a0
   // assign csr_pmpcfg3 = dec_csr_rdaddr_d[11:0] == CSR_PMPCFG3;          // 0x3a1, rv32 only
   assign csr_pmpaddr0 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR0;
   assign csr_pmpaddr1 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR1;
   assign csr_pmpaddr2 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR2;
   assign csr_pmpaddr3 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR3;
   assign csr_pmpaddr4 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR4;
   assign csr_pmpaddr5 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR5;
   assign csr_pmpaddr6 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR6;
   assign csr_pmpaddr7 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR7;
   assign csr_pmpaddr8 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR8;
   assign csr_pmpaddr9 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR9;
   assign csr_pmpaddr10 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR10;
   assign csr_pmpaddr11 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR11;
   assign csr_pmpaddr12 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR12;
   assign csr_pmpaddr13 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR13;
   assign csr_pmpaddr14 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR14;
   assign csr_pmpaddr15 = dec_csr_rdaddr_d[11:0] == CSR_PMPADDR15;

   assign csr_mcycle = dec_csr_rdaddr_d[11:0] == CSR_MCYCLE;            // 0xb00
   // assign csr_mcycleh = dec_csr_rdaddr_d[11:0] == CSR_MCYCLEH;          // 0xb80, rv32 only
   assign csr_minstret = dec_csr_rdaddr_d[11:0] == CSR_MINSTRET;       // 0xb02
   // assign csr_minstreth = dec_csr_rdaddr_d[11:0] == CSR_MINSTRETH;      // 0xb82, rv32 only
   assign csr_mhpmcounter = dec_csr_rdaddr_d[11:0] == CSR_MHPM_COUNTER_3 |    // 0xb03
                            dec_csr_rdaddr_d[11:0] == CSR_MHPM_COUNTER_4 |    // 0xb04
                            dec_csr_rdaddr_d[11:0] == CSR_MHPM_COUNTER_5 |    // 0xb05
                            dec_csr_rdaddr_d[11:0] == CSR_MHPM_COUNTER_6 |    // 0xb06
                            dec_csr_rdaddr_d[11:0] == CSR_MHPM_COUNTER_7 |    // 0xb07
                            dec_csr_rdaddr_d[11:0] == CSR_MHPM_COUNTER_8 |    // 0xb08
                            dec_csr_rdaddr_d[11:0] == CSR_MHPM_COUNTER_9 |    // 0xb09
                            dec_csr_rdaddr_d[11:0] == CSR_MHPM_COUNTER_10;    // 0xb0a

   assign csr_mcinhibit = dec_csr_rdaddr_d[11:0] == CSR_MCOUNTINHIBIT;  // 0x320
   assign csr_mhpmevent = dec_csr_rdaddr_d[11:0] == CSR_MHPM_EVENT_3 |     // 0x323
                          dec_csr_rdaddr_d[11:0] == CSR_MHPM_EVENT_4 |     // 0x324
                          dec_csr_rdaddr_d[11:0] == CSR_MHPM_EVENT_5 |     // 0x325
                          dec_csr_rdaddr_d[11:0] == CSR_MHPM_EVENT_6 |     // 0x326
                          dec_csr_rdaddr_d[11:0] == CSR_MHPM_EVENT_7 |     // 0x327
                          dec_csr_rdaddr_d[11:0] == CSR_MHPM_EVENT_8 |     // 0x328
                          dec_csr_rdaddr_d[11:0] == CSR_MHPM_EVENT_9 |     // 0x329
                          dec_csr_rdaddr_d[11:0] == CSR_MHPM_EVENT_10;     // 0x32a

   // machine level read-only csr
   assign csr_mvendorid = dec_csr_rdaddr_d[11:0] == CSR_MVENDORID;      // 0xf11
   assign csr_marchid = dec_csr_rdaddr_d[11:0] == CSR_MARCHID;          // 0xf12
   assign csr_mimpid = dec_csr_rdaddr_d[11:0] == CSR_MIMPID;            // 0xf13
   assign csr_mhartid = dec_csr_rdaddr_d[11:0] == CSR_MHARTID;          // 0xf14

   // EH1 custom read/write csr
   assign csr_mrac = dec_csr_rdaddr_d[11:0] == `MRAC;                   // 0x7c0
   assign csr_mcpc = dec_csr_rdaddr_d[11:0] == `MCPC;                   // 0x7c2
   assign csr_dmst = 0;                                                 // not use
   assign csr_mpmc = dec_csr_rdaddr_d[11:0] == `MPMC;                   // 0x7c6
   assign csr_dicawics = dec_csr_rdaddr_d[11:0] == `DICAWICS;           // 0x7c8
   assign csr_dicad0 = dec_csr_rdaddr_d[11:0] == `DICAD0;               // 0x7c9
   assign csr_dicad1 = dec_csr_rdaddr_d[11:0] == `DICAD1;               // 0x7ca
   assign csr_dicago = dec_csr_rdaddr_d[11:0] == `DICAGO;               // 0x7cb
   assign csr_mgpmc = dec_csr_rdaddr_d[11:0] == `MGPMC;                 // 0x7d0
   assign csr_micect = dec_csr_rdaddr_d[11:0] == `MICECT;               // 0x7f0
   assign csr_miccmect = dec_csr_rdaddr_d[11:0] == `MICCMECT;           // 0x7f1
   assign csr_mdccmect = dec_csr_rdaddr_d[11:0] == `MDCCMECT;           // 0x7f2
   assign csr_mcgc = dec_csr_rdaddr_d[11:0] == `MCGC;                   // 0x7f8
   assign csr_mfdc = dec_csr_rdaddr_d[11:0] == `MFDC;                   // 0x7f9
   assign csr_meivt = dec_csr_rdaddr_d[11:0] == `MEIVT;                 // 0xbc8
   assign csr_meipt = dec_csr_rdaddr_d[11:0] == `MEIPT;                 // 0xbc9
   assign csr_meicidpl = dec_csr_rdaddr_d[11:0] == `MEICIDPL;           // 0xbcb
   assign csr_meicurpl = dec_csr_rdaddr_d[11:0] == `MEICURPL;           // 0xbcc

   // EH1 custom read-only csr
   assign csr_mdseac = dec_csr_rdaddr_d[11:0] == `MDSEAC;               // 0xfc0
   assign csr_meihap = dec_csr_rdaddr_d[11:0] == `MEIHAP;               // 0xfc8
   //=========================== Machine Level CSR ==============================//

   //========================== Supervisor Level CSR ============================//
   // Supervisor level read/write csr
   assign csr_sstatus = dec_csr_rdaddr_d[11:0] == CSR_SSTATUS;          // 0x100
   assign csr_sie = dec_csr_rdaddr_d[11:0] == CSR_SIE;                  // 0x104
   assign csr_stvec = dec_csr_rdaddr_d[11:0] == CSR_STVEC;              // 0x105
   assign csr_scounteren = dec_csr_rdaddr_d[11:0] == CSR_SCOUNTEREN;    // 0x106
   assign csr_sscratch = dec_csr_rdaddr_d[11:0] == CSR_SSCRATCH;        // 0x140
   assign csr_sepc = dec_csr_rdaddr_d[11:0] == CSR_SEPC;                // 0x141
   assign csr_scause = dec_csr_rdaddr_d[11:0] == CSR_SCAUSE;            // 0x142
   assign csr_stval = dec_csr_rdaddr_d[11:0] == CSR_STVAL;              // 0x143
   assign csr_sip = dec_csr_rdaddr_d[11:0] == CSR_SIP;                  // 0x144
   assign csr_satp = dec_csr_rdaddr_d[11:0] == CSR_SATP;                // 0x180
   //========================== Supervisor Level CSR ============================//


   //============================ User Level CSR ================================//
   // User level read-only csr
   assign csr_cycle = dec_csr_rdaddr_d[11:0] == CSR_CYCLE;              // 0xc00
   // assign csr_time = dec_csr_rdaddr_d[11:0] == CSR_TIME;                // 0xc01
   assign csr_instret = dec_csr_rdaddr_d[11:0] == CSR_INSTRET;          // 0xc02
   assign csr_hpmcounter = dec_csr_rdaddr_d[11:0] == CSR_HPM_COUNTER_3 |// 0xc03
                           dec_csr_rdaddr_d[11:0] == CSR_HPM_COUNTER_4 |// 0xc04
                           dec_csr_rdaddr_d[11:0] == CSR_HPM_COUNTER_5 |// 0xc05
                           dec_csr_rdaddr_d[11:0] == CSR_HPM_COUNTER_6 |// 0xc06
                           dec_csr_rdaddr_d[11:0] == CSR_HPM_COUNTER_7 |// 0xc07
                           dec_csr_rdaddr_d[11:0] == CSR_HPM_COUNTER_8 |// 0xc08
                           dec_csr_rdaddr_d[11:0] == CSR_HPM_COUNTER_9 |// 0xc09
                           dec_csr_rdaddr_d[11:0] == CSR_HPM_COUNTER_10;// 0xc0a

   // User level read/write csr
   assign csr_fflags = dec_csr_rdaddr_d[11:0] == CSR_FFLAGS;            // 0x001
   assign csr_frm = dec_csr_rdaddr_d[11:0] == CSR_FRM;                  // 0x002
   assign csr_fcsr = dec_csr_rdaddr_d[11:0] == CSR_FCSR;                // 0x003
   //============================ User Level CSR ================================//

   //============================ Debug Mode CSR ================================//
   assign csr_mtsel = dec_csr_rdaddr_d[11:0] == CSR_TSELECT;            // 0x7a0
   assign csr_mtdata1 = dec_csr_rdaddr_d[11:0] == CSR_TDATA1;           // 0x7a1
   assign csr_mtdata2 = dec_csr_rdaddr_d[11:0] == CSR_TDATA2;           // 0x7a2

   assign csr_dcsr = dec_csr_rdaddr_d[11:0] == CSR_DCSR;                // 0x7b0
   assign csr_dpc = dec_csr_rdaddr_d[11:0] == CSR_DPC;                  // 0x7b1
   assign csr_dscratch0 = dec_csr_rdaddr_d[11:0] == CSR_DSCRATCH0;      // 0x7b2
   assign csr_dscratch1 = dec_csr_rdaddr_d[11:0] == CSR_DSCRATCH1;      // 0x7b3
   //============================ Debug Mode CSR ================================//

   // assign presync = (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]
   //    &dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
   //    &dec_csr_rdaddr_d[4]) | (!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
   //    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
   //    &dec_csr_rdaddr_d[1]) | (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[4]
   //    &!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]) | (
   //    dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
   //    &dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[7]
   //    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
   //    &!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]);

   // assign postsync = (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]
   //    &dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[6]
   //    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[0]) | (
   //    !dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[1]
   //    &dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[4]
   //    &!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[10]
   //    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
   //    &dec_csr_rdaddr_d[1]) | (!dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[7]
   //    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[1]) | (
   //    !dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[2]
   //    &dec_csr_rdaddr_d[0]);


   // logic legal_csr;
   // assign legal_csr = (!dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
   //    &dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]
   //    &dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
   //    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]
   //    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
   //    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[1]) | (
   //    dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
   //    &dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
   //    &!dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[9]
   //    &dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]
   //    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[2]
   //    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]
   //    &dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
   //    &dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]) | (
   //    !dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
   //    &dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]
   //    &dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
   //    &!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]
   //    &dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
   //    &!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]) | (
   //    dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
   //    &dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
   //    &dec_csr_rdaddr_d[1]) | (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
   //    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[7]
   //    &!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[2]) | (
   //    !dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
   //    &dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]
   //    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]
   //    &!dec_csr_rdaddr_d[2]) | (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[9]
   //    &dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]
   //    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
   //    &dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]) | (
   //    dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
   //    &dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
   //    &dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
   //    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]
   //    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]
   //    &!dec_csr_rdaddr_d[2]) | (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
   //    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]
   //    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]
   //    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]
   //    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
   //    &dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]
   //    &dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
   //    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[0]) | (
   //    !dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
   //    &dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]
   //    &dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
   //    &!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[11]
   //    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[2]) | (
   //    !dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
   //    &dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]
   //    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
   //    &!dec_csr_rdaddr_d[2]) | (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
   //    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[7]
   //    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
   //    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[11]
   //    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[1]) | (
   //    !dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
   //    &dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]
   //    &dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[3]) | (!dec_csr_rdaddr_d[11]
   //    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
   //    &dec_csr_rdaddr_d[4]) | (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
   //    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[6]
   //    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[3]) | (dec_csr_rdaddr_d[11]
   //    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
   //    &!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]);




      
   // assign dec_tlu_presync_d = presync & dec_csr_any_unq_d & ~dec_csr_wen_unq_d;
   // assign dec_tlu_postsync_d = postsync & dec_csr_any_unq_d;
   assign dec_tlu_presync_d = 1'b0;
   assign dec_tlu_postsync_d = 1'b0;
   // assign valid_csr = ( legal_csr & (~(csr_dcsr | csr_dpc | csr_dmst | csr_dicawics | csr_dicad0 | csr_dicad1 | csr_dicago) | dbg_tlu_halted_f));
   // assign valid_csr = 1'b1;
      
   // assign dec_csr_legal_d = ( dec_csr_any_unq_d &  
   //                            valid_csr &          // of a valid CSR
   //                            ~(dec_csr_wen_unq_d & (csr_mvendorid | csr_marchid | csr_mimpid | csr_mhartid | csr_mdseac | csr_meihap)) // that's not a write to a RO CSR
   //                            ); 

   always_comb begin
      csr_access_privilege_violation = 1'b0;
      csr_access_write_violation = 1'b0;
      csr_access_read_violation = 1'b0;
      write_csr_flush_pipeline = 1'b0;
      if((priv_lvl_o & dec_csr_rdaddr_d[9:8]) != dec_csr_rdaddr_d[9:8]) begin //not conside HS
         csr_access_privilege_violation = 1'b1;
      end
      if(dec_csr_rdaddr_d[11:5] == 7'b1100_000) begin // access counter and timer: 0xC00 ~ 0xC1F
         if(priv_lvl_o == PRIV_LVL_S) begin
            csr_access_privilege_violation = ~mcounteren[dec_csr_rdaddr_d[4:0]];
         end
         if(priv_lvl_o == PRIV_LVL_U) begin
            csr_access_privilege_violation = ~(mcounteren[dec_csr_rdaddr_d[4:0]] & scounteren[dec_csr_rdaddr_d[4:0]]);
         end
      end

      if(dec_csr_wen_unq_d && dec_csr_rdaddr_d[11] && dec_csr_rdaddr_d[10]) begin // write read-only csr
         csr_access_write_violation = 1'b1;
      end
   
      if((mstatus.fs == riscv::Off) && dec_csr_any_unq_d && (csr_fflags || csr_frm || csr_fcsr)) begin //read or write float csr when Float is off
         csr_access_read_violation = 1'b1;
         csr_access_write_violation = 1'b1;
      end

      if((priv_lvl_o == PRIV_LVL_S) && mstatus.tvm && dec_csr_any_unq_d && csr_satp) begin   // TVM==1 will block access satp csr in S-mode
         csr_access_read_violation = 1'b1;
         csr_access_write_violation = 1'b1;
      end

      if(!(|{csr_fflags, csr_frm, csr_fcsr, csr_unpriv_counter_region, 
             csr_sstatus, csr_sie, csr_stvec, csr_scounteren, csr_sscratch, csr_sepc, csr_scause, csr_stval, csr_sip, csr_satp,
             csr_mvendorid, csr_marchid, csr_mimpid, csr_mhartid, csr_mstatus, csr_misa, csr_medeleg, csr_mideleg, csr_mie,
             csr_mtvec, csr_mcounteren, csr_mscratch, csr_mepc, csr_mcause, csr_mtval, csr_mip, /*csr_menvcfg,*/ 
             csr_pmpcfg_region, csr_pmpaddr_region,
             csr_priv_counter_region, csr_priv_event_region,
             csr_mtsel, csr_mtdata1, csr_mtdata2, csr_dcsr, csr_dpc, csr_dscratch0, csr_dscratch1,
             csr_mrac, csr_mcpc, csr_mpmc, csr_dicawics, csr_dicad0, csr_dicad1, csr_dicago, csr_mgpmc, 
             csr_micect, csr_miccmect, csr_mdccmect, csr_mcgc, csr_mfdc, csr_meivt, csr_meipt, csr_meicidpl, csr_meicurpl,
             csr_mdseac, csr_meihap})) begin // not a valid csr
         csr_access_read_violation = 1'b1;
      end

      if(|{csr_fflags, csr_frm, csr_fcsr, csr_satp}) begin // write those csrs will flush pipeline
         write_csr_flush_pipeline = 1'b1;
      end

   end
   assign write_csr_flush_e4 = (dec_tlu_packet_e4.pmu_i0_itype == CSRWRITE || dec_tlu_packet_e4.pmu_i0_itype == CSRRDWR) & dec_tlu_i0_valid_e4 & ~i0_trigger_hit_e4 & write_csr_flush_pipeline_f;

   always @(posedge clk, negedge rst_l) begin
      if(!rst_l) begin
         write_csr_flush_pipeline_f <= 1'b0;
      end else begin
         if(dec_csr_wen_unq_d) begin
            write_csr_flush_pipeline_f <= write_csr_flush_pipeline;
         end
      end
   end
   assign dec_csr_legal_d = ~(csr_access_privilege_violation | csr_access_write_violation | csr_access_read_violation);

   // CSR read mux
   //64位修改　添加32'b0
   assign dec_csr_rddata_d[63:0] = {64{~csr_access_read_violation}} & 
                                 ( ({64{csr_fflags}}    & {{59{1'b0}}, fcsr.fflags}) |
                                    ({64{csr_frm}}       & {{61{1'b0}}, fcsr.frm}) |
                                    ({64{csr_fcsr}}      & {{56{1'b0}}, fcsr.frm, fcsr.fflags}) |
                                    ({64{csr_misa}}      & ISA_CODE) |
                                    ({64{csr_medeleg}}   & medeleg[63:0])           |
                                    ({64{csr_mideleg}}   & mideleg[63:0])           |
                                    ({64{csr_dscratch0}}   & dscratch0[63:0])           |
                                    ({64{csr_dscratch1}}   & dscratch1[63:0])           |
                                    ({64{csr_mvendorid}} & {32'b0,32'h00000045}) |
                                    ({64{csr_marchid}}   & {32'b0,32'h0000000b}) |
                                    ({64{csr_mimpid}}    & 64'h1) |
                                    ({64{csr_mhartid}}   & hart_id_i[63:0]) |
                                    ({64{csr_mstatus}}   & mstatus ) |
                                    ({64{csr_sstatus}}   & mstatus & SSTATUS_READ_MASK ) |  
                                    ({64{csr_sie}}       & mie & mideleg) |
                                    ({64{csr_stvec}}     & stvec[63:0]) |
                                    ({64{csr_scounteren}}& {32'b0, scounteren[31:0]}) |
                                    ({64{csr_sscratch}}  & sscratch[63:0]) |
                                    ({64{csr_sepc}}      & {sepc[63:1],1'b0}) |
                                    ({64{csr_stval}}     & stval[63:0])  |
                                    ({64{csr_scause}}    & scause[63:0]) |
                                    ({64{csr_sip}}       & mip & mideleg) |
                                    ({64{csr_satp}}      & satp[63:0]) |
                                    ({64{csr_mtvec}}     & mtvec[63:0]) |
                                    ({64{csr_mip}}       & mip[63:0]) |
                                    ({64{csr_mie}}       & mie[63:0]) |
                                    // ({64{csr_menvcfg}}   & menvcfg[63:0]) |
                                    ({64{csr_mcycle}}    & mcycle[63:0]) |
                                    ({64{csr_minstret}}  & minstret[63:0]) |
                                    ({64{csr_mcounteren}}& {32'b0, mcounteren[31:0]}) |
                                    ({64{csr_cycle}}     & mcycle[63:0])   |
                                    // ({64{csr_time}}      & mcycle[63:0])   |
                                    ({64{csr_instret}}   & minstret[63:0])   |
                                    ({64{csr_mscratch}}  & mscratch[63:0]) |
                                    ({64{csr_mepc}}      & {mepc[63:1], 1'b0}) |
                                    ({64{csr_mcause}}    & mcause[63:0]) |
                                    ({64{csr_mtval}}     & mtval[63:0]) |
                                    ({64{csr_mrac}}      & mrac[63:0]) |
                                    ({64{csr_mdseac}}    & mdseac[63:0]) |
                                    ({64{csr_meivt}}     & {meivt[63:10], 10'b0}) |
                                    ({64{csr_meihap}}    & {meivt[63:10], meihap[9:2], 2'b0}) |
                                    ({64{csr_meicurpl}}  & {60'b0, meicurpl[3:0]}) |
                                    ({64{csr_meicidpl}}  & {60'b0, meicidpl[3:0]}) |
                                    ({64{csr_meipt}}     & {60'b0, meipt[3:0]}) |
                                    ({64{csr_mcgc}}      & {55'b0, mcgc[8:0]}) |
                                    ({64{csr_mfdc}}      & {45'b0, mfdc[18:0]}) |
                                    ({64{csr_dcsr}}      & {32'b0, dcsr[31:0]}) |
                                    ({64{csr_dpc}}       & {dpc[63:1], 1'b0}) |
                                    ({64{csr_dicad0}}    & dicad0[63:0]) |
   `ifdef RV_ICACHE_ECC
                                    ({64{csr_dicad1}}    & {54'b0, dicad1[9:0]}) |
   `else
                                    ({64{csr_dicad1}}    & {62'b0, dicad1[1:0]}) |
   `endif
                                    ({64{csr_dicawics}}  & {39'b0, dicawics[18], 2'b0, dicawics[17:16], 4'b0, dicawics[15:2], 2'b0}) |
                                    ({64{csr_mtsel}}     & {62'b0, mtsel[1:0]}) |
                                    ({64{csr_mtdata1}}   & {mtdata1_tsel_out[63:0]}) |
                                    ({64{csr_mtdata2}}   & {mtdata2_tsel_out[63:0]}) |
                                    ({64{csr_micect}}    & {micect[63:0]}) |
                                    ({64{csr_miccmect}}  & {miccmect[63:0]}) |
                                    ({64{csr_mdccmect}}  & {mdccmect[63:0]}) |
                                    ({64{csr_mhpmcounter}} & mhpmcounter[dec_csr_rdaddr_d[MHPMCOUNTER_INDEX_SIZE:0]-3][63:0]) |
                                    ({64{csr_hpmcounter}}& mhpmcounter[dec_csr_rdaddr_d[MHPMCOUNTER_INDEX_SIZE:0]-3][63:0]) |
                                    ({64{csr_mcinhibit}} & {32'b0, mcountinhibit[31:0]}) |
                                    ({64{csr_mhpmevent}} & mhpmevent[dec_csr_rdaddr_d[MHPMCOUNTER_INDEX_SIZE:0]-3][63:0]) |
                                    ({64{csr_mgpmc}}     & {63'b0, mgpmc}) |
                                    ({64{csr_pmpcfg0}}   & {pmpcfg[7:0]})  |
                                    ({64{csr_pmpcfg2}}   & {pmpcfg[15:8]}) |
                                    ({64{csr_pmpaddr0}}  & {10'b0, pmpaddr[0][PLEN-3:1], pmpcfg[0].addr_mode[1]})   |
                                    ({64{csr_pmpaddr1}}  & {10'b0, pmpaddr[1][PLEN-3:1], pmpcfg[1].addr_mode[1]})   |
                                    ({64{csr_pmpaddr2}}  & {10'b0, pmpaddr[2][PLEN-3:1], pmpcfg[2].addr_mode[1]})   |
                                    ({64{csr_pmpaddr3}}  & {10'b0, pmpaddr[3][PLEN-3:1], pmpcfg[3].addr_mode[1]})   |
                                    ({64{csr_pmpaddr4}}  & {10'b0, pmpaddr[4][PLEN-3:1], pmpcfg[4].addr_mode[1]})   |
                                    ({64{csr_pmpaddr5}}  & {10'b0, pmpaddr[5][PLEN-3:1], pmpcfg[5].addr_mode[1]})   |
                                    ({64{csr_pmpaddr6}}  & {10'b0, pmpaddr[6][PLEN-3:1], pmpcfg[6].addr_mode[1]})   |
                                    ({64{csr_pmpaddr7}}  & {10'b0, pmpaddr[7][PLEN-3:1], pmpcfg[7].addr_mode[1]})   |
                                    ({64{csr_pmpaddr8}}  & {10'b0, pmpaddr[8][PLEN-3:1], pmpcfg[8].addr_mode[1]})   |
                                    ({64{csr_pmpaddr9}}  & {10'b0, pmpaddr[9][PLEN-3:1], pmpcfg[9].addr_mode[1]})   |
                                    ({64{csr_pmpaddr10}} & {10'b0, pmpaddr[10][PLEN-3:1], pmpcfg[10].addr_mode[1]}) |
                                    ({64{csr_pmpaddr11}} & {10'b0, pmpaddr[11][PLEN-3:1], pmpcfg[11].addr_mode[1]}) |
                                    ({64{csr_pmpaddr12}} & {10'b0, pmpaddr[12][PLEN-3:1], pmpcfg[12].addr_mode[1]}) |
                                    ({64{csr_pmpaddr13}} & {10'b0, pmpaddr[13][PLEN-3:1], pmpcfg[13].addr_mode[1]}) |
                                    ({64{csr_pmpaddr14}} & {10'b0, pmpaddr[14][PLEN-3:1], pmpcfg[14].addr_mode[1]}) |
                                    ({64{csr_pmpaddr15}} & {10'b0, pmpaddr[15][PLEN-3:1], pmpcfg[15].addr_mode[1]})
                                    );

   /*------------------ csr update logic ----------------- */

   logic dmi_debug_req_valid;
   //waiting for a postsync instr execution finish
   assign dmi_debug_req_valid = dmi_debug_req_i;
   rvdff #(1) dmi_debug_ff1 (.*, .din(dmi_debug_req_valid), .dout(dmi_debug_req_f1));
   // rvdff #(1) dmi_debug_ff2 (.*, .din(dmi_debug_req_f1), .dout(dmi_debug_req_f2));
   // rvdff #(1) dmi_debug_ff3 (.*, .din(dmi_debug_req_f2), .dout(dmi_debug_req_f3));
   // assign dmi_debug_req_posedge = dmi_debug_req_valid & !dmi_debug_req_f1;  // t: dmi_debug_req input
   // assign dmi_debug_req_posedge_f1 = dmi_debug_req_f1 & !dmi_debug_req_f2; // t+1: flush_e4
   // assign dmi_debug_req_posedge_f2 = dmi_debug_req_f2 & !dmi_debug_req_f3; // t+2: flush_wb, store npc_wb to dpc
   assign dmi_debug_req_valid_e4 = ~debug_mode & dmi_debug_req_f1 & ~tlu_flush_lower_wb & ~freeze_dc4 & ~synchronous_flush_e4 & ~lsu_block_interrupts_i;
   rvdff #(1) dmi_debug_req_ff4 (.*, .din(dmi_debug_req_valid_e4), .dout(dmi_debug_req_valid_e5));



   always_comb begin : csr_write_blk
      privilege_level_n = privilege_level;

      mstatus_n = mstatus;
      mepc_n = mepc;
      mcause_n = mcause;
      mtval_n = mtval;
      mideleg_n = mideleg;
      medeleg_n = medeleg;
      mie_n = mie;
      mip_n = mip;
      mtvec_n = mtvec;

      // menvcfg_n = menvcfg;

      stvec_n = stvec;
      sscratch_n = sscratch;
      sepc_n = sepc;
      scause_n = scause;
      stval_n = stval;
      satp_n = satp;

      pmpcfg_n = pmpcfg;
      pmpaddr_n = pmpaddr;

      dcsr_n = dcsr;
      debug_mode_n = debug_mode;
      dpc_n = dpc;
      mrac_n = mrac;

      fcsr_n = fcsr;

      // Counter
      // User cycle counter, TODO: privilege manage
      // cycle_n = cycle;
      // if(!debug_mode) begin
      //    cycle_n = cycle + 1'b1;
      // end
      // pause counter
      mcycle_n = mcycle + 1'b1;
      if(mcountinhibit[0] || (debug_mode && dcsr.stopcount)) begin
         mcycle_n = mcycle;
      end
      // the number of instructions have retired
      minstret_n = minstret + i0_valid_wb + i1_valid_wb;
      if(mcountinhibit[2] || (debug_mode && dcsr.stopcount)) begin
         minstret_n = minstret;
      end
      // hardware performance monitor
      for(int i=0; i<MHPMCOUNTER_NUM; i++) begin
         mhpmcounter_n[i] = mhpmcounter[i] + mhpmc_inc_wb[i][0] + mhpmc_inc_wb[i][1];
         if(mcountinhibit[i+3] || (debug_mode && dcsr.stopcount)) begin
            mhpmcounter_n[i] = mhpmcounter;
         end
         mhpmevent_n[i] = mhpmevent[i];
      end

      scounteren_n = scounteren;
      mcounteren_n = mcounteren;

      fcsr_is_dirty = 1'b0;

      if(dec_csr_wen_wb_mod) begin
         case(dec_csr_wraddr_wb)
            CSR_FFLAGS: begin
               fcsr_n.fflags = dec_csr_wrdata_wb[4:0];
               fcsr_is_dirty = 1'b1;
            end
            CSR_FRM: begin
               fcsr_n.frm = dec_csr_wrdata_wb[2:0];
               fcsr_is_dirty = 1'b1;
            end
            CSR_FCSR: begin
               fcsr_n[7:0] = dec_csr_wrdata_wb[7:0];
               fcsr_is_dirty = 1'b1;
            end

            CSR_MSTATUS: begin // 0x300
               mstatus_n = dec_csr_wrdata_wb;
               mstatus_n.xs = riscv::Off;
            `ifndef EH1_RV64D
               mstatus_n.fs = riscv::Off; // now, fpu is not implemente
            `endif
            `ifndef ARA_EH1
               mstatus_n.vs = riscv::Off;
            `endif

               //S-mode and U-mode XLEN both are 64 bits
               mstatus_n.sxl = xlen_e'(2'b10);
               mstatus_n.uxl = xlen_e'(2'b10);
            end
            CSR_MEPC: begin //0x341
               mepc_n = dec_csr_wrdata_wb;
            end
            CSR_MCAUSE: begin // 0x342
               mcause_n = dec_csr_wrdata_wb;
            end
            CSR_MTVAL: begin // 0x343
               mtval_n = dec_csr_wrdata_wb;
            end
            CSR_MEDELEG: begin // 0x302
               medeleg_n = dec_csr_wrdata_wb;
            end
            CSR_MIDELEG: begin // 0x303
               mideleg_n = dec_csr_wrdata_wb;
            end
            CSR_MIE: begin // 0x304
               mie_n = (mie & ~MIE_WRITE_MASK) | (dec_csr_wrdata_wb & MIE_WRITE_MASK);
            end
            CSR_MTVEC: begin // 0x305
               mtvec_n = {dec_csr_wrdata_wb[63:2], 1'b0, dec_csr_wrdata_wb[0]};
            end
            CSR_MIP: begin // 0x344
               mip_n = (mip & ~MIP_WRITE_MASK) | (dec_csr_wrdata_wb & MIP_WRITE_MASK);
            end

            CSR_MCOUNTEREN: begin // 0x306
               mcounteren_n = dec_csr_wrdata_wb[31:0];
            end
            CSR_MCYCLE: begin // 0xb00
               mcycle_n = dec_csr_wrdata_wb;
            end
            CSR_MINSTRET: begin // 0xb02
               minstret_n = dec_csr_wrdata_wb;
            end
            CSR_MHPM_COUNTER_3, // 0xb03
            CSR_MHPM_COUNTER_4, // 0xb04
            CSR_MHPM_COUNTER_5, // 0xb05
            CSR_MHPM_COUNTER_6, // 0xb06
            CSR_MHPM_COUNTER_7, // 0xb07
            CSR_MHPM_COUNTER_8, // 0xb08
            CSR_MHPM_COUNTER_9, // 0xb09
            CSR_MHPM_COUNTER_10:// 0xb0a
            begin
               mhpmcounter_n[dec_csr_wraddr_wb[MHPMCOUNTER_INDEX_SIZE:0]-3] = dec_csr_wrdata_wb;
            end
            CSR_MHPM_EVENT_3, // 0x323
            CSR_MHPM_EVENT_4, // 0x324
            CSR_MHPM_EVENT_5, // 0x325
            CSR_MHPM_EVENT_6, // 0x326
            CSR_MHPM_EVENT_7, // 0x327
            CSR_MHPM_EVENT_8, // 0x328
            CSR_MHPM_EVENT_9, // 0x329
            CSR_MHPM_EVENT_10:// 0x32a
            begin
               mhpmevent_n[dec_csr_wraddr_wb[MHPMCOUNTER_INDEX_SIZE:0]-3] = {58'b0, dec_csr_wrdata_wb[5:0]};
            end

            CSR_SSTATUS: begin // 0x100
               // set sstatus mask
               mstatus_n = (mstatus & ~SSTATUS_WRITE_MASK) | (dec_csr_wrdata_wb & SSTATUS_WRITE_MASK);
            end
            CSR_SIE: begin // 0x104
               mie_n = (mie & ~mideleg) | (dec_csr_wrdata_wb & mideleg);
            end
            CSR_STVEC: begin // 0x105
               stvec_n = {dec_csr_wrdata_wb[63:2], 1'b0, dec_csr_wrdata_wb[0]};
            end
            CSR_SCOUNTEREN: begin // 0x106
               scounteren_n = dec_csr_wrdata_wb[31:0];
            end
            CSR_SSCRATCH: begin // 0x140
               sscratch_n = dec_csr_wrdata_wb;
            end
            CSR_SEPC: begin //0x141
               sepc_n = dec_csr_wrdata_wb;
            end
            CSR_SCAUSE: begin // 0x142
               scause_n = dec_csr_wrdata_wb;
            end
            CSR_STVAL: begin // 0x143
               stval_n = dec_csr_wrdata_wb;
            end
            CSR_SIP: begin // 0x144
               mip_n = (mip & ~(SIP_WRITE_MASK & mideleg)) | (dec_csr_wrdata_wb & (SIP_WRITE_MASK & mideleg));
            end

            CSR_SATP: begin // 0x180
               // only update if actually support this mode
               if(dec_csr_wrdata_wb[63:60] == ModeOff || dec_csr_wrdata_wb[63:60] == MODE_SV) begin
                  satp_n.mode = dec_csr_wrdata_wb[63:60];
                  satp_n.asid = {{(ASIDW-ASID_WIDTH){1'b0}}, dec_csr_wrdata_wb[44+ASID_WIDTH-1:44]};
                  satp_n.ppn = dec_csr_wrdata_wb[43:0];
               end
               // TODO: flush pipeline
            end


            // PMP CSR write logic
            // 4 PPMPCFG CSRs, only 0 and 2 CSRs are used, each PMPCFG CSR include 8 PMPCFGs because of 64 bits
            CSR_PMPCFG0: begin
               for(int i=0; i<8; i=i+1) begin
                  if(!pmpcfg[i].locked) begin // if locked, pmpcfg cannot be changed
                     pmpcfg_n[i] = dec_csr_wrdata_wb[i*8+:8];
                  end
               end
            end
            CSR_PMPCFG1: begin
               
            end
            CSR_PMPCFG2: begin
               for(int i=0; i<8; i=i+1) begin
                  if(!pmpcfg[i+8].locked) begin // if locked, pmpcfg cannot be changed
                     pmpcfg_n[i+8] = dec_csr_wrdata_wb[i*8+:8];
                  end
               end
            end
            CSR_PMPCFG3: begin
               
            end

            CSR_PMPADDR0: begin
               // if this pmpcfg is locked or next pmpcfg is locked and its addr_mode is TOR, then this pmpaddr csr cannot be changed
               if(!pmpcfg[0].locked && !(pmpcfg[1].locked && pmpcfg[1].addr_mode == TOR)) begin 
                  pmpaddr_n[0] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR1: begin
               if(!pmpcfg[1].locked && !(pmpcfg[2].locked && pmpcfg[2].addr_mode == TOR)) begin 
                  pmpaddr_n[1] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR2: begin
               if(!pmpcfg[2].locked && !(pmpcfg[3].locked && pmpcfg[3].addr_mode == TOR)) begin 
                  pmpaddr_n[2] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR3: begin
               if(!pmpcfg[3].locked && !(pmpcfg[4].locked && pmpcfg[4].addr_mode == TOR)) begin 
                  pmpaddr_n[3] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR4: begin
               if(!pmpcfg[4].locked && !(pmpcfg[5].locked && pmpcfg[5].addr_mode == TOR)) begin 
                  pmpaddr_n[4] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR5: begin
               if(!pmpcfg[5].locked && !(pmpcfg[6].locked && pmpcfg[6].addr_mode == TOR)) begin 
                  pmpaddr_n[5] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR6: begin
               if(!pmpcfg[6].locked && !(pmpcfg[7].locked && pmpcfg[7].addr_mode == TOR)) begin 
                  pmpaddr_n[6] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR7: begin
               if(!pmpcfg[7].locked && !(pmpcfg[8].locked && pmpcfg[8].addr_mode == TOR)) begin 
                  pmpaddr_n[7] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR8: begin
               if(!pmpcfg[8].locked && !(pmpcfg[9].locked && pmpcfg[9].addr_mode == TOR)) begin 
                  pmpaddr_n[8] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR9: begin
               if(!pmpcfg[9].locked && !(pmpcfg[10].locked && pmpcfg[10].addr_mode == TOR)) begin 
                  pmpaddr_n[9] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR10: begin
               if(!pmpcfg[10].locked && !(pmpcfg[11].locked && pmpcfg[11].addr_mode == TOR)) begin 
                  pmpaddr_n[10] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR11: begin
               if(!pmpcfg[11].locked && !(pmpcfg[12].locked && pmpcfg[12].addr_mode == TOR)) begin 
                  pmpaddr_n[11] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR12: begin
               if(!pmpcfg[12].locked && !(pmpcfg[13].locked && pmpcfg[13].addr_mode == TOR)) begin 
                  pmpaddr_n[12] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR13: begin
               if(!pmpcfg[13].locked && !(pmpcfg[14].locked && pmpcfg[14].addr_mode == TOR)) begin 
                  pmpaddr_n[13] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR14: begin
               if(!pmpcfg[14].locked && !(pmpcfg[15].locked && pmpcfg[15].addr_mode == TOR)) begin 
                  pmpaddr_n[14] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end
            CSR_PMPADDR15: begin
               if(!pmpcfg[15].locked) begin 
                  pmpaddr_n[15] = dec_csr_wrdata_wb[PLEN-3:0];
               end
            end


            CSR_DCSR: begin // 0x7b0
               dcsr_n = dec_csr_wrdata_wb[31:0]; // only 32-bit csr
               dcsr_n.xdebugver = 4'b0; // TODO: now not support debug
               dcsr_n.nmip = 1'b0;
               dcsr_n.stopcount= 1'b0;
               dcsr_n.stoptime = 1'b0;
            end
            CSR_DPC: begin
               dpc_n = dec_csr_wrdata_wb;
            end

            `MRAC: begin // 0x7c0
               mrac_n = mrac_in;
            end

            default: begin

            end

         endcase

         // 
      end

      // update MIP by interrupt source
      mip_n[30] = ce_int;
      // mip_n[IRQ_M_EXT] = mexintpend;
      mip_n[IRQ_M_EXT] = ext_int_sync[0];
      mip_n[IRQ_S_EXT] = ext_int_sync[1];
      mip_n[IRQ_M_TIMER] = timer_int_sync;
      mip_n[IRQ_M_SOFT] = software_int_sync;

   `ifdef EH1_RV64D
      if(fcsr_is_dirty | fp_state_is_dirty_i) begin
         mstatus_n.fs = riscv::Dirty;
      end
   `endif

   `ifdef ARA_EH1
      if(vector_state_is_dirty_i) begin
         mstatus_n.vs = riscv::Dirty;
      end
   `endif

      mstatus_n.sd = (mstatus_n.fs == riscv::Dirty) | (mstatus_n.vs == riscv::Dirty) | (mstatus_n.xs == riscv::Dirty);

      if(fpu_flags_wen_wb_i) begin
         fcsr_n.fflags = fpu_flags_wb_i[4:0] | fcsr.fflags[4:0];
      end

      // handle trap
      if(exc_or_int_valid_wb) begin
         // privilege_level_n = PRIV_LVL_M;
         // hanle trap in S mode if a trap is set in mideleg or medeleg
         // if((interrupt_valid_wb && mideleg[exc_cause_wb[4:0]]) || (!interrupt_valid_wb && medeleg[exc_cause_wb[4:0]])) begin
         //    privilege_level_n = (priv_lvl_o == PRIV_LVL_M)? PRIV_LVL_M : PRIV_LVL_S;
         // end
         privilege_level_n = trap_into_priv_lvl_wb;
         // TODO: judge exception trap in S-mode
         if(privilege_level_n == PRIV_LVL_S) begin
            mstatus_n.sie = 1'b0;
            mstatus_n.spie = mstatus.sie;
            mstatus_n.spp = privilege_level[0];

            scause_n = {interrupt_valid_wb, 58'b0, exc_cause_wb[4:0]};
            //
            sepc_n = {((interrupt_valid_wb)? npc_wb[63:1] : pc_wb[63:1]), 1'b0};
            stval_n = tval_ns;

         end else begin // privilege_level_n == PRIV_LVL_M
            mstatus_n.mie = 1'b0;
            mstatus_n.mpie = mstatus.mie;
            mstatus_n.mpp = privilege_level;

            mcause_n = {interrupt_valid_wb, 58'b0, exc_cause_wb[4:0]};
            mepc_n = {((interrupt_valid_wb)? npc_wb[63:1] : pc_wb[63:1]), 1'b0};
            mtval_n = tval_ns;

         end
      end


      // handle xret
      if(mret_wb) begin
         mstatus_n.mie = mstatus.mpie;
         privilege_level_n = mstatus.mpp;
         // set mpie = 1
         mstatus_n.mpie = 1'b1;
         // set mpp to U-mode
         mstatus_n.mpp = PRIV_LVL_U;
      end
      if(sret_wb) begin
         mstatus_n.sie = mstatus.spie;
         privilege_level_n = priv_lvl_t'({1'b0, mstatus.spp});
         mstatus_n.spie = 1'b1;
         mstatus_n.spp = 1'b0;
      end
      if(dret_wb) begin
         privilege_level_n = dcsr.prv;
         debug_mode_n = 1'b0;
      end

      // enter debug
      if(!debug_mode) begin
         // if() begin // trigger prio:4 not implemented
         if(ebreak_to_debug_mode_wb) begin // ebreak prio:3
            dcsr_n.prv = priv_lvl_o;
            dcsr_n.cause = dm::CauseBreakpoint;
            debug_mode_n = 1'b1;
            dpc_n = {pc_wb[63:1], 1'b0}; // dpc = ebreak instruction address
         end else if(dmi_debug_req_valid_e5) begin // haltreq prio:1
            dcsr_n.prv = priv_lvl_o;
            dcsr_n.cause = dm::CauseRequest;
            debug_mode_n = 1'b1;
            dpc_n = {npc_wb[63:1], 1'b0}; // dpc = next instruction address
         end else if(dcsr_single_step_done_f) begin // single step prio:0
            dcsr_n.prv = priv_lvl_o;
            dcsr_n.cause = dm::CauseSingleStep;
            debug_mode_n = 1'b1;
            dpc_n = {npc_wb[63:1], 1'b0}; // dpc = next instruction address
         end
      end

   end


   always @(posedge clk, negedge rst_l) begin
      if(!rst_l) begin
         mstatus <= {1'b1, 27'b0, 4'b1010, 13'b0, 1'b1, 3'b0, 2'b00, 2'b00, 2'b00, 9'b0}; // SD=1'b1, SXL=2'b10, UXL=2'b10,SUM=1'b1, FS=2'b00, VS=2'b00
      `ifdef EH1_RV64D
         mstatus.fs <= riscv::Initial; // support fpu, now initial status. TODO: update status
      `endif
      `ifdef ARA_EH1
         mstatus.vs <= riscv::Initial; //support vector, now initail status. TODO: update status(20250327 done)
      `endif
         mepc <= 64'b0;
         mcause <= 64'b0;
         mtval <= 64'b0;
         medeleg <= 64'b0;
         mideleg <= 64'b0;
         mie <= 64'b0;
         mip <= 64'b0;
         mtvec <= 64'b0;

         // menvcfg <= 64'b0;

         mcycle <= 64'b0;
         minstret <= 64'b0;
         mcounteren <= 32'b0;
         mcountinhibit <= 32'b0; // now mcountinhibit is always 0, means counters are always increment
         for(int i=0; i<MHPMCOUNTER_NUM; i++) begin
            mhpmcounter[i] <= 64'b0;
            mhpmevent[i]   <= 64'b0;
         end

         stvec <= 64'b0;
         scounteren <= 32'b0;
         sscratch <= 64'b0;
         sepc <= 64'b0;
         scause <= 64'b0;
         stval <= 64'b0;
         satp <= 64'b0;

         pmpcfg <= '0;
         pmpaddr <= '0;

         dcsr <= '0;
         debug_mode <= 0;
         dpc <= '0;

         mrac <= 64'h5555_5555_aaaa_aaaa;  // TODO: 40-bit address, just [39:35] = 5'b00000 cacheable

         // cycle <= 'b0;

         fcsr <= 32'b0;

      end else begin
         mstatus <= mstatus_n;
         mepc <= mepc_n;
         mcause <= mcause_n;
         mtval <= mtval_n;
         medeleg <= medeleg_n;
         mideleg <= mideleg_n;
         mie <= mie_n;
         mip <= mip_n;
         mtvec <= mtvec_n;

         // menvcfg <= menvcfg_n;

         mcycle <= mcycle_n;
         minstret <= minstret_n;
         mcounteren <= mcounteren_n;
         for(int i=0; i<MHPMCOUNTER_NUM; i++) begin
            mhpmcounter[i] <= mhpmcounter_n[i];
            mhpmevent[i]   <= mhpmevent_n[i];
         end

         stvec <= stvec_n;
         scounteren <= scounteren_n;
         sscratch <= sscratch_n;
         sepc <= sepc_n;
         scause <= scause_n;
         stval <= stval_n;
         satp <= satp_n;

         dcsr <= dcsr_n;
         debug_mode <= debug_mode_n;
         dpc <= dpc_n;
         mrac <= mrac_n;

         // cycle <= cycle_n;
         fcsr <= fcsr_n;
         for(int i=0; i<16; i++) begin
            // only support >=8-byte granularity, NA4 is not selectable
            if(pmpcfg_n[i].addr_mode != riscv::NA4 && !(pmpcfg_n[i].access_type.r == '0 && pmpcfg_n[i].access_type.w == '1)) begin
               pmpcfg[i] <= pmpcfg_n[i];
            end else begin
               pmpcfg[i] <= pmpcfg[i];
            end
            pmpaddr[i] <= pmpaddr_n[i];
         end
      end
   end
   
   /*------------------ csr write logic ----------------- */


   /* ----------------- privilege level ----------------- */
   always @(posedge clk, negedge rst_l) begin
      if(!rst_l) begin
         privilege_level <= PRIV_LVL_M;
      end else begin
         privilege_level <= privilege_level_n;
      end
   end

   /* ----------------- privilege level ----------------- */

   // TODO: other debug
   assign priv_lvl_o = (debug_mode) ? PRIV_LVL_M : privilege_level;

   // M-mode do not translate
   assign enable_fetch_translation_o = (satp.mode == MODE_SV && priv_lvl_o != PRIV_LVL_M) ? 1'b1 : 1'b0;
   // TODO: mprv 
   assign enable_ls_translation_o = (satp.mode == MODE_SV && ls_priv_lvl_o != PRIV_LVL_M) ? 1'b1 : 1'b0;

   // mprv: modify privilege of load and store
   assign ls_priv_lvl_o = (mstatus.mprv && (!debug_mode || dcsr.mprven))? mstatus.mpp : priv_lvl_o;
   // permit supervisor user memory access
   assign sum_o = mstatus.sum;
   // make executable readable
   assign mxr_o = mstatus.mxr;
   assign satp_ppn_o = satp.ppn;
   assign asid_o = satp.asid[ASID_WIDTH-1:0];

   assign mstatus_tvm_o = mstatus.tvm; // control sfence.vma and sinval.vma
   assign mstatus_tsr_o = mstatus.tsr; // control sret

   assign csr_pmpcfg_o = pmpcfg;
   assign csr_pmpaddr_o = pmpaddr;

                                  
                    
`undef MSTATUS_MIE
`undef MISA
`undef MVENDORID
`undef MARCHID
`undef MIMPID
`undef MHARTID
`undef MSTATUS
`undef MTVEC
`undef MIP
`undef MIP_MEIP
`undef MIP_MTIP
`undef MIP_MSIP
`undef MIE
`undef MIE_MEIE
`undef MIE_MTIE
`undef MCYCLEL
`undef MCYCLEH
`undef MINSTRETL
`undef MINSTRETH
`undef MSCRATCH
`undef MEPC
`undef MCAUSE
`undef MTVAL
`undef MRAC
`undef MDSEAC
`undef MEIHAP
`undef MEIPT
`undef MEICURPL

   
endmodule // dec_tlu_ctl

