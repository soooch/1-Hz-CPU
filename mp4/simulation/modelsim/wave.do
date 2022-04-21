onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /mp4_tb/dut/cpu/clk
add wave -noupdate /mp4_tb/dut/cpu/rst
add wave -noupdate -radix hexadecimal -childformat {{{/mp4_tb/dut/cpu/rf/data[0]} -radix hexadecimal -childformat {{{/mp4_tb/dut/cpu/rf/data[0][31]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][30]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][29]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][28]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][27]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][26]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][25]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][24]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][23]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][22]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][21]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][20]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][19]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][18]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][17]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][16]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][15]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][14]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][13]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][12]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][11]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][10]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][9]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][8]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][7]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][6]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][5]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][4]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][3]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][2]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][1]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][0]} -radix hexadecimal}}} {{/mp4_tb/dut/cpu/rf/data[1]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[2]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[3]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[4]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[5]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[6]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[7]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[8]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[9]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[10]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[11]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[12]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[13]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[14]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[15]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[16]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[17]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[18]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[19]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[20]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[21]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[22]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[23]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[24]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[25]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[26]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[27]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[28]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[29]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[30]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[31]} -radix hexadecimal}} -subitemconfig {{/mp4_tb/dut/cpu/rf/data[0]} {-height 16 -radix hexadecimal -childformat {{{/mp4_tb/dut/cpu/rf/data[0][31]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][30]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][29]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][28]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][27]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][26]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][25]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][24]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][23]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][22]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][21]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][20]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][19]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][18]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][17]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][16]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][15]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][14]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][13]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][12]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][11]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][10]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][9]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][8]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][7]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][6]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][5]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][4]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][3]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][2]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][1]} -radix hexadecimal} {{/mp4_tb/dut/cpu/rf/data[0][0]} -radix hexadecimal}}} {/mp4_tb/dut/cpu/rf/data[0][31]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][30]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][29]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][28]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][27]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][26]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][25]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][24]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][23]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][22]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][21]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][20]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][19]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][18]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][17]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][16]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][15]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][14]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][13]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][12]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][11]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][10]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][9]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][8]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][7]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][6]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][5]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][4]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][3]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][2]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][1]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[0][0]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[1]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[2]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[3]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[4]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[5]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[6]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[7]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[8]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[9]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[10]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[11]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[12]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[13]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[14]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[15]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[16]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[17]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[18]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[19]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[20]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[21]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[22]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[23]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[24]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[25]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[26]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[27]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[28]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[29]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[30]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/rf/data[31]} {-height 16 -radix hexadecimal}} /mp4_tb/dut/cpu/rf/data
add wave -noupdate /mp4_tb/dut/cpu/stall
add wave -noupdate /mp4_tb/dut/cpu/mispred
add wave -noupdate -group cacheline_adapter -radix hexadecimal /mp4_tb/dut/cacheline_adaptor/line_i
add wave -noupdate -group cacheline_adapter -radix hexadecimal /mp4_tb/dut/cacheline_adaptor/line_o
add wave -noupdate -group cacheline_adapter -radix hexadecimal /mp4_tb/dut/cacheline_adaptor/address_i
add wave -noupdate -group cacheline_adapter /mp4_tb/dut/cacheline_adaptor/read_i
add wave -noupdate -group cacheline_adapter /mp4_tb/dut/cacheline_adaptor/write_i
add wave -noupdate -group cacheline_adapter /mp4_tb/dut/cacheline_adaptor/resp_o
add wave -noupdate -group cacheline_adapter -radix hexadecimal /mp4_tb/dut/cacheline_adaptor/burst_i
add wave -noupdate -group cacheline_adapter -radix hexadecimal /mp4_tb/dut/cacheline_adaptor/burst_o
add wave -noupdate -group cacheline_adapter -radix hexadecimal /mp4_tb/dut/cacheline_adaptor/address_o
add wave -noupdate -group cacheline_adapter /mp4_tb/dut/cacheline_adaptor/read_o
add wave -noupdate -group cacheline_adapter /mp4_tb/dut/cacheline_adaptor/write_o
add wave -noupdate -group cacheline_adapter /mp4_tb/dut/cacheline_adaptor/resp_i
add wave -noupdate -group arbiter -radix hexadecimal /mp4_tb/dut/arb/inst_cline_addr
add wave -noupdate -group arbiter -radix hexadecimal /mp4_tb/dut/arb/inst_cline_rdata
add wave -noupdate -group arbiter /mp4_tb/dut/arb/inst_cline_read
add wave -noupdate -group arbiter /mp4_tb/dut/arb/inst_cline_resp
add wave -noupdate -group arbiter -radix hexadecimal /mp4_tb/dut/arb/data_cline_addr
add wave -noupdate -group arbiter -radix hexadecimal /mp4_tb/dut/arb/data_cline_rdata
add wave -noupdate -group arbiter -radix hexadecimal /mp4_tb/dut/arb/data_cline_wdata
add wave -noupdate -group arbiter /mp4_tb/dut/arb/data_cline_read
add wave -noupdate -group arbiter /mp4_tb/dut/arb/data_cline_write
add wave -noupdate -group arbiter /mp4_tb/dut/arb/data_cline_resp
add wave -noupdate -group arbiter -radix hexadecimal /mp4_tb/dut/arb/pmem_address
add wave -noupdate -group arbiter -radix hexadecimal /mp4_tb/dut/arb/pmem_rdata
add wave -noupdate -group arbiter -radix hexadecimal /mp4_tb/dut/arb/pmem_wdata
add wave -noupdate -group arbiter /mp4_tb/dut/arb/pmem_read
add wave -noupdate -group arbiter /mp4_tb/dut/arb/pmem_write
add wave -noupdate -group arbiter /mp4_tb/dut/arb/pmem_resp
add wave -noupdate -group arbiter /mp4_tb/dut/arb/state
add wave -noupdate -group arbiter /mp4_tb/dut/arb/next_state
add wave -noupdate -group arbiter /mp4_tb/dut/arb/inst_sig
add wave -noupdate -group arbiter /mp4_tb/dut/arb/data_sig
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/mem_address
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/mem_rdata
add wave -noupdate -group icache /mp4_tb/dut/icache/mem_read
add wave -noupdate -group icache /mp4_tb/dut/icache/mem_resp
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/pmem_address
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/pmem_rdata
add wave -noupdate -group icache /mp4_tb/dut/icache/pmem_read
add wave -noupdate -group icache /mp4_tb/dut/icache/pmem_resp
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/address_0
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/tag_0
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/index_0
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/offset_0
add wave -noupdate -group icache /mp4_tb/dut/icache/read_0
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/address_1
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/tag_1
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/index_1
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/offset_1
add wave -noupdate -group icache /mp4_tb/dut/icache/read_1
add wave -noupdate -group icache /mp4_tb/dut/icache/valid_1
add wave -noupdate -group icache /mp4_tb/dut/icache/hit_1
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/tag_o
add wave -noupdate -group icache -radix hexadecimal /mp4_tb/dut/icache/rline
add wave -noupdate -group icache /mp4_tb/dut/icache/valids
add wave -noupdate -group fetch0 -radix hexadecimal -childformat {{{/mp4_tb/dut/cpu/pc_f0[31]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[30]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[29]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[28]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[27]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[26]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[25]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[24]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[23]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[22]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[21]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[20]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[19]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[18]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[17]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[16]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[15]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[14]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[13]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[12]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[11]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[10]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[9]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[8]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[7]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[6]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[5]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[4]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[3]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[2]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[1]} -radix hexadecimal} {{/mp4_tb/dut/cpu/pc_f0[0]} -radix hexadecimal}} -subitemconfig {{/mp4_tb/dut/cpu/pc_f0[31]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[30]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[29]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[28]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[27]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[26]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[25]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[24]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[23]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[22]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[21]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[20]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[19]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[18]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[17]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[16]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[15]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[14]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[13]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[12]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[11]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[10]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[9]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[8]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[7]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[6]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[5]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[4]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[3]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[2]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[1]} {-radix hexadecimal} {/mp4_tb/dut/cpu/pc_f0[0]} {-radix hexadecimal}} /mp4_tb/dut/cpu/pc_f0
add wave -noupdate -group fetch0 /mp4_tb/dut/cpu/bh_f0
add wave -noupdate -group fetch1 -radix hexadecimal /mp4_tb/dut/cpu/pc_f1
add wave -noupdate -group fetch1 /mp4_tb/dut/cpu/bh_f1
add wave -noupdate -group fetch1 -radix binary /mp4_tb/dut/cpu/taken_f1
add wave -noupdate -group fetch1 -radix binary /mp4_tb/dut/cpu/bht_resp_f1
add wave -noupdate -group fetch1 -radix hexadecimal /mp4_tb/dut/cpu/nxl_target
add wave -noupdate -group fetch1 -radix hexadecimal /mp4_tb/dut/cpu/btb_target
add wave -noupdate -group fetch1 -radix hexadecimal /mp4_tb/dut/cpu/ras_target
add wave -noupdate -group fetch1 -radix hexadecimal -childformat {{{/mp4_tb/dut/cpu/instr_f1[31]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[30]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[29]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[28]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[27]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[26]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[25]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[24]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[23]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[22]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[21]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[20]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[19]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[18]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[17]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[16]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[15]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[14]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[13]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[12]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[11]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[10]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[9]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[8]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[7]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[6]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[5]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[4]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[3]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[2]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[1]} -radix hexadecimal} {{/mp4_tb/dut/cpu/instr_f1[0]} -radix hexadecimal}} -subitemconfig {{/mp4_tb/dut/cpu/instr_f1[31]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[30]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[29]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[28]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[27]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[26]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[25]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[24]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[23]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[22]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[21]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[20]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[19]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[18]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[17]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[16]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[15]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[14]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[13]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[12]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[11]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[10]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[9]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[8]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[7]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[6]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[5]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[4]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[3]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[2]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[1]} {-height 16 -radix hexadecimal} {/mp4_tb/dut/cpu/instr_f1[0]} {-height 16 -radix hexadecimal}} /mp4_tb/dut/cpu/instr_f1
add wave -noupdate -group fetch1 /mp4_tb/dut/inst_resp
add wave -noupdate -group decode -radix hexadecimal /mp4_tb/dut/cpu/pc_dc
add wave -noupdate -group decode /mp4_tb/dut/cpu/bh_dc
add wave -noupdate -group decode /mp4_tb/dut/cpu/taken_dc
add wave -noupdate -group decode /mp4_tb/dut/cpu/bht_resp_dc
add wave -noupdate -group decode -radix hexadecimal /mp4_tb/dut/cpu/instr_dc
add wave -noupdate -group decode -radix hexadecimal /mp4_tb/dut/cpu/dec_target
add wave -noupdate -group decode /mp4_tb/dut/cpu/dec_redir
add wave -noupdate -group decode -childformat {{/mp4_tb/dut/cpu/ctrl_sigs_dc.rd -radix unsigned} {/mp4_tb/dut/cpu/ctrl_sigs_dc.rs1 -radix unsigned} {/mp4_tb/dut/cpu/ctrl_sigs_dc.rs2 -radix unsigned} {/mp4_tb/dut/cpu/ctrl_sigs_dc.packed_imm -radix hexadecimal}} -expand -subitemconfig {/mp4_tb/dut/cpu/ctrl_sigs_dc.rd {-height 16 -radix unsigned} /mp4_tb/dut/cpu/ctrl_sigs_dc.rs1 {-height 16 -radix unsigned} /mp4_tb/dut/cpu/ctrl_sigs_dc.rs2 {-height 16 -radix unsigned} /mp4_tb/dut/cpu/ctrl_sigs_dc.packed_imm {-height 16 -radix hexadecimal}} /mp4_tb/dut/cpu/ctrl_sigs_dc
add wave -noupdate -group queue /mp4_tb/dut/cpu/is_nop_dc
add wave -noupdate -group queue /mp4_tb/dut/cpu/needs_pc_dc
add wave -noupdate -group queue /mp4_tb/dut/cpu/push_iq0
add wave -noupdate -group queue /mp4_tb/dut/cpu/pop_iq0
add wave -noupdate -group queue /mp4_tb/dut/cpu/empty_iq0
add wave -noupdate -group queue /mp4_tb/dut/cpu/full_iq0
add wave -noupdate -group queue /mp4_tb/dut/cpu/push_pq0
add wave -noupdate -group queue /mp4_tb/dut/cpu/pop_pq0
add wave -noupdate -group queue /mp4_tb/dut/cpu/empty_pq0
add wave -noupdate -group queue /mp4_tb/dut/cpu/full_pq0
add wave -noupdate -group queue /mp4_tb/dut/cpu/can_issue
add wave -noupdate -group queue /mp4_tb/dut/cpu/needs_pc_is
add wave -noupdate -group rrd/issue -radix hexadecimal /mp4_tb/dut/cpu/pc_is
add wave -noupdate -group rrd/issue /mp4_tb/dut/cpu/bh_is
add wave -noupdate -group rrd/issue /mp4_tb/dut/cpu/ctrl_sigs_iq
add wave -noupdate -group rrd/issue -childformat {{/mp4_tb/dut/cpu/ctrl_sigs_rd.rd -radix unsigned} {/mp4_tb/dut/cpu/ctrl_sigs_rd.rs1 -radix unsigned} {/mp4_tb/dut/cpu/ctrl_sigs_rd.rs2 -radix unsigned} {/mp4_tb/dut/cpu/ctrl_sigs_rd.packed_imm -radix hexadecimal}} -subitemconfig {/mp4_tb/dut/cpu/ctrl_sigs_rd.rd {-height 16 -radix unsigned} /mp4_tb/dut/cpu/ctrl_sigs_rd.rs1 {-height 16 -radix unsigned} /mp4_tb/dut/cpu/ctrl_sigs_rd.rs2 {-height 16 -radix unsigned} /mp4_tb/dut/cpu/ctrl_sigs_rd.packed_imm {-height 16 -radix hexadecimal}} /mp4_tb/dut/cpu/ctrl_sigs_rd
add wave -noupdate -group rrd/issue -childformat {{/mp4_tb/dut/cpu/ctrl_sigs_is.rd -radix unsigned -childformat {{{[4]} -radix decimal} {{[3]} -radix decimal} {{[2]} -radix decimal} {{[1]} -radix decimal} {{[0]} -radix decimal}}} {/mp4_tb/dut/cpu/ctrl_sigs_is.rs1 -radix unsigned} {/mp4_tb/dut/cpu/ctrl_sigs_is.rs2 -radix unsigned} {/mp4_tb/dut/cpu/ctrl_sigs_is.packed_imm -radix hexadecimal}} -subitemconfig {/mp4_tb/dut/cpu/ctrl_sigs_is.rd {-height 16 -radix unsigned -childformat {{{[4]} -radix decimal} {{[3]} -radix decimal} {{[2]} -radix decimal} {{[1]} -radix decimal} {{[0]} -radix decimal}}} {/mp4_tb/dut/cpu/ctrl_sigs_is.rd[4]} {-radix decimal} {/mp4_tb/dut/cpu/ctrl_sigs_is.rd[3]} {-radix decimal} {/mp4_tb/dut/cpu/ctrl_sigs_is.rd[2]} {-radix decimal} {/mp4_tb/dut/cpu/ctrl_sigs_is.rd[1]} {-radix decimal} {/mp4_tb/dut/cpu/ctrl_sigs_is.rd[0]} {-radix decimal} /mp4_tb/dut/cpu/ctrl_sigs_is.rs1 {-height 16 -radix unsigned} /mp4_tb/dut/cpu/ctrl_sigs_is.rs2 {-height 16 -radix unsigned} /mp4_tb/dut/cpu/ctrl_sigs_is.packed_imm {-height 16 -radix hexadecimal}} /mp4_tb/dut/cpu/ctrl_sigs_is
add wave -noupdate -group rrd/issue /mp4_tb/dut/cpu/alufn_is
add wave -noupdate -group rrd/issue -radix hexadecimal /mp4_tb/dut/cpu/alu_opr1_is
add wave -noupdate -group rrd/issue -radix hexadecimal /mp4_tb/dut/cpu/alu_opr2_is
add wave -noupdate -group rrd/issue /mp4_tb/dut/cpu/lsu_ctrl_is
add wave -noupdate -group rrd/issue -radix hexadecimal /mp4_tb/dut/cpu/lsu_valu_is
add wave -noupdate -group rrd/issue -radix hexadecimal /mp4_tb/dut/cpu/lsu_base_is
add wave -noupdate -group rrd/issue -radix hexadecimal /mp4_tb/dut/cpu/lsu_ofst_is
add wave -noupdate -group rrd/issue /mp4_tb/dut/cpu/brfn_is
add wave -noupdate -group rrd/issue -radix hexadecimal /mp4_tb/dut/cpu/bru_cmp1_is
add wave -noupdate -group rrd/issue -radix hexadecimal /mp4_tb/dut/cpu/bru_cmp2_is
add wave -noupdate -group rrd/issue -radix hexadecimal /mp4_tb/dut/cpu/bru_ofst_is
add wave -noupdate -group rrd/issue -radix hexadecimal /mp4_tb/dut/cpu/bru_add1_is
add wave -noupdate -group rrd/issue -group scoreboard /mp4_tb/dut/cpu/sb0/exu_type
add wave -noupdate -group rrd/issue -group scoreboard /mp4_tb/dut/cpu/sb0/has_rd
add wave -noupdate -group rrd/issue -group scoreboard /mp4_tb/dut/cpu/sb0/has_rs1
add wave -noupdate -group rrd/issue -group scoreboard /mp4_tb/dut/cpu/sb0/has_rs2
add wave -noupdate -group rrd/issue -group scoreboard -radix unsigned /mp4_tb/dut/cpu/sb0/rd
add wave -noupdate -group rrd/issue -group scoreboard -radix unsigned /mp4_tb/dut/cpu/sb0/rs1
add wave -noupdate -group rrd/issue -group scoreboard -radix unsigned /mp4_tb/dut/cpu/sb0/rs2
add wave -noupdate -group rrd/issue -group scoreboard /mp4_tb/dut/cpu/sb0/ready
add wave -noupdate -group rrd/issue -group scoreboard /mp4_tb/dut/cpu/sb0/has_rd_wb
add wave -noupdate -group rrd/issue -group scoreboard -radix unsigned /mp4_tb/dut/cpu/sb0/rd_wb
add wave -noupdate -group rrd/issue -group scoreboard /mp4_tb/dut/cpu/can_issue
add wave -noupdate -group rrd/issue -group scoreboard /mp4_tb/dut/cpu/sb0/reg_states
add wave -noupdate -group rrd/issue -group scoreboard /mp4_tb/dut/cpu/sb0/exu_status
add wave -noupdate -group alu_exec /mp4_tb/dut/cpu/alu_has_rd_ex
add wave -noupdate -group alu_exec -radix decimal /mp4_tb/dut/cpu/alu_rd_ex
add wave -noupdate -group alu_exec /mp4_tb/dut/cpu/alufn_ex
add wave -noupdate -group alu_exec -radix hexadecimal /mp4_tb/dut/cpu/alu_opr1_ex
add wave -noupdate -group alu_exec -radix hexadecimal /mp4_tb/dut/cpu/alu_opr2_ex
add wave -noupdate -group alu_exec -radix hexadecimal /mp4_tb/dut/cpu/alu_res_ex
add wave -noupdate -group lsu_exec /mp4_tb/dut/cpu/lsu_has_rd_ex
add wave -noupdate -group lsu_exec -radix unsigned /mp4_tb/dut/cpu/lsu_rd_ex
add wave -noupdate -group lsu_exec /mp4_tb/dut/cpu/lsu_ctrl_ex
add wave -noupdate -group lsu_exec -radix hexadecimal /mp4_tb/dut/cpu/lsu_valu_ex
add wave -noupdate -group lsu_exec -radix hexadecimal /mp4_tb/dut/cpu/lsu_base_ex
add wave -noupdate -group lsu_exec -radix hexadecimal /mp4_tb/dut/cpu/lsu_ofst_ex
add wave -noupdate -group lsu_exec -radix hexadecimal /mp4_tb/dut/cpu/lsu_res_ex
add wave -noupdate -group lsu_exec -expand -group agu -radix decimal /mp4_tb/dut/cpu/agu_rd_ex
add wave -noupdate -group lsu_exec -expand -group agu /mp4_tb/dut/cpu/agu_ctrl_ex
add wave -noupdate -group lsu_exec -expand -group agu -radix hexadecimal /mp4_tb/dut/cpu/agu_valu_ex
add wave -noupdate -group lsu_exec -expand -group agu -radix hexadecimal /mp4_tb/dut/cpu/agu_base_ex
add wave -noupdate -group lsu_exec -expand -group agu -radix hexadecimal /mp4_tb/dut/cpu/agu_ofst_ex
add wave -noupdate -group lsu_exec -expand -group agu -radix hexadecimal /mp4_tb/dut/cpu/agu_addr_ex
add wave -noupdate -group lsu_exec -expand -group agu /mp4_tb/dut/cpu/agu_mbe_ex
add wave -noupdate -group lsu_exec -expand -group agu /mp4_tb/dut/cpu/agu_has_rd_ex
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/mem_address
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/mem_rdata
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache /mp4_tb/dut/dcache/mem_read
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache /mp4_tb/dut/dcache/mem_resp
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/pmem_address
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/pmem_rdata
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache /mp4_tb/dut/dcache/pmem_read
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache /mp4_tb/dut/dcache/pmem_resp
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/address_0
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/tag_0
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/index_0
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/offset_0
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache /mp4_tb/dut/dcache/read_0
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/address_1
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/tag_1
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/index_1
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/offset_1
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache /mp4_tb/dut/dcache/read_1
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache /mp4_tb/dut/dcache/valid_1
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache /mp4_tb/dut/dcache/hit_1
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/tag_o
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache -radix hexadecimal /mp4_tb/dut/dcache/rline
add wave -noupdate -group lsu_exec -expand -group mem -expand -group dcache /mp4_tb/dut/dcache/valids
add wave -noupdate -group lsu_exec -expand -group mem /mp4_tb/dut/cpu/mem_has_rd_ex
add wave -noupdate -group lsu_exec -expand -group mem -radix unsigned /mp4_tb/dut/cpu/mem_rd_ex
add wave -noupdate -group lsu_exec -expand -group mem /mp4_tb/dut/cpu/mem_ctrl_ex
add wave -noupdate -group lsu_exec -expand -group mem -radix hexadecimal /mp4_tb/dut/cpu/mem_valu_ex
add wave -noupdate -group lsu_exec -expand -group mem -radix hexadecimal /mp4_tb/dut/cpu/mem_addr_ex
add wave -noupdate -group lsu_exec -expand -group mem /mp4_tb/dut/cpu/mem_stall
add wave -noupdate -group lsu_exec -expand -group mem /mp4_tb/dut/cpu/data_read
add wave -noupdate -group lsu_exec -expand -group mem /mp4_tb/dut/cpu/data_write
add wave -noupdate -group lsu_exec -expand -group mem /mp4_tb/dut/cpu/data_resp
add wave -noupdate -group lsu_exec -expand -group mem -radix hexadecimal /mp4_tb/dut/cpu/mem_mbe_ex
add wave -noupdate -group lsu_exec -expand -group mem -radix hexadecimal /mp4_tb/dut/cpu/mem_res_ex
add wave -noupdate -group bru_exec -radix hexadecimal /mp4_tb/dut/cpu/pc_br
add wave -noupdate -group bru_exec /mp4_tb/dut/cpu/bru_bh_ex
add wave -noupdate -group bru_exec /mp4_tb/dut/cpu/bru_has_rd_ex
add wave -noupdate -group bru_exec -radix unsigned /mp4_tb/dut/cpu/bru_rd_ex
add wave -noupdate -group bru_exec /mp4_tb/dut/cpu/brfn_ex
add wave -noupdate -group bru_exec -radix hexadecimal /mp4_tb/dut/cpu/bru_cmp1_ex
add wave -noupdate -group bru_exec -radix hexadecimal /mp4_tb/dut/cpu/bru_cmp2_ex
add wave -noupdate -group bru_exec -radix hexadecimal /mp4_tb/dut/cpu/bru_ofst_ex
add wave -noupdate -group bru_exec -radix hexadecimal /mp4_tb/dut/cpu/bru_add1_ex
add wave -noupdate -group bru_exec /mp4_tb/dut/cpu/bru_taken_ex
add wave -noupdate -group bru_exec /mp4_tb/dut/cpu/bru_redir
add wave -noupdate -group bru_exec -radix hexadecimal /mp4_tb/dut/cpu/bru_res_ex
add wave -noupdate -group bru_exec -radix hexadecimal /mp4_tb/dut/cpu/bru_target
add wave -noupdate -group bru_exec /mp4_tb/dut/cpu/bru_bh_snap
add wave -noupdate -group alu_wb /mp4_tb/dut/cpu/alu_has_rd_wb
add wave -noupdate -group alu_wb -radix unsigned /mp4_tb/dut/cpu/alu_rd_wb
add wave -noupdate -group alu_wb /mp4_tb/dut/cpu/alu_res_wb
add wave -noupdate -group lsu_wb /mp4_tb/dut/cpu/lsu_has_rd_wb
add wave -noupdate -group lsu_wb -radix unsigned /mp4_tb/dut/cpu/lsu_rd_wb
add wave -noupdate -group lsu_wb -radix hexadecimal /mp4_tb/dut/cpu/lsu_res_wb
add wave -noupdate -group bru_wb /mp4_tb/dut/cpu/bru_has_rd_wb
add wave -noupdate -group bru_wb -radix unsigned /mp4_tb/dut/cpu/bru_rd_wb
add wave -noupdate -group bru_wb -radix hexadecimal /mp4_tb/dut/cpu/bru_res_wb
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {5793 ps} 0} {{Cursor 2} {2134439508 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 362
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ps} {139262 ps}
