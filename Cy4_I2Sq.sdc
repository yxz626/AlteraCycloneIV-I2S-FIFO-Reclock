# Cy4_I2Sq（EP4CE6F17C8）的时序约束
# 两路纯净音频主时钟：45.1584 MHz 与 49.152 MHz。
create_clock -name {clk_45m} -period 22.143 [get_ports {clk_45m}]
create_clock -name {clk_49m} -period 20.345 [get_ports {clk_49m}]
# 输入 I2S 位时钟：64 × FS，最坏情况 64 × 384 kHz = 24.576 MHz。
# native DSD 模式下同一引脚为 DCLK，最高 DSD512(48k 族) 亦为 24.576 MHz，
# 故该约束同时覆盖两种格式的最高位速率。
create_clock -name {i2s_bclk_i} -period 40.690 [get_ports {i2s_bclk_i}]

derive_pll_clocks
derive_clock_uncertainty

# 三个时钟域互相异步；设计在
# 每个跨时钟域处使用复位桥 / 格雷码同步器。
set_clock_groups -asynchronous \
    -group [get_clocks {clk_45m}] \
    -group [get_clocks {clk_49m}] \
    -group [get_clocks {i2s_bclk_i}]

# clk_sel_r 是硬件时钟 MUX 的慢速电平控制信号，不是时钟。
set_false_path -from [get_keepers {clk_sel_r*}] -to [all_registers]
# 同上，但终点是时钟输出端口：clk_sel_r 组合穿过 ALTCLKCTRL 到
# sdram_clk 并非数据路径，切换期间 clk_ena_r=0 会把 MUX 输出关断，
# 选择电平稳定后才重新使能时钟。不加这条例外时，TimeQuest 会把它
# 纳入 sdram_clk 的 8 ns tCO 约束，布局稍有变化即产生虚假负裕量。
set_false_path -from [get_keepers {clk_sel_r*}] -to [get_ports {sdram_clk}]

#------------------------------------------------------------------------------
# SDRAM 接口（W9825G6JH）
# sdram_clk = ~clk_core（半周期相位余量）。核心时钟由
# ALTCLKCTRL 从两路晶振中选一，故 IO 延迟同时参考
# 两个主时钟（用 -add_delay）；异步时钟组会自动切除
# 不可能的跨晶振族组合。若在输出端口另建生成时钟，
# 会产生虚假的反相/移相 launch-latch 组合。
#------------------------------------------------------------------------------
# 6 ns 建立 / 0 ns 保持预算覆盖 49 MHz 下的 FPGA tCO 与板级走线延迟。
set_output_delay -clock clk_49m -max 6.000 [get_ports {sdram_addr[*] sdram_ba[*] sdram_cke sdram_cs_n sdram_ras_n sdram_cas_n sdram_we_n sdram_dqm[*] sdram_data[*]}]
set_output_delay -clock clk_49m -min 0.000 -add_delay [get_ports {sdram_addr[*] sdram_ba[*] sdram_cke sdram_cs_n sdram_ras_n sdram_cas_n sdram_we_n sdram_dqm[*] sdram_data[*]}]
set_output_delay -clock clk_45m -max 6.000 -add_delay [get_ports {sdram_addr[*] sdram_ba[*] sdram_cke sdram_cs_n sdram_ras_n sdram_cas_n sdram_we_n sdram_dqm[*] sdram_data[*]}]
set_output_delay -clock clk_45m -min 0.000 -add_delay [get_ports {sdram_addr[*] sdram_ba[*] sdram_cke sdram_cs_n sdram_ras_n sdram_cas_n sdram_we_n sdram_dqm[*] sdram_data[*]}]

# 读数据在同一双向总线上返回。
set_input_delay -clock clk_49m -max 6.000 [get_ports {sdram_data[*]}]
set_input_delay -clock clk_49m -min 0.000 -add_delay [get_ports {sdram_data[*]}]
set_input_delay -clock clk_45m -max 6.000 -add_delay [get_ports {sdram_data[*]}]
set_input_delay -clock clk_45m -min 0.000 -add_delay [get_ports {sdram_data[*]}]

# 生成时钟输出本身：约束其 tCO（两种主时钟均考虑）。
set_max_delay 8.000 -from [get_clocks {clk_49m}] -to [get_ports {sdram_clk}]
set_max_delay 8.000 -from [get_clocks {clk_45m}] -to [get_ports {sdram_clk}]
set_min_delay 0.000 -from [get_clocks {clk_49m}] -to [get_ports {sdram_clk}]
set_min_delay 0.000 -from [get_clocks {clk_45m}] -to [get_ports {sdram_clk}]

#------------------------------------------------------------------------------
# I2S 输入：LRCK / SDATA 与输入 BCLK 源同步。
# DSD 输入：同两脚为 DSD_L / DSD_R，同样在 DCLK 上升沿被锁存，
#           因此使用完全相同的源同步预算。
# 接收器在 BCLK 上升沿采样；4 ns 建立 / 0.5 ns 保持是
# 40.690 ns（最坏情况）位时钟周期下的保守预算。
#------------------------------------------------------------------------------
set_input_delay -clock i2s_bclk_i -max 4.000 [get_ports {i2s_lrck_i i2s_sdata_i}]
set_input_delay -clock i2s_bclk_i -min 0.500 -add_delay [get_ports {i2s_lrck_i i2s_sdata_i}]

#------------------------------------------------------------------------------
# 非数据输入
# clk_45m / clk_49m 是送入 ALTCLKCTRL MUX 与
# 常开管理域的时钟端口（时钟路径而非数据路径）。key_k3 是
# 使用前经过同步与消抖的慢速机械按键。
# 对这三者，正确的 SDC 写法是 false-path 例外。
#------------------------------------------------------------------------------
set_false_path -from [get_ports {clk_45m clk_49m key_k3}] -to [all_registers]

#------------------------------------------------------------------------------
# 音频输出源同步时序签核
# 所有音频输出直接在 clk_core 上寄存，而 clk_core 由 ALTCLKCTRL
# 从 clk_45m 或 clk_49m 派生，故对两个主时钟都加约束。
#------------------------------------------------------------------------------
# 8 ns 板级/DAC 建立预算：对 3.3-V LVTTL 足够（tCO 约 2~6 ns 加走线）。
set_output_delay -clock clk_49m -max 8.000 [get_ports {i2s_sdata_o i2s_lrck_o do_l do_r ad_le}]
set_output_delay -clock clk_49m -min 0.000 -add_delay [get_ports {i2s_sdata_o i2s_lrck_o do_l do_r ad_le}]
set_output_delay -clock clk_45m -max 8.000 -add_delay [get_ports {i2s_sdata_o i2s_lrck_o do_l do_r ad_le}]
set_output_delay -clock clk_45m -min 0.000 -add_delay [get_ports {i2s_sdata_o i2s_lrck_o do_l do_r ad_le}]

# 时钟类输出：约束 tCO（建立）与保持下限（min delay），使
# BCLK / MCLK / AD_CLK 相对 DATA 保持确定相位，且
# 不留下未约束的 Hold 侧路径。
set_max_delay 8.000 -from [get_clocks {clk_49m}] -to [get_ports {i2s_bclk_o i2s_mclk_o ad_bclk}]
set_max_delay 8.000 -from [get_clocks {clk_45m}] -to [get_ports {i2s_bclk_o i2s_mclk_o ad_bclk}]
set_min_delay 0.000 -from [get_clocks {clk_49m}] -to [get_ports {i2s_bclk_o i2s_mclk_o ad_bclk}]
set_min_delay 0.000 -from [get_clocks {clk_45m}] -to [get_ports {i2s_bclk_o i2s_mclk_o ad_bclk}]

# LED 与 DSDOE / F0~F3 都是静态状态电平：播放期间不翻转，仅在
# 格式或速率切换时改变一次，由常开 clk_49m 域寄存输出，DAC 在
# 播放建立阶段采样，无需与音频时钟建立时序关系。
set_false_path -to [get_ports {led[*] dsd_on fsel[*]}]
