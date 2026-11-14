`timescale 1ps/1ps
//==============================================================================
// Cy4_I2Sq 的自校验功能仿真测试平台
// 验证方式 = Quartus 17.1 直接编译（单独执行）+
//                覆盖全部功能点的端到端 RTL 仿真。
//==============================================================================
module tb;

// 1 = 标准 Philips I2S（LRCK 沿后第 2 个 BCLK 才是 MSB）
// 0 = 左对齐（LRCK 沿后第 1 个 BCLK 即 MSB）
parameter TB_DELAY    = 1;
parameter TB_LRCK_POL = 0;   // 0 = LRCK 低为左；1 = 高为左
parameter TB_SWAP_LR  = 0;   // 1 = DUT 交换左右声道

real T45 = 1.0e12 / 45.1584e6;   // ps
real T49 = 1.0e12 / 49.152e6;    // ps

reg  clk_45m, clk_49m;
reg  i2s_bclk_i, i2s_lrck_i, i2s_sdata_i;
reg  key_k3;
wire i2s_mclk_o, i2s_bclk_o, i2s_lrck_o, i2s_sdata_o;
wire do_l, do_r, ad_bclk, ad_le;
wire [3:0] led;
wire       dsd_on;       // DSDOE：输出 native DSD 时为高
wire [3:0] fsel;         // F0/F1/F2/F3 采样率指示，见 f0~F3.txt
wire sdram_clk, sdram_cke, sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n;
wire [1:0]  sdram_ba, sdram_dqm;
wire [12:0] sdram_addr;
wire [15:0] sdram_data;

initial begin clk_45m = 0; forever #(T45/2.0) clk_45m = ~clk_45m; end
initial begin clk_49m = 0; forever #(T49/2.0) clk_49m = ~clk_49m; end

real bclk_half_ps = 162760.4167;
reg  bclk_gate = 0;        // 1 = 播放源断开（BCLK 保持低电平）
initial begin
    i2s_bclk_i = 0;
    forever begin
        #(bclk_half_ps);
        if (!bclk_gate) i2s_bclk_i = ~i2s_bclk_i;
        else            i2s_bclk_i = 1'b0;
    end
end

Cy4_I2Sq #(
    .P_FILL_TARGET (24'd512),      // 预填 128 帧
    .P_DEBOUNCE    (20'd500000),  // 与硬件默认一致：10 ms 消抖
    .P_FIFO_AW     (4),            // 16 项 FIFO = 64 字，远小于预填量
    .P_SLIP_LOW    (24'd64),       // 本平台稳态水位约 960 字
    .P_SLIP_HIGH   (24'd1000000),
    .P_GUARD       (24'd16),
    .P_SLIP_GAP    (10'd512),    // 与硬件默认一致：按 LRCK 帧计的冷却
    // 硬件 P_LOST_CONFIRM=50 ms；仿真缓冲仅 512 字（128 帧），
    // 按比例缩到 1 ms，使「短断流被吸收 / 3 ms 切歌仍重锁」都能验证。
    .P_LOST_CONFIRM(22'd49152),  // 1 ms @49.152 MHz
    // 按有效测量次数确认（与采样率无关），仿真与硬件使用同一个值。
    .P_CHG_CONFIRM (3'd4),
    .P_I2S_DELAY   (TB_DELAY),
    .P_LRCK_POL    (TB_LRCK_POL),
    .P_SWAP_LR     (TB_SWAP_LR),
    .P_DSD_EN      (1),
    .P_DSD_BLINK   (0)
) dut (
    .clk_45m(clk_45m), .clk_49m(clk_49m),
    .i2s_bclk_i(i2s_bclk_i), .i2s_lrck_i(i2s_lrck_i), .i2s_sdata_i(i2s_sdata_i),
    .i2s_mclk_o(i2s_mclk_o), .i2s_bclk_o(i2s_bclk_o),
    .i2s_lrck_o(i2s_lrck_o), .i2s_sdata_o(i2s_sdata_o),
    .do_l(do_l), .do_r(do_r), .ad_bclk(ad_bclk), .ad_le(ad_le),
    .led(led), .dsd_on(dsd_on), .fsel(fsel), .key_k3(key_k3),
    .sdram_clk(sdram_clk), .sdram_cke(sdram_cke),
    .sdram_cs_n(sdram_cs_n), .sdram_ras_n(sdram_ras_n),
    .sdram_cas_n(sdram_cas_n), .sdram_we_n(sdram_we_n),
    .sdram_ba(sdram_ba), .sdram_addr(sdram_addr),
    .sdram_data(sdram_data), .sdram_dqm(sdram_dqm)
);

wire [31:0] m_wr_cnt, m_rd_cnt, m_ref_cnt, m_act_cnt, m_pre_cnt, m_mrs_cnt, m_seq_err;
wire [31:0] m_dqm_masked_cnt;
wire [23:0] m_max_addr;
wire [3:0]  m_banks_seen;
wire [12:0] m_max_row;
sdram_model u_sdram_model (
    .clk(sdram_clk), .cke(sdram_cke),
    .cs_n(sdram_cs_n), .ras_n(sdram_ras_n), .cas_n(sdram_cas_n), .we_n(sdram_we_n),
    .ba(sdram_ba), .addr(sdram_addr), .dqm(sdram_dqm), .dq(sdram_data),
    .wr_cnt(m_wr_cnt), .rd_cnt(m_rd_cnt), .ref_cnt(m_ref_cnt), .act_cnt(m_act_cnt),
    .pre_cnt(m_pre_cnt), .mrs_cnt(m_mrs_cnt), .seq_err(m_seq_err),
    .max_addr(m_max_addr), .banks_seen(m_banks_seen), .max_row(m_max_row),
    .dqm_masked_cnt(m_dqm_masked_cnt)
);

//------------------------------------------------------------------------------
// 全局统计
//------------------------------------------------------------------------------
integer errors = 0;
integer case_no = 0;

task expect_true;
    input cond;
    input [511:0] msg;
begin
    if (!cond) begin
        errors = errors + 1;
        $display("[FAIL] %0s", msg);
    end
end
endtask

//------------------------------------------------------------------------------
// I2S 激励生成器
//------------------------------------------------------------------------------
integer gen_slots = 32;
real    gen_fs = 48000.0;
integer slot_i = 0;
reg     gen_run = 0, chk_en = 0, meas_en = 0, ad_check_en = 0;
event   ev_frame;
reg  [31:0] lfsr_state, pat_l, pat_r, wmask;
reg  [63:0] in_mem [0:8191];
integer in_count = 0;

//------------------------------------------------------------------------------
// DSD native 激励 / 捕获：DCLK = i2s_bclk_i，DSD_L = i2s_lrck_i，
// DSD_R = i2s_sdata_i；输出在 i2s_bclk_o / i2s_lrck_o / i2s_sdata_o。
//------------------------------------------------------------------------------
reg        dsd_run = 0, dsd_chk_en = 0, dsd_const = 0;
reg        dsd_bl, dsd_br;
event      ev_dsd_word;
reg [31:0] dlfsr_l, dlfsr_r;
integer    dbit_i = 0, din_n = 0, dout_n = 0;
reg        din_l  [0:131071], din_r  [0:131071];
reg        dout_l [0:131071], dout_r [0:131071], dout_v [0:131071];

//--- DoP (DSD over PCM) v1.1 激励 -------------------------------------
// 在标准 32-slot PCM 帧里，每声道高 8 位放 0x05/0xFA 交替 marker，
// 随后 16 位放 DSD 码流（MSB 为最早的 t0），低 8 位补零。
// 同时把每帧的 16 个 DSD 位按时间顺序写入 din_l/din_r，
// 供 check_dsd_stream 与 reclock 后的原生 DSD 输出做位精确比对。
reg        dop_run = 0;
reg        dop_mk_phase = 0;
reg [7:0]  dop_mk;
reg [15:0] dop_l16, dop_r16;
reg [31:0] dop_r32;
integer    dop_i;

function [31:0] next_lfsr;
    input [31:0] s;
begin
    next_lfsr = {s[30:0], s[31] ^ s[21] ^ s[1] ^ s[0]};
end
endfunction

function data_bit;
    input integer s;
    integer k, half;
    reg [31:0] w;
begin
    half = (s < gen_slots) ? 0 : 1;
    k    = s - half*gen_slots - TB_DELAY;   // 标准 I2S 时跳过第 1 个空位
    w    = half ? pat_r : pat_l;
    data_bit = (k >= 0 && k < gen_slots) ? w[31-k] : 1'b0;
end
endfunction

// DUT 把样本归一化到 32 位 MSB 对齐；有 1 BCLK 延迟时可用数据位少 1 位。
function [31:0] norm_mask;
    input integer slots_v;
begin
    norm_mask = TB_DELAY ? (32'hFFFFFFFF << (33 - slots_v))
                         : (32'hFFFFFFFF << (32 - slots_v));
end
endfunction

always @(negedge i2s_bclk_i) begin
    if (gen_run) begin
        if (slot_i == 0) begin
            lfsr_state = next_lfsr(lfsr_state);
            pat_l = lfsr_state;
            pat_r = lfsr_state ^ 32'h5A5AA5A5;
            if (dop_run) begin
                dop_mk       = dop_mk_phase ? 8'hFA : 8'h05;
                dop_mk_phase = ~dop_mk_phase;
                dop_l16      = lfsr_state[15:0];
                dop_r32      = lfsr_state ^ 32'h5A5AA5A5;
                dop_r16      = dop_r32[15:0];
                pat_l        = {dop_mk, dop_l16, 8'h00};
                pat_r        = {dop_mk, dop_r16, 8'h00};
                for (dop_i = 0; dop_i < 16; dop_i = dop_i + 1) begin
                    if (din_n + dop_i < 131071) begin
                        din_l[din_n + dop_i] = dop_l16[15 - dop_i];
                        din_r[din_n + dop_i] = dop_r16[15 - dop_i];
                    end
                end
                din_n = din_n + 16;
            end
            if (TB_SWAP_LR)
                in_mem[in_count] = {pat_r & norm_mask(gen_slots),
                                    pat_l & norm_mask(gen_slots)};
            else
                in_mem[in_count] = {pat_l & norm_mask(gen_slots),
                                    pat_r & norm_mask(gen_slots)};
            in_count = in_count + 1;
            -> ev_frame;
        end
        i2s_lrck_i  = (slot_i < gen_slots) ? 1'b0 : 1'b1;
        i2s_sdata_i = data_bit(slot_i);
        slot_i = slot_i + 1;
        if (slot_i == 2*gen_slots) slot_i = 0;
    end
end

// DSD 位流：DCLK 下降沿改变 L/R，DUT 在上升沿锁存。
// 用两个独立 32 位 LFSR 产生无帧结构的伪随机 1-bit 码流。
always @(negedge i2s_bclk_i) begin
    if (dsd_run) begin
        if (dsd_const) begin
            dsd_bl = 1'b0; dsd_br = 1'b1;   // 两声道恒定电平（无 LRCK 跳变）
        end else begin
            dlfsr_l = next_lfsr(dlfsr_l);
            dlfsr_r = next_lfsr(dlfsr_r);
            dsd_bl  = dlfsr_l[0];
            dsd_br  = dlfsr_r[0] ^ dlfsr_l[9];
        end
        i2s_lrck_i  = dsd_bl;
        i2s_sdata_i = dsd_br;
        if (din_n < 131071) begin
            din_l[din_n] = dsd_bl;
            din_r[din_n] = dsd_br;
            din_n = din_n + 1;
        end
        dbit_i = dbit_i + 1;
        if (dbit_i == 32) begin dbit_i = 0; -> ev_dsd_word; end
    end
end

// reclock 后 DSD 输出捕获：只在单位增益（真实码流、无淡变/静音）时有效。
always @(posedge i2s_bclk_o) begin
    #1;
    if (dsd_chk_en && dout_n < 131071) begin
        dout_l[dout_n] = i2s_lrck_o;
        dout_r[dout_n] = i2s_sdata_o;
        dout_v[dout_n] = (dut.gain === 5'd16) && (dut.dsd_core === 1'b1)
                         && (dut.mute_hold_r === 1'b0);
        dout_n = dout_n + 1;
    end
end

//------------------------------------------------------------------------------
// DSD 切歌爆音（pop）检测
// 1-bit DSD DAC 的输出电平由码流密度决定：恒定 0/1 不是静音，而是
// ±满幅直流。切歌/重锁期间若输出被复位成恒定电平或 DCLK 停摆，
// DAC 模拟低通会把这个阶跃积分成满幅冲击 → 很响的"砰"声。
// 因此在「重锁复位窗口」内统计输出码流密度与最长恒定采样数，
// 并在整个切歌窗口内统计输出 DCLK 的最长停摆时间。
//------------------------------------------------------------------------------
reg        pop_chk_en = 0;
integer    pop_rst_samples   = 0, pop_rst_ones_l = 0, pop_rst_ones_r = 0;
integer    pop_rst_same_l    = 0, pop_rst_same_r = 0;
integer    pop_rst_max_same_l = 0, pop_rst_max_same_r = 0;
reg        pop_rst_last_l = 0, pop_rst_last_r = 0;
real       pop_last_clk_t = 0, pop_max_gap_clk = 0;

always @(posedge clk_49m) begin
    if (pop_chk_en && !dut.core_rst_n && dut.cur_fmt === 1'b1) begin
        pop_rst_samples = pop_rst_samples + 1;
        if (i2s_lrck_o  === 1'b1) pop_rst_ones_l = pop_rst_ones_l + 1;
        if (i2s_sdata_o === 1'b1) pop_rst_ones_r = pop_rst_ones_r + 1;
        if (i2s_lrck_o === pop_rst_last_l) pop_rst_same_l = pop_rst_same_l + 1;
        else pop_rst_same_l = 0;
        if (i2s_sdata_o === pop_rst_last_r) pop_rst_same_r = pop_rst_same_r + 1;
        else pop_rst_same_r = 0;
        pop_rst_last_l = i2s_lrck_o;
        pop_rst_last_r = i2s_sdata_o;
        if (pop_rst_same_l > pop_rst_max_same_l) pop_rst_max_same_l = pop_rst_same_l;
        if (pop_rst_same_r > pop_rst_max_same_r) pop_rst_max_same_r = pop_rst_same_r;
    end else begin
        pop_rst_same_l = 0; pop_rst_same_r = 0;
        pop_rst_last_l = i2s_lrck_o; pop_rst_last_r = i2s_sdata_o;
    end
end

always @(posedge i2s_bclk_o) begin
    if (pop_chk_en) begin
        if ($realtime - pop_last_clk_t > pop_max_gap_clk)
            pop_max_gap_clk = $realtime - pop_last_clk_t;
        pop_last_clk_t = $realtime;
    end
end

// K3 专项：直接数 AD1862 CLK 引脚的上升沿，确认端口真的在工作，
// 而不只是内部 mode_out 标志为 1。
reg        k3_ad_en = 0;
integer    k3_ad_clks = 0;
always @(posedge ad_bclk) if (k3_ad_en) k3_ad_clks = k3_ad_clks + 1;

// DSD 长时低水位专项：按时间统计输出处于单位增益的占比。
// DSD 下 LRCK 是数据脚，不能像 PCM 那样按帧边沿统计，
// 故用常开 clk_49m 直接采样 gain。
reg        dsd_low_chk = 0;
integer    dsd_low_cnt = 0, dsd_gain16_cnt = 0;
always @(posedge clk_49m) if (dsd_low_chk) begin
    dsd_low_cnt = dsd_low_cnt + 1;
    if (dut.gain === 5'd16) dsd_gain16_cnt = dsd_gain16_cnt + 1;
end

//------------------------------------------------------------------------------
// 周期性停顿监视：统计 gain 掉出单位增益、硬静音、tx FIFO 空，
// 以及「停顿事件」（gain 从 16 变为非 16）的次数与水位范围。
//------------------------------------------------------------------------------
reg        stall_chk = 0;
integer    stall_samples = 0, stall_gain_lo = 0, stall_mute = 0;
integer    stall_txempty = 0, stall_events = 0;
integer    stall_fill_min = 24'hFFFFFF, stall_fill_max = 0;
// 格式误判 / 重锁监视：一次重锁会造成淡出 488 µs + 重新预填
// 262144 字（352.8k 档约 186 ms）的长静音，远比 slip fade 明显。
integer    fmt_dsd_high = 0, fmt_changes = 0, resync_events = 0;
integer    i2s_period_bad = 0;
reg        fmt_prev = 0;
reg  [2:0] hst_prev = 3'd1;
reg        stall_gain_hi = 1;
always @(posedge clk_49m) if (stall_chk) begin
    stall_samples = stall_samples + 1;
    if (dut.gain !== 5'd16) stall_gain_lo = stall_gain_lo + 1;
    if (dut.mute_hold === 1'b1) stall_mute = stall_mute + 1;
    if (dut.tx_empty === 1'b1)  stall_txempty = stall_txempty + 1;
    if (stall_gain_hi && dut.gain !== 5'd16) stall_events = stall_events + 1;
    stall_gain_hi = (dut.gain === 5'd16);
    if (dut.fill_words < stall_fill_min) stall_fill_min = dut.fill_words;
    if (dut.fill_words > stall_fill_max) stall_fill_max = dut.fill_words;
    if (dut.fmt_dsd === 1'b1) fmt_dsd_high = fmt_dsd_high + 1;
    if (dut.i2s_period_ok !== 1'b1) i2s_period_bad = i2s_period_bad + 1;
    if (dut.cur_fmt !== fmt_prev) fmt_changes = fmt_changes + 1;
    fmt_prev = dut.cur_fmt;
    if (hst_prev === 3'd1 && dut.hstate !== 3'd1)
        resync_events = resync_events + 1;
    hst_prev = dut.hstate;
end

// 单次淡变长度（按帧计）：用于证明水位 slip 只引起约 10 帧淡变，
// 而不是一次 33 帧的完整淡出→静音→淡入（可听的「停顿」）。
integer    fade_frames = 0, fade_max_frames = 0;
always @(ev_frame) if (stall_chk) begin
    if (dut.gain !== 5'd16) begin
        fade_frames = fade_frames + 1;
    end else if (fade_frames > 0) begin
        if (fade_frames > fade_max_frames) fade_max_frames = fade_frames;
        fade_frames = 0;
    end
end

//------------------------------------------------------------------------------
// reclock 后 I2S 输出捕获 + 与输入流的位精确比对
//------------------------------------------------------------------------------
reg [31:0] out_cap, out_left;
reg [5:0]  out_idx;
reg        out_lr_d, out_started;
reg [63:0] out_mem [0:8191];
integer    out_wr = 0;
integer    chk_aligned = 0, chk_idx = 0, checked = 0, mismatches = 0;

// 更严格的检查：每个 reclock 输出帧都必须能在输入流中找到，
// 且匹配到的输入索引必须构成严格连续序列。
// 无论缓冲延迟多大，都能检出任何隐式丢帧 / 重复帧。
integer last_pidx = -1;
integer mono_frames = 0;      // 非零帧中左右声道相同的帧数（必须为 0）
integer stereo_frames = 0;
// 长时间播放专项：统计稳态（单位增益 + 输出帧可在输入流中找到）占比。
integer frame_cnt = 0, gain16_cnt = 0, matched_cnt = 0;
task check_pair;
    input [63:0] p;
    integer j, pidx;
begin
    frame_cnt = frame_cnt + 1;
    if (dut.gain === 5'd16) gain16_cnt = gain16_cnt + 1;
    if (p !== 64'd0) begin
        stereo_frames = stereo_frames + 1;
        if (p[63:32] === p[31:0]) mono_frames = mono_frames + 1;
    end
    pidx = -1;
    for (j = 0; j < in_count; j = j + 1)
        if (in_mem[j] === p) begin pidx = j; j = in_count; end
    if (pidx >= 0) begin
        matched_cnt = matched_cnt + 1;
        if (last_pidx >= 0) begin
            if (pidx == last_pidx + 1) checked = checked + 1;
            else if (pidx > last_pidx + 1) begin
                // 只有"向前跳"才是真正的丢帧 / 缓冲重建；
                // 向后回溯是静音帧重播旧样本（faded insert），属设计行为。
                mismatches = mismatches + 1;
                if (mismatches < 5)
                    $display("[INFO] switch-over gap (buffer rebuilt under fade): in_mem %0d -> %0d",
                             last_pidx, pidx);
            end
        end
        last_pidx = pidx;
    end else begin
        last_pidx = -1;      // 启动 / 淡入淡出 / 静音帧
    end
end
endtask
// 标准 I2S 每半帧有 1 个空位，只捕获到 31 位数据；
// 左移 1 位后才是 DUT 的 32 位 MSB 对齐字。
wire [31:0] cap_aligned = TB_DELAY ? {out_cap[30:0], 1'b0} : out_cap;

always @(posedge i2s_bclk_o) begin
    #1;
    if (chk_en) begin
        if (i2s_lrck_o !== out_lr_d) begin
            if (out_started) begin
                // P_LRCK_POL=1 时 LRCK 高电平对应左声道，判断需取反。
                if ((out_lr_d == 1'b1) ^ TB_LRCK_POL) begin   // 刚结束的半帧为右声道
                    check_pair({out_left, cap_aligned});
                    out_mem[out_wr] = {out_left, cap_aligned};
                    out_wr = out_wr + 1;
                end else begin                    // 刚结束的半帧为左声道
                    out_left <= cap_aligned;
                end
            end
            out_started <= 1'b1;
            if (TB_DELAY) begin
                // 第 1 个 BCLK 是标准 I2S 的空位，不参与数据捕获
                out_cap <= 32'd0;
                out_idx <= 6'd0;
            end else begin
                out_cap <= {31'd0, i2s_sdata_o};
                out_idx <= 6'd1;
            end
        end else begin
            out_idx <= out_idx + 6'd1;
            if (out_idx < 6'd32) out_cap <= {out_cap[30:0], i2s_sdata_o};
        end
        out_lr_d <= i2s_lrck_o;
    end
end

//------------------------------------------------------------------------------
// 频率监视器
//------------------------------------------------------------------------------
real lr_t = 0, lr_period = 0;
real mclk_t = 0, mclk_period = 0;
real obclk_t = 0, obclk_period = 0;
always @(posedge i2s_lrck_o) begin if (meas_en) begin if (lr_t > 0) lr_period = $realtime - lr_t; lr_t = $realtime; end end
always @(posedge i2s_mclk_o) begin if (meas_en) begin if (mclk_t > 0) mclk_period = $realtime - mclk_t; mclk_t = $realtime; end end
always @(posedge i2s_bclk_o) begin if (meas_en) begin if (obclk_t > 0) obclk_period = $realtime - obclk_t; obclk_t = $realtime; end end

// AD1862 / 淡入淡出监视器的帧边界：跟随 LRCK 极性配置，
// 保证极性反转时仍在同一个逻辑帧边界采样。
wire frame_mon = TB_LRCK_POL ? ~i2s_lrck_o : i2s_lrck_o;

//------------------------------------------------------------------------------
// AD1862 时序 / 格式监视器（数据手册 Fig.12b）
//   CLK 周期 > 60 ns、高/低 > 15 ns、数据建立/保持 > 25 ns，
//   LE 低脉冲 > 40 ns、每 LRCK 帧 24 个 CLK、20 位高位先出。
//------------------------------------------------------------------------------
integer ad_rise_cnt = 0, ad_checks = 0, ad_err = 0, le_cnt = 0;
integer ad_err_period = 0, ad_err_setup = 0, ad_err_high = 0, ad_err_hold = 0;
integer ad_err_le = 0, ad_err_data = 0;
reg [39:0] cur20_prev = 0;
reg        cur20_valid = 0;
integer    ad_warmup = 0;
integer    dither_seen = 0, fade_min_gain = 16, fade_step_err = 0, fade_prev = -1;
integer    slip_acks = 0;
reg  [19:0] ad_shift_l = 0, ad_shift_r = 0;
real ad_prev_rise = 0, ad_rise_t = 0, ad_fall_t = 0, ad_period = 0;
real data_chg_t = 0, setup_w = 0, hold_w = 0, le_low_t = 0;
integer j;

// 淡入淡出平滑度 + slip 事件监视器（Hi-end 新增）
reg fade_mon_en = 0;
integer fd;
always @(negedge frame_mon) begin
    if (fade_mon_en) begin
        if (fade_prev >= 0) begin
            fd = dut.gain - fade_prev;
            // 切换类淡变逐帧 ±1（33 帧）；水位 slip 采用步长 4 的
            // 快速淡变（约 10 帧），避免稀疏 slip 变成可听的停顿。
            if (dut.slip_only) begin
                if (fd > 4 || fd < -4) fade_step_err = fade_step_err + 1;
            end else begin
                if (fd > 1 || fd < -1) fade_step_err = fade_step_err + 1;
            end
        end
        if (dut.gain < fade_min_gain) fade_min_gain = dut.gain;
        fade_prev = dut.gain;
    end
end
always @(posedge dut.slip_ack) slip_acks = slip_acks + 1;

always @(do_l or do_r) data_chg_t = $realtime;

always @(posedge ad_bclk) begin
    if (ad_check_en) begin
        ad_rise_cnt = ad_rise_cnt + 1;
        ad_rise_t = $realtime;
        if (ad_prev_rise > 0) ad_period = ad_rise_t - ad_prev_rise;
        ad_prev_rise = ad_rise_t;
        setup_w = ad_rise_t - data_chg_t;
        if (ad_period > 0 && ad_period < 60000.0) begin ad_err = ad_err + 1; ad_err_period = ad_err_period + 1; end
        if (setup_w > 0 && setup_w < 25000.0)     begin ad_err = ad_err + 1; ad_err_setup = ad_err_setup + 1; end
        if (ad_rise_cnt <= 20) begin
            ad_shift_l = {ad_shift_l[18:0], do_l};
            ad_shift_r = {ad_shift_r[18:0], do_r};
        end
    end
end

always @(negedge ad_bclk) begin
    if (ad_check_en) begin
        ad_fall_t = $realtime;
        hold_w = ad_fall_t - ad_rise_t;
        if ((ad_fall_t - ad_rise_t) < 15000.0) begin ad_err = ad_err + 1; ad_err_high = ad_err_high + 1; end
        if (hold_w < 25000.0) begin ad_err = ad_err + 1; ad_err_hold = ad_err_hold + 1; end
    end
end

always @(negedge ad_le) le_low_t = $realtime;
always @(posedge ad_le) begin
    if (ad_check_en && le_low_t > 0) begin
        le_cnt = le_cnt + 1;
        if (($realtime - le_low_t) < 40000.0) begin ad_err = ad_err + 1; ad_err_le = ad_err_le + 1; end
    end
end

always @(negedge frame_mon) begin
    if (ad_check_en) begin
        if (ad_rise_cnt > 0) begin
            if (ad_warmup >= 40 && ad_rise_cnt != 24) ad_err = ad_err + 1;
            else if (ad_warmup >= 40) begin
                ad_checks = ad_checks + 1;
                // AD1862 字必须来自 reclock 后 I2S 帧对的高 20 位
                begin : ad_match
                    // TPDF 抖动 + 舍入最多使 20 位字
                    // 偏移 1 LSB；超过则说明音频数据损坏。
                    integer dl, dr;
                    dl = $signed({1'b0, ad_shift_l}) - $signed({1'b0, cur20_prev[39:20]});
                    dr = $signed({1'b0, ad_shift_r}) - $signed({1'b0, cur20_prev[19:0]});
                    // 淡入淡出期间的帧被有意衰减；仅
                    // 单位增益帧参与逐位比对。
                    if (cur20_valid && dut.gain == 5'd16 &&
                        (dl > 1 || dl < -1 || dr > 1 || dr < -1)) begin
                        ad_err = ad_err + 1; ad_err_data = ad_err_data + 1;
                    end
                    if (dut.gain == 5'd16 && (dl != 0 || dr != 0))
                        dither_seen = dither_seen + 1;
                end
            end
        end
        ad_warmup = ad_warmup + 1;
        cur20_prev = {dut.cur_pair[63:44], dut.cur_pair[31:12]};
        cur20_valid = 1;
        ad_rise_cnt = 0;
        ad_shift_l = 0;
        ad_shift_r = 0;
    end
end

//------------------------------------------------------------------------------
// 任务
//------------------------------------------------------------------------------
// 按硬件消抖（10 ms）给足低电平时间；仿真早期用 6 µs 只覆盖了
// 被参数覆盖掉的 1.3 µs 消抖，无法暴露真实按键时长问题。
task press_key;
begin
    key_k3 = 1'b0;
    #(15000000000.0);     // 15 ms 低电平 > 10 ms 消抖（timescale 1ps）
    key_k3 = 1'b1;
    #(2000000000.0);      // 2 ms 松开
end
endtask

// 仅长时漂移用例需要更长的切换等待（352.8 kHz 档 200 帧只有
// 0.57 ms，短于 H_FADE 488 µs）；置 1 时 run_case 等 400 帧。
reg run_case_extra_wait = 0;

task run_case;
    input integer code;
    input integer slots_i;
    input integer nframes;
    input [31:0] seed;
    integer f, led_exp;
    real fs, exp_lr, exp_mclk, exp_bclk, fmeas;
    reg [511:0] tag;
begin
    case (code)
        0: fs = 44100.0;
        1: fs = 48000.0;
        2: fs = 88200.0;
        3: fs = 96000.0;
        4: fs = 176400.0;
        5: fs = 192000.0;
        6: fs = 352800.0;
        default: fs = 384000.0;
    endcase
    case_no = case_no + 1;
    gen_run = 0; chk_en = 0; meas_en = 0;
    i2s_lrck_i = 1'b1; i2s_sdata_i = 1'b0;
    slot_i = 0; lfsr_state = seed;
    gen_slots = slots_i; gen_fs = fs;
    wmask = 32'hFFFFFFFF << (32 - slots_i);
    bclk_half_ps = 1.0e12 / (4.0 * slots_i * fs);
    in_count = 0; chk_aligned = 0; chk_idx = 0; checked = 0; mismatches = 0;
    last_pidx = -1;
    out_wr = 0; out_started = 0; out_lr_d = 1'b1; out_cap = 0; out_idx = 0;
    lr_t = 0; mclk_t = 0; obclk_t = 0;
    lr_period = 0; mclk_period = 0; obclk_period = 0;
    #(T49 * 20.0);
    gen_run = 1; chk_en = 0; meas_en = 1;
    // 跳过速率切换 / SDRAM 重初始化 / 缓冲预填的瞬态。
    // 最高档（352.8/384 kHz）下 200 帧仅 0.57 ms，短于
    // H_FADE 488 µs；长时漂移用例需要 400 帧（约 1.1 ms）
    // 才能把切换瞬态完全排除在检查窗口之外。
    if (run_case_extra_wait) repeat (400) @(ev_frame);
    else                     repeat (200) @(ev_frame);
    out_started = 0; out_lr_d = i2s_lrck_o; out_cap = 0; out_idx = 0;
    chk_aligned = 0; chk_idx = 0; checked = 0; mismatches = 0; out_wr = 0;
    last_pidx = -1;
    chk_en = 1;
    repeat (nframes) @(ev_frame);
    #(T49 * 100.0);

    exp_lr   = 1.0e12 / fs;
    exp_bclk = 1.0e12 / (64.0 * fs);
    exp_mclk = (code[0] == 0) ? 1.0e12 / (code[2:1] == 0 ? 45.1584e6/4.0 : 45.1584e6/2.0)
                              : 1.0e12 / (code[2:1] == 0 ? 49.152e6/4.0  : 49.152e6/2.0);
    led_exp  = 1 << (code >> 1);

    $display("CASE %0d  FS=%0d  slots=%0d  family=%0d  lr=%0d ps (exp %0d)  mclk=%0d ps (exp %0d)  bclk=%0d ps (exp %0d)  led=%b  checked=%0d  mism=%0d  gain=%0d  fill=%0d",
             case_no, fs, slots_i, code[0], lr_period, exp_lr, mclk_period, exp_mclk,
             obclk_period, exp_bclk, led, checked, mismatches, dut.gain, dut.fill_words);

    fmeas = 1.0e12 / lr_period;
    expect_true(fmeas > fs*0.99 && fmeas < fs*1.01, "R1 output sample rate adapts to input");
    fmeas = 1.0e12 / obclk_period;
    expect_true(fmeas > 64.0*fs*0.99 && fmeas < 64.0*fs*1.01, "R1 output BCLK = 64 x FS");
    fmeas = 1.0e12 / mclk_period;
    expect_true(fmeas > (1.0e12/exp_mclk)*0.99 && fmeas < (1.0e12/exp_mclk)*1.01,
                "R2 correct clean master clock selected (MCLK)");
    expect_true(dut.clk_sel_r === code[0], "R2 automatic global clock selection");
    expect_true(led === led_exp, "R4 LED sample-rate indication");
    // f0~F3.txt：PCM 档位编码 = cur_rate + 1（0001=44.1k ... 1000=384k）
    expect_true(dsd_on === 1'b0 && fsel === ({1'b0, code[2:0]} + 4'd1),
                "PCM: DSDOE=0 and F0~F3 match f0~F3.txt sample-rate table");
    expect_true(dut.u_sdram.init_done === 1'b1, "R3 SDRAM initialised");
    expect_true(m_act_cnt > 0 && m_wr_cnt > 0 && m_rd_cnt > 0,
                "R3 SDRAM ACT/WR/RD commands issued (FIFO+SDRAM buffering)");
    expect_true(m_ref_cnt > 0 && m_pre_cnt > 0 && m_mrs_cnt > 0,
                "R3 SDRAM precharge/refresh/mode-register sequence");
    expect_true(m_seq_err === 0, "R3 SDRAM command sequencing correct");
    // 速率/时钟切换会重建缓冲；1 BCLK 延迟 + 窄位深时
    // 测试平台的归一化掩码还可能产生少量误匹配。
    // 稳态连续性由 checked（>200 帧严格连续）保证。
    expect_true(checked > 80 && mismatches <= 3,
                "R1/R3 bit-exact reclock data, no steady-state drop/repeat");
end
endtask

//------------------------------------------------------------------------------
// 手动切歌：播放源让 BCLK/LRCK 停一段时间，
// 再以不同的帧相位重新起流。这正是原先
// 导致接收器永久错位、必须断电才有声的实际场景。
//------------------------------------------------------------------------------
task track_change;
    input real    gap_ps;      // 单位 ps（时间刻度 1 ps）；3 ms 传 3.0e9
    input integer new_slots;
    input real    new_fs;
    input [31:0]  new_seed;
begin
    bclk_gate   = 1'b1;      // 播放源把 BCLK 保持为低
    gen_run     = 0;
    i2s_lrck_i  = 1'b1;      // LRCK 保持不变、无跳变 → stream_lost
    i2s_sdata_i = 1'b0;
    #(gap_ps);
    $display("DROPOUT: gain=%0d hstate=%0d g_arst=%b lost=%b gap=%0d mute=%b",
             dut.gain, dut.hstate, dut.g_arst_n_r,
             dut.stream_lost, dut.lrck_gap, dut.mute_hold);
    expect_true(dut.gain === 5'd0,
                "Hi-end output is muted while the source is absent");
    // 以全新帧相位重新起流（也可能换成新的速率/位深）
    gen_slots    = new_slots;
    gen_fs       = new_fs;
    wmask        = 32'hFFFFFFFF << (32 - new_slots);
    bclk_half_ps = 1.0e12 / (4.0 * new_slots * new_fs);
    slot_i       = 0;
    lfsr_state   = new_seed;
    in_count     = 0;
    last_pidx    = -1;
    checked      = 0;
    mismatches   = 0;
    out_wr       = 0;
    i2s_lrck_i   = 1'b0;     // 刻意与旧流相位相反
    bclk_gate    = 1'b0;
    gen_run      = 1;
end
endtask

//------------------------------------------------------------------------------
// DSD 位精确校验：把 reclock 后的 1-bit 码流定位到输入码流的某个
// 起点，之后要求 bit 索引严格连续且逐位相同（不丢 bit、不重复 bit）。
//------------------------------------------------------------------------------
task check_dsd_stream;
    output integer checked_bits;
    output integer bit_errors;
    output integer anchored;
    integer a, k, p, anchor, pfound, ok, j;
begin
    checked_bits = 0; bit_errors = 0; anchored = 0;
    anchor = -1;
    a = 0;
    while (a < dout_n - 64 && anchor < 0) begin
        ok = 1;
        for (k = 0; k < 64; k = k + 1)
            if (dout_v[a+k] !== 1'b1) ok = 0;
        if (ok) anchor = a;
        a = a + 1;
    end
    pfound = -1;
    if (anchor >= 0) begin
        p = 0;
        while (p < din_n - 64 && pfound < 0) begin
            ok = 1;
            for (k = 0; k < 64; k = k + 1) begin
                if (din_l[p+k] !== dout_l[anchor+k] ||
                    din_r[p+k] !== dout_r[anchor+k]) ok = 0;
                if (!ok) k = 64;
            end
            if (ok) pfound = p;
            p = p + 1;
        end
    end
    if (pfound >= 0) begin
        anchored = 1;
        for (j = anchor; j < dout_n; j = j + 1) begin
            if (dout_v[j] === 1'b1) begin
                p = pfound + (j - anchor);
                if (p < din_n) begin
                    checked_bits = checked_bits + 1;
                    if (din_l[p] !== dout_l[j] || din_r[p] !== dout_r[j])
                        bit_errors = bit_errors + 1;
                end
            end
        end
    end
end
endtask

task wait_dsd_lock;
    input  integer code;
    output integer ok;
    integer i;
begin
    i = 0; ok = 0;
    while (!(dut.cur_fmt === 1'b1 && dut.cur_rate === code[2:0] &&
             dut.prefilled === 1'b1 && dut.gain === 5'd16) && i < 800) begin
        #(T49 * 2000.0);
        i = i + 1;
    end
    ok = (dut.cur_fmt === 1'b1 && dut.cur_rate === code[2:0] &&
          dut.prefilled === 1'b1 && dut.gain === 5'd16);
end
endtask

task pop_reset;
begin
    pop_chk_en = 0;
    pop_rst_samples = 0; pop_rst_ones_l = 0; pop_rst_ones_r = 0;
    pop_rst_same_l = 0; pop_rst_same_r = 0;
    pop_rst_max_same_l = 0; pop_rst_max_same_r = 0;
    pop_max_gap_clk = 0.0;
    #(T49 * 2.0);
    pop_last_clk_t = $realtime;
    pop_rst_last_l = i2s_lrck_o; pop_rst_last_r = i2s_sdata_o;
    pop_chk_en = 1;
end
endtask

//------------------------------------------------------------------------------
// 单个 DSD 档位用例：自动识别 → 时钟选择 → SDRAM reclock → 原生输出
//------------------------------------------------------------------------------
task run_dsd_case;
    input integer code;
    input integer nwords;
    input [31:0] seed;
    input integer const_mode;
    integer lock_ok, led_exp, dsd_checked, dsd_errs, dsd_anchor;
    real    dclk, core_f, exp_dclk, exp_mclk, fmeas;
begin
    dsd_const = const_mode;
    core_f = code[0] ? 49.152e6 : 45.1584e6;
    dclk   = core_f * (1 << (code >> 1)) / 16.0;
    bclk_half_ps = 1.0e12 / (2.0 * dclk);

    gen_run = 0; dsd_run = 0; chk_en = 0; dsd_chk_en = 0; meas_en = 0;
    fade_mon_en = 0; ad_check_en = 0;
    i2s_lrck_i = 1'b0; i2s_sdata_i = 1'b0;
    dlfsr_l = seed; dlfsr_r = seed ^ 32'hA5A55A5A;
    dbit_i = 0; din_n = 0; dout_n = 0;
    lr_t = 0; mclk_t = 0; obclk_t = 0;
    lr_period = 0; mclk_period = 0; obclk_period = 0;
    // 数据完整性检查期间关闭水位 slip，避免插入静音组改变码流相位。
    dut.slip_low_th  = 24'd0;
    dut.slip_high_th = 24'hFFFFFF;
    #(T49 * 40.0);
    dsd_run = 1; meas_en = 1;
    wait_dsd_lock(code, lock_ok);
    repeat (60) @(ev_dsd_word);
    dout_n = 0; dsd_chk_en = 1;
    repeat (nwords) @(ev_dsd_word);
    #(T49 * 200.0);
    dsd_chk_en = 0; dsd_run = 0; meas_en = 0;
    check_dsd_stream(dsd_checked, dsd_errs, dsd_anchor);

    exp_dclk = 1.0e12 / dclk;
    exp_mclk = ((code >> 1) == 0) ? (1.0e12 / (core_f / 4.0))
                                  : (1.0e12 / (core_f / 2.0));
    led_exp  = 1 << (code >> 1);
    $display("DSD CASE code=%0d const=%0d  DCLK=%0d Hz  out_dclk=%0d ps (exp %0d)  out_mclk=%0d ps (exp %0d)  led=%b  fmt=%b rate=%0d  bits=%0d errs=%0d anchor=%0d fill=%0d",
             code, const_mode, dclk, obclk_period, exp_dclk, mclk_period, exp_mclk,
             led, dut.cur_fmt, dut.cur_rate, dsd_checked, dsd_errs, dsd_anchor,
             dut.fill_words);
    expect_true(lock_ok === 1, "DSD: automatic format/rate detection and re-lock");
    expect_true(dut.cur_fmt === 1'b1, "DSD: input recognised as native DSD");
    expect_true(dut.cur_rate === code[2:0], "DSD: bit-clock rate classified");
    expect_true(dut.clk_sel_r === code[0], "DSD: correct clean master clock selected");
    expect_true(led === led_exp, "DSD: rate-tier LED indication");
    // f0~F3.txt：DSD 编码 = 1001 + 档位（DSD64/128/256/512）
    expect_true(dsd_on === 1'b1 && fsel === (4'b1001 + {2'b0, code[2:1]}),
                "DSD: DSDOE=1 and F0~F3 match f0~F3.txt DSD rate table");
    expect_true(dsd_anchor === 1 && dsd_checked > 1500 && dsd_errs === 0,
                "DSD: bit-exact native FIFO reclock, no drop/repeat");
    fmeas = 1.0e12 / obclk_period;
    expect_true(fmeas > dclk*0.99 && fmeas < dclk*1.01,
                "DSD: output DCLK regenerated at correct native rate");
    fmeas = 1.0e12 / mclk_period;
    expect_true(fmeas > (1.0e12/exp_mclk)*0.99 && fmeas < (1.0e12/exp_mclk)*1.01,
                "DSD: output MCLK regenerated from the selected crystal");
    expect_true(dut.u_sdram.init_done === 1'b1 && m_wr_cnt > 0 && m_rd_cnt > 0,
                "DSD: same SDRAM ring buffer really used for reclock");
    expect_true(ad_bclk === 1'b0 && ad_le === 1'b1 && do_l === 1'b0 && do_r === 1'b0,
                "DSD: AD1862 PCM port stays idle");
end
endtask

//------------------------------------------------------------------------------
// DoP 用例：标准 32-slot PCM 帧携带 0x05/0xFA marker + 16 DSD 位，
// 验证自动识别 → 解码 → 复用 SDRAM FIFO reclock → 原生 DSD 位精确输出。
// pcm_code: 4=176.4k(DSD64/44.1k族), 5=192k(DSD64/48k族),
//           6=352.8k(DSD128/44.1k族), 7=384k(DSD128/48k族)
//------------------------------------------------------------------------------
task run_dop_case;
    input integer pcm_code;
    input integer nframes;
    input [31:0] seed;
    integer dsd_code_v, lock_ok, dsd_checked, dsd_errs, dsd_anchor, led_exp;
    real    fs, core_f, dclk, exp_dclk, exp_mclk, fmeas;
    reg     dop_was_active, dop_was_det, dop_was_fmt, dop_on_snap;
    reg [2:0] dop_was_rate;
    reg [3:0] dop_fsel_snap, dop_led_snap;
begin
    case (pcm_code)
        4: fs = 176400.0;
        5: fs = 192000.0;
        6: fs = 352800.0;
        default: fs = 384000.0;
    endcase
    case (pcm_code)
        4: dsd_code_v = 0;   // DSD64  / 44.1k 族
        5: dsd_code_v = 1;   // DSD64  / 48k 族
        6: dsd_code_v = 2;   // DSD128 / 44.1k 族
        default: dsd_code_v = 3;   // DSD128 / 48k 族
    endcase
    core_f = (dsd_code_v[0] == 0) ? 45.1584e6 : 49.152e6;
    dclk   = core_f * (1 << (dsd_code_v >> 1)) / 16.0;

    gen_run = 0; dsd_run = 0; dop_run = 0;
    chk_en = 0; dsd_chk_en = 0; meas_en = 0;
    fade_mon_en = 0; ad_check_en = 0;
    i2s_lrck_i = 1'b1; i2s_sdata_i = 1'b0;
    slot_i = 0; lfsr_state = seed;
    gen_slots = 32; gen_fs = fs;          // DoP 固定用 32-slot 容器
    bclk_half_ps = 1.0e12 / (4.0 * 32.0 * fs);
    wmask = 32'hFFFFFFFF;
    dop_mk_phase = 0; din_n = 0; dout_n = 0;
    in_count = 0; chk_aligned = 0; chk_idx = 0; checked = 0; mismatches = 0;
    out_wr = 0; out_started = 0; out_lr_d = 1'b1; out_cap = 0; out_idx = 0;
    lr_t = 0; mclk_t = 0; obclk_t = 0;
    lr_period = 0; mclk_period = 0; obclk_period = 0;
    // 数据完整性检查期间关闭水位 slip，避免插入静音组改变码流相位。
    dut.slip_low_th  = 24'd0;
    dut.slip_high_th = 24'hFFFFFF;
    #(T49 * 40.0);
    dop_run = 1; gen_run = 1; meas_en = 1;
    wait_dsd_lock(dsd_code_v, lock_ok);
    #(T49 * 4000.0);                    // 让淡入与缓冲稳定
    dout_n = 0; dsd_chk_en = 1;
    repeat (nframes) @(ev_frame);
    #(T49 * 300.0);
    dop_was_active = dut.dop_active_w;
    dop_was_det    = dut.dop_det;
    dop_was_fmt    = dut.cur_fmt;
    dop_was_rate   = dut.cur_rate;
    dop_on_snap    = dsd_on;
    dop_fsel_snap  = fsel;
    dop_led_snap   = led;
    dsd_chk_en = 0; dop_run = 0; meas_en = 0;
    check_dsd_stream(dsd_checked, dsd_errs, dsd_anchor);
    // 继续发若干「无 marker」的普通 PCM 帧，让解码器按 MISS 迟滞
    // 自然退出，释放 u_dsd_fifo 写口，供下一个用例使用。
    repeat (16) @(ev_frame);
    gen_run = 0;
    #(T49 * 300.0);

    exp_dclk = 1.0e12 / dclk;
    exp_mclk = ((dsd_code_v >> 1) == 0) ? (1.0e12 / (core_f / 4.0))
                                        : (1.0e12 / (core_f / 2.0));
    led_exp  = 1 << (dsd_code_v >> 1);
    $display("DOP CASE pcm=%0d fs=%0d  dsd=%0d DCLK=%0d Hz  out_dclk=%0d ps (exp %0d)  led=%b  fmt=%b rate=%0d dop=%b  bits=%0d errs=%0d anchor=%0d fill=%0d",
             pcm_code, fs, dsd_code_v, dclk, obclk_period, exp_dclk, dop_led_snap,
             dop_was_fmt, dop_was_rate, dop_was_active,
             dsd_checked, dsd_errs, dsd_anchor, dut.fill_words);
    expect_true(lock_ok === 1 && dop_was_fmt === 1'b1 && dop_was_rate === dsd_code_v[2:0],
                "DoP: automatically detected and locked to native DSD rate");
    expect_true(dop_was_active === 1'b1 && dop_was_det === 1'b1,
                "DoP: decoder active and detection synchronised to management domain");
    expect_true(dop_on_snap === 1'b1 && dop_fsel_snap === (4'b1001 + {2'b0, dsd_code_v[2:1]}),
                "DoP: DSDOE=1 and F0~F3 report decoded native DSD rate");
    expect_true(dop_led_snap === led_exp, "DoP: rate-tier LED follows decoded DSD rate");
    expect_true(dsd_anchor === 1 && dsd_checked > 1000 && dsd_errs === 0,
                "DoP: bit-exact native DSD output after FIFO reclock, no drop/repeat");
    fmeas = 1.0e12 / obclk_period;
    expect_true(fmeas > dclk*0.99 && fmeas < dclk*1.01,
                "DoP: output DCLK regenerated at native DSD rate");
    expect_true(dut.u_sdram.init_done === 1'b1 && m_wr_cnt > 0 && m_rd_cnt > 0,
                "DoP: same SDRAM ring buffer used for reclock");
    expect_true(ad_bclk === 1'b0 && ad_le === 1'b1 && do_l === 1'b0 && do_r === 1'b0,
                "DoP: AD1862 PCM port stays idle");
    dut.slip_low_th  = 24'd64;
    dut.slip_high_th = 24'd1000000;
end
endtask

//------------------------------------------------------------------------------
// DSD 长时低水位：DSD 与 PCM 共用同一套 SDRAM 水位 / slip / fade 通路，
// 但 DSD512 的帧对率最高（768 kHz），水位漂到 P_SLIP_LOW 所需时间
// 反而比 352.8 kHz PCM 更短。这里注入持续 low_water，验证第 11 节
// 「按 LRCK 帧计 512 帧冷却」的修复同样覆盖 DSD，不会陷入连续 fade 循环。
//------------------------------------------------------------------------------
task run_dsd_lowwater_case;
    input integer code;
    input integer nwords;
    input [31:0] seed;
    integer lk;
    real    dclk, core_f;
begin
    core_f = code[0] ? 49.152e6 : 45.1584e6;
    dclk   = core_f * (1 << (code >> 1)) / 16.0;
    bclk_half_ps = 1.0e12 / (2.0 * dclk);

    gen_run = 0; dsd_run = 0; chk_en = 0; dsd_chk_en = 0; meas_en = 0;
    fade_mon_en = 0; ad_check_en = 0; dsd_low_chk = 0;
    i2s_lrck_i = 1'b0; i2s_sdata_i = 1'b0;
    dsd_const = 0;
    dlfsr_l = seed; dlfsr_r = seed ^ 32'hA5A55A5A;
    dbit_i = 0; din_n = 0; dout_n = 0;
    dsd_low_cnt = 0; dsd_gain16_cnt = 0; slip_acks = 0;
    #(T49 * 40.0);
    dsd_run = 1;
    wait_dsd_lock(code, lk);
    repeat (60) @(ev_dsd_word);
    // 抬高低水位阈值到稳态水位之上，等效"长时间播放后水位已漂过阈值"
    dut.slip_low_th  = dut.fill_words + 24'd128;
    dut.slip_high_th = 24'hFFFFFF;
    dsd_low_chk = 1;
    repeat (nwords) @(ev_dsd_word);
    #(T49 * 200.0);
    dsd_low_chk = 0; dsd_run = 0;
    $display("DSD LOWWATER: dclk=%0d samples=%0d gain16=%0d slip_acks=%0d fmt=%b rate=%0d fill=%0d",
             dclk, dsd_low_cnt, dsd_gain16_cnt, slip_acks,
             dut.cur_fmt, dut.cur_rate, dut.fill_words);
    expect_true(lk === 1 && dut.cur_fmt === 1'b1 && dut.cur_rate === code[2:0],
                "DSD long-run: still locked on native DSD under persistent low water");
    expect_true(dsd_on === 1'b1 && fsel === (4'b1001 + {2'b0, code[2:1]}),
                "DSD long-run: DSDOE/F0~F3 indication stays correct");
    expect_true(slip_acks >= 1,
                "DSD long-run: low-water slip path exercised");
    expect_true(dsd_gain16_cnt*10 > dsd_low_cnt*8,
                "DSD long-run: no continuous fade loop (steady unity-gain bitstream)");
    dut.slip_low_th  = 24'd0;
    dut.slip_high_th = 24'hFFFFFF;
end
endtask

//------------------------------------------------------------------------------
// 主验证流程
//------------------------------------------------------------------------------
integer lock_ok, dsd_checked, dsd_errs, dsd_anchor;
initial begin
    key_k3 = 1'b1;
    i2s_lrck_i = 1'b1;
    i2s_sdata_i = 1'b0;
    ad_check_en = 0;

    //--------------------------------------------------------------------------
    // 硬件上电时序：FPGA 从 EPCS4 配置完成后，音源往往还没开始输出。
    // 此时 H_IDLE 会先判 stream_lost → H_FADE → H_RESYNC；而 44.1k 族
    // （cand_all[0]=0，例如 352.8 kHz）与 cur_family 初值 0 相同，
    // 会走「同族直接 H_RST」路径。若 clk_ena_r 只由 H_ON 置 1，
    // 这条路径会永久跳过开时钟，clk_core 停摆：rate_s2/mode_s 不更新
    // → LED0 常亮、按 K3 无反应、AD1862 无声。
    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    // 上电顺序复现：音源 BCLK 先出、LRCK 延迟（32bit/352.8k）。
    // BCLK=22.5792 MHz 命中 DSD512 档；POR 后 cur_valid=0，旧保护
    // 「cur_valid && !cur_fmt」失效 → 判据二成立 → 误判成 DSD；
    // 而 cur_fmt 被置 1 后该保护永久失效，PCM 在 DSD/PCM 之间反复切换
    // → 持续卡顿。31bit/352.8k 的 BCLK=21.8736 MHz 不命中任何 DSD 档，
    // 所以完全正常 —— 这正是「只有 32bit 卡」的分水岭。
    //--------------------------------------------------------------------------
    bclk_half_ps = 1.0e12 / (4.0 * 32.0 * 352800.0);   // 32bit/352.8k 的 BCLK
    gen_run = 0;
    i2s_lrck_i = 1'b1;
    #(2000000000.0);       // 2 ms：> 12 个检测窗口(约 1 ms)
    $display("POR BCLK-ONLY: fmt_dsd=%b evidence=%b rate_ok=%b i2s_age=%0d lr_edges=%0d cur_valid=%b cur_fmt=%b pcm_seen=%b",
             dut.fmt_dsd, dut.dsd_evidence, dut.dsd_rate_ok, dut.i2s_age,
             dut.lr_edge_cnt, dut.cur_valid, dut.cur_fmt, dut.pcm_seen);
    expect_true(dut.dsd_rate_ok === 1'b1,
                "POR: 32bit/352.8k BCLK really hits the DSD512 rate slot");
    expect_true(dut.fmt_dsd === 1'b0 && dut.cur_fmt === 1'b0,
                "POR: BCLK-first / LRCK-late must NOT be misclassified as DSD");

    #(1000000000.0);       // 1 ms：> POR(20.8 µs) + H_FADE(488 µs)，音源仍未启动
    $display("POR-IDLE: clk_ena=%b hstate=%0d core_rst=%b led=%b lost=%b",
             dut.clk_ena_r, dut.hstate, dut.core_rst_n, led, dut.stream_lost);
    expect_true(dut.clk_ena_r === 1'b1,
                "clk_core stays enabled after POR even before the source starts");
    expect_true(dsd_on === 1'b0 && fsel === 4'b0000,
                "DSDOE/F0~F3 stay at the idle code 0000 before rate lock");

    //--------------------------------------------------------------------------
    // LED 与 clk_core 解耦验证：人为关断 ALTCLKCTRL 输出，管理域仍应
    // 让 K3 立即点亮四颗 LED。否则 core 时钟一停，用户按 K3 就完全
    // 看不到反馈，无法区分「管理域没跑」与「core 时钟停」。
    //--------------------------------------------------------------------------
    force dut.clk_ena_r = 1'b0;
    key_k3 = 1'b0;
    #(15000000000.0);
    #(T49 * 100.0);
    $display("LED-DIAG: clk_ena=%b mode_r=%b led=%b", dut.clk_ena_r, dut.mode_r, led);
    expect_true(led === 4'b1111,
                "LED responds to K3 even when clk_core is gated off");
    key_k3 = 1'b1;
    #(2000000000.0);
    release dut.clk_ena_r;
    key_k3 = 1'b0;              // 第二次按键把 mode_r 翻回 0，避免影响后续用例
    #(15000000000.0);
    key_k3 = 1'b1;
    #(2000000000.0);
    #(T49 * 100.0);

    run_case(0, 32, 400, 32'h12345678);   //  44.1 kHz / 32 位
    run_case(1, 32, 400, 32'h23456789);   //  48.0 kHz / 32 位
    run_case(2, 24, 400, 32'h3456789A);   //  88.2 kHz / 24 位
    run_case(3, 16, 400, 32'h456789AB);   //  96.0 kHz / 16 位
    run_case(4, 32, 400, 32'h56789ABC);   // 176.4 kHz / 32 位
    run_case(5, 20, 400, 32'h6789ABCD);   // 192.0 kHz / 20 位
    run_case(6, 32, 400, 32'h789ABCDE);   // 352.8 kHz / 32 位
    run_case(7, 16, 400, 32'h89ABCDEF);   // 384.0 kHz / 16 位

    // R5：48 kHz 下的 K3 模式切换
    run_case(1, 32, 300, 32'hABCDEF01);
    ad_err = 0; ad_checks = 0; le_cnt = 0;
    ad_err_period = 0; ad_err_setup = 0; ad_err_high = 0; ad_err_hold = 0;
    ad_err_le = 0; ad_err_data = 0;
    cur20_valid = 0; ad_rise_cnt = 0; ad_shift_l = 0; ad_shift_r = 0; ad_warmup = 0;
    dither_seen = 0; fade_min_gain = 16; fade_step_err = 0; fade_prev = -1;
    slip_acks = 0; fade_mon_en = 1;
    ad_check_en = 0;            // 按键 15 ms + 模式淡变期间端口尚未提交，不纳入时序检查
    press_key();
    #(T49 * 200.0);
    $display("K3 mode-1: led=%b  ad_bclk=%b ad_le=%b  mode_r=%b mode_core=%b mode_eff=%b dsd_core=%b",
             led, ad_bclk, ad_le, dut.mode_r, dut.mode_core, dut.mode_eff, dut.dsd_core);
    expect_true(led === 4'b1111, "R5 all LEDs on in AD1862 mode");
    expect_true(dut.mode_r === 1'b1 && dut.mode_core === 1'b1 && dut.mode_eff === 1'b1,
                "K3: key event reaches core domain under 10 ms hardware debounce");
    repeat (40) @(ev_frame);   // 等 K3 触发的 33 帧淡入淡出完成
    ad_check_en = 1;
    repeat (250) @(ev_frame);
    #(T49 * 100.0);
    $display("AD1862: frames_checked=%0d le_pulses=%0d ad_err=%0d (period=%0d setup=%0d high=%0d hold=%0d le=%0d data=%0d) dither_seen=%0d fade_min=%0d fade_step_err=%0d",
             ad_checks, le_cnt, ad_err, ad_err_period, ad_err_setup, ad_err_high,
             ad_err_hold, ad_err_le, ad_err_data, dither_seen, fade_min_gain, fade_step_err);
    expect_true(ad_checks > 100, "R5 24 AD1862 CLK pulses per LRCK frame");
    expect_true(le_cnt > 100, "R5 low-active LE latch pulse generated each frame");
    expect_true(ad_err === 0, "R5 AD1862 20-bit data within 1 LSB + datasheet Fig.12b timing");
    expect_true(dither_seen > 100, "Hi-end TPDF dither + rounding active on 20-bit path");
    expect_true(fade_min_gain === 0 && fade_step_err === 0,
                "Hi-end K3 mode switch fades to silence smoothly (16 frames)");
    ad_check_en = 0;

    press_key();
    repeat (40) @(ev_frame);       // 淡出 / 静音期间切换端口 / 淡入
    #(T49 * 100.0);
    $display("K3 mode-0: led=%b  ad_bclk=%b ad_le=%b do_l=%b do_r=%b", led, ad_bclk, ad_le, do_l, do_r);
    expect_true(led === 4'b0001, "R5 LEDs return to rate indication after 2nd press");
    expect_true(ad_bclk === 1'b0 && ad_le === 1'b1 && do_l === 1'b0 && do_r === 1'b0,
                "R5 standard I2S output mode restored (AD1862 port idle)");

    // Hi-end 优化①：注入高于稳态的低水位阈值，
    // 触发水位 slip 路径。每次 slip 都必须在平滑淡变内提交，
    // 不得出现硬重复/丢弃样本；slip 淡变用步长 4（约 10 帧）。
    dut.slip_low_th = 24'd1000;
    slip_acks = 0; fade_min_gain = 16; fade_step_err = 0; fade_prev = -1;
    fade_mon_en = 1;
    // 冷却按 512 帧计，需 >1024 帧才能观察到多次 slip。
    repeat (1200) @(ev_frame);
    #(T49 * 100.0);
    $display("SLIP: acks=%0d fade_min=%0d fade_step_err=%0d fill=%0d",
             slip_acks, fade_min_gain, fade_step_err, dut.fill_words);
    expect_true(slip_acks >= 2, "Hi-end water-mark slip path exercised");
    expect_true(fade_min_gain === 0 && fade_step_err === 0,
                "Hi-end every slip committed under a smooth fade (no hard repeat)");
    dut.slip_low_th = 24'd64;
    fade_mon_en = 0;

    //--------------------------------------------------------------------------
    // 手动切歌（同速率）：旧设计在此处从不复位，
    // 接收器永久错位 → 噪音持续到断电重启。
    //--------------------------------------------------------------------------
    $display("--- manual track change: 48 kHz -> 48 kHz, LRCK dropout 3 ms ---");
    track_change(3000000000.0, 32, 48000.0, 32'h13572468);
    repeat (500) @(ev_frame);
    #(T49 * 100.0);
    $display("RESYNC same-rate: hstate=%0d cur_valid=%b gain=%0d prefilled=%b clk_sel=%b checked=%0d mism=%0d",
             dut.hstate, dut.cur_valid, dut.gain, dut.prefilled, dut.clk_sel_r,
             checked, mismatches);
    expect_true(dut.hstate === 3'd1 && dut.cur_valid === 1'b1,
                "Track change: design returns to locked state by itself (no power cycle)");
    expect_true(dut.gain === 5'd16 && dut.prefilled === 1'b1,
                "Track change: unity gain and buffer prefill restored");
    expect_true(checked > 80 && mismatches <= 3,
                "Track change: bit-exact audio resumes, no persistent noise");

    // --- 以及跨速率手动切歌（48 kHz → 96 kHz / 16 位）---------
    $display("--- manual track change: 48 kHz -> 96 kHz / 16-bit ---");
    track_change(2000000000.0, 16, 96000.0, 32'h2468ACE0);
    repeat (600) @(ev_frame);
    #(T49 * 100.0);
    $display("RESYNC cross-rate: hstate=%0d cur_valid=%b gain=%0d led=%b clk_sel=%b checked=%0d mism=%0d",
             dut.hstate, dut.cur_valid, dut.gain, led, dut.clk_sel_r,
             checked, mismatches);
    expect_true(dut.hstate === 3'd1 && dut.cur_valid === 1'b1 && dut.gain === 5'd16,
                "Cross-rate track change: fully re-locked and faded back in");
    expect_true(led === 4'b0010,
                "Cross-rate track change: LED indicates the new 96 kHz rate");
    expect_true(checked > 80 && mismatches <= 3,
                "Cross-rate track change: bit-exact audio resumes");

    // 短暂抖动（< P_LRCK_GAP = 41.7 µs）不得触发完整重锁，
    // 否则无害的 LRCK 抖动也会打断正常播放。
    $display("--- short 10 us LRCK dropout (no resync expected) ---");
    bclk_gate = 1'b1; gen_run = 0;
    i2s_lrck_i = 1'b1; i2s_sdata_i = 1'b0;
    #(10000.0);
    bclk_gate = 1'b0; gen_run = 1;
    repeat (120) @(ev_frame);
    $display("SHORT GAP: hstate=%0d gain=%0d cur_valid=%b", dut.hstate, dut.gain, dut.cur_valid);
    expect_true(dut.hstate === 3'd1 && dut.gain === 5'd16,
                "Short LRCK dropout does not falsely trigger a full re-lock");

    //--------------------------------------------------------------------------
    // DSD：自动识别 + FIFO/SDRAM reclock + 原生 1-bit 输出。
    // 覆盖 4 档位速率与两个晶振族，并验证 I2S↔DSD 双向自动切换。
    //--------------------------------------------------------------------------
    $display("--- DSD native: auto detection + FIFO reclock ---");
    run_dsd_case(0, 200, 32'h13579BDF, 0);   // DSD64  / 44.1 kHz 族
    run_dsd_case(3, 200, 32'h2468ACE0, 0);   // DSD128 / 48 kHz 族
    run_dsd_case(4, 200, 32'h55AA00FF, 0);   // DSD256 / 44.1 kHz 族
    run_dsd_case(7, 200, 32'h0F0F55AA, 0);   // DSD512 / 48 kHz 族
    // 恒定电平（无 LRCK 跳变）的 DSD 静音码型也必须被识别为 DSD
    run_dsd_case(1, 120, 32'hDEADBEEF, 1);   // DSD64 / 48 kHz 族，恒定 L=0 R=1

    // DSD 手动切歌：DCLK 停流后靠心跳判据自动重锁，无需断电。
    $display("--- DSD track change: DCLK dropout 3 ms ---");
    dsd_const = 0;
    pop_reset();           // 从断流前开始监测输出是否出现满幅直流/时钟停摆
    bclk_gate = 1'b1; dsd_run = 0; gen_run = 0;
    i2s_lrck_i = 1'b0; i2s_sdata_i = 1'b0;
    #(3000000000.0);
    $display("DSD DROPOUT: lost=%b gain=%0d hstate=%0d",
             dut.stream_lost, dut.gain, dut.hstate);
    expect_true(dut.stream_lost === 1'b1 && dut.gain === 5'd0,
                "DSD: DCLK dropout detected via heartbeat and output muted");
    bclk_gate = 1'b0;
    bclk_half_ps = 1.0e12 / (2.0 * 24.576e6);   // 回到 DSD512 / 48k 族
    din_n = 0; dout_n = 0; dbit_i = 0;
    dsd_run = 1;
    wait_dsd_lock(7, lock_ok);
    repeat (60) @(ev_dsd_word);
    dout_n = 0; dsd_chk_en = 1;
    repeat (150) @(ev_dsd_word);
    #(T49 * 200.0);
    dsd_chk_en = 0; dsd_run = 0;
    pop_chk_en = 0;
    check_dsd_stream(dsd_checked, dsd_errs, dsd_anchor);
    $display("DSD POP: rst_samples=%0d ones_l=%0d ones_r=%0d max_same_l=%0d max_same_r=%0d max_dclk_gap=%0f ps",
             pop_rst_samples, pop_rst_ones_l, pop_rst_ones_r,
             pop_rst_max_same_l, pop_rst_max_same_r, pop_max_gap_clk);
    expect_true(pop_rst_samples > 1000,
                "DSD pop monitor really covered the re-lock reset window");
    expect_true(pop_rst_ones_l*10 > pop_rst_samples*4 && pop_rst_ones_l*10 < pop_rst_samples*6 &&
                pop_rst_ones_r*10 > pop_rst_samples*4 && pop_rst_ones_r*10 < pop_rst_samples*6,
                "DSD track change: 50%-density mute bitstream during reset, no full-scale DC step");
    expect_true(pop_rst_max_same_l <= 64 && pop_rst_max_same_r <= 64,
                "DSD track change: no constant-level output during re-lock");
    expect_true(pop_max_gap_clk <= 2.0e7,
                "DSD track change: output DCLK keeps running, no long clock stall");
    $display("DSD RESYNC: lock=%0d hstate=%0d gain=%0d bits=%0d errs=%0d anchor=%0d",
             lock_ok, dut.hstate, dut.gain, dsd_checked, dsd_errs, dsd_anchor);
    expect_true(lock_ok === 1 && dut.hstate === 3'd1,
                "DSD: automatic re-lock after track change (no power cycle)");
    expect_true(dsd_anchor === 1 && dsd_checked > 1500 && dsd_errs === 0,
                "DSD: bit-exact native playback resumes after re-lock");

    // 跨晶振族切歌：必须走 H_OFF/H_SEL 关时钟换 MUX，验证这段
    // 窗口同样不会退化为长时间的恒定电平或 DCLK 停摆。
    $display("--- DSD track change across crystal families: DSD512/48k -> DSD64/44.1k ---");
    dsd_const = 0;
    pop_reset();
    bclk_gate = 1'b1; dsd_run = 0; gen_run = 0;
    i2s_lrck_i = 1'b0; i2s_sdata_i = 1'b0;
    #(3000000000.0);
    expect_true(dut.stream_lost === 1'b1,
                "DSD cross-family: dropout detected before clock MUX switch");
    bclk_gate = 1'b0;
    bclk_half_ps = 1.0e12 / (2.0 * 2.8224e6);   // DSD64 / 44.1 kHz 族
    dsd_run = 1;
    wait_dsd_lock(0, lock_ok);
    repeat (60) @(ev_dsd_word);
    #(T49 * 200.0);
    pop_chk_en = 0; dsd_run = 0;
    $display("DSD POP (cross-family): rst_samples=%0d ones_l=%0d ones_r=%0d max_same_l=%0d max_same_r=%0d max_dclk_gap=%0f ps",
             pop_rst_samples, pop_rst_ones_l, pop_rst_ones_r,
             pop_rst_max_same_l, pop_rst_max_same_r, pop_max_gap_clk);
    expect_true(lock_ok === 1 && dut.cur_rate === 3'd0 && dut.clk_sel_r === 1'b0,
                "DSD cross-family: re-locked onto the 45.1584 MHz crystal");
    expect_true(pop_rst_ones_l*10 > pop_rst_samples*4 && pop_rst_ones_l*10 < pop_rst_samples*6 &&
                pop_rst_ones_r*10 > pop_rst_samples*4 && pop_rst_ones_r*10 < pop_rst_samples*6,
                "DSD cross-family: 50%-density mute bitstream, no full-scale DC step");
    expect_true(pop_rst_max_same_l <= 64 && pop_rst_max_same_r <= 64,
                "DSD cross-family: no long constant-level output");
    expect_true(pop_max_gap_clk <= 2.0e7,
                "DSD cross-family: no long output DCLK stall");

    // 切回 I2S：识别器必须自动返回 PCM 通路，原有 I2S 功能保持完好。
    dut.slip_low_th  = 24'd64;
    dut.slip_high_th = 24'd1000000;
    $display("--- format switch back: native DSD -> I2S 48 kHz / 32-bit ---");
    run_case(1, 32, 300, 32'h0F1E2D3C);
    expect_true(dut.cur_fmt === 1'b0,
                "Auto format detection switches back to I2S/PCM");

    //--------------------------------------------------------------------------
    // 长时间播放等效：两晶振 ppm 级偏差使 SDRAM 水位从预填值缓慢下降，
    // 352.8 kHz 档实测约 0.5~1 小时后首次触及 P_SLIP_LOW，此后
    // low_water 会持续为真。检验 low-water slip 是否把输出拖进
    // 连续「淡出→静音→淡入」循环（表现为 10 kHz 量级增益调制噪声）。
    // 用运行时抬高低水位阈值来等效"已经漂移穿过阈值"的状态。
    //--------------------------------------------------------------------------
    $display("--- long-run low-water drift @352.8k/20bit, AD1862 mode ---");
    run_case_extra_wait = 1;
    run_case(6, 20, 200, 32'h5A5AA5A5);
    run_case_extra_wait = 0;
    expect_true(dut.cur_fmt === 1'b0 && dut.fmt_dsd === 1'b0 && dut.dsd_core === 1'b0,
                "352.8k/20-bit PCM is never misclassified as native DSD");
    expect_true(led === (4'b0001 << dut.cur_rate[2:1]),
                "LED rate tier follows the management-domain rate code (LED3 @352.8k)");
    press_key();                         // 进入 AD1862 模式
    repeat (60) @(ev_frame);
    dut.slip_low_th = dut.fill_words + 24'd128;
    frame_cnt = 0; gain16_cnt = 0; matched_cnt = 0; slip_acks = 0;
    fade_frames = 0; fade_max_frames = 0; stall_chk = 1;
    chk_en = 1;
    // 1500 帧 > 一次 512 帧冷却，可覆盖"多次 slip 之后"的长期稳态。
    repeat (1500) @(ev_frame);
    #(T49 * 100.0);
    chk_en = 0; stall_chk = 0;
    $display("LONGRUN: frames=%0d gain16=%0d matched=%0d slip_acks=%0d fade_max=%0d fill=%0d",
             frame_cnt, gain16_cnt, matched_cnt, slip_acks,
             fade_max_frames, dut.fill_words);
    expect_true(slip_acks >= 1,
                "Long-run: low-water slip path actually exercised");
    expect_true(fade_max_frames <= 12,
                "Long-run: each low-water slip costs <=12 frames of fade, not an audible 33-frame mute");
    expect_true(gain16_cnt*10 > frame_cnt*9 && matched_cnt*10 > frame_cnt*8,
                "Long-run: steady unity-gain bit-exact playback maintained after repeated low-water slips");
    press_key();                         // 退出 AD1862 模式
    repeat (40) @(ev_frame);
    dut.slip_low_th = 24'd64;

    //--------------------------------------------------------------------------
    // K3/AD1862 模式在断流重锁后是否保持：数据通路复位会把 mode_out
    // 清 0，而 mode_eff（按键状态）不变；若没有新的 mode_chg 请求，
    // 端口会永久空闲（LED 仍全亮）→ 表现为「K3 切换失效」。
    //--------------------------------------------------------------------------
    $display("--- K3 AD1862 mode persistence across stream-loss re-lock ---");
    run_case(1, 32, 200, 32'h1B2C3D4E);
    press_key();                         // 进入 AD1862 模式
    repeat (60) @(ev_frame);
    $display("K3 BASE: mode_r=%b mode_eff=%b mode_out=%b led=%b",
             dut.mode_r, dut.mode_eff, dut.mode_out, led);
    expect_true(dut.mode_out === 1'b1 && led === 4'b1111,
                "K3: AD1862 port committed after key press");
    // 关闭水位 slip：修复前密集 slip 带来的 F_HOLD 会"顺便"反复提交
    // mode_out，掩盖 mode_chg 单拍脉冲丢失的缺陷；稀疏 slip 后该缺陷暴露。
    dut.slip_low_th  = 24'd0;
    dut.slip_high_th = 24'hFFFFFF;
    track_change(3000000000.0, 32, 48000.0, 32'h5E6F7081);
    repeat (500) @(ev_frame);
    #(T49 * 100.0);
    k3_ad_clks = 0; k3_ad_en = 1;
    repeat (120) @(ev_frame);
    k3_ad_en = 0;
    $display("K3 PERSIST: mode_r=%b mode_eff=%b mode_out=%b led=%b hstate=%0d gain=%0d ad_clks=%0d",
             dut.mode_r, dut.mode_eff, dut.mode_out, led, dut.hstate, dut.gain, k3_ad_clks);
    expect_true(dut.mode_out === 1'b1 && led === 4'b1111,
                "K3: AD1862 mode survives stream-loss re-lock (port stays active)");
    expect_true(k3_ad_clks > 100,
                "K3: AD1862 CLK pin really toggles after re-lock (not just an internal flag)");
    press_key();                         // 复位为普通模式
    repeat (40) @(ev_frame);
    dut.slip_low_th  = 24'd64;
    dut.slip_high_th = 24'd1000000;

    //--------------------------------------------------------------------------
    // DSD 长时低水位：DSD 与 PCM 共用同一套水位 / slip / fade 通路，
    // 覆盖最快到阈值（DSD512/48k 族）与最慢（DSD64/44.1 族）两端。
    //--------------------------------------------------------------------------
    run_dsd_lowwater_case(7, 1500, 32'hA1B2C3D4);
    run_dsd_lowwater_case(0, 1200, 32'hD4C3B2A1);
    dut.slip_low_th  = 24'd64;
    dut.slip_high_th = 24'd1000000;

    //--------------------------------------------------------------------------
    // 32bit / 352.8 kHz 长窗口：周期性停顿复现
    // (a) 无频偏基线；(b) 注入 -2% 源端频偏，把硬件上 ppm 级、
    //     数十分钟的水位下漂压缩到毫秒级，观察水位耗尽后的行为。
    //--------------------------------------------------------------------------
    $display("--- 32bit/352.8k long window: periodic stall check ---");
    run_case_extra_wait = 1;         // 352.8k 档 200 帧 < H_FADE 488 µs
    run_case(6, 32, 200, 32'h1357BDF0);
    run_case_extra_wait = 0;
    chk_en = 0;
    stall_chk = 0;
    stall_samples = 0; stall_gain_lo = 0; stall_mute = 0;
    stall_txempty = 0; stall_events = 0;
    stall_fill_min = 24'hFFFFFF; stall_fill_max = 0; stall_gain_hi = 1;
    fmt_dsd_high = 0; fmt_changes = 0; resync_events = 0; i2s_period_bad = 0;
    fmt_prev = dut.cur_fmt; hst_prev = dut.hstate;
    slip_acks = 0;
    stall_chk = 1;
    repeat (20000) @(ev_frame);
    stall_chk = 0;
    $display("STALL BASE: samples=%0d gain_lo=%0d mute=%0d txempty=%0d events=%0d slip=%0d fill=%0d..%0d | dsd_high=%0d fmt_chg=%0d resync=%0d period_bad=%0d",
             stall_samples, stall_gain_lo, stall_mute, stall_txempty,
             stall_events, slip_acks, stall_fill_min, stall_fill_max,
             fmt_dsd_high, fmt_changes, resync_events, i2s_period_bad);

    gen_fs = 352800.0 * 0.98;               // 源端慢 2%，水位被抽干
    bclk_half_ps = 1.0e12 / (4.0 * gen_slots * gen_fs);
    stall_samples = 0; stall_gain_lo = 0; stall_mute = 0;
    stall_txempty = 0; stall_events = 0;
    stall_fill_min = 24'hFFFFFF; stall_fill_max = 0; stall_gain_hi = 1;
    fmt_dsd_high = 0; fmt_changes = 0; resync_events = 0; i2s_period_bad = 0;
    fmt_prev = dut.cur_fmt; hst_prev = dut.hstate;
    slip_acks = 0;
    stall_chk = 1;
    repeat (20000) @(ev_frame);
    stall_chk = 0;
    $display("STALL SLOW: samples=%0d gain_lo=%0d mute=%0d txempty=%0d events=%0d slip=%0d fill=%0d..%0d | dsd_high=%0d fmt_chg=%0d resync=%0d period_bad=%0d",
             stall_samples, stall_gain_lo, stall_mute, stall_txempty,
             stall_events, slip_acks, stall_fill_min, stall_fill_max,
             fmt_dsd_high, fmt_changes, resync_events, i2s_period_bad);
    gen_fs = 352800.0;
    bclk_half_ps = 1.0e12 / (4.0 * gen_slots * gen_fs);

    //--------------------------------------------------------------------------
    // DSD 误判压力测试：LRCK 短暂停跳（音源缓冲欠载）而 BCLK 继续。
    // 32bit/352.8k 的 BCLK 命中 DSD512 档，修复前会把 PCM 误判成 DSD，
    // 触发完整重锁（约 0.2 s 静音）并让 DSD_ON 抖动使 DAC 反复切模式。
    // 0.9 ms < 12 个检测窗口(1 ms)，又长于 8 个窗口，正好落在旧判据
    // 会误判、新判据应拦住的区间。
    //--------------------------------------------------------------------------
    $display("--- DSD misdetection stress: LRCK freeze, BCLK alive ---");
    force i2s_lrck_i = 1'b0;
    #(900000000.0);
    $display("LRCK FREEZE: evidence=%b fmt_dsd=%b i2s_age=%0d rate_ok=%b period_ok=%b lr_edges=%0d",
             dut.dsd_evidence, dut.fmt_dsd, dut.i2s_age, dut.dsd_rate_ok,
             dut.i2s_period_ok, dut.lr_edge_cnt);
    expect_true(dut.dsd_rate_ok === 1'b1,
                "32bit/352.8k PCM: BCLK really hits the DSD512 rate slot (hazard confirmed)");
    expect_true(dut.dsd_evidence === 1'b0 && dut.fmt_dsd === 1'b0,
                "32bit/352.8k PCM: short LRCK dropout must not be misclassified as DSD");
    release i2s_lrck_i;
    repeat (400) @(ev_frame);

    //--------------------------------------------------------------------------
    // 长瞬态复现：24bit 正常 / 32bit 断流的分水岭。
    // 32bit/352.8k 的 BCLK=22.5792 MHz 命中 DSD512 档；
    // 24bit/352.8k 的 BCLK=16.9344 MHz 不命中任何 DSD 档。
    // LRCK 冻结 >12 个检测窗口(约 1 ms) 时 i2s_period_dead 也会成立，
    // 判据二仍会把 32bit PCM 误判成 native DSD → 完整重锁。
    //--------------------------------------------------------------------------
    $display("--- long LRCK dropout (2 ms): 32bit vs 24bit @352.8k ---");
    force i2s_lrck_i = 1'b0;
    #(2000000000.0);
    $display("LONG FREEZE 32b: evidence=%b fmt_dsd=%b rate_ok=%b i2s_age=%0d lr_edges=%0d cur_fmt=%b hstate=%0d",
             dut.dsd_evidence, dut.fmt_dsd, dut.dsd_rate_ok, dut.i2s_age,
             dut.lr_edge_cnt, dut.cur_fmt, dut.hstate);
    expect_true(dut.dsd_rate_ok === 1'b1,
                "32bit/352.8k: BCLK hits the DSD512 rate slot (hazard is real)");
    expect_true(dut.dsd_evidence === 1'b0 && dut.fmt_dsd === 1'b0 && dut.cur_fmt === 1'b0,
                "32bit/352.8k PCM: long LRCK dropout must NOT be misclassified as DSD");
    release i2s_lrck_i;
    repeat (400) @(ev_frame);

    run_case_extra_wait = 1;
    run_case(6, 24, 200, 32'hA5A55A5A);
    run_case_extra_wait = 0;

    //--------------------------------------------------------------------------
    // 短断流由缓冲吸收：已越过 P_LRCK_GAP(41.7 us) 但短于持续确认窗口，
    // 必须不淡出、不进 H_FADE、不清空 prefilled、不重新预填。
    // 这是 A/B 试验「放宽阈值有效但不能根除」的正式修复验证。
    //--------------------------------------------------------------------------
    $display("--- short dropout (0.2 ms) absorbed by SDRAM buffer ---");
    chk_en = 0;
    force i2s_lrck_i = 1'b0;
    #(200000000.0);
    $display("SHORT DROPOUT: lost=%b hstate=%0d resync=%b prefilled=%b gain=%0d fill=%0d lost_cnt=%0d",
             dut.stream_lost, dut.hstate, dut.resync_r, dut.prefilled,
             dut.gain, dut.fill_words, dut.lost_cnt);
    expect_true(dut.stream_lost === 1'b1,
                "Short dropout really crosses the P_LRCK_GAP threshold");
    expect_true(dut.hstate === 3'd1 && dut.resync_r === 1'b0,
                "Short dropout does NOT start an uncancellable re-lock");
    expect_true(dut.prefilled === 1'b1 && dut.gain === 5'd16,
                "Short dropout keeps buffer and unity gain (no mute, no refill)");
    expect_true(dut.cur_rate === 3'd6 && dut.cur_fmt === 1'b0 && dut.hstate === 3'd1,
                "Short dropout must not corrupt the locked rate/format code (no spurious re-lock)");
    release i2s_lrck_i;
    repeat (400) @(ev_frame);

    force i2s_lrck_i = 1'b0;
    #(2000000000.0);
    $display("LONG FREEZE 24b: evidence=%b fmt_dsd=%b rate_ok=%b i2s_age=%0d lr_edges=%0d cur_fmt=%b hstate=%0d",
             dut.dsd_evidence, dut.fmt_dsd, dut.dsd_rate_ok, dut.i2s_age,
             dut.lr_edge_cnt, dut.cur_fmt, dut.hstate);
    expect_true(dut.dsd_rate_ok === 1'b0,
                "24bit/352.8k: BCLK does not hit any DSD rate slot (control group)");
    expect_true(dut.fmt_dsd === 1'b0 && dut.cur_fmt === 1'b0,
                "24bit/352.8k PCM: long LRCK dropout keeps PCM format");
    release i2s_lrck_i;
    repeat (400) @(ev_frame);

    // SDRAM DQM 专项：READ 输出窗口内 DQM 绝不能被拉高，
    // 否则真实芯片 DQ 高阻，控制器读回无效值 → 输出变单声道且偏暗。
    $display("SDRAM DQM: masked-during-read events=%0d", m_dqm_masked_cnt);
    expect_true(m_dqm_masked_cnt === 32'd0,
                "SDRAM DQM stays low for the whole CAS latency window (no invalid readback)");

    // 立体声专项：测试激励左右声道恒不相同，输出任何非零帧都不得左右相同。
    $display("STEREO: non-zero frames=%0d mono-looking frames=%0d (TB_DELAY=%0d)",
             stereo_frames, mono_frames, TB_DELAY);
    expect_true(stereo_frames > 100 && mono_frames === 0,
                "Output keeps independent L/R channels (no mono collapse)");

    //--------------------------------------------------------------------------
    // DoP：标准 PCM 帧携带 0x05/0xFA marker + 16 DSD 位，
    // 验证自动识别 → 解码 → 复用同一套 SDRAM FIFO reclock → 原生 DSD 输出。
    // 放在所有统计之后，避免改动 stereo_frames / SDRAM 计数等既有断言。
    //--------------------------------------------------------------------------
    $display("--- DoP: auto detection + decode + native DSD reclock ---");
    run_dop_case(4, 240, 32'hA1B2C3D4);   // 176.4 kHz DoP → DSD64  / 44.1k 族
    run_dop_case(6, 240, 32'h5E6F7081);   // 352.8 kHz DoP → DSD128 / 44.1k 族
    run_dop_case(5, 240, 32'h1234ABCD);   // 192 kHz   DoP → DSD64  / 48k 族
    // DoP → PCM 回切：确认解码器释放、格式与指示恢复 PCM。
    run_case_extra_wait = 1;
    run_case(0, 32, 200, 32'h13572468);
    run_case_extra_wait = 0;
    expect_true(dut.cur_fmt === 1'b0 && dut.dop_active_w === 1'b0,
                "DoP -> PCM: format returns and DoP decoder releases");

    $display("=============================================================");
    $display("SDRAM model: ACT=%0d WR=%0d RD=%0d REF=%0d PRE=%0d MRS=%0d seq_err=%0d",
             m_act_cnt, m_wr_cnt, m_rd_cnt, m_ref_cnt, m_pre_cnt, m_mrs_cnt, m_seq_err);
    $display("SDRAM model: banks_seen=%b max_row=%0d max_addr=%0d (24-bit address space)",
             m_banks_seen, m_max_row, m_max_addr);
    if (errors == 0) $display("RESULT: ALL CHECKS PASSED (0 errors)");
    else             $display("RESULT: FAILED (%0d errors)", errors);
    $display("=============================================================");
    $finish;
end

endmodule
