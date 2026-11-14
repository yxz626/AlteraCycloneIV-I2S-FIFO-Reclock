//==============================================================================
// Cy4_I2S_reclock.v  --  EP4CE6F17C8 核心板的 Hi-end I2S FIFO reclock 设计
// Quartus Prime 17.1 / Cyclone IV E / 工程“Cy4_I2Sq”
//
// 需求映射（见《工程要求说明.txt》）：
//  [R1] 自适应位深与采样率的 I2S Hi-end FIFO reclock。
//  [R2] 依据实测输入 LRCK 频率，在 45.1584 MHz（E15）与
//       49.152 MHz（E16）之间自动选择全局时钟。
//  [R3] 双时钟 FIFO + W9825G6JH SDRAM 环形缓冲（完整 24 位
//       地址空间 = 4 bank × 8192 行 × 512 列 × 16 bit = 256 Mbit）。
//  [R4] LED0~LED3 采样率指示（高有效）。
//  [R5] key_k3（低有效）切换 AD1862 20 位 DR/DL 输出模式，
//       AD1862 时序符合数据手册 Fig.12，该模式下四颗 LED 全亮。
//  [R6] K1/K2 已拆除；两路纯净时钟接 E15 / E16。
//  [R7] 输出 EPCS4 的 .jic 编程文件（由 quartus_cpf 生成）。
//
//  [DSD] 自动识别输入是 I2S 还是 native DSD，并对 DSD 位流做
//        同样的 FIFO + SDRAM reclock，之后在原 reclock 排针上
//        原生输出 1-bit DSD（不做 PCM 变换）。引脚复用约定
//        （Amanero / XMOS / AK4493 常用的 3 线 native DSD）：
//          输入：i2s_bclk_i = DCLK，i2s_lrck_i = DSD_L，
//                i2s_sdata_i = DSD_R；
//          输出：i2s_bclk_o = DCLK，i2s_lrck_o = DSD_L，
//                i2s_sdata_o = DSD_R，i2s_mclk_o = MCLK。
//        数据在 DCLK 下降沿改变、上升沿被锁存。DSD64/128/256/512
//        分别对应 45.1584(49.152)MHz 的 /16、/8、/4、/2 分频。
//
// Hi-end 措施：音频输出域仅由所选
// 纯净晶振经专用硬件时钟控制块
// （ALTCLKCTRL）驱动；所有输出都在该单一全局
// 时钟上以时钟使能脉冲直接寄存（绝不使用行波/门控派生时钟）；输入
// 抖动被异步 FIFO 与大容量 SDRAM
// 环形缓冲完全隔离；字锁存沿（LE / LRCK）为主时钟整数分频，
// 即零附加抖动。
//==============================================================================
`timescale 1ns/1ps

//------------------------------------------------------------------------------
// 顶层模块：实例名与 Cy4_I2Sq.qsf 中的 TOP_LEVEL_ENTITY 一致。
//------------------------------------------------------------------------------
module Cy4_I2Sq (
    input  wire        clk_45m,        // 引脚 E15：45.1584 MHz（44.1 kHz 族）
    input  wire        clk_49m,        // 引脚 E16：49.1520 MHz（48 kHz 族）
    input  wire        i2s_bclk_i,     // 引脚 L4
    input  wire        i2s_lrck_i,     // 引脚 P1
    input  wire        i2s_sdata_i,    // 引脚 N1
    output wire        i2s_mclk_o,     // 引脚 T9
    output wire        i2s_bclk_o,     // 引脚 T10
    output wire        i2s_lrck_o,     // 引脚 T11
    output wire        i2s_sdata_o,    // 引脚 T12
    output wire        do_l,           // 引脚 N9：AD1862（左声道芯片）DATA
    output wire        do_r,           // 引脚 N11：AD1862（右声道芯片）DATA
    output wire        ad_bclk,        // 引脚 R9：AD1862 CLK = 24 × FS
    output wire        ad_le,          // 引脚 T13：AD1862 锁存使能，低有效
    output wire [3:0]  led,            // 引脚 R13/T14/R14/T15，高有效
    output wire        dsd_on,         // 引脚 F2：DSDOE，输出 native DSD 时为高
    output wire [3:0]  fsel,           // F0/F1/F2/F3 = C2/A2/A3/A4，见 f0~F3.txt
    input  wire        key_k3,         // 引脚 N15，低有效
    output wire        sdram_clk,      // 引脚 B14
    output wire        sdram_cke,      // 引脚 F16
    output wire        sdram_cs_n,     // 引脚 K10
    output wire        sdram_ras_n,    // 引脚 K11
    output wire        sdram_cas_n,    // 引脚 J12
    output wire        sdram_we_n,     // 引脚 J13
    output wire [1:0]  sdram_ba,       // 引脚 F13/G11
    output wire [12:0] sdram_addr,     // 见核心板 IO 引脚分配表
    inout  wire [15:0] sdram_data,     // 见核心板 IO 引脚分配表
    output wire [1:0]  sdram_dqm       // 引脚 G15 / J14（DQM1 / DQM0）
);

//------------------------------------------------------------------------------
// 参数（可在功能仿真测试平台中覆盖）
//------------------------------------------------------------------------------
parameter [23:0] P_FILL_TARGET = 24'h040000;   // 预填 262144 字（约 1.4 s @48 kHz）
                                                // 整个环形缓冲为 24'h1000000 字
// 按键消抖：20 ms 对机械按键偏长，快速点按（<20 ms）会完全没有
// 反应，表现为「K3 失效」。10 ms 足以滤除触点抖动，又能覆盖短按。
parameter [19:0] P_DEBOUNCE    = 20'd500000;   // 约 10 ms @49.152 MHz
// 1 = 标准 Philips I2S（LRCK 沿后第 2 个 BCLK 才是 MSB，《工程要求说明》R5 的"标准 I2S"）
// 0 = 左对齐（LRCK 沿后第 1 个 BCLK 即 MSB）；若音源/DAC 是左对齐，把此值改 0 重新编译。
parameter        P_I2S_DELAY   = 1'b1;
// AK4493 兼容：datasheet Table 22 中 I2S 兼容模式（Mode 3/7）的 LRCK 极性是 L/H（低为左），
// 而 MSB/LSB justified 模式（含默认 Mode 6）是 H/L（高为左），两者相反。
// 0 = LRCK 低电平为左声道；1 = LRCK 高电平为左声道。
parameter        P_LRCK_POL    = 1'b0;
// 1 = 交换左右声道（用于 DAC 声道定义或板级 L/R 接线相反时）。
parameter        P_SWAP_LR     = 1'b0;
// Hi-end 优化①③ 在 I2S 输出路径上的开关。单声道故障的根因已确认是
// SDRAM DQM 读窗口屏蔽，与增益路径无关，故默认启用完整 Hi-end 处理：
// 0（默认）= I2S 输出施加增益淡入淡出，slip 与切换只在静音帧提交；
// 1 = 临时旁路为优化前的纯位精确直通，仅供板级调试对照使用。
parameter        P_I2S_LEGACY  = 1'b0;
// 1 = 声道诊断：左声道正常、右声道强制静音，用于判断单声道来自 FPGA 还是 DAC/后级。
parameter        P_CH_TEST     = 1'b0;
parameter [15:0] P_INIT_WAIT   = 16'd12000;    // SDRAM 上电等待 200 µs
parameter [15:0] P_REF_INT     = 16'd320;      // 自动刷新周期（< 7.8125 µs）
parameter        P_FIFO_AW     = 8;            // 每个 FIFO 256 项
// Hi-end 优化开关（见《验证报告.md》/《工程概述.md》）：
parameter        P_DITHER_EN   = 1'b1;         // 20 位 DAC 路径的 TPDF 抖动 + 舍入
parameter [23:0] P_SLIP_LOW    = 24'd8192;     // 低水位 → 淡出后插入一帧
parameter [23:0] P_SLIP_HIGH   = 24'hFFFDF0;   // 高水位 → 淡出后丢弃一帧
parameter [23:0] P_GUARD       = 24'd64;       // 硬欠流 → 纯净静音
// 两次 slip 之间的最小帧间隔（按 LRCK 帧计，不是 core 时钟）。
// 水位触到阈值是「电平」条件：一次 slip 只补 1 个帧对（4 字），
// 不足以让水位脱离阈值。若冷却只有 64 个 core 时钟（远短于一帧），
// slip_req 会在一次 33 帧的 fade 尚未结束时就被重新置位，发射器
// 陷入连续的「淡出→静音→淡入」循环：352.8 kHz 档调制约 10.7 kHz，
// 听感就是持续的嘶嘶噪声。512 帧 >> 一次 fade 的 33 帧，又远小于
// 两晶振 ppm 级偏差自然需要的两次 slip 间隔，因此既不限流补偿，
// 又能保证绝大多数时间处于单位增益稳态播放。
parameter [9:0]  P_SLIP_GAP    = 10'd512;
// 手动切歌：播放源停止翻转 LRCK，随后以
// 全新的帧相位重新起流；在常开的 49.152 MHz 域检测该间隙。
// 2048 周期 = 41.7 µs，约为最长 LRCK 半帧（11.34 µs @44.1 kHz）的 3.7 倍。
parameter [15:0] P_LRCK_GAP    = 16'd2048;
// 断流「持续确认」窗口：stream_lost 必须连续保持这么久才判定为
// 真正的切歌/停播并走完整重锁。短于此值的音源瞬态（树莓派 ALSA/DMA
// 欠载）由 SDRAM 缓冲直接吸收：不淡出、不复位、不清空 prefilled、
// 不重新预填，输出连续。原实现只要 lrck_gap 越过 41.7 us 就立刻
// 进入不可取消的 H_FADE，哪怕流在 50 us 后已恢复，仍要走完
// 488 us 淡出 + 重新预填 262144 字（352.8 kHz 档约 186 ms 静音），
// 这正是「周期性卡顿」；单纯放大 P_LRCK_GAP 只能减少触发次数，
// 无法根除（一旦超过阈值仍然完整重锁），A/B 试验结果与此吻合。
// 缓冲 262144 字在 352.8 kHz 档约 186 ms；50 ms 断流仅消耗约
// 70560 字，剩余 191584 字仍远高于 P_SLIP_LOW=8192，不会触发 slip。
parameter [21:0] P_LOST_CONFIRM = 22'd2457600;  // 50 ms @49.152 MHz
// 速率/格式变化「连续有效测量」确认次数：cand_all 与 cur_all 不一致
// 必须连续 N 次 det_all（每次代表 8 帧周期测量 + 2 窗口稳定）才提交
// 重锁。输入 BCLK 很高（32bit/352.8k 达 22.5792 MHz）时，LRCK 上的
// 偶发毛刺/边沿抖动会让周期测量瞬时落到相邻档位，cand_all 短暂偏离；
// 原实现一采到就进入不可取消的 H_FADE，重新预填 262144 字
// （352.8k 档约 186 ms 静音），听感正是「随机短时按暂停」。
// 按测量次数而非绝对时间计数：低速档测量恢复期长也不会误填满，
// 真正的速率/格式切换连续偏离 4 次即提交（352.8k 档约 0.18 ms，
// 44.1k 档约 1.5 ms），不影响正常使用。
parameter [2:0]  P_CHG_CONFIRM  = 3'd4;
// DSD 支持开关：0 = 完全退回纯 I2S 行为，1 = 自动识别 I2S/native DSD。
parameter        P_DSD_EN      = 1'b1;
// DSD 模式下让速率档 LED 慢闪，以便与同档位的 PCM 播放区分（0 = 常亮）。
parameter        P_DSD_BLINK   = 1'b1;

localparam [23:0] CAP_WORDS = 24'hFFFFF0;      // 可用环形缓冲字数（整片）
// 格式/位时钟测量窗口：4096 个 clk_49m 周期 = 83.33 us，另留 16 周期闭锁。
localparam [12:0] DSD_WIN       = 13'd4096;
localparam [12:0] DSD_WIN_TOTAL = 13'd4112;
// 窗口内 DCLK 上升沿计数的期望值（8 档 DSD 位速率）。
// 44.1k 族：45.1584M/16,8,4,2；48k 族：49.152M/16,8,4,2。
localparam [15:0] DCNT0 = 16'd235,  DCNT1 = 16'd256,  DCNT2 = 16'd470,  DCNT3 = 16'd512,
                  DCNT4 = 16'd941,  DCNT5 = 16'd1024, DCNT6 = 16'd1882, DCNT7 = 16'd2048;
localparam [7:0]  DSD_SIL_PAT = 8'h69;   // 50% 密度的 DSD 静音码型

// 以 clk_49m 周期计的 LRCK 周期累加值（8 帧）：
// 49.152 MHz / FS × 8，FS 取 44.1k、48k、88.2k、96k、176.4k、192k、352.8k、384k
localparam [15:0] S0 = 16'd8916, S1 = 16'd8192, S2 = 16'd4458, S3 = 16'd4096,
                  S4 = 16'd2229, S5 = 16'd2048, S6 = 16'd1115, S7 = 16'd1024;
localparam [15:0] STOL = 16'd16;

//--- DoP (DSD over PCM) v1.1 支持 --------------------------------------
// 规范：24-bit PCM 帧的高 8 位为 marker，同一帧 L/R 声道相同，
// 相邻帧在 0x05 / 0xFA 之间严格交替；低 16 位是该声道连续的
// 16 个 DSD 位（MSB = 最早到达的 t0）。PCM→DSD 需 32 个连续
// marker，DSD→PCM 只需一个缺失 marker（此处加几帧迟滞抗毛刺）。
// 本工程只支持 DSD64 (176.4/192 kHz 承载) 与 DSD128
// (352.8/384 kHz 承载)，解码后位速率远低于 DSD512，SDRAM 无瓶颈。
parameter        P_DOP_EN      = 1'b1;
parameter [5:0]  P_DOP_CONFIRM = 6'd32;   // 规范：32 个连续 marker
parameter [2:0]  P_DOP_MISS    = 3'd4;    // 退出迟滞：连续 4 帧 marker 缺失

//==============================================================================
// 1. 管理域（常开 clk_49m，绕过音频时钟 MUX）：
//    上电复位、采样率/晶振族检测、按键消抖，
//    以及无毛刺全局时钟切换状态机。
//==============================================================================
reg  [15:0] por_cnt;
reg  [15:0] hcnt;
reg  [2:0]  hstate;
reg         clk_ena_r;      // ALTCLKCTRL 时钟使能（切换期间关断）
reg         clk_sel_r;      // 0 → 45.1584 MHz，1 → 49.152 MHz
reg         g_arst_n_r;     // 全局异步置位复位
reg         mode_r;         // 0 = 标准 I2S，1 = 增加 AD1862 20 位输出
reg         switch_mute_r;  // 在任何时钟/复位切换前请求淡出
reg         resync_r;       // 输入流丢失后的重新锁定进行中
reg         cur_valid;      // 至少成功检测到一次有效速率
reg         cur_family;
reg         cur_fmt;        // 已提交的输入格式：0 = I2S/PCM，1 = native DSD
reg  [2:0]  cur_rate;
reg  [3:0]  new_all;        // {is_dsd, rate_code[2:0]}，切换时提交

initial begin
    por_cnt = 0; hcnt = 0; hstate = 0;
    clk_ena_r = 1'b0; clk_sel_r = 1'b0; g_arst_n_r = 1'b0;
    mode_r = 1'b0; cur_valid = 1'b0; cur_family = 1'b0; cur_fmt = 1'b0;
    switch_mute_r = 1'b0; resync_r = 1'b0;
    cur_rate = 3'd0; new_all = 4'd0;
end
wire [3:0] cur_all = {cur_fmt, cur_rate};

wire por_done = (por_cnt >= 16'd1024);
always @(posedge clk_49m) begin
    if (por_cnt != 16'hFFFF) por_cnt <= por_cnt + 16'd1;
end

//--- 输入流存在性监测 -------------------------------------------
// 手动切歌会让播放源 LRCK 停止数十微秒甚至更久，
// 随后以全新帧相位重新起流。仅在常开的
// clk_49m 域采样（LRCK ≤ 384 kHz，采样无歧义；不使用 BCLK，
// 因为 384 kHz × 64 时 BCLK 达 24.576 MHz，本域每周期
// 仅 2 个采样点，可能漏掉边沿）。
reg  [1:0]  lrck_mon;
reg [15:0] lrck_gap;
wire lrck_edge = (lrck_mon[1] != lrck_mon[0]);
initial begin lrck_mon = 2'b00; lrck_gap = 16'd0; end
always @(posedge clk_49m) begin
    lrck_mon <= {lrck_mon[0], i2s_lrck_i};
    if (lrck_edge) lrck_gap <= 16'd0;
    else if (lrck_gap != 16'hFFFF) lrck_gap <= lrck_gap + 16'd1;
end

//--- DSD 位时钟活动监测 -----------------------------------------
// DSD 时 LRCK 引脚上跑的是左声道 1-bit 数据，静音码型也可能是
// 长时间恒定电平，因此不能再靠 LRCK 跳变判断断流。改为在
// BCLK 域把 DCLK 分频成心跳，再同步到常开域看门狗计数。
// DSD64 时心跳周期约 5.7 us，远小于 P_LRCK_GAP(41.7 us)。
reg  [3:0]  hb_div;
reg         hb;
reg         hb_s1, hb_s2, hb_s2_d;
reg [15:0]  hb_gap;
initial begin hb_div = 0; hb = 0; hb_s1 = 0; hb_s2 = 0; hb_s2_d = 0; hb_gap = 16'hFFFF; end
always @(posedge i2s_bclk_i) begin
    hb_div <= hb_div + 4'd1;
    if (hb_div == 4'd15) hb <= ~hb;
end
always @(posedge clk_49m) begin
    hb_s1   <= hb;
    hb_s2   <= hb_s1;
    hb_s2_d <= hb_s2;
    if (hb_s2 != hb_s2_d) hb_gap <= 16'd0;
    else if (hb_gap != 16'hFFFF) hb_gap <= hb_gap + 16'd1;
end
wire bclk_alive = (hb_gap < P_LRCK_GAP);

// I2S 沿用 LRCK 间隙判据；DSD 用 DCLK 心跳判据。
wire stream_lost = cur_fmt ? !bclk_alive : (lrck_gap >= P_LRCK_GAP);
// 播放源断开期间持续清零速率测量状态，使新流从干净状态开始测量
// （无陈旧/回绕的 per_cnt 饱和值）。注意必须保持 stream_lost 电平，
// 不能改成「仅重锁流程内清理」：H_RESYNC 正是靠重新测量产生 det_all
// 才能锁定，若在该状态持续清零测量就会死锁在 H_RESYNC。
// 短断流造成的测量重置由 P_CHG_CONFIRM（10 ms 持续确认）兜底，
// 恢复期短暂的档位偏离不会触发完整重锁。
wire meas_scrub = stream_lost;

// 断流持续确认计数器（常开 clk_49m 域）。stream_lost 是电平信号，
// LRCK 一恢复就立即撤销；只有连续保持 P_LOST_CONFIRM 才提交重锁。
reg [21:0] lost_cnt;
initial lost_cnt = 22'd0;

// 速率/格式变化的连续有效测量计数，避免瞬时测量偏离触发完整重锁。
reg [2:0] chg_cnt;
initial chg_cnt = 3'd0;

//--- LRCK 周期测量（8 帧平均），clk_49m 域 ---------------
reg  [1:0]  lr_s;
reg  [15:0] per_cnt, per_sum, meas_sum;
reg  [2:0]  per_idx;
reg         meas_valid;
wire        lr_rise = (lr_s == 2'b01);

initial begin
    lr_s = 0; per_cnt = 0; per_sum = 0; meas_sum = 0; per_idx = 0; meas_valid = 0;
end

always @(posedge clk_49m) begin
    lr_s <= {lr_s[0], i2s_lrck_i};
    meas_valid <= 1'b0;
    if (meas_scrub) begin
        per_cnt  <= 16'd0;
        per_sum  <= 16'd0;
        meas_sum <= 16'd0;
        per_idx  <= 3'd0;
    end else if (lr_rise) begin
        if (per_idx == 3'd7) begin
            meas_sum   <= per_sum + per_cnt;
            meas_valid <= 1'b1;
            per_sum    <= 16'd0;
            per_idx    <= 3'd0;
        end else begin
            per_sum <= per_sum + per_cnt;
            per_idx <= per_idx + 3'd1;
        end
        per_cnt <= 16'd0;
    end else begin
        // 采用饱和而非回绕：播放源断流后的第一次
        // 测量不能基于已回绕的周期计数值。
        if (per_cnt != 16'hFFFF) per_cnt <= per_cnt + 16'd1;
    end
end

//--- 速率分类 -----------------------------------------------------
wire [7:0] rate_match;
assign rate_match[0] = (meas_sum >= S0 - STOL) && (meas_sum <= S0 + STOL); // 44.1k
assign rate_match[1] = (meas_sum >= S1 - STOL) && (meas_sum <= S1 + STOL); // 48k
assign rate_match[2] = (meas_sum >= S2 - STOL) && (meas_sum <= S2 + STOL); // 88.2k
assign rate_match[3] = (meas_sum >= S3 - STOL) && (meas_sum <= S3 + STOL); // 96k
assign rate_match[4] = (meas_sum >= S4 - STOL) && (meas_sum <= S4 + STOL); // 176.4k
assign rate_match[5] = (meas_sum >= S5 - STOL) && (meas_sum <= S5 + STOL); // 192k
assign rate_match[6] = (meas_sum >= S6 - STOL) && (meas_sum <= S6 + STOL); // 352.8k
assign rate_match[7] = (meas_sum >= S7 - STOL) && (meas_sum <= S7 + STOL); // 384k
wire rate_onehot = (rate_match != 8'd0) && ((rate_match & (rate_match - 8'd1)) == 8'd0);

reg  [2:0] cand_code, last_cand;
reg  [1:0] stable_cnt;
reg        det_valid;

always @* begin
    case (1'b1)
        rate_match[7]: cand_code = 3'd7;
        rate_match[6]: cand_code = 3'd6;
        rate_match[5]: cand_code = 3'd5;
        rate_match[4]: cand_code = 3'd4;
        rate_match[3]: cand_code = 3'd3;
        rate_match[2]: cand_code = 3'd2;
        rate_match[1]: cand_code = 3'd1;
        default:       cand_code = 3'd0;
    endcase
end

initial begin
    last_cand = 3'd7; stable_cnt = 0; det_valid = 0;
end

always @(posedge clk_49m) begin
    det_valid <= 1'b0;
    if (meas_scrub) begin
        last_cand  <= 3'd7;
        stable_cnt <= 2'd0;
    end else if (meas_valid) begin
        if (rate_onehot) begin
            if (cand_code == last_cand) begin
                if (stable_cnt >= 2'd2) begin
                    det_valid <= 1'b1;
                end else begin
                    stable_cnt <= stable_cnt + 2'd1;
                end
            end else begin
                last_cand  <= cand_code;
                stable_cnt <= 2'd0;
            end
        end else begin
            last_cand  <= 3'd7;
            stable_cnt <= 2'd0;
        end
    end
end

// 速率码 bit0 = 晶振族（0=44.1k → 45.1584 MHz，1=48k → 49.152 MHz）
// det_valid 有效时提交的是 cand_code 的值。

//==============================================================================
// 1b. I2S / native DSD 自动格式识别（常开 clk_49m 域）
//   判据一：LRCK 引脚跳变密度。I2S 的 LRCK 是帧时钟，每
//           16/20/24/32 个 BCLK 才跳一次；DSD 时该引脚是左声道
//           1-bit 码流，跳变密度高得多；用 DCLK 计数归一化后
//           与速率无关。
//   判据二：DCLK 频率落在 8 档 DSD 位速率之一，且近期没有有效的
//           I2S LRCK 周期测量（覆盖双声道恒定电平的静音码型）。
//   两个判据都需连续 3 个窗口成立才提交，避免切换瞬态误判。
//==============================================================================
reg  [12:0] win_cnt, lr_edge_cnt;
wire        win_end   = (win_cnt == DSD_WIN_TOTAL - 13'd1);
wire        meas_gate = (win_cnt < DSD_WIN);
initial begin win_cnt = 13'd0; lr_edge_cnt = 13'd0; end
always @(posedge clk_49m)
    win_cnt <= win_end ? 13'd0 : (win_cnt + 13'd1);
always @(posedge clk_49m) begin
    if (win_end) lr_edge_cnt <= 13'd0;
    else if (lrck_edge) lr_edge_cnt <= lr_edge_cnt + 13'd1;
end

//--- BCLK(=DCLK) 域：窗口内计数 DCLK 上升沿，并闭锁结果 ---------
reg         gate_b1, gate_b2, gate_b2_d;
reg [15:0]  bcnt, bcap;
reg         cap_tog;
initial begin gate_b1=0; gate_b2=0; gate_b2_d=0; bcnt=0; bcap=0; cap_tog=0; end
always @(posedge i2s_bclk_i) begin
    gate_b1   <= meas_gate;
    gate_b2   <= gate_b1;
    gate_b2_d <= gate_b2;
    if (gate_b2) begin
        if (bcnt != 16'hFFFF) bcnt <= bcnt + 16'd1;
    end else begin
        if (gate_b2_d) begin bcap <= bcnt; cap_tog <= ~cap_tog; end
        bcnt <= 16'd0;
    end
end

//--- 闭锁结果回传常开域（值稳定，用 toggle 握手同步）-----------
reg         tog_s1, tog_s2, tog_s2_d;
reg [15:0]  bcap_49;
reg [3:0]   cap_age, i2s_age;
initial begin
    tog_s1=0; tog_s2=0; tog_s2_d=0; bcap_49=0; cap_age=4'hF; i2s_age=4'hF;
end
wire cap_new = (tog_s2 != tog_s2_d);
always @(posedge clk_49m) begin
    tog_s1 <= cap_tog; tog_s2 <= tog_s1; tog_s2_d <= tog_s2;
    if (cap_new) bcap_49 <= bcap;
    if (cap_new) cap_age <= 4'd0;
    else if (win_end && cap_age != 4'hF) cap_age <= cap_age + 4'd1;
    if (meas_valid && rate_onehot) i2s_age <= 4'd0;
    else if (win_end && i2s_age != 4'hF) i2s_age <= i2s_age + 4'd1;
end
wire dsd_rate_fresh = (cap_age < 4'd4);
wire i2s_period_ok  = (i2s_age < 4'd8);

//--- DSD 速率码：与 I2S 共用 {档位[2:1], 晶振族[0]} 编码 -------
wire [15:0] dtol = (bcap_49 >> 6) + 16'd4;
wire [7:0]  dsd_match;
assign dsd_match[0] = (bcap_49 >= DCNT0 - dtol) && (bcap_49 <= DCNT0 + dtol);
assign dsd_match[1] = (bcap_49 >= DCNT1 - dtol) && (bcap_49 <= DCNT1 + dtol);
assign dsd_match[2] = (bcap_49 >= DCNT2 - dtol) && (bcap_49 <= DCNT2 + dtol);
assign dsd_match[3] = (bcap_49 >= DCNT3 - dtol) && (bcap_49 <= DCNT3 + dtol);
assign dsd_match[4] = (bcap_49 >= DCNT4 - dtol) && (bcap_49 <= DCNT4 + dtol);
assign dsd_match[5] = (bcap_49 >= DCNT5 - dtol) && (bcap_49 <= DCNT5 + dtol);
assign dsd_match[6] = (bcap_49 >= DCNT6 - dtol) && (bcap_49 <= DCNT6 + dtol);
assign dsd_match[7] = (bcap_49 >= DCNT7 - dtol) && (bcap_49 <= DCNT7 + dtol);
wire dsd_onehot = (dsd_match != 8'd0) && ((dsd_match & (dsd_match - 8'd1)) == 8'd0);
reg  [2:0]  dsd_code;
always @* begin
    case (1'b1)
        dsd_match[7]: dsd_code = 3'd7;
        dsd_match[6]: dsd_code = 3'd6;
        dsd_match[5]: dsd_code = 3'd5;
        dsd_match[4]: dsd_code = 3'd4;
        dsd_match[3]: dsd_code = 3'd3;
        dsd_match[2]: dsd_code = 3'd2;
        dsd_match[1]: dsd_code = 3'd1;
        default:      dsd_code = 3'd0;
    endcase
end
wire dsd_rate_ok = dsd_rate_fresh && dsd_onehot && (bcap_49 > 16'd64);

wire dsd_dense        = dsd_rate_fresh && ({lr_edge_cnt, 3'b000} > (bcap_49 + 16'd16));
wire i2s_frame_alive  = (lr_edge_cnt >= 13'd4);
// 32bit / 352.8 kHz 的 BCLK = 64×FS = 22.5792 MHz，恰好等于
// DSD512 / 44.1k 族的 DCLK，是 8 档 PCM 中唯一让 dsd_rate_ok
// 恒为真的组合（20bit 档 BCLK 仅 14.112 MHz，不命中任何 DSD 档）。
// 若音源出现瞬态（缓冲欠载、LRCK 短暂停跳），lr_edge_cnt 会跌到
// <4，i2s_period_ok 也会在 8 个窗口后失效，于是判据二短暂成立而把
// PCM 误判成 native DSD：cand_all 变化 → 完整重锁（淡出 488 µs +
// 重新预填 262144 字，352.8k 档约 0.2 s 静音）；同时 DSD_ON 抖动
// 会让后级 DAC 反复切 PCM/DSD 模式，听感就是周期性「停顿一下」。
// 判据二加「I2S 帧周期测量已持续失效 ≥12 个窗口」的时间门槛，
// 能挡住 <1 ms 的瞬态，但挡不住数 ms 的 DMA 欠载（i2s_age 饱和到 15）。
// 物理上「PCM 暂停（LRCK 静默、BCLK 继续）」与「DSD 恒定电平静音
// 码型」在引脚波形上完全相同，唯一可靠的区分依据是此前已锁定的格式：
//   · PCM 已锁定 → 只允许判据一（LRCK 跳变密度）切到 DSD；
//     任何暂停/瞬态都不改变格式，DSD_ON 也不会抖动。
//   · 未锁定或当前已是 DSD → 才启用判据二，保证「上电即为恒定电平
//     DSD 静音码型」和「DSD 切歌进入静音段」仍能被识别。
// 代价：从 PCM 直接切到一段全静音的 DSD 时，识别会延后到出现实际
// 码流（此时判据一生效）；正常音乐开头都有跳变，不受影响。
// 「PCM 帧时钟历史」锁存，用它替代 cur_valid && !cur_fmt：
//   · 旧条件在 POR 后 cur_valid=0 期间失效。若音源 BCLK 先出、LRCK 后到
//     （播放器启动顺序，32bit/352.8k 时 BCLK=22.5792 MHz 恰好命中 DSD512
//     档），判据二会成立并把 fmt_dsd 置 1；此后 cur_fmt=1 使
//     「cur_valid && !cur_fmt」永久为 0 —— 保护自我失效，PCM 在 DSD 与
//     PCM 之间反复切换，表现为持续卡顿。31bit/352.8k 的 BCLK=21.8736 MHz
//     不命中任何 DSD 档，所以完全正常，这正是「只有 32bit 卡」的分水岭。
//   · 改为锁存：默认按 PCM 处理（初值 1），LRCK 出现正常帧跳变即保持
//     置位，只有确凿的高密度 DSD 码流（dsd_dense）才清除。这样一次瞬态
//     不会污染保护状态；真 DSD 码流仍由判据一即时识别。
//   代价：上电即为「恒定电平 DSD 静音码型」时不再自动识别，需出现实际
//     码流（正常音乐开头都有跳变）。该场景极罕见，优先保证 PCM 绝不误判。
reg pcm_seen;
initial pcm_seen = 1'b1;
always @(posedge clk_49m) begin
    if (win_end) begin
        // 确凿的 DSD 码流，或已确认处于 DSD 且位时钟仍命中档位
        // （覆盖恒定电平静音码型），都保持「非 PCM」锁存。
        if (dsd_dense || (fmt_dsd && dsd_rate_ok)) pcm_seen <= 1'b0;
        else if (i2s_frame_alive)                  pcm_seen <= 1'b1;
    end
end
wire pcm_locked       = pcm_seen;
wire i2s_period_dead  = (i2s_age >= 4'd12);
wire dsd_evidence     = P_DSD_EN && (dsd_dense ||
                        (!pcm_locked && dsd_rate_ok && !i2s_period_ok &&
                         !i2s_frame_alive && i2s_period_dead));

reg  [1:0] fmt_cnt;
reg        fmt_dsd;
initial begin fmt_cnt = 2'd0; fmt_dsd = 1'b0; end
always @(posedge clk_49m) begin
    if (win_end) begin
        if (dsd_evidence) begin
            if (fmt_cnt != 2'd3) fmt_cnt <= fmt_cnt + 2'd1;
        end else if (fmt_cnt != 2'd0) fmt_cnt <= fmt_cnt - 2'd1;
        if (dsd_evidence && fmt_cnt == 2'd2) fmt_dsd <= 1'b1;
        else if (!dsd_evidence && fmt_cnt == 2'd1) fmt_dsd <= 1'b0;
    end
end

// DSD 速率连续两个窗口一致才产生一次可提交事件。
reg  [2:0] dsd_last;
reg  [1:0] dsd_stable;
reg        dsd_det_valid;
initial begin dsd_last = 3'd7; dsd_stable = 2'd0; dsd_det_valid = 1'b0; end
always @(posedge clk_49m) begin
    dsd_det_valid <= 1'b0;
    if (win_end) begin
        if (fmt_dsd && dsd_evidence && dsd_rate_ok) begin
            if (dsd_code == dsd_last) begin
                if (dsd_stable >= 2'd1) dsd_det_valid <= 1'b1;
                else dsd_stable <= dsd_stable + 2'd1;
            end else begin
                dsd_last   <= dsd_code;
                dsd_stable <= 2'd0;
            end
        end else dsd_stable <= 2'd0;
    end
end

//--- DoP 识别结果（来自输入 BCLK 域，两级同步到管理域）---------------
// dop_active_w 由 dop_dec 在 i2s_bclk_i 域产生；此处只做电平同步，
// 与 PCM/native DSD 的窗口积分器互不干扰。
reg [1:0] dop_s;
initial dop_s = 2'd0;
always @(posedge clk_49m) dop_s <= {dop_s[0], dop_active_w};
wire dop_det = P_DOP_EN && dop_s[1];

// DoP 承载 PCM 速率码 → 解码后的 native DSD 档位码。
// DSD64：176.4 kHz(44.1k 族)/192 kHz(48k 族) 帧，每帧 16 DSD 位
//         → 2.8224 / 3.072 MHz；
// DSD128：352.8 kHz(44.1k 族)/384 kHz(48k 族) 帧
//         → 5.6448 / 6.144 MHz。
// 低于 176.4 kHz 的 PCM 帧率不足以承载 DSD64，不认定为 DoP。
reg [2:0] dop_code;
always @* begin
    case (cand_code)
        3'd4:    dop_code = 3'd0;   // 176.4k → DSD64 / 44.1k 族
        3'd5:    dop_code = 3'd1;   // 192k   → DSD64 / 48k 族
        3'd6:    dop_code = 3'd2;   // 352.8k → DSD128 / 44.1k 族
        3'd7:    dop_code = 3'd3;   // 384k   → DSD128 / 48k 族
        default: dop_code = 3'd7;
    endcase
end
wire dop_ok = (cand_code >= 3'd4);

// DoP 优先级高于「PCM 速率检测」但低于「确凿的 native DSD 码流」：
// 只有 fmt_dsd=0 且 dop_det=1 才把 cand_all 改成 DSD 档位，
// 从而触发 H_FADE → H_RST 清空缓冲、把输出侧切到 dsd_core=1。
wire [3:0] cand_all = (dop_det && dop_ok) ? {1'b1, dop_code} :
                      (fmt_dsd           ? {1'b1, dsd_code} :
                                           {1'b0, cand_code});
wire       det_all  = fmt_dsd ? dsd_det_valid : det_valid;

//--- key_k3 消抖与模式翻转 -------------------------------------------
reg  [1:0]  key_s;
reg         key_stable, key_low_d;
reg  [19:0] db_cnt;
initial begin key_s = 2'b11; key_stable = 1'b1; key_low_d = 1'b0; db_cnt = 0; end

wire db_done = (db_cnt == P_DEBOUNCE);
always @(posedge clk_49m) begin
    key_s <= {key_s[0], key_k3};
    if (key_s[1] != key_stable) begin
        key_stable <= key_s[1];
        db_cnt     <= 20'd0;
    end else if (!db_done) begin
        db_cnt <= db_cnt + 20'd1;
    end
end
wire key_low   = (key_stable == 1'b0) && db_done;
wire key_press = key_low && !key_low_d;
always @(posedge clk_49m) begin
    key_low_d <= key_low;
    if (key_press) mode_r <= ~mode_r;
end

//--- 无毛刺全局时钟切换状态机 ---------------------------------------
localparam [2:0] H_POR = 3'd0, H_IDLE = 3'd1, H_OFF = 3'd2,
                 H_SEL = 3'd3, H_ON  = 3'd4, H_RST = 3'd5,
                 H_FADE = 3'd6, H_RESYNC = 3'd7;

always @(posedge clk_49m) begin
    if (!por_done) begin
        hstate     <= H_POR;
        hcnt       <= 16'd0;
        clk_ena_r  <= 1'b0;
        g_arst_n_r <= 1'b0;
        lost_cnt   <= 22'd0;
        chg_cnt    <= 3'd0;
    end else begin
        case (hstate)
            H_POR: begin
                hstate <= H_IDLE;
            end
            H_IDLE: begin
                g_arst_n_r <= 1'b1;
                // 稳态必须有时钟。clk_ena_r 原先只在 H_ON 里置 1，
                // 但上电时若音源尚未起流，H_IDLE 会先判 stream_lost
                // → H_FADE → H_RESYNC；44.1k 族（cand_all[0]=0，
                // 如 352.8 kHz）与 cur_family 初值 0 相同，会走
                // 「同族直接 H_RST」而跳过 H_OFF/H_SEL/H_ON，
                // 于是 clk_core 永远停摆：rate_s2/mode_s 不更新 →
                // LED0 常亮、按 K3 无反应、AD1862 无声。
                // H_IDLE 时 MUX 选择已确定（POR 后为默认值，或刚由
                // H_ON 提交），开时钟不会产生切换毛刺。
                clk_ena_r  <= 1'b1;
                if (stream_lost) begin
                    // 短断流（音源缓冲欠载）由 SDRAM 直接吸收，
                    // 绝不启动重锁；只有持续断流达到 P_LOST_CONFIRM
                    // 才认定为真正的切歌/停播。H_FADE 一旦进入就
                    // 不可取消，所以必须在进入之前做持续确认。
                    if (lost_cnt >= P_LOST_CONFIRM) begin
                        switch_mute_r <= cur_valid;
                        resync_r      <= 1'b1;
                        hcnt          <= 16'd0;
                        hstate        <= H_FADE;
                        lost_cnt      <= 22'd0;
                    end else begin
                        lost_cnt <= lost_cnt + 22'd1;
                    end
                end else begin
                    lost_cnt <= 22'd0;
                end
                if (!stream_lost && det_all && !cur_valid) begin
                    // 首次锁定：测量已通过连续一致性确认，立即提交。
                    new_all        <= cand_all;
                    hcnt           <= 16'd0;
                    switch_mute_r <= 1'b0;
                    hstate        <= H_FADE;
                    chg_cnt        <= 3'd0;
                end else if (!stream_lost && det_all && (cand_all != cur_all)) begin
                    // 速率/格式变化：必须由连续 P_CHG_CONFIRM 次
                    // 「有效测量」都确认偏离才提交。单次毛刺造成的
                    // 档位瞬时偏离不会触发 186 ms 的随机静音。
                    if (chg_cnt >= P_CHG_CONFIRM) begin
                        new_all        <= cand_all;
                        hcnt           <= 16'd0;
                        // 先让发射器在旧时钟上淡出到静音，
                        // 再关断/复位任何部件，避免切换爆音。
                        switch_mute_r <= cur_valid;
                        hstate        <= H_FADE;
                        chg_cnt        <= 3'd0;
                    end else begin
                        chg_cnt <= chg_cnt + 3'd1;
                    end
                end else if (!stream_lost && det_all) begin
                    // 有效测量确认档位未变：清除累积，避免断续毛刺
                    // 被累加成一次虚假切换。
                    chg_cnt <= 3'd0;
                end
            end
            H_FADE: begin              // 24000 × 49.152 MHz ≈ 488 µs：足够覆盖
                                       // 最慢档（DSD64 / 44.1k 族）16 帧淡出的约 363 µs，
                                       // 保证进入复位前增益已真正降到 0，
                                       // 不会从还有音量的码流硬切到静音码流。
                if (hcnt >= 16'd24000) begin
                    switch_mute_r <= 1'b0;
                    hcnt          <= 16'd0;
                    // 在复位期开始之前提交新格式/速率，让 clk_core 域
                    // 的同步器有足够时间稳定，复位释放后不会选错 FIFO。
                    if (!resync_r) begin
                        cur_fmt    <= new_all[3];
                        cur_rate   <= new_all[2:0];
                        cur_family <= new_all[0];
                        cur_valid  <= 1'b1;
                    end
                    if (resync_r) begin
                        resync_r <= 1'b0;
                        hstate   <= H_RESYNC;  // 输出已静音，可安全复位
                    end else if (!cur_valid || (new_all[0] != cur_family))
                        hstate <= H_OFF;       // 晶振族变化：切换时钟 MUX
                    else
                        hstate <= H_RST;       // 同族内的速率/格式变化
                end else hcnt <= hcnt + 16'd1;
            end
            H_RESYNC: begin           // 等待干净且重新测量过的输入流
                g_arst_n_r   <= 1'b0;    // 接收器/FIFO/SDRAM 数据通路保持复位
                switch_mute_r <= 1'b0;
                if (!stream_lost && det_all) begin
                    new_all    <= cand_all;   // 重新测得的速率/格式
                    cur_fmt    <= cand_all[3];
                    cur_rate   <= cand_all[2:0];
                    cur_family <= cand_all[0];
                    cur_valid  <= 1'b1;
                    hcnt       <= 16'd0;
                    // 同族重锁不需要切换晶振：不关断 clk_core，
                    // 输出级在整个 H_RST 复位期仍持续送出 50% 密度
                    // 静音码流与连续 DCLK，换曲全程无静态电平。
                    // 仅当晶振族真的变化时才走关时钟的 MUX 流程。
                    if (cand_all[0] != cur_family) hstate <= H_OFF;
                    else                           hstate <= H_RST;
                end
            end
            H_OFF: begin                  // 关断全局时钟
                clk_ena_r  <= 1'b0;
                g_arst_n_r <= 1'b0;
                if (hcnt >= 16'd15) begin hstate <= H_SEL; hcnt <= 0; end
                else hcnt <= hcnt + 16'd1;
            end
            H_SEL: begin                  // 在关断期间改变 MUX 选择
                clk_sel_r <= new_all[0];
                if (hcnt >= 16'd15) begin hstate <= H_ON; hcnt <= 0; end
                else hcnt <= hcnt + 16'd1;
            end
            H_ON: begin                   // 重新使能纯净全局时钟
                clk_ena_r <= 1'b1;
                if (hcnt >= 16'd15) begin
                    cur_fmt    <= new_all[3];
                    cur_rate   <= new_all[2:0];
                    cur_family <= new_all[0];
                    cur_valid  <= 1'b1;
                    hstate     <= H_RST;
                    hcnt       <= 16'd0;
                end else hcnt <= hcnt + 16'd1;
            end
            H_RST: begin                  // 复位 / 重新初始化所有数据通路
                g_arst_n_r <= 1'b0;
                // rst_bridge 是异步置位、同步释放：若 clk_core 停着，
                // core_rst_n 永远无法释放，整个数据通路被锁在复位里；
                // 输出级也无法持续送出 DSD 静音码流。同族重锁会直接
                // 从 H_RESYNC 跳到这里，必须在此显式恢复时钟。
                clk_ena_r  <= 1'b1;
                if (hcnt >= 16'd255) begin
                    g_arst_n_r <= 1'b1;
                    hstate     <= H_IDLE;
                    hcnt       <= 16'd0;
                end else begin
                    hcnt <= hcnt + 16'd1;
                end
            end
            default: hstate <= H_IDLE;
        endcase
    end
end

//==============================================================================
// 2. 硬件全局时钟 MUX（专用时钟控制块，低抖动）
//==============================================================================
wire clk_core;
altclkctrl u_clkctrl (
    .inclk    ({clk_49m, clk_45m}),   // [0]=45.1584 MHz，[1]=49.152 MHz
    .clkselect(clk_sel_r),
    .ena      (clk_ena_r),
    .outclk   (clk_core)
);
defparam u_clkctrl.clock_type = "AUTO",
         u_clkctrl.ena_register_mode = "FALLING EDGE",
         u_clkctrl.implement_in_les  = "OFF",
         u_clkctrl.number_of_clocks  = 2,
         u_clkctrl.use_glitch_free_switch_over_implementation = "OFF",
         u_clkctrl.width_clkselect   = 1;

// 各时钟域复位桥（异步置位、同步释放）
wire core_rst_n, wr_rst_n;
rst_bridge u_rst_core (.clk(clk_core),    .arst_n(g_arst_n_r), .rst_n(core_rst_n));
rst_bridge u_rst_wr   (.clk(i2s_bclk_i),  .arst_n(g_arst_n_r), .rst_n(wr_rst_n));

// 控制/状态同步到音频（core）时钟域
reg  [1:0] mode_s, fmt_s;
reg  [2:0] rate_s1, rate_s2;
reg  [5:0] slots_s1, slots_s2;
// DSD 模式下让速率 LED 慢闪（约 1.5 Hz），便于与同档 PCM 区分。
reg [24:0] blink_cnt;
initial blink_cnt = 25'd0;
always @(posedge clk_49m) blink_cnt <= blink_cnt + 25'd1;
wire dsd_blink_49 = P_DSD_BLINK ? blink_cnt[24] : 1'b1;
initial begin
    mode_s = 0; fmt_s = 0;
    rate_s1 = 0; rate_s2 = 0; slots_s1 = 6'd32; slots_s2 = 6'd32;
end
always @(posedge clk_core) begin
    mode_s   <= {mode_s[0], mode_r};
    fmt_s    <= {fmt_s[0], cur_fmt};
    rate_s1  <= cur_rate;
    rate_s2  <= rate_s1;
    slots_s1 <= rx_slots;
    slots_s2 <= slots_s1;
end
wire        mode_core = mode_s[1];
wire        dsd_core  = fmt_s[1];
// K3 是用户显式按下的输出模式开关，必须由按键状态单独决定，
// 不能被自动格式识别结果屏蔽：原写法 mode_core && !dsd_core 在
// 输入被判为 DSD（含任何误判/瞬态）时会让 K3 完全失效——LED 不
// 全亮、端口永久空闲，且用户无法从面板判断原因。
// DSD 输入下若用户仍按了 K3，端口照常使能，但数据强制为零
// （见下方 out_pair），因此只会静音，不会把 1-bit 码流当 PCM 输出。
wire        mode_eff = mode_core;
wire [1:0]  rate_sh   = rate_s2[2:1];        // 0,1,2,3 -> FS x1,x2,x4,x8
wire [15:0] frame_len = 16'd1024 >> rate_sh; // 每个 LRCK 帧的 core 时钟数
wire [4:0]  bclk_period = frame_len[10:6];   // = 16/N (16,8,4,2)
wire [4:0]  bclk_half   = bclk_period >> 1;  // = 8/N  (8,4,2,1)

//==============================================================================
// 3. I2S / DSD 接收器（输入 BCLK 域）+ 各自的异步（双时钟）FIFO
//    两个接收器始终并行工作，各自写入独立的双时钟 FIFO；
//    格式选择在 clk_core 域对读口做多路复用，避免在输入时钟域
//    引入未同步的格式选择信号。未被选中的 FIFO 只会写满后自停，
//    格式切换时由全局复位统一清空。
//==============================================================================
wire [63:0] rx_pair, dsd_pair;
wire        rx_valid, dsd_valid;
wire [5:0]  rx_slots;
wire        rx_empty_i2s, rx_full, dsd_empty, dsd_full;
wire [63:0] rx_rdata_i2s, dsd_rdata;
reg         rx_pop;
wire        rx_pop_i2s = rx_pop && !dsd_core;
wire        rx_pop_dsd = rx_pop &&  dsd_core;
wire [63:0] rx_rdata   = dsd_core ? dsd_rdata : rx_rdata_i2s;
wire        rx_empty   = dsd_core ? dsd_empty : rx_empty_i2s;

i2s_rx #(.P_DELAY(P_I2S_DELAY)) u_rx (
    .bclk(i2s_bclk_i), .rst_n(wr_rst_n),
    .lrck(i2s_lrck_i), .sdata(i2s_sdata_i),
    .pair(rx_pair), .pair_valid(rx_valid), .slots(rx_slots)
);

// DSD 接收：DCLK 上升沿锁存 L/R 各 1 bit，每 32 bit 组成一个
// 64 bit 帧对 {L[31:0], R[31:0]}，MSB 为最早到达的 bit。
dsd_rx u_dsd_rx (
    .bclk(i2s_bclk_i), .rst_n(wr_rst_n),
    .d_l(i2s_lrck_i), .d_r(i2s_sdata_i),
    .pair(dsd_pair), .pair_valid(dsd_valid)
);

// DoP 解码器：从 i2s_rx 的归一化帧对里提取 marker / DSD 位，
// 输出与 dsd_pair 同构的 64bit 帧对。dop_active_w 同时送到管理域
// 参与格式判定，以及在这里选择 u_dsd_fifo 的写口来源。
wire [63:0] dop_pair;
wire        dop_valid, dop_active_w;
dop_dec #(.CONFIRM(P_DOP_CONFIRM), .MISS(P_DOP_MISS)) u_dop_dec (
    .bclk(i2s_bclk_i), .rst_n(wr_rst_n),
    .pair(rx_pair), .pair_valid(rx_valid),
    .dop_pair(dop_pair), .dop_valid(dop_valid), .dop_active(dop_active_w)
);

// native DSD 与 DoP 共用同一个异步 FIFO：两者帧对格式相同，
// 输出侧只要 dsd_core=1 就按 32bit 组串行化，无需区分来源。
//
// DoP 帧对背压队列：dop_valid 是单拍脉冲，若此刻 u_dsd_fifo 写满，
// 该帧对本会永久丢失，DSD 码流相位错位 → 可听的破音。
// 解码帧对速率（88.2k / 176.4k 帧对每秒）与输出消费速率相等，
// 但 SDRAM 刷新+读写调度会让 buffer_manager 的取数突发停顿，
// DSD64 下 u_dsd_fifo 可能连续数帧写满，单槽不够。
// 用 8 项 BCLK 域同步小队列做 valid/ready 握手，满时排队不丢帧。
reg [63:0] dop_q [0:31];
reg [4:0]  dop_q_wp, dop_q_rp;
reg [5:0]  dop_q_cnt;
reg        dop_q_beat;      // 每个解码帧对只请求一次 FIFO 写入
initial begin
    dop_q_wp = 5'd0; dop_q_rp = 5'd0; dop_q_cnt = 6'd0; dop_q_beat = 1'b0;
end
wire       dop_q_push = dop_active_w && dop_valid;
// 关键：pop 必须按「帧对产生节拍」进行，而不是每个 BCLK 一次。
// 若用电平 (cnt!=0) 直接 pop，队列会以 BCLK 速率把 u_dsd_fifo
// 瞬间灌满，队列随即溢出、帧对丢失。按节拍写则与 native DSD 的
// dsd_valid 速率完全一致，读写两侧天然平衡。32 项足以吸收
// SDRAM 刷新/读写调度带来的突发积压。
wire       dop_q_pop  = dop_active_w && (dop_q_cnt != 6'd0) &&
                        !dsd_full && dop_q_beat;
wire [63:0] dsd_fifo_wdata = dop_active_w ? dop_q[dop_q_rp] : dsd_pair;
wire        dsd_fifo_winc  = dop_active_w ? dop_q_pop : dsd_valid;

always @(posedge i2s_bclk_i) begin
    // DoP 撤销（marker 消失 / 切回 PCM）时必须丢弃队列残留，
    // 否则下一次进入 DoP 会把上一个流的旧帧对灌进 FIFO，
    // 造成码流相位错位。检测期 dop_active=0 时持续清空也无害。
    if (!wr_rst_n || !dop_active_w) begin
        dop_q_wp   <= 5'd0;
        dop_q_rp   <= 5'd0;
        dop_q_cnt  <= 6'd0;
        dop_q_beat <= 1'b0;
    end else begin
        if (dop_valid)            dop_q_beat <= 1'b1;
        else if (dop_q_pop)       dop_q_beat <= 1'b0;
        if (dop_q_push && dop_q_cnt < 6'd32) begin
            dop_q[dop_q_wp] <= dop_pair;
            dop_q_wp        <= dop_q_wp + 5'd1;
        end
        if (dop_q_pop) dop_q_rp <= dop_q_rp + 5'd1;
        if (dop_q_push && dop_q_cnt < 6'd32 && !dop_q_pop)
            dop_q_cnt <= dop_q_cnt + 6'd1;
        else if (!dop_q_push && dop_q_pop)
            dop_q_cnt <= dop_q_cnt - 6'd1;
    end
end

async_fifo #(.DSIZE(64), .AW(P_FIFO_AW)) u_rx_fifo (
    .wclk(i2s_bclk_i), .wrst_n(wr_rst_n),
    .winc(rx_valid && !rx_full), .wdata(rx_pair), .wfull(rx_full),
    .rclk(clk_core), .rrst_n(core_rst_n),
    .rinc(rx_pop_i2s), .rdata(rx_rdata_i2s), .rempty(rx_empty_i2s)
);

// 该 FIFO 同时承接 native DSD 与 DoP 解码帧对。DoP 输入的 BCLK 是
// 输出 DCLK 的 4 倍，写侧突发更密；仿真把 P_FIFO_AW 覆盖成 4（16 项）
// 时会长时间写满、让 DoP 队列溢出并丢帧。这里给深度设 64 项下限，
// 硬件 P_FIFO_AW=8（256 项）不受影响，不额外占用存储资源。
localparam integer DSD_FIFO_AW = (P_FIFO_AW > 6) ? P_FIFO_AW : 6;
async_fifo #(.DSIZE(64), .AW(DSD_FIFO_AW)) u_dsd_fifo (
    .wclk(i2s_bclk_i), .wrst_n(wr_rst_n),
    .winc(dsd_fifo_winc && !dsd_full), .wdata(dsd_fifo_wdata), .wfull(dsd_full),
    .rclk(clk_core), .rrst_n(core_rst_n),
    .rinc(rx_pop_dsd), .rdata(dsd_rdata), .rempty(dsd_empty)
);

//==============================================================================
// 4. SDRAM 缓冲管理器 + W9825G6JH 控制器（core 时钟域）
//==============================================================================
wire        sd_wr_req, sd_rd_req, sd_wr_ack, sd_rd_valid, sd_busy, sd_init_done;
wire [23:0] sd_wr_addr, sd_rd_addr;
wire [15:0] sd_wr_data, sd_rd_data;
wire [15:0] sd_dq_out;
wire        sd_dq_oe;
wire [15:0] sd_dq_in = sdram_data;

reg  [63:0] wbuf, rbuf;
reg  [23:0] wr_ptr, rd_ptr;
reg  [3:0]  bstate;
reg         tx_push;
reg  [63:0] tx_wdata;
// 水位变化缓慢，将其寄存一拍可切断
// wr_ptr/rd_ptr → 24 位减法 → 比较器 → 增益 → sdata_r 的
// 长组合链，否则该链正好卡在 49.152 MHz 的时序极限上。
reg [23:0] fill_words;
initial fill_words = 24'd0;
always @(posedge clk_core)
    if (!core_rst_n) fill_words <= 24'd0;
    else             fill_words <= wr_ptr - rd_ptr;

// 运行时可写的水位阈值（默认取参数值；验证平台可
// 注入较小数值，无需等待数小时即可触发 slip 路径）。
reg  [23:0] slip_low_th, slip_high_th;
initial begin slip_low_th = P_SLIP_LOW; slip_high_th = P_SLIP_HIGH; end

reg         prefilled;   // 预填完成后读侧跟随输出时钟
reg         slip_req;    // 请求带淡入淡出的帧对齐 slip
reg         slip_ack;    // 发射器在静音帧确认该 slip
reg  [9:0]  slip_gap;    // 两次 slip 之间的冷却计数（按帧递减）
reg         drop_pend;   // 淡出期间丢弃一个输入帧

wire low_water  = (fill_words <  slip_low_th);
wire high_water = (fill_words >  slip_high_th);
wire mute_hold  = (fill_words <  P_GUARD);     // 硬欠流 → 纯净静音
// 寄存副本只供音频数据通路使用：把 24 位比较器
// 移出 sdata_r / do_l_r 的建立时间路径（Hi-end 时序余量）。
reg  mute_hold_r;
initial mute_hold_r = 1'b1;
always @(posedge clk_core)
    if (!core_rst_n) mute_hold_r <= 1'b1;
    else             mute_hold_r <= mute_hold;
wire wr_ok = !rx_empty && (fill_words < CAP_WORDS - 24'd8);
wire rd_ok = prefilled && !tx_full && !mute_hold;

localparam [3:0] B_IDLE = 4'd0, B_W0 = 4'd1, B_W1 = 4'd2, B_W2 = 4'd3,
                 B_W3 = 4'd4, B_R0 = 4'd5, B_R1 = 4'd6, B_R2 = 4'd7,
                 B_R3 = 4'd8, B_PUSH = 4'd9;

initial begin
    wbuf = 0; rbuf = 0; wr_ptr = 0; rd_ptr = 0; bstate = B_IDLE;
    tx_push = 0; tx_wdata = 0; rx_pop = 0;
    prefilled = 0; slip_req = 0; slip_ack = 0; slip_gap = 0; drop_pend = 0;
end

wire [1:0] wix = (bstate >= B_W0 && bstate <= B_W3) ? (bstate[1:0] - 2'd1) : 2'd0;
wire [1:0] rix = (bstate >= B_R0 && bstate <= B_R3) ? (bstate[1:0] - 2'd1) : 2'd0;
assign sd_wr_req  = (bstate >= B_W0) && (bstate <= B_W3);
assign sd_rd_req  = (bstate >= B_R0) && (bstate <= B_R3);
assign sd_wr_addr = wr_ptr + {22'd0, wix};
assign sd_rd_addr = rd_ptr + {22'd0, rix};

reg [15:0] sd_wr_data_r;
always @* begin
    case (wix)
        2'd0: sd_wr_data_r = wbuf[15:0];
        2'd1: sd_wr_data_r = wbuf[31:16];
        2'd2: sd_wr_data_r = wbuf[47:32];
        default: sd_wr_data_r = wbuf[63:48];
    endcase
end
assign sd_wr_data = sd_wr_data_r;

always @(posedge clk_core) begin
    if (!core_rst_n) begin
        wbuf <= 64'd0; rbuf <= 64'd0;
        wr_ptr <= 24'd0; rd_ptr <= 24'd0;
        bstate <= B_IDLE;
        rx_pop <= 1'b0; tx_push <= 1'b0; tx_wdata <= 64'd0;
        prefilled <= 1'b0; slip_req <= 1'b0; slip_gap <= 10'd0;
        drop_pend <= 1'b0;
    end else begin
        rx_pop  <= 1'b0;
        tx_push <= 1'b0;
        // 水位驱动的 slip 管理。slip 只在
        // 输出已淡出到静音时才提交，因此任何硬重复/丢弃样本
        // 都不会到达 DAC（长期漂移也不会出现“卡样”爆音）。
        if (!prefilled && (fill_words >= P_FILL_TARGET)) prefilled <= 1'b1;
        if (slip_ack) begin
            slip_req  <= 1'b0;
            slip_gap  <= P_SLIP_GAP;
            drop_pend <= high_water;
        end else if (slip_gap != 10'd0) begin
            // 按 LRCK 帧递减：一次 fade 需 33 帧，512 帧的冷却
            // 保证 slip 不会在前一次淡入完成前被重新请求。
            if (load_pair) slip_gap <= slip_gap - 10'd1;
        end else if ((low_water || high_water) && !mute_hold) begin
            slip_req  <= 1'b1;
        end
        case (bstate)
            B_IDLE: begin
                // 写操作优先：输入（异步）FIFO 很浅，
                // 而 SDRAM 环形缓冲很大。若让读抢先，会溢出
                // 输入 FIFO 并无声丢弃音频帧。
                if (drop_pend && !rx_empty) begin
                    rx_pop    <= 1'b1;          // 丢弃一个输入帧对
                    drop_pend <= 1'b0;
                end else if (wr_ok) begin
                    wbuf    <= rx_rdata;
                    rx_pop  <= 1'b1;
                    bstate  <= B_W0;
                end else if (rd_ok) begin
                    bstate <= B_R0;
                end
            end
            B_W0, B_W1, B_W2, B_W3: begin
                if (sd_wr_ack) begin
                    if (bstate == B_W3) begin
                        wr_ptr <= wr_ptr + 24'd4;
                        bstate <= B_IDLE;
                    end else begin
                        bstate <= bstate + 4'd1;
                    end
                end
            end
            B_R0, B_R1, B_R2, B_R3: begin
                if (sd_rd_valid) begin
                    case (bstate)
                        B_R0: rbuf[15:0]  <= sd_rd_data;
                        B_R1: rbuf[31:16] <= sd_rd_data;
                        B_R2: rbuf[47:32] <= sd_rd_data;
                        default: rbuf[63:48] <= sd_rd_data;
                    endcase
                    if (bstate == B_R3) bstate <= B_PUSH;
                    else                bstate <= bstate + 4'd1;
                end
            end
            B_PUSH: begin
                // 只有帧对确实被接收时才推进 rd_ptr：
                // 否则 tx FIFO 满时会无声丢弃音频帧。
                if (!tx_full) begin
                    tx_wdata <= rbuf;
                    tx_push  <= 1'b1;
                    rd_ptr   <= rd_ptr + 24'd4;
                    bstate   <= B_IDLE;
                end
            end
            default: bstate <= B_IDLE;
        endcase
    end
end

sdram_ctrl #(
    .INIT_WAIT(P_INIT_WAIT), .REF_INT(P_REF_INT)
) u_sdram (
    .clk(clk_core), .rst_n(core_rst_n),
    .wr_req(sd_wr_req),  .wr_addr(sd_wr_addr),  .wr_data(sd_wr_data),  .wr_ack(sd_wr_ack),
    .rd_req(sd_rd_req),  .rd_addr(sd_rd_addr),  .rd_data(sd_rd_data),  .rd_valid(sd_rd_valid),
    .busy(sd_busy), .init_done(sd_init_done),
    .cke(sdram_cke), .cs_n(sdram_cs_n), .ras_n(sdram_ras_n),
    .cas_n(sdram_cas_n), .we_n(sdram_we_n),
    .ba(sdram_ba), .addr(sdram_addr), .dqm(sdram_dqm),
    .dq_in(sd_dq_in), .dq_out(sd_dq_out), .dq_oe(sd_dq_oe)
);
assign sdram_clk  = ~clk_core;             // 为 SDRAM 提供半周期相位余量
assign sdram_data = sd_dq_oe ? sd_dq_out : 16'bz;

//==============================================================================
// 5. 输出（tx）同步 FIFO、I2S reclock 发射器与 AD1862 端口
//==============================================================================
wire [63:0] tx_rdata;
wire        tx_empty, tx_full;
reg         tx_pop;
sync_fifo #(.DSIZE(64), .AW(P_FIFO_AW)) u_tx_fifo (
    .clk(clk_core), .rst_n(core_rst_n),
    .winc(tx_push), .wdata(tx_wdata), .wfull(tx_full),
    .rinc(tx_pop),  .rdata(tx_rdata), .rempty(tx_empty)
);

//----- 发射器寄存器 -------------------------------------------------
reg  [63:0] cur_pair;
reg  [5:0]  slot;
reg  [4:0]  sub;
reg         bclk_r, lrck_r, sdata_r, mclk_r;
reg  [3:0]  mdiv;
reg  [15:0] ad_acc;
reg         ad_bclk_r, ad_le_r, do_l_r, do_r_r;
reg  [4:0]  ad_slot;
// DSD 原生输出：移位寄存器只用固定位选择（[63]/[31]），
// 不引入 64:1 变量译码，输出建立时间余量与 PCM 路径一致。
reg  [63:0] dsd_shift;      // {剩余 L[31:0], 剩余 R[31:0]}，MSB 先出
reg  [15:0] fade_lfsr;      // DSD 淡入淡出用的伪随机混合权重

// Hi-end 新增：16 级淡入淡出、帧边界模式提交、dither LFSR。
reg  [4:0]  gain;           // 0~16，16 表示单位增益
reg  [1:0]  fstate;
reg         mode_out;       // AD1862 端口只在静音帧切换
reg  [1:0]  sm_s, mode_d;
reg         sm_d, mute_d;
reg  [2:0]  rate_d;
reg         fade_pend, slip_pend;
// 本次淡变是否仅由水位 slip 引起。slip 只补/丢 1 个帧对，前后码流
// 连续，不需要 33 帧的完整淡入淡出；用 4 级步长压到约 10 帧（352.8 kHz
// 档约 28 µs）即可，且仍保留平滑过渡。速率/格式/模式切换时码流完全
// 改变，仍走原来的 ±1、33 帧完整淡变。
reg         slip_only;
wire [4:0]  fade_step = slip_only ? 5'd4 : 5'd1;
wire [5:0]  gain_sub  = {1'b0, gain} - {1'b0, fade_step};  // 6 位避免下溢
wire [5:0]  gain_add  = {1'b0, gain} + {1'b0, fade_step};
reg  [15:0] dlfsr_l, dlfsr_r;
localparam [1:0] F_IDLE = 2'd0, F_OUT = 2'd1, F_HOLD = 2'd2, F_IN = 2'd3;

initial begin
    cur_pair = 0; slot = 0; sub = 0;
    bclk_r = 0; lrck_r = 0; sdata_r = 0; mclk_r = 0; mdiv = 0;
    ad_acc = 0; ad_bclk_r = 0; ad_le_r = 1'b1;
    do_l_r = 0; do_r_r = 0; ad_slot = 0; tx_pop = 0;
    gain = 0; fstate = F_IN; mode_out = 0;
    sm_s = 0; mode_d = 0; sm_d = 0; mute_d = 0; rate_d = 0;
    fade_pend = 0; slip_pend = 0; slip_ack = 0; slip_only = 0;
    dlfsr_l = 16'hACE1; dlfsr_r = 16'h5A3C;
    dsd_shift = 64'd0; fade_lfsr = 16'h51D3;
end

//--- 变化检测：任何切换 / 模式变化 / slip 都请求一次淡入淡出 ---
always @(posedge clk_core) begin
    sm_s   <= {sm_s[0], switch_mute_r};
    sm_d   <= sm_s[1];
    mode_d <= {mode_d[0], mode_eff};
    rate_d <= rate_s2;
    mute_d <= mute_hold;
end
wire sm_rise     = sm_s[1] && !sm_d;
// 模式请求必须是「电平锁存」而不是单拍脉冲：mode_chg 原本只有
// 1 个 core 时钟宽，一旦落在 fade 期间就会被丢弃；而数据通路复位
// 会把 mode_out 清 0、mode_eff 却不变，此后没有任何新的边沿可再
// 触发提交 —— LED 仍全亮，AD1862 端口却永久空闲（K3 看似失效）。
// 只要已提交的 mode_out 与 mode_eff 不一致就持续请求一次淡变，
// 直到 F_HOLD 重新提交成功为止，任何复位/竞态都能自恢复。
wire mode_chg    = (mode_d[1] != mode_d[0]) || (mode_out != mode_eff);
wire rate_chg    = (rate_d != rate_s2);
wire mute_release = mute_d && !mute_hold;
wire silent_fr   = mute_hold_r || (slip_pend && (fstate == F_HOLD));

//--- DSD 原生输出：1-bit 码流 + 随机化淡入淡出 -----------------
// DSD 没有 PCM 意义上的音量，切换/淡入淡出改为在真实 bit 与
// 50% 密度的 DSD 静音码型之间做伪随机概率混合：gain=0 全静音
// 码型、gain=16 全真实码流，中间逐帧平滑过渡，不会产生直流台阶。
wire [7:0]  sil_pat  = DSD_SIL_PAT;
wire [2:0]  sil_idx  = slot[2:0];
wire        sil_l    = sil_pat[sil_idx];
wire        sil_r    = sil_pat[(sil_idx + 3'd4) & 3'd7];
wire [3:0]  fade_rnd = fade_lfsr[3:0];
wire        dsd_real = ({1'b0, fade_rnd} < gain);
wire        dsd_l_bit = dsd_shift[63];
wire        dsd_r_bit = dsd_shift[31];
wire        dsd_boundary = (slot[4:0] == 5'd31);
wire        dsd_tick  = (sub == bclk_half - 5'd1);
// 每 32 个 DCLK 取一个 64bit 帧对；F_HOLD 静音帧故意不取，
// 即在此处插入一个 32bit 的静音组（水位 slip 的 DSD 等价形式）。
wire        dsd_load  = dsd_core && dsd_tick && dsd_boundary &&
                        !tx_empty && !silent_fr;
wire [15:0] fade_nxt = {fade_lfsr[14:0],
                        fade_lfsr[15]^fade_lfsr[13]^fade_lfsr[12]^fade_lfsr[10]};

//--- 增益施加于 32 位 MSB 对齐样本；单位增益 = 16/16 ---------
wire signed [31:0] s_l = $signed(cur_pair[63:32]);
wire signed [31:0] s_r = $signed(cur_pair[31:0]);
wire signed [36:0] pg_l = s_l * $signed({1'b0, gain});
wire signed [36:0] pg_r = s_r * $signed({1'b0, gain});
wire signed [36:0] pg_l_sh = pg_l >>> 4;
wire signed [36:0] pg_r_sh = pg_r >>> 4;
wire [31:0] g_l = pg_l_sh[31:0];
wire [31:0] g_r = pg_r_sh[31:0];
// DSD 输入时 cur_pair 不是有效 PCM 帧（发射器不消费 tx FIFO），
// 若用户此时按了 K3，AD1862 端口只能输出静音，避免把 1-bit
// 码流/残留 PCM 帧锁存成噪声或直流。
wire [63:0] out_pair = dsd_core ? 64'd0 : {g_l, g_r};

//--- 20 位 AD1862 路径的 TPDF 抖动 + 舍入 + 饱和 ----------
// 两路独立 16 位 LFSR；低半字节相减得到三角分布（-15~15）。
wire [15:0] nlf_l = {dlfsr_l[14:0], dlfsr_l[15]^dlfsr_l[13]^dlfsr_l[12]^dlfsr_l[10]};
wire [15:0] nlf_r = {dlfsr_r[14:0], dlfsr_r[15]^dlfsr_r[12]^dlfsr_r[9]^dlfsr_r[7]};
wire signed [5:0] tpdf_l = $signed({1'b0, dlfsr_l[3:0]}) - $signed({1'b0, dlfsr_l[7:4]});
wire signed [5:0] tpdf_r = $signed({1'b0, dlfsr_r[3:0]}) - $signed({1'b0, dlfsr_r[7:4]});
// 标准 TPDF 幅度：1 LSB 峰峰值（±0.5 LSB）。叠加
// +0.5 LSB 的舍入偏置后，20 位结果与直接
// 截断相差最多 +1 LSB。
wire signed [31:0] dith_l = {{26{tpdf_l[5]}}, tpdf_l} <<< 7;
wire signed [31:0] dith_r = {{26{tpdf_r[5]}}, tpdf_r} <<< 7;

wire signed [32:0] ext_l  = {out_pair[63], out_pair[63:32]};
wire signed [32:0] ext_r  = {out_pair[31], out_pair[31:0]};
wire signed [32:0] sum_l  = ext_l + 33'sd2048 + (P_DITHER_EN ? {dith_l[31], dith_l} : 33'sd0);
wire signed [32:0] sum_r  = ext_r + 33'sd2048 + (P_DITHER_EN ? {dith_r[31], dith_r} : 33'sd0);
// 在 >>>12 之前先钳位到完整 32 位有符号范围；若按
// 20 位满量程钳位，接近满量程的样本会被截掉 15 LSB。
wire signed [32:0] clip_l = (sum_l >  33'sd2147483647) ?  33'sd2147483647 :
                            (sum_l < -33'sd2147483648) ? -33'sd2147483648 : sum_l;
wire signed [32:0] clip_r = (sum_r >  33'sd2147483647) ?  33'sd2147483647 :
                            (sum_r < -33'sd2147483648) ? -33'sd2147483648 : sum_r;
wire signed [32:0] clip_l_sh = clip_l >>> 12;
wire signed [32:0] clip_r_sh = clip_r >>> 12;
wire [19:0] left20  = clip_l_sh[19:0];
wire [19:0] right20 = clip_r_sh[19:0];

//--- 从下一帧（经增益处理）选择 I2S 串行位 --------------------
wire [5:0] bits_core = (slots_s2 == 6'd0 || slots_s2 > 6'd32) ? 6'd32 : slots_s2;
// 用固定 6 位表达式，避免 QSF 把参数覆盖成 32 位整数时产生位宽截断。
wire [5:0] tx_bits   = bits_core - (P_I2S_DELAY ? 6'd1 : 6'd0);
wire [5:0] next_slot = (slot == 6'd63) ? 6'd0 : (slot + 6'd1);
wire       load_pair = (sub == bclk_half - 5'd1) && (slot == 6'd63);
// 直通模式下不对 I2S 帧流做增益/静音帧门控，与 Hi-end 优化前完全一致。
wire       load_eff = load_pair && !dsd_core && !tx_empty && (P_I2S_LEGACY || !silent_fr);
wire [63:0] nxt_raw = load_eff ? tx_rdata : cur_pair;
wire signed [36:0] ng_l = $signed(nxt_raw[63:32]) * $signed({1'b0, gain});
wire signed [36:0] ng_r = $signed(nxt_raw[31:0])  * $signed({1'b0, gain});
wire signed [36:0] ng_l_sh = ng_l >>> 4;
wire signed [36:0] ng_r_sh = ng_r >>> 4;
wire [31:0] pn_l = ng_l_sh[31:0];
wire [31:0] pn_r = ng_r_sh[31:0];
wire [63:0] pair_next = P_I2S_LEGACY ? nxt_raw : {pn_l, pn_r};
wire [4:0] k_slot = next_slot[4:0];
wire [5:0] eff_k  = P_I2S_DELAY ? ((k_slot == 5'd0) ? 6'd63 : {1'b0, k_slot} - 6'd1)
                                 : {1'b0, k_slot};
// 左右声道映射可配置：AK4493 的 justified 模式声道定义与 I2S 相反，
// 板级 L/R 接线也可能对调，故用参数而非硬编码。
wire [31:0] ch_l = P_SWAP_LR ? pair_next[31:0]  : pair_next[63:32];
wire [31:0] ch_r = P_SWAP_LR ? pair_next[63:32] : pair_next[31:0];
wire [31:0] ch_r_out = P_CH_TEST ? 32'd0 : ch_r;   // 声道诊断：右声道静音
wire [31:0] ch_word = (next_slot < 6'd32) ? ch_l : ch_r_out;
wire        next_sdata = (eff_k < tx_bits) ? ch_word[31 - eff_k[4:0]] : 1'b0;

wire        lrck_norm = (next_slot >= 6'd32) ? 1'b1 : 1'b0;
wire        lrck_val  = P_LRCK_POL ? ~lrck_norm : lrck_norm;

wire [3:0] sh = 4'd4 - {2'd0, rate_sh};
wire [15:0] cyc_comb = ({10'd0, slot} << sh) + {11'd0, sub};
wire        frame_start = (cyc_comb == 16'd0);

// AD1862 分数 NCO：每个 LRCK 帧恰好 24 个 CLK 上升沿，
// 每相位 ≥ 2 个 core 时钟（CLK 高/低 > 15 ns，数据建立/保持 > 25 ns）。
wire [15:0] ad_next = ad_acc + 16'd48;
wire        ad_tog  = (ad_next >= frame_len);
wire        ad_rise = ad_tog && !ad_bclk_r;
wire        ad_fall = ad_tog &&  ad_bclk_r;

always @(posedge clk_core) begin
    if (!core_rst_n) begin
        // 输出时钟/数据寄存器不在此清零：由下方独立的输出级统一驱动。
        // 1-bit DSD 的恒定 0/1 等于 ±满幅直流，若复位期间把
        // DSD_L/DSD_R 固定为 0，DAC 低通会把换曲瞬间的电平跳变
        // 积分成满幅冲击（爆音）。
        cur_pair <= 64'd0; tx_pop <= 1'b0;
        ad_acc <= 16'd0; ad_bclk_r <= 1'b0; ad_le_r <= 1'b1;
        do_l_r <= 1'b0; do_r_r <= 1'b0; ad_slot <= 5'd0;
        gain <= 5'd0; fstate <= F_IN; mode_out <= 1'b0;
        fade_pend <= 1'b0; slip_pend <= 1'b0; slip_ack <= 1'b0;
        slip_only <= 1'b0;
        dlfsr_l <= 16'hACE1; dlfsr_r <= 16'h5A3C;
        dsd_shift <= 64'd0; fade_lfsr <= 16'h51D3;
    end else begin
        tx_pop   <= 1'b0;
        slip_ack <= 1'b0;

        //---- Hi-end 淡入淡出 / slip 控制，仅在帧边界推进 ---
        // 切换（速率/格式/模式）时码流完全改变，用 ±1 的 33 帧完整
        // 淡变；水位 slip 只补/丢 1 个帧对、前后码流连续，改用步长 4
        // 的约 10 帧快速淡变。否则在 slip 稀疏（按 512 帧冷却）之后，
        // 晶振 1~2 ppm 偏差下每几秒一次的孤立 33 帧静音会被听成
        // 「停顿一下」（晶振偏差大时则退化为连续 fade 循环的嘶声）。
        // 新请求只在稳态被接受，一次淡变过程中不会被重复触发。
        if (fstate == F_IDLE) begin
            if (sm_rise || mode_chg || rate_chg) begin
                fade_pend <= 1'b1;
                slip_only <= 1'b0;          // 切换优先使用完整淡变
            end
            if (slip_req) begin
                fade_pend <= 1'b1;
                slip_pend <= 1'b1;
                if (!(sm_rise || mode_chg || rate_chg)) slip_only <= 1'b1;
            end
        end
        // 只在稳态发起新的淡入，不打断
        // 正在进行的淡入淡出（播放源断流期间的水位抖动
        // 不能锁死增益/淡入淡出状态机）。
        if (mute_release && fstate == F_IDLE) begin fstate <= F_IN; gain <= 5'd0; end
        if (load_pair) begin
            dlfsr_l <= nlf_l;
            dlfsr_r <= nlf_r;
            case (fstate)
                F_IDLE: if (fade_pend || slip_pend) begin
                            fstate    <= F_OUT;
                            gain      <= 5'd16;
                            fade_pend <= 1'b0;
                        end
                F_OUT:  begin
                            if (gain != 5'd0)
                                gain <= (gain >= fade_step) ? gain_sub[4:0] : 5'd0;
                            if (gain <= fade_step) fstate <= F_HOLD;
                        end
                F_HOLD: begin
                            mode_out <= mode_eff;      // 端口在静音期间切换（DSD 时保持空闲）
                            if (slip_pend) begin
                                slip_pend <= 1'b0;
                                slip_ack  <= 1'b1;     // 通知缓冲管理器
                            end
                            fstate <= F_IN;
                        end
                F_IN:   begin
                            if (gain < 5'd16)
                                gain <= (gain_add >= 6'd16) ? 5'd16 : gain_add[4:0];
                            else fstate <= F_IDLE;
                        end
            endcase
        end
        if (mute_hold) gain <= 5'd0;

        if (load_eff || dsd_load) begin
            if (!dsd_core) cur_pair <= tx_rdata;  // 每帧一个立体声帧对
            tx_pop <= 1'b1;
        end

        //--- AD1862 20 位串行端口（数据手册 Fig.12a / Fig.12b）
        if (frame_start) begin
            ad_acc    <= 16'd24;
            ad_bclk_r <= 1'b0;                   // 帧边界处抑制一次翻转
            ad_slot   <= 5'd0;
            ad_le_r   <= 1'b1;                   // LE 无效（高电平）
            do_l_r    <= left20[19];             // 高位先出
            do_r_r    <= right20[19];
        end else begin
            ad_acc <= ad_tog ? (ad_next - frame_len) : ad_next;
            if (ad_tog) ad_bclk_r <= ~ad_bclk_r;
            if (ad_rise) ad_slot <= (ad_slot == 5'd23) ? 5'd0 : (ad_slot + 5'd1);
            if (ad_fall) begin
                // ad_slot 指向下一个上升沿
                if (ad_slot < 5'd20) begin
                    do_l_r <= left20[19 - ad_slot];
                    do_r_r <= right20[19 - ad_slot];
                end else begin
                    do_l_r <= 1'b0;
                    do_r_r <= 1'b0;
                end
                ad_le_r <= (ad_slot >= 5'd20) ? 1'b0 : 1'b1;
            end
        end
    end

    //==========================================================================
    // 输出级：与数据通路复位解耦，始终在纯净全局时钟上寄存。
    // PCM 复位沿用原行为（输出与帧计数器清零，全 0 就是数字静音）；
    // DSD 复位/重锁期间则必须继续输出 50% 密度静音码型与连续的
    // DCLK/MCLK —— 1-bit DAC 把恒定 0/1 解释为 ±满幅直流，
    // 一旦输出被冻结，换曲起止处就会产生满幅阶跃（爆音）。
    // 淡出已完成（gain=0，输出本就是同一串静音码型），因此
    // 进入/退出复位时相位连续，没有任何电平台阶。
    //==========================================================================
    if (!core_rst_n && !dsd_core) begin
        sub    <= 5'd0;
        slot   <= 6'd0;
        bclk_r <= 1'b0; lrck_r <= 1'b0; sdata_r <= 1'b0;
        mclk_r <= 1'b0; mdiv <= 4'd0;
    end else begin
        //--- 主时钟：256FS（×1）、256FS（×2）、128FS（×4）、64FS（×8）
        if (mdiv >= ((rate_sh == 2'd0) ? 4'd1 : 4'd0)) begin
            mdiv   <= 4'd0;
            mclk_r <= ~mclk_r;
        end else begin
            mdiv <= mdiv + 4'd1;
        end

        // 用 >= 而非 ==：速率切换后 sub 可能已越过新的 bclk_period，
        // 立即回绕可避免一长段没有 DCLK 边沿的输出停摆。
        if (sub >= bclk_period - 5'd1) begin
            sub    <= 5'd0;
            bclk_r <= 1'b1;                       // BCLK / DCLK 上升沿
            slot   <= next_slot;
        end else begin
            sub <= sub + 5'd1;
            if (dsd_tick) begin
                bclk_r <= 1'b0;                   // BCLK / DCLK 下降沿
                if (dsd_core) begin
                    lrck_r  <= (core_rst_n && dsd_real) ? dsd_l_bit : sil_l;  // DSD_L
                    sdata_r <= (core_rst_n && dsd_real) ? dsd_r_bit : sil_r;  // DSD_R
                    if (core_rst_n) begin
                        fade_lfsr <= fade_nxt;
                        if (dsd_boundary) begin
                            if (dsd_load) dsd_shift <= tx_rdata;  // 取下一个 32bit 组
                            // 静音帧不取、也不移位：插入一个静音组
                        end else begin
                            dsd_shift <= {dsd_shift[62:32], 1'b0,
                                          dsd_shift[30:0], 1'b0};
                        end
                    end
                end else begin
                    lrck_r  <= lrck_val;
                    sdata_r <= next_sdata;
                end
            end
        end
    end
end

// DSD 模式下这四根排针复用为 MCLK / DCLK / DSD_L / DSD_R。
assign i2s_mclk_o  = mclk_r;
assign i2s_bclk_o  = bclk_r;
assign i2s_lrck_o  = lrck_r;
assign i2s_sdata_o = sdata_r;
assign ad_bclk     = mode_out ?  ad_bclk_r : 1'b0;
assign ad_le       = mode_out ?  ad_le_r   : 1'b1;
assign do_l        = mode_out ?  do_l_r    : 1'b0;
assign do_r        = mode_out ?  do_r_r    : 1'b0;

// [R4] LED0：44.1/48k 或 DSD64，LED1：88.2/96k 或 DSD128，
//      LED2：176.4/192k 或 DSD256，LED3：352.8/384k 或 DSD512。
// [R5] AD1862 模式有效时四颗 LED 全亮。DSD 模式下该档位 LED 慢闪，
//      以便与同档位的 PCM 播放区分（P_DSD_BLINK=0 可关闭）。
//------------------------------------------------------------------------------
// LED 状态指示改由常开 clk_49m 域直接寄存，不再依赖被监测的 clk_core。
// 原实现用 rate_s2 / mode_eff / dsd_core（都在 clk_core 域）驱动 LED；
// 一旦 ALTCLKCTRL 输出因任何原因停摆，LED 就永远冻结在上电初值
// 0001（LED0）：既看不到真实速率，按 K3 也毫无反馈，无法区分
// 「管理域没跑」与「core 时钟停」。cur_rate / cur_fmt / cur_valid /
// mode_r 本来就在管理域，直接编码即可，让面板成为可靠的诊断窗口：
//   全亮 = K3 按键事件已生效；
//   LED3 = 352.8 kHz 等 ×8 档已锁定；LED0 = 尚未锁定速率；
//   DSD 模式下档位灯慢闪（P_DSD_BLINK=0 时常亮）。
// 注意：LED 与数据通路状态解耦，只影响指示，不改变任何音频行为。
//------------------------------------------------------------------------------
reg  [3:0] led_r;
initial led_r = 4'b0001;
wire [3:0] rate_led_49 = 4'b0001 << cur_rate[2:1];
always @(posedge clk_49m) begin
    if (mode_r)                         led_r <= 4'b1111;
    else if (!cur_valid)                led_r <= 4'b0001;
    else if (cur_fmt && !dsd_blink_49)  led_r <= 4'b0000;
    else                                led_r <= rate_led_49;
end
assign led = led_r;

//==============================================================================
// DSDOE / F0~F3 采样率指示（定义见工程根目录 f0~F3.txt）
// --------------------------------------------------------------------------
// 引脚：dsd_on = PIN_F2（DSDOE）；fsel[3:0] 的 F0/F1/F2/F3 分别在
//       PIN_C2 / PIN_A2 / PIN_A3 / PIN_A4。
//       注意物理 PIN_F2 与信号 fsel[2]（在 PIN_A3）是两个不同引脚。
//
// DSDOE=0（PCM）：0000=32k, 0001=44.1k, 0010=48k, 0011=88.2k,
//                 0100=96k, 0101=176.4k, 0110=192k, 0111=352.8k,
//                 1000=384k, 1001=705.6k, 1010=768k, 1011=1411.2k,
//                 1100=1536k
//   本工程 8 档速率 code 0..7 恰好对应 0001..1000（= code + 1）；
//   705.6k 以上的 1001~1100 档本工程不支持。
//   f0~F3.txt 第 16 行把 1411.2kHz 误写成 1001（与 705.6kHz 重复），
//   按二进制递增应为 1011，此处以递增规律为准。
//
// DSDOE=1（DSD）：1001=DSD64, 1010=DSD128, 1011=DSD256,
//                 1100=DSD512, 1101=DSD1024（本工程不支持 DSD1024）
//   DSD 档位 = cur_rate[2:1]，即 1001 + 档位；不区分 44.1k/48k 族，
//   与文件定义一致。
//
// 与 LED 一样放在常开 clk_49m 域寄存：DSDOE/F0~F3 是给 DAC 的静态
// 配置电平，不应依赖被监测的 clk_core；且在格式切换时提前于输出侧
// 一点拉高 DSDOE 更安全（避免 DAC 用 PCM 模式去解 DSD 码流）。
//==============================================================================
reg        dsd_on_r;
reg  [3:0] fsel_r;
initial begin dsd_on_r = 1'b0; fsel_r = 4'b0000; end
always @(posedge clk_49m) begin
    dsd_on_r <= cur_fmt && cur_valid;
    if (!cur_valid) begin
        fsel_r <= 4'b0000;                       // 未锁定：32kHz / 无效
    end else if (cur_fmt) begin
        case (cur_rate[2:1])
            2'd0:    fsel_r <= 4'b1001;          // DSD64
            2'd1:    fsel_r <= 4'b1010;          // DSD128
            2'd2:    fsel_r <= 4'b1011;          // DSD256
            default: fsel_r <= 4'b1100;          // DSD512
        endcase
    end else begin
        case (cur_rate)
            3'd0:    fsel_r <= 4'b0001;          //  44.1 kHz
            3'd1:    fsel_r <= 4'b0010;          //  48.0 kHz
            3'd2:    fsel_r <= 4'b0011;          //  88.2 kHz
            3'd3:    fsel_r <= 4'b0100;          //  96.0 kHz
            3'd4:    fsel_r <= 4'b0101;          // 176.4 kHz
            3'd5:    fsel_r <= 4'b0110;          // 192.0 kHz
            3'd6:    fsel_r <= 4'b0111;          // 352.8 kHz
            default: fsel_r <= 4'b1000;          // 384.0 kHz
        endcase
    end
end
assign dsd_on = dsd_on_r;
assign fsel   = fsel_r;

endmodule

//==============================================================================
// 同名封装模块：允许按文件名实例化本文件。
//==============================================================================
module Cy4_I2S_reclock (
    input  wire        clk_45m, input  wire        clk_49m,
    input  wire        i2s_bclk_i, input  wire i2s_lrck_i, input wire i2s_sdata_i,
    output wire        i2s_mclk_o, output wire i2s_bclk_o,
    output wire        i2s_lrck_o, output wire i2s_sdata_o,
    output wire        do_l, output wire do_r,
    output wire        ad_bclk, output wire ad_le,
    output wire [3:0]  led,
    output wire        dsd_on, output wire [3:0] fsel,
    input  wire        key_k3,
    output wire        sdram_clk, output wire sdram_cke,
    output wire        sdram_cs_n, output wire sdram_ras_n,
    output wire        sdram_cas_n, output wire sdram_we_n,
    output wire [1:0]  sdram_ba, output wire [12:0] sdram_addr,
    inout  wire [15:0] sdram_data, output wire [1:0] sdram_dqm
);
Cy4_I2Sq u_top (
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
endmodule

//==============================================================================
// 复位桥：异步置位，同步释放。
//==============================================================================
module rst_bridge (
    input  wire clk,
    input  wire arst_n,
    output reg  rst_n
);
reg q;
initial begin q = 1'b0; rst_n = 1'b0; end
always @(posedge clk or negedge arst_n) begin
    if (!arst_n) begin q <= 1'b0; rst_n <= 1'b0; end
    else         begin q <= 1'b1; rst_n <= q;    end
end
endmodule

//==============================================================================
//==============================================================================
// DoP (DSD over PCM) v1.1 解码器，工作在输入 BCLK 域。
// ---------------------------------------------------------------------------
// 输入是 i2s_rx 归一化后的 64bit 帧对 {L[31:0], R[31:0]}（MSB 对齐）。
// 因此 24-bit DoP 帧 / 32-bit 容器高 24 位的布局为：
//     word[31:24] = marker，word[23:8] = 16 个 DSD 位   → 布局 A
// 32-bit 容器低 24 位（右对齐）则为：
//     word[23:16] = marker，word[15:0] = 16 个 DSD 位   → 布局 B
// 两种都支持，布局在首次识别成功时锁存；同一帧 L/R 必须带相同 marker，
// 相邻帧 marker 必须在 0x05 / 0xFA 间交替（起始相位自适应）。
//
// 每 2 个 PCM 帧拼成 1 个 64bit DSD 帧对 {L 32bit, R 32bit}，
// 与 dsd_rx 输出完全同构：早期 16 位放高半区，MSB 先输出。
// 输出侧每 32 个 DCLK 消费一个帧对，正好等于 2 个 PCM 帧
// （DSD64：176.4k×16 = 2.8224 MHz），读写速率天然匹配，
// SDRAM 环 / tx FIFO / DSD 串行化无需任何改动。
//
// 注意 dop_active 不受数据通路复位 wr_rst_n 清零：格式切换时
// H_RST 会清空两个 FIFO，若这里也把 DoP 状态清掉，就要重新等
// 32 帧才恢复供数，dsd_core 期间会输出乱码。
//==============================================================================
module dop_dec #(
    parameter [5:0] CONFIRM = 6'd32,
    parameter [2:0] MISS    = 3'd4
) (
    input  wire        bclk,
    input  wire        rst_n,
    input  wire [63:0] pair,
    input  wire        pair_valid,
    output reg  [63:0] dop_pair,
    output reg         dop_valid,
    output reg         dop_active
);
localparam [7:0] MK_A = 8'h05, MK_B = 8'hFA;

reg        phase, phase_set;
reg [5:0]  ok_cnt;
reg [2:0]  miss_cnt;
reg        layout;              // 0=A(marker 在 [31:24])，1=B([23:16])
reg [15:0] l_hi, r_hi;
reg        half;

initial begin
    phase = 1'b0; phase_set = 1'b0; ok_cnt = 6'd0; miss_cnt = 3'd0;
    layout = 1'b0; l_hi = 16'd0; r_hi = 16'd0; half = 1'b0;
    dop_pair = 64'd0; dop_valid = 1'b0; dop_active = 1'b0;
end

wire [7:0] mk_a_l = pair[63:56], mk_a_r = pair[31:24];
wire [7:0] mk_b_l = pair[55:48], mk_b_r = pair[23:16];
wire       valid_a = (mk_a_l == mk_a_r) && ((mk_a_l == MK_A) || (mk_a_l == MK_B));
wire       valid_b = (mk_b_l == mk_b_r) && ((mk_b_l == MK_A) || (mk_b_l == MK_B));
wire       mk_ok   = valid_a || valid_b;
wire [7:0] mk      = valid_a ? mk_a_l : mk_b_l;
wire       use_b   = valid_a ? 1'b0 : 1'b1;
wire       phase_ok = !phase_set || (mk == (phase ? MK_B : MK_A));
wire       hit      = mk_ok && phase_ok;

wire [15:0] dsd_l = layout ? pair[47:32] : pair[55:40];
wire [15:0] dsd_r = layout ? pair[15:0]  : pair[23:8];
wire        active_now = dop_active || (hit && ok_cnt >= CONFIRM - 6'd1);

always @(posedge bclk) begin
    dop_valid <= 1'b0;
    // 不使用「无帧超时」自动退出：格式切换 / SDRAM 重初始化期间
    // BCLK 仍在跑而 i2s_rx 处于复位、pair_valid 会暂停数百 µs，
    // 任何超时都会误把 dop_active 清掉，打乱 2 帧拼 1 帧对的相位。
    // 退出只由「连续 MISS 帧 marker 缺失」触发，流真正结束由
    // 后续 PCM 帧（无 marker）自然完成。
    if (!rst_n) begin
        phase_set <= 1'b0; ok_cnt <= 6'd0; miss_cnt <= 3'd0; half <= 1'b0;
        // dop_active / layout 刻意保留：见模块头注释。
    end else if (pair_valid) begin
        if (hit) begin
            phase     <= (mk == MK_A) ? 1'b1 : 1'b0;
            phase_set <= 1'b1;
            miss_cnt  <= 3'd0;
            if (ok_cnt < CONFIRM) ok_cnt <= ok_cnt + 6'd1;
            if (!dop_active && ok_cnt >= CONFIRM - 6'd1) dop_active <= 1'b1;
            if (!dop_active) layout <= use_b;
            if (active_now) begin
                if (!half) begin
                    l_hi <= dsd_l; r_hi <= dsd_r; half <= 1'b1;
                end else begin
                    dop_pair  <= {l_hi, dsd_l, r_hi, dsd_r};
                    dop_valid <= 1'b1;
                    half      <= 1'b0;
                end
            end
        end else begin
            ok_cnt    <= 6'd0;
            phase_set <= 1'b0;
            half      <= 1'b0;
            if (dop_active) begin
                if (miss_cnt >= MISS - 3'd1) begin
                    dop_active <= 1'b0;
                    miss_cnt   <= 3'd0;
                end else begin
                    miss_cnt <= miss_cnt + 3'd1;
                end
            end
        end
    end
end
endmodule

//==============================================================================
// native DSD 接收器：DCLK 上升沿同时锁存 L/R 各 1 bit，
// 每 32 bit 打包成一个 64bit 帧对 {L[31:0], R[31:0]}，
// MSB 为最早到达的 bit。DSD 无帧结构，任意打包边界都
// 保持位精确；输出侧按同样的 32bit 组串行化即可还原码流。
//==============================================================================
module dsd_rx (
    input  wire        bclk,
    input  wire        rst_n,
    input  wire        d_l,
    input  wire        d_r,
    output reg  [63:0] pair,
    output reg         pair_valid
);
reg [31:0] cap_l, cap_r;
reg [4:0]  bidx;
initial begin
    cap_l = 32'd0; cap_r = 32'd0; bidx = 5'd0;
    pair = 64'd0; pair_valid = 1'b0;
end
always @(posedge bclk) begin
    if (!rst_n) begin
        cap_l <= 32'd0; cap_r <= 32'd0; bidx <= 5'd0;
        pair <= 64'd0; pair_valid <= 1'b0;
    end else begin
        pair_valid <= 1'b0;
        if (bidx == 5'd31) begin
            pair       <= {{cap_l[30:0], d_l}, {cap_r[30:0], d_r}};
            pair_valid <= 1'b1;
            cap_l      <= 32'd0;
            cap_r      <= 32'd0;
            bidx       <= 5'd0;
        end else begin
            cap_l <= {cap_l[30:0], d_l};
            cap_r <= {cap_r[30:0], d_r};
            bidx  <= bidx + 5'd1;
        end
    end
end
endmodule

//==============================================================================
// I2S 接收器：自动测量帧长（位深），MSB 优先
// 捕获，并把每声道归一化为 32 位 MSB 对齐字。
//==============================================================================
module i2s_rx #(parameter P_DELAY = 0) (
    input  wire        bclk,
    input  wire        rst_n,
    input  wire        lrck,
    input  wire        sdata,
    output reg  [63:0] pair,
    output reg         pair_valid,
    output reg  [5:0]  slots
);
reg        lr_d;
reg [31:0] cap, left;
reg [6:0]  idx;
reg        first, have_left;
initial begin
    lr_d = 0; cap = 0; left = 0; idx = 0; first = 1'b1; have_left = 1'b0;
    pair = 0; pair_valid = 0; slots = 6'd32;
end

wire [6:0]  used    = (idx > 7'd32) ? 7'd32 : idx;
wire [31:0] cap_adj = P_DELAY ? {cap[30:0], 1'b0} : cap;
wire [31:0] word32  = cap_adj << (7'd32 - used);

always @(posedge bclk) begin
    if (!rst_n) begin
        lr_d <= 1'b0; cap <= 32'd0; left <= 32'd0; idx <= 7'd0;
        first <= 1'b1; have_left <= 1'b0;
        pair <= 64'd0; pair_valid <= 1'b0; slots <= 6'd32;
    end else begin
        pair_valid <= 1'b0;
        lr_d <= lrck;
        if (lrck != lr_d) begin
            if (!first) begin
                if (lr_d == 1'b0) begin
                    left      <= word32;            // 刚结束的半帧为左声道
                    have_left <= 1'b1;
                end else if (have_left) begin
                    pair       <= {left, word32};  // 刚结束的半帧为右声道
                    pair_valid <= 1'b1;
                end
            end
            slots <= used[5:0];
            first <= 1'b0;
            idx   <= 7'd1;
            cap   <= {31'd0, sdata};
        end else begin
            // 采用饱和：若 LRCK 停止而 BCLK 继续（播放源暂停），
            // 7 位 idx 回绕会产生错误的移位量，
            // 把陈旧的捕获位变成刺耳噪声。
            if (idx < 7'd63) idx <= idx + 7'd1;
            if (idx < 7'd32) cap <= {cap[30:0], sdata};
        end
    end
end
endmodule

//==============================================================================
// 真双时钟（异步）FIFO，格雷码指针同步器，
// 首字直通（FWFT）读数据。
//==============================================================================
module async_fifo #(
    parameter DSIZE = 64,
    parameter AW    = 8
)(
    input  wire              wclk,
    input  wire              wrst_n,
    input  wire              winc,
    input  wire [DSIZE-1:0]  wdata,
    output wire              wfull,
    input  wire              rclk,
    input  wire              rrst_n,
    input  wire              rinc,
    output wire [DSIZE-1:0]  rdata,
    output wire              rempty
);
localparam DEPTH = 1 << AW;
// no_rw_check：满/空逻辑保证同一时钟内读写不会指向
// 同一地址，因此 Quartus 无需为推断的双时钟 RAM
// 增加读期间写直通逻辑。
(* ramstyle = "no_rw_check" *) reg [DSIZE-1:0] mem [0:DEPTH-1];
reg  [AW:0] wptr, rptr;
reg  [AW:0] rgray_w1, rgray_w2, wgray_r1, wgray_r2;
wire [AW:0] wgray = (wptr >> 1) ^ wptr;
wire [AW:0] rgray = (rptr >> 1) ^ rptr;
integer i;
initial begin
    wptr = 0; rptr = 0; rgray_w1 = 0; rgray_w2 = 0; wgray_r1 = 0; wgray_r2 = 0;
    for (i = 0; i < DEPTH; i = i + 1) mem[i] = {DSIZE{1'b0}};
end

assign wfull  = (wgray == {~rgray_w2[AW:AW-1], rgray_w2[AW-2:0]});
assign rempty = (rgray == wgray_r2);
assign rdata  = mem[rptr[AW-1:0]];

always @(posedge wclk) begin
    if (!wrst_n) begin
        wptr <= 0; rgray_w1 <= 0; rgray_w2 <= 0;
    end else begin
        rgray_w1 <= rgray;
        rgray_w2 <= rgray_w1;
        if (winc && !wfull) begin
            mem[wptr[AW-1:0]] <= wdata;
            wptr <= wptr + 1'b1;
        end
    end
end

always @(posedge rclk) begin
    if (!rrst_n) begin
        rptr <= 0; wgray_r1 <= 0; wgray_r2 <= 0;
    end else begin
        wgray_r1 <= wgray;
        wgray_r2 <= wgray_r1;
        if (rinc && !rempty) rptr <= rptr + 1'b1;
    end
end
endmodule

//==============================================================================
// 单时钟 FWFT FIFO，位于 SDRAM 读回与发射器之间。
//==============================================================================
module sync_fifo #(
    parameter DSIZE = 64,
    parameter AW    = 8
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              winc,
    input  wire [DSIZE-1:0]  wdata,
    output wire              wfull,
    input  wire              rinc,
    output wire [DSIZE-1:0]  rdata,
    output wire              rempty
);
localparam DEPTH = 1 << AW;
(* ramstyle = "no_rw_check" *) reg [DSIZE-1:0] mem [0:DEPTH-1];
reg  [AW:0] wptr, rptr, count;
integer i;
initial begin
    wptr = 0; rptr = 0; count = 0;
    for (i = 0; i < DEPTH; i = i + 1) mem[i] = {DSIZE{1'b0}};
end
assign wfull  = (count == DEPTH);
assign rempty = (count == 0);
assign rdata  = mem[rptr[AW-1:0]];

always @(posedge clk) begin
    if (!rst_n) begin
        wptr <= 0; rptr <= 0; count <= 0;
    end else begin
        case ({winc && !wfull, rinc && !rempty})
            2'b10: count <= count + 1'b1;
            2'b01: count <= count - 1'b1;
            default: count <= count;
        endcase
        if (winc && !wfull) begin
            mem[wptr[AW-1:0]] <= wdata;
            wptr <= wptr + 1'b1;
        end
        if (rinc && !rempty) rptr <= rptr + 1'b1;
    end
end
endmodule

//==============================================================================
// W9825G6JH（4M × 4 bank × 16 bit）控制器。
// 单字访问、自动预充电、CL=2、周期性自动刷新。
// 字地址 {bank[1:0], row[12:0], col[8:0]} → 用满 256 Mbit。
//==============================================================================
module sdram_ctrl #(
    parameter INIT_WAIT = 16'd12000,
    parameter REF_INT   = 16'd320,
    parameter T_RP      = 4'd3,
    // 45~49 MHz 下 tCK≈20~22 ns，远超 W9825G6JH 的 tRCD(min)=18 ns，
    // 2 个周期已很保守；CL=2 在 CL2 允许的 tCK(min)=10 ns 内也成立。
    // 缩短这两个等待让单字访问从 17 拍降到 14 拍，从而使
    // DSD512（22.5792/24.576 Mbps）也能走完整 SDRAM 环形缓冲。
    parameter T_RCD     = 4'd2,
    parameter T_RFC     = 4'd8,
    parameter T_WR      = 4'd3,
    parameter T_MRD     = 4'd3,
    parameter CL        = 4'd2
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        wr_req,
    input  wire [23:0] wr_addr,
    input  wire [15:0] wr_data,
    output reg         wr_ack,
    input  wire        rd_req,
    input  wire [23:0] rd_addr,
    output reg  [15:0] rd_data,
    output reg         rd_valid,
    output wire        busy,
    output reg         init_done,
    output reg         cke,
    output reg         cs_n, ras_n, cas_n, we_n,
    output reg  [1:0]  ba,
    output reg  [12:0] addr,
    output reg  [1:0]  dqm,
    input  wire [15:0] dq_in,
    output reg  [15:0] dq_out,
    output reg         dq_oe
);
// {cs_n,ras_n,cas_n,we_n}。空闲时用 DEVICE DESELECT（cs_n=1）
// 而非全选中的 NOP，使 cs_n 成为真实命令信号而非固定电平。
localparam [3:0] CMD_NOP = 4'b1111, CMD_ACT = 4'b0011,
                 CMD_RD  = 4'b0101, CMD_WR  = 4'b0100,
                 CMD_PRE = 4'b0010, CMD_REF = 4'b0001,
                 CMD_MRS = 4'b0000;
localparam [12:0] MRS_VAL = 13'h0020;  // A6:A4=010 → CL=2；BL=1、顺序、突发写
localparam [12:0] PRE_ALL = 13'h0400;  // A10=1：预充电所有 bank

localparam [3:0] S_WAIT = 4'd0,  S_PRE = 4'd1,  S_REF0 = 4'd2,
                 S_MRS  = 4'd3,  S_IDLE = 4'd4, S_AUTO_REF = 4'd5,
                 S_WACT = 4'd6,  S_WCMD = 4'd7,
                 S_RACT = 4'd8,  S_RCMD = 4'd9;

reg [3:0]  st;
reg [15:0] t;
reg [3:0]  ref_cnt;
reg [15:0] ref_timer;
reg        ref_due;
reg [3:0]  cmd_r;
reg [23:0] a_latch;

initial begin
    st = S_WAIT; t = 0; ref_cnt = 0; ref_timer = 0; ref_due = 0;
    cmd_r = CMD_NOP; a_latch = 0; init_done = 0;
    cke = 1'b0; ba = 0; addr = 0; dq_out = 0; dq_oe = 0; dqm = 2'b11;
    wr_ack = 0; rd_valid = 0; rd_data = 0;
    cs_n = CMD_NOP[3]; ras_n = CMD_NOP[2];
    cas_n = CMD_NOP[1]; we_n = CMD_NOP[0];
end

assign busy = (st != S_IDLE);
// DQM 必须在 READ 命令后的整个 CAS 延迟窗口内保持为 0。
// 若只在该命令那一拍打开、之后拉高屏蔽，真实 W9825G6JH 会把
// DQ 输出置为高阻，控制器读回无效值：4 个 16 位字会变成相同或无效数据，
// 表现为输出变单声道且高频丢失（行为仿真模型此前未覆盖这一点）。
// 空闲/刷新时才屏蔽为 2'b11，同时避免该引脚被优化成固定电平。
wire dqm_open = (st == S_WACT) || (st == S_WCMD) ||
                (st == S_RACT) || (st == S_RCMD);

// 字地址 = {bank[1:0], row[12:0], col[8:0]}（a_latch，在下方解码）
wire [12:0] col_cmd = {2'b00, 1'b1, 1'b0, 9'b0}; // A12/11=0，A10=1（自动预充电），A9=0

always @(posedge clk) begin
    if (!rst_n) begin
        st <= S_WAIT; t <= 16'd0; ref_cnt <= 4'd0; ref_timer <= 16'd0;
        ref_due <= 1'b0; init_done <= 1'b0; cmd_r <= CMD_NOP;
        // 复位时 DQM 保持打开：此时 cke=0 / dq_oe=0，SDRAM 不响应
        // 命令、总线也不由 FPGA 驱动，屏蔽没有意义；反而若复位恰好
        // 落在一条尚未结束的 READ 的 CAS 延迟窗口内，拉高 DQM 会让
        // 真实芯片把 DQ 置为高阻（读窗口被屏蔽）。空闲态仍由
        // dqm_open 控制为 2'b11，引脚不会被优化成固定电平。
        cke <= 1'b0; ba <= 2'd0; addr <= 13'd0; dqm <= 2'b00;
        dq_out <= 16'd0; dq_oe <= 1'b0;
        wr_ack <= 1'b0; rd_valid <= 1'b0; rd_data <= 16'd0;
        a_latch <= 24'd0;
    end else begin
        wr_ack   <= 1'b0;
        rd_valid <= 1'b0;
        dqm      <= dqm_open ? 2'b00 : 2'b11;   // 读/写窗口内全程打开
        // 刷新定时器（8192 周期 / 64 ms → 一次 AUTO REFRESH < 7.8125 µs）
        if (ref_timer >= REF_INT - 16'd1) begin
            ref_timer <= 16'd0;
            ref_due   <= 1'b1;
        end else begin
            ref_timer <= ref_timer + 16'd1;
        end

        case (st)
            S_WAIT: begin
                cmd_r <= CMD_NOP;        // 200 µs 上电期间 CKE 保持低
                if (t >= INIT_WAIT) begin cke <= 1'b1; st <= S_PRE; t <= 0; end
                else t <= t + 16'd1;
            end
            S_PRE: begin
                if (t == 16'd0) begin cmd_r <= CMD_PRE; addr <= PRE_ALL; ba <= 2'd0; end
                else cmd_r <= CMD_NOP;
                if (t >= T_RP - 16'd1) begin st <= S_REF0; t <= 0; end
                else t <= t + 16'd1;
            end
            S_REF0: begin
                if (t == 16'd0) begin
                    cmd_r   <= CMD_REF;
                    ref_cnt <= ref_cnt + 4'd1;
                    ref_due <= 1'b0;
                end else cmd_r <= CMD_NOP;
                if (t >= T_RFC - 16'd1) begin
                    t <= 0;
                    if (ref_cnt >= 4'd8) st <= S_MRS;
                end else t <= t + 16'd1;
            end
            S_MRS: begin
                if (t == 16'd0) begin cmd_r <= CMD_MRS; addr <= MRS_VAL; ba <= 2'd0; end
                else cmd_r <= CMD_NOP;
                if (t >= T_MRD - 16'd1) begin
                    st <= S_IDLE; t <= 0; init_done <= 1'b1;
                end else t <= t + 16'd1;
            end
            S_IDLE: begin
                cmd_r <= CMD_NOP; addr <= 13'd0; ba <= 2'd0; dq_oe <= 1'b0;
                t <= 16'd0;
                if (ref_due) begin
                    st <= S_AUTO_REF;
                end else if (wr_req) begin
                    a_latch <= wr_addr;
                    st <= S_WACT;
                end else if (rd_req) begin
                    a_latch <= rd_addr;
                    st <= S_RACT;
                end
            end
            S_AUTO_REF: begin
                if (t == 16'd0) begin cmd_r <= CMD_REF; ref_due <= 1'b0; end
                else cmd_r <= CMD_NOP;
                if (t >= T_RFC - 16'd1) begin st <= S_IDLE; t <= 0; end
                else t <= t + 16'd1;
            end
            S_WACT: begin
                if (t == 16'd0) begin
                    cmd_r <= CMD_ACT;
                    ba    <= a_latch[23:22];
                    addr  <= a_latch[21:9];
                end else cmd_r <= CMD_NOP;
                if (t >= T_RCD - 16'd1) begin st <= S_WCMD; t <= 0; end
                else t <= t + 16'd1;
            end
            S_WCMD: begin
                if (t == 16'd0) begin
                    cmd_r  <= CMD_WR;
                    ba     <= a_latch[23:22];
                    addr   <= col_cmd | {4'd0, a_latch[8:0]};
                    dq_out <= wr_data;
                    dq_oe  <= 1'b1;
                    wr_ack <= 1'b1;   // DQM 由 dqm_open 在整个写窗口保持 0
                end else begin
                    cmd_r <= CMD_NOP;
                    dq_oe <= 1'b0;
                end
                if (t >= T_WR - 16'd1) begin st <= S_IDLE; t <= 0; end
                else t <= t + 16'd1;
            end
            S_RACT: begin
                if (t == 16'd0) begin
                    cmd_r <= CMD_ACT;
                    ba    <= a_latch[23:22];
                    addr  <= a_latch[21:9];
                end else cmd_r <= CMD_NOP;
                if (t >= T_RCD - 16'd1) begin st <= S_RCMD; t <= 0; end
                else t <= t + 16'd1;
            end
            S_RCMD: begin
                if (t == 16'd0) begin
                    cmd_r <= CMD_RD;
                    ba    <= a_latch[23:22];
                    addr  <= col_cmd | {4'd0, a_latch[8:0]};
                end else cmd_r <= CMD_NOP;
                // DQM 由 dqm_open 在整个 CAS 延迟窗口保持 0，
                // 直到 rd_data 锁存完成，否则 DQ 会被屏蔽成高阻。
                // 多等待一个周期，使缓冲管理器能先推进
                // 到下一个字地址，IDLE 再重新锁存 rd_addr
                if (t == CL + 16'd1) begin
                    rd_data  <= dq_in;              // CL 再加一个 core 时钟
                    rd_valid <= 1'b1;
                    t <= t + 16'd1;
                end else if (t == CL + 16'd2) begin
                    st <= S_IDLE; t <= 16'd0;
                end else t <= t + 16'd1;
            end
            default: st <= S_IDLE;
        endcase
    end
end

always @(*) begin
    cs_n  = cmd_r[3];
    ras_n = cmd_r[2];
    cas_n = cmd_r[1];
    we_n  = cmd_r[0];
end
endmodule
