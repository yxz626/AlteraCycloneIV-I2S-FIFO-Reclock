`timescale 1ps/1ps
// 仅用于功能验证的 W9825G6JH 行为级模型。
// 4 bank × 8192 行 × 512 列 × 16 bit；CL 可配（与控制器一致，默认 2）；自动预充电。
module sdram_model #(parameter CL = 2) (
    input  wire        clk,
    input  wire        cke,
    input  wire        cs_n,
    input  wire        ras_n,
    input  wire        cas_n,
    input  wire        we_n,
    input  wire [1:0]  ba,
    input  wire [12:0] addr,
    input  wire [1:0]  dqm,
    inout  wire [15:0] dq,
    output reg  [31:0] wr_cnt,
    output reg  [31:0] rd_cnt,
    output reg  [31:0] ref_cnt,
    output reg  [31:0] act_cnt,
    output reg  [31:0] pre_cnt,
    output reg  [31:0] mrs_cnt,
    output reg  [31:0] seq_err,
    output reg  [23:0] max_addr,
    output reg  [3:0]  banks_seen,
    output reg  [12:0] max_row,
    // READ 数据输出期间 DQM 被拉高的次数（必须为 0，否则真实 SDRAM 输出高阻）
    output reg  [31:0] dqm_masked_cnt
);
localparam [3:0] CMD_PRE = 4'b0010, CMD_REF = 4'b0001,
                 CMD_MRS = 4'b0000, CMD_ACT = 4'b0011,
                 CMD_WR  = 4'b0100, CMD_RD  = 4'b0101;

reg [15:0] mem [0:16777215];
reg [12:0] active_row [0:3];
reg [3:0]  row_open;
reg [15:0] rd_pipe [0:3];
reg [3:0]  rv;
wire [3:0] cmd = {cs_n, ras_n, cas_n, we_n};
wire [23:0] idx = {ba, active_row[ba], addr[8:0]};
integer i;

initial begin
    wr_cnt = 0; rd_cnt = 0; ref_cnt = 0; act_cnt = 0; pre_cnt = 0;
    mrs_cnt = 0; seq_err = 0; max_addr = 0; banks_seen = 0; max_row = 0;
    dqm_masked_cnt = 0; row_open = 0; rv = 0;
    for (i = 0; i < 4; i = i + 1) begin active_row[i] = 0; rd_pipe[i] = 0; end
end

// READ 命令后第 CL 个时钟沿驱动数据（CL=2 与 RTL 控制器一致）。
always @(posedge clk) begin
    // READ 输出窗口内 DQM 必须为 0；拉高会使 DQ 进入高阻，
    // 控制器读回无效值。统计该违规以便仿真抓到硬件才会暴露的问题。
    if (rv[CL] && (cmd != CMD_WR) && (dqm != 2'b00))
        dqm_masked_cnt <= dqm_masked_cnt + 1;

    rv[3]      <= rv[2];      rd_pipe[3] <= rd_pipe[2];
    rv[2]      <= rv[1];      rd_pipe[2] <= rd_pipe[1];
    rv[1]      <= rv[0];      rd_pipe[1] <= rd_pipe[0];
    rv[0]      <= 1'b0;       rd_pipe[0] <= 16'd0;

    if (cke && !cs_n) begin
        case (cmd)
            CMD_PRE: begin
                row_open <= 4'b0000;
                pre_cnt  <= pre_cnt + 1;
            end
            CMD_REF: begin
                ref_cnt <= ref_cnt + 1;
            end
            CMD_MRS: begin
                mrs_cnt <= mrs_cnt + 1;
                if (addr[6:4] == CL[2:0] && addr[2:0] == 3'b000)
                    ; // 正确的 CAS Latency、突发长度 1、顺序方式
                else
                    seq_err <= seq_err + 1;
            end
            CMD_ACT: begin
                active_row[ba] <= addr[12:0];
                row_open[ba]   <= 1'b1;
                act_cnt        <= act_cnt + 1;
                banks_seen     <= banks_seen | (4'b0001 << ba);
                if (addr[12:0] > max_row) max_row <= addr[12:0];
            end
            CMD_WR: begin
                if (!row_open[ba]) seq_err <= seq_err + 1;
                wr_cnt <= wr_cnt + 1;
                if (idx > max_addr) max_addr <= idx;
                if (!dqm[0]) mem[idx][7:0]  = dq[7:0];
                if (!dqm[1]) mem[idx][15:8] = dq[15:8];
                if (addr[10]) row_open[ba] <= 1'b0;
            end
            CMD_RD: begin
                if (!row_open[ba]) seq_err <= seq_err + 1;
                rd_cnt     <= rd_cnt + 1;
                rv[0]      <= 1'b1;
                rd_pipe[0] <= mem[idx];
                if (addr[10]) row_open[ba] <= 1'b0;
            end
            default: ;
        endcase
    end
end

// 按字节实现 DQM 输出屏蔽，与 W9825G6JH 的 DQ 行为一致。
assign dq[7:0]  = (rv[CL] && (cmd != CMD_WR) && !dqm[0]) ? rd_pipe[CL][7:0]  : 8'bz;
assign dq[15:8] = (rv[CL] && (cmd != CMD_WR) && !dqm[1]) ? rd_pipe[CL][15:8] : 8'bz;
endmodule
