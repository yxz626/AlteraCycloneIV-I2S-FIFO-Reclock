`timescale 1ns/1ps
// Quartus ALTCLKCTRL 宏功能的行为级仿真替身。
module altclkctrl (
    inclk, clkselect, ena, outclk
);
parameter clock_type = "AUTO";
parameter ena_register_mode = "FALLING EDGE";
parameter implement_in_les = "OFF";
parameter number_of_clocks = 4;
parameter use_glitch_free_switch_over_implementation = "OFF";
parameter width_clkselect = 2;

input  [number_of_clocks-1:0] inclk;
input  [width_clkselect-1:0]  clkselect;
input                         ena;
output                        outclk;

assign outclk = ena ? inclk[clkselect] : 1'b0;
endmodule
