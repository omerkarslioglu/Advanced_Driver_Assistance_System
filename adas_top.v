module adas_top
(
    // default inputs
    input clk;
    input rst_n;

    // from car
    input timer_trick;
    input mode_i;
    input [1:0] redlight_i;         // (1) lidar - (0) camera
    input [1:0] crosswall_i;
    input [7:0] distance_lidar_i;
    input [7:0] distance_cam_i;
    input [7:0] speed_measured_i;

    // from user
    input [7:0] distance_i;
    input [7:0] speed_i;

    // autonom
    output gas_i;
    output brake_i;

    // assistance
    output redlight_o;
    output crosswall_o;
    output follow_distance_o;
);


endmodule