// MokuOS >=4.1.1 CustomInstrument port declaration.
// Target Moku:Go, 31.25 MHz, three-slot Multi-Instrument Mode.
module CustomInstrument (
  input wire Clk, Reset,
  input wire [31:0] Sync,
  input wire signed [15:0] InputA, InputB, InputC, InputD,
  output wire signed [15:0] OutputA, OutputB, OutputC, OutputD,
  input wire [31:0] Control [0:15],
  output wire [31:0] Status [0:15]
);
  laser_control core(.Clk(Clk), .Reset(Reset), .InputA(InputA), .InputB(InputB),
    .InputC(InputC), .InputD(InputD), .Control(Control), .Status(Status),
    .OutputA(OutputA), .OutputB(OutputB));
  assign OutputC=16'sd0;
  assign OutputD=16'sd0;
endmodule
