// max 4 submodule which compares H table in 4 PEs
module MA4(H0,H1,H2,H3,MA_p,MA_out);
	
	input  [6:0] H0,H1,H2,H3;
	output [1:0] MA_p;
	output [6:0] MA_out;
	
	wire [6:0] H01,H23;
	wire [1:0] MA_01,MA_23;
	
	assign	H01 = (H0>=H1)? H0:H1;
	assign  H23 = (H2>=H3)? H2:H3;
	
	assign	MA_01 = (H0>=H1)? 2'd0:2'd1;
	assign  MA_23 = (H2>=H3)? 2'd2:2'd3;
	
	assign  MA_p = (H01>=H23)? MA_01:MA_23;
	assign  MA_out = (H01>=H23)? H01:H23;
endmodule
