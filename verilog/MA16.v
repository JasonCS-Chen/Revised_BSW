// max 16 submodule which compares H table in 16 PEs
module MA16(H0,H1,H2,H3,H4,H5,H6,H7,H8,H9,H10,H11,H12,H13,H14,H15,MA_p,MA_out,R_shift_sig, D_shift_sig);
	
	input  [6:0] H0,H1,H2,H3,H4,H5,H6,H7,H8,H9,H10,H11,H12,H13,H14,H15;
	output [3:0] MA_p;
	output [6:0] MA_out;
	output R_shift_sig, D_shift_sig;
	
	
	wire [6:0] H_sub0,H_sub1,H_sub2,H_sub3;
	wire [1:0] MA_p_sub_0,MA_p_sub_1,MA_p_sub_2,MA_p_sub_3,MA_p_temp;
	reg  [3:0] MA_p_final;
	wire [9:0] L_sum, R_sum;
	
	assign MA_p = MA_p_final;
	
	assign L_sum = H0+H1+H2+H3+H4+H5+H6+H7;
	assign R_sum = H8+H9+H10+H11+H12+H13+H14+H15;
	
	assign R_shift_sig = (L_sum - 2 > R_sum);
	assign D_shift_sig = (L_sum < R_sum - 2);
	
	MA4 max_sub_0(.H0(H0),.H1(H1),.H2(H2),.H3(H3),.MA_p(MA_p_sub_0),.MA_out(H_sub0));
	MA4 max_sub_1(.H0(H4),.H1(H5),.H2(H6),.H3(H7),.MA_p(MA_p_sub_1),.MA_out(H_sub1));
	MA4 max_sub_2(.H0(H8),.H1(H9),.H2(H10),.H3(H11),.MA_p(MA_p_sub_2),.MA_out(H_sub2));
	MA4 max_sub_3(.H0(H12),.H1(H13),.H2(H14),.H3(H15),.MA_p(MA_p_sub_3),.MA_out(H_sub3));
	
	MA4 max_sub_4(.H0(H_sub0),.H1(H_sub1),.H2(H_sub2),.H3(H_sub3),.MA_p(MA_p_temp),.MA_out(MA_out));
	
	always @(*) begin
        case(MA_p_temp)
			0: MA_p_final = MA_p_sub_0;
			1: MA_p_final = MA_p_sub_1 + 4;
			2: MA_p_final = MA_p_sub_2 + 8;
            3: MA_p_final = MA_p_sub_3 + 12;
        endcase
    end

endmodule
