module SW #(parameter WIDTH_SCORE = 8, parameter WIDTH_POS_REF = 7, parameter WIDTH_POS_QUERY = 6)
(
    input           clk,
    input           reset,
    input           valid,
    input [1:0]     data_ref,
    input [1:0]     data_query,
    output          finish,
    output [WIDTH_SCORE - 1:0]   max,
    output [WIDTH_POS_REF - 1:0]   pos_ref,
    output [WIDTH_POS_QUERY - 1:0]   pos_query
);

//------------------------------------------------------------------
// parameter
//------------------------------------------------------------------
	
	// state variable
	localparam IDLE = 4'b0000;
	localparam INIT = 4'd1;
	localparam INIF = 4'd2;
	localparam DELA = 4'd3;
	localparam RR   = 4'd4;
	localparam DR   = 4'd5;
	localparam RD   = 4'd6;
	localparam DD   = 4'd7;
	localparam RF   = 4'd8;
	localparam DF   = 4'd9;
	localparam REFF = 4'd10;
	localparam QUEF = 4'd11;
	localparam OUTP = 4'd12;
	
	// number of PEs
	localparam PE_num = 16;
	
	// for loop
	integer m, n, p;
	
//------------------------------------------------------------------
// reg & wire
//------------------------------------------------------------------
	
	// 20221202 ver. 
	// DONE: finish the code (HOW?, Reg table?, SYN?) and test the code (Not pass all tb, still have errors)
	// TODO: correct the errors -> 1203 finish debugging, extend to all input length, simplify code
	// COMMENT: if use memory as queue, can reduce the input buffer
	
	// state register
	reg [3:0] state, state_nxt;
	
	// signed counter for comparison
	reg signed [8:0] counter, counter_nxt;
	
	// register updating max
	reg [WIDTH_SCORE - 2:0] max_cur, max_nxt;
	
	// register for input sequences
	reg [2*PE_num-1:0]   ref_in_shift,   ref_in_shift_n;
	reg [2*PE_num-1:0] query_in_shift, query_in_shift_n;
	
	// input buffer and shift positions
	reg [1:0] qbuffer [0:47], qbuffer_n [0:47];
	reg [1:0] rbuffer [0:63], rbuffer_n [0:63];
	reg [WIDTH_POS_REF - 1:0]   R_shift, R_shift_nxt;
	reg [WIDTH_POS_QUERY - 1:0] D_shift, D_shift_nxt;
	
	// S I D H tables
	reg signed [2:0] S [0:2*PE_num - 1];  // for convenience
	reg signed [WIDTH_SCORE - 1:0] I  [0:PE_num - 1], I_nxt  [0:PE_num - 1];
	reg signed [WIDTH_SCORE - 1:0] D  [0:PE_num - 1], D_nxt  [0:PE_num - 1];
	reg signed [WIDTH_SCORE - 1:0] H  [0:PE_num - 1], H_nxt  [0:PE_num - 1];
	reg signed [WIDTH_SCORE - 1:0] Hd [0:PE_num - 1], Hd_nxt [0:PE_num - 1];
	
	// S I D H inputs
	reg signed [WIDTH_SCORE - 1:0] I_in[0:PE_num - 1], D_in[0:PE_num - 1], H_in0[0:PE_num - 1], H_in1[0:PE_num - 1], H_in2[0:PE_num - 1];
	
	// I D H outputs
	wire signed [WIDTH_SCORE - 1:0] I_out[0:PE_num - 1], D_out[0:PE_num - 1];
	wire signed [WIDTH_SCORE - 2:0] H_out[0:PE_num - 1];
	
	// control
	wire R_shift_sig, D_shift_sig;

	// MA16
	wire [3:0] MA_p;
	reg  [3:0] MA_p_r, MA_p_rn;
	wire [WIDTH_SCORE - 2:0] MA_out;
	
	// PE 0 position at max
	reg [WIDTH_POS_REF - 1:0] imax, imax_nxt;
	reg [WIDTH_POS_QUERY - 1:0] jmax, jmax_nxt;
	
	// outputs assignment
	assign finish = (state == OUTP)?1'b1:1'b0;
	assign max = {1'b0,max_cur};
	
	assign pos_ref   = imax - MA_p_r;
	assign pos_query = jmax + MA_p_r;


//------------------------------------------------------------------
// submodule
//------------------------------------------------------------------

	// 16 PEs
	PE pe0(.S(S[0]),.I_in(I_in[0]),.D_in(D_in[0]),.H_in0(H_in0[0]),.H_in1(H_in1[0]),.H_in2(H_in2[0]),.I_out(I_out[0]),.D_out(D_out[0]),.H_out(H_out[0])); 
	PE pe1(.S(S[2]),.I_in(I_in[1]),.D_in(D_in[1]),.H_in0(H_in0[1]),.H_in1(H_in1[1]),.H_in2(H_in2[1]),.I_out(I_out[1]),.D_out(D_out[1]),.H_out(H_out[1]));
	PE pe2(.S(S[4]),.I_in(I_in[2]),.D_in(D_in[2]),.H_in0(H_in0[2]),.H_in1(H_in1[2]),.H_in2(H_in2[2]),.I_out(I_out[2]),.D_out(D_out[2]),.H_out(H_out[2])); 
	PE pe3(.S(S[6]),.I_in(I_in[3]),.D_in(D_in[3]),.H_in0(H_in0[3]),.H_in1(H_in1[3]),.H_in2(H_in2[3]),.I_out(I_out[3]),.D_out(D_out[3]),.H_out(H_out[3]));
	PE pe4(.S(S[8]),.I_in(I_in[4]),.D_in(D_in[4]),.H_in0(H_in0[4]),.H_in1(H_in1[4]),.H_in2(H_in2[4]),.I_out(I_out[4]),.D_out(D_out[4]),.H_out(H_out[4]));
	PE pe5(.S(S[10]),.I_in(I_in[5]),.D_in(D_in[5]),.H_in0(H_in0[5]),.H_in1(H_in1[5]),.H_in2(H_in2[5]),.I_out(I_out[5]),.D_out(D_out[5]),.H_out(H_out[5]));
	PE pe6(.S(S[12]),.I_in(I_in[6]),.D_in(D_in[6]),.H_in0(H_in0[6]),.H_in1(H_in1[6]),.H_in2(H_in2[6]),.I_out(I_out[6]),.D_out(D_out[6]),.H_out(H_out[6]));
	PE pe7(.S(S[14]),.I_in(I_in[7]),.D_in(D_in[7]),.H_in0(H_in0[7]),.H_in1(H_in1[7]),.H_in2(H_in2[7]),.I_out(I_out[7]),.D_out(D_out[7]),.H_out(H_out[7]));
	PE pe8(.S(S[16]),.I_in(I_in[8]),.D_in(D_in[8]),.H_in0(H_in0[8]),.H_in1(H_in1[8]),.H_in2(H_in2[8]),.I_out(I_out[8]),.D_out(D_out[8]),.H_out(H_out[8]));
	PE pe9(.S(S[18]),.I_in(I_in[9]),.D_in(D_in[9]),.H_in0(H_in0[9]),.H_in1(H_in1[9]),.H_in2(H_in2[9]),.I_out(I_out[9]),.D_out(D_out[9]),.H_out(H_out[9]));
	PE pe10(.S(S[20]),.I_in(I_in[10]),.D_in(D_in[10]),.H_in0(H_in0[10]),.H_in1(H_in1[10]),.H_in2(H_in2[10]),.I_out(I_out[10]),.D_out(D_out[10]),.H_out(H_out[10]));
	PE pe11(.S(S[22]),.I_in(I_in[11]),.D_in(D_in[11]),.H_in0(H_in0[11]),.H_in1(H_in1[11]),.H_in2(H_in2[11]),.I_out(I_out[11]),.D_out(D_out[11]),.H_out(H_out[11]));
	PE pe12(.S(S[24]),.I_in(I_in[12]),.D_in(D_in[12]),.H_in0(H_in0[12]),.H_in1(H_in1[12]),.H_in2(H_in2[12]),.I_out(I_out[12]),.D_out(D_out[12]),.H_out(H_out[12]));
	PE pe13(.S(S[26]),.I_in(I_in[13]),.D_in(D_in[13]),.H_in0(H_in0[13]),.H_in1(H_in1[13]),.H_in2(H_in2[13]),.I_out(I_out[13]),.D_out(D_out[13]),.H_out(H_out[13]));
	PE pe14(.S(S[28]),.I_in(I_in[14]),.D_in(D_in[14]),.H_in0(H_in0[14]),.H_in1(H_in1[14]),.H_in2(H_in2[14]),.I_out(I_out[14]),.D_out(D_out[14]),.H_out(H_out[14]));
	PE pe15(.S(S[30]),.I_in(I_in[15]),.D_in(D_in[15]),.H_in0(H_in0[15]),.H_in1(H_in1[15]),.H_in2(H_in2[15]),.I_out(I_out[15]),.D_out(D_out[15]),.H_out(H_out[15]));
	
	// max 16
	MA16 max0(.H0(H_out[0]),.H1(H_out[1]),.H2(H_out[2]),  .H3(H_out[3]),  .H4(H_out[4]),  .H5(H_out[5]),  .H6(H_out[6]),  .H7(H_out[7]),
			  .H8(H_out[8]),.H9(H_out[9]),.H10(H_out[10]),.H11(H_out[11]),.H12(H_out[12]),.H13(H_out[13]),.H14(H_out[14]),.H15(H_out[15]),
			  .MA_p(MA_p), .MA_out(MA_out),.R_shift_sig(R_shift_sig),.D_shift_sig(D_shift_sig));

//------------------------------------------------------------------
// combinational part
//------------------------------------------------------------------

	// finite state machine and state means previous two direction
    always @(*) begin
        case(state)
			IDLE: begin
				if(~valid) state_nxt = IDLE;
				else state_nxt = INIT;
			end
			
			INIT: state_nxt = (counter != PE_num - 1)? INIT: INIF;
			
			INIF: state_nxt = DELA;	 // delay for input
			
			DELA: begin
				if (R_shift_sig)         state_nxt = RR;
				else if (D_shift_sig)    state_nxt = RD;
				else                     state_nxt = RD;
			end
			
			RR, DR: begin
				if (R_shift == 64)       state_nxt = RF;
				else if (R_shift_sig)    state_nxt = RR;
				else if (D_shift_sig)    state_nxt = RD;
				else                     state_nxt = RD;
			end
			
			RD, DD: begin
				if (D_shift == 33)       state_nxt = DF;
				else if (R_shift_sig)    state_nxt = DR;
				else if (D_shift_sig)    state_nxt = DD;
				else                     state_nxt = DR;
			end
			
			RF: state_nxt = REFF;	
			DF: state_nxt = QUEF;
			REFF: state_nxt = (counter != 64 + 48 + 1)? REFF: OUTP;
			QUEF: state_nxt = (counter != 64 + 48 + 1)? QUEF: OUTP;
			OUTP: state_nxt = IDLE;
			
            default: state_nxt = state;
        endcase
    end
	
	// counter
	always @(*) begin
		case(state)
            IDLE: begin
				if(~valid) counter_nxt = 0;
				else counter_nxt = 1;
			end
			
            default: counter_nxt = (counter != (64 + 48 + 1))? (counter + 1) : 0;
        endcase
	end
	
	// input buffer
	always @(*) begin
		case(state)
            IDLE: begin
				for (m = 0; m <= 63; m = m+1) rbuffer_n[m] = 0;
				for (n = 0; n <= 47; n = n+1) qbuffer_n[n] = 0;
				
				if(valid) begin
					qbuffer_n[counter] = data_ref;
					rbuffer_n[counter] = data_query;
				end
			end
			
			
            default: begin
				for (m = 0; m <= 63; m = m+1) rbuffer_n[m] = rbuffer[m];
				for (n = 0; n <= 47; n = n+1) qbuffer_n[n] = qbuffer[n];
				if (counter <= 47) begin	
					rbuffer_n[counter] = data_ref;
					qbuffer_n[counter] = data_query;
				end
				
				else if (counter <= 63) begin
					rbuffer_n[counter] = data_ref;
				end
			end
        endcase
	end
	
	
	// next input sequences and consider cases for next cycle
	always @(*) begin
		case(state)
            IDLE: begin
				ref_in_shift_n    = 0;
				query_in_shift_n  = 0; 
				
				R_shift_nxt       = 0;
				D_shift_nxt       = 0;
				
				if(valid) begin
					ref_in_shift_n[1:0]   = data_ref;
					query_in_shift_n[1:0] = data_query;
					
					R_shift_nxt         = 1;
				    D_shift_nxt         = 1;
				end
			end
			
			INIT: begin
				R_shift_nxt       = R_shift + 1;
				D_shift_nxt       = D_shift;
				
				ref_in_shift_n      = ref_in_shift << 2;
				ref_in_shift_n[1:0] = data_ref;
				
				query_in_shift_n    = (data_query << (counter << 1)) + query_in_shift;
			end
			
            RR, DR, RD, DD, DELA: begin
				if (R_shift == 64) begin
					ref_in_shift_n    = ref_in_shift;
					
					query_in_shift_n        = query_in_shift >> 2;
					query_in_shift_n[31:30] = qbuffer[D_shift + 15]; // index from 0
					
					R_shift_nxt       = R_shift;
					D_shift_nxt       = D_shift + 1;
				end
				
				else if (D_shift == 33) begin  // 48 - 15
					ref_in_shift_n        = ref_in_shift << 2;
					ref_in_shift_n[1:0]   = rbuffer[R_shift];
					
					query_in_shift_n  = query_in_shift;
					
					R_shift_nxt       = R_shift + 1;
					D_shift_nxt       = D_shift;
				end
				
				else if (R_shift_sig) begin // shift right
					ref_in_shift_n        = ref_in_shift << 2;
					ref_in_shift_n[1:0]   = rbuffer[R_shift];
					
					query_in_shift_n  = query_in_shift;
					
					R_shift_nxt       = R_shift + 1;
					D_shift_nxt       = D_shift;
				end
				
				else if (D_shift_sig) begin // shift down
					ref_in_shift_n    = ref_in_shift;
					
					query_in_shift_n        = query_in_shift >> 2;
					query_in_shift_n[31:30] = qbuffer[D_shift + 15]; // index from 0
					
					R_shift_nxt       = R_shift;
					D_shift_nxt       = D_shift + 1;
				end
				
				else begin // toggle
					if (state == RD || state == DD) begin
						ref_in_shift_n        = ref_in_shift << 2;
						ref_in_shift_n[1:0]   = rbuffer[R_shift];
						
						query_in_shift_n  = query_in_shift;
						
						R_shift_nxt       = R_shift + 1;
						D_shift_nxt       = D_shift;
					end
					
					else begin
						ref_in_shift_n    = ref_in_shift;
						
						query_in_shift_n        = query_in_shift >> 2;
						query_in_shift_n[31:30] = qbuffer[D_shift + 15];
						
						R_shift_nxt       = R_shift;
						D_shift_nxt       = D_shift + 1;
					end
				end
			end
			
			REFF, RF: begin
				ref_in_shift_n          = ref_in_shift;
				query_in_shift_n        = query_in_shift >> 2;
				query_in_shift_n[31:30] = (D_shift >= 33)? 2'b00: qbuffer[D_shift + 15];
				
				R_shift_nxt       = R_shift;
				D_shift_nxt       = D_shift + 1;
			end
			
			QUEF, DF: begin
				ref_in_shift_n      = ref_in_shift << 2;
				ref_in_shift_n[1:0] = (R_shift >= 64)? 2'b00: rbuffer[R_shift];
				query_in_shift_n    = query_in_shift;
				
				R_shift_nxt       = R_shift + 1;
				D_shift_nxt       = D_shift;
			end
			
            default: begin
				ref_in_shift_n   = ref_in_shift;
				query_in_shift_n = query_in_shift; 
				
				R_shift_nxt       = R_shift;
				D_shift_nxt       = D_shift;
			end
        endcase
	end
	
	
	// assign S I D H input and state means previous two direction
	always @(*) begin
		case(state)
			IDLE: begin
				for (p = 0; p <= PE_num - 1; p = p+1) begin
					I_in[p]  = -128;
					D_in[p]  = -128;
					H_in0[p] = 0;
					H_in1[p] = 0;
					H_in2[p] = 0;
				end	
				for (m = 0; m <= 2*PE_num - 1; m = m+2) begin
					S[m]     = -1;
				end
			end
			
			INIT, INIF: begin
				for (p = 0; p <= PE_num - 1; p = p+1) begin
					I_in[p]  = (p == 0)? -128:  I[p-1];
					D_in[p]  = D[p];
					H_in0[p] = (p == 0)?    0: Hd[p-1];
					H_in1[p] = (p == 0)?    0:  H[p-1];
					H_in2[p] = H[p];
				end
				for (m = 0; m <= 2*PE_num - 1; m = m+2) begin
					if (m > (counter << 1) - 1) begin
						S[m]     = -1;  // PE valid signal 
					end
					else begin
						S[m]     = (ref_in_shift[m] == query_in_shift[m] && ref_in_shift[m+1] == query_in_shift[m+1]) ? 2 : -1;
					end
				end
			end
			
			RR, QUEF, DELA: begin
				for (p = 0; p <= PE_num - 1; p = p+1) begin
					I_in[p]  = (p == 0)? -128:  I[p-1];
					D_in[p]  = D[p];
					H_in0[p] = (p == 0)?    0: Hd[p-1];
					H_in1[p] = (p == 0)?    0:  H[p-1];
					H_in2[p] = H[p];
				end
				for (m = 0; m <= 2*PE_num - 1; m = m+2) begin
					S[m]     = (ref_in_shift[m] == query_in_shift[m] && ref_in_shift[m+1] == query_in_shift[m+1]) ? 2 : -1;
				end
			end
			
			
			DR, DF: begin
				for (p = 0; p <= PE_num - 1; p = p+1) begin
					I_in[p]  = (p == 0)? -128:  I[p-1];
					D_in[p]  = D[p];
					H_in0[p] = Hd[p];
					H_in1[p] = (p == 0)?    0:  H[p-1];
					H_in2[p] = H[p];
				end
				for (m = 0; m <= 2*PE_num - 1; m = m+2) begin
					S[m]     = (ref_in_shift[m] == query_in_shift[m] && ref_in_shift[m+1] == query_in_shift[m+1]) ? 2 : -1;
				end
			end
			
			RD, RF: begin
				for (p = 0; p <= PE_num - 1; p = p+1) begin
					I_in[p]  = I[p];
					D_in[p]  = (p == PE_num-1)? -128: D[p+1];
					H_in0[p] = Hd[p];
					H_in1[p] = H[p];
					H_in2[p] = (p == PE_num-1)?    0: H[p+1];
				end
				for (m = 0; m <= 2*PE_num - 1; m = m+2) begin
					S[m]     = (ref_in_shift[m] == query_in_shift[m] && ref_in_shift[m+1] == query_in_shift[m+1]) ? 2 : -1;
				end
			end
			
			DD, REFF: begin
				for (p = 0; p <= PE_num - 1; p = p+1) begin
					I_in[p]  = I[p];
					D_in[p]  = (p == PE_num-1)? -128: D[p+1];
					H_in0[p] = (p == PE_num-1)?    0:Hd[p+1];
					H_in1[p] = H[p];
					H_in2[p] = (p == PE_num-1)?    0: H[p+1];
				end
				for (m = 0; m <= 2*PE_num - 1; m = m+2) begin
					S[m]     = (ref_in_shift[m] == query_in_shift[m] && ref_in_shift[m+1] == query_in_shift[m+1]) ? 2 : -1;
				end
			end
			
            default:  begin
				for (p = 0; p <= PE_num - 1; p = p+1) begin
					I_in[p]  = -128;
					D_in[p]  = -128;
					H_in0[p] = 0;
					H_in1[p] = 0;
					H_in2[p] = 0;
				end	
				for (m = 0; m <= 2*PE_num - 1; m = m+2) begin
					S[m]     = -1;
				end
				
			end
        endcase
	end
	
	// I D H Hd table
	always @(*) begin	
		if (state == IDLE) begin
			for (m = 0; m <= PE_num-1; m = m+1) begin
				I_nxt[m] = -128;
				D_nxt[m] = -128;
				H_nxt[m] = 0;
				Hd_nxt[m]= 0;
			end
		end
		else if (state == DELA)begin			
			for (m = 0; m <= PE_num-1; m = m+1) begin
				I_nxt[m] = I[m];
				D_nxt[m] = D[m];
				H_nxt[m] = H[m];
				Hd_nxt[m]= Hd[m];
			end
        end
		else begin			
			for (m = 0; m <= PE_num-1; m = m+1) begin
				I_nxt[m] = I_out[m];
				D_nxt[m] = D_out[m];
				H_nxt[m] = H_out[m];
				Hd_nxt[m]= H[m];
			end
        end
	end
	
	// updating max and its positions
	always @(*) begin		
		case(state)
			IDLE: begin
				max_nxt  = 0;
				imax_nxt = 1;
				jmax_nxt = 1;
				MA_p_rn  = 0;
			end
			
			default: begin
				if( MA_out > max_cur) begin
					max_nxt  = MA_out;
					MA_p_rn  = MA_p;
					imax_nxt = R_shift;
					jmax_nxt = D_shift;
				end
				
				else begin
					max_nxt  = max_cur;
					MA_p_rn  = MA_p_r;
					imax_nxt = imax;
					jmax_nxt = jmax;
				end
			end
        endcase
	end
	
//------------------------------------------------------------------
// sequential part
//------------------------------------------------------------------
    
	always@(posedge clk or posedge reset) begin
		if(reset) begin
			state   <= IDLE;
			counter <= 0;
			max_cur <= 0;
			R_shift <= 0;
			D_shift <= 0;
			ref_in_shift   <= 0;
			query_in_shift <= 0;
			imax   <= 0;
			jmax   <= 0;
			MA_p_r <= 0;
			
			for (p = 0; p <= PE_num -1; p = p+1) begin
				I[p]  <= 0;
				D[p]  <= 0;
				H[p]  <= 0;
				Hd[p] <= 0;
			end
			for (m = 0; m <= 63; m = m+1) begin
				rbuffer[m]      <= 0;
			end
			for (n = 0; n <= 47; n = n+1) begin
				qbuffer[n]      <= 0;
			end
		end
		
		else begin
			state   <= state_nxt;
			counter <= counter_nxt;
			max_cur <= max_nxt;
			R_shift <= R_shift_nxt;
			D_shift <= D_shift_nxt;
			ref_in_shift    <= ref_in_shift_n;
			query_in_shift  <= query_in_shift_n;
			imax   <= imax_nxt;
			jmax   <= jmax_nxt;
			MA_p_r <= MA_p_rn;
			
			for (p = 0; p <= PE_num -1; p = p+1) begin
				I[p]  <= I_nxt[p];
				D[p]  <= D_nxt[p];
				H[p]  <= H_nxt[p];
				Hd[p] <= Hd_nxt[p];
			end
			for (m = 0; m <= 63; m = m+1) begin
				rbuffer[m]      <= rbuffer_n[m];
			end
			for (n = 0; n <= 47; n = n+1) begin
				qbuffer[n]      <= qbuffer_n[n];
			end
		end
    end
    
endmodule

//------------------------------------------------------------------
// submodule declaration
//------------------------------------------------------------------

// PE submodule
module PE(S, I_in, D_in, H_in0, H_in1, H_in2, I_out, D_out, H_out);
    
    input  signed [2:0] S;
	input  signed [7:0] I_in,D_in;
	input  signed [7:0] H_in0,H_in1,H_in2;
	output signed [7:0] I_out,D_out;
	output [6:0] H_out;
	
	wire signed [7:0] H_t1,H_t2;
	
	assign	I_out = (H_in1 - 2 >= I_in - 1)? (H_in1 - 2):(I_in - 1);
	
	assign	D_out = (H_in2 - 2 >= D_in - 1)? (H_in2 - 2):(D_in - 1);

	assign H_t1 = (H_in0 + S >= 0)?(H_in0 + S):0;
	assign H_t2 = (I_out >= D_out)?I_out:D_out;

	assign H_out = (H_t1 >= H_t2)? H_t1:H_t2;
endmodule

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

// max 16 submodule which compares H table in 16 PEs
module MA16(H0,H1,H2,H3,H4,H5,H6,H7,H8,H9,H10,H11,H12,H13,H14,H15,MA_p,MA_out,R_shift_sig, D_shift_sig);
	
	input  [6:0] H0,H1,H2,H3,H4,H5,H6,H7,H8,H9,H10,H11,H12,H13,H14,H15;
	output [3:0] MA_p;
	output [6:0] MA_out;
	output R_shift_sig, D_shift_sig;
	
	
	wire [6:0] H_sub0,H_sub1,H_sub2,H_sub3;
	wire [1:0] MA_p_sub_0,MA_p_sub_1,MA_p_sub_2,MA_p_sub_3,MA_p_temp;
	reg  [3:0] MA_p_final;
	
	assign MA_p = MA_p_final;
	
	assign R_shift_sig = (H0+H1+H2+H3+H4+H5+H6+H7 - 2 > H8+H9+H10+H11+H12+H13+H14+H15);
	assign D_shift_sig = (H0+H1+H2+H3+H4+H5+H6+H7 < H8+H9+H10+H11+H12+H13+H14+H15 - 2);
	
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

