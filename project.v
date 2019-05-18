// Part 2 skeleton

module project
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
		KEY,							// On Board Keys
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,						//	VGA Blue[9:0]
		LEDR,
		GPIO_0,
		HEX0,
		HEX1,
		SW,
		PS2_CLK,
		PS2_DAT
	);

	input			CLOCK_50;				//	50 MHz
	input	[3:0]	KEY;
	input 	[9:0] SW;	
	input [32:24] GPIO_0;
	// Declare your inputs and outputs here
	
	
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[7:0]	VGA_R;   				//	VGA Red[7:0] Changed from 10 to 8-bit DAC
	output	[7:0]	VGA_G;	 				//	VGA Green[7:0]
	output	[7:0]	VGA_B;   				//	VGA Blue[7:0]
	output	[9:0] LEDR;
	output 	[6:0] HEX0, HEX1;
	
 wire left_key, right_key, space_key, enter_key;
 inout PS2_CLK;
 inout PS2_DAT;
	wire resetn;
	assign resetn = SW[1];
		
//	assign L1 = LEDR [8];	
//	assign M1 = LEDR [7];	
//	assign R1 = LEDR [6];	
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [8:0] colour,colour0,colourS1;
	wire [8:0] colourS;
	wire [8:0] colour1;
	wire [8:0] colour2;
	wire [8:0] colour3;
	wire [8:0] colourR;
	wire [8:0] colourL;
	wire [8:0] x,x1,pos_x,pos_x2,xout9,colour9;
	wire [7:0] y,y1, pos_y,pos_y2,yout9;
	wire [8:0] xS,xS1;
	wire [7:0] yS,yS1;
	wire writeEn,leftaz,rightaz ;
	wire write,sel,sel1,sel2,mv1,mv,mv2,en9,done9;
	wire counter;
	wire [8:0] xout;
	wire [7:0] yout;
	wire [8:0] xout2;
	wire [7:0] yout2;
	wire [8:0] xout3;
	wire [8:0] xoutR;
	wire [8:0] xoutL;
	wire [7:0] yout3, youtR,youtL;
	wire bg,drawS,signal,drawl,drawr,left,hit1,hit2;
	wire en1,en2,on,done,en4,en5,ld_left,ld_right,L1,R1,M1,hit;
	wire L2,R2,M2,L3,R3,M3,select;
	wire [3:0] h1,h2;
	wire [6:0] bin;
	wire [2:0] roll1,roll2;
	wire clk_out;
	wire  counter_blk, done_erase, print_blk, draw, erase, move,bot1,bot2;
	wire  counter_blk2, done_erase2, print_blk2, draw2, erase2, move2;
	assign writeEn = (bot1||bot2) ? 1'b0 : 1'b1;
	assign x = (!signal) ? x1 : (select ? xS1 : xS);
	assign y = (!signal) ? y1 :(select ? yS1 : yS);
	assign colour = (!signal) ? colour0 : (select ? colourS1 : colourS);
	//assign leftaz = 
	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "320x240";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 3;
		defparam VGA.BACKGROUND_IMAGE = "newbg.mif";
		
		keyboard_interface_test_mode0 k1(
    .CLOCK_50 (CLOCK_50),
	 .resetn(resetn),
	 
	 .PS2_CLK(PS2_CLK),
	 .PS2_DAT(PS2_DAT) ,
	 
	 .left_key(left_key),
	 .right_key(right_key),
	 .space_key(space_key),
	 .enter_key(enter_key),
	 );
		
		FSM_control u3(.clock(CLOCK_50),
				.plot(space_key),
				.xout(xout),
				.yout(yout),
				.xout2(xout2),
				.xout3(xout3),
				.xoutR(xoutR),
				.xoutL(xoutL),
				.yout2(yout2),
				.yout3(yout3),
				.youtR(youtR),
				.youtL(youtL),
				.colour1(colour1),
				.colour2(colour2),
				.colour3(colour3),
				.colourR(colourR),
				.colourL(colourL),
				.resetn(resetn),
				.go(enter_key), 
				.done(bg),
				.done2(drawS),
				.done3(done),
				.doneR(drawr),
				.doneL(drawl),
				.x(x1),
				.y(y1),
				.en1(en1),
				.en2(en2),
				.en3(on),
				.en4(en5),
				.enable(write),
				.colour(colour0),
				.left(left_key),
				.right(right_key),
				.start(signal),
				.init(SW[2]),
				.ld_left(ld_left),
				.ld_right(ld_right),
				.hit1(hit1),
				.hit2(hit2),
				.done9(done9),
				.xout9(xout9),
				.yout9(yout9),
				.colour9 (colour9),
				.en9(en9)
    				);

				  
		man2 u2 (         .inx(9'b0),
				  .iny(8'b0),
				  .do(en1),
				  .reset(resetn),
				  .clock(CLOCK_50),
				  .xout(xout),
				  .yout(yout),
				  .col(colour1),
				  .done(bg));
		complete e2 (         .inx(9'b0),
				  .iny(8'b0),
				  .do(en9),
				  .reset(resetn),
				  .clock(CLOCK_50),
				  .xout(xout9),
				  .yout(yout9),
				  .col(colour9),
				  .done(done9));		  

		man u1 (	.left(ld_left),
					.right(ld_right),
				  .do(en2),
				  .reset(resetn),
				  .clock(CLOCK_50),
				  .xout(xout2),
				  .yout(yout2),
				  .col(colour2),
				  .done2(drawS),
				  .L1(L1),
				  .R1(R1),
				  .M1(M1)
				  
				  );
				  
		dice rl(.clock(CLOCK_50), .readin(bot1),.roll(roll1));
		dice r2(.clock(CLOCK_50), .readin(bot2),.roll(roll2));
		
		hitdetection hi1(.clock(CLOCK_50),.left(L1),.middle(M1),.right(R1),
							.inx1(pos_x),.iny1(pos_y),.hit(hit2),.bot(bot1));
							
		hitdetection hi2(.clock(CLOCK_50),.left(L1),.middle(M1),.right(R1),
							.inx1(pos_x2),.iny1(pos_y2),.hit(hit1),.bot(bot2));
				  
		monitor u6(.clock(CLOCK_50),.signal(GPIO_0[32]),.out(LEDR[8]),.bin1(bin));
		Binary_Decoder b1(.inBin(bin),.H1(h1),.H2(h2));
		HEX_Decoder u4 (h1, HEX0);
		HEX_Decoder u5 (h2, HEX1);
		erase s1(.clock(CLOCK_50),.on(on),.inx(9'd60),.iny(8'd190),.done_erase(done),
			.xout(xout3),.yout(yout3),.colour(colour3),.reset(resetn));
		
		FSM2 c0(
				.clock(CLOCK_50),
				.resetn(resetn),
				.go(signal),
				.counter_blk(counter_blk),
				.done_erase(done_erase),
				.select(select),
				.draw(draw),
				.move(move),
				.erase(erase),
				.print_blk(print_blk),
				.counter_blk2(counter_blk2),
				.done_erase2(done_erase2),
				.draw2(draw2),
				.move2(move2),
				.erase2(erase2),
				.print_blk2(print_blk2)
				);
	
	datapath d1(
			.clock(CLOCK_50),
			.data(SW[9:7]),
			.resetn(resetn),
			.draw(draw),
			.move(move),
			.erase(erase),
			.print_blk(print_blk),			
			.counter_blk(counter_blk),
			.done_erase(done_erase),
			.pos_x(pos_x),.pos_y(pos_y),
			.X(xS),
			.Y(yS),
			.colour(colourS),
			.rand(roll1),.bin(bot1)
			);
	datapath2 de2(
			.clock(CLOCK_50),
			.data(SW[6:4]),
			.resetn(resetn),
			.draw(draw2),
			.move(move2),
			.erase(erase2),
			.print_blk(print_blk2),			
			.counter_blk(counter_blk2),
			.done_erase(done_erase2),
			.pos_x(pos_x2),.pos_y(pos_y2),
			.X(xS1),
			.Y(yS1),
			.colour(colourS1),
			.rand(roll2),.bin(bot2)
			);
	
			
endmodule
//
module man (left,right,reset,clock,do,xout,yout,col,done2,L1,R1,M1);
				input do,left,right;
				input reset;
				input clock;
				reg [8:0] x;
				reg [7:0] y;
				reg [8:0] x_ini;
				output reg [8:0] col;
				output reg done2,L1,R1,M1;
	
	wire [8:0] colour;
	reg [5:0] countx;
	reg [5:0] county;
	//reg [5:0] count;
	reg [11:0] memory_address;
	output [8:0] xout;
	output [7:0] yout;
	

	
	always @(posedge clock)
		if (!reset)
		begin
			col<=0;
			countx<=0;
			county<=0;
			memory_address<=0;
			done2<=0;
			L1 <= 0;
			R1 <= 0;
			M1 <= 0;
			
		end
		else if(do)
			begin
			
			if (done2) begin
				col<=0;
				countx<=0;
				county<=0;
				memory_address<=0;
				done2<=0;
				L1 <= 0;
				R1 <= 0;
				M1 <= 0;
			
			end
			if (x_ini == 9'd55) begin
					L1 <= 1'b1;
				if (left) begin
					x <= 9'd55;
					y <= 8'd190;
					
					
				end
				else if (right) begin
					x <= 9'd135;
					y <= 8'd190;
					
					
				end
			end
			else if (x_ini == 9'd135) begin
				M1 <= 1'b1;
				if (left) begin
					x <= 9'd55;
					y <= 8'd190;
					
					
				end
				else if (right) begin
					x <= 9'd195;
					y <= 8'd190;
					
					
				end
			end
			else if (x_ini == 9'd195) begin
				R1 <= 1'b1;
				if (left) begin
					x <= 9'd135;
					y <= 8'd190;
					
					
				end
				else if (right) begin
					x <= 9'd195;
					y <= 8'd190;
					
					
				end
			end
			else begin
				
					x <= 9'd135;
					x_ini <= 9'd135;
					y <= 8'd190;
					M1 <= 1'b1;
				
				
				end
			
			col <=colour;
		   memory_address<= memory_address +1;
     
			if (countx < 6'b110001)
			begin
				countx <= countx +1;
			end
			if (countx>= 6'b110001 && county < 6'b110001)
			begin
				county <= county +1;
				countx<= 0;
			end
			if ({countx, county} == 12'b110001110000)
			begin
			memory_address<= 0;
			county<=0;
			done2 <= 1;
			x_ini <= x;
			end
			
		end
		
		assign xout = x + countx;
		assign yout = y + county;
		
	manleft u1(.address(memory_address), .clock(clock), .data(9'b0), 
						.wren(1'b0),.q(colour));
endmodule


module complete (inx,iny,reset,clock,do,xout,yout,col,done);
				input do;
				input reset;
				input clock;
				input [8:0] inx;
				input [7:0] iny;
				reg [8:0] x;
				reg [7:0] y;
				output reg [8:0] col;
				output reg done;
	
	wire [8:0] colour;
	reg [8:0] countx;
	reg [7:0] county;
	//reg [5:0] count;
	reg [16:0] memory_address;
	output [8:0] xout;
	output [7:0] yout;
	
	
		gameover u2(.address(memory_address), .clock(clock), .data(9'b0), 
						.wren(1'b0),.q(colour));
	
	always @(posedge clock)
		if (!reset)
		begin
			x<=0;
			y<=0;
			col<=0;
			countx<=0;
			county<=0;
			memory_address<=0;
			done<=0;
			x<=inx;
			y<=iny;
		end
		else if(do)
			begin
			if (done) begin
				col<=0;
				countx<=0;
				county<=0;
				memory_address<=0;
				done<=0;
			end
			col <=colour;
			memory_address<=memory_address+1;
			if (countx < 9'b100111111)
			begin
				countx <= countx +1;
			end
			if (countx>= 9'b100111111 && county < 8'b11101111)
			begin
				county <= county +1;
				countx<= 0;
			end
			if ({countx, county} == 17'b10011111011101110)
			begin
			memory_address<= 0;
			county<=0;
			done <= 1;
			end
			
			
		end
		
	
		
		assign xout = x + countx;
		assign yout = y + county;
		
	
endmodule		

module man2 (inx,iny,reset,clock,do,xout,yout,col,done);
				input do;
				input reset;
				input clock;
				input [8:0] inx;
				input [7:0] iny;
				reg [8:0] x;
				reg [7:0] y;
				output reg [8:0] col;
				output reg done;
	
	wire [8:0] colour;
	reg [8:0] countx;
	reg [7:0] county;
	//reg [5:0] count;
	reg [16:0] memory_address;
	output [8:0] xout;
	output [7:0] yout;
	
	
		bg2 u2(.address(memory_address), .clock(clock), .data(9'b0), 
						.wren(1'b0),.q(colour));
	
	always @(posedge clock)
		if (!reset)
		begin
			x<=0;
			y<=0;
			col<=0;
			countx<=0;
			county<=0;
			memory_address<=0;
			done<=0;
			x<=inx;
			y<=iny;
		end
		else if(do)
			begin
			if (done) begin
				col<=0;
				countx<=0;
				county<=0;
				memory_address<=0;
				done<=0;
			end
			col <=colour;
			memory_address<=memory_address+1;
			if (countx < 9'b100111111)
			begin
				countx <= countx +1;
			end
			if (countx>= 9'b100111111 && county < 8'b11101111)
			begin
				county <= county +1;
				countx<= 0;
			end
			if ({countx, county} == 17'b10011111011101110)
			begin
			memory_address<= 0;
			county<=0;
			done <= 1;
			end
			
			
		end
		
	
		
		assign xout = x + countx;
		assign yout = y + county;
		
	
endmodule

module FSM_control(clock, plot, xout, yout, xout2, yout2, resetn, go, done, done2, colour1, colour2,x, 
			y , enable, en1, en2,colour,left,right,done3,xout3,yout3,colour3,en3,start,colourR,doneR,doneL,
			colourL,xoutL,xoutR,youtR,youtL,en4,en5,init,ld_left,ld_right,hit1,hit2,en9,xout9,yout9,done9,colour9
    );
	 input clock,plot,resetn,go,done,done2, left, right, done3,doneR,doneL,init,hit1,hit2,done9;
	 input [8:0] colour1, colour2, colour3, colourR, colourL,colour9;
	 input [8:0] xout, xout2, xout3, xoutR, xoutL,xout9;
	 input [7:0] yout, yout2, yout3, youtR, youtL,yout9;
	 output reg [8:0] x, colour;
	 output reg [7:0] y;
	 output reg enable , en1, en2, en3,en4,en5, en9,start,ld_left,ld_right;
	 
	wire counter;
	wire [25:0] w1;
	assign counter = (w1 == 26'b0)?1:0;
	
	rate_divider a4Hz(
				.clock(clock),
				.enable(1'b1),
				.load(26'd9999999),
				.reset(~resetn),
				.count(w1)
				);

    reg [5:0] current_state, next_state; 
	
    localparam  S_WAIT  	  = 5'd0,
				BG_LOAD        = 5'd1,
				S_LOAD		  = 5'd2,
				M_WAIT			= 5'd3,
			   S_hold_wait_left = 5'd4,
    			S_hold_wait_right = 5'd5,
				M_LEFT			=5'd6,
				MOVED_L			=5'd7,
				M_RIGHT			=5'd8,
				wait1 = 5'd9,
				wait2 = 5'd10,
				MOVED_R			=5'd11,
				GameOver = 5'd12;
				
				

    // Next state logic aka our state table
    always @(*)
    begin: state_table 
            case (current_state)
                S_WAIT: next_state = go ? BG_LOAD :  S_WAIT;
                BG_LOAD: next_state = done ? S_LOAD : BG_LOAD; // Loop in current state until go signal goes low
                S_LOAD: next_state = plot ? M_WAIT : S_LOAD;
					 M_WAIT: next_state = (hit1||hit2) ? GameOver:(left ? S_hold_wait_left : (right ? S_hold_wait_right: M_WAIT));
					 S_hold_wait_left: next_state = left ? S_hold_wait_left : M_LEFT;
   				S_hold_wait_right: next_state = right ? S_hold_wait_right : M_RIGHT;
					 M_LEFT: next_state = done3 ? MOVED_L : M_LEFT;
					 MOVED_L: next_state =  done2 ? M_WAIT : MOVED_L;
					 M_RIGHT: next_state = done3 ? MOVED_R : M_RIGHT;
					 MOVED_R: next_state = done2 ? M_WAIT : MOVED_R;
					 GameOver : next_state = done9 ? S_WAIT : GameOver;
					
					 
		default:     next_state = S_WAIT;
        endcase
    end // state_table
   

    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
        // By default make all our signals 0 to avoid latches.
        x= 9'b0;
		  y= 8'b0;
	     en1= 1'b0;
	     en2= 1'b0;
	     en3= 1'b0;
		  en4=1'b0;
		  en5 =1'b0;
	    start = 1'b0;
	     enable = 1'b0;
	     colour = 9'b0;
		 ld_left = 1'b0;
		 ld_right = 1'b0;
				en9 = 1'b0;

        case (current_state)
			BG_LOAD: begin
				     en1 <= 1'b1;	
				     x <= xout;
				     y <= yout;
				  colour <= colour1;
				     enable <= 1'b1;
					  
				     
					  end		  
         S_LOAD:	begin
					
					en2 <= 1'b1; 
					  x <= xout2;
					  y <= yout2;
				     colour <= colour2;
				     enable <= 1'b1;
					  
					  end
			M_WAIT: begin
					start <= 1'b1;
					
					end
			
			S_hold_wait_left: begin
					start <= 1'b1;
					
					end
			S_hold_wait_right: begin
					start <= 1'b1;
					
					end
			 M_LEFT:	begin
					start = 1'b0;
					en3 <= 1'b1;	
				     x <= xout3;
				     y <= yout3;
				  colour <= colour3;
				     enable <= 1'b1;
					  end
			 MOVED_L:	begin
					start = 1'b0;
					en2 <= 1'b1; 
					ld_left <= 1'b1;
					  x <= xout2;
					  y <= yout2;
				     colour <= colour2;
				     enable <= 1'b1;
					  
					  end	
			M_RIGHT:	begin
					start <= 1'b0;
					en3 <= 1'b1;	
				     x <= xout3;
				     y <= yout3;
				  colour <= colour3;
				     enable <= 1'b1;
					  
					  end
					  
			MOVED_R:	begin
					start <= 1'b0;
					ld_right <= 1'b1;
					en2 <= 1'b1;	
				     x <= xout2;
				     y <= yout2;
				  colour <= colour2;
				     enable <= 1'b1;
					 
					  end
			  GameOver: begin
				     en9 <= 1'b1;	
				     x <= xout9;
				     y <= yout9;
				  colour <= colour9;
				     enable <= 1'b1;
			
					end


        endcase
    end // enable_signals
   
    // current_state registers
    always@(posedge clock)
    begin: state_FFs
        if(!resetn)
            current_state <= S_WAIT;
        else
            current_state <= next_state;
    end // state_FFS
endmodule

module erase (clock,on,inx,iny,done_erase,xout,yout,colour,reset);
	input on,clock,reset;
	input [8:0] inx;
	input [7:0] iny;
	wire [8:0] colour1;
	reg [7:0] countx;
	reg [6:0] county;
	//reg [5:0] count;
	reg [13:0] memory_address;
	output [8:0] xout;
	output [7:0] yout;
	reg [8:0] x;
	reg [7:0] y;
	output reg done_erase;
	output reg [8:0] colour;
	
		erase1 u2(.address(memory_address), .clock(clock), .data(9'b0), 
						.wren(1'b0),.q(colour1));
	
	always @(posedge clock)
		if (!reset)
		begin
			x<=0;
			y<=0;
			colour<=0;
			countx<=0;
			county<=0;
			memory_address<=0;
			done_erase<=0;
			x<=inx;
			y<=iny;
		end
		else if(on)
			begin
			if (done_erase) begin
				colour<=0;
				countx<=0;
				county<=0;
				memory_address<=0;
				done_erase<=0;
			end
			colour <=colour1;
			memory_address<=memory_address+1;
			if (countx < 8'b11000001)
			begin
				countx <= countx +1;
			end
			if (countx>= 8'b11000001 && county < 7'b1000011)
			begin
				county <= county +1;
				countx<= 0;
			end
			if ({countx, county} == 15'b110000011000010 )
			begin
			memory_address<= 0;
			county<=0;
			done_erase <= 1;
			end
			
			
		end
		
	
		
		assign xout = x + countx;
		assign yout = y + county;
endmodule





module FSM2(
    input clock, resetn, go, counter_blk, done_erase,counter_blk2, done_erase2,

    output reg erase, move, draw, print_blk, erase2, move2, draw2, print_blk2,select
    );

    reg [5:0] current_state, next_state; 
	
    localparam  S_WAIT  	  = 5'd0,
				S_ERASE       = 5'd1,
				S_MOVE	 	  = 5'd2,
				S_MOVE_PAUSE  = 5'd3,
				S_DRAW 	      = 5'd4,
				S_ERASE2       = 5'd5,
				S_MOVE2	 	  = 5'd6,
				S_MOVE_PAUSE2  = 5'd7,
				S_DRAW2 	      = 5'd8;
	
	wire [25:0] load;
	assign load = (counter3) ? 26'd99999:((counter2) ? 26'd199999 : 26'd29999);
	wire counter;
	wire [25:0] w1;
	assign counter = (w1 == 26'b0)?1:0;
	
	rate_divider u4Hz(
				.clock(clock),
				.enable(1'b1),
				.load(load),
				.reset(~resetn),
				.count(w1)
				);
	wire counter2;
	wire [25:0] w2;
	assign counter2 = (w2 == 26'b0)?1:0;
	
	rate_divider u24Hz(
				.clock(clock),
				.enable(1'b1),
				.load(26'd399999),
				.reset(~resetn),
				.count(w2)
				);
	wire counter3;
	wire [25:0] w3;
	assign counter3 = (w3 == 26'b0)?1:0;
	
	rate_divider u34Hz(
				.clock(clock),
				.enable(1'b1),
				.load(26'd499999),
				.reset(~resetn),
				.count(w3)
				);
	
 
    // Next state logic aka our state table
    always @(*)
    begin: state_table 
            case (current_state)
                S_WAIT: next_state = go ? S_ERASE : S_WAIT;
                S_ERASE: next_state = (done_erase && go) ? S_MOVE : S_ERASE;
					 S_MOVE: next_state = S_MOVE_PAUSE;
					 S_MOVE_PAUSE: next_state = S_DRAW;
					 S_DRAW: next_state = counter ? S_ERASE2 : S_DRAW;
					 S_ERASE2: next_state = (done_erase2) ? S_MOVE2 : S_ERASE2;
					 S_MOVE2: next_state = S_MOVE_PAUSE2;
					 S_MOVE_PAUSE2: next_state = S_DRAW2;
					 S_DRAW2: next_state = counter ? S_ERASE : S_DRAW2;
		default:     next_state = S_WAIT;
        endcase
    end // state_table
   

    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
        // By default make all our signals 0 to avoid latches.
		erase = 1'b0;
		move = 1'b0;
		draw = 1'b0;
		print_blk = 1'b0;
		erase2 = 1'b0;
		move2 = 1'b0;
		draw2 = 1'b0;
		print_blk2 = 1'b0;
		select = 1'b0;

        case (current_state)
			S_ERASE: begin erase = 1'b1;
									select = 1'b0; end
			S_MOVE: begin move = 1'b1;
									select = 1'b0; end
            S_DRAW:  begin draw = 1'b1;
									select = 1'b0; end
			S_ERASE2: begin erase2 = 1'b1;
									select = 1'b1; end
			S_MOVE2: begin move2 = 1'b1;
									select = 1'b1; end
				S_DRAW2:  begin draw2 = 1'b1;
									select = 1'b1; end
			
        endcase
    end // enable_signals
   
    // current_state registers
    always@(posedge clock)
    begin: state_FFs
        if(!resetn)
            current_state <= S_WAIT;
        else
            current_state <= next_state;
    end // state_FFS
endmodule



module datapath(
			input clock, erase, move, draw, print_blk, resetn,bin,
			input [2:0] data,rand,
			output reg counter_blk, done_erase,
			output reg [8:0] X,
			output reg [7:0] Y,
			output reg [8:0] colour,
			output reg [8:0] pos_x,
			output reg [7:0] pos_y 
			);
		reg [8:0] pos_xr;
	 reg [8:0] pos_x1 = 9'd135;
	  reg [7:0] pos_y1 = 8'd0;
	reg done_draw;
	wire [8:0] col;
	wire [8:0] col2;
	reg [4:0] countx;
	reg [5:0] county;
	//reg [5:0] count;
	reg [10:0] memory_address;
	reg [10:0] memory_address2;
	
	zombieL C1 (.address(memory_address), .clock(clock), .data(9'b0), .wren(1'b0), .q(col));
	erase2 Cg1 (.address(memory_address2), .clock(clock), .data(9'b0), .wren(1'b0), .q(col2));
	
	always @(posedge clock)
	begin
		if((rand==3'd1)&&!move) begin
			pos_xr <= 9'd55;
			pos_x1 <= pos_xr;
			//pos_y1 <= 8'd0;
		end
		else if((rand==3'd2)&&!move) begin
			pos_xr <= 9'd195;
			pos_x1 <= pos_xr;
			//pos_y1 <= 8'd0;
		end
		else if((rand==3'd3)&&!move) begin
			pos_xr <= 9'd135;
			pos_x1 <= pos_xr;
			//pos_y1 <= 8'd0;
		end
		else if((rand==3'd4)&&!move) begin
			pos_xr <= 9'd55;
			pos_x1 <= pos_xr;
			//pos_y1 <= 8'd0;
		end
		else if((rand==3'd6) && !move) begin
			pos_xr <= 9'd195;
			pos_x1 <= pos_xr;
			//pos_y1 <= 8'd0;
		end
		
		else if(move) begin
			pos_y1 <= pos_y1 + 1;
			pos_y <= pos_y1;
			pos_x <= pos_x1;
			//pos_y1 <= 8'd0;
//			if (bin) begin
//				pos_y1 <= 8'd0;
//			end
		end
		
		
	end
	
	
	integer i, j;
	
	always @(posedge clock)
	begin	
		if(!resetn) begin
			X <= 9'b0;
			Y <= 8'b0;
		end
	
		else if(erase) begin
			colour <=col2;
		   memory_address2<= memory_address2 +1;
			X <= pos_x1 + countx;
			Y <=  pos_y1 + county;
			if (countx < 5'b11110)
			begin
				countx <= countx +1;
			end
			if (countx>= 5'b11110 && county < 6'b110001)
			begin
				county <= county +1;
				countx<= 0;
			end
			if ({countx, county} == 11'b11110110000)
			begin
			memory_address2<= 0;
			county<=0;
			done_erase <= 1;end
		end
		
		else if(draw && !done_draw) begin
			colour <=col;
		   memory_address<= memory_address +1;
			X <= pos_x1 + countx;
			Y <=  pos_y1 + county;
			if (countx < 5'b11110)
			begin
				countx <= countx +1;
			end
			if (countx>= 5'b11110 && county < 6'b110001)
			begin
				county <= county +1;
				countx<= 0;
			end
			if ({countx, county} == 11'b11110110000)
			begin
			memory_address<= 0;
			county<=0;
			done_draw <= 1;end
		end
		

		
		else begin
			counter_blk <= 0;
			done_erase <= 0;
			if(!draw) done_draw <= 0;
			i <= 0;
			j <= 0;
			X <= pos_x1;
			Y <= pos_y1;
		end
	end
	
endmodule

module datapath2(
			input clock, erase, move, draw, print_blk, resetn,bin,
			input [2:0] data,rand,
			output reg counter_blk, done_erase,
			output reg [8:0] X,
			output reg [7:0] Y,
			output reg [8:0] colour,
			output reg [8:0] pos_x,
			output reg [7:0] pos_y 
			);
		reg [8:0] pos_xr;
	 reg [8:0] pos_x1 = 9'd135;
	  reg [7:0] pos_y1 = 8'd0;
	reg done_draw;
	wire [8:0] col;
	wire [8:0] col2;
	reg [4:0] countx;
	reg [5:0] county;
	//reg [5:0] count;
	reg [10:0] memory_address;
	reg [10:0] memory_address2;
	
	zombieL Cg1 (.address(memory_address), .clock(clock), .data(9'b0), .wren(1'b0), .q(col));
	erase2 dc1 (.address(memory_address2), .clock(clock), .data(9'b0), .wren(1'b0), .q(col2));
	
	always @(posedge clock)
	begin
		if((rand==3'd1)&&!move) begin
			pos_xr <= 9'd200;
			pos_x1 <= pos_xr;
			//pos_y1 <= 8'd0;
		end
		else if((rand==3'd2)&&!move) begin
			pos_xr <= 9'd135;
			pos_x1 <= pos_xr;
			//pos_y1 <= 8'd0;
		end
		else if((rand==3'd3)&&!move) begin
			pos_xr <= 9'd55;
			pos_x1 <= pos_xr;
			//pos_y1 <= 8'd0;
		end
		else if((rand==3'd4)&&!move) begin
			pos_xr <= 9'd195;
			pos_x1 <= pos_xr;
			//pos_y1 <= 8'd0;
		end
		else if((rand==3'd6) && !move) begin
			pos_xr <= 9'd55;
			pos_x1 <= pos_xr;
			//pos_y1 <= 8'd0;
		end
		
		else if(move) begin
			pos_y1 <= pos_y1 + 1;
			pos_y <= pos_y1;
			pos_x <= pos_x1;
			//pos_y1 <= 8'd0;
//			if (bin) begin
//				pos_y1 <= 8'd0;
//			end
		end
		
		
	end
	
	
	integer i, j;
	
	always @(posedge clock)
	begin	
		if(!resetn) begin
			X <= 9'b0;
			Y <= 8'b0;
		end
	
		else if(erase) begin
			colour <=col2;
		   memory_address2<= memory_address2 +1;
			X <= pos_x1 + countx;
			Y <=  pos_y1 + county;
			if (countx < 5'b11110)
			begin
				countx <= countx +1;
			end
			if (countx>= 5'b11110 && county < 6'b110001)
			begin
				county <= county +1;
				countx<= 0;
			end
			if ({countx, county} == 11'b11110110000)
			begin
			memory_address2<= 0;
			county<=0;
			done_erase <= 1;end
		end
		
		else if(draw && !done_draw) begin
			colour <=col;
		   memory_address<= memory_address +1;
			X <= pos_x1 + countx;
			Y <=  pos_y1 + county;
			if (countx < 5'b11110)
			begin
				countx <= countx +1;
			end
			if (countx>= 5'b11110 && county < 6'b110001)
			begin
				county <= county +1;
				countx<= 0;
			end
			if ({countx, county} == 11'b11110110000)
			begin
			memory_address<= 0;
			county<=0;
			done_draw <= 1;end
		end
		

		
		else begin
			counter_blk <= 0;
			done_erase <= 0;
			if(!draw) done_draw <= 0;
			i <= 0;
			j <= 0;
			X <= pos_x1;
			Y <= pos_y1;
		end
	end
	
endmodule


module rate_divider(clock, enable, load, reset, count);

	input [25:0] load;
	input clock, enable, reset;
	output reg [25:0] count;
	
	always @(posedge clock, posedge reset)
	begin
		if(reset == 1'b1)
			count <= load;
		else if(count == 26'b0 && enable == 1'b1)
			count <= load;
		else if(enable == 1'b1)
			count <= count - 1;
	end
endmodule

module rate_divider2(clock, enable, load, reset, count);

	input [27:0] load;
	input clock, enable, reset;
	output reg [27:0] count;
	
	always @(posedge clock, posedge reset)
	begin
		if(reset == 1'b1)
			count <= load;
		else if(count == 28'b0 && enable == 1'b1)
			count <= load;
		else if(enable == 1'b1)
			count <= count - 1;
	end
endmodule



module monitor (clock,signal,out,bin1);
	input clock, signal;
	output reg  out;
	reg [32:0] countT = 33'b0;
	output reg [6:0]	bin1;
	reg [6:0]	beat1 = 7'b0;
	
	
	always @(posedge clock)
	begin
		if(countT == 33'b10110010110100000101111000000000)
			begin
			bin1 <= beat1;
		
			
			end
		else if(countT > 33'b10110010110100000101111000000000)
			begin
			
			beat1 <= 7'b0;
			
			end
		else
			begin
				if (signal == 1)
				begin
				out <= signal;
				beat1<=beat1+1;
				
				countT <= countT +1;
				end
				else if (signal ==0)
				begin
				countT <= countT +1;
				out <= signal;
				end
			end
//		countT = 33'b0;
//		hex = beats;
//		end
	end
endmodule

module HEX_Decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule

module Binary_Decoder(inBin,H1,H2);
    input [6:0] inBin;
    output reg [3:0] H1, H2 ;
   
    always @(*)
        case (inBin)
            7'd0: begin
			H1= 4'h0;
			H2= 4'h0;
			end
	   7'd1: begin
			H1= 4'h1;
			H2= 4'h0;
			end
		7'd3: begin 
		
				H1= 4'h3;
				H2= 4'h0;
				end
				
7'd4: begin 
		
				H1= 4'h4;
				H2= 4'h0;
				end
				
7'd5: begin 
			
				H1= 4'h5;
				H2= 4'h0;
				end
				
7'd6: begin 
			
				H1= 4'h6;
				H2= 4'h0;
				end
				
7'd7: begin 
			
				H1= 4'h7;
				H2= 4'h0;
				end

7'd8: begin 
			
				H1= 4'h8;
				H2= 4'h0;
				end
				
7'd9: begin 
			
				H1= 4'h9;
				H2= 4'h0;
				end
				
7'd10: begin 
		
				H1= 4'h0;
				H2= 4'h1;
				end
				
7'd11: begin 
		
				H1= 4'h1;
				H2= 4'h1;
				end				
				
				
7'd12: begin 
		
				H1= 4'h2;
				H2= 4'h1;
				end				
				
				
7'd13: begin 
		
				H1= 4'h3;
				H2= 4'h1;
				end		
		

7'd14: begin 
		
				H1= 4'h4;
				H2= 4'h1;
				end
		
7'd15: begin 
	
				H1= 4'h5;
				H2= 4'h1;
				end
		
7'd16: begin 
		
				H1= 4'h6;
				H2= 4'h1;
				end
		
7'd17: begin 
		
				H1= 4'h7;
				H2= 4'h1;
				end
		

7'd18: begin 
		
				H1= 4'h8;
				H2= 4'h1;
				end
		
7'd19: begin 
		
				H1= 4'h9;
				H2= 4'h1;
				end
		
7'd20: begin 
		
				H1= 4'h0;
				H2= 4'h2;
				end		
					
				
7'd21: begin 
		
				H1= 4'h1;
				H2= 4'h2;
				end
								
7'd22: begin 
		
				H1= 4'h2;
				H2= 4'h2;
				end	
				
7'd23: begin 
		
				H1= 4'h3;
				H2= 4'h2;
				end		

7'd24: begin 
		
				H1= 4'h4;
				H2= 4'h2;
				end
		
7'd25: begin 
		
				H1= 4'h5;
				H2= 4'h2;
				end		
				
7'd26: begin 
		
				H1= 4'h6;
				H2= 4'h2;
				end		
				
		
7'd27: begin 
		
				H1= 4'h7;
				H2= 4'h2;
				end		
				
		
7'd28: begin 
		
				H1= 4'h8;
				H2= 4'h2;
				end		
				
		
7'd29: begin 
		
				H1= 4'h9;
				H2= 4'h2;
				end		
				
		
7'd30: begin 
		
				H1= 4'h0;
				H2= 4'h3;
				end		
				
		
7'd31: begin 
		
				H1= 4'h1;
				H2= 4'h3;
				end		
				
		
7'd32: begin 
		
				H1= 4'h2;
				H2= 4'h3;
				end		
				
		
7'd33: begin 
		
				H1= 4'h3;
				H2= 4'h3;
				end		
				
		
7'd34: begin 
		
				H1= 4'h4;
				H2= 4'h3;
				end		
				
		
7'd35: begin 
		
				H1= 4'h5;
				H2= 4'h3;
				end
		
7'd36: begin 
		
				H1= 4'h6;
				H2= 4'h3;
				end
		
7'd37: begin 
		
				H1= 4'h7;
				H2= 4'h3;
				end
		
7'd38: begin 
		
				H1= 4'h8;
				H2= 4'h3;
				end
		
7'd39: begin 
		
				H1= 4'h9;
				H2= 4'h3;
				end
		
7'd40: begin 
		
				H1= 4'h0;
				H2= 4'h4;
				end
			
7'd41: begin 
		
				H1= 4'h1;
				H2= 4'h4;
				end
		
7'd42: begin 
		
				H1= 4'h2;
				H2= 4'h4;
				end		
						
				
7'd43: begin 
		
				H1= 4'h3;
				H2= 4'h4;
				end		
	
7'd44: begin 
		
				H1= 4'h4;
				H2= 4'h4;
				end		
						
7'd45: begin 
		
				H1= 4'h5;
				H2= 4'h4;
				end	
			
7'd46: begin 
		
				H1= 4'h6;
				H2= 4'h4;
				end		
						
7'd47: begin 
		
				H1= 4'h7;
				H2= 4'h4;
				end		
						
7'd48: begin 
		
				H1= 4'h8;
				H2= 4'h4;
				end		
						
7'd49: begin 
		
				H1= 4'h9;
				H2= 4'h4;
				end	
			
7'd50: begin 
		
				H1= 4'h0;
				H2= 4'h5;
				end
		
7'd51: begin 
		
				H1= 4'h1;
				H2= 4'h5;
				end
		
7'd52: begin 
		
				H1= 4'h2;
				H2= 4'h5;
				end
		
7'd53: begin 
		
				H1= 4'h3;
				H2= 4'h5;
				end
		
7'd54: begin 
		
				H1= 4'h4;
				H2= 4'h5;
				end
		
7'd55: begin 
		
				H1= 4'h5;
				H2= 4'h5;
				end
		
7'd56: begin 
		
				H1= 4'h6;
				H2= 4'h5;
				end
		
7'd57: begin 
		
				H1= 4'h7;
				H2= 4'h5;
				end
		
7'd58: begin 
		
				H1= 4'h8;
				H2= 4'h5;
				end
		
7'd59: begin 
		
				H1= 4'h9;
				H2= 4'h5;
				end
		
7'd60: begin 
		
				H1= 4'h0;
				H2= 4'h6;
				end
		
7'd61: begin 
		
				H1= 4'h1;
				H2= 4'h6;
				end	
		
7'd62: begin 
	
				H1= 4'h2;
				H2= 4'h6;
				end	
		
7'd63: begin 
		
				H1= 4'h3;
				H2= 4'h6;
				end	
		
7'd64: begin 
		
				H1= 4'h4;
				H2= 4'h6;
				end	
		
7'd65: begin 
		
				H1= 4'h5;
				H2= 4'h6;
				end	
		
7'd66: begin 
		
				H1= 4'h6;
				H2= 4'h6;
				end	
		
7'd67: begin 
		
				H1= 4'h7;
				H2= 4'h6;
				end	
		
7'd68: begin 
		
				H1= 4'h8;
				H2= 4'h6;
				end	
		
7'd69: begin 
		
				H1= 4'h9;
				H2= 4'h6;
				end	
		
7'd70: begin 
		
				H1= 4'h0;
				H2= 4'h7;
				end	
		
7'd71: begin 
		
				H1= 4'h1;
				H2= 4'h7;
				end
	
7'd72: begin 
		
				H1= 4'h2;
				H2= 4'h7;
				end			
					
7'd73: begin 
		
				H1= 4'h3;
				H2= 4'h7;
				end
	
7'd74: begin 
		
				H1= 4'h4;
				H2= 4'h7;
				end
	
7'd75: begin 
		
				H1= 4'h5;
				H2= 4'h7;
				end
	
7'd76: begin 
		
				H1= 4'h6;
				H2= 4'h7;
				end
	
7'd77: begin 
		
				H1= 4'h7;
				H2= 4'h7;
				end
	
7'd78: begin 
		
				H1= 4'h8;
				H2= 4'h7;
				end
	
7'd79: begin 
		
				H1= 4'h9;
				H2= 4'h7;
				end
	
7'd80: begin 
	
				H1= 4'h0;
				H2= 4'h8;
				end
	
7'd81: begin 
		
				H1= 4'h1;
				H2= 4'h8;
				end
	
7'd82: begin 
		
				H1= 4'h2;
				H2= 4'h8;
				end
	
7'd83: begin 
		
				H1= 4'h3;
				H2= 4'h8;
				end
			
7'd84: begin 
		
				H1= 4'h4;
				H2= 4'h8;
				end
				
7'd85: begin 
		
				H1= 4'h5;
				H2= 4'h8;
				end
				
7'd86: begin 
		
				H1= 4'h6;
				H2= 4'h8;
				end
		
7'd87: begin 
		
				H1= 4'h7;
				H2= 4'h8;
				end
				
7'd88: begin 
		
				H1= 4'h8;
				H2= 4'h8;
				end
				
7'd89: begin 
		
				H1= 4'h9;
				H2= 4'h8;
				end
				
7'd90: begin 
		
				H1= 4'h0;
				H2= 4'h9;
				end
			
7'd91: begin 
		
				H1= 4'h1;
				H2= 4'h9;
				end
				
7'd92: begin 
		
				H1= 4'h2;
				H2= 4'h9;
				end
				
7'd93: begin 
		
				H1= 4'h3;
				H2= 4'h9;
				end
				
7'd94: begin 
		
				H1= 4'h4;
				H2= 4'h9;
				end
				
7'd95: begin 
		
				H1= 4'h5;
				H2= 4'h9;
				end
				
7'd96: begin 
		
				H1= 4'h6;
				H2= 4'h9;
				end

7'd97: begin 
		
				H1= 4'h7;
				H2= 4'h9;
				end

7'd98: begin 
		
				H1= 4'h8;
				H2= 4'h9;
				end
			
7'd99: begin 
		
				H1= 4'h9;
				H2= 4'h9;
				end

            default: begin
			H1= 4'h1;
			H2= 4'h1;
			end
        endcase
endmodule

module hitdetection (clock,left,middle,right,inx1,iny1,hit,bot);

input clock,left,middle,right;
input [8:0] inx1;
input [7:0] iny1;
output reg hit,bot;
integer i,j,k,l;

reg [8:0] inx;
reg [7:0] iny;
reg done ;


always @(posedge clock)
begin
	if (!done) begin
	inx <= inx1;
	iny <= iny1;
	i <= 0;
	j <= 0;
	done <= 1;
	hit <= 0;
	bot <= 0;
	end
	if (left && done) begin
		if ((inx == (55+i)) && (iny ==(190+j))) begin
		hit <= 1'b1;
	end
	else begin
	if(j == 50) begin
				i <= i + 1;
				j <= 0;
			end
			
	else begin j <= j + 1; end
	
			
	if(i == 50 && j == 50)begin
				done <= 0;	
			end	
		end
	end
	
	else if (middle && done) begin
	if ((inx == (135+i)) && (iny ==(190+j))) begin
		hit <= 1'b1;
	end
	else begin
	if(j == 50) begin
				i <= i + 1;
				j <= 0;
			end
			
	else begin j <= j + 1; end
	
			
	if(i == 50 && j == 50)begin
				done <= 0;	
			end
		end
	end
	
	else if (right && done) begin
	if ((inx == (190+i)) && (iny ==(190+j))) begin
		hit <= 1'b1;
	end
	else begin
	if(j == 50) begin
				i <= i + 1;
				j <= 0;
			end
			
	else begin j <= j + 1; end
	
			
	if(i == 50 && j == 50)begin
				done <= 0;	
			end	
		end
	end
	else if (iny == 240) begin
			bot <= 1'b1;
			done <=1'b0;
			hit <= 1'b0; end
end	
endmodule
	
module dice(input clock, input readin,output reg [2:0]roll);
	reg [2:0] count = 3'd1;
	
	always @(posedge clock)
	begin
		
	if(count == 3'd6)
		count <= 3'd1;
	else
		count <= count + 3'd1;
	end

	always @(posedge readin)
	begin
		roll <= count;
	end
endmodule		
	
	
	
	
	
	
	
	