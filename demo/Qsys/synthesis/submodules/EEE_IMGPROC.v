module EEE_IMGPROC(
	// global clock & reset
	clk,
	reset_n,
	
	// mm slave
	s_chipselect,
	s_read,
	s_write,
	s_readdata,
	s_writedata,
	s_address,

	// stream sink
	sink_data,
	sink_valid,
	sink_ready,
	sink_sop,
	sink_eop,
	
	// streaming source
	source_data,
	source_valid,
	source_ready,
	source_sop,
	source_eop,
	
	// conduit
	mode
	
);


// global clock & reset
input	clk;
input	reset_n;

// mm slave
input							s_chipselect;
input							s_read;
input							s_write;
output	reg	[31:0]	s_readdata;
input	[31:0]				s_writedata;
input	[2:0]					s_address;


// streaming sink
input	[23:0]            	sink_data;
input								sink_valid;
output							sink_ready;
input								sink_sop;
input								sink_eop;

// streaming source
output	[23:0]			  	   source_data;
output								source_valid;
input									source_ready;
output								source_sop;
output								source_eop;

// conduit export
input                         mode;

////////////////////////////////////////////////////////////////////////
//
parameter IMAGE_W = 11'd640;
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 10;
parameter BB_COL_DEFAULT = 24'h00ff00;


wire [7:0]   red, green, blue, grey;
wire [7:0]   red_out, green_out, blue_out;

wire         sop, eop, in_valid, out_ready;
////////////////////////////////////////////////////////////////////////

// Detect red areas
wire red_detect;
wire blue_detect;
wire yellow_detect;
assign red_detect = red[7] & ~green[7] & ~blue[7];
assign blue_detect = ~red[7] & ~green[7] & blue[7];
assign yellow_detect = red[7] & green[7] & ~blue[7];

// Find boundary of cursor box

// Highlight detected areas
wire [23:0] red_high;
assign grey = green[7:1] + red[7:2] + blue[7:2]; //Grey = green/2 + red/4 + blue/4
assign red_high  =  red_detect ? {8'hff, 8'h0, 8'h0} : {grey, grey, grey};

// Show bounding box
wire [23:0] new_image;
wire bb_active;
assign bb_active = (x == left) | (x == right) | (y == top) | (y == bottom);
assign new_image = bb_active ? bb_col : red_high;

// Switch output pixels depending on mode switch
// Don't modify the start-of-packet word - it's a packet discriptor
// Don't modify data in non-video packets
assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? new_image : {red,green,blue};

//Count valid pixels to tget the image coordinates. Reset and detect packet type on Start of Packet.
reg [10:0] x, y;
reg packet_video;
always@(posedge clk) begin
	if (sop) begin
		x <= 11'h0;
		y <= 11'h0;
		packet_video <= (blue[3:0] == 3'h0);
	end
	else if (in_valid) begin
		if (x == IMAGE_W-1) begin
			x <= 11'h0;
			y <= y + 11'h1;
		end
		else begin
			x <= x + 11'h1;
		end
	end
end

//Find first and last red pixels
reg [10:0] x_min_r, y_min_r, x_max_r, y_max_r;
reg [10:0] x_min_b, y_min_b, x_max_b, y_max_b;
reg [10:0] x_min_y, y_min_y, x_max_y, y_max_y;
always@(posedge clk) begin
	if (red_detect & in_valid) begin	//Update bounds when the pixel is red
		if (x < x_min_r) x_min_r <= x;
		if (x > x_max_r) x_max_r <= x;
		if (y < y_min_r) y_min_r <= y;
		if (y > y_max_r) y_max_r <= y;
	end
	if (blue_detect & in_valid) begin	//Update bounds when the pixel is blue
		if (x < x_min_b) x_min_b <= x;
		if (x > x_max_b) x_max_b <= x;
		if (y < y_min_b) y_min_b <= y;
		if (y > y_max_b) y_max_b <= y;
	end
	if (yellow_detect & in_valid) begin	//Update bounds when the pixel is yellow
		if (x < x_min_y) x_min_y <= x;
		if (x > x_max_y) x_max_y <= x;
		if (y < y_min_y) y_min_y <= y;
		if (y > y_max_y) y_max_y <= y;
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
		x_min_r <= IMAGE_W-11'h1;
		x_max_r <= 0;
		y_min_r <= IMAGE_H-11'h1;
		y_max_r <= 0;
		x_min_b <= IMAGE_W-11'h1;
		x_max_b <= 0;
		y_min_b <= IMAGE_H-11'h1;
		y_max_b <= 0;
		x_min_y <= IMAGE_W-11'h1;
		x_max_y <= 0;
		y_min_y <= IMAGE_H-11'h1;
		y_max_y <= 0;
	end
end

//Process bounding box at the end of the frame.
reg [2:0] msg_state;
reg [10:0] left, right, top, bottom;
reg [7:0] frame_count;
always@(posedge clk) begin
	if (eop & in_valid & packet_video) begin  //Ignore non-video packets
		
		//Latch edges for display overlay on next frame
		left <= x_min_r;
		right <= x_max_r;
		top <= y_min_r;
		bottom <= y_max_r;
		
		
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
		frame_count <= frame_count - 1;
		
		if (frame_count == 0 && msg_buf_size < MESSAGE_BUF_MAX - 3) begin
			msg_state <= 3'b001;
			frame_count <= MSG_INTERVAL-1;
		end
	end
	
	//Cycle through message writer states once started
	if (msg_state != 3'b000) msg_state <= msg_state + 3'b001;

end
	
//Generate output messages for CPU
reg [31:0] msg_buf_in; 
wire [31:0] msg_buf_out;
reg msg_buf_wr;
wire msg_buf_rd, msg_buf_flush;
wire [7:0] msg_buf_size;
wire msg_buf_empty;

`define START_MSG_ID "RBY"

always@(*) begin	//Write words to FIFO as state machine advances
	case(msg_state)
		3'b000: begin
			msg_buf_in = 32'b0;
			msg_buf_wr = 1'b0;
		end
		3'b001: begin
			msg_buf_in = `START_MSG_ID;	//Message ID
			msg_buf_wr = 1'b1;
		end
		3'b010: begin
			msg_buf_in = {5'b0, x_min_r, 5'b0, y_min_r};	//Top left coordinate
			msg_buf_wr = 1'b1;
		end
		3'b011: begin
			msg_buf_in = {5'b0, x_max_r, 5'b0, y_max_r}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
		3'b100: begin
			msg_buf_in = {5'b0, x_min_b, 5'b0, y_min_b};	//Top left coordinate
			msg_buf_wr = 1'b1;
		end
		3'b101: begin
			msg_buf_in = {5'b0, x_max_b, 5'b0, y_max_b}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
		3'b110: begin
			msg_buf_in = {5'b0, x_min_y, 5'b0, y_min_y};	//Top left coordinate
			msg_buf_wr = 1'b1;
		end
		3'b111: begin
			msg_buf_in = {5'b0, x_max_y, 5'b0, y_max_y}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
	endcase
end


//Output message FIFO
MSG_FIFO	MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),
	.empty (msg_buf_empty)
	);


//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready),
	.valid_out(in_valid),
	.data_out({red,green,blue,sop,eop}),
	.ready_in(out_ready),
	.valid_in(sink_valid),
	.data_in({sink_data,sink_sop,sink_eop})
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(out_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop}),
	.ready_in(source_ready),
	.valid_in(in_valid),
	.data_in({red_out, green_out, blue_out, sop, eop})
);


/////////////////////////////////
/// Memory-mapped port		 /////
/////////////////////////////////

// Addresses
`define REG_STATUS    			0
`define READ_MSG    				1
`define READ_ID    				2
`define REG_BBCOL					3

//Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused


// Process write

reg  [7:0]   reg_status;
reg	[23:0]	bb_col;

always @ (posedge clk)
begin
	if (~reset_n)
	begin
		reg_status <= 8'b0;
		bb_col <= BB_COL_DEFAULT;
	end
	else begin
		if(s_chipselect & s_write) begin
		   if      (s_address == `REG_STATUS)	reg_status <= s_writedata[7:0];
		   if      (s_address == `REG_BBCOL)	bb_col <= s_writedata[23:0];
		end
	end
end


//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush = (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);


// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
		if   (s_address == `REG_BBCOL) s_readdata <= {8'h0, bb_col};
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);
						


endmodule

