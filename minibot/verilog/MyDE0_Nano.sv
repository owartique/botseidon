
//=======================================================
//  MyARM
//=======================================================

module MyDE0_Nano(

//////////// CLOCK //////////
input logic 		          		CLOCK_50,

//////////// LED //////////
output logic		     [7:0]		LED,

//////////// KEY //////////
input logic 		     [1:0]		KEY,

//////////// SW //////////
input logic 		     [3:0]		SW,

//////////// SDRAM //////////
output logic		    [12:0]		DRAM_ADDR,
output logic		     [1:0]		DRAM_BA,
output logic		          		DRAM_CAS_N,
output logic		          		DRAM_CKE,
output logic		          		DRAM_CLK,
output logic		          		DRAM_CS_N,
inout logic 		    [15:0]		DRAM_DQ,
output logic		     [1:0]		DRAM_DQM,
output logic		          		DRAM_RAS_N,
output logic		          		DRAM_WE_N,

//////////// EPCS //////////
output logic		          		EPCS_ASDO,
input logic 		          		EPCS_DATA0,
output logic		          		EPCS_DCLK,
output logic		          		EPCS_NCSO,

//////////// Accelerometer and EEPROM //////////
output logic		          		G_SENSOR_CS_N,
input logic 		          		G_SENSOR_INT,
output logic		          		I2C_SCLK,
inout logic 		          		I2C_SDAT,

//////////// ADC //////////
output logic		          		ADC_CS_N,
output logic		          		ADC_SADDR,
output logic		          		ADC_SCLK,
input logic 		          		ADC_SDAT,

//////////// 2x13 GPIO Header //////////
inout logic 		    [12:0]		GPIO_2,
input logic 		     [2:0]		GPIO_2_IN,

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_0_PI,
input logic 		     [1:0]		GPIO_0_PI_IN,

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_1,
input logic 		     [1:0]		GPIO_1_IN

);

//=======================================================
//	SPI
//=======================================================

	logic spi_clk, spi_cs, spi_mosi, spi_miso;
	logic [31:0] DataToPI, DataFromPI;

	spi_slave spi_slave_instance(
		.sck(spi_clk),
		.mosi(spi_mosi),
		.miso(spi_miso),
		.reset(),
		.d(DataToPI),
		.q(DataFromPI)
	);

	assign spi_clk  		= GPIO_0_PI[11];	// SCLK = pin 16 = GPIO_11
	assign spi_cs   		= GPIO_0_PI[9];	// CE0  = pin 14 = GPIO_9
	assign spi_mosi     	= GPIO_0_PI[15];	// MOSI = pin 20 = GPIO_15
	
	assign GPIO_0_PI[13] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 18 = GPIO_13			 

//=======================================================
//	MINIBOT
//=======================================================

	logic A,B;

	assign A =  GPIO_1[0];
	assign B =  GPIO_1_IN[0];

/*	encoder myencoder(A,B,CLOCK_50,speed);
	

	logic re,cs;
	logic [31:0] wd,rd;
	logic [4:0] adr;
	
	decode mydecode(DataFromPI,re,wd,adr);
	memory mymemory(CLOCK_50,re,cs,adr,wd,rd);
*/
	assign DataToPI = 32'b1111_0000_1111_0000_1111_0000_1111_0000;



endmodule

//=======================================================
//	MEMORY
//=======================================================

	module memory (input  logic clk,re,cs,
						input  logic [4:0] adr,
						input  logic [31:0] wd,
						output logic [31:0] rd);
		
	logic [31:0] RAM [4:0];


	always_ff @(posedge clk)
	begin
		if(cs & re) rd <= RAM[adr];
		else if(cs & ~re) RAM[adr] <= wd;
		else rd <= 31'bx;	
	end	
	
	endmodule
	
	

//=======================================================
//	ENCODER
//=======================================================
	
	module encoder(input logic A,B,clk,
					  output logic [9:0] speed);

	logic 		 sens; // sens = 0 : sens anti-horlogique, sens = 1 : sens horlogique			  
	logic [19:0] cnt;
	logic [30:0] tickA, tickB, tick;
	logic [31:0] w;


	always_ff @(posedge clk)
		begin
				cnt <= cnt+1;
				if(cnt==20'b11110100001001000000) // si cnt = 1e6 alors 0.2 sec se sont écoulées
					begin
							tick <= tickA + tickB;
							w <= {sens, tick};
							cnt <= 0;
							tick <= 0;
					end
		end


	always_ff @(posedge A)
	begin
		tickA <= tickA + 1;
		if (A & ~B) sens <= 1'b1;
	end
	always_ff @(posedge B)
	begin
		tickB <= tickB + 1;
		if (~A & B) sens <= 1'b0;
	end

	assign speed = w;

							
	endmodule
	
//=======================================================
//	DECODE
//=======================================================

module decode(input  logic [31:0] msg,
				  output logic re,
				  output logic [11:0] wd,
				  output logic [4:0] adr);
				  
assign re = msg[31];
assign adr = msg[30:26];
assign wd = re ? 12'bx : msg[25:14];

endmodule
 