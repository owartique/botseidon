
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

	logic leftA, leftB, rightA, rightB;
	logic [15:0] leftPulse, rightPulse;
	
	assign leftA =  GPIO_1[0];
	assign leftB =  GPIO_1_IN[0];
	
	assign rightA =  GPIO_1[2];
	assign rightB =  GPIO_1[1];
	
	encoder leftEncoder(leftA,leftB,CLOCK_50,leftPulse);
	encoder rightEncoder(rightA,rightB,CLOCK_50,rightPulse);

	assign DataToPI = {leftPulse,rightPulse};
	
	
endmodule


//=======================================================
//	ENCODER
//=======================================================
	
	module encoder(input logic A,B,clk,
					  output logic [15:0] tick);
					  
	logic [15:0] pulse;
	logic [31:0] cnt;
	
	always_ff @(posedge clk) begin
		if (cnt==32'b11110100001001000000)begin
   // on ajoute un bias de 32768 comme ça pas besoin de s'emmerder avec le signe
	// par exemple si le robot roule en arrière on pourrait avoir un nombre de tick négatif
	// mais en additionnant avec 32768 on s'assure qu'il est positif
	// Il faudra en tenir compte dans le code en C 
			assign tick = pulse + 16'b10000000_00000000;
			cnt = 32'b0;
			pulse = 16'b0;
		end
		else begin
			// a chaque changement d'état de A, si A=B alors cela veut dire qu'on tourne en sens horlogique				  
			always_ff @(posedge A, negedge A)begin
				pulse = (A==B) ? pulse+1'b1 : pulse-1'b1; // A leads B for counter clockwise rotation (cfr datasheet)
			end
			cnt = cnt + 1'b1;
		end
	end
								
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
	
	


 