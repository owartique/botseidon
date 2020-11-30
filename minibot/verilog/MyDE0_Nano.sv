
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
	logic [31:0] leftPulse, rightPulse;
	
	assign leftA =  GPIO_1[0];
	assign leftB =  GPIO_1_IN[0];
	
	assign rightA =  GPIO_1[2];
	assign rightB =  GPIO_1[1];
	
	logic clock_div;
	logic [31:0] divider;
	assign divider = 32'b11110100001001000000;
	
	// on divise la clock par 1e6 pour avoir une clock de 50Hz (période de 0.02 sec)
	clockdivider myclockdivider(CLOCL_50,divider,clock_div);
	// on utilise cette clock de 50Hz pour calculer le nombre de pulse pendant 0.02 sec
	encoder leftEncoder(leftA,leftB,clock_div,leftPulse);
	encoder rightEncoder(rightA,rightB,clock_div,rightPulse);
	
	// en fonction du message reçu du raspberry on assigne la donnée à envoyer
	always_comb begin
		case(DataFromPI)
			31'b1   : DataToPI = leftPulse;
			31'b10  : DataToPI = rightPulse;
			default : DataToPI = 32'bx;
		endcase
	end
	
	
endmodule


//=======================================================
//	ENCODER
//=======================================================
	
	module encoder(input logic  A,B,clk_div,
					   output logic [31:0] tick);
					  
	logic [31:0] pulse;
	
	always_ff @(posedge clk_div) begin
		// on ajoute un bias de 2147483647 comme ça pas besoin de s'emmerder avec le signe
		// par exemple si le robot roule en arrière on pourrait avoir un nombre de tick négatif
		// mais en additionnant avec 2147483647 on s'assure qu'il est positif
		// on suppose par contre qu'il peut avoir max 2147483647 pulse en 0.02 sec sinon overflow
		// Il faudra en tenir compte dans le code en C 
		pulse = 32'b0111_1111_1111_1111_1111_1111_1111_1111;
	end
	
	// a chaque changement d'état de A, si A=B alors cela veut dire qu'on tourne en sens horlogique				  
	always_ff @(posedge A, negedge A)begin
		pulse = (A==B) ? pulse+1'b1 : pulse-1'b1; // A leads B for counter clockwise rotation (cfr datasheet)
	end
	
	assign tick = pulse;
						
	endmodule
  
 
//=======================================================
//	CLOCK DIVIDER
//=======================================================
	
	module clockdivider(input logic clk,
							  input logic [31:0] divider,
							  output logic clk_div);
	
	logic [31:0] cnt;
	
	always_ff @(posedge clk) begin
		if(cnt==divider)begin
			cnt <= 0;
			clk_div <= ~clk_div;
		end	
	end
	
	endmodule
	


 