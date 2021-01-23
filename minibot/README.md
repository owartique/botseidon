# minibot

The directory WhiteSpirit_C contains the files of the minibot (6 dec 18h) \
The directory GrayCode_C contains the files given by the assistants (unmodified) \
The directory Python contains the python script to read in SPI \
The directory WhiteSpirit_python contains the python files given by the assistants (unmodified)

## Useful resources
* GitHub slamtec/rplidar for C++ : https://github.com/slamtec/rplidar_sdk
* Manual of the sdk from Slamtec : http://bucket.download.slamtec.com/351a5409ddfba077ad11ec5071e97ba5bf2c5d0a/LR002_SLAMTEC_rplidar_sdk_v1.0_en.pdf?fbclid=IwAR1bmmi_aIBUWp19rJtgu2DuJIjdSW2BhuIUCGTzpyjakVROLNjMV3zWs0I
* Slamtec RRLIDAR : Interface Protocol and Application Notes : https://www.robotshop.com/media/files/pdf2/rpk-02-communication-protocol.pdf
* Slamtec install tuto : http://developer.slamtec.com/docs/slamware/cpp-sdk-en/2.8.0_rtm/?fbclid=IwAR0n3ewoXUr1POoVsF6gEOWcJtpf7d9gVip1L2fVje4NJ_ixXStBRh5o0sw

## Convention
* By definition every comment/variable/function/... must be written in English
* Functions must be commented and before every definition the input/output must be documented as follow : 
```c
/*
* param a : first number
* param b : second number
* return : a+b
* description : computes the addition between two numbers and returns the result
*/
int addition(int a, int b){
  return a + b;
}
```
* Keep the main.cc file as clean as possible
* Every function must be declared in a header file and included in the main file
* Use pointers and references as much as possible
* Do not create functions that work in only one specific case
* Keep the code as simple as possible, if the function gets too complicated then split it in multiple subfunctions



