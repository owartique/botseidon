import RPi.GPIO as GPIO
from time import sleep
import spidev

MyARM_ResetPin = 19 # Pin 4 of connector = BCM19 = GPIO[1]

MySPI_FPGA = spidev.SpiDev()
MySPI_FPGA.open(0,0)
MySPI_FPGA.max_speed_hz = 500000

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(MyARM_ResetPin, GPIO.OUT)

GPIO.output(MyARM_ResetPin, GPIO.HIGH)
sleep(0.1)
GPIO.output(MyARM_ResetPin, GPIO.LOW)
sleep(0.1)

# set the baud rate
ToSPI = [0x80, 0x00, 0x4F, 0x00, 0x00]
FromSPI = MySPI_FPGA.xfer2(ToSPI)

# write the value 5 in the register at address 1
ToSPI = [0x81, 0x00, 0x00, 0x00, 0x05]
FromSPI = MySPI_FPGA.xfer2(ToSPI)

# write the value 4 in the register at address 2
ToSPI = [0x82, 0x00, 0x00, 0x00, 0x04]
FromSPI = MySPI_FPGA.xfer2(ToSPI)

# 
GPIO.output(MyARM_ResetPin, GPIO.HIGH)
sleep(0.1)
GPIO.output(MyARM_ResetPin, GPIO.LOW)
sleep(1)

ToSPI = [0x03, 0x00, 0x00, 0x00, 0x00]
FromSPI = MySPI_FPGA.xfer2(ToSPI)
print(FromSPI)


