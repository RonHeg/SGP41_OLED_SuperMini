SGP41 OLED SuperMini is a small PCB from RJ Hegler Technologies, LLC contyaining Temperature, Humidity, Pressure, VOC and NOx sensors 
meant to connect to the ESP32C3 SuperMini microcontroller. The PCB is also configurable to easily attach to a XIAO microcontroller or, 
with jumper wires, you couled attach it to other Arduino processors.

The sensors are: Sensirion AG SHT-40-AD1B Temperature and Humidity. Sensirion AG SGP41-D-R4 NOx and VOC. TDK ICP-10111 Barometric pressure and temperature. 
The PBD uses its hosts 3.3v supply and communicates via I2C.

EVs-2_SGP41_VOC6.ino sends data thru the serial port at a baud rate of 115,200
and also displays 4 lines on a OLED display (please verify pinout, some OLED displays have Vcc and GND reversed).
The display reads:
	SGP41 VOC Sensor
	78.23F, RH 48.00%
	NOx = 1, VOC = 94
	28.97inHg
(Numbers in the data are an example, your readings should be the actual for your conditions)

Example off data sent through the serial port is:

SHT40
T: 21.53        RH: 42.09
Tticks: 24911   RHticks: 25212

SGP41
compensationRh: 25212   compensationT: 24911
raw VOC Index: 30327    raw NOx Index: 14924
VOC Index: 103  NOx Index: 1

ICP-10111
Temperature: 22.41C
97909.06Pa      28.92inHg
Altitude: 302.64m
------------------------------
