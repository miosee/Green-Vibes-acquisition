EESchema Schematic File Version 4
LIBS:acquisition-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Amplifier_Operational:MCP601-xP U2
U 1 1 60DACD36
P 3450 4400
F 0 "U2" H 3450 4550 50  0000 L CNN
F 1 "MCP6271" H 3400 4700 50  0000 L CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 3350 4200 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21314g.pdf" H 3600 4550 50  0001 C CNN
	1    3450 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 60DACD95
P 2500 4300
F 0 "R6" V 2400 4300 50  0000 C CNN
F 1 "1k" V 2600 4300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2430 4300 50  0001 C CNN
F 3 "~" H 2500 4300 50  0001 C CNN
	1    2500 4300
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 60DACDF8
P 3000 1050
F 0 "C1" H 3115 1097 50  0000 L CNN
F 1 "220u" H 3115 1004 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 3038 900 50  0001 C CNN
F 3 "~" H 3000 1050 50  0001 C CNN
	1    3000 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 60DACEF7
P 3350 4700
F 0 "#PWR017" H 3350 4450 50  0001 C CNN
F 1 "GND" H 3355 4525 50  0000 C CNN
F 2 "" H 3350 4700 50  0001 C CNN
F 3 "" H 3350 4700 50  0001 C CNN
	1    3350 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 60DACF0F
P 3000 1200
F 0 "#PWR02" H 3000 950 50  0001 C CNN
F 1 "GND" H 3005 1025 50  0000 C CNN
F 2 "" H 3000 1200 50  0001 C CNN
F 3 "" H 3000 1200 50  0001 C CNN
	1    3000 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 60DACF48
P 3000 900
F 0 "#PWR01" H 3000 750 50  0001 C CNN
F 1 "+5V" H 3015 1075 50  0000 C CNN
F 2 "" H 3000 900 50  0001 C CNN
F 3 "" H 3000 900 50  0001 C CNN
	1    3000 900 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR015
U 1 1 60DACF60
P 3350 4100
F 0 "#PWR015" H 3350 3950 50  0001 C CNN
F 1 "+5V" H 3365 4275 50  0000 C CNN
F 2 "" H 3350 4100 50  0001 C CNN
F 3 "" H 3350 4100 50  0001 C CNN
	1    3350 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 60DACF9D
P 2900 4150
F 0 "R5" H 2970 4197 50  0000 L CNN
F 1 "51k" H 2970 4104 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2830 4150 50  0001 C CNN
F 3 "~" H 2900 4150 50  0001 C CNN
	1    2900 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 4300 2900 4300
Connection ~ 2900 4300
Wire Wire Line
	2900 4300 3150 4300
$Comp
L power:+5V #PWR014
U 1 1 60DAD04E
P 2900 4000
F 0 "#PWR014" H 2900 3850 50  0001 C CNN
F 1 "+5V" H 2915 4175 50  0000 C CNN
F 2 "" H 2900 4000 50  0001 C CNN
F 3 "" H 2900 4000 50  0001 C CNN
	1    2900 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 60DAD077
P 3550 5000
F 0 "R8" V 3450 5000 50  0000 C CNN
F 1 "51k" V 3650 5000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3480 5000 50  0001 C CNN
F 3 "~" H 3550 5000 50  0001 C CNN
	1    3550 5000
	0    1    1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 60DAD0AF
P 3100 5150
F 0 "R9" H 3170 5197 50  0000 L CNN
F 1 "2k" H 3170 5104 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3030 5150 50  0001 C CNN
F 3 "~" H 3100 5150 50  0001 C CNN
	1    3100 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 5000 3100 5000
Wire Wire Line
	3100 5000 3100 4500
Wire Wire Line
	3100 4500 3150 4500
Connection ~ 3100 5000
$Comp
L power:GND #PWR019
U 1 1 60DAD10B
P 3100 5300
F 0 "#PWR019" H 3100 5050 50  0001 C CNN
F 1 "GND" H 3105 5125 50  0000 C CNN
F 2 "" H 3100 5300 50  0001 C CNN
F 3 "" H 3100 5300 50  0001 C CNN
	1    3100 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 4400 3750 5000
Wire Wire Line
	3750 5000 3700 5000
$Comp
L MCU_Module:Arduino_Nano_v3.x U3
U 1 1 60DAD216
P 5850 2400
F 0 "U3" H 5350 3250 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 5600 3500 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 6000 1450 50  0001 L CNN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 5850 1400 50  0001 C CNN
	1    5850 2400
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR04
U 1 1 60DAD24C
P 5650 1400
F 0 "#PWR04" H 5650 1250 50  0001 C CNN
F 1 "+5V" H 5665 1575 50  0000 C CNN
F 2 "" H 5650 1400 50  0001 C CNN
F 3 "" H 5650 1400 50  0001 C CNN
	1    5650 1400
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 60DAD417
P 5800 3400
F 0 "#PWR012" H 5800 3150 50  0001 C CNN
F 1 "GND" H 5805 3225 50  0000 C CNN
F 2 "" H 5800 3400 50  0001 C CNN
F 3 "" H 5800 3400 50  0001 C CNN
	1    5800 3400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5750 3400 5800 3400
Connection ~ 5800 3400
Wire Wire Line
	5800 3400 5850 3400
Connection ~ 3750 4400
Text Label 1900 4300 0    50   ~ 0
I
Text Label 3950 4400 0    50   ~ 0
Vi
$Comp
L Amplifier_Operational:MCP601-xP U1
U 1 1 60E69843
P 3500 2400
F 0 "U1" H 3500 2550 50  0000 L CNN
F 1 "MCP6271" H 3450 2700 50  0000 L CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 3400 2200 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21314g.pdf" H 3650 2550 50  0001 C CNN
	1    3500 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 60E6984A
P 2550 2300
F 0 "R2" V 2450 2300 50  0000 C CNN
F 1 "7.5k" V 2650 2300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2480 2300 50  0001 C CNN
F 3 "~" H 2550 2300 50  0001 C CNN
	1    2550 2300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 60E69851
P 3400 2700
F 0 "#PWR09" H 3400 2450 50  0001 C CNN
F 1 "GND" H 3405 2525 50  0000 C CNN
F 2 "" H 3400 2700 50  0001 C CNN
F 3 "" H 3400 2700 50  0001 C CNN
	1    3400 2700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR08
U 1 1 60E69857
P 3400 2100
F 0 "#PWR08" H 3400 1950 50  0001 C CNN
F 1 "+5V" H 3415 2275 50  0000 C CNN
F 2 "" H 3400 2100 50  0001 C CNN
F 3 "" H 3400 2100 50  0001 C CNN
	1    3400 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 60E6985D
P 2950 2150
F 0 "R1" H 3020 2197 50  0000 L CNN
F 1 "15k" H 3020 2104 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2880 2150 50  0001 C CNN
F 3 "~" H 2950 2150 50  0001 C CNN
	1    2950 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 2300 2950 2300
Connection ~ 2950 2300
Wire Wire Line
	2950 2300 3200 2300
$Comp
L power:+5V #PWR07
U 1 1 60E69867
P 2950 2000
F 0 "#PWR07" H 2950 1850 50  0001 C CNN
F 1 "+5V" H 2965 2175 50  0000 C CNN
F 2 "" H 2950 2000 50  0001 C CNN
F 3 "" H 2950 2000 50  0001 C CNN
	1    2950 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 60E6986D
P 3600 3000
F 0 "R3" V 3500 3000 50  0000 C CNN
F 1 "7.5k" V 3700 3000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3530 3000 50  0001 C CNN
F 3 "~" H 3600 3000 50  0001 C CNN
	1    3600 3000
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 60E69874
P 3150 3150
F 0 "R4" H 3220 3197 50  0000 L CNN
F 1 "15k" H 3220 3104 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3080 3150 50  0001 C CNN
F 3 "~" H 3150 3150 50  0001 C CNN
	1    3150 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 3000 3150 3000
Wire Wire Line
	3150 3000 3150 2500
Wire Wire Line
	3150 2500 3200 2500
Connection ~ 3150 3000
$Comp
L power:GND #PWR011
U 1 1 60E6987F
P 3150 3300
F 0 "#PWR011" H 3150 3050 50  0001 C CNN
F 1 "GND" H 3155 3125 50  0000 C CNN
F 2 "" H 3150 3300 50  0001 C CNN
F 3 "" H 3150 3300 50  0001 C CNN
	1    3150 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 2400 3800 3000
Wire Wire Line
	3800 3000 3750 3000
Connection ~ 3800 2400
Text Label 2050 2300 0    50   ~ 0
E
Text Label 4000 2400 0    50   ~ 0
Ve
$Comp
L Device:R R7
U 1 1 60E69AD4
P 2100 4800
F 0 "R7" H 2170 4847 50  0000 L CNN
F 1 "1" H 2170 4754 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2030 4800 50  0001 C CNN
F 3 "~" H 2100 4800 50  0001 C CNN
	1    2100 4800
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 60E69B35
P 1500 2850
F 0 "J2" H 1650 2800 50  0000 C CNN
F 1 "potentiometer" H 1600 2650 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1500 2850 50  0001 C CNN
F 3 "~" H 1500 2850 50  0001 C CNN
	1    1500 2850
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 60E69BCE
P 2100 4950
F 0 "#PWR018" H 2100 4700 50  0001 C CNN
F 1 "GND" H 2105 4775 50  0000 C CNN
F 2 "" H 2100 4950 50  0001 C CNN
F 3 "" H 2100 4950 50  0001 C CNN
	1    2100 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 4650 2100 4300
Connection ~ 2100 4300
Wire Wire Line
	2100 4300 2350 4300
Wire Wire Line
	1700 4300 1700 2950
Wire Wire Line
	1700 4300 2100 4300
Text Notes 3900 2600 0    50   ~ 0
Ve = E + 2.5V
Text Notes 3850 4600 0    50   ~ 0
Vi = 26*I(A) + 2.55V
Wire Wire Line
	3800 2400 5350 2400
Wire Wire Line
	5350 2500 4650 2500
Wire Wire Line
	4650 2500 4650 4400
Wire Wire Line
	3750 4400 4650 4400
$Comp
L power:GND #PWR05
U 1 1 60E6DE41
P 7300 1950
F 0 "#PWR05" H 7300 1700 50  0001 C CNN
F 1 "GND" H 7305 1775 50  0000 C CNN
F 2 "" H 7300 1950 50  0001 C CNN
F 3 "" H 7300 1950 50  0001 C CNN
	1    7300 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR03
U 1 1 60E6DE93
P 7300 1650
F 0 "#PWR03" H 7300 1500 50  0001 C CNN
F 1 "+5V" H 7315 1825 50  0000 C CNN
F 2 "" H 7300 1650 50  0001 C CNN
F 3 "" H 7300 1650 50  0001 C CNN
	1    7300 1650
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J3
U 1 1 60E6F2AC
P 8550 4000
F 0 "J3" H 8630 4043 50  0000 L CNN
F 1 "Accelerometer" H 8630 3950 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical" H 8550 4000 50  0001 C CNN
F 3 "~" H 8550 4000 50  0001 C CNN
	1    8550 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 3800 8350 3800
Text Label 8100 3800 0    50   ~ 0
Vin
Text Label 8100 3900 0    50   ~ 0
GND
Text Label 8100 4000 0    50   ~ 0
SDA
Text Label 8100 4100 0    50   ~ 0
SCL
Text Label 8100 4200 0    50   ~ 0
INT
$Comp
L power:+5V #PWR013
U 1 1 60E718EE
P 7950 3800
F 0 "#PWR013" H 7950 3650 50  0001 C CNN
F 1 "+5V" H 7965 3975 50  0000 C CNN
F 2 "" H 7950 3800 50  0001 C CNN
F 3 "" H 7950 3800 50  0001 C CNN
	1    7950 3800
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 60E71919
P 7850 4350
F 0 "#PWR016" H 7850 4100 50  0001 C CNN
F 1 "GND" H 7855 4175 50  0000 C CNN
F 2 "" H 7850 4350 50  0001 C CNN
F 3 "" H 7850 4350 50  0001 C CNN
	1    7850 4350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7850 3900 7850 4350
Wire Wire Line
	7850 3900 8350 3900
Wire Wire Line
	6350 2000 6500 2000
Wire Wire Line
	6500 2000 6500 4200
Wire Wire Line
	6500 4200 8350 4200
Wire Wire Line
	5150 4000 5150 2800
Wire Wire Line
	5150 2800 5350 2800
Wire Wire Line
	5150 4000 8350 4000
Wire Wire Line
	5250 4100 5250 2900
Wire Wire Line
	5250 2900 5350 2900
Wire Wire Line
	5250 4100 8350 4100
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 60EEEDC5
P 800 3350
F 0 "J1" H 950 3300 50  0000 C CNN
F 1 "Coil" H 719 3477 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 800 3350 50  0001 C CNN
F 3 "~" H 800 3350 50  0001 C CNN
	1    800  3350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2400 2300 1700 2300
Wire Wire Line
	1700 2300 1700 2850
Wire Wire Line
	1700 2300 1000 2300
Wire Wire Line
	1000 2300 1000 3350
Connection ~ 1700 2300
NoConn ~ 5750 1400
NoConn ~ 5950 1400
NoConn ~ 6350 1800
NoConn ~ 6350 1900
NoConn ~ 5350 1800
NoConn ~ 5350 1900
NoConn ~ 6350 2100
NoConn ~ 5350 2200
NoConn ~ 5350 2600
NoConn ~ 5350 2700
NoConn ~ 6350 2800
NoConn ~ 6350 2900
NoConn ~ 6350 3000
NoConn ~ 6350 3100
NoConn ~ 5350 3000
NoConn ~ 5350 3100
Wire Wire Line
	1000 4950 2100 4950
Wire Wire Line
	1000 3450 1000 4950
Connection ~ 2100 4950
Wire Wire Line
	6350 2200 8350 2200
Wire Wire Line
	6350 2300 8350 2300
Wire Wire Line
	6350 2400 8350 2400
Wire Wire Line
	6350 2500 8350 2500
Wire Wire Line
	6350 2600 8350 2600
Wire Wire Line
	6350 2700 8350 2700
$Comp
L Device:R_POT RV1
U 1 1 60F16450
P 7300 1800
F 0 "RV1" H 7230 1753 50  0000 R CNN
F 1 "10k" H 7230 1846 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3296W_Vertical" H 7300 1800 50  0001 C CNN
F 3 "~" H 7300 1800 50  0001 C CNN
	1    7300 1800
	1    0    0    1   
$EndComp
Text Label 8050 2200 0    50   ~ 0
RS
Text Label 8050 2100 0    50   ~ 0
R-W
Text Label 8050 2300 0    50   ~ 0
Enable
Text Label 8050 2400 0    50   ~ 0
DB4
Text Label 8050 2500 0    50   ~ 0
DB5
Text Label 8050 2600 0    50   ~ 0
DB6
Text Label 8050 2700 0    50   ~ 0
DB7
Text Label 8050 2000 0    50   ~ 0
GND
Text Label 8050 1900 0    50   ~ 0
VO
Text Label 8050 1800 0    50   ~ 0
VDD
$Comp
L Connector_Generic:Conn_01x10 J4
U 1 1 60F2F446
P 8550 2200
F 0 "J4" H 8630 2193 50  0000 L CNN
F 1 "LCD" H 8630 2100 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 8550 2200 50  0001 C CNN
F 3 "~" H 8550 2200 50  0001 C CNN
	1    8550 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 1900 7750 1900
Wire Wire Line
	7750 1900 7750 1800
Wire Wire Line
	7450 1800 7750 1800
Wire Wire Line
	8350 2000 7850 2000
Wire Wire Line
	7500 2000 7500 1950
Wire Wire Line
	7500 1950 7300 1950
Connection ~ 7300 1950
Wire Wire Line
	8350 2100 7850 2100
Wire Wire Line
	7850 2100 7850 2000
Connection ~ 7850 2000
Wire Wire Line
	7850 2000 7500 2000
Wire Wire Line
	8350 1800 7850 1800
Wire Wire Line
	7850 1800 7850 1650
Wire Wire Line
	7850 1650 7300 1650
Connection ~ 7300 1650
$EndSCHEMATC
