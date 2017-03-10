EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:particle.io
LIBS:goldentee-topper-base-cache
EELAYER 25 0
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
L +5V #PWR01
U 1 1 58BA155D
P 3150 5950
F 0 "#PWR01" H 3150 5800 50  0001 C CNN
F 1 "+5V" H 3150 6090 50  0000 C CNN
F 2 "" H 3150 5950 50  0000 C CNN
F 3 "" H 3150 5950 50  0000 C CNN
	1    3150 5950
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR02
U 1 1 58BA1590
P 4250 5950
F 0 "#PWR02" H 4250 5800 50  0001 C CNN
F 1 "+3V3" H 4250 6090 50  0000 C CNN
F 2 "" H 4250 5950 50  0000 C CNN
F 3 "" H 4250 5950 50  0000 C CNN
	1    4250 5950
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P1
U 1 1 58BA16AD
P 3050 3200
F 0 "P1" H 3050 3450 50  0000 C CNN
F 1 "CONN_01X04" V 3150 3200 50  0000 C CNN
F 2 "goldentee_topper_base:CONN_0.156" H 3050 3200 50  0001 C CNN
F 3 "" H 3050 3200 50  0000 C CNN
	1    3050 3200
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X16 P2
U 1 1 58BA16C6
P 3050 4550
F 0 "P2" H 3050 5400 50  0000 C CNN
F 1 "CONN_01X16" V 3150 4550 50  0000 C CNN
F 2 "goldentee_topper_base:CONN_1x16_0.156" H 3050 4550 50  0001 C CNN
F 3 "" H 3050 4550 50  0000 C CNN
	1    3050 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 3350 2350 3350
Wire Wire Line
	2850 3050 2350 3050
$Comp
L +12V #PWR03
U 1 1 58BA1741
P 2350 3250
F 0 "#PWR03" H 2350 3100 50  0001 C CNN
F 1 "+12V" V 2350 3450 50  0000 C CNN
F 2 "" H 2350 3250 50  0000 C CNN
F 3 "" H 2350 3250 50  0000 C CNN
	1    2350 3250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2850 3150 2350 3150
Wire Wire Line
	2350 3250 2850 3250
$Comp
L GND #PWR04
U 1 1 58BA176A
P 2650 5300
F 0 "#PWR04" H 2650 5050 50  0001 C CNN
F 1 "GND" V 2650 5050 50  0000 C CNN
F 2 "" H 2650 5300 50  0000 C CNN
F 3 "" H 2650 5300 50  0000 C CNN
	1    2650 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	2650 5300 2850 5300
Wire Wire Line
	2850 5000 1650 5000
Wire Wire Line
	1550 3800 2850 3800
Text Label 1650 3900 0    60   ~ 0
ROW_1_8_PWR
Text Label 1650 3800 0    60   ~ 0
ROW_2_9_PWR
Text Label 1650 4000 0    60   ~ 0
ROW_3_10_PWR
Text Label 1650 4200 0    60   ~ 0
ROW_4_11_PWR
Text Label 1650 4400 0    60   ~ 0
ROW_5_12_PWR
Text Label 1650 4100 0    60   ~ 0
ROW_6_13_PWR
Text Label 1650 4300 0    60   ~ 0
ROW_7_14_PWR
Wire Wire Line
	1550 3900 2850 3900
Wire Wire Line
	1550 4000 2850 4000
Wire Wire Line
	1550 4100 2850 4100
Wire Wire Line
	1550 4200 2850 4200
Wire Wire Line
	1550 4300 2850 4300
Wire Wire Line
	1550 4400 2850 4400
Text Label 1650 4500 0    60   ~ 0
#OUTPUT_ON_5V
Text Label 1650 4600 0    60   ~ 0
RCK_ALL_5V
Text Label 1650 4700 0    60   ~ 0
BOTTOM_DATA_5V
Text Label 1650 4800 0    60   ~ 0
TOP_DATA_5V
Text Label 1650 4900 0    60   ~ 0
BOTTOM_CLK_5V
Text Label 1650 5100 0    60   ~ 0
TOP_CLK_5V
Wire Wire Line
	2650 5200 2850 5200
Wire Wire Line
	1550 4500 2850 4500
Wire Wire Line
	1550 4600 2850 4600
Wire Wire Line
	1550 4700 2850 4700
Wire Wire Line
	1550 4800 2850 4800
Wire Wire Line
	1550 4900 2850 4900
Wire Wire Line
	1550 5100 2850 5100
Text Label 9650 1600 2    60   ~ 0
ROW_1_8_PWR
$Comp
L R_Small R1
U 1 1 58BA2900
P 8400 1200
F 0 "R1" V 8300 1150 50  0000 L CNN
F 1 "47K" V 8350 1150 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 8400 1200 50  0001 C CNN
F 3 "" H 8400 1200 50  0000 C CNN
	1    8400 1200
	0    1    1    0   
$EndComp
Text Label 9650 3050 2    60   ~ 0
ROW_3_10_PWR
$Comp
L GND #PWR05
U 1 1 58BD97C4
P 1750 1500
F 0 "#PWR05" H 1750 1250 50  0001 C CNN
F 1 "GND" H 1750 1350 50  0000 C CNN
F 2 "" H 1750 1500 50  0000 C CNN
F 3 "" H 1750 1500 50  0000 C CNN
	1    1750 1500
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR06
U 1 1 58BD9897
P 2200 1200
F 0 "#PWR06" H 2200 1050 50  0001 C CNN
F 1 "+5V" H 2200 1340 50  0000 C CNN
F 2 "" H 2200 1200 50  0000 C CNN
F 3 "" H 2200 1200 50  0000 C CNN
	1    2200 1200
	1    0    0    -1  
$EndComp
$Comp
L C_Small C3
U 1 1 58BD9A8A
P 1150 1300
F 0 "C3" H 1160 1370 50  0000 L CNN
F 1 "10uF 25V" H 1160 1220 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210_HandSoldering" H 1150 1300 50  0001 C CNN
F 3 "" H 1150 1300 50  0000 C CNN
	1    1150 1300
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 58BD9B0C
P 2200 1300
F 0 "C4" H 2210 1370 50  0000 L CNN
F 1 "10uF 25V" H 2210 1220 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210_HandSoldering" H 2200 1300 50  0001 C CNN
F 3 "" H 2200 1300 50  0000 C CNN
	1    2200 1300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 58BD9C3D
P 2200 1400
F 0 "#PWR07" H 2200 1150 50  0001 C CNN
F 1 "GND" H 2200 1250 50  0000 C CNN
F 2 "" H 2200 1400 50  0000 C CNN
F 3 "" H 2200 1400 50  0000 C CNN
	1    2200 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 58BD9C99
P 1150 1400
F 0 "#PWR08" H 1150 1150 50  0001 C CNN
F 1 "GND" H 1150 1250 50  0000 C CNN
F 2 "" H 1150 1400 50  0000 C CNN
F 3 "" H 1150 1400 50  0000 C CNN
	1    1150 1400
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR09
U 1 1 58BD9CF5
P 1150 1200
F 0 "#PWR09" H 1150 1050 50  0001 C CNN
F 1 "+12V" H 1150 1340 50  0000 C CNN
F 2 "" H 1150 1200 50  0000 C CNN
F 3 "" H 1150 1200 50  0000 C CNN
	1    1150 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1200 2200 1200
Wire Wire Line
	1450 1200 1150 1200
Wire Wire Line
	3250 5950 3150 5950
$Comp
L GND #PWR010
U 1 1 58BDA8A9
P 5400 7200
F 0 "#PWR010" H 5400 6950 50  0001 C CNN
F 1 "GND" H 5400 7050 50  0000 C CNN
F 2 "" H 5400 7200 50  0000 C CNN
F 3 "" H 5400 7200 50  0000 C CNN
	1    5400 7200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 58BDA908
P 1500 7150
F 0 "#PWR011" H 1500 6900 50  0001 C CNN
F 1 "GND" H 1500 7000 50  0000 C CNN
F 2 "" H 1500 7150 50  0000 C CNN
F 3 "" H 1500 7150 50  0000 C CNN
	1    1500 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 6050 1500 6050
Wire Wire Line
	1500 6050 1500 7150
Wire Wire Line
	4150 6250 5400 6250
Text Label 7050 1400 0    60   ~ 0
ROW_1_8_SW
$Comp
L SN74LVC2T45 U2
U 1 1 58BE5B60
P 5050 1850
F 0 "U2" H 4700 2400 60  0000 C CNN
F 1 "SN74LVC2T45" H 4950 1800 60  0000 C CNN
F 2 "goldentee_topper_base:SN74LVC2T45DCTR" H 5150 1850 60  0001 C CNN
F 3 "" H 5150 1850 60  0001 C CNN
	1    5050 1850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 58BE5C1B
P 4450 1850
F 0 "#PWR012" H 4450 1600 50  0001 C CNN
F 1 "GND" H 4450 1700 50  0000 C CNN
F 2 "" H 4450 1850 50  0000 C CNN
F 3 "" H 4450 1850 50  0000 C CNN
	1    4450 1850
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR013
U 1 1 58BE5E42
P 4450 1150
F 0 "#PWR013" H 4450 1000 50  0001 C CNN
F 1 "+3V3" H 4450 1290 50  0000 C CNN
F 2 "" H 4450 1150 50  0000 C CNN
F 3 "" H 4450 1150 50  0000 C CNN
	1    4450 1150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR014
U 1 1 58BE5EA7
P 5650 1150
F 0 "#PWR014" H 5650 1000 50  0001 C CNN
F 1 "+5V" H 5650 1290 50  0000 C CNN
F 2 "" H 5650 1150 50  0000 C CNN
F 3 "" H 5650 1150 50  0000 C CNN
	1    5650 1150
	1    0    0    -1  
$EndComp
$Comp
L C_Small C2
U 1 1 58BE5F4D
P 5350 1150
F 0 "C2" V 5250 1050 50  0000 L CNN
F 1 "0.1uF 25V" V 5150 1050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5350 1150 50  0001 C CNN
F 3 "" H 5350 1150 50  0000 C CNN
	1    5350 1150
	0    1    1    0   
$EndComp
$Comp
L C_Small C1
U 1 1 58BE5FD9
P 4750 1150
F 0 "C1" V 4650 1050 50  0000 L CNN
F 1 "0.1uF 25V" V 4550 1050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4750 1150 50  0001 C CNN
F 3 "" H 4750 1150 50  0000 C CNN
	1    4750 1150
	0    1    1    0   
$EndComp
$Comp
L GND #PWR015
U 1 1 58BE6064
P 5050 1150
F 0 "#PWR015" H 5050 900 50  0001 C CNN
F 1 "GND" H 5050 1000 50  0000 C CNN
F 2 "" H 5050 1150 50  0000 C CNN
F 3 "" H 5050 1150 50  0000 C CNN
	1    5050 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 1450 4450 1150
Wire Wire Line
	4450 1150 4650 1150
Wire Wire Line
	4850 1150 5250 1150
Connection ~ 5050 1150
Wire Wire Line
	5450 1150 5650 1150
Wire Wire Line
	5650 1150 5650 1450
Wire Wire Line
	5650 1750 5750 1750
$Comp
L +3V3 #PWR016
U 1 1 58BE6F8F
P 5750 1750
F 0 "#PWR016" H 5750 1600 50  0001 C CNN
F 1 "+3V3" V 5750 1950 50  0000 C CNN
F 2 "" H 5750 1750 50  0000 C CNN
F 3 "" H 5750 1750 50  0000 C CNN
	1    5750 1750
	0    1    1    0   
$EndComp
$Comp
L SN74LVC2T45 U4
U 1 1 58BE732C
P 5050 2950
F 0 "U4" H 4700 3500 60  0000 C CNN
F 1 "SN74LVC2T45" H 4950 2900 60  0000 C CNN
F 2 "goldentee_topper_base:SN74LVC2T45DCTR" H 5150 2950 60  0001 C CNN
F 3 "" H 5150 2950 60  0001 C CNN
	1    5050 2950
	1    0    0    -1  
$EndComp
$Comp
L SN74LVC2T45 U5
U 1 1 58BE73AD
P 5050 4050
F 0 "U5" H 4700 4600 60  0000 C CNN
F 1 "SN74LVC2T45" H 4950 4000 60  0000 C CNN
F 2 "goldentee_topper_base:SN74LVC2T45DCTR" H 5150 4050 60  0001 C CNN
F 3 "" H 5150 4050 60  0001 C CNN
	1    5050 4050
	1    0    0    -1  
$EndComp
$Comp
L C_Small C7
U 1 1 58BE748D
P 4750 2250
F 0 "C7" V 4650 2150 50  0000 L CNN
F 1 "0.1uF 25V" V 4550 2150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4750 2250 50  0001 C CNN
F 3 "" H 4750 2250 50  0000 C CNN
	1    4750 2250
	0    1    1    0   
$EndComp
$Comp
L C_Small C9
U 1 1 58BE7506
P 4750 3350
F 0 "C9" V 4650 3250 50  0000 L CNN
F 1 "0.1uF 25V" V 4550 3250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4750 3350 50  0001 C CNN
F 3 "" H 4750 3350 50  0000 C CNN
	1    4750 3350
	0    1    1    0   
$EndComp
$Comp
L C_Small C8
U 1 1 58BE7580
P 5350 2250
F 0 "C8" V 5250 2150 50  0000 L CNN
F 1 "0.1uF 25V" V 5150 2150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5350 2250 50  0001 C CNN
F 3 "" H 5350 2250 50  0000 C CNN
	1    5350 2250
	0    1    1    0   
$EndComp
$Comp
L C_Small C10
U 1 1 58BE75FF
P 5350 3350
F 0 "C10" V 5250 3250 50  0000 L CNN
F 1 "0.1uF 25V" V 5150 3250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5350 3350 50  0001 C CNN
F 3 "" H 5350 3350 50  0000 C CNN
	1    5350 3350
	0    1    1    0   
$EndComp
$Comp
L GND #PWR017
U 1 1 58BE76FA
P 5050 2250
F 0 "#PWR017" H 5050 2000 50  0001 C CNN
F 1 "GND" H 5050 2100 50  0000 C CNN
F 2 "" H 5050 2250 50  0000 C CNN
F 3 "" H 5050 2250 50  0000 C CNN
	1    5050 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 58BE7765
P 5050 3350
F 0 "#PWR018" H 5050 3100 50  0001 C CNN
F 1 "GND" H 5050 3200 50  0000 C CNN
F 2 "" H 5050 3350 50  0000 C CNN
F 3 "" H 5050 3350 50  0000 C CNN
	1    5050 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 2250 5250 2250
Connection ~ 5050 2250
Wire Wire Line
	4650 2250 4450 2250
Wire Wire Line
	4450 2250 4450 2550
Wire Wire Line
	5450 2250 5650 2250
Wire Wire Line
	5650 2250 5650 2550
$Comp
L +3V3 #PWR019
U 1 1 58BE7C15
P 4450 2250
F 0 "#PWR019" H 4450 2100 50  0001 C CNN
F 1 "+3V3" H 4450 2390 50  0000 C CNN
F 2 "" H 4450 2250 50  0000 C CNN
F 3 "" H 4450 2250 50  0000 C CNN
	1    4450 2250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR020
U 1 1 58BE7C80
P 5650 2250
F 0 "#PWR020" H 5650 2100 50  0001 C CNN
F 1 "+5V" H 5650 2390 50  0000 C CNN
F 2 "" H 5650 2250 50  0000 C CNN
F 3 "" H 5650 2250 50  0000 C CNN
	1    5650 2250
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR021
U 1 1 58BE7DCB
P 4450 3350
F 0 "#PWR021" H 4450 3200 50  0001 C CNN
F 1 "+3V3" H 4450 3490 50  0000 C CNN
F 2 "" H 4450 3350 50  0000 C CNN
F 3 "" H 4450 3350 50  0000 C CNN
	1    4450 3350
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR022
U 1 1 58BE7E36
P 5650 3350
F 0 "#PWR022" H 5650 3200 50  0001 C CNN
F 1 "+5V" H 5650 3490 50  0000 C CNN
F 2 "" H 5650 3350 50  0000 C CNN
F 3 "" H 5650 3350 50  0000 C CNN
	1    5650 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 3650 5650 3350
Wire Wire Line
	5650 3350 5450 3350
Wire Wire Line
	4850 3350 5250 3350
Connection ~ 5050 3350
Wire Wire Line
	4650 3350 4450 3350
Wire Wire Line
	4450 3350 4450 3650
$Comp
L GND #PWR023
U 1 1 58BE822F
P 4450 2950
F 0 "#PWR023" H 4450 2700 50  0001 C CNN
F 1 "GND" H 4450 2800 50  0000 C CNN
F 2 "" H 4450 2950 50  0000 C CNN
F 3 "" H 4450 2950 50  0000 C CNN
	1    4450 2950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 58BE829A
P 4450 4050
F 0 "#PWR024" H 4450 3800 50  0001 C CNN
F 1 "GND" H 4450 3900 50  0000 C CNN
F 2 "" H 4450 4050 50  0000 C CNN
F 3 "" H 4450 4050 50  0000 C CNN
	1    4450 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 4050 4450 3950
Wire Wire Line
	4450 2950 4450 2850
$Comp
L +3V3 #PWR025
U 1 1 58BE849C
P 5750 2850
F 0 "#PWR025" H 5750 2700 50  0001 C CNN
F 1 "+3V3" V 5750 3050 50  0000 C CNN
F 2 "" H 5750 2850 50  0000 C CNN
F 3 "" H 5750 2850 50  0000 C CNN
	1    5750 2850
	0    1    1    0   
$EndComp
$Comp
L +3V3 #PWR026
U 1 1 58BE8507
P 5750 3950
F 0 "#PWR026" H 5750 3800 50  0001 C CNN
F 1 "+3V3" V 5750 4150 50  0000 C CNN
F 2 "" H 5750 3950 50  0000 C CNN
F 3 "" H 5750 3950 50  0000 C CNN
	1    5750 3950
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 3950 5750 3950
Wire Wire Line
	5650 2850 5750 2850
Wire Wire Line
	4450 1750 4450 1850
Text Label 6400 3750 2    60   ~ 0
#OUTPUT_ON_5V
Text Label 6400 3850 2    60   ~ 0
RCK_ALL_5V
Text Label 6400 2750 2    60   ~ 0
TOP_CLK_5V
Text Label 6400 2650 2    60   ~ 0
BOTTOM_CLK_5V
Text Label 6400 1650 2    60   ~ 0
TOP_DATA_5V
Text Label 6400 1550 2    60   ~ 0
BOTTOM_DATA_5V
Text Label 3650 1550 0    60   ~ 0
BOTTOM_DATA_3V3
Text Label 3650 1650 0    60   ~ 0
TOP_DATA_3V3
Text Label 3700 3750 0    60   ~ 0
#OUTPUT_ON_3V3
Text Label 3700 3850 0    60   ~ 0
RCK_ALL_3V3
Text Label 4200 6650 0    60   ~ 0
BOTTOM_DATA_3V3
Text Label 4200 6750 0    60   ~ 0
TOP_DATA_3V3
Text Label 4200 6450 0    60   ~ 0
#OUTPUT_ON_3V3
Text Label 4200 6550 0    60   ~ 0
RCK_ALL_3V3
Text Label 3650 2650 0    60   ~ 0
BOTTOM_CLK_3V3
Wire Wire Line
	3650 2650 4450 2650
Text Label 4200 6850 0    60   ~ 0
BOTTOM_CLK_3V3
Wire Wire Line
	4450 3850 3700 3850
Wire Wire Line
	3700 3750 4450 3750
Wire Wire Line
	5650 3750 6400 3750
Wire Wire Line
	5650 3850 6400 3850
Wire Wire Line
	6400 2750 5650 2750
Wire Wire Line
	5650 2650 6400 2650
Wire Wire Line
	6400 1650 5650 1650
Wire Wire Line
	5650 1550 6400 1550
Wire Wire Line
	3650 1650 4450 1650
Wire Wire Line
	4450 1550 3650 1550
$Comp
L GND #PWR027
U 1 1 58BEC3A7
P 8000 1600
F 0 "#PWR027" H 8000 1350 50  0001 C CNN
F 1 "GND" H 8000 1450 50  0000 C CNN
F 2 "" H 8000 1600 50  0000 C CNN
F 3 "" H 8000 1600 50  0000 C CNN
	1    8000 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 1200 9650 1200
Wire Wire Line
	8000 1200 8300 1200
Connection ~ 8200 1200
Wire Wire Line
	7700 1400 7050 1400
Wire Wire Line
	8200 1200 8200 1400
Wire Wire Line
	8200 1400 8400 1400
$Comp
L R_Small R5
U 1 1 58BEE505
P 8400 2650
F 0 "R5" V 8300 2600 50  0000 L CNN
F 1 "47K" V 8350 2600 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 8400 2650 50  0001 C CNN
F 3 "" H 8400 2650 50  0000 C CNN
	1    8400 2650
	0    1    1    0   
$EndComp
Text Label 7050 2850 0    60   ~ 0
ROW_3_10_SW
Wire Wire Line
	7050 2850 7700 2850
Wire Wire Line
	8000 2650 8300 2650
Wire Wire Line
	8500 2650 9650 2650
Wire Wire Line
	8400 2850 8200 2850
Wire Wire Line
	8200 2850 8200 2650
Connection ~ 8200 2650
$Comp
L GND #PWR028
U 1 1 58BEE90D
P 8000 3050
F 0 "#PWR028" H 8000 2800 50  0001 C CNN
F 1 "GND" H 8000 2900 50  0000 C CNN
F 2 "" H 8000 3050 50  0000 C CNN
F 3 "" H 8000 3050 50  0000 C CNN
	1    8000 3050
	1    0    0    -1  
$EndComp
Text Label 9650 2350 2    60   ~ 0
ROW_2_9_PWR
$Comp
L R_Small R3
U 1 1 58BEFCF0
P 8400 1950
F 0 "R3" V 8300 1900 50  0000 L CNN
F 1 "47K" V 8350 1900 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 8400 1950 50  0001 C CNN
F 3 "" H 8400 1950 50  0000 C CNN
	1    8400 1950
	0    1    1    0   
$EndComp
Text Label 7050 2150 0    60   ~ 0
ROW_2_9_SW
Wire Wire Line
	7050 2150 7700 2150
Wire Wire Line
	8000 1950 8300 1950
Wire Wire Line
	8500 1950 9650 1950
Wire Wire Line
	8400 2150 8200 2150
Wire Wire Line
	8200 2150 8200 1950
Connection ~ 8200 1950
$Comp
L GND #PWR029
U 1 1 58BEFD03
P 8000 2350
F 0 "#PWR029" H 8000 2100 50  0001 C CNN
F 1 "GND" H 8000 2200 50  0000 C CNN
F 2 "" H 8000 2350 50  0000 C CNN
F 3 "" H 8000 2350 50  0000 C CNN
	1    8000 2350
	1    0    0    -1  
$EndComp
Text Label 9650 3750 2    60   ~ 0
ROW_4_11_PWR
$Comp
L R_Small R7
U 1 1 58BEFDA0
P 8400 3350
F 0 "R7" V 8300 3300 50  0000 L CNN
F 1 "47K" V 8350 3300 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 8400 3350 50  0001 C CNN
F 3 "" H 8400 3350 50  0000 C CNN
	1    8400 3350
	0    1    1    0   
$EndComp
Text Label 7050 3550 0    60   ~ 0
ROW_4_11_SW
Wire Wire Line
	7050 3550 7700 3550
Wire Wire Line
	8000 3350 8300 3350
Wire Wire Line
	8500 3350 9650 3350
Wire Wire Line
	8400 3550 8200 3550
Wire Wire Line
	8200 3550 8200 3350
Connection ~ 8200 3350
$Comp
L GND #PWR030
U 1 1 58BEFDB3
P 8000 3750
F 0 "#PWR030" H 8000 3500 50  0001 C CNN
F 1 "GND" H 8000 3600 50  0000 C CNN
F 2 "" H 8000 3750 50  0000 C CNN
F 3 "" H 8000 3750 50  0000 C CNN
	1    8000 3750
	1    0    0    -1  
$EndComp
Text Label 9650 4450 2    60   ~ 0
ROW_5_12_PWR
$Comp
L R_Small R12
U 1 1 58BEFED0
P 8400 4050
F 0 "R12" V 8300 4000 50  0000 L CNN
F 1 "47K" V 8350 4000 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 8400 4050 50  0001 C CNN
F 3 "" H 8400 4050 50  0000 C CNN
	1    8400 4050
	0    1    1    0   
$EndComp
Text Label 7050 4250 0    60   ~ 0
ROW_5_12_SW
Wire Wire Line
	7050 4250 7700 4250
Wire Wire Line
	8000 4050 8300 4050
Wire Wire Line
	8500 4050 9650 4050
Wire Wire Line
	8400 4250 8200 4250
Wire Wire Line
	8200 4250 8200 4050
Connection ~ 8200 4050
$Comp
L GND #PWR031
U 1 1 58BEFEE3
P 8000 4450
F 0 "#PWR031" H 8000 4200 50  0001 C CNN
F 1 "GND" H 8000 4300 50  0000 C CNN
F 2 "" H 8000 4450 50  0000 C CNN
F 3 "" H 8000 4450 50  0000 C CNN
	1    8000 4450
	1    0    0    -1  
$EndComp
Text Label 9650 5150 2    60   ~ 0
ROW_6_13_PWR
$Comp
L R_Small R21
U 1 1 58BEFF74
P 8400 4750
F 0 "R21" V 8300 4700 50  0000 L CNN
F 1 "47K" V 8350 4700 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 8400 4750 50  0001 C CNN
F 3 "" H 8400 4750 50  0000 C CNN
	1    8400 4750
	0    1    1    0   
$EndComp
Text Label 7050 4950 0    60   ~ 0
ROW_6_13_SW
Wire Wire Line
	7050 4950 7700 4950
Wire Wire Line
	8000 4750 8300 4750
Wire Wire Line
	8500 4750 9650 4750
Wire Wire Line
	8400 4950 8200 4950
Wire Wire Line
	8200 4950 8200 4750
Connection ~ 8200 4750
$Comp
L GND #PWR032
U 1 1 58BEFF87
P 8000 5150
F 0 "#PWR032" H 8000 4900 50  0001 C CNN
F 1 "GND" H 8000 5000 50  0000 C CNN
F 2 "" H 8000 5150 50  0000 C CNN
F 3 "" H 8000 5150 50  0000 C CNN
	1    8000 5150
	1    0    0    -1  
$EndComp
Text Label 9650 5850 2    60   ~ 0
ROW_7_14_PWR
$Comp
L R_Small R26
U 1 1 58BF0048
P 8400 5450
F 0 "R26" V 8300 5400 50  0000 L CNN
F 1 "47K" V 8350 5400 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 8400 5450 50  0001 C CNN
F 3 "" H 8400 5450 50  0000 C CNN
	1    8400 5450
	0    1    1    0   
$EndComp
Text Label 7050 5650 0    60   ~ 0
ROW_7_14_SW
Wire Wire Line
	7050 5650 7700 5650
Wire Wire Line
	8000 5450 8300 5450
Wire Wire Line
	8500 5450 9650 5450
Wire Wire Line
	8400 5650 8200 5650
Wire Wire Line
	8200 5650 8200 5450
Connection ~ 8200 5450
$Comp
L GND #PWR033
U 1 1 58BF005B
P 8000 5850
F 0 "#PWR033" H 8000 5600 50  0001 C CNN
F 1 "GND" H 8000 5700 50  0000 C CNN
F 2 "" H 8000 5850 50  0000 C CNN
F 3 "" H 8000 5850 50  0000 C CNN
	1    8000 5850
	1    0    0    -1  
$EndComp
Text Label 2300 6950 0    60   ~ 0
ROW_1_8_SW
Text Label 2300 7050 0    60   ~ 0
ROW_2_9_SW
Text Label 2300 6850 0    60   ~ 0
ROW_3_10_SW
Text Label 2300 6650 0    60   ~ 0
ROW_4_11_SW
Text Label 2300 6450 0    60   ~ 0
ROW_5_12_SW
Text Label 2300 6750 0    60   ~ 0
ROW_6_13_SW
Text Label 2300 6550 0    60   ~ 0
ROW_7_14_SW
$Comp
L R_Small R29
U 1 1 58BF0C64
P 5250 6450
F 0 "R29" V 5200 6250 50  0000 L CNN
F 1 "47K" V 5250 6400 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5250 6450 50  0001 C CNN
F 3 "" H 5250 6450 50  0000 C CNN
	1    5250 6450
	0    1    1    0   
$EndComp
$Comp
L R_Small R31
U 1 1 58BF0FB2
P 5250 6550
F 0 "R31" V 5200 6350 50  0000 L CNN
F 1 "47K" V 5250 6500 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5250 6550 50  0001 C CNN
F 3 "" H 5250 6550 50  0000 C CNN
	1    5250 6550
	0    1    1    0   
$EndComp
$Comp
L R_Small R33
U 1 1 58BF108C
P 5250 6650
F 0 "R33" V 5200 6450 50  0000 L CNN
F 1 "47K" V 5250 6600 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5250 6650 50  0001 C CNN
F 3 "" H 5250 6650 50  0000 C CNN
	1    5250 6650
	0    1    1    0   
$EndComp
$Comp
L R_Small R35
U 1 1 58BF11A8
P 5250 6750
F 0 "R35" V 5200 6550 50  0000 L CNN
F 1 "47K" V 5250 6700 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5250 6750 50  0001 C CNN
F 3 "" H 5250 6750 50  0000 C CNN
	1    5250 6750
	0    1    1    0   
$EndComp
$Comp
L R_Small R37
U 1 1 58BF11AE
P 5250 6850
F 0 "R37" V 5200 6650 50  0000 L CNN
F 1 "47K" V 5250 6800 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5250 6850 50  0001 C CNN
F 3 "" H 5250 6850 50  0000 C CNN
	1    5250 6850
	0    1    1    0   
$EndComp
$Comp
L R_Small R39
U 1 1 58BF11B4
P 5250 6950
F 0 "R39" V 5200 6750 50  0000 L CNN
F 1 "47K" V 5250 6900 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5250 6950 50  0001 C CNN
F 3 "" H 5250 6950 50  0000 C CNN
	1    5250 6950
	0    1    1    0   
$EndComp
$Comp
L R_Small R40
U 1 1 58BF123E
P 2050 6450
F 0 "R40" V 2000 6250 50  0000 L CNN
F 1 "47K" V 2050 6400 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2050 6450 50  0001 C CNN
F 3 "" H 2050 6450 50  0000 C CNN
	1    2050 6450
	0    1    1    0   
$EndComp
Wire Wire Line
	5400 6250 5400 7200
Wire Wire Line
	4150 6450 5150 6450
Wire Wire Line
	4150 6550 5150 6550
Wire Wire Line
	4150 6650 5150 6650
Wire Wire Line
	4150 6750 5150 6750
Wire Wire Line
	4150 6850 5150 6850
Wire Wire Line
	4150 6950 5150 6950
Wire Wire Line
	5350 6450 5400 6450
Connection ~ 5400 6450
Wire Wire Line
	5350 6550 5400 6550
Connection ~ 5400 6550
Wire Wire Line
	5350 6650 5400 6650
Connection ~ 5400 6650
Wire Wire Line
	5350 6750 5400 6750
Connection ~ 5400 6750
Wire Wire Line
	5350 6850 5400 6850
Connection ~ 5400 6850
Wire Wire Line
	5350 6950 5400 6950
Connection ~ 5400 6950
Wire Wire Line
	4150 5950 4250 5950
$Comp
L R_Small R30
U 1 1 58BF951A
P 2050 6550
F 0 "R30" V 2000 6350 50  0000 L CNN
F 1 "47K" V 2050 6500 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2050 6550 50  0001 C CNN
F 3 "" H 2050 6550 50  0000 C CNN
	1    2050 6550
	0    1    1    0   
$EndComp
$Comp
L R_Small R32
U 1 1 58BF960C
P 2050 6650
F 0 "R32" V 2000 6450 50  0000 L CNN
F 1 "47K" V 2050 6600 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2050 6650 50  0001 C CNN
F 3 "" H 2050 6650 50  0000 C CNN
	1    2050 6650
	0    1    1    0   
$EndComp
$Comp
L R_Small R34
U 1 1 58BF96D6
P 2050 6750
F 0 "R34" V 2000 6550 50  0000 L CNN
F 1 "47K" V 2050 6700 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2050 6750 50  0001 C CNN
F 3 "" H 2050 6750 50  0000 C CNN
	1    2050 6750
	0    1    1    0   
$EndComp
$Comp
L R_Small R36
U 1 1 58BF97A3
P 2050 6850
F 0 "R36" V 2000 6650 50  0000 L CNN
F 1 "47K" V 2050 6800 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2050 6850 50  0001 C CNN
F 3 "" H 2050 6850 50  0000 C CNN
	1    2050 6850
	0    1    1    0   
$EndComp
$Comp
L R_Small R38
U 1 1 58BF9871
P 2050 6950
F 0 "R38" V 2000 6750 50  0000 L CNN
F 1 "47K" V 2050 6900 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2050 6950 50  0001 C CNN
F 3 "" H 2050 6950 50  0000 C CNN
	1    2050 6950
	0    1    1    0   
$EndComp
Wire Wire Line
	2150 6550 3250 6550
Wire Wire Line
	1950 6550 1500 6550
Connection ~ 1500 6550
Wire Wire Line
	1950 6650 1500 6650
Connection ~ 1500 6650
Wire Wire Line
	1950 6750 1500 6750
Connection ~ 1500 6750
Wire Wire Line
	1950 6850 1500 6850
Connection ~ 1500 6850
Wire Wire Line
	1950 6950 1500 6950
Connection ~ 1500 6950
Wire Wire Line
	2150 6950 3250 6950
Wire Wire Line
	2150 6850 3250 6850
Wire Wire Line
	2150 6750 3250 6750
Wire Wire Line
	2150 6650 3250 6650
$Comp
L R_Small R24
U 1 1 58BFAA78
P 1450 5100
F 0 "R24" V 1400 4900 50  0000 L CNN
F 1 "47K" V 1450 5050 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 1450 5100 50  0001 C CNN
F 3 "" H 1450 5100 50  0000 C CNN
	1    1450 5100
	0    1    1    0   
$EndComp
$Comp
L R_Small R23
U 1 1 58BFAC72
P 1450 4900
F 0 "R23" V 1400 4700 50  0000 L CNN
F 1 "47K" V 1450 4850 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 1450 4900 50  0001 C CNN
F 3 "" H 1450 4900 50  0000 C CNN
	1    1450 4900
	0    1    1    0   
$EndComp
$Comp
L R_Small R22
U 1 1 58BFAD4B
P 1450 4800
F 0 "R22" V 1400 4600 50  0000 L CNN
F 1 "47K" V 1450 4750 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 1450 4800 50  0001 C CNN
F 3 "" H 1450 4800 50  0000 C CNN
	1    1450 4800
	0    1    1    0   
$EndComp
$Comp
L R_Small R20
U 1 1 58BFAEF4
P 1450 4700
F 0 "R20" V 1400 4500 50  0000 L CNN
F 1 "47K" V 1450 4650 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 1450 4700 50  0001 C CNN
F 3 "" H 1450 4700 50  0000 C CNN
	1    1450 4700
	0    1    1    0   
$EndComp
$Comp
L R_Small R19
U 1 1 58BFAFD1
P 1450 4600
F 0 "R19" V 1400 4400 50  0000 L CNN
F 1 "47K" V 1450 4550 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 1450 4600 50  0001 C CNN
F 3 "" H 1450 4600 50  0000 C CNN
	1    1450 4600
	0    1    1    0   
$EndComp
$Comp
L R_Small R18
U 1 1 58BFB1CA
P 1450 4500
F 0 "R18" V 1400 4300 50  0000 L CNN
F 1 "47K" V 1450 4450 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 1450 4500 50  0001 C CNN
F 3 "" H 1450 4500 50  0000 C CNN
	1    1450 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	1350 4500 1100 4500
Connection ~ 1100 4500
Wire Wire Line
	1350 4600 1100 4600
Connection ~ 1100 4600
Wire Wire Line
	1100 4700 1350 4700
Connection ~ 1100 4700
Wire Wire Line
	1100 4800 1350 4800
Connection ~ 1100 4800
Wire Wire Line
	1100 4900 1350 4900
Connection ~ 1100 4900
Wire Wire Line
	1100 5100 1350 5100
Connection ~ 1100 5100
$Comp
L photon_module M1
U 1 1 58BE0B56
P 3800 6550
F 0 "M1" H 3500 7300 60  0000 C CNN
F 1 "photon_module" H 3800 5900 60  0000 C CNN
F 2 "goldentee_topper_base:photon" H 4200 6150 60  0001 C CNN
F 3 "" H 4200 6150 60  0001 C CNN
	1    3800 6550
	1    0    0    -1  
$EndComp
$Comp
L Q_PMOS_GDS Q2
U 1 1 58BE2FB7
P 8600 1400
F 0 "Q2" H 8800 1450 50  0000 L CNN
F 1 "Q_PMOS_GDS" H 8800 1350 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H 8800 1500 50  0001 C CNN
F 3 "" H 8600 1400 50  0000 C CNN
	1    8600 1400
	1    0    0    1   
$EndComp
Wire Wire Line
	8700 1600 9650 1600
$Comp
L Q_PMOS_GDS Q4
U 1 1 58BE7291
P 8600 2150
F 0 "Q4" H 8800 2200 50  0000 L CNN
F 1 "Q_PMOS_GDS" H 8800 2100 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H 8800 2250 50  0001 C CNN
F 3 "" H 8600 2150 50  0000 C CNN
	1    8600 2150
	1    0    0    1   
$EndComp
$Comp
L Q_PMOS_GDS Q6
U 1 1 58BE7373
P 8600 2850
F 0 "Q6" H 8800 2900 50  0000 L CNN
F 1 "Q_PMOS_GDS" H 8800 2800 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H 8800 2950 50  0001 C CNN
F 3 "" H 8600 2850 50  0000 C CNN
	1    8600 2850
	1    0    0    1   
$EndComp
$Comp
L Q_PMOS_GDS Q8
U 1 1 58BE7460
P 8600 3550
F 0 "Q8" H 8800 3600 50  0000 L CNN
F 1 "Q_PMOS_GDS" H 8800 3500 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H 8800 3650 50  0001 C CNN
F 3 "" H 8600 3550 50  0000 C CNN
	1    8600 3550
	1    0    0    1   
$EndComp
$Comp
L Q_PMOS_GDS Q10
U 1 1 58BE7DC2
P 8600 4250
F 0 "Q10" H 8800 4300 50  0000 L CNN
F 1 "Q_PMOS_GDS" H 8800 4200 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H 8800 4350 50  0001 C CNN
F 3 "" H 8600 4250 50  0000 C CNN
	1    8600 4250
	1    0    0    1   
$EndComp
$Comp
L Q_PMOS_GDS Q12
U 1 1 58BE7EA3
P 8600 4950
F 0 "Q12" H 8800 5000 50  0000 L CNN
F 1 "Q_PMOS_GDS" H 8800 4900 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H 8800 5050 50  0001 C CNN
F 3 "" H 8600 4950 50  0000 C CNN
	1    8600 4950
	1    0    0    1   
$EndComp
$Comp
L Q_PMOS_GDS Q14
U 1 1 58BE8089
P 8600 5650
F 0 "Q14" H 8800 5700 50  0000 L CNN
F 1 "Q_PMOS_GDS" H 8800 5600 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H 8800 5750 50  0001 C CNN
F 3 "" H 8600 5650 50  0000 C CNN
	1    8600 5650
	1    0    0    1   
$EndComp
Wire Wire Line
	9650 2350 8700 2350
Wire Wire Line
	8700 3050 9650 3050
Wire Wire Line
	9650 3750 8700 3750
Wire Wire Line
	8700 4450 9650 4450
Wire Wire Line
	9650 5150 8700 5150
Wire Wire Line
	9650 5850 8700 5850
Wire Wire Line
	1100 4500 1100 5300
$Comp
L R_Small R2
U 1 1 58BEE9CE
P 2050 7050
F 0 "R2" V 2000 6850 50  0000 L CNN
F 1 "47K" V 2050 7000 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2050 7050 50  0001 C CNN
F 3 "" H 2050 7050 50  0000 C CNN
	1    2050 7050
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 7050 1500 7050
Connection ~ 1500 7050
Wire Wire Line
	2150 7050 3250 7050
Text Label 4200 6950 0    60   ~ 0
TOP_CLK_3V3
Text Label 3650 2750 0    60   ~ 0
TOP_CLK_3V3
Wire Wire Line
	3650 2750 4450 2750
Text Label 2350 3050 0    60   ~ 0
LED_GND
Text Label 1650 5000 0    60   ~ 0
LED_GND
$Comp
L GND #PWR034
U 1 1 58BEFB1D
P 1100 5300
F 0 "#PWR034" H 1100 5050 50  0001 C CNN
F 1 "GND" H 1100 5150 50  0000 C CNN
F 2 "" H 1100 5300 50  0000 C CNN
F 3 "" H 1100 5300 50  0000 C CNN
	1    1100 5300
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR035
U 1 1 58BEFD0A
P 2650 5200
F 0 "#PWR035" H 2650 5050 50  0001 C CNN
F 1 "+12V" V 2650 5450 50  0000 C CNN
F 2 "" H 2650 5200 50  0000 C CNN
F 3 "" H 2650 5200 50  0000 C CNN
	1    2650 5200
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR036
U 1 1 58BF050F
P 2350 3350
F 0 "#PWR036" H 2350 3100 50  0001 C CNN
F 1 "GND" V 2350 3150 50  0000 C CNN
F 2 "" H 2350 3350 50  0000 C CNN
F 3 "" H 2350 3350 50  0000 C CNN
	1    2350 3350
	0    1    1    0   
$EndComp
Text Label 2350 3150 0    60   ~ 0
LED_PWR
Text Label 9650 1200 2    60   ~ 0
LED_PWR
Connection ~ 8700 1200
Text Label 9650 1950 2    60   ~ 0
LED_PWR
Text Label 9650 2650 2    60   ~ 0
LED_PWR
Text Label 9650 3350 2    60   ~ 0
LED_PWR
Text Label 9650 4050 2    60   ~ 0
LED_PWR
Text Label 9650 4750 2    60   ~ 0
LED_PWR
Text Label 9650 5450 2    60   ~ 0
LED_PWR
Connection ~ 8700 5450
Connection ~ 8700 4750
Connection ~ 8700 4050
Connection ~ 8700 3350
Connection ~ 8700 1950
Connection ~ 8700 2650
Wire Wire Line
	2150 6450 3250 6450
Wire Wire Line
	1950 6450 1500 6450
Connection ~ 1500 6450
$Comp
L NCP1117 U1
U 1 1 58BF9190
P 1750 1200
F 0 "U1" H 1750 1425 50  0000 C CNN
F 1 "NCP1117" H 1750 1350 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 1800 950 50  0001 L CNN
F 3 "" H 1750 1200 50  0000 C CNN
	1    1750 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1300 2100 1300
Wire Wire Line
	2100 1300 2100 1200
Connection ~ 2100 1200
$Comp
L CONN_01X06 P3
U 1 1 58BF8C75
P 5300 5000
F 0 "P3" H 5300 5350 50  0000 C CNN
F 1 "CONN_01X06" V 5400 5000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 5300 5000 50  0001 C CNN
F 3 "" H 5300 5000 50  0000 C CNN
	1    5300 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR037
U 1 1 58BF8D84
P 5100 4750
F 0 "#PWR037" H 5100 4500 50  0001 C CNN
F 1 "GND" V 5100 4500 50  0000 C CNN
F 2 "" H 5100 4750 50  0000 C CNN
F 3 "" H 5100 4750 50  0000 C CNN
	1    5100 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	3250 6250 2250 6250
Wire Wire Line
	3250 6150 2250 6150
Wire Wire Line
	5100 5050 4300 5050
Wire Wire Line
	5100 5150 4300 5150
Text Label 4300 5150 0    60   ~ 0
UART_RXD
Text Label 4300 5050 0    60   ~ 0
UART_TXD
Text Label 2250 6150 0    60   ~ 0
UART_RXD
Text Label 2250 6250 0    60   ~ 0
UART_TXD
$Comp
L Q_NMOS_GSD Q1
U 1 1 58C1F2CD
P 7900 1400
F 0 "Q1" H 7650 1700 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 7650 1650 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 8100 1500 50  0001 C CNN
F 3 "" H 7900 1400 50  0000 C CNN
	1    7900 1400
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q3
U 1 1 58C2035C
P 7900 2150
F 0 "Q3" H 7650 2450 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 7650 2400 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 8100 2250 50  0001 C CNN
F 3 "" H 7900 2150 50  0000 C CNN
	1    7900 2150
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q5
U 1 1 58C20EFC
P 7900 2850
F 0 "Q5" H 7650 3150 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 7650 3100 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 8100 2950 50  0001 C CNN
F 3 "" H 7900 2850 50  0000 C CNN
	1    7900 2850
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q7
U 1 1 58C20FB6
P 7900 3550
F 0 "Q7" H 7650 3850 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 7650 3800 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 8100 3650 50  0001 C CNN
F 3 "" H 7900 3550 50  0000 C CNN
	1    7900 3550
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q9
U 1 1 58C2106E
P 7900 4250
F 0 "Q9" H 7650 4550 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 7650 4500 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 8100 4350 50  0001 C CNN
F 3 "" H 7900 4250 50  0000 C CNN
	1    7900 4250
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q11
U 1 1 58C2112A
P 7900 4950
F 0 "Q11" H 7650 5250 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 7650 5200 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 8100 5050 50  0001 C CNN
F 3 "" H 7900 4950 50  0000 C CNN
	1    7900 4950
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q13
U 1 1 58C21290
P 7900 5650
F 0 "Q13" H 7650 5950 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 7650 5900 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 8100 5750 50  0001 C CNN
F 3 "" H 7900 5650 50  0000 C CNN
	1    7900 5650
	1    0    0    -1  
$EndComp
$EndSCHEMATC
