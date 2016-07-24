	  ______                              _
	 / _____)             _              | |
	( (____  _____ ____ _| |_ _____  ____| |__
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2014 Semtech-Cycleo

Background Spectral Scan for LoRa gateway
=========================================


1. Introduction
----------------

This software is used to scan the spectral band where the LoRa gateway operates.
It simply computes a RSSI histogram on several frequencies, that will help to
detect occupied bands and get interferer profiles.
It logs the histogram in a .csv file.

This utility program is meant to run on the LoRa gateway reference design
SX1301AP2 (with FPGA and additionnal SX127x).

The background RSSI scan is a diagnostic tool and must be run on top of the
gateway activity. Moreover the two SX1257 radios have to be configured in RX
mode to optimize the matching impedance with SX127x. The 32MHz clock provided
to the SX127x is available once SX1301 has enabled the two SX1257 radios, so
the background RSSI scan must be launched after the packet forwarder.

2. Command line options
------------------------

`-h`
will display a short help

`-f start:step:stop`
Frequency vector to scan in MHz: start:step:stop
Valid range: start > 800, step > 0.005, stop < 1000

`-n`
Total number of RSSI points.
Valid range: [1,65535]

`-r`
Divide factor of RSSI sampling rate, which is 32MHz/(div+1030)
Valid range: [1,65535]

`-l`
Log file name


3. Use
-------

The format of the log file is the following:
Freq_1, RSSI_1, histo_1, ...., RSSI_128, histo_128
Freq_2, RSSI_1, histo_1, ...., RSSI_128, histo_128
...

RSSI_n is the nth value of RSSI in dBm, starting at -142 dBm

Default setup:
- freq 863 : 0.2 : 870
- 65535 RSSI points in total at 32kHz rate
- FTDI SPI interface

Example with frequencies from 865 MHz to 870 MHz, by step of 1MHz,
2500 RSSI points processed at 10kHz rate, saved in "log.csv":
./util_spectral_scan -f 865:1:870 -n 2500 -r 20170 -l "log"
