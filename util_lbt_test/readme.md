	 / _____)             _              | |    
	( (____  _____ ____ _| |_ _____  ____| |__  
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \ 
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
		©2013 Semtech-Cycleo

Lora Gateway LBT basic test application
=======================================

1. Introduction
----------------

This software configures the FPGA for "Liste-Before-Talk" feature and
continuously reads the LBT channels timestamps which indicate when was the last
instant when the channel was free.

2. Dependencies
----------------

A SX1301AP2 Ref Design board with its FPGA programmed with LBT feature

3. Usage
---------

Before running the util_lbt_test application, the concentrator MUST be first
started with the HAL, using for example util_pkt_logger or the packet forwarder
with LBT feature disabled, as the util_lbt_test will configure it.

For a description of the command line options available:
./util_lbt_test -h

ex:
./util_lbt_test -f 867.1 -r -80 -s 5000

This will set 8 LBT channels, starting from 867.1 MHz, then each subsequent
channel being set to the frequency of the previous channel +200 KHz (867.3,
867.5, ...).

The above test will run for ever, with a CHANNEL_SCAN_TIME of 5000µs
and a target RSSI of -80dBm.

Please refer to the lora_gateway library readme.md to get more details on the
LBT feature implementation and configuration.

4. Changelog
-------------

2016-03-03	v1.0	Initial version
2016-08-31	v1.1	LBT feature rework
