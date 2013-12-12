	 / _____)             _              | |    
	( (____  _____ ____ _| |_ _____  ____| |__  
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \ 
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
		©2013 Semtech-Cycleo

Lora Gateway packet logger
===========================

1. Introduction
----------------

This software is used to set up a Lora concentrator using a JSON configuration
file and then record all the packets received in a log file, indefinitely, until
the user stops the application.
No filtering is done and all packets that are Lora packets with the correct RF
parameters (frequency, datarate, bandwidth) should appear in the log.

2. Dependencies
----------------

This program uses the Parson library (http://kgabis.github.com/parson/) by
Krzysztof Gabis for JSON parsing.
Many thanks to him for that very practical and well written library.

This program is a typical example of Lora Gateway HAL usage for receiving
packets.

Only high-level functions are used (the ones contained in loragw_hal) so there
is no hardware dependencies assuming the HAL is matched with the proper version
of the hardware.
Data structures of the received packets are accessed by name (ie. not at a
binary level) so new functionalities can be added to the API without affecting
that program at all.

It was tested with v1.0.0 of the libloragw library, and should be compatible
with any later version of the library assuming the API is downward-compatible.

3. Usage
---------

To stop the application, press Ctrl+C.

The only optional parameter when launching the application is the log rotation
time (in seconds).

The way the program takes configuration files into account is the following:
 * if there is a debug_conf.json parse it, others are ignored
 * if there is a global_conf.json parse it, look for the next file
 * if there is a local_conf.json parse it
If some parameters are defined in both global and local configuration files, the
local definition overwrites the global definition.

The global configuration file should be exactly the same throughout your
network, contain all global parameters (parameters for "sensor" radio channels)
and preferably default "safe" values for parameters that are specific for each
gateway (eg. specify a default MAC address).

The local configuration file should contain parameters that are specific to each
gateway (eg. MAC address, frequency for backhaul radio channels).

In each configuration file, the program looks for a JSON object named
"SX1301_conf" that should contain the parameters for the Lora concentrator board
(RF channels definition, modem parameters, etc) and another JSON object called
"gateway_conf" that should contain the gateway parameters (gateway MAC address,
IP address of the Lora MAC controller, network authentication parameters, etc).

To learn more about the JSON configuration format, read the provided JSON files
and the API documentation. A dedicated document will be available later on.

The received packets are put in a CSV file whose name include the MAC address of
the gateway in hexadecimal format and a UTC timestamp of log starting time in
ISO 8601 recommended compact format:
yyyymmddThhmmssZ (eg. 20131009T172345Z for October 9th, 2013 at 5:23:45PM UTC)

To able continuous monitoring, the current log file is closed is closed and a
new one is opened every hour (by default, rotation interval is settable by the
user using -r command line option).
No packet is lost during that rotation of log file.
Every log file but the current one can then be modified, uploaded and/or deleted
without any consequence for the program execution.

4. Changelog
-------------

2013-10-24, v1
Initial version.
