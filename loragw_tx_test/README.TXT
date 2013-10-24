	 / _____)             _              | |    
	( (____  _____ ____ _| |_ _____  ____| |__  
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \ 
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
		©2013 Semtech-Cycleo

Lora Gateway packet sender
===========================

1. Introduction
----------------

This software is used to send test packets with a Lora concentrator. The packets
contain little information, on no protocol (ie. MAC address) information but
can be used to assess the functionality of a gateway downlink using other
gateways as receivers.

2. Dependencies
----------------

This program is a typical example of Lora Gateway HAL usage for sending packets.

Only high-level functions are used (the ones contained in loragw_hal) so there
is no hardware dependencies assuming the HAL is matched with the proper version
of the hardware.
Data structures of the sent packets are accessed by name (ie. not at a
binary level) so new functionalities can be added to the API without affecting
that program at all.

It was tested with v1.0.0 of the libloragw library, and should be compatible
with any later version of the library assuming the API is downward-compatible.

3. Usage
---------

The application runs until the specified number of packets have been send.
Press Ctrl+C to stop the application before that.

Use the -f option followed by a real number (decimal point and scientific
'E notation' are OK) to specify the modulation central frequency.
Use the -s option to specify the Spreading Factor of Lora modulation (values 7
to 12 are valid).
Use the -b option to set Lora modulation bandwidth in kHz (accepted values: 125,
250 or 500).
Use the -p option to set the concentrator TX power in dBm. Not all values are
supported by hardware (typically 14 et 20 dBm are supported, other values might
not give expected power). Check with a RF power meter before connecting any
sensitive equipment.
Use the -t option to specify the number of milliseconds of pause between
packets. Using zero will result in a quasi-continuous emission.
Use the -x to specify how many packets should be sent.

The packets are 20 bytes long, and protected by the smallest supported ECC.

The payload content is:
[T][E][S][T][packet counter MSB][packet counter MSB] followed by ASCII padding.
All Lora data is whitened, so the padding has no influence whatsoever on the
packet error rate.

4. Changelog
-------------

2013-10-24, v1
Initial version.
