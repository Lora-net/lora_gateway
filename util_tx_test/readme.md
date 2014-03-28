	 / _____)             _              | |    
	( (____  _____ ____ _| |_ _____  ____| |__  
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \ 
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2013 Semtech-Cycleo

LoRa concentrator packet sender
================================

1. Introduction
----------------

This software is used to send test packets with a LoRa concentrator. The packets
contain little information, on no protocol (ie. MAC address) information but
can be used to assess the functionality of a gateway downlink using other
gateways as receivers.

2. Dependencies
----------------

This program is a typical example of LoRa concentrator HAL usage for sending
packets.

Only high-level functions are used (the ones contained in loragw_hal) so there
is no hardware dependencies assuming the HAL is matched with the proper version
of the hardware.
Data structures of the sent packets are accessed by name (ie. not at a
binary level) so new functionalities can be added to the API without affecting
that program at all.

It was tested with v1.3.0 of the libloragw library, and should be compatible
with any later version of the library assuming the API is downward-compatible.

3. Usage
---------

The application runs until the specified number of packets have been sent.
Press Ctrl+C to stop the application before that.

Use the -f option followed by a real number (decimal point and scientific
'E notation' are OK) to specify the modulation central frequency.

Use the -s option to specify the Spreading Factor of LoRa modulation (values 7
to 12 are valid).

Use the -b option to set LoRa modulation bandwidth in kHz (accepted values: 125,
250 or 500).

Use the -p option to set the concentrator TX power in dBm. Not all values are
supported by hardware (typically 14 et 20 dBm are supported, other values might
not give expected power). Check with a RF power meter before connecting any
sensitive equipment.

Use the -t option to specify the number of milliseconds of pause between
packets. Using zero will result in a quasi-continuous emission.

Use the -x option to specify how many packets should be sent.

Use the -i option to invert the LoRa modulation polarity.

The packets are 20 bytes long, and protected by the smallest supported ECC.

The payload content is:
[T][E][S][T][packet counter MSB][packet counter LSB] followed by ASCII padding.
All LoRa data is whitened, so the padding has no influence whatsoever on the
packet error rate.

4. License
-----------

Copyright (c) 2013, SEMTECH S.A.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the Semtech corporation nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL SEMTECH S.A. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*EOF*