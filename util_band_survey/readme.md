	 / _____)             _              | |    
	( (____  _____ ____ _| |_ _____  ____| |__  
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \ 
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2013 Semtech-Cycleo

Gateway RF band survey application
===================================

1. Introduction
----------------

This software is used to scan the RF band and measure background RSSI and some
measurement of interferer pattern.
The RSSI is purposefully skew towards high values, to detect high power
interferers (eg. a gateway installed in the beam of a powerful base station),
and not to characterize the noise floor, or accurately measure the time-domain
profile of interferers.

2. Dependencies
----------------

This software call functions in the loragw_reg and loragw_reg sub-modules of
loragwlib. loragw_spi is used indirectly, and the loragw_hal sub-module is not
used at all, except for constants define at the top level.

It has been qualified with the LoRa concentrator HAL library version 1.3.0, and
should be compatible with any compatible later version that use the same API, or
a downward-compatible one.

Because some non-public functions from loragw_hal had to be re-implemented, any
change in the IP might affect this program even if an updated LoRa concentrator
HAL library is provided.
Connecting an incompatible concentrator board should give you the error:
"ERROR: fail to connect to concentrator board"

3. Usage
---------

To stop the application before the end of the measurement, press Ctrl+C.

By default, the program scans the whole band (start and stop frequency defined
in the loragw_hal.h file) with a 200 kHz measurement step.
Use -f option to change start and stop frequency, and measurement step.

The resolution bandwidth of the scan is 200 kHz and cannot be set by the user.

The measurement results are put in a CSV file whose name include a UTC timestamp
of measurement starting time in ISO 8601 recommended compact format:
yyyymmddThhmmssZ (eg. 20131009T172345Z for October 9th, 2013 at 5:23:45PM UTC)

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