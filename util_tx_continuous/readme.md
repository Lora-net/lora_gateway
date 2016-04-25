	  ______                              _
	 / _____)             _              | |
	( (____  _____ ____ _| |_ _____  ____| |__
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2014 Semtech-Cycleo

Tx continuous program for LoRa concentrator
===========================================


1. Introduction
----------------

This software is used to set LoRa concentrator in Tx continuous mode, for
spectral measurement.
The user can set the modulation type, the modulation parameters, and the
multiple gains of the Tx chain.
The program runs indefinitely, until the user stops the application.


2. Usage
--------

See command line help to get the list of all available options:
./util_tx_continuous -h

Example:
./util_tx_continuous -f 868 -r 1257 --dig 0 --mix 14 --pa 3 --mod "LORA" --sf 7 --bw 125


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
