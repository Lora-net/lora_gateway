	 / _____)             _              | |    
	( (____  _____ ____ _| |_ _____  ____| |__  
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \ 
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2013 Semtech-Cycleo

LoRa concentrator HAL user manual
============================

1. Introduction
---------------

The LoRa concentrator Hardware Abstraction Layer is a C library that allow you
to use a Semtech concentrator chip through a reduced number of high level C
functions to configure the hardware, send and receive packets.

The Semtech LoRa concentrator is a digital multi-channel multi-standard packet
radio used to send and receive packets wirelessly using LoRa or FSK modulations.

2. Components of the library
----------------------------

The library is composed of 6(8) modules:

* loragw_hal
* loragw_reg
* loragw_spi
* loragw_aux
* loragw_gps
* loragw_radio
* loragw_fpga (only for SX1301AP2 ref design)
* loragw_lbt (only for SX1301AP2 ref design)

The library also contains basic test programs to demonstrate code use and check
functionality.

### 2.1. loragw_hal ###

This is the main module and contains the high level functions to configure and
use the LoRa concentrator:

* lgw_board_setconf, to set the configuration of the concentrator 
* lgw_rxrf_setconf, to set the configuration of the radio channels
* lgw_rxif_setconf, to set the configuration of the IF+modem channels
* lgw_txgain_setconf, to set the configuration of the concentrator gain table
* lgw_start, to apply the set configuration to the hardware and start it
* lgw_stop, to stop the hardware
* lgw_receive, to fetch packets if any was received
* lgw_send, to send a single packet (non-blocking, see warning in usage section)
* lgw_status, to check when a packet has effectively been sent

For an standard application, include only this module.
The use of this module is detailed on the usage section.

/!\ When sending a packet, there is a delay (approx 1.5ms) for the analog
circuitry to start and be stable. This delay is adjusted by the HAL depending
on the board version (lgw_i_tx_start_delay_us).

In 'timestamp' mode, this is transparent: the modem is started
lgw_i_tx_start_delay_us microseconds before the user-set timestamp value is
reached, the preamble of the packet start right when the internal timestamp
counter reach target value.

In 'immediate' mode, the packet is emitted as soon as possible: transferring the
packet (and its parameters) from the host to the concentrator takes some time,
then there is the lgw_i_tx_start_delay_us, then the packet is emitted.

In 'triggered' mode (aka PPS/GPS mode), the packet, typically a beacon, is
emitted lgw_i_tx_start_delay_us microsenconds after a rising edge of the
trigger signal. Because there is no way to anticipate the triggering event and
start the analog circuitry beforehand, that delay must be taken into account in
the protocol.

### 2.2. loragw_reg ###

This module is used to access to the LoRa concentrator registers by name instead
of by address:

* lgw_connect, to initialise and check the connection with the hardware
* lgw_disconnect, to disconnect the hardware
* lgw_soft_reset, to reset the whole hardware by resetting the register array
* lgw_reg_check, to check all registers vs. their default value and output the
result to a file
* lgw_reg_r, read a named register
* lgw_reg_w, write a named register
* lgw_reg_rb, read a name register in burst
* lgw_reg_wb, write a named register in burst

This module handles pagination, read-only registers protection, multi-byte
registers management, signed registers management, read-modify-write routines
for sub-byte registers and read/write burst fragmentation to respect SPI
maximum burst length constraints.

It make the code much easier to read and to debug.
Moreover, if registers are relocated between different hardware revisions but
keep the same function, the code written using register names can be reused "as
is".

If you need access to all the registers, include this module in your
application.

**/!\ Warning** please be sure to have a good understanding of the LoRa
concentrator inner working before accessing the internal registers directly.

### 2.3. loragw_spi ###

This module contains the functions to access the LoRa concentrator register
array through the SPI interface:

* lgw_spi_r to read one byte
* lgw_spi_w to write one byte
* lgw_spi_rb to read two bytes or more
* lgw_spi_wb to write two bytes or more

Please *do not* include that module directly into your application.

**/!\ Warning** Accessing the LoRa concentrator register array without the
checks and safety provided by the functions in loragw_reg is not recommended.

### 2.4. loragw_aux ###

This module contains a single host-dependant function wait_ms to pause for a
defined amount of milliseconds.

The procedure to start and configure the LoRa concentrator hardware contained in
the loragw_hal module requires to wait for several milliseconds at certain
steps, typically to allow for supply voltages or clocks to stabilize after been
switched on.

An accuracy of 1 ms or less is ideal.
If your system does not allow that level of accuracy, make sure that the actual
delay is *longer* that the time specified when the function is called (ie.
wait_ms(X) **MUST NOT** before X milliseconds under any circumstance).

If the minimum delays are not guaranteed during the configuration and start
procedure, the hardware might not work at nominal performance.
Most likely, it will not work at all.

### 2.5. loragw_gps ###

This module contains functions to synchronize the concentrator internal 
counter with an absolute time reference, in our case a GPS satellite receiver.

The internal concentrator counter is used to timestamp incoming packets and to 
triggers outgoing packets with a microsecond accuracy.
In some cases, it might be useful to be able to transform that internal 
timestamp (that is independent for each concentrator running in a typical 
networked system) into an absolute GPS time.

In a typical implementation a GPS specific thread will be called, doing the
following things after opening the serial port:

* blocking reads on the serial port (using system read() function)
* parse UBX messages (using lgw_parse_ubx) to get actual native GPS time
* parse NMEA sentences (using lgw_parse_nmea) to get location and UTC time
Note: the RMC sentence gives UTC time, not native GPS time.

And each time an NAV-TIMEGPS UBX message has been received:

* get the concentrator timestamp (using lgw_get_trigcnt, mutex needed to 
  protect access to the concentrator)
* get the GPS time contained in the UBX message (using lgw_gps_get)
* call the lgw_gps_sync function (use mutex to protect the time reference that 
  should be a global shared variable).

Then, in other threads, you can simply used that continuously adjusted time 
reference to convert internal timestamps to GPS time (using lgw_cnt2gps) or
the other way around (using lgw_gps2cnt). Inernal concentrator timestamp can
also be converted to/from UTC time using lgw_cnt2utc/lgw_utc2cnt functions.

### 2.6. loragw_radio ###

This module contains functions to handle the configuration of SX125x and
SX127x radios.

### 2.7. loragw_fpga ###

This module contains the description of the FPGA registers, the functions to
read/write those registers, and a function to configure the FPGA features.

This module is only required for SX1301AP2 reference design.

### 2.8. loragw_lbt ###

This module contains functions to configure and use the "Listen-Before-Talk"
feature (refered as LBT below). It depends on the loragw_fpga and loragw_radio
modules.

LBT feature is only available on SX1301AP2 reference design, which provides the
FPGA and the SX127x radio required to accomplish the feature.

The FPGA implements the following Finite State Machine (FSM) to scan the defined
LBT channels (8 max), and also compute the RSSI histogram for spectral scan,
using the SX127x radio.


                          +-------+
      +------------------>+ idle  +------------------+
      |                   +-------+                  v
      |                       |                +-----------+
      |                       |                | clean mem |
      |                       v                +-----------+
      |                  +----------+                |
      |                  | set freq |<---------------+
      |                  +----------+
      |                       |
      |                       v
      |                  +----------+
      |                  | wait pll |
      |                  |   lock   |
      |                  +----------+
      |                       |                (SCAN_CHANNEL)
      |                       v                +-----------+
      |                 +-----------+          |           |
      |                 |           +----------+           v
      |             +-->| read RSSI |                +------------+
      |             |   |           +<---------------+ calc histo |
      |             |   +-----------+   SCANNING     +------------+
      |             |         |                            |
      |    SCANNING |         | (LBT_CHANNEL)              |
      |             |         v                            |
      |             |  +-------------+                     |
      |             |  |   compare   |                     |
      |             +--+     with    |                     |
      |                | RSSI_TARGET |          HISTO_DONE |
      |                +-------------+                     |
      |                       |                            |
      |             SCAN DONE |                            |
      |                       v                            |
      |                 +------------+                     |
      |                 |  increase  |                     |
      +-----------------+            +<--------------------+
                        |    freq    |
                        +------------+



In order to configure the LBT, the following parameters have to be set:
- RSSI_TARGET: signal strength target used to detect if the channel is clear
               or not.
               RSSI_TARGET_dBm = -RSSI_TARGET/2
- LBT_CHx_FREQ_OFFSET: with x=[0..7], offset from the predefined LBT start
                       frequency (863MHz or 915MHz depending on FPGA image),
                       in 100KHz unit.
- LBT_SCAN_TIME_CHx: with x=[0..7], the channel scan time to be used for this
                     LBT channel: 128µs or 5000µs

With this FSM, the FPGA keeps the last instant when each channel was free during
more than LBT_SCAN_TIME_CHx µs.

Then, the HAL, when receiving a downlink request, will first determine on which
LBT channel this downlink is supposed to be sent and then checks if the channel
is busy or if downlink is allowed.

In order to determine if a downlink is allowed or not, the HAL does:
- read the LBT_TIMESTAMP_CH of the channel on which downlink is requested. This
  gives the last time when channel was free (LBT_TIME).
- compute the time on air of the downlink packet to determine the end time of
  the packet emission (PKT_END_TIME).
- if ((PKT_END_TIME - LBT_TIME) < TX_MAX_TIME)
    ALLOWED = TRUE
  else
    ALLOWED = FALSE
  endif
    where TX_MAX_TIME is the maximum time allowed to send a packet since the
    last channel free time (this depends on the channel scan time ).


3. Software build process
--------------------------

### 3.1. Details of the software ###

The library is written following ANSI C conventions but using C99 explicit
length data type for all data exchanges with hardware and for parameters.

The loragw_aux module contains POSIX dependant functions for millisecond
accuracy pause.
For embedded platforms, the function could be rewritten using hardware timers.

### 3.2. Building options ###

All modules use a fprintf(stderr,...) function to display debug diagnostic
messages if the DEBUG_xxx is set to 1 in library.cfg

### 3.3. Building procedures ###

For cross-compilation set the ARCH and CROSS_COMPILE variables in the Makefile,
or in your shell environment, with the correct toolchain name and path.
ex:
export PATH=/home/foo/rpi-toolchain/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:$PATH
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

The Makefile in the libloragw directory will parse the library.cfg file and 
generate a config.h C header file containing #define options.
Those options enables and disables sections of code in the loragw_xxx.h files 
and the *.c source files.

The library.cfg is also used directly to select the proper set of dynamic 
libraries to be linked with.

### 3.4. Export ###

Once build, to use that library on another system, you need to export the
following files :

* libloragw/library.cfg  -> root configuration file
* libloragw/libloragw.a  -> static library, to be linked with a program
* libloragw/readme.md  -> required for license compliance
* libloragw/inc/config.h  -> C configuration flags, derived from library.cfg
* libloragw/inc/loragw_*.h  -> take only the ones you need (eg. _hal and _gps)

After statically linking the library to your application, only the license 
is required to be kept or copied inside your program documentation.

4. Hardware dependencies
------------------------

### 4.1. Hardware revision ###

The loragw_reg and loragw_hal are written for a specific version on the Semtech
hardware (IP and/or silicon revision).

This code has been written for:

* Semtech SX1301 chip
* Semtech SX1257 or SX1255 I/Q transceivers

The library will not work if there is a mismatch between the hardware version 
and the library version. You can use the test program test_loragw_reg to check 
if the hardware registers match their software declaration.

### 4.2. SPI communication ###

loragw_spi contains 4 SPI functions (read, write, burst read, burst write) that
are platform-dependant.
The functions must be rewritten depending on the SPI bridge you use:

* SPI master matched to the Linux SPI device driver (provided)
* SPI over USB using FTDI components (not provided)
* native SPI using a microcontroller peripheral (not provided)

You can use the test program test_loragw_spi to check with a logic analyser
that the SPI communication is working

### 4.3. GPS receiver (or other GNSS system) ###

To use the GPS module of the library, the host must be connected to a GPS 
receiver via a serial link (or an equivalent receiver using a different 
satellite constellation).
The serial link must appear as a "tty" device in the /dev/ directory, and the 
user launching the program must have the proper system rights to read and 
write on that device.
Use `chmod a+rw` to allow all users to access that specific tty device, or use
sudo to run all your programs (eg. `sudo ./test_loragw_gps`).

In the current revision, the library only reads data from the serial port, 
expecting to receive NMEA frames that are generally sent by GPS receivers as 
soon as they are powered up, and UBX messages which are proprietary to u-blox
modules.

The GPS receiver **MUST** send UBX messages shortly after sending a PPS pulse
on to allow internal concentrator timestamps to be converted to absolute GPS time.
If the GPS receiver sends a GGA NMEA sentence, the gateway 3D position will
also be available.

5. Usage
--------

### 5.1. Setting the software environment ###

For a typical application you need to:

* include loragw_hal.h in your program source
* link to the libloragw.a static library during compilation
* link to the librt library due to loragw_aux dependencies (timing functions)

For an application that will also access the concentrator configuration 
registers directly (eg. for advanced configuration) you also need to:

* include loragw_reg.h in your program source

### 5.2. Using the software API ###

To use the HAL in your application, you must follow some basic rules:

* configure the radios path and IF+modem path before starting the radio
* the configuration is only transferred to hardware when you call the *start*
  function
* you cannot receive packets until one (or +) radio is enabled AND one (or +)
  IF+modem part is enabled AND the concentrator is started
* you cannot send packets until one (or +) radio is enabled AND the concentrator
  is started
* you must stop the concentrator before changing the configuration

A typical application flow for using the HAL is the following:

	<configure the radios and IF+modems>
	<start the LoRa concentrator>
	loop {
		<fetch packets that were received by the concentrator>
		<process, store and/or forward received packets>
		<send packets through the concentrator>
	}
	<stop the concentrator>

**/!\ Warning** The lgw_send function is non-blocking and returns while the
LoRa concentrator is still sending the packet, or even before the packet has
started to be transmitted if the packet is triggered on a future event.
While a packet is emitted, no packet can be received (limitation intrinsic to
most radio frequency systems).

Your application *must* take into account the time it takes to send a packet or 
check the status (using lgw_status) before attempting to send another packet.

Trying to send a packet while the previous packet has not finished being send
will result in the previous packet not being sent or being sent only partially
(resulting in a CRC error in the receiver).

### 5.3. Debugging mode ###

To debug your application, it might help to compile the loragw_hal function
with the debug messages activated (set DEBUG_HAL=1 in library.cfg).
It then send a lot of details, including detailed error messages to *stderr*.

6. License
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
