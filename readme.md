	 / _____)             _              | |    
	( (____  _____ ____ _| |_ _____  ____| |__  
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \ 
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2013 Semtech-Cycleo

LoRa Gateway project
=====================

1. Core library: libloragw
---------------------------

This directory contains the sources of the library to build a gateway based on 
a Semtech LoRa multi-channel RF receiver (a.k.a. concentrator).
Once compiled all the code is contained in the libloragw.a file that will be 
statically linked (ie. integrated in the final executable).

The library must be configured by editing the library.cfg file to set target
chip, radio, SPI interface, etc.

The library also comes with a bunch of basic tests programs that are used to 
test the different sub-modules of the library.

2. Helper programs
-------------------

Those programs are included in the project to provide examples on how to use 
the HAL library, and to help the system builder test different parts of it.

### 2.1. util_band_survey ###

This software is used to scan the RF band and measure background RSSI and some
measurement of interferer pattern.

### 2.2. util_pkt_logger ###

This software is used to set up a LoRa concentrator using a JSON configuration
file and then record all the packets received in a log file, indefinitely, until
the user stops the application.

### 2.3. util_spi_stress ###

This software is used to check the reliability of the link between the host
platform (on which the program is run) and the LoRa concentrator register file
that is the interface through which all interaction with the LoRa concentrator
happens.

### 2.4. util_tx_test ###

This software is used to send test packets with a LoRa concentrator. The packets
contain little information, on no protocol (ie. MAC address) information but
can be used to assess the functionality of a gateway downlink using other
gateways as receivers.

3. Changelog
-------------

### v1.5.0 ###

* Adding option to discriminate LoRa MAC networks from private LoRa networks at PHY level.

### v1.4.1 ###

* Enabling support for FSK per LoRa MAC specification
* Adjusting TX and RX calibration set on 868 reference board
* Added specific RX/TX calibration set for Kerlink 868 "IoT station" gateway
* Changed default SPI port for native driver to Kerlink SPI device number

### v1.4.0 ###

* Added calibration routine to optimize RF performance
* Added support for SX1301 433 MHz reference board
* Improved AGC firmware
* Improved RSSI accuracy
* Improved utilities Makefile

### v1.3.0 ###

* Added TX power management.
* Added full support for SX1301 reference board.
* Changed build system with configuration for multiple chip/radio/band support.
* SX125x bandwidth set to 1MHz by default (was 800 kHz).
* Solved warnings with 64b integer printf when compiling on x86_64.
* Renamed helper programs to reduce the concentrator vs. gateway confusion.

### v1.2.2 ###

* Added a GPIO toggle on the FTDI SPI module to reset the SX1301 board.

### v1.2.1 ###

* Fixed 'floating point exception' crash when concentrator returned a packet with SF=0 (CRC error on LoRa header).
* Fixed buggy timezone handling.

### v1.2.0 ###

* Added feature: new GPS module in the library for synchronization.
* Removed feature: no more missed deadline detection in TX because of incompatibility with GPS.
* Added documentation for GPS and legal notice.
* Added flags in Makefiles for easier cross-compilation.

### v1.1.0 ###

* Fixed bug 'no TX on radio B' (rfch 1).
* Added feature: concentrator processing delay compensation in the receive() function for accurate 'end of packet' even timestamping.
* Added feature: TX 'start delay' compensation in the send() function to emit packet exactly on target timestamp.
* Added feature: timestamp counter verification in send() function, return an error if scheduling was too late.
* Switched license to 'Revised BSD'.

### v1.0.0 (from beta 8) ###

* Switched FTDI as default SPI phy layer in library.cfg.
* Fixed a bug in TX power control; still only two TW power available, 14 and 24 dBm.
* Changed library directory name from loragw_hal to libloragw to follow usual conventions.

### Beta 8 (from beta 7) ###

* API: lgw_receive now return info on RX frequency and RF path for each packet (no need to keep track of RF/IF settings).
* Unified some portion of the code with the 470 MHz variant of the HAL (use SX1255 radios instead of SX1257).
* Improved AGC and ARB firmwares.
* Adding -Wall -Wextra for compilation, fixing all the new warnings for cleaner code.
* Fixed bugs in handling of FSK datarate.
* test_loragw_hal now dumps the content of all LoRa registers after configuration in reg_dump.log.

### Beta 7 (from beta 5) ###

* Reduced number of SPI transactions to fetch a packet (improved number a packets par second that can be downloaded from concentrator).
* Streamlined build process, main target is now a static library: libloragw.a.
* Change memory allocation for payload: they are now part of the struct for TX/RX, no need to malloc/free.
* All RX chains can use any of the two radios now.
* FSK is available and working in TX and RX (variable length mode).
* Calibrated RSSI for FSK.
* lgw_connect now check the CHIP_ID.
* Added a license file and a changelog.
* Added a function returning a version string to allow identification of the version/options once compiled.

### Beta 6 ###

Private release, not taken into account in that changelog.

### Beta 5 (from beta 4) ###

* Updated registers, firmware and configuration to align with r986 bitstream revision.
* Calibrated RSSI for LoRa "multi" and LoRa "stand alone" modems.
* Renamed some confusing TX status code.
* Added preliminary FSK support.

### Beta 4 (from beta 3) ###

* Unified build environment with selectable SPI layer (Linux native or FTDI SPI-over-USB bridge).
* Remove the 500 kHz limit on radio bandwith, back to the nominal 800 kHz.
* Renamed debug flags.

4. Legal notice
----------------

The information presented in this project documentation does not form part of 
any quotation or contract, is believed to be accurate and reliable and may be 
changed without notice. No liability will be accepted by the publisher for any 
consequence of its use. Publication thereof does not convey nor imply any 
license under patent or other industrial or intellectual property rights. 
Semtech assumes no responsibility or liability whatsoever for any failure or 
unexpected operation resulting from misuse, neglect improper installation, 
repair or improper handling or unusual physical or electrical stress 
including, but not limited to, exposure to parameters beyond the specified 
maximum ratings or operation outside the specified range. 

SEMTECH PRODUCTS ARE NOT DESIGNED, INTENDED, AUTHORIZED OR WARRANTED TO BE 
SUITABLE FOR USE IN LIFE-SUPPORT APPLICATIONS, DEVICES OR SYSTEMS OR OTHER 
CRITICAL APPLICATIONS. INCLUSION OF SEMTECH PRODUCTS IN SUCH APPLICATIONS IS 
UNDERSTOOD TO BE UNDERTAKEN SOLELY AT THE CUSTOMER’S OWN RISK. Should a 
customer purchase or use Semtech products for any such unauthorized 
application, the customer shall indemnify and hold Semtech and its officers, 
employees, subsidiaries, affiliates, and distributors harmless against all 
claims, costs damages and attorney fees which could arise.

*EOF*