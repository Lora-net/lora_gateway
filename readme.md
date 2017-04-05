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

The library also comes with a bunch of basic tests programs that are used to 
test the different sub-modules of the library.

2. Helper programs
-------------------

Those programs are included in the project to provide examples on how to use 
the HAL library, and to help the system builder test different parts of it.

### 2.1. util_pkt_logger ###

This software is used to set up a LoRa concentrator using a JSON configuration
file and then record all the packets received in a log file, indefinitely, until
the user stops the application.

### 2.2. util_spi_stress ###

This software is used to check the reliability of the link between the host
platform (on which the program is run) and the LoRa concentrator register file
that is the interface through which all interaction with the LoRa concentrator
happens.

### 2.3. util_tx_test ###

This software is used to send test packets with a LoRa concentrator. The packets
contain little information, on no protocol (ie. MAC address) information but
can be used to assess the functionality of a gateway downlink using other
gateways as receivers.

### 2.4. util_tx_continuous ###

This software is used to set LoRa concentrator in Tx continuous mode,
for spectral measurement.

### 2.5. util_spectral_scan ###

This software is used to scan the spectral band in background, where the LoRa
gateway operates.

### 2.6. util_lbt_test ###

This software is used to test "Listen-Before-Talk" channels timestamps.

3. Helper scripts
-----------------

### 3.1. reset_lgw.sh

This script must be launched on IoT Start Kit platform to reset concentrator
chip through GPIO, before starting any application using the concentrator.

4. Changelog
-------------

### v5.0.1 ###

* HAL: Reworked the way the TX start delay is calculated, taking into account
the delay introduced by TX notch filter (when enabled) and the delay linked to
signal bandwidth separately.

### v5.0.0 ###

* HAL: Changed GPS module to get native GPS time (monotonic, no leap second).
**WARNING**: The native GPS time is not given in standard NMEA messages, so we
get it from a proprietary message of the GPS module used on gateway reference
design, u-blox 7. If you are not using the same GPS module, you may have to
update the lgw_parse_ubx() function.
* HAL: Added lgw_cnt2gps() and lgw_gps2cnt() functions for SX1301<->GPS time
conversion.
* HAL: Changed serial port configuration for GPS to properly handle binary
messages (UBX).
* HAL: Added a lgw_gps_disable() function to restore serial configuration as
it was before HAL initialization.
* HAL: Fixed packet time on air calculation using the actual packet preamble
size.
* HAL: Adjusted TX_START_DELAY based on the board reference design to ensure
that a packet is sent exactly at 1500µs after the TX trigger (TIMESTAMP or PPS),
with a tolerance of +/- 1µs. This is mandatory to comply with LoRaWAN
specification for Class-B beaconing precise timing.
**WARNING**: This release provides tx start delay values to be used for Semtech
reference designs AP1 and AP2. The HAL automatically detects the board version
by detecting a FPGA or not. If you are using a different reference design or
a different FPGA binary version than the one provided with this release, the
value to be used for TX start delay may be different.

### v4.1.3 ###

* HAL: Reference clock frequency error improvement: The lora_gateway HAL has
been updated (3 registers changed) to improve the performance of all gateways
based on SX130x. The fix greatly improves the reception of packet at SF12, when
the frequency offset of the incoming packet is large (mostly below -20ppm of
frequency offset).

WARNING: Systems which do not have the patch will be more prone to packet loss
over time, when the crystals of the end-devices will be ageing and have more
frequency offset.

### v4.1.2 ###

* HAL: Changed configuration of IQ polarity of FPGA for TX to comply with FPGA
version greater than v27. (Only required for AP2 Semtech reference design)
* HAL: Updated default LoRa preamble size according to LoRaWAN spec.

### v4.1.1 ###

* HAL: Fixed bug in "Listen-Before-Talk" which was preventing from configuring
the Scan Time to 5ms.
* MISC: Added GPIO number to reset_lgw.sh command arguments.

### v4.1.0 ###

* HAL: Reworked "Listen-Before-Talk" feature to have more flexibility to define
LBT channels frequency, and to be able to have spectral scan running in parallel
* HAL: Updated lgw_time_on_air() function for FSK packets
* HAL: Disabled GPS UART input being re-echoed as output to avoid sending wrong
commands to GPS module
* HAL: Fixed IF frequency configuration check issue for channel bandwidths 250K
and 500HKz.
* FPGA: Updated to v31 for new LBT and spectral scan design.
* util_spectral_scan: updated to match new spectral scan FPGA sequence
* util_lbt_test: updated to match LBT rework

Note: The provided LBT feature has been validated for Japan only, and supports
8 downlink channels maximum.

### v4.0.1 ###

* HAL: SX1301AP2: Only FPGA v27 is supported, removed (v18,v19) from the list
        of supported FPGA images.

WARNING: If you are using a Semtech SX1301AP2 ref design (GW1.5), the FPGA must
be reprogrammed with one of the images provided with this release (fpga/ dir).

### v4.0.0 ###

* HAL: Added "Listen-Before-Talk" support for Semtech SX1301AP2 Ref Design.
       A description of the feature implementation can be found in
       libloragw/readme.md.
* HAL: Updated FSK RSSI calculation for better linearization
* util_lbt_test: New utility provided for basic "Listen-Before-Talk" testing.
* util_tx_test: Extended to configure and test "LBT" through the HAL.
* Added a reset_lgw.sh script to be used with IoT Starter Kit (v1.0) to reset
the concentrator through the HOST GPIO pin.

### v3.2.1 ###

* HAL: Fixed downlink support for SX1301AP2 reference design: soft reset of the
FPGA was missing for proper IQ inversion configuration.
* HAL: Added support for several versions of FPGA (currently v18 and v19)
* HAL: Reduced radio TX PLL bandwidth to reduce the noise level.
* util_tx_test: Added FSK support and added minimal TX gain LUT.
* util_spectral_scan: Removed FPGA soft reset, now done by the HAL.
* util_tx_continous: reworked to use HAL functions instead of 'manual' config,
and use same SX1301 calibration firmware as the HAL.
* Updated all makefiles to handle the creation of obj directory when necessary.
* Change cs_change usage policy in SPI module to let the driver handle the chip
select.

### v3.2.0 ###

* Added support for SX1301AP2 reference design (with FPGA and additional
SX127x). When a FPGA is detected at startup, the HAL automatically adapts SPI
communication requests (using SPI header or not).
* Added util_spectral_scan diagnostic tool to scan the spectral band in
background, where the LoRa gateway operates. (can only be used with SX1301AP2
or similar design). By default it uses the same SPI device as the one used by
the HAL, but it can be changed depending on the hardware architecture on which
it is used by updating the SPI_DEV_PATH constant defined in file
util_spectral_scan/src/loragw_fpga_spi.c.
Note: when using same SPI device from 2 applications, we rely on the host SPI
driver and OS to properly handle concurrent SPI requests. It has been tested on
Raspberry Pi / Raspbian with spi_bcm2708 driver.* Removed SPI FTDI support due
to lack of performances to properly handle heavy packet traffic. Only native
SPI usage is recommended.
* HAL: added a check that SX1301 firmwares have been properly loaded at startup.

### v3.1.0 ###

* Removed GPIO module from HAL, that was specific to IoT Starter Kit platform.
GPIO configuration will be done from application script instead.
* Removed CFG_BRD configuration from library.cfg, not needed anymore

### v3.0.2 ###

* Bugfix: Fixed frequency calculation on uplinks: lgw_receive() function was
using a variable to calculate the frequency before it was initialized with
correct value. 
* Bugfix: util_pkt_logger crashed when no gateway_ID is not defined in
global_conf.json

### v3.0.1 ###

* Bufgix: Fixed util_tx_continuous compilation issue, by adding empty obj
directory
* Bugfix: Fixed HAL compilation issue for CFG_SPI=ftdi, removed dependency on
loragw_gpio in this case

### v3.0.0 ###

* Added new HAL function lgw_board_setconf() to configure board/concentrator
specific parameters: network type (LoRa public or private), concentrator clock
source. Note: those parameters are not any more set from the library.cfg file
configuration (CFG_NET, CFG_BRD),
and should be passed at initialization by the application.
* Added new HAL function lgw_txgain_setconf() to configure concentrator TX gain
table. It can now be dynamically set by the application at initialization time.
* Changed HAL function lgw_rxrf_setconf(), it will now also configure the radio
type (CFG_RADIO has been removed from library.cfg), the RSSI offset to be used
for this radio and if TX is enabled or not on this radio.
* Added support of IoT Starter Kit platform, which is now the default board.
* Added util_tx_continuous utility for gateway TX power calibration and
spectral emission measurements/qualification.
* Removed CFG_BAND configuration from library.cfg. Band configuration is done
by application and passed dynamically at initialization time.
* Updated makefiles to allow cross compilation from environment variable (ARCH,
CROSS_COMPILE).

** WARNING: **
** Known issue: a problem with carrier leakage calibration has been seen on
433MHz boards. **

### v2.0.0 ###

* Added support for Kerlink 868 27dBm gateway
* Updated global_conf.eu868.json (in packet logger) to new LoRaWAN frequency
plan
* Added version numbers to AGC, arbiter and calibration firmware (those
versions are checked at startup)
* Added test_loragw_cal to test radio calibrations
* Fixed minor bug in error coverage in register read/write functions

/!\ warning: Kerlink 868 27dBm gateway includes a FPGA that MUST be programmed
before running any application

### v1.7.0 ###

* Added TX 'start delay' compensation for timestamp mode (fix time window
alignment issue at low SF and/or high BW)
* Added adaptive narrowband/wideband TX filtering for LoRa
* Added a command-line option to set CR in util_tx_test
* Added notes for TX 'start delay' in immediate and triggered mode

/!\ warning: due to start delay compensation being implemented, TX that were 
previously 1.5ms late will be sent on time. At low datarate, this is not an 
issue. At high LoRa data rate (and FSK) you might have to adjust your timing.

### v1.6.0 ###

* Fixed bug with 250kHz and 500 kHz TX filtering
* Adjusted FSK timestamp calibration in RX for accurate RX/TX alignment
* Added lgw_abort_tx() function to stop a TX at any time (scheduled or ongoing)
* Added support for user-settable FSK sync word (same for RX and TX)
* Added support for the Chinese 780 MHz band
* Added support for Kerlink 433 gateway
* Added support for Cisco 433, 470 & 780 MHz concentrators boards

### v1.5.0 ###

* Adding option to isolate public LoRa MAC networks at PHY level.

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

* Fixed 'floating point exception' crash when concentrator returned a packet
with SF=0 (CRC error on LoRa header).
* Fixed buggy timezone handling.

### v1.2.0 ###

* Added feature: new GPS module in the library for synchronization.
* Removed feature: no more missed deadline detection in TX because of
incompatibility with GPS.
* Added documentation for GPS and legal notice.
* Added flags in Makefiles for easier cross-compilation.

### v1.1.0 ###

* Fixed bug 'no TX on radio B' (rfch 1).
* Added feature: concentrator processing delay compensation in the receive()
function for accurate 'end of packet' even timestamping.
* Added feature: TX 'start delay' compensation in the send() function to emit
packet exactly on target timestamp.
* Added feature: timestamp counter verification in send() function, return an
error if scheduling was too late.
* Switched license to 'Revised BSD'.

### v1.0.0 (from beta 8) ###

* Switched FTDI as default SPI phy layer in library.cfg.
* Fixed a bug in TX power control; still only two TW power available, 14 and
24dBm.
* Changed library directory name from loragw_hal to libloragw to follow usual
conventions.

### Beta 8 (from beta 7) ###

* API: lgw_receive now return info on RX frequency and RF path for each packet
(no need to keep track of RF/IF settings).
* Unified some portion of the code with the 470 MHz variant of the HAL (use
SX1255 radios instead of SX1257).
* Improved AGC and ARB firmwares.
* Adding -Wall -Wextra for compilation, fixing all the new warnings for cleaner
code.
* Fixed bugs in handling of FSK datarate.
* test_loragw_hal now dumps the content of all LoRa registers after
configuration in reg_dump.log.

### Beta 7 (from beta 5) ###

* Reduced number of SPI transactions to fetch a packet (improved number a
packets par second that can be downloaded from concentrator).
* Streamlined build process, main target is now a static library: libloragw.a.
* Change memory allocation for payload: they are now part of the struct for
TX/RX, no need to malloc/free.
* All RX chains can use any of the two radios now.
* FSK is available and working in TX and RX (variable length mode).
* Calibrated RSSI for FSK.
* lgw_connect now check the CHIP_ID.
* Added a license file and a changelog.
* Added a function returning a version string to allow identification of the
version/options once compiled.

### Beta 6 ###

Private release, not taken into account in that changelog.

### Beta 5 (from beta 4) ###

* Updated registers, firmware and configuration to align with r986 bitstream
revision.
* Calibrated RSSI for LoRa "multi" and LoRa "stand alone" modems.
* Renamed some confusing TX status code.
* Added preliminary FSK support.

### Beta 4 (from beta 3) ###

* Unified build environment with selectable SPI layer (Linux native or FTDI
SPI-over-USB bridge).
* Remove the 500 kHz limit on radio bandwith, back to the nominal 800 kHz.
* Renamed debug flags.

5. Legal notice
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
UNDERSTOOD TO BE UNDERTAKEN SOLELY AT THE CUSTOMER'S OWN RISK. Should a
customer purchase or use Semtech products for any such unauthorized 
application, the customer shall indemnify and hold Semtech and its officers, 
employees, subsidiaries, affiliates, and distributors harmless against all 
claims, costs damages and attorney fees which could arise.

*EOF*
