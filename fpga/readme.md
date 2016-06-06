	 / _____)             _              | |    
	( (____  _____ ____ _| |_ _____  ____| |__  
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \ 
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2013 Semtech-Cycleo

FPGA images for LoRa Gateway SX1301AP2-PCB_E336
===============================================

1. Content
----------

This directory contains the FPGA images to be programmed in the Semtech's
Reference Design board (SX1301AP2-PCB_E336) flash memory.

The different images contain the following features:

* SX1301_FPGA_125K_NOTCH_LBT_bitmap_v27.bin:
    - 125K Notch filter for TX
    - Listen-Before-Talk

* SX1301_FPGA_125K_NOTCH_SPECTRAL_SCAN_bitmap_v27.bin:
    - 125K Notch filter for TX
    - Background Spectral Scan

2. Usage
--------

The following parameters have to be set when using the Lattice Diamond
Programmer software:

Device Family -> iCE40
Device -> iCE40LP1K
Operation -> SPI Flash Programming
          -> Programming file: select one of the provided bin image
          -> SPI Vendor: Micron
          -> SPI Device: SPI-M25P10-A

3. Legal notice
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
