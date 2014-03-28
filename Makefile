### Environment constants 

CROSS_COMPILE :=
export

### general build targets

all:
	$(MAKE) all -e -C libloragw
	$(MAKE) all -e -C util_band_survey
	$(MAKE) all -e -C util_pkt_logger
	$(MAKE) all -e -C util_spi_stress
	$(MAKE) all -e -C util_tx_test

clean:
	$(MAKE) clean -e -C libloragw
	$(MAKE) clean -e -C util_band_survey
	$(MAKE) clean -e -C util_pkt_logger
	$(MAKE) clean -e -C util_spi_stress
	$(MAKE) clean -e -C util_tx_test

### EOF