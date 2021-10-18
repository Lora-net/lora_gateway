# Builds libloragw for each SPI bus and copies each to
# $OUTPUT_DIR/$SPI_BUS respectively. Also copies reset_lgw.sh
# to $OUTPUT_DIR.

FROM balenalib/raspberry-pi-debian:buster-build as lora-gateway-sx1301-builder

ENV ROOT_DIR=/opt

# Output built files to this location
ENV OUTPUT_DIR="$ROOT_DIR/output"

# Used in libloragw/Makefile value and supplied to loragw_spi.native.c
ENV SPI_SPEED=2000000

WORKDIR "$ROOT_DIR"

# Copy upstream source into expected location
COPY . "$ROOT_DIR"

# Compile libloragw
RUN . "$ROOT_DIR/compile_libloragw.sh"
