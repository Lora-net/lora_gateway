# Builds libloragw for each SPI bus and copies each to
# $OUTPUT_DIR/$SPI_BUS respectively. Also copies reset_lgw.sh
# to $OUTPUT_DIR.

FROM balenalib/raspberry-pi-debian:buster-build as sx1301-builder

ENV ROOT_DIR=/opt
ENV OUTPUT_DIR="$ROOT_DIR/output"

# Overwirtes value in loragw_spi.native.c
# https://github.com/NebraLtd/lora_gateway/blob/971c52e3e0f953102c0b057c9fff9b1df8a84d66/libloragw/src/loragw_spi.native.c#L56
ENV SPI_SPEED=2000000

WORKDIR "$ROOT_DIR"

# Copy upstream source into expected location
COPY . "$ROOT_DIR"

RUN . "$ROOT_DIR/compile_libloragw.sh"
