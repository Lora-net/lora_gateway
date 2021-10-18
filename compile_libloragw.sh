#!/usr/bin/env sh

compile_libloragw_for_spi_bus() {
    spi_bus="$1"
    echo "Compiling upstream libloragw for sx1301 on $spi_bus"

    cd "$ROOT_DIR/libloragw" || exit
    export SPI_DEV_DIR="/dev/$spi_bus"
    make clean
    make -j 4

    cd "$ROOT_DIR"
    cp -r "$ROOT_DIR/libloragw/" "$OUTPUT_DIR/$spi_bus"

    echo "Finished building libloragw for sx1301 on $spi_bus in $OUTPUT_DIR"
}

compile_libloragw() {
    echo "Compiling libloragw for sx1301 concentrator on all the necessary SPI buses in $OUTPUT_DIR"
    
    # Built outputs will be copied to this directory
    mkdir -p "$OUTPUT_DIR"
    
    # In order to be more portable, intentionally not interating over an array
    compile_libloragw_for_spi_bus spidev0.0
    compile_libloragw_for_spi_bus spidev0.1
    compile_libloragw_for_spi_bus spidev1.0
    compile_libloragw_for_spi_bus spidev1.1
    compile_libloragw_for_spi_bus spidev1.2
    compile_libloragw_for_spi_bus spidev2.0
    compile_libloragw_for_spi_bus spidev2.1
    compile_libloragw_for_spi_bus spidev32766.0
}

copy_reset_script() {
    cp "$ROOT_DIR/reset_lgw.sh" "$OUTPUT_DIR/reset_lgw.sh"
}

compile_libloragw
copy_reset_script
