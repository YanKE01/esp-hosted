#!/bin/bash

# Ensure script stops on errors
set -e

TGT_NAME=$1

# Navigate to the target directory
cd esp_hosted_ng/esp/esp_driver/

# Setup IDF and required libraries
yes | ./setup.sh -f

cd esp-idf
echo "Exporting variables"
. ./export.sh

cd ../network_adapter

# Check if the second argument (assumed to be a string) matches "spi" or "usb"
if [ "$2" = "spi" ]; then
    # Check if sdkconfig.ci exists
    if [ -f "sdkconfig.ci" ]; then
        # Append the content of sdkconfig.ci to sdkconfig.defaults
        echo "appending ci config to default"
        cat sdkconfig.ci >> sdkconfig.defaults
    else
        echo "Error: sdkconfig.ci does not exist."
        exit 1
    fi
fi

echo "Setting target as $TGT_NAME"
idf.py set-target "$TGT_NAME"

# For USB, append CONFIG_ESP_USB_HOST_INTERFACE=y to sdkconfig after set-target
if [ "$2" = "usb" ]; then
    if [ -f "sdkconfig" ]; then
        echo "Appending CONFIG_ESP_USB_HOST_INTERFACE=y to sdkconfig"
        echo "CONFIG_ESP_USB_HOST_INTERFACE=y" >> sdkconfig
    else
        echo "Error: sdkconfig does not exist after set-target"
        exit 1
    fi
fi

idf.py build

# Check if the build was successful
if [ ! -f "build/network_adapter.bin" ]; then
    echo "Compilation failed; exit 1"
    exit 1
fi

echo "Build successful"
