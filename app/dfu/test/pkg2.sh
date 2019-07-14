#!/bin/bash

nrfutil settings generate --family NRF52 --application app.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 setting.hex
mergehex -m sdk.hex boot.hex -o sd_boot.hex
mergehex -m sd_boot.hex app.hex -o sd_boot_app.hex
mergehex -m sd_boot_app.hex setting.hex -o out.hex
rm setting.hex
rm sd_boot.hex
rm sd_boot_app.hex
nrfutil pkg generate --application app.hex --application-version 0x01 --hw-version 52 --sd-req 0xA9 --key-file priv.pem router_app.zip


#nrfutil settings generate --family NRF52840 --application app.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 setting.hex

#mergehex -m sdk.hex boot.hex -o sd_boot.hex
#mergehex -m sd_boot.hex app.hex -o sd_boot_app.hex
#mergehex -m sd_boot_app.hex setting.hex -o out.hex

#rm setting.hex
#rm sd_boot.hex
#rm sd_boot_app.hex

#nrfutil pkg generate --application app.hex --application-version 0x01 --hw-version 52 --sd-req 0x009D --key-file priv.pem router_app.zip
