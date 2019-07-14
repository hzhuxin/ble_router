
#!/bin/bash

nrfutil settings generate --family NRF52840 --application app.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 setting.hex

#nrfutil pkg generate --application nrf52832_app.hex --application-version 0x01 --hw-version 52 --sd-req 0x00A8 --key-file priv.pem nrf52832_ota_app.zip

mergehex -m sdk.hex boot.hex -o sd_boot.hex
mergehex -m sd_boot.hex app.hex -o sd_boot_app.hex
mergehex -m sd_boot_app.hex setting.hex -o sd_boot_app_setting.hex

#generate the settings file




nrfutil pkg generate --application app/_build/nrf52840_xxaa.hex --application-version 0x01 --hw-version 52 --sd-req 0x009D --key-file priv.pem router_app.zip


