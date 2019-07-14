
#!/bin/bash
#nrfjprog --family NRF52 --eraseall
#nrfjprog --family NRF52 --program collarapp.hex --verify --reset
nrfjprog -f nrf52 --program out.hex --sectorerase --reset
