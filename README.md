BSEC environment
================

Utilize Bosch's Bosch Software Environmental Cluster (BSEC) library to process BME680
sensor signals and provide compensated values for temperature, pressure and humidity,
as well as indoor air quality (IAQ) value.

Values provided by BSEC library are printed in stdout as JSON-formatted strings, so that
output can be redirected into another process to handle further processing, displaying etc.

Installation
------------

1. Initialize the [BME680_driver submodule](https://github.com/BoschSensortec/BME680_driver):

```
git submodule init
git submodule update
```

2. Download and unzip [Bosch Sensortec Environmental Cluster (BSEC) Software](https://www.bosch-sensortec.com/bst/products/all_products/bsec)
distribution file.

3. Edit `Makefile` to update:
    - Path where the BSEC distribution was unzipped
    - BSEC configuration to use (e.g. `generic_33v_3s_4d` for 3.3V, LP mode polling,
     4 days background calibration)
    - Library to use; use the folder name under `algo/bin` folder of BSEC distribution for the
    architecture of choice (e.g. `RaspberryPI/PiThree_ArmV8-a-64bits` for PiThree)
