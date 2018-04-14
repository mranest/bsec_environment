CC=gcc

# Path where the BSEC distribution is
BSEC_PATH=./BSEC_1.4.6.0_Generic_Release_20180409

# BSEC configuration to use (e.g. generic 3.3V, ULP mode polling, 28days background calibration)
BSEC_CONFIG=generic_33v_3s_28d

# Library version to use
BSEC_LIB=algo/bin/Normal_version/RaspberryPI/PiThree_ArmV8-a-64bits
#BSEC_LIB=algo/bin/Normal_version/RaspberryPI/PiZero_ArmV6-32bits

CFLAGS=-I${BSEC_PATH}/${BSEC_LIB} -I${BSEC_PATH}/config/${BSEC_CONFIG} -L${BSEC_PATH}/${BSEC_LIB}
LIBS=-lalgobsec -lm

bsec_environment: bsec_environment.c bsec_integration.c
	$(CC) -o bsec_environment \
	    BME680_driver/bme680.c \
	    ${BSEC_PATH}/config/$(BSEC_CONFIG)/bsec_serialized_configurations_iaq.c \
	    bsec_integration.c \
	    bsec_environment.c \
	    $(CFLAGS) $(LIBS)
