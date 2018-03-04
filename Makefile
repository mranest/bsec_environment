CC=gcc

# Path where the BSEC distribution is
BSEC_PATH=./BSEC_1.4.5.1_Generic_Release_20171214

# BSEC configuration to use (e.g. generic 3.3V, ULP mode polling, 4days background calibration)
BSEC_CONFIG=generic_33v_3s_4d

# Library to use (PiThree or PiZero)
BSEC_LIB=RaspberryPI/PiThree_ArmV8-a-64bits
#BSEC_LIB=RaspberryPI/PiZero_ArmV6-32bits

CFLAGS=-I${BSEC_PATH}/algo -I${BSEC_PATH}/config/${BSEC_CONFIG} -L${BSEC_PATH}/algo/bin/${BSEC_LIB}
LIBS=-lalgobsec -lm

bsec_environment: bsec_environment.c bsec_integration.c
	$(CC) -o bsec_environment \
	    BME680_driver/bme680.c \
	    ${BSEC_PATH}/config/$(BSEC_CONFIG)/bsec_serialized_configurations_iaq.c \
	    bsec_integration.c \
	    bsec_environment.c \
	    $(CFLAGS) $(LIBS)
