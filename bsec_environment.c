/*
 * Based on Bosch BSEC bsec_iot_example.c sample file
 */

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <memory.h>

#include "bsec_integration.h"
#include "bsec_serialized_configurations_iaq.h"

const char* STATE_FILE = "bsec_state.bin";

/**********************************************************************************************************************/
/* i2c initialization and handling */
/**********************************************************************************************************************/

// I2C Linux device handle
int g_i2cFid;

// Open the Linux device
void i2cOpen() {
    g_i2cFid = open("/dev/i2c-1", O_RDWR);
    if (g_i2cFid < 0) {
        perror("i2cOpen");
        exit(1);
    }
}

// Close the Linux device
// TODO hook via signal handler
void i2cClose() {
    close(g_i2cFid);
}

// Set the I2C slave address for all subsequent I2C device transfers
void i2cSetAddress(int address) {
    if (ioctl(g_i2cFid, I2C_SLAVE, address) < 0) {
        perror("i2cSetAddress");
        exit(1);
    }
}

/**********************************************************************************************************************/
/* BSEC callback functions */
/**********************************************************************************************************************/

/*!
 * @brief           Write operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */


    uint8_t reg[16];
    reg[0] = reg_addr;

    for (int i = 1; i < data_len + 1; i++)
        reg[i] = reg_data_ptr[i - 1];

    if (write(g_i2cFid, reg, data_len + 1) != data_len + 1) {
        perror("user_i2c_write");
        rslt = 1;
        exit(1);
    }

    return rslt;
}

/*!
 * @brief           Read operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t reg[1];
    reg[0] = reg_addr;

    if (write(g_i2cFid, reg, 1) != 1) {
        perror("user_i2c_read_reg");
        rslt = 1;
    }
    if (read(g_i2cFid, reg_data_ptr, data_len) != data_len) {
        perror("user_i2c_read_data");
        rslt = 1;
    }

    return rslt;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in milliseconds
 *
 * @return          none
 */
void delay_ms(uint32_t t_ms) {
    struct timespec ts;
    ts.tv_sec = t_ms / 1000;
    ts.tv_nsec = (t_ms % 1000) * 1000000;

    nanosleep(&ts, NULL);
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us() {
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);
    int64_t sec_to_nsec = 1000000000;
    int64_t system_current_time_ns = spec.tv_sec * sec_to_nsec;
    system_current_time_ns += spec.tv_nsec;
    int64_t system_current_time_us = system_current_time_ns / 1000;

    return system_current_time_us;
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp,
                  float iaq,
                  uint8_t iaq_accuracy,
                  float temperature,
                  float humidity,
                  float pressure,
                  float raw_temperature,
                  float raw_humidity,
                  float gas,
                  bsec_library_return_t bsec_status) {
    printf("{ \"timestamp_ms\": %.lld", timestamp / 1000 / 1000);
    printf(", \"iaq\": %.2f", iaq);
    printf(", \"iaq_accuracy\": %d", iaq_accuracy);
    printf(", \"temperature\": %.2f", temperature);     // degC
    printf(", \"raw_temperature\": %.2f", raw_temperature);
    printf(", \"humidity\": %.2f", humidity);           // %rH
    printf(", \"raw_humidity\": %.2f", raw_humidity);
    printf(", \"pressure\": %.2f", pressure / 100.0f);  // hPa
    printf(", \"gas\": %.0f", gas);                     // Ohms
    printf(", \"bsec_status\": %d", bsec_status);
    printf("}\n");

    fflush(stdout);
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer) {
    FILE *state = fopen(STATE_FILE, "rb");
    if (state != NULL) {
        uint32_t bytes_read = fread(state_buffer, sizeof(uint8_t), n_buffer, state);
        fclose(state);

        fprintf(stderr, "** State loaded\n");

        return bytes_read;
    } else {
        return 0;
    }
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length) {
    FILE *state = fopen(STATE_FILE, "wb");
    if (state != NULL) {
        fwrite(state_buffer, sizeof(uint8_t), length, state);
        fclose(state);

        time_t timer;
        time(&timer);
        struct tm* tm_info = localtime(&timer);

        char buffer[26];
        strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);

        fprintf(stderr, "** State saved on %s\n", buffer);
    } else {
        fprintf(stderr, "** Could not save state\n");
    }
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer) {
    memcpy(config_buffer, bsec_config_iaq, n_buffer);

    return n_buffer;
}

static void graceful_shutdown(int signo) {
    i2cClose();

    fprintf(stderr, "Shuting down BSEC environment\n");

    exit(99);
}

/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
int main(int argc, char* argv[]) {
    float temperature_offset = 0.0f;
    if (argc == 2) {
        temperature_offset = atof(argv[1]);
    }

    // Signal trapping
    signal(SIGINT, graceful_shutdown);
    signal(SIGQUIT, graceful_shutdown);
    signal(SIGTERM, graceful_shutdown);

    // I2C initialization
    i2cOpen();
    i2cSetAddress(BME680_I2C_ADDR_PRIMARY);

    /* Call to the function which initializes the BSEC library 
     * Switch on low-power mode and provide no temperature offset */
    return_values_init ret =
            bsec_environment_init(BSEC_SAMPLE_RATE_LP, temperature_offset, bus_write, bus_read, delay_ms, state_load, config_load);
    if (ret.bme680_status) {
        /* Could not intialize BME680 */
        return (int) ret.bme680_status;
    } else if (ret.bsec_status) {
        /* Could not intialize BSEC library */
        return (int) ret.bsec_status;
    }

    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 600 samples, which means every 600 * 3 secs = 30 minutes  */
    bsec_environment_loop(delay_ms, get_timestamp_us, output_ready, state_save, 600);

    return 0;
}

