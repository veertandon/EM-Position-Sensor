#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define SIMULATION_MODE 0 // 1 = no hardware, use simulated values; 0 = real sensor

// // --- LDC1612 Register Addresses ---
#define REG_DATA_CH1_MSB 0x02
#define REG_DATA_CH1_LSB 0x03
#define REG_RCOUNT_CH1 0x09         // Reference Count setting for Channel 1
#define REG_SETTLECOUNT_CH1 0x11    // Channel 1 Settling Reference Count
#define REG_CLOCK_DIVIDERS_CH1 0x15 // Reference and Sensor Divider settings for Channel 1
#define REG_DRIVE_CURRENT_CH1 0x1F  // Channel 1 sensor current drive configuration
#define REG_CONFIG 0x1A             // Conversion Configuratio
#define REG_MUX_CONFIG 0x1B         // Channel Multiplexing Configuration
#define REG_ERROR_CONFIG 0x19

// --- I2C Master Configuration ---
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LDC1612_ADDR 0x2B

static const char *TAG = "LDC1612_I2C";

// --- Sensor/Calculation Constants ---
#define TANK_CAPACITANCE 76e-12
#define BASE_FREQ 4708740       // from VNA testing
#define BASE_L_VALUE 1.3642E-05 // from VNA testing
#define R_COUNT_VALUE 0xFFFF
#define CLK_SPEED 43400000 // Internal LDC clock

// --- Function Prototypes ---
esp_err_t ldc1612_i2c_write(uint8_t reg_addr, uint16_t data);
esp_err_t ldc1612_i2c_read(uint8_t reg_addr, uint16_t *data);
esp_err_t ldc1612_init();

// --- Delta L Calculation ---
double calc_delta_L(double measured_freq, double base_freq, double base_L)
{
    double delta_f = measured_freq - base_freq;
    double delta_L = (delta_f * base_L * -2.0) / base_freq;
    return delta_L;
}

// --- I2C Master Initialization ---
void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "I2C Master Initialized.");
}

// --- I2C Write Function ---
esp_err_t ldc1612_i2c_write(uint8_t reg_addr, uint16_t data)
{
    uint8_t write_buf[3];
    write_buf[0] = reg_addr;
    // Data is MSB first
    write_buf[1] = (uint8_t)(data >> 8);
    write_buf[2] = (uint8_t)(data & 0xFF);

    return i2c_master_write_to_device(I2C_MASTER_NUM, LDC1612_ADDR, write_buf, 3, 1000 / portTICK_PERIOD_MS);
}

// --- I2C Read Function ---
esp_err_t ldc1612_i2c_read(uint8_t reg_addr, uint16_t *data)
{
    uint8_t read_buf[2];
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, LDC1612_ADDR, &reg_addr, 1, read_buf, 2, 1000 / portTICK_PERIOD_MS);

    if (ret == ESP_OK)
    {
        // Data is MSB first
        *data = (read_buf[0] << 8) | read_buf[1];
    }
    return ret;
}

esp_err_t ldc1612_init()
{
    ESP_LOGI(TAG, "Starting LDC1612 initialization for Channel 1...");
    esp_err_t ret;

    // Wait for sensor to be ready after power-up
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 1. Set RCOUNT for Channel 1 (max resolution)
    ret = ldc1612_i2c_write(REG_RCOUNT_CH1, R_COUNT_VALUE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "RCOUNT_CH1 write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "RCOUNT_CH1 set to 0x%04X", R_COUNT_VALUE);

    // 2. Set SETTLECOUNT for Channel 1
    // Formula: SETTLECOUNT ≥ Q_sensor × f_REF / (16 × f_SENSOR)
    // For safety, using 10 (0x000A) as a starting value
    uint16_t settle_val = 0x000A;
    ret = ldc1612_i2c_write(REG_SETTLECOUNT_CH1, settle_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SETTLECOUNT_CH1 write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "SETTLECOUNT_CH1 set to 0x%04X", settle_val);

    // 3. Set CLOCK_DIVIDERS for Channel 1
    // Bits [15:12] = FIN_DIVIDER1 = 0x1 (divide by 1)
    // Bits [11:10] = Reserved = 0b00
    // Bits [9:0] = FREF_DIVIDER1 = 0x001 (divide by 1)
    uint16_t clk_div_val = 0x1001;
    ret = ldc1612_i2c_write(REG_CLOCK_DIVIDERS_CH1, clk_div_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "CLOCK_DIVIDERS_CH1 write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "CLOCK_DIVIDERS_CH1 set to 0x%04X", clk_div_val);

    // 4. Set ERROR_CONFIG (enable data ready interrupt)
    uint16_t error_config = 0x0001; // Enable DRDY_2INT
    ret = ldc1612_i2c_write(REG_ERROR_CONFIG, error_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR_CONFIG write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "ERROR_CONFIG set to 0x%04X", error_config);

    // 5. Set MUX_CONFIG
    // Bit [15] = AUTOSCAN_EN = 0 (single channel continuous)
    // Bits [14:13] = RR_SEQUENCE = 0b00 (not used in single channel)
    // Bits [12:3] = Reserved = 0b00 0100 0001
    // Bits [2:0] = DEGLITCH = 0b101 (10 MHz) for ~5MHz sensor
    uint16_t mux_val = 0x020D; // Single channel, deglitch filter
    ret = ldc1612_i2c_write(REG_MUX_CONFIG, mux_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MUX_CONFIG write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "MUX_CONFIG set to 0x%04X", mux_val);

    // 6. **CRITICAL** Set DRIVE_CURRENT for Channel 1
    // For a sensor with Rp in the range of 5-10 kΩ, use IDRIVE value around 0b10010
    // Bits [15:11] = IDRIVE1 = 0b10010 (18 decimal) ≈ 195 µA
    // Bits [10:6] = INIT_IDRIVE1 = 0b00000 (set to 0 when writing)
    // Bits [5:0] = Reserved = 0b00 0000
    uint16_t drive_current = 0x3800; // IDRIVE1 = 0b10010 (18 decimal)
    ret = ldc1612_i2c_write(REG_DRIVE_CURRENT_CH1, drive_current);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "DRIVE_CURRENT_CH1 write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "DRIVE_CURRENT_CH1 set to 0x%04X", drive_current);

    // 7. Set CONFIG register LAST to select Channel 1 and wake up device
    // Bits [15:14] = ACTIVE_CHAN = 0b01 (Channel 1 for continuous conversion)
    // Bit [13] = SLEEP_MODE_EN = 0 (active mode)
    // Bit [12] = RP_OVERRIDE_EN = 1 (use programmed IDRIVE)
    // Bit [11] = SENSOR_ACTIVATE_SEL = 0 (full current activation)
    // Bit [10] = AUTO_AMP_DIS = 1 (disable auto amplitude correction)
    // Bit [9] = REF_CLK_SRC = 0 (internal oscillator)
    // Bit [8] = Reserved = 0
    // Bit [7] = INTB_DIS = 0 (INTB enabled)
    // Bit [6] = HIGH_CURRENT_DRV = 0 (normal current)
    // Bits [5:0] = Reserved = 0b00 0001 (must be set to this value)
    uint16_t config_val = 0x5401; // Channel 1 selected
    ret = ldc1612_i2c_write(REG_CONFIG, config_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "CONFIG write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "CONFIG set to 0x%04X - Channel 1 Active", config_val);

    // Wait for sensor to stabilize
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "LDC1612 Channel 1 initialized successfully.");
    return ESP_OK;
}

// Calibration
#define NUM_POINTS 26
// need 80 for interval, end value at 75mm
static const float pos_mm[NUM_POINTS] = {
    0.0, 5.0, 10.0, 15.0, 17.5, 20.0,
    22.5, 25.0, 27.5, 30.0, 32.5, 35.0,
    37.5, 40.0, 42.5, 45.0, 47.5,
    50.0, 52.5, 55.0, 57.5, 60.0,
    65.0, 70.0, 75.0, 80};

static const float freq_MHz[NUM_POINTS] = {
    4.38466, 4.38727, 4.39402, 4.40353, 4.40991,
    4.41472, 4.42542, 4.42793, 4.43923, 4.45070,
    4.45593, 4.46924, 4.47470, 4.48383, 4.48852,
    4.48951, 4.50409, 4.51242, 4.51403, 4.51935,
    4.52886, 4.53555, 4.54403, 4.5839, 4.61895, 4.63};

// Converts measured frequency → position_mm.
// check again - wrong position at 100mm and 5, 10mm (combine with linear interpolation?)
float interpolate_position(float f_meas)
{
    // Clamping
    //  if (f_meas <= freq_MHz[0])
    //  {
    //      ESP_LOGW(TAG, "Target Metal not in linear range");
    //      return NAN;
    //  }
    if (f_meas >= 4.65) // allows for last position at 75mm to be read
    {
        ESP_LOGW(TAG, "Target Metal not in linear range");
        return NAN;
    }

    // Find interval where freq lies
    for (int i = 0; i < NUM_POINTS - 1; i++)
    {
        float f1 = freq_MHz[i];
        float f2 = freq_MHz[i + 1];

        if ((f_meas >= f1 && f_meas <= f2) || (f_meas >= f2 && f_meas <= f1))
        {
            float x1 = pos_mm[i];
            float x2 = pos_mm[i + 1];

            // Linear interpolation:
            // x = x1 + (f - f1) * (x2 - x1) / (f2 - f1)
            float position = x1 + (f_meas - f1) * (x2 - x1) / (f2 - f1);
            return position;
        }
    }

    return NAN;
}

#define MA_WINDOW 10

static float freq_buffer[MA_WINDOW] = {0};
static int freq_index = 0;
static int freq_count = 0;

// 3-point moving average
float moving_average_3(float new_sample)
{
    freq_buffer[freq_index] = new_sample;
    freq_index = (freq_index + 1) % MA_WINDOW;

    if (freq_count < MA_WINDOW)
        freq_count++;

    float sum = 0;
    for (int i = 0; i < freq_count; i++)
        sum += freq_buffer[i];

    float avg = sum / freq_count;
    // ESP_LOGI("AVG", "Freq buffer [%d samples]: avg=%.5f MHz", freq_count, avg);

    return avg;
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    printf("---- BOOTED ----\n");

    vTaskDelay(pdMS_TO_TICKS(500));
    i2c_master_init();

    // Initialize LDC1612 Channel 1
    if (ldc1612_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LDC1612. Aborting.");
        return;
    }

    // Wait for first conversion to complete
    vTaskDelay(500 / portTICK_PERIOD_MS);

// Calibration constants for Linear interpolation
#define POS_NEAR_MM 0.0
#define POS_FAR_MM 100.0
#define F_NEAR_HZ 5.904 // 0 mm
#define F_FAR_HZ 6.332  // 10 mm

    // --- Main Loop for Reading Channel 1 Data ---
    while (1)
    {
        uint16_t msb, lsb;

        esp_err_t ret_msb = ldc1612_i2c_read(REG_DATA_CH1_MSB, &msb);
        esp_err_t ret_lsb = ldc1612_i2c_read(REG_DATA_CH1_LSB, &lsb);

        if (ret_msb == ESP_OK && ret_lsb == ESP_OK)
        {
            uint8_t err_flags = (msb >> 12) & 0x0F;
            if (err_flags != 0)
            {
                ESP_LOGW(TAG, "CH1 Error flags: 0x%X", err_flags);
                if (err_flags & 0x08)
                    ESP_LOGW(TAG, "  - ERR_UR1: Under-range error");
                if (err_flags & 0x04)
                    ESP_LOGW(TAG, "  - ERR_OR1: Over-range error");
                if (err_flags & 0x02)
                    ESP_LOGW(TAG, "  - ERR_WD1: Watchdog timeout");
                if (err_flags & 0x01)
                    ESP_LOGW(TAG, "  - ERR_AE1: Amplitude error");
            }

            // Combine into 28-bit value (bits 27:0)
            // MSB contains bits [27:16], LSB contains bits [15:0]
            uint32_t raw28 = (((uint32_t)(msb & 0x0FFF)) << 16) | lsb;

            // Check for special error conditions
            if (raw28 == 0x00000000)
            {
                ESP_LOGE(TAG, "CH1 Under-range condition detected!");
            }
            else if (raw28 == 0x0FFFFFFF)
            {
                ESP_LOGE(TAG, "CH1 Over-range condition detected!");
            }

            // Calculate frequency using the formula from datasheet
            // f_sensor = (DATA × f_CLK) / (RCOUNT × 2^28)
            // const double SCALE_FACTOR = pow(2.0, 28.0) * (double)R_COUNT_VALUE;
            // double measured_freq = ((double)raw28 * CLK_SPEED) / SCALE_FACTOR;

            // If using internal clock and FREF_DIVIDER1 = 1

            const double f_ref1 = CLK_SPEED / 1.0;
            double measured_freq = ((double)raw28 * f_ref1) / pow(2.0, 28.0);
            double measured_freq_MHz = measured_freq / 1e6;
            double measured_freq_ave = moving_average_3(measured_freq_MHz);
            // Calculate change in inductance
            // double delta_L = calc_delta_L(measured_freq, BASE_FREQ, BASE_L_VALUE);

            // // Calculate absolute inductance using L = 1/(4π²f²C)
            // double calculated_L = 0;
            // if (measured_freq > 0)
            // {
            //     calculated_L = 1.0 / (4.0 * M_PI * M_PI * measured_freq * measured_freq * TANK_CAPACITANCE);
            // }

            float position_mm;
            // Look up + Interpolation
            position_mm = interpolate_position(measured_freq_ave);
            // Linear interpolation between (F_NEAR_HZ, 0 mm) and (F_FAR_HZ, 10 mm) (WORKS)
            // if (measured_freq <= F_NEAR_HZ)
            // {
            //     position_mm = POS_NEAR_MM; // clamp to 0 mm
            // }
            // else if (measured_freq >= F_FAR_HZ)
            // {
            //     position_mm = POS_FAR_MM; // clamp to 10 mm
            // }
            // else
            // {
            // double c = -((POS_FAR_MM - POS_NEAR_MM) / (F_FAR_HZ - F_NEAR_HZ)) * (F_NEAR_HZ);
            // position_mm = ((POS_FAR_MM - POS_NEAR_MM) / (F_FAR_HZ - F_NEAR_HZ)) * (measured_freq) + c;
            // }

            // ESP_LOGI(TAG, "CH1 raw: MSB=0x%04X LSB=0x%04X raw28=0x%08X (%u)",msb, lsb, (unsigned int)raw28, (unsigned int)raw28);
            ESP_LOGI(TAG, "Measured Resonant Frequency: %.5f MHz | Target position: %.3f mm", measured_freq_ave, position_mm);
            //  ESP_LOGI(TAG, "Calculated Inductance: %.6e H (%.3f µH)", calculated_L, calculated_L * 1e6);
            //  ESP_LOGI(TAG, "Change in Inductance (Delta L): %e H", delta_L);
            //  ESP_LOGI(TAG, "----------------------------------------");
        }
        else
        {
            ESP_LOGE(TAG, "I2C read failed (MSB=%s, LSB=%s)",
                     esp_err_to_name(ret_msb), esp_err_to_name(ret_lsb));
        }

        // Wait for the next measurement
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        // freq reading every 0.5s
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// // Testing if LDC1612 is detected on the I2C bus
// void app_main()
// {
//     i2c_master_init();

//     // Give I2C bus time to stabilize
//     vTaskDelay(500 / portTICK_PERIOD_MS);

//     ESP_LOGI("SCAN", "Starting I2C scan...");
//     ESP_LOGI("SCAN", "SDA=GPIO%d, SCL=GPIO%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

//     int devices_found = 0;

//     for (int addr = 1; addr < 127; addr++)
//     {
//         // Use a proper I2C probe - just send address with no data
//         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//         i2c_master_start(cmd);
//         i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
//         i2c_master_stop(cmd);
//         esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
//         i2c_cmd_link_delete(cmd);

//         if (ret == ESP_OK)
//         {
//             ESP_LOGW("SCAN", "Found device at address 0x%02X (%d)", addr, addr);
//             devices_found++;
//         }

//         vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay between probes
//     }

//     ESP_LOGI("SCAN", "Scan complete. Found %d device(s).", devices_found);

//     // Also try to read LDC1612 Device ID register (0x7F) at both possible addresses
//     ESP_LOGI("SCAN", "Attempting to read LDC1612 Device ID...");

//     uint8_t addrs_to_try[] = {0x2A, 0x2B};
//     for (int i = 0; i < 2; i++)
//     {
//         uint8_t addr = addrs_to_try[i];
//         uint8_t reg = 0x7F; // Device ID register
//         uint8_t read_buf[2];

//         esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, read_buf, 2, 500 / portTICK_PERIOD_MS);

//         if (ret == ESP_OK)
//         {
//             uint16_t device_id = (read_buf[0] << 8) | read_buf[1];
//             ESP_LOGW("SCAN", "LDC1612 found at 0x%02X! Device ID: 0x%04X", addr, device_id);
//         }
//         else
//         {
//             ESP_LOGI("SCAN", "No response at 0x%02X: %s", addr, esp_err_to_name(ret));
//         }
//     }

//     while (1)
//     {
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }
