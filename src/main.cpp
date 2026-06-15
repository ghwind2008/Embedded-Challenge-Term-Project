#include "mbed.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "lsm6dsl.h"
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/gatt/GattService.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "events/EventQueue.h"

using namespace std::chrono;
using namespace ble;
using namespace events;

BufferedSerial pc(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) { return &pc; }

// ====== I2C PINS ======
I2C i2c(PB_11, PB_10);

// ====== SAMPLING PARAMETERS ======
static constexpr int SAMPLE_RATE_HZ = 52;
static constexpr int WINDOW_SECONDS = 3;
static constexpr int CAPTURE_SAMPLES = SAMPLE_RATE_HZ * WINDOW_SECONDS;
static constexpr int FFT_SIZE = 256;

// ====== HARDWARE ======
DigitalOut physical_led1(LED1);
DigitalOut physical_led2(LED2);
DigitalOut buzzer(D3);

Ticker sampleTicker;

// ====== DATA BUFFERS ======
static float accel_x[CAPTURE_SAMPLES];
static float accel_y[CAPTURE_SAMPLES];
static float accel_z[CAPTURE_SAMPLES];
static float gyro_x[CAPTURE_SAMPLES];
static float gyro_y[CAPTURE_SAMPLES];
static float gyro_z[CAPTURE_SAMPLES];
static float fft_in[FFT_SIZE];
static float fft_out[FFT_SIZE];

// ====== CONTROL FLAGS ======
static volatile uint32_t pendingSamples = 0;
static volatile int sampleIndex = 0;
static volatile bool dataReady = false;

// ====== DETECTION STATE ======
static uint8_t tremor_state = 0;
static uint8_t dysk_state = 0;
static uint8_t fog_state = 0;

// ====== THRESHOLDS ======
static constexpr float MAG_THRESHOLD = 0.25f;
static constexpr float MIN_PROMINENCE = 0.08f;
static constexpr float MIN_PEAK_RATIO = 1.3f;
static constexpr float FOG_STEP_THRESHOLD = 0.6f;
static constexpr float FOG_WALKING_THRESHOLD = 0.15f;
static constexpr float FOG_FREEZE_THRESHOLD = 0.12f;
static constexpr int FOG_MIN_STEPS = 4;
static constexpr int FOG_MIN_WALKING_WINDOWS = 2;
static constexpr int FOG_MIN_FREEZE_WINDOWS = 2;

// ====== FOG DETECTOR ======
struct FOGDetector {
    bool is_walking = false;
    bool was_walking = false;
    int step_count = 0;
    int walking_windows = 0;
    int still_windows = 0;
};
static FOGDetector fog_detector;

// ====== BLE SETUP - Following course pattern ======
BLE &ble_interface = BLE::Instance();
EventQueue event_queue;

// Service and characteristic UUIDs
const UUID PD_SERVICE_UUID(0xA000);
const UUID TREMOR_CHAR_UUID(0xA001);
const UUID DYSK_CHAR_UUID(0xA002);
const UUID FOG_CHAR_UUID(0xA003);

// Characteristic values
uint8_t tremor_value = 0;
uint8_t dysk_value = 0;
uint8_t fog_value = 0;

// Create characteristics
ReadOnlyGattCharacteristic<uint8_t> tremorCharacteristic(
    TREMOR_CHAR_UUID,
    &tremor_value,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

ReadOnlyGattCharacteristic<uint8_t> dyskCharacteristic(
    DYSK_CHAR_UUID,
    &dysk_value,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

ReadOnlyGattCharacteristic<uint8_t> fogCharacteristic(
    FOG_CHAR_UUID,
    &fog_value,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

// Put characteristics in table
GattCharacteristic *charTable[] = {&tremorCharacteristic, &dyskCharacteristic, &fogCharacteristic};

// Create service
GattService pdService(PD_SERVICE_UUID, charTable, 3);

// ====== SENSOR FUNCTIONS ======
static void readAccelerometer(float &x, float &y, float &z) {
    int16_t rawX, rawY, rawZ;
    if (LSM6DSL_ReadAccelXYZ(&rawX, &rawY, &rawZ) == 0) {
        x = rawX / 16384.0f;
        y = rawY / 16384.0f;
        z = rawZ / 16384.0f;
    } else {
        x = y = z = 0.0f;
    }
}

static void readGyroscope(float &x, float &y, float &z) {
    int16_t rawX, rawY, rawZ;
    if (LSM6DSL_ReadGyroXYZ(&rawX, &rawY, &rawZ) == 0) {
        x = rawX * 0.00875f;
        y = rawY * 0.00875f;
        z = rawZ * 0.00875f;
    } else {
        x = y = z = 0.0f;
    }
}

static void sampleTrigger() {
    pendingSamples++;
}

static inline float hann(int n, int N) {
    const float pi = 3.14159265358979323846f;
    const float angle = 2.0f * pi * (float)n / (float)(N - 1);
    return 0.5f - 0.5f * cosf(angle);
}

static inline float magnitude3D(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

// ====== FOG DETECTION ======
static uint8_t detectFreezingGait() {
    float total_movement = 0.0f;
    int potential_steps = 0;
    
    for (int i = 1; i < CAPTURE_SAMPLES; i++) {
        float curr_mag = magnitude3D(accel_x[i], accel_y[i], accel_z[i]);
        float prev_mag = magnitude3D(accel_x[i-1], accel_y[i-1], accel_z[i-1]);
        float delta = fabsf(curr_mag - prev_mag);
        
        total_movement += delta;
        if (delta > FOG_STEP_THRESHOLD) potential_steps++;
    }
    
    float avg_movement = total_movement / CAPTURE_SAMPLES;
    
    if (potential_steps >= FOG_MIN_STEPS && avg_movement > FOG_WALKING_THRESHOLD) {
        fog_detector.walking_windows++;
        fog_detector.still_windows = 0;
        fog_detector.step_count = potential_steps;
        
        if (fog_detector.walking_windows >= FOG_MIN_WALKING_WINDOWS) {
            fog_detector.is_walking = true;
            fog_detector.was_walking = true;
        }
    }
    else if (fog_detector.was_walking && avg_movement < FOG_FREEZE_THRESHOLD) {
        fog_detector.still_windows++;
        fog_detector.walking_windows = 0;
        
        if (fog_detector.still_windows >= FOG_MIN_FREEZE_WINDOWS) {
            printf("\n!!! FREEZING GAIT DETECTED !!!\n");
            fog_detector.is_walking = false;
            fog_detector.was_walking = false;
            fog_detector.still_windows = 0;
            return 1;
        }
    }
    else if (avg_movement < 0.05f) {
        fog_detector.walking_windows = 0;
        fog_detector.still_windows = 0;
        fog_detector.is_walking = false;
    }
    
    return 0;
}

// ====== UPDATE BLE ======
void send_ble_notification() {
    tremor_value = tremor_state;
    dysk_value = dysk_state;
    fog_value = fog_state;
    
    ble_interface.gattServer().write(
        tremorCharacteristic.getValueHandle(),
        &tremor_value,
        sizeof(tremor_value)
    );
    
    ble_interface.gattServer().write(
        dyskCharacteristic.getValueHandle(),
        &dysk_value,
        sizeof(dysk_value)
    );
    
    ble_interface.gattServer().write(
        fogCharacteristic.getValueHandle(),
        &fog_value,
        sizeof(fog_value)
    );
}

// ====== FFT PROCESSING ======
static void processFFTWindow() {
    float accel_mag[CAPTURE_SAMPLES];
    for (int i = 0; i < CAPTURE_SAMPLES; i++) {
        accel_mag[i] = magnitude3D(accel_x[i], accel_y[i], accel_z[i]);
    }
    
    float mean = 0.0f;
    for (int i = 0; i < CAPTURE_SAMPLES; i++) mean += accel_mag[i];
    mean /= (float)CAPTURE_SAMPLES;
    
    for (int i = 0; i < FFT_SIZE; i++) fft_in[i] = 0.0f;
    for (int i = 0; i < CAPTURE_SAMPLES; i++) {
        fft_in[i] = (accel_mag[i] - mean) * hann(i, FFT_SIZE);
    }
    
    static arm_rfft_fast_instance_f32 fft_instance;
    static bool fft_initialized = false;
    if (!fft_initialized) {
        arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
        fft_initialized = true;
    }
    arm_rfft_fast_f32(&fft_instance, fft_in, fft_out, 0);
    
    static float mags[FFT_SIZE/2 + 1];
    mags[0] = fabsf(fft_out[0]);
    mags[FFT_SIZE/2] = fabsf(fft_out[1]);
    for (int k = 1; k < FFT_SIZE/2; k++) {
        float re = fft_out[2*k];
        float im = fft_out[2*k + 1];
        mags[k] = sqrtf(re*re + im*im);
    }
    
    const float freq_res = (float)SAMPLE_RATE_HZ / (float)FFT_SIZE;
    int k_start = (int)ceilf(3.0f / freq_res);
    int k_end = (int)floorf(7.0f / freq_res);
    if (k_start < 1) k_start = 1;
    if (k_end > FFT_SIZE/2 - 1) k_end = FFT_SIZE/2 - 1;
    
    float max_mag = 0.0f;
    int max_k = k_start;
    for (int k = k_start; k <= k_end; k++) {
        if (mags[k] > max_mag) {
            max_mag = mags[k];
            max_k = k;
        }
    }
    
    float detected_freq = max_k * freq_res;
    
    float left_neighbor = (max_k > k_start) ? mags[max_k - 1] : 0.0f;
    float right_neighbor = (max_k < k_end) ? mags[max_k + 1] : 0.0f;
    float avg_neighbor = (left_neighbor + right_neighbor) / 2.0f;
    
    float prominence = max_mag - avg_neighbor;
    float peak_ratio = (avg_neighbor > 0.001f) ? (max_mag / avg_neighbor) : 0.0f;
    
    bool valid_signal = (max_mag > MAG_THRESHOLD && 
                        prominence > MIN_PROMINENCE && 
                        peak_ratio > MIN_PEAK_RATIO);
    
    tremor_state = 0;
    dysk_state = 0;
    
    if (valid_signal) {
        if (detected_freq >= 3.0f && detected_freq < 5.0f) {
            tremor_state = 1;
            printf("\n>>> TREMOR DETECTED <<<\n");
            printf("  Freq: %.2f Hz | Mag: %.3f\n", detected_freq, max_mag);
        }
        else if (detected_freq >= 5.0f && detected_freq <= 7.0f) {
            dysk_state = 1;
            printf("\n>>> DYSKINESIA DETECTED <<<\n");
            printf("  Freq: %.2f Hz | Mag: %.3f\n", detected_freq, max_mag);
        }
    }
    else {
        if (max_mag <= MAG_THRESHOLD) {
            printf("No detection: mag=%.3f too low\n", max_mag);
        }
    }
    
    fog_state = detectFreezingGait();
    
    physical_led1 = 0;
    physical_led2 = 0;
    buzzer = 0;
    
    if (tremor_state) {
        physical_led1 = 1;
        buzzer = 1;
        wait_us(200000);
        buzzer = 0;
    }
    else if (dysk_state) {
        physical_led2 = 1;
        buzzer = 1;
        wait_us(150000);
        buzzer = 0;
        wait_us(100000);
        buzzer = 1;
        wait_us(150000);
        buzzer = 0;
    }
    else if (fog_state) {
        physical_led1 = 1;
        physical_led2 = 1;
        buzzer = 1;
        wait_us(500000);
        buzzer = 0;
    }
    
    // Send BLE notification
    send_ble_notification();
}

// ====== BLE INITIALIZATION CALLBACK - Following course pattern ======
void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params) {
    if (params->error != BLE_ERROR_NONE) {
        printf("BLE initialization failed\n");
        return;
    }
    
    printf("BLE initialized successfully\n");
    
    // Add service to GATT server
    ble_interface.gattServer().addService(pdService);
    
    // Create advertising data buffer
    uint8_t adv_buffer[LEGACY_ADVERTISING_MAX_SIZE];
    AdvertisingDataBuilder adv_data(adv_buffer);
    
    // Set standard BLE flags
    adv_data.setFlags();
    
    // Set device name
    adv_data.setName("PD-Monitor");
    
    // Configure advertising parameters
    ble_interface.gap().setAdvertisingParameters(
        LEGACY_ADVERTISING_HANDLE,
        AdvertisingParameters(advertising_type_t::CONNECTABLE_UNDIRECTED, adv_interval_t(160))
    );
    
    // Set advertising payload
    ble_interface.gap().setAdvertisingPayload(
        LEGACY_ADVERTISING_HANDLE,
        adv_data.getAdvertisingData()
    );
    
    // Start advertising
    ble_interface.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);
    
    printf("BLE advertising started as PD-Monitor\n");
}

// ====== BLE EVENT HANDLER - Following course pattern ======
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(callback(&ble_interface, &BLE::processEvents));
}

// ====== DETECTION TASK ======
void detection_task() {
    static uint32_t window_count = 0;
    
    if (pendingSamples > 0 && sampleIndex < CAPTURE_SAMPLES) {
        pendingSamples--;
        
        float ax, ay, az, gx, gy, gz;
        readAccelerometer(ax, ay, az);
        readGyroscope(gx, gy, gz);
        
        accel_x[sampleIndex] = ax;
        accel_y[sampleIndex] = ay;
        accel_z[sampleIndex] = az;
        gyro_x[sampleIndex] = gx;
        gyro_y[sampleIndex] = gy;
        gyro_z[sampleIndex] = gz;
        
        sampleIndex++;
        
        if (sampleIndex >= CAPTURE_SAMPLES) {
            dataReady = true;
            sampleTicker.detach();
        }
    }
    
    if (dataReady) {
        window_count++;
        printf("[Window %lu] ", window_count);
        
        processFFTWindow();
        
        printf("T=%d D=%d F=%d\n\n", tremor_state, dysk_state, fog_state);
        
        sampleIndex = 0;
        dataReady = false;
        pendingSamples = 0;
        sampleTicker.attach(&sampleTrigger, 19231us);
    }
}

int main() {
    wait_us(2000000);
    
    printf("\n========================================\n");
    printf("  Parkinson's Disease Monitor\n");
    printf("========================================\n\n");
    
    physical_led1 = 0;
    physical_led2 = 0;
    buzzer = 0;
    
    // Initialize sensor
    printf("Initializing sensor...\n");
    if (LSM6DSL_Init(&i2c) != 0) {
        printf("ERROR: Sensor init failed!\n");
        while(1) {
            physical_led1 = !physical_led1;
            wait_us(200000);
        }
    }
    
    uint8_t whoami = 0;
    LSM6DSL_ReadWhoAmI(&whoami);
    printf("WHO_AM_I: 0x%02X %s\n\n", whoami, whoami == 0x6A ? "OK" : "FAIL");
    
    LSM6DSL_SetAccelODR(52.0f);
    LSM6DSL_SetAccelFullScale(2);
    LSM6DSL_SetGyroODR(52.0f);
    LSM6DSL_SetGyroFullScale(245);
    
    printf("Starting BLE...\n");
    
    // Set up BLE event handler
    ble_interface.onEventsToProcess(schedule_ble_events);
    
    // Initialize BLE with callback
    ble_interface.init(on_ble_init_complete);
    
    printf("\n========================================\n");
    printf("Monitoring started\n");
    printf("Service: 0xA000\n");
    printf("Tremor: 0xA001 | Dysk: 0xA002 | FOG: 0xA003\n");
    printf("========================================\n\n");
    
    // Start sampling
    sampleTicker.attach(&sampleTrigger, 19231us);
    
    // Schedule detection task to run repeatedly
    event_queue.call_every(10ms, detection_task);
    
    // Run event queue forever
    event_queue.dispatch_forever();
}
