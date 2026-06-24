/* Includes ---------------------------------------------------------------- */
#include <Harish_007-project-1_inferencing.h>
#include "esp_camera.h"
#include <ESP32Servo.h>

/* Camera Pin Configuration - XIAO ESP32S3 Sense */
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

/* Servo Configuration */
Servo wasteServo;
const int servoPin = D0;

/* Servo Positions */
const int CENTER_POS = 90;
const int LEFT_POS   = 45;    // Plastic
const int RIGHT_POS  = 135;   // Paper

/* Detection Settings */
const float CONFIDENCE_THRESHOLD = 0.85;
const int REQUIRED_STABLE_FRAMES = 3;

int stableFrameCount = 0;
String lastDetectedClass = "None";

static bool is_initialised = false;

/* Edge Impulse Settings */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS EI_CLASSIFIER_INPUT_WIDTH
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS EI_CLASSIFIER_INPUT_HEIGHT

static uint8_t frame_buffer[
    EI_CAMERA_RAW_FRAME_BUFFER_COLS *
    EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3
];

/* Camera Configuration */
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM
};

/* Function Prototypes */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void moveServoAndReturn(int angle);

/* Setup */
void setup() {
    Serial.begin(115200);
    while (!Serial);

    wasteServo.attach(servoPin);
    wasteServo.write(CENTER_POS);

    delay(1000);

    if (!ei_camera_init()) {
        Serial.println("Camera initialization failed!");
        return;
    }

    Serial.println("Camera initialized.");
    Serial.println("Waste Segregation System Ready.");
}

/* Main Loop */
void loop() {

    if (!is_initialised) {
        return;
    }

    signal_t signal;
    signal.total_length =
        EI_CAMERA_RAW_FRAME_BUFFER_COLS *
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    signal.get_data = &ei_camera_get_data;

    if (!ei_camera_capture(
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            frame_buffer)) {

        Serial.println("Image capture failed");
        return;
    }

    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR res =
        run_classifier(&signal, &result, false);

    if (res != EI_IMPULSE_OK) {
        Serial.printf("Classifier Error: %d\n", res);
        return;
    }

    String detectedClass = "UNKNOWN";
    float highestScore = 0;

    Serial.println("---------------");

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {

        Serial.printf("%s : %.3f\n",
                      result.classification[ix].label,
                      result.classification[ix].value);

        if (result.classification[ix].value > highestScore) {

            highestScore =
                result.classification[ix].value;

            detectedClass =
                result.classification[ix].label;
        }
    }

    Serial.println("---------------");

    /* Stability Check */

    if (highestScore >= CONFIDENCE_THRESHOLD) {

        if (detectedClass == lastDetectedClass) {

            stableFrameCount++;

        } else {

            stableFrameCount = 1;
            lastDetectedClass = detectedClass;
        }

    } else {

        stableFrameCount = 0;
        lastDetectedClass = "None";
    }

    /* Confirm Detection */
if (stableFrameCount >= REQUIRED_STABLE_FRAMES){

        Serial.printf(
            "\n>>> DETECTED: %s (%.2f) <<<\n",
            detectedClass.c_str(),
            highestScore);

        if (detectedClass == "PLASTIC") {

            Serial.println("PLASTIC -> LEFT");
            moveServoAndReturn(LEFT_POS);

        }
        else if (detectedClass == "PAPER") {

            Serial.println("PAPER -> RIGHT");
            moveServoAndReturn(RIGHT_POS);
        }

        stableFrameCount = 0;
        lastDetectedClass = "None";
    }

    delay(50);
}

/* Servo Movement */
void moveServoAndReturn(int angle) {

    wasteServo.write(angle);
    delay(1500);

    wasteServo.write(CENTER_POS);
    delay(1000);
}
/* Edge Impulse Data Callback */
int ei_camera_get_data(size_t offset,
                       size_t length,
                       float *out_ptr) {

    size_t pixel_ix = offset * 3;

    for (size_t i = 0; i < length; i++) {

        uint8_t r = frame_buffer[pixel_ix];
        uint8_t g = frame_buffer[pixel_ix + 1];
        uint8_t b = frame_buffer[pixel_ix + 2];

        out_ptr[i] =
            ((uint32_t)r << 16) +
            ((uint32_t)g << 8) +
            b;

        pixel_ix += 3;
    }

    return 0;
}

/* Camera Init */
bool ei_camera_init(void) {

    if (is_initialised) {
        return true;
    }

    esp_err_t err = esp_camera_init(&camera_config);

    if (err != ESP_OK) {

        Serial.printf(
            "Camera init failed: 0x%x\n",
            err);

        return false;
    }

    sensor_t *s = esp_camera_sensor_get();

    if (s->id.PID == OV2640_PID) {
        s->set_vflip(s, 1);
    }

    is_initialised = true;
    return true;
}

/* Camera Deinit */
void ei_camera_deinit(void) {

    esp_camera_deinit();
    is_initialised = false;
}

/* Capture and Resize */
bool ei_camera_capture(
    uint32_t img_width,
    uint32_t img_height,
    uint8_t *out_buf) {

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        return false;
    }

    uint32_t src_w = 320;
    uint32_t src_h = 240;

    uint32_t crop_size = 240;

    uint32_t start_x =
        (src_w - crop_size) / 2;

    uint32_t start_y =
        (src_h - crop_size) / 2;

    uint32_t pixel_count = 0;

    for (uint32_t y = 0; y < img_height; y++) {

        uint32_t src_y =
            start_y + (y * crop_size) / img_height;

        uint16_t *src_row =
            (uint16_t *)fb->buf + (src_y * src_w);

        for (uint32_t x = 0; x < img_width; x++) {

            uint32_t src_x =
                start_x + (x * crop_size) / img_width;

            uint16_t pixel = src_row[src_x];

            pixel = (pixel >> 8) | (pixel << 8);

            uint8_t r =
                ((pixel >> 11) & 0x1F) << 3;

            uint8_t g =
                ((pixel >> 5) & 0x3F) << 2;

            uint8_t b =
                (pixel & 0x1F) << 3;

            out_buf[pixel_count * 3] = r;
            out_buf[pixel_count * 3 + 1] = g;
            out_buf[pixel_count * 3 + 2] = b;

            pixel_count++;
        }
    }

    esp_camera_fb_return(fb);

    return true;
}