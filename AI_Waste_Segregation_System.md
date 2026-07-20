**AI-Powered Plastic & Paper**

**Waste Segregation System**

_On-device TinyML waste classification & automated sorting using the Seeed XIAO ESP32S3 Sense_

# **Aim**

Develop an embedded AI-powered prototype that automatically identifies plastic and paper waste in real time and physically segregates it into the correct bin - without relying on the cloud or manual sorting.

# **Our Solution**

We built a low-cost, self-contained waste segregation device using a **Seeed XIAO ESP32S3 Sense** microcontroller with an onboard camera. Unlike server-based classification systems, our model runs entirely **on-device (TinyML)** using Edge Impulse, so the prototype can classify and sort waste instantly, with no internet connection required.

_Prototype hardware - camera module and assembled sorting unit_

# **Workflow of the Waste Segregation System**

- **Object Input:** An item (plastic or paper) is placed in front of the device's camera.
- **Image Capture:** The onboard OV2640 camera captures a QVGA (320×240) frame.
- **Image Processing:** The frame is center-cropped to 240×240 and converted from RGB565 to RGB888 to match the model's input format.
- **Feature Extraction & Classification:** The processed image is passed to an Edge Impulse-trained image classification model running directly on the ESP32S3, which extracts features and evaluates them against the trained classes.
- **Predicted Category:** The model outputs a confidence score for each class - PLASTIC or PAPER.
- **Stability Check:** The prediction must stay consistent with confidence ≥ 0.85 for 3 consecutive frames before being accepted, filtering out noisy/false detections.
- **Waste Segregation & Disposal:** Once confirmed, a servo motor rotates a diverter arm to drop the item into the corresponding bin, then returns to its resting position.

# **Key Features**

## **1\. Automated Sorting Process**

- Uses an Edge Impulse-trained image classification model to distinguish between plastic and paper waste.
- Once identified, the item is automatically routed to the correct bin via a servo-actuated diverter - left (45°) for plastic, right (135°) for paper.

## **2\. On-Device (Edge) Inference**

- The entire classification pipeline - capture, preprocessing, and inference - runs locally on the ESP32S3, with no cloud dependency or internet connection required.
- This enables fast, real-time decisions suitable for a compact, standalone hardware unit.

## **3\. False-Positive Filtering**

- A confidence threshold (0.85) combined with a 3-frame stability check ensures the servo only acts on confirmed, consistent detections rather than a single noisy prediction.

# **Hardware**

| **Component**                   | **Purpose**                                                                |
| ------------------------------- | -------------------------------------------------------------------------- |
| Seeed Studio XIAO ESP32S3 Sense | Main controller - dual-core ESP32-S3 with built-in OV2640 camera and PSRAM |
| OV2640 Camera Module (onboard)  | Captures the waste item image for classification                           |
| Micro Servo Motor               | Diverts the item into the plastic or paper bin                             |
| Two-compartment bin/chute       | Physical enclosure with a servo-actuated divider gate                      |
| 5V power supply                 | Powers the controller and servo                                            |

# **Camera Pin Mapping (XIAO ESP32S3 Sense)**

| **Signal**    | **GPIO**                       |
| ------------- | ------------------------------ |
| XCLK          | 10                             |
| SIOD (SDA)    | 40                             |
| SIOC (SCL)    | 39                             |
| Y9-Y2 (D7-D0) | 48, 11, 12, 14, 16, 18, 17, 15 |
| VSYNC         | 38                             |
| HREF          | 47                             |
| PCLK          | 13                             |

## **Servo Wiring**

| **Signal** | **Pin** |
| ---------- | ------- |
| Signal     | D0      |
| VCC        | 5V      |
| GND        | GND     |

# **Development Process**

- **Data Collection:** Collected a dataset of plastic and paper waste images to train the classification model.
- **Model Training:** Trained an image classification model using Edge Impulse Studio, targeting a lightweight architecture suitable for deployment on the ESP32S3's limited memory and compute.
- **Model Deployment:** Exported the trained model as an Arduino library (Harish_007-project-1_inferencing.h) and integrated it into the firmware alongside the ESP32 camera driver and ESP32Servo library.
- **Firmware Development:** Built the capture → crop/convert → classify → stability-check → servo-actuate pipeline in Arduino C++ (waste_detection_ino.ino).
- **Parameter Tuning:** Tuned CONFIDENCE_THRESHOLD and REQUIRED_STABLE_FRAMES to balance responsiveness against false-positive sorting actions.
- **Hardware Integration:** Mounted the camera and servo to a two-bin enclosure and calibrated the servo angles (45° / 90° / 135°) for accurate diversion.
- **Continuous Evaluation:** Tested the system with varied plastic and paper items and lighting conditions, refining the model and thresholds for reliable real-world sorting.

# **Challenges**

- Ambiguous Object Identification: Some items (e.g., glossy paper vs. thin plastic film) can be visually similar, occasionally causing misclassification.
- Lighting & Camera Conditions: Varying ambient light and camera framing affect classification confidence and consistency.
- On-Device Resource Constraints: Fitting a reasonably accurate image classification model within the ESP32S3's limited memory and compute budget required careful model size and input resolution trade-offs.
- Mechanical Timing: Coordinating servo movement timing with item drop so sorting is reliable without jamming or double-triggering.

# **Software & Dependencies**

- Arduino IDE with ESP32 board support
- Board: XIAO_ESP32S3 (Sense variant), PSRAM enabled
- Libraries: esp_camera.h (ESP32 Arduino core), ESP32Servo, Edge Impulse Arduino library exported from the trained project

# **Setup Instructions**

- Install the ESP32 board package in Arduino IDE.
- Select Tools > Board > Seeed XIAO ESP32S3, enable PSRAM, and set Partition Scheme to a camera-compatible option.
- Install the ESP32Servo library via Library Manager.
- Export the trained model from Edge Impulse Studio as an Arduino library and import via Sketch > Include Library > Add .ZIP Library.
- Open waste_detection_ino.ino, select the correct COM port, and upload.
- Open Serial Monitor at 115200 baud to view live classification output.

# **Project Structure**

.

├── waste_detection_ino.ino # Main firmware (camera + inference + servo control)

├── Harish_007-project-1_inferencing.h # Exported Edge Impulse model library

└── README.md

# **Future Improvements**

- Expand classification to more categories (metal, glass, organic waste)
- Add an IR/ultrasonic proximity sensor to auto-trigger capture instead of continuous polling
- Log sorting statistics (counts, timestamps) to an SD card or over Wi-Fi/MQTT
- Add an LCD/OLED display or LED indicators for live status
- Design and share a 3D-printed enclosure

# **Contributors**

| **Harish Maya.V**    | **Akthar Barvish Khan.A** |
| -------------------- | ------------------------- |
| **Dharshini Elango** | **Kamatchi.S**            |