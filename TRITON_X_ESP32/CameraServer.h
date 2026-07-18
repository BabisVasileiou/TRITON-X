/********** TRITON-X camera server
 * Purpose
 *   Provides an optional QVGA OV2640 MJPEG stream on TCP port 82.
 * Hardware
 *   AI-Thinker ESP32-CAM with PSRAM and OV2640 camera.
 * Software
 *   Uses esp_camera and the ESP-IDF HTTP server. Camera streaming is
 *   disabled at boot and controlled independently from the robot mode.
 * Safety
 *   The camera never sends commands to the Arduino motor controller.
 * Reference
 *   v4.0 Final, based on the Espressif CameraWebServer architecture,
 *   C. Vasileiou, July 2026.
 **********/

#ifndef TRITON_X_CAMERA_SERVER_H
#define TRITON_X_CAMERA_SERVER_H

#include <Arduino.h>
#include "esp_camera.h"
#include "esp_http_server.h"

bool cameraReady = false;
bool cameraAvailable = false;
volatile bool cameraEnabled = false;
esp_err_t cameraLastError = ESP_OK;
httpd_handle_t cameraHttpServer = NULL;

static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=frame";
static const char* STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char* STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/******** function cameraStreamHandler
 * Purpose
 *   Serves MJPEG frames while camera streaming is enabled.
 * Arguments
 *   request ESP-IDF HTTP request context.
 * Results
 *   Returns an ESP-IDF success or failure status.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
esp_err_t cameraStreamHandler(httpd_req_t* request) {
  if (!cameraReady || !cameraEnabled) {
    httpd_resp_set_status(request, "503 Service Unavailable");
    httpd_resp_set_type(request, "text/plain");
    return httpd_resp_send(request, "Camera disabled", HTTPD_RESP_USE_STRLEN);
  }

  esp_err_t result = httpd_resp_set_type(
    request, STREAM_CONTENT_TYPE
  );
  if (result != ESP_OK) {
    return result;
  }
  httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(request, "Cache-Control", "no-store");

  char partBuffer[64];
  while (cameraEnabled) {
    camera_fb_t* frame = esp_camera_fb_get();
    if (frame == NULL) {
      result = ESP_FAIL;
      break;
    }

    size_t headerLength = snprintf(
      partBuffer,
      sizeof(partBuffer),
      STREAM_PART,
      frame->len
    );

    result = httpd_resp_send_chunk(
      request, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)
    );
    if (result == ESP_OK) {
      result = httpd_resp_send_chunk(
        request, partBuffer, headerLength
      );
    }
    if (result == ESP_OK) {
      result = httpd_resp_send_chunk(
        request,
        (const char*)frame->buf,
        frame->len
      );
    }

    esp_camera_fb_return(frame);
    if (result != ESP_OK) {
      break;
    }
    delay(CAMERA_FRAME_INTERVAL_MS);
  }
  return result;
}

/******** function initializeCamera
 * Purpose
 *   Configures and initializes the AI-Thinker OV2640 camera.
 * Arguments
 *   None.
 * Results
 *   Returns true when the reported condition or operation succeeds.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
bool initializeCamera(void) {
  camera_config_t config;
  memset(&config, 0, sizeof(config));
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_Y2_PIN;
  config.pin_d1 = CAM_Y3_PIN;
  config.pin_d2 = CAM_Y4_PIN;
  config.pin_d3 = CAM_Y5_PIN;
  config.pin_d4 = CAM_Y6_PIN;
  config.pin_d5 = CAM_Y7_PIN;
  config.pin_d6 = CAM_Y8_PIN;
  config.pin_d7 = CAM_Y9_PIN;
  config.pin_xclk = CAM_XCLK_PIN;
  config.pin_pclk = CAM_PCLK_PIN;
  config.pin_vsync = CAM_VSYNC_PIN;
  config.pin_href = CAM_HREF_PIN;
  config.pin_sccb_sda = CAM_SIOD_PIN;
  config.pin_sccb_scl = CAM_SIOC_PIN;
  config.pin_pwdn = CAM_PWDN_PIN;
  config.pin_reset = CAM_RESET_PIN;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 14;
  config.fb_count = psramFound() ? 2 : 1;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = psramFound() ?
    CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  cameraLastError = esp_camera_init(&config);
  if (cameraLastError != ESP_OK) {
    cameraReady = false;
    cameraAvailable = false;
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor != NULL) {
    sensor->set_framesize(sensor, FRAMESIZE_QVGA);
    sensor->set_quality(sensor, 14);
  }
  cameraReady = true;
  cameraEnabled = false;
  return true;
}

/******** function startCameraServer
 * Purpose
 *   Starts the dedicated port-82 camera HTTP server.
 * Arguments
 *   None.
 * Results
 *   Returns true when the reported condition or operation succeeds.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
bool startCameraServer(void) {
  httpd_config_t serverConfig = HTTPD_DEFAULT_CONFIG();
  serverConfig.server_port = CAMERA_PORT;
  serverConfig.ctrl_port = CAMERA_PORT + 1000;
  serverConfig.max_open_sockets = 2;
  serverConfig.stack_size = 8192;
  serverConfig.lru_purge_enable = true;

  httpd_uri_t streamUri;
  memset(&streamUri, 0, sizeof(streamUri));
  streamUri.uri = "/stream";
  streamUri.method = HTTP_GET;
  streamUri.handler = cameraStreamHandler;
  streamUri.user_ctx = NULL;

  if (httpd_start(&cameraHttpServer, &serverConfig) != ESP_OK) {
    cameraHttpServer = NULL;
    cameraAvailable = false;
    return false;
  }

  esp_err_t registerResult = httpd_register_uri_handler(
    cameraHttpServer, &streamUri
  );
  if (registerResult != ESP_OK) {
    httpd_stop(cameraHttpServer);
    cameraHttpServer = NULL;
    cameraAvailable = false;
    return false;
  }

  cameraAvailable = true;
  return true;
}

/******** function setCameraEnabled
 * Purpose
 *   Applies the requested camera stream enable state.
 * Arguments
 *   enabled Requested camera enable state.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void setCameraEnabled(bool enabled) {
  cameraEnabled = cameraReady && enabled;
}

#endif
