/*

Copyright (c) 2023 N. Melih Sensoy

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


This code contains codes derived from the following projects, 
which are issued in the public domain (CC0 license)

https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/restful_server
https://github.com/espressif/esp-idf/blob/master/examples/protocols/http_server/ws_echo_server
https://github.com/espressif/esp-idf/tree/master/examples/storage/sd_card/sdmmc
https://github.com/espressif/esp32-camera/blob/master/README.md

*/

/* Arduino core headers */
#include <WiFi.h>

/* ESP-IDF headers */
#include "esp_http_server.h"
#include "cJSON.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"

/* Other libraries */
#include "esp_camera.h"

static const char *TAG = "esp-cam"; // used for tagging esp-idf logs
static const char *REST_TAG = "esp-rest"; //used for tagging esp-idf http logs

/* Wifi configuration */
const char* ssid           = "ssidtest";
const char* password       = "wifipassword";
const int   channel        = 10;
const bool  hide_SSID      = false;                  
const int   max_connection = 1;     
IPAddress local_ip(192,168,0,1);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);
IPAddress local_IP(192, 168, 1, 80);

/* Camera Pins of the CAMERA_MODEL_AI_THINKER */
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
camera_config_t config;

int port_number; //used for creating multi http server instance

#define MOUNT_POINT "/sdcard" //shows where to access sd card in the file system

esp_err_t init_sd(void){
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t* card;
    const char mount_point[] = MOUNT_POINT;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    /* we are changing spi bus to 1bit mode because 4bit mode requires 2pin and 
    it's already internally connected to the onboard led and 
    causes blink when accessing the sd card */
    slot_config.width = 1; 

    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    return ret;
}

/* Servo constants & utilities */
// https://www.mouser.com/datasheet/2/321/900-00360-Feedback-360-HS-Servo-v1.2-1147206.pdf
#define CW_T 1280 //microseconds, 140RPM
#define STOP_T 1520 //microseconds, 0RPM
#define CCW_T 1720 //microseconds, 140RPM

/* Channel 0 is used by the camera so 2 and 3 chosen for the servos*/
#define LEFT_SERVO_CHANNEL LEDC_CHANNEL_2
#define RIGHT_SERVO_CHANNEL LEDC_CHANNEL_3

/* We are Using the UART pins because there is no enough pins. */
#define LEFT_SERVO_PIN 3
#define RIGHT_SERVO_PIN 1

/* Calculates tick from uS duty cycle
   Resolution can goes up to 20bit as the LEDC_TIMER_20_BIT exists in ledc_types.h
   Most servos requires 20ms period which is 20000uS

    tControl = wanted_tick_length * one_tick_length
    wanted_tick_length = tControl/one_tick_length
    one_tick_length =  period/timer_width
    wanted_tick_length = tControl/(period/timer_width)
  
*/
static inline float calculateDuty(int t){ //tControl
  return t / (20000 / 65536.0 );
}

/* Direction enumeration */
enum DIR{
  STOP = 0,
  FORWARD = 1,
  BACK = 2,
  LEFT = 3,
  RIGHT = 4
};

/* HTTP server constants & utilities */
#define ESP_VFS_PATH_MAX 10
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)

typedef struct rest_server_context {
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
} rest_server_context_t;

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath){
    const char *type = "text/plain";
    if (CHECK_FILE_EXTENSION(filepath, ".html")) {
        type = "text/html";
    } else if (CHECK_FILE_EXTENSION(filepath, ".js")) {
        type = "application/javascript";
    } else if (CHECK_FILE_EXTENSION(filepath, ".css")) {
        type = "text/css";
    } else if (CHECK_FILE_EXTENSION(filepath, ".png")) {
        type = "image/png";
    } else if (CHECK_FILE_EXTENSION(filepath, ".ico")) {
        type = "image/x-icon";
    } else if (CHECK_FILE_EXTENSION(filepath, ".svg")) {
        type = "text/xml";
    }
    return httpd_resp_set_type(req, type);
}

/* HTTP header constants for jpeg streaming */
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/* Maps http routes to file system paths and responds with requested file content */
static esp_err_t webapp_handler(httpd_req_t *req){
    char filepath[FILE_PATH_MAX];

    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    strlcpy(filepath, rest_context->base_path, sizeof(filepath));
    if (req->uri[strlen(req->uri) - 1] == '/') {
        strlcat(filepath, "/index.html", sizeof(filepath));
    } else {
        strlcat(filepath, req->uri, sizeof(filepath));
    }
    int fd = open(filepath, O_RDONLY, 0);
    if (fd == -1) {
        ESP_LOGE(REST_TAG, "Failed to open file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filepath);

    char *chunk = rest_context->scratch;
    ssize_t read_bytes;
    do {
        /* Read file in chunks into the scratch buffer */
        read_bytes = read(fd, chunk, SCRATCH_BUFSIZE);
        if (read_bytes == -1) {
            ESP_LOGE(REST_TAG, "Failed to read file : %s", filepath);
        } else if (read_bytes > 0) {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK) {
                close(fd);
                ESP_LOGE(REST_TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);
    /* Close file after sending complete */
    close(fd);
    ESP_LOGI(REST_TAG, "File sending complete");
    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

httpd_handle_t stream_httpd = NULL;
httpd_handle_t webapp_httpd = NULL;
httpd_handle_t websocket_httpd = NULL;

/* Turns servos by the json's 'dir' object */
static void robot_command_handler(cJSON *json){
  const cJSON *direction = NULL;
  DIR parsedDirection = STOP;
  direction = cJSON_GetObjectItem(json, "dir");
  parsedDirection = (DIR)direction->valueint;

  if(parsedDirection == FORWARD){
    ledcWrite(LEFT_SERVO_CHANNEL, calculateDuty(CCW_T));
    ledcWrite(RIGHT_SERVO_CHANNEL, calculateDuty(CW_T));
  }else if(parsedDirection == BACK){
    ledcWrite(LEFT_SERVO_CHANNEL, calculateDuty(CW_T));
    ledcWrite(RIGHT_SERVO_CHANNEL, calculateDuty(CCW_T));
  }else if(parsedDirection == LEFT){
    ledcWrite(LEFT_SERVO_CHANNEL, calculateDuty(CW_T));
    ledcWrite(RIGHT_SERVO_CHANNEL, calculateDuty(CW_T));
  }else if(parsedDirection == RIGHT){
    ledcWrite(LEFT_SERVO_CHANNEL, calculateDuty(CCW_T));
    ledcWrite(RIGHT_SERVO_CHANNEL, calculateDuty(CCW_T));
  }else{
    ledcWrite(LEFT_SERVO_CHANNEL, calculateDuty(STOP_T));
    ledcWrite(RIGHT_SERVO_CHANNEL, calculateDuty(STOP_T));
  }
}

/* Tries to parse given string as json then sends to robot_command_handler */
static void ws_payload_handler(char *payload){
  //Serial.println(payload);
  const cJSON *cmd_type = NULL;
  cJSON *cmd_json = cJSON_Parse(payload);
  if (cmd_json == NULL){
      const char *error_ptr = cJSON_GetErrorPtr();
      if (error_ptr != NULL)
      {
          Serial.printf("Error before: %s\n", error_ptr);
      }
    cJSON_Delete(cmd_json);
    return;
  }

    cmd_type = cJSON_GetObjectItemCaseSensitive(cmd_json, "type");
    if (cJSON_IsString(cmd_type) && (cmd_type->valuestring != NULL))
    {
      Serial.printf("Checking type \"%s\"\n", cmd_type->valuestring);
      if(strcmp(cmd_type->valuestring, "robot") == 0){
        robot_command_handler(cmd_json);
      }
    }
}

/* Implements websocket protocol and sends text payloads to ws_payload_handler */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len) {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    }
    ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT){
      ws_payload_handler((char*)ws_pkt.payload);
    }
    
    free(buf);
    return ret;
}

/* Accesses framebuffer stored in the memory by using cam driver, 
compresses with jpeg then responses in http as chunked manner. forever. */
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }
  
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed"); 
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
          if (fb->format != PIXFORMAT_JPEG){
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            esp_camera_fb_return(fb);
            fb = NULL;
            if (!jpeg_converted){
                log_e("JPEG compression failed");
                res = ESP_FAIL;
            }
        }else{
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }
      }
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      log_e("Send frame failed");
      break;
    }
  }
  last_frame = 0;
  return res;
}

/* Creates the all http server instances and their socket listener freertos tasks.
  These tasks will execute their assigned handler function when a http request made.
 */
void start_server(const char *base_path){
  rest_server_context_t *rest_context = (rest_server_context_t*)calloc(1, sizeof(rest_server_context_t));
  strlcpy(rest_context->base_path, base_path, sizeof(rest_context->base_path));
  
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.uri_match_fn = httpd_uri_match_wildcard;

  httpd_uri_t webapp_get_uri = {
    .uri = "/*",
    .method = HTTP_GET,
    .handler = webapp_handler,
    .user_ctx = rest_context
  };
  
  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

   httpd_uri_t ws_uri = {
    .uri        = "/ws",
    .method     = HTTP_GET,
    .handler    = ws_handler,
    .user_ctx   = NULL,
    .is_websocket = true,
    .handle_ws_control_frames = true
  };

  Serial.printf("Web server started on port: '%d'\n", config.server_port);
  if (httpd_start(&webapp_httpd, &config) == ESP_OK) {
    //assigning request handler for port 80
    httpd_register_uri_handler(webapp_httpd, &webapp_get_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;

  Serial.printf("Stream server started on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    //assigning request handler for port 81
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }

  port_number = config.server_port;
  config.server_port += 1;
  config.ctrl_port += 1;

  Serial.printf("Websocket server started on port: '%d'\n", config.server_port);
  if (httpd_start(&websocket_httpd, &config) == ESP_OK) {
    //assigning request handler for port 82
    httpd_register_uri_handler(websocket_httpd, &ws_uri);
  }
}


void init_servos(){
  /* Most servos requires 50hz pwm signal */
  ledcSetup(LEFT_SERVO_CHANNEL, 50/*hz*/, LEDC_TIMER_16_BIT);
  ledcSetup(RIGHT_SERVO_CHANNEL, 50/*hz*/, LEDC_TIMER_16_BIT);

  ledcAttachPin(LEFT_SERVO_PIN, LEFT_SERVO_CHANNEL);
  ledcAttachPin(RIGHT_SERVO_PIN, RIGHT_SERVO_CHANNEL);
}

/* Creates wifi access point */
void init_wifi(){
  Serial.println("\n[*] Creating AP");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password, channel, hide_SSID, max_connection);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());
}

void init_camera(){
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 16000000; //20000000;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;//CAMERA_GRAB_LATEST; //CAMERA_GRAB_WHEN_EMPTY
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.frame_size = FRAMESIZE_HD;
  config.jpeg_quality = 20;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void setup(){
  //Serial.begin(115200); //commented out when debugging
  init_servos(); //uncommented when debugging
  init_camera();
  init_sd();
  init_wifi();
  start_server(MOUNT_POINT);
}

void loop(){
  delay(10);
}