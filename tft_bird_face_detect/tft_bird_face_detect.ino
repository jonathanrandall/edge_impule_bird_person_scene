/*
 *This code outline follows robotzero1 selfiecam code 
 * https://github.com/robotzero1/esp32cam-selfiecam
 * and uses for processing the edge impulse code
 * https://github.com/edgeimpulse/example-esp32-cam
 * Note: the edge impulse code requires an install of esp32 board 1.0.6
 * which can be done in the Arduino IDE board manager.
 * It isdeprecated for the latest board release. 
*/

/// For ESP32 Dev board (only tested with ILI9341 display)
//// The hardware SPI can be mapped to any pins
////esp32 cam ai thinkier
//// set these in TFT_eSPI libary, User_setup.h
//#define TFT_MISO 13 //yellow or gray
//#define TFT_MOSI 12 //blue
//#define TFT_SCLK 14 //green
//#define TFT_CS   15  // Chip select control pin purple
//#define TFT_DC    2  // Data Command control pin brown
//#define TFT_RST   16  // Reset pin (could connect to RST pin) white

#include <ESPAsyncWebServer.h>
#include <Arduino.h>
#include "soc/soc.h" // Disable brownout problems
#include "soc/rtc_cntl_reg.h" // Disable brownout problems
#include <bird_classification_3_inferencing.h>
#include "esp_camera.h"
#include "camera_index.h"
#include "ssid_stuff.h"
#include "SPIFFS.h"
#include <fb_gfx.h>




AsyncWebServer webserver(80);
AsyncWebSocket ws("/ws");


//const char* ssid = "NSA";
//const char* password = "orange";

//#include "esp_http_server.h"
#include "img_converters.h"
#include "image_util.h"
//#include "esp_camera.h"

#include <SPI.h>
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();
#include <TFT_eFEX.h>
TFT_eFEX  fex = TFT_eFEX(&tft);

const int push_button = 4;
bool is_start = true;
bool stream_or_display = true;

unsigned long drawTime = 0;
#define TEXT "starting app..." // Text that will be printed on screen in any font

#include "Free_Fonts.h" // Include the header file attached to this sketch
#include "helper.h"

aFrameBuffer OSD(320, 240);
//AsyncWebServer webserver(80);
//AsyncWebSocket ws("/ws");

dl_matrix3du_t *resized_matrix = NULL;
size_t out_len = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
ei_impulse_result_t result = {0};



int tf = 1;



String filelist;
camera_fb_t * fb = NULL;
String incoming;
long current_millis;
long last_capture_millis = 0;
static esp_err_t cam_err;
static esp_err_t card_err;
char strftime_buf[64];
long file_number = 0;

// CAMERA_MODEL_AI_THINKER
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


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);//disable brownout
  // Set all chip selects high to avoid bus contention during initialisation of each
  digitalWrite( TFT_CS, HIGH); // SD card chips select, must use GPIO 5 (ESP32 SS)
  Serial.begin(115200);
  
  
  pinMode(push_button, INPUT);;// initialize io4 as an output for LED flash.
  digitalWrite(4, LOW); // flash off/

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialisation failed!");
    ESP.restart();
    SPIFFS.begin(true);// Formats SPIFFS - could lose data https://github.com/espressif/arduino-esp32/issues/638
  }
  Serial.println("\r\nInitialisation done.");

  
  if(CheckPossibility()) init_wifi();
  
  SPI.begin(TFT_SCLK,TFT_MISO,TFT_MOSI,TFT_CS);
  tft.begin();
  tft.setRotation(3);  // 0 & 2 Portrait. 1 & 3 landscape
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(35,55);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
//  tft.println(WiFi.localIP());
  delay(1000);

  camera_config_t config;
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format =  PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  cam_err = esp_camera_init(&config);
  if (cam_err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", cam_err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_vflip(s, 1);

  digitalWrite(4, LOW);



tft.setTextDatum(MC_DATUM);

  // Set text colour to orange with black background
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  tft.fillScreen(TFT_BLACK);            // Clear screen
  tft.setFreeFont(FSS12);                 // Select the font
  tft.drawString(sFF1, 160, 60, GFXFF);// Print the string name of the font
  tft.setFreeFont(FF1);                 // Select the font
  tft.drawString(TEXT, 160, 120, GFXFF);// Print the string name of the font
//  tft.drawString(String(WiFi.localIP()).c_str(), 160, 180, GFXFF);
  tft.setCursor(50, 180, 2);
  tft.println(WiFi.localIP());
  
  tft.setTextColor(TFT_GREEN, TFT_BLACK);    tft.setTextFont(4);
  
  delay(1000);
  delay(1000);
  digitalWrite(4, LOW);

  ws.onEvent(onEvent);
  webserver.addHandler(&ws);

  webserver.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.print("Sending interface...");
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_ov2640_html_gz, sizeof(index_ov2640_html_gz));
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  webserver.on("/image", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("requesting image from SPIFFS");
    if (request->hasParam("id")) {
      AsyncWebParameter* p = request->getParam("id");
      Serial.printf("GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
      String imagefile = p->value();
      imagefile = imagefile.substring(4); // remove img_
      request->send(SPIFFS, "/" + imagefile);
    }
  });
  webserver.serveStatic("/", SPIFFS, "/");

  webserver.begin();

  
  fex.listSPIFFS(); // Lists the files so you can see what is in the SPIFFS
  
//  File file = SPIFFS.open("/test.txt", "w");
//  
//  file.print(0);
//  file.close();
//   SPIFFS.remove("/selfie_f_0.jpg");
//   SPIFFS.remove("/selfie_f_0.jpg");
   
}

int raw_feature_get_data(size_t offset, size_t out_len, float *signal_ptr)
{
  size_t pixel_ix = offset * 3;
  size_t bytes_left = out_len;
  size_t out_ptr_ix = 0;

  // read byte for byte
  while (bytes_left != 0) {
    // grab the values and convert to r/g/b
    uint8_t r, g, b;
    r = resized_matrix->item[pixel_ix];
    g = resized_matrix->item[pixel_ix + 1];
    b = resized_matrix->item[pixel_ix + 2];

    // then convert to out_ptr format
    float pixel_f = (r << 16) + (g << 8) + b;
    signal_ptr[out_ptr_ix] = pixel_f;

    // and go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    bytes_left--;
  }
  return 0;
}

int readInt(fs::FS &fs, const char * path) {
  Serial.printf("Reading from file: %s\r\n", path);
  File file = fs.open(path);
  if (!file) {
    Serial.println("readInt: - failed to open file for reading");
    return (0);
  }
  char iBuffer[10]; //integer range is -32768 ie 6 chars
  int x = file.readBytesUntil('\n', iBuffer, sizeof(iBuffer) - 1);
  iBuffer[x] = 0; //ensure char array is null terminated for atoi
  Serial.print(": rData is ");
  int rData = atoi (iBuffer);
  Serial.println(rData);
  //file.close();
  return (rData);
}

void classify()
{
  Serial.println("Getting signal...");
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_WIDTH;
  signal.get_data = &raw_feature_get_data;

  Serial.println("Run classifier...");
  // Feed signal to the classifier
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false /* debug */);

  // Returned error variable "res" while data object.array in "result"
  ei_printf("run_classifier returned: %d\n", res);
  if (res != 0)
    return;

  // print the predictions
  tft.setCursor(0, 0, 2);
  tft.println("results");
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: \t%f\r\n", result.classification[ix].label, result.classification[ix].value);
    tft.print(result.classification[ix].label); tft.print(": "); tft.println(result.classification[ix].value);
//    OSD.print(result.classification[ix].label); OSD.print(": "); OSD.println(result.classification[ix].value);
//    tft.println(" %s: \t%f\r\n", result.classification[ix].label, result.classification[ix].value);
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %f\r\n", result.anomaly);
#endif
}

void classify_obj()
{
  Serial.println("Getting signal...");
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_WIDTH;
  signal.get_data = &raw_feature_get_data;

  Serial.println("Run classifier...");
  // Feed signal to the classifier
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false /* debug */);

  // Returned error variable "res" while data object.array in "result"
  ei_printf("run_classifier returned: %d\n", res);
  if (res != 0)
    return;

  // print the predictions
//  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
//            result.timing.dsp, result.timing.classification, result.timing.anomaly);
//  for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
//    ei_printf("    %s: \t%f\t%d\t%d\r\n", result.bounding_boxes[ix].label, result.bounding_boxes[ix].value, result.bounding_boxes[ix].x, result.bounding_boxes[ix].y);
//  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %f\r\n", result.anomaly);
#endif
}

bool CheckPossibility(){
 int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      if(WiFi.SSID(i) == ssid){ //enter the ssid which you want to search
      Serial.println("The network you are looking for is available");
      return true;
      }
    }
  }
  return false;
}

bool init_wifi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  
  delay(1000);
  return true;
}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len)
{
  // String incoming = String((char *)data); No idea why.. gave extra characters in data for short names.
  // so ....
  for (size_t i = 0; i < len; i++) {
    incoming += (char)(data[i]);
  }
  Serial.println(incoming);

  if (incoming.substring(0, 7) == "delete:") {
    String deletefile = incoming.substring(7);
    incoming = "";
    int fromUnderscore = deletefile.lastIndexOf('_') + 1;
    int untilDot = deletefile.lastIndexOf('.');
    String fileId = deletefile.substring(fromUnderscore, untilDot);
    Serial.println(fileId);
    Serial.println("image delete");
    SPIFFS.remove("/selfie_t_" + fileId + ".jpg");
    SPIFFS.remove("/selfie_f_" + fileId + ".jpg");
    client->text("removed:" + deletefile); // file deleted. request browser update
    
  } else {
    Serial.println("sending list");
    client->text(filelist_spiffs());
  }
}

String filelist_spiffs()
{

  filelist = "";
  fs::File root = SPIFFS.open("/");

  fs::File file = root.openNextFile();
  while (file) {
    String fileName = file.name();
    // Serial.println(fileName);
    filelist = filelist + fileName;
    file = root.openNextFile();
  }
  Serial.println(filelist);
  return filelist;
}

void latestFileSPIFFS()
{
  fs::File root = SPIFFS.open("/");

  fs::File file = root.openNextFile();
  while (file) {
    String fileName = file.name();
    Serial.println(fileName);
    int fromUnderscore = fileName.lastIndexOf('_') + 1;
    int untilDot = fileName.lastIndexOf('.');
    String fileId = fileName.substring(fromUnderscore, untilDot);
    Serial.println(fileId);
    file_number = max(file_number, fileId.toInt()); // replace filenumber if fileId is higher
    file = root.openNextFile();
  }
}

//void rgb_print(fb_data_t *fb, uint32_t color, const char *str)
//{
//    fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
//}

void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str, int y){
               fb_data_t fb;
               fb.width = image_matrix->w;
               fb.height = image_matrix->h;
               fb.data = image_matrix->item;
               fb.bytes_per_pixel = 3;
               fb.format = FB_BGR888;
//               fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, y, color, str);
               fb_gfx_print(&fb, (2+ (strlen(str) * 4)) / 2, y, color, str);
}



void loop()
{
  
  int push_button_state  = digitalRead(push_button);

  if(is_start) {
    push_button_state = true;
    is_start = false;
  }
  
//  Serial.print("push button state = ");
//  Serial.println(push_button_state);
//  Serial.print("stream or display = ");
//  Serial.println(stream_or_display);
//  delay(1000);

if(stream_or_display) {  //take photo and output to tft.
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
//    httpd_resp_send_500(req);
//    return ESP_FAIL;
  }
  fex.drawJpg((const uint8_t*)fb->buf, fb->len, 0, 6);
  esp_camera_fb_return(fb);
  }

if(push_button_state){
  delay(200);
  stream_or_display = !stream_or_display;
  Serial.print("push button state now stream or display = ");
//  Serial.println(stream_or_display);
  if(!stream_or_display){
  //take phot and stream.
  Serial.println("Capture image");
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
//    httpd_resp_send_500(req);
//    return ESP_FAIL;
  }
  fex.drawJpg((const uint8_t*)fb->buf, fb->len, 0, 6);
  Serial.println("Converting to RGB888...");
  // Allocate rgb888_matrix buffer
  //do some file stuff here
  file_number =  readInt(SPIFFS, "/test.txt");
  if (file_number<70) file_number = 71;
  File file2 = SPIFFS.open("/test.txt", "r");
  Serial.println(file_number);
  //if(file_number > 100) file_number = 0;
  
  char *full_filename = (char*)malloc(23 + sizeof(file_number));
  sprintf(full_filename, "/spiffs/selfie_f_%d.jpg", file_number);

  FILE *fullres = fopen(full_filename, "w");
   Serial.println(sizeof((fb->buf)[1]));
//   tft.readRectRGB(0, 0, 320*240, 1, fb->buf);
  if (fullres != NULL)  {
    size_t err = fwrite(fb->buf, 1, fb->len, fullres);
    Serial.printf("File saved: %s\n", full_filename);
  }  else  {
    Serial.println("Could not open file"); 
  }
  fclose(fullres);
  
  dl_matrix3du_t *rgb888_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  fmt2rgb888(fb->buf, fb->len, fb->format, rgb888_matrix->item);

  Serial.println("Resizing the frame buffer...");
  resized_matrix = dl_matrix3du_alloc(1, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3);
  image_resize_linear(resized_matrix->item, rgb888_matrix->item, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3, fb->width, fb->height);

  
  // --- Free memory ---

  dl_matrix3du_free(rgb888_matrix);

  
//  esp_camera_fb_return(fb);
  
  

  classify();

  dl_matrix3du_free(resized_matrix);
  
  //print to image
  rgb888_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  fmt2rgb888(fb->buf, fb->len, fb->format, rgb888_matrix->item);

  // HERE print some text
//  rgb_print(rgb888_matrix, 0x000000FF, "Hello!");
    /////

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        
        char tmp[32];
        sprintf(tmp, "%s: %0.2f", result.classification[ix].label, result.classification[ix].value);
        rgb_print(rgb888_matrix, 0x000000FF, tmp, ix*15 + 2);
      }

  Serial.println("printing to image");
  size_t _jpg_buf_len = 0;
uint8_t * _jpg_buf = NULL;

  
//  free(fb->buf);
//  fb->buf = NULL;
  bool jpeg_converted = fmt2jpg(rgb888_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 80, &_jpg_buf, &_jpg_buf_len); //&fb->buf
  Serial.println("converted jpg");
  dl_matrix3du_free(rgb888_matrix);
  
  //more file stuff
//  file_number++;
  full_filename = (char*)malloc(23 + sizeof(file_number));
  sprintf(full_filename, "/spiffs/selfie_t_%d.jpg", file_number);
  fullres = fopen(full_filename, "w");
//   Serial.println(sizeof((fb->buf)[1]));
//   tft.readRectRGB(0, 0, 320*240, 1, fb->buf);
  if (fullres != NULL)  {
    size_t err = fwrite(_jpg_buf, 1, _jpg_buf_len, fullres);
    free(_jpg_buf);
    Serial.printf("File saved: %s\n", full_filename);
  }  else  {
    Serial.println("Could not open file"); 
  }
  fclose(fullres);
  esp_camera_fb_return(fb);
  fb = NULL;
  free(full_filename);
//  delay(5000);
  delay(500);
  char *addtobrowser = (char*)malloc(24 + sizeof(file_number));
  sprintf(addtobrowser, "added:selfie_t_%d.jpg", file_number);
  file_number++;
  File file = SPIFFS.open("/test.txt", "w");
  file.print(file_number);
  file.close();

  //ws.textAll((char*)addtobrowser);// file added. request browser update
  free(addtobrowser);
//  ws.cleanupClients();
}

}

}

//static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str){
//               fb_data_t fb;
//               fb.width = image_matrix->w;
//               fb.height = image_matrix->h;
//               fb.data = image_matrix->item;
//               fb.bytes_per_pixel = 3;
//               fb.format = FB_BGR888;
//               fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
//}
