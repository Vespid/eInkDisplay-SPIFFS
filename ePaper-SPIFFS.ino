#define ENABLE_GxEPD2_GFX 0
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>

#if defined(ESP32)
  #include "SPIFFS.h"
#endif

#include <FS.h>
#define FileClass fs::File

#include <EEPROM.h>
#include <ArduinoJson.h>

#if defined (ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
#endif

#include <WiFiClient.h>
#include <WiFiClientSecure.h>

#include "SPIFFS-helper.h"
#include "GPIO.h"
#include "credentials.h"
#include "server.h"

const int httpPort  = 80;
const int httpsPort = 443;
const char* fp_api_github_com = "df b2 29 c6 a6 38 1a 59 9d c9 ad 92 2d 26 f5 3c 83 8f a5 87"; // as of 25.11.2020
const char* fp_github_com     = "5f 3f 7a c2 56 9f 50 a4 66 76 47 c6 a1 8c a0 07 aa ed bb 8e"; // as of 25.11.2020
const char* fp_rawcontent     = "70 94 de dd e6 c4 69 48 3a 92 70 a1 48 56 78 2d 18 64 e0 b7"; // as of 25.11.2020
const char* certificate_rawcontent =
  "-----BEGIN CERTIFICATE-----\n"
  "MIIEsTCCA5mgAwIBAgIQBOHnpNxc8vNtwCtCuF0VnzANBgkqhkiG9w0BAQsFADBsMQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGln"
  "aUNlcnQgSW5jMRkwFwYDVQQLExB3d3cuZGlnaWNlcnQuY29tMSswKQYDVQQDEyJEaWdpQ2VydCBIaWdoIEFzc3VyYW5jZSBFViBS"
  "b290IENBMB4XDTEzMTAyMjEyMDAwMFoXDTI4MTAyMjEyMDAwMFowcDELMAkGA1UEBhMCVVMxFTATBgNVBAoTDERpZ2lDZXJ0IElu"
  "YzEZMBcGA1UECxMQd3d3LmRpZ2ljZXJ0LmNvbTEvMC0GA1UEAxMmRGlnaUNlcnQgU0hBMiBIaWdoIEFzc3VyYW5jZSBTZXJ2ZXIg"
  "Q0EwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQC24C/CJAbIbQRf1+8KZAayfSImZRauQkCbztyfn3YHPsMwVYcZuU+U"
  "DlqUH1VWtMICKq/QmO4LQNfE0DtyyBSe75CxEamu0si4QzrZCwvV1ZX1QK/IHe1NnF9Xt4ZQaJn1itrSxwUfqJfJ3KSxgoQtxq2l"
  "nMcZgqaFD15EWCo3j/018QsIJzJa9buLnqS9UdAn4t07QjOjBSjEuyjMmqwrIw14xnvmXnG3Sj4I+4G3FhahnSMSTeXXkgisdaSc"
  "us0Xsh5ENWV/UyU50RwKmmMbGZJ0aAo3wsJSSMs5WqK24V3B3aAguCGikyZvFEohQcftbZvySC/zA/WiaJJTL17jAgMBAAGjggFJ"
  "MIIBRTASBgNVHRMBAf8ECDAGAQH/AgEAMA4GA1UdDwEB/wQEAwIBhjAdBgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwNAYI"
  "KwYBBQUHAQEEKDAmMCQGCCsGAQUFBzABhhhodHRwOi8vb2NzcC5kaWdpY2VydC5jb20wSwYDVR0fBEQwQjBAoD6gPIY6aHR0cDov"
  "L2NybDQuZGlnaWNlcnQuY29tL0RpZ2lDZXJ0SGlnaEFzc3VyYW5jZUVWUm9vdENBLmNybDA9BgNVHSAENjA0MDIGBFUdIAAwKjAo"
  "BggrBgEFBQcCARYcaHR0cHM6Ly93d3cuZGlnaWNlcnQuY29tL0NQUzAdBgNVHQ4EFgQUUWj/kK8CB3U8zNllZGKiErhZcjswHwYD"
  "VR0jBBgwFoAUsT7DaQP4v0cB1JgmGggC72NkK8MwDQYJKoZIhvcNAQELBQADggEBABiKlYkD5m3fXPwdaOpKj4PWUS+Na0QWnqxj"
  "9dJubISZi6qBcYRb7TROsLd5kinMLYBq8I4g4Xmk/gNHE+r1hspZcX30BJZr01lYPf7TMSVcGDiEo+afgv2MW5gxTs14nhr9hctJ"
  "qvIni5ly/D6q1UEL2tU2ob8cbkdJf17ZSHwD2f2LSaCYJkJA69aSEaRkCldUxPUd1gJea6zuxICaEnL6VpPX/78whQYwvwt/Tv9X"
  "BZ0k7YXDK/umdaisLRbvfXknsuvCnQsH6qqF0wGjIChBWUMo0oHjqvbsezt3tkBigAVBRQHvFwY+3sAzm2fTYS5yh+Rp/BIAV0AecPUeybQ=\n"
  "-----END CERTIFICATE-----\n";

const char* host_rawcontent   = "raw.githubusercontent.com";
const char* path_rawcontent   = "/ZinggJM/GxEPD2/master/extras/bitmaps/";
const char* path_prenticedavid   = "/prenticedavid/MCUFRIEND_kbv/master/extras/bitmaps/";
const char* path_waveshare_c  = "/waveshare/e-Paper/master/RaspberryPi_JetsonNano/c/pic/";
const char* path_waveshare_py = "/waveshare/e-Paper/master/RaspberryPi_JetsonNano/python/pic/";

  
//Function Prototypes
void drawBitmapFromSpiffs(const char *filename, int16_t x, int16_t y, bool with_color = true);
void drawBitmapFromSpiffs_Buffered(const char *filename, int16_t x, int16_t y, bool with_color = true, bool partial_update = false, bool overwrite = false);
void downloadFile_HTTPS(const char* host, const char* path, const char* filename, const char* fingerprint, const char* target, const char* certificate = certificate_rawcontent);



void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("GxEPD2_Spiffs_Example");

  display.init(115200);
  #if defined(ESP32)
    // *** special handling for Waveshare ESP32 Driver board *** //
    // ********************************************************* //
    SPI.end(); // release standard SPI pins, e.g. SCK(18), MISO(19), MOSI(23), SS(5)
    //SPI: void begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);
    SPI.begin(13, 12, 14, 15); // map and init SPI pins SCK(13), MISO(12), MOSI(14), SS(15)
    // *** end of special handling for Waveshare ESP32 Driver board *** //
    // **************************************************************** //
  #endif

  SPIFFS.begin();
  Serial.println("SPIFFS started");
  
  listFiles();
  wifiSetup();
  //downloadFile_HTTPS(host_rawcontent, path_prenticedavid, "betty_4.bmp", fp_rawcontent, "betty_4.bmp");
  downloadJSON();
//  listFiles();
  drawBitmapFromSpiffs("Angels.bmp", 0, 0);
  delay(5000);
  //ESP.deepSleep(image1["update"]);
  drawBitmapFromSpiffs("Bud.bmp", 0, 0);
  delay(5000);
  drawBitmapFromSpiffs("Chiefs.bmp", 0, 0);
  delay(5000);
  drawBitmapFromSpiffs("Coke.bmp", 0, 0);
  delay(5000);
  drawBitmapFromSpiffs("DrPepper.bmp", 0, 0);
  delay(5000);
  drawBitmapFromSpiffs("Indians.bmp", 0, 0);
  delay(5000);
  drawBitmapFromSpiffs("Reds.bmp", 0, 0);
//  delay(5000);
//  drawBitmapFromSpiffs("/test.bmp", 0, 0);
  //ESP.deepSleep(60e6); //Deep sleep for 60 seconds

  /*
  EEPROM.update(1, image1["hash"]);
  if (EEPROM.update(1, image1["hash"]){
    //download new image file
  }

   */

  
  }

void loop() {
  // put your main code here, to run repeatedly:

}

void downloadJSON(){
  #if USE_BearSSL
    BearSSL::WiFiClientSecure client;
  #else
    WiFiClientSecure client;
  #endif
  const char* host = "raw.githubusercontent.com";
  const char* filename = "/Vespid/eInkDisplay/master/update.json";
  
  //client.setFingerprint(fp_rawcontent);
  #if defined (ESP8266)
    client.setFingerprint(fp_rawcontent);
  #elif defined (ESP32)
    client.setCACert(certificate_rawcontent);
  #endif

  if (!client.connect(host, httpsPort))
  {
    Serial.println("connection failed");
    return;
  }
  client.print(String("GET ") + filename + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: GxEPD2_Spiffs_Loader\r\n" +
               "Connection: close\r\n\r\n");
  bool ok = false;

//from arduinojson
  char status[32] = {0};
  client.readBytesUntil('\r', status, sizeof(status));
  if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
    Serial.print(F("Unexpected response: "));
    Serial.println(status);
    client.stop();
    return;
  }

  char endOfHeaders[] = "\r\n\r\n";
  if (!client.find(endOfHeaders)) {
    Serial.println(F("Invalid response"));
    client.stop();
    return;
  }
  
  StaticJsonDocument<200> doc;
  deserializeJson(doc, client);
  long d1 = doc["delay"][0];
  long li[4] = {};
  Serial.println("Delay: ");
  for (int i=0; i<4; i++){
    li[i] = doc["delay"][i];
    Serial.println(li[i]);
  }
  String s1 = doc["image1"]["hash"];
  String i1 = doc["image1"]["address"];

  Serial.println("API test...");
  Serial.print("Delay 0:");
  Serial.println(d1,3);
  Serial.print("image1 hash:");
  Serial.println(s1);
  Serial.print("image1 address:");
  Serial.println(i1);
}



static const uint16_t input_buffer_pixels = 800; // may affect performance

static const uint16_t max_row_width = 1448; // for up to 6" display 1448x1072
static const uint16_t max_palette_pixels = 256; // for depth <= 8

uint8_t input_buffer[3 * input_buffer_pixels]; // up to depth 24
uint8_t output_row_mono_buffer[max_row_width / 8]; // buffer for at least one row of b/w bits
uint8_t output_row_color_buffer[max_row_width / 8]; // buffer for at least one row of color bits
uint8_t mono_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 b/w
uint8_t color_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 c/w
uint16_t rgb_palette_buffer[max_palette_pixels]; // palette buffer for depth <= 8 for buffered graphics, needed for 7-color display

void drawBitmapFromSpiffs(const char *filename, int16_t x, int16_t y, bool with_color)
{
  fs::File file;
  bool valid = false; // valid format to be handled
  bool flip = true; // bitmap is stored bottom-to-top
  uint32_t startTime = millis();
  if ((x >= display.epd2.WIDTH) || (y >= display.epd2.HEIGHT)) return;
  Serial.println();
  Serial.print("Loading image '");
  Serial.print(filename);
  Serial.println('\'');
#if defined(ESP32)
  file = SPIFFS.open(String("/") + filename, "r");
#else
  file = SPIFFS.open(filename, "r");
#endif
  if (!file)
  {
    Serial.print("File not found");
    return;
  }
  // Parse BMP header
  if (read16(file) == 0x4D42) // BMP signature
  {
    uint32_t fileSize = read32(file);
    uint32_t creatorBytes = read32(file);
    uint32_t imageOffset = read32(file); // Start of image data
    uint32_t headerSize = read32(file);
    uint32_t width  = read32(file);
    uint32_t height = read32(file);
    uint16_t planes = read16(file);
    uint16_t depth = read16(file); // bits per pixel
    uint32_t format = read32(file);
    if ((planes == 1) && ((format == 0) || (format == 3))) // uncompressed is handled, 565 also
    {
      Serial.print("File size: "); Serial.println(fileSize);
      Serial.print("Image Offset: "); Serial.println(imageOffset);
      Serial.print("Header size: "); Serial.println(headerSize);
      Serial.print("Bit Depth: "); Serial.println(depth);
      Serial.print("Image size: ");
      Serial.print(width);
      Serial.print('x');
      Serial.println(height);
      // BMP rows are padded (if needed) to 4-byte boundary
      uint32_t rowSize = (width * depth / 8 + 3) & ~3;
      if (depth < 8) rowSize = ((width * depth + 8 - depth) / 8 + 3) & ~3;
      if (height < 0)
      {
        height = -height;
        flip = false;
      }
      uint16_t w = width;
      uint16_t h = height;
      if ((x + w - 1) >= display.epd2.WIDTH)  w = display.epd2.WIDTH  - x;
      if ((y + h - 1) >= display.epd2.HEIGHT) h = display.epd2.HEIGHT - y;
      if (w <= max_row_width) // handle with direct drawing
      {
        valid = true;
        uint8_t bitmask = 0xFF;
        uint8_t bitshift = 8 - depth;
        uint16_t red, green, blue;
        bool whitish, colored;
        if (depth == 1) with_color = false;
        if (depth <= 8)
        {
          if (depth < 8) bitmask >>= depth;
          //file.seek(54); //palette is always @ 54
          file.seek(imageOffset - (4 << depth)); // 54 for regular, diff for colorsimportant
          for (uint16_t pn = 0; pn < (1 << depth); pn++)
          {
            blue  = file.read();
            green = file.read();
            red   = file.read();
            file.read();
            whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
            colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
            if (0 == pn % 8) mono_palette_buffer[pn / 8] = 0;
            mono_palette_buffer[pn / 8] |= whitish << pn % 8;
            if (0 == pn % 8) color_palette_buffer[pn / 8] = 0;
            color_palette_buffer[pn / 8] |= colored << pn % 8;
          }
        }
        display.clearScreen(); //causes blank screen whiler loading
        uint32_t rowPosition = flip ? imageOffset + (height - h) * rowSize : imageOffset;
        for (uint16_t row = 0; row < h; row++, rowPosition += rowSize) // for each line
        {
          uint32_t in_remain = rowSize;
          uint32_t in_idx = 0;
          uint32_t in_bytes = 0;
          uint8_t in_byte = 0; // for depth <= 8
          uint8_t in_bits = 0; // for depth <= 8
          uint8_t out_byte = 0xFF; // white (for w%8!=0 border)
          uint8_t out_color_byte = 0xFF; // white (for w%8!=0 border)
          uint32_t out_idx = 0;
          file.seek(rowPosition);
          for (uint16_t col = 0; col < w; col++) // for each pixel
          {
            // Time to read more pixel data?
            if (in_idx >= in_bytes) // ok, exact match for 24bit also (size IS multiple of 3)
            {
              in_bytes = file.read(input_buffer, in_remain > sizeof(input_buffer) ? sizeof(input_buffer) : in_remain);
              in_remain -= in_bytes;
              in_idx = 0;
            }
            switch (depth)
            {
              case 24:
                blue = input_buffer[in_idx++];
                green = input_buffer[in_idx++];
                red = input_buffer[in_idx++];
                whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
                break;
              case 16:
                {
                  uint8_t lsb = input_buffer[in_idx++];
                  uint8_t msb = input_buffer[in_idx++];
                  if (format == 0) // 555
                  {
                    blue  = (lsb & 0x1F) << 3;
                    green = ((msb & 0x03) << 6) | ((lsb & 0xE0) >> 2);
                    red   = (msb & 0x7C) << 1;
                  }
                  else // 565
                  {
                    blue  = (lsb & 0x1F) << 3;
                    green = ((msb & 0x07) << 5) | ((lsb & 0xE0) >> 3);
                    red   = (msb & 0xF8);
                  }
                  whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                  colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
                }
                break;
              case 1:
              case 4:
              case 8:
                {
                  if (0 == in_bits)
                  {
                    in_byte = input_buffer[in_idx++];
                    in_bits = 8;
                  }
                  uint16_t pn = (in_byte >> bitshift) & bitmask;
                  whitish = mono_palette_buffer[pn / 8] & (0x1 << pn % 8);
                  colored = color_palette_buffer[pn / 8] & (0x1 << pn % 8);
                  in_byte <<= depth;
                  in_bits -= depth;
                }
                break;
            }
            if (whitish)
            {
              // keep white
            }
            else if (colored && with_color)
            {
              out_color_byte &= ~(0x80 >> col % 8); // colored
            }
            else
            {
              out_byte &= ~(0x80 >> col % 8); // black
            }
            if ((7 == col % 8) || (col == w - 1)) // write that last byte! (for w%8!=0 border)
            {
              output_row_color_buffer[out_idx] = out_color_byte;
              output_row_mono_buffer[out_idx++] = out_byte;
              out_byte = 0xFF; // white (for w%8!=0 border)
              out_color_byte = 0xFF; // white (for w%8!=0 border)
            }
          } // end pixel
          uint16_t yrow = y + (flip ? h - row - 1 : row);
          display.writeImage(output_row_mono_buffer, output_row_color_buffer, x, yrow, w, 1);
        } // end line
        Serial.print("loaded in "); Serial.print(millis() - startTime); Serial.println(" ms");
        display.refresh();
      }
    }
  }
  file.close();
  if (!valid)
  {
    Serial.println("bitmap format not handled.");
  }
}

uint16_t read16(fs::File& f)
{
  // BMP data is stored little-endian, same as Arduino.
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(fs::File& f)
{
  // BMP data is stored little-endian, same as Arduino.
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}


//FROM: GxEPD2_SPIFFS_LOADER.ino
void downloadFile_HTTP(const char* host, const char* path, const char* filename, const char* target)
{
  WiFiClient client;
  Serial.print("connecting to ");
  Serial.println(host);
  if (!client.connect(host, httpPort))
  {
    Serial.println("connection failed");
    return;
  }
  Serial.print("requesting URL: ");
  Serial.println(String("http://") + host + path + filename);
  client.print(String("GET ") + path + filename + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: GxEPD2_Spiffs_Loader\r\n" +
               "Connection: close\r\n\r\n");
  Serial.println("request sent");
  bool ok = false;
  while (client.connected() || client.available())
  {
    String line = client.readStringUntil('\n');
    if (!ok)
    {
      ok = line.startsWith("HTTP/1.1 200 OK");
      if (ok) Serial.println(line);
      //if (!ok) Serial.println(line);
    }
    if (!ok) Serial.println(line);
    //Serial.println(line);
    if (line == "\r")
    {
      Serial.println("headers received");
      break;
    }
  }
  if (!ok) return;
  uint8_t buffer[512];
#if defined(ESP8266)
  client.peekBytes(buffer, 2);
  Serial.write(buffer[0]); Serial.write(buffer[1]); Serial.println();
#endif
  size_t total = 0;
#if defined(ESP32)
  fs::File file = SPIFFS.open(String("/") + target, "w+");
#else
  fs::File file = SPIFFS.open(target, "w+");
#endif
  if (!file)
  {
    Serial.print(target); Serial.println(" open failed");
    return;
  }
  while (client.connected() || client.available())
  {
    size_t available = client.available();
    size_t fetch = available <= sizeof(buffer) ? available : sizeof(buffer);
    if (fetch > 0)
    {
      size_t got = client.read(buffer, fetch);
      file.write(buffer, got);
      total += got;
    }
    delay(1); // yield();
  }
  file.close();
  Serial.print("done, "); Serial.print(total); Serial.println(" bytes transferred");
}

void downloadFile_HTTPS(const char* host, const char* path, const char* filename, const char* fingerprint, const char* target, const char* certificate)
{
  // Use WiFiClientSecure class to create TLS connection
#if USE_BearSSL
  BearSSL::WiFiClientSecure client;
#else
  WiFiClientSecure client;
#endif
  Serial.println(); Serial.print("downloading file \""); Serial.print(filename);  Serial.println("\"");
  Serial.print("connecting to "); Serial.println(host);
#if defined (ESP8266)
  if (fingerprint) client.setFingerprint(fingerprint);
#elif defined (ESP32)
  if (certificate) client.setCACert(certificate);
#endif
  if (!client.connect(host, httpsPort))
  {
    Serial.println("connection failed");
    return;
  }
  Serial.print("requesting URL: ");
  Serial.println(String("https://") + host + path + filename);
  client.print(String("GET ") + path + filename + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: GxEPD2_Spiffs_Loader\r\n" +
               "Connection: close\r\n\r\n");
  Serial.println("request sent");
  bool ok = false;
  while (client.connected() || client.available())
  {
    String line = client.readStringUntil('\n');
    if (!ok)
    {
      ok = line.startsWith("HTTP/1.1 200 OK");
      if (ok) Serial.println(line);
      //if (!ok) Serial.println(line);
    }
    if (!ok) Serial.println(line);
    //Serial.println(line);
    if (line == "\r")
    {
      Serial.println("headers received");
      break;
    }
  }
  if (!ok) return;
  uint8_t buffer[512];
#if defined(ESP8266)
  client.peekBytes(buffer, 2);
  Serial.write(buffer[0]); Serial.write(buffer[1]); Serial.println();
#endif
  size_t total = 0;
#if defined(ESP32)
  fs::File file = SPIFFS.open(String("/") + target, "w+");
#else
  fs::File file = SPIFFS.open(target, "w+");
#endif
  if (!file)
  {
    Serial.print(target); Serial.println(" open failed");
    return;
  }
  while (client.connected() || client.available())
  {
    // this doesn't work as expected, but it helps for long downloads
    int32_t start = millis();
    for (int16_t t = 0, dly = 50; t < 20; t++, dly += 50)
    {
      if (!(client.connected() || client.available())) break;
      if (client.available()) break; // read would not recover after having returned 0
      delay(dly);
    }
    if (!(client.connected() || client.available())) break;
    int32_t elapsed = millis() - start;
    if (elapsed > 250)
    {
      Serial.print("waited for available "); Serial.print(millis() - start); Serial.print(" ms @ "); Serial.println(total);
    }
    size_t available = client.available();
    if (0 == available)
    {
      Serial.print("download error: timeout on available() after "); Serial.print(total); Serial.println(" bytes");
      break; // don't hang forever
    }
    size_t fetch = available <= sizeof(buffer) ? available : sizeof(buffer);
    if (fetch > 0)
    {
      size_t got = client.read(buffer, fetch);
      file.write(buffer, got);
      total += got;
    }
    delay(1); // yield();
    if (total > 30000) delay(250); // helps for long downloads
  }
  file.close();
  Serial.print("done, "); Serial.print(total); Serial.println(" bytes transferred");
}
