WiFiClient client;

#if defined (ESP8266)
  HTTPClient http;
#endif

  //WiFiClientSecure client;
  //client.setInsecure();

void wifiSetup(void) {
  //Sets up WiFi connection
  //IPAddress myIP; //may have to declare in global scope
  Serial.print("\n\nConnecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  // Waiting the connection to a router
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  //Print IP
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
}

//String httpGETRequest(const char* serverName){
//  //performs get request from URL (serverName)
//  http.begin(client, serverName);
//
//  int httpResponseCode = http.GET();
//  String payload = "{}";
//
//  if (httpResponseCode>0) {
//    Serial.print("HTTP Response code: ");
//    Serial.println(httpResponseCode);
//    payload = http.getString();
//  }
//  else {
//    Serial.print("Error code: ");
//    Serial.println(httpResponseCode);
//  }
//  // Free resources
//  http.end();
//  return payload;
//}
//
//String httpGETRequestString(String serverName){
//  //performs get request from URL (serverName)
//  http.begin(client, serverName);
//
//  int httpResponseCode = http.GET();
//  String payload = "{}";
//
//  if (httpResponseCode>0) {
//    Serial.print("HTTP Response code: ");
//    Serial.println(httpResponseCode);
//    payload = http.getString();
//  }
//  else {
//    Serial.print("Error code: ");
//    Serial.println(httpResponseCode);
//  }
//  // Free resources
//  http.end();
//  return payload;
//}
