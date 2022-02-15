#if defined (ESP8266)
  GxEPD2_3C<GxEPD2_290_Z13c, GxEPD2_290_Z13c::HEIGHT> display(GxEPD2_290_Z13c(/*CS=D8*/ 15, /*DC=D3*/ 9, /*RST=D4*/ 5, /*BUSY=D2*/ 4)); // GDEW029Z13 v3 waveshare? vmak pins
#elif defined (ESP32)
  GxEPD2_3C<GxEPD2_290_Z13c, GxEPD2_290_Z13c::HEIGHT> display(GxEPD2_290_Z13c(/*CS=5*/ 15, /*DC=*/ 27, /*RST=*/ 26, /*BUSY=*/ 25)); // GDEH029Z13
#endif
