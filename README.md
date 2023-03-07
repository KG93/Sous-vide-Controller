# Sous-vide-Controller
A small arduino firmware project for a sous-vide-controller. The controller offers the following functionality:
 * (timed) PID-control of a heater relay via Arduino output pins.
 * temperature input with a MAX31865 RTD-to-Digital Converter
 * user feedback with a SSD1306 display and a buzzer
 * user input with a pushbutton and a rotational encoder for setting the temperature and simple menu navigation
 * an autotune mode, to adapt the controller to different heaters/environments

## Hardware
The hardware list:
 * an Arduino 
 * a MAX31865 RTD-to-Digital Converter
 * a SSD1306 oled display
 * a suitable solid state relay, beware of [fakes]
 * a buzzer
 * a rotary encoder with an included button

## Libraries
Library requirements:
 * [QuickPID] up to version 2.5
 * [ClickEncoder]
 * [SSD1306Ascii]
 * [Adafruit_MAX31865]
 * [PT100RTD]

[QuickPID]:https://github.com/Dlloydev/QuickPID/releases/tag/2.5.0
[ClickEncoder]:https://github.com/0xPIT/encoder
[Adafruit_MAX31865]:https://github.com/adafruit/Adafruit_MAX31865
[SSD1306Ascii]:https://github.com/greiman/SSD1306Ascii
[PT100RTD]:https://github.com/drhaney/pt100rtd

[fakes]:https://protosupplies.com/inferior-counterfeit-fotek-ssr-25-solid-state-relays-on-the-market/
