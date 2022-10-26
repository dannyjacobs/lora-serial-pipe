Lora Serial Pipe

Use two LORA 32u4 or similar modules to make a wireless USB tty connection
Currently configured to add a bit of link telemetry to the mix

Caveats:
 - currently only one way. Buffering is hard.
 - Currently configured to add a bit of link telemetry to the mix. Its value added!


Instructions
the code is in: lora_serial_pipe.iso
Make one LORA module the TX side: Set MODE = TX
Make other module the RX side: Set MODE=RD

Requires: RadioHead
http://www.airspayce.com/mikem/arduino/RadioHead/
Based on Arduino tutorial for LORA RFM 9X
https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/using-the-rfm-9x-radio
Tested on a LoRa 32v4 and a Arduino Feather 32v4
