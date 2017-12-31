# ZOEdisplay
An onboard realtime status display for Renault ZOE vehicles

Die von mir verwendete Hardware:

* Arduino Uno https://www.reichelt.de/?ARTICLE=119045
* CAN-BUS Shield V1.2 https://www.seeedstudio.com/CAN-BUS-Shield-V1.2-p-2256.html
* Arduino LCD KeyPad Shield (SKU: DFR0009) https://www.dfrobot.com/wiki/index.php/Arduino_LCD_KeyPad_Shield_(SKU:_DFR0009)


CAN-Bus-Shield Konfiguration: CS->D10 (vorgesehene Lötbrücke)

Modifikation des LCD-Shields notwendig (Leiterbahn unterbrechen und bei Bedarf umlegen): Leiterbahn von im Lieferzustand Pin 10 des LCD-Shields auf Pin 3 (D3) des Arduino-Board umlegen (für PWM-Helligkeitsregelung nach Lichtsituation der LCD-Hintergrundbeleuchtung).

Option: Temperatursensor via OneWire-Bus an Pin A1 (DATA) sowie +5V und GND. Separater 4.7kΩ-Pullup-Widerstand zwischen +5V und DATA erforderlich.
