# ZOEdisplay
An onboard realtime status display for Renault ZOE vehicles

Die von mir verwendete Hardware:

* Arduino Uno https://www.reichelt.de/?ARTICLE=119045
* CAN-BUS Shield V1.2 https://www.seeedstudio.com/CAN-BUS-Shield-V1.2-p-2256.html
* Arduino LCD KeyPad Shield (SKU: DFR0009) https://www.dfrobot.com/wiki/index.php/Arduino_LCD_KeyPad_Shield_(SKU:_DFR0009) und LCD Keypad Shield V2.0 (SKU: DFR0374) https://www.dfrobot.com/wiki/index.php/LCD_Keypad_Shield_V2.0_SKU:_DFR0374


CAN-Bus-Shield Konfiguration: CS->D10 (vorgesehene Lötbrücke)
Achtung: Die winzige Standard-Leiterbahnverbindung zwischend den Lötpads für CS->D9 muss dazu natürlich entfernt werden! Nachmessen!

Modifikation des LCD-Shields notwendig (Leiterbahn unterbrechen oder Pin umbiegen und bei Bedarf umlegen): Leiterbahn von im Lieferzustand Pin 10 des LCD-Shields auf Pin 3 (D3) des Arduino-Board umlegen (für PWM-Helligkeitsregelung nach Lichtsituation der LCD-Hintergrundbeleuchtung).

Option: Temperatursensor via OneWire-Bus an Pin A1 (DATA) sowie +5V und GND. Separater 4.7kΩ-Pullup-Widerstand zwischen +5V und DATA erforderlich.


Benötigte Softwarebibliotheken:
https://github.com/premultiply/MCP_CAN_lib
https://github.com/premultiply/LcdBarGraph
sowie https://github.com/rlogiacco/AnalogButtons, https://github.com/Chris--A/PrintEx, https://github.com/RobTillaart/Arduino/tree/master/libraries/StopWatch, http://playground.arduino.cc/Code/Timer1


Einige (inzwischen veraltete) Fotos im Betrieb:

![](https://cdn.goingelectric.de/forum/resources/image/thumb/44462)


Pin-Verwendung:  
A0    Display Shield   Buttons (select, up, right, down and left)  
D3    Display Shield   Backlit Control  
D4 	  Display Shield   DB4  
D5   	Display Shield   DB5  
D6   	Display Shield   DB6  
D7   	Display Shield   DB7  
D8   	Display Shield   RS (Data or Signal Display Selection)  
D9 	  Display Shield   Enable  
D10   CAN-Bus Shield   CS
