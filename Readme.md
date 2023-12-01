Arduino/Pro Micro sketch software that allows an nes controller to act as an HID keyboard or an XInput Xbox controller.</b>
The keybord is easier but the xbox has some benefits.  Stick with keyboard if youre unsure.</b>

For using the pro micro/arduino as a HID keyboard:</b>

Download the .ino file and upload it using the Arduino IDE.</b>

button to keyboard binds:</b></b>
nes a = keyboard x</b></b>
nes b = keyboard z</b></b>
nes select	= unbinded(select + b = keyboard escape)</b></b>
nes start 	= keyboard enter/return(keyboard r in emulator friendly startup mode)</b></b>
nes up 		= keyboard up</b></b>
nes down	= keyboard down</b></b>
nes left	= keyboard left</b></b>
nes right = keyboard right</b></b>
nes select + nes start + nes a = keyboard s (zoom tetris effect camera out)</b></b>
nes select + nes a = fz1</b></b>
nes select + nes dpad = emotes on tetris effect</b></b>

	
For using the pro micro/arduino as an Xbox controller:</b>
add to board manager</br>
https://raw.githubusercontent.com/dmadison/ArduinoXInput_Boards/master/package_dmadison_xinput_index.json</br>
Comment out the line "#define KEYBOARD" in the .ino sketch file.
note: must ground reset pin twice two times.  Ground the pin twice before hitting clicking the upload button the immediately</b> 
after the bottom right nofication says "uploading", then ground it twice again. May take several attempts</br>
Make sure to have arduino, sparkfun, teensy board variants selected appropriately</br>

nes button binds:</br>
up -> xbox up</br>
down -> xbox down</br>
left -> xbox left</br>
right -> xbox right</br>
select -> see: SELECT MODE</br>
start -> xbox start</br>
a -> xbox b</br>
b -> xbox a</br>


SELECT MODE:</br>
while select is pressed the following keybinds are replaced for emoting and moving camera in and out: </br>
up -> right joystick up</br>
down -> right joystick down</br>
left -> right joystick left</br>
right -> right joystick right</br>
start -> see: SELECT/START MODE</br>
a -> left joystick up</br>
b -> left joystick down</br>


SELECT/START MODE: </br>
while select and start are pressed the following keybinds are replaced for extra menu navigation: </br>
a -> xbox x</br>
b -> xbox y</br>


Startup Modes:</b>

PAL:</br>
Hold buttons A and B on the controller while plugging in to have PAL 50hz debouncing</br>

Emulation Friendly Binds:</b>
Hold buttons Select and Down while controller is being plugged in bind select and start buttons to q and r, respectively. </b>

Goofy Foot:</b>
Hole buttons A and B on the controller while plugging in to switch from 60hz ntsc to 50hz pal debouncing.</b>



DIY adapter build instructions as of 10/23:</br>


NES Adapter Guide:<br />

Notice:<br />
	-The materials are available as of 10/23.  Likely won't be in the near future.<br />
	-Materials and construction and installation directions haven't been tested.  Use at your discretion.
	
Materials Needed:<br />

[Arduino Pro Micro with Headers](https://www.amazon.com/Arduino-Micro-Headers-A000053-Controller/dp/B00AFY2S56?th=1)<br />
	
[Female NES Port](https://www.aliexpress.us/item/2251832689560217.html?srcSns=sns_Copy&spreadType=socialShare&bizType=ProductDetail&social_params=60361119023&aff_fcid=1da70e612d4048338cb9946ae5951742-1698699388161-05325-_EjiqCHD&tt=MG&aff_fsk=_EjiqCHD&aff_platform=default&sk=_EjiqCHD&aff_trace_key=1da70e612d4048338cb9946ae5951742-1698699388161-05325-_EjiqCHD&shareId=60361119023&businessType=ProductDetail&platform=AE&terminal_id=c4452e27a79e41f58865781377c836c2&afSmartRedirect=y&gatewayAdapt=glo2usa4itemAdapt) <br />

[Wire Jumpers to connect Arduino Header pins to Nes port pins](https://www.amazon.com/Breadboard-Jumper-Arduino-Raspberry-Prototyping/dp/B08NVKVSSP/ref=sr_1_14?crid=2495J9U9WL7YM&keywords=female+to+female+jumper+wires&qid=1698706338&s=electronics&sprefix=female+to+female+jumper+wire%2Celectronics%2C104&sr=1-14) <br />
	
[Micro usb cable to connect Arudino to computer](https://www.amazon.com/Amazon-Basics-Charging-Transfer-Gold-Plated/dp/B07232M876/ref=sr_1_3?keywords=micro+usb+to+usb+cable&qid=1698706739&sr=8-3) <br />
	
![Wiring Diagram:](https://github.com/alex-ong/LaglessNESUSB/raw/master/Software/Images/nes.png)

Construction and installation directions:
	
1. Connect Arduino pins to NES port pins corresponding to Wiring Diagram.<br />
2. Connect Arduino to computer with usb micro to usb cable<br />
3. Download and install Arudino IDE from https://www.arduino.cc/en/software<br />
4. Open Arudino IDE, navigate to Sketch -> Include Library -> Manage Libraries and search for Keyboard, and click Install under "Keyboard by Arduino".<br />
5. Navigate to Tools -> Board and click Arduino Leonardo  <br />
6. Navigate to Tools -> Port and select available com port<br />
7. Copy text from https://raw.githubusercontent.com/PolicyChanges/FastNESArduinoAdapter/main/FastNESArduinoHidAdapter/FastNESArduinoHidAdapter.ino<br />
	into the Arduino IDE and click the forward(upload) button<br />
8. If promted to save file, enter FastNESArduinoHidAdapter and save.<br />
9. Once the upload is finished, the NES controller should behave as a low latency keyboard<br />
