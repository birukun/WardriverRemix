# WardriverRemix
A remix of the wardriver.uk project - removed GSM module and updated layout in a new custom PCB from MrBill

Included code is a stripped down version of the wardriver.uk code 
- removed GSM Module code
- removed BLE scanning
- removed advanced features like web portal etc.
This code starts working as soon as power is applied and a MicroSD card is present and writable.
In order to retrieve files, you must copy them by hand from the MicroSD card. I recommend mounting the MicroSD card reader at 90 degrees to make card exchanges easy. 


# Dependencies
- Arduino IDE
- Libraries (install using IDE)
    - GParser Library
    - MicroNMEA
    - ESP32Time
    - OneWire

# Debugging
Use Arduino IDE serial monitor at 115200 baud, both ESP32s will output basic serial logging. 
