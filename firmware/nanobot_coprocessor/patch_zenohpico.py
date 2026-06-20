Import("env")
import os

# zenoh-pico's arduino-esp32 serial open calls HardwareSerial(uart).begin(baudrate)
# WITHOUT explicit pins, relying on the core's default UART pins. That proved
# unreliable here (UART2 didn't drive GPIO17). Force explicit pins from the locator.
LIB = ".pio/libdeps/esp32dev/zenoh-pico/src/system/arduino/esp32/network.cpp"
OLD = "sock->_serial->begin(baudrate);"
NEW = "sock->_serial->begin(baudrate, SERIAL_8N1, rxpin, txpin);"

if os.path.exists(LIB):
    s = open(LIB).read()
    if OLD in s:
        open(LIB, "w").write(s.replace(OLD, NEW))
        print("[patch_zenohpico] forced explicit UART pins in network.cpp")
    else:
        print("[patch_zenohpico] already patched / pattern not found")
