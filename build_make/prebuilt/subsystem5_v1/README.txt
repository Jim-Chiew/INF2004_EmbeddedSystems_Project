Buddy 5 prototype v1.0 - Pico W, micro T-Kernel 3.0 SMP, USB-CDC console + Wi-Fi.
Built from branch buddy5-ultrasonic with: make clean && make -j8 (PICO_SDK_PATH set).
Flash: hold BOOTSEL, plug in the Pico W, drag the .uf2 onto the RPI-RP2 drive.
Serial: USB serial device at 115200 baud, expect "[s5] started: threshold=150 mm".
Wiring: see app_program/subsystem5/README.md.
