# Bootloader
Stm32F4 Bootloader

How to flash app sw via bootloader.

0. Flash bootloader to chip

1.  Build sw with base addr = 0x8008000 (fls sector 2 base addr)
    Copy and rename .bin file to ./Host/user_app.bin

2. Wired - PC <-> USB TTL <-> STM32(UART Tx/Rx)

3. Pressed user button and reset chip -> blue led is on, bootloader is running.

4.  Open host program with python
    Erase fls memory from sector 2 to end of fls

5. Fls application sw to via cmd '8', addr 0x8008000

6. Reset chip with out pressing of user button
