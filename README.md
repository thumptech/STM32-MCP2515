# STM32-MCP2515
 C Driver for MCP2515 on STM32 MCU
This is a backport of the C++ Arduino driver found here https://github.com/autowp/arduino-mcp2515

SPI settings are hardcoded.

Set the correct HAL includes for the target platform.  This has only been used on STM32F4 so far.

I added the MCP_ prefix to the class functions, otherwise the documentations for that project should apply.
