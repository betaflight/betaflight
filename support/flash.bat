@@echo OFF

:: User configuration

:: set your port number. ie: for COM6 , port is 6
set PORT=2
:: location of stmflashloader.exe, this is default
set FLASH_LOADER="C:\Program Files (x86)\STMicroelectronics\Software\Flash Loader Demonstrator\STMFlashLoader.exe"
:: path to firmware
set FIRMWARE="D:\baseflight.hex"

:: ----------------------------------------------

mode COM%PORT% BAUD=115200 PARITY=N DATA=8 STOP=1 XON=OFF DTR=OFF RTS=OFF
echo/|set /p ="R" > COM%PORT%:

TIMEOUT /T 3

cd %LOADER_PATH%

%FLASH_LOADER% ^
    -c --pn %PORT% --br 115200 --db 8 ^
    -i STM32_Med-density_128K ^
    -e --all ^
    -d --fn %FIRMWARE% ^
    -r --a 0x8000000
