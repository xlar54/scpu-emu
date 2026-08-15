@echo off
setlocal EnableExtensions

set "ROOT=%~dp0"
set "BASIC=%ROOT%basic"
set "ASM=%ROOT%asm"
set "OBJ=%ROOT%obj"
set "PRG=%ROOT%prg"
set "DISKS=%ROOT%diskimages"
set "PETCAT=%ROOT%petcat.exe"
set "TASS=%ROOT%64tass.exe"
set "C1541=%ROOT%c1541.exe"
set "DISK=%DISKS%\SCPU-TESTS.d81"

for %%T in ("%PETCAT%" "%TASS%" "%C1541%") do (
    if not exist "%%~T" (
        echo ERROR: Required tool not found: %%~T
        exit /b 1
    )
)

if not exist "%OBJ%" mkdir "%OBJ%"
if not exist "%PRG%" mkdir "%PRG%"
if not exist "%DISKS%" mkdir "%DISKS%"

del /q "%PRG%\*.prg" 2>nul
del /q "%OBJ%\*.lst" "%OBJ%\*.lbl" 2>nul

call :basic "00-detect.bas" "00-DETECT"
if errorlevel 1 exit /b 1
call :basic "01-speed.bas" "01-SPEED"
if errorlevel 1 exit /b 1
call :asm "02-private.asm" "02-PRIVATE"
if errorlevel 1 exit /b 1
call :basic "03-screen.bas" "03-SCREEN"
if errorlevel 1 exit /b 1
call :basic "04-raster.bas" "04-RASTER"
if errorlevel 1 exit /b 1
call :basic "05-animation.bas" "05-ANIMATION"
if errorlevel 1 exit /b 1
call :asm "06-cube.asm" "06-CUBE"
if errorlevel 1 exit /b 1
call :asm "07-mandelbrot.asm" "07-MANDELBROT"
if errorlevel 1 exit /b 1
call :asm "08-soundtest.asm" "08-SOUNDTEST"
if errorlevel 1 exit /b 1
call :asm "10-cpu816.asm" "10-CPU816"
if errorlevel 1 exit /b 1
call :asm "11-superram.asm" "11-SUPERRAM"
if errorlevel 1 exit /b 1
call :asm "12-rasterirq.asm" "12-RASTERIRQ"
if errorlevel 1 exit /b 1
call :asm "13-vicbanks.asm" "13-VICBANKS"
if errorlevel 1 exit /b 1
call :asm "14-spriteballs.asm" "14-SPRITEBALLS"
if errorlevel 1 exit /b 1
call :asm "15-scpu128probe.asm" "15-SCPU128PROBE"
if errorlevel 1 exit /b 1
call :asm "16-nmitiming.asm" "16-NMITIMING"
if errorlevel 1 exit /b 1
call :asm "supermon816.asm" "17-SUPERMON816"
if errorlevel 1 exit /b 1
call :asm "18-ramspeed.asm" "18-RAMSPEED"
if errorlevel 1 exit /b 1

if exist "%DISK%" del /q "%DISK%"
if exist "%DISKS%\SCPU-TESTS.d64" del /q "%DISKS%\SCPU-TESTS.d64"
"%C1541%" -format "scpu tests,st" d81 "%DISK%" ^
    -write "%PRG%\00-DETECT.prg" "00-detect" ^
    -write "%PRG%\01-SPEED.prg" "01-speed" ^
    -write "%PRG%\02-PRIVATE.prg" "02-private" ^
    -write "%PRG%\03-SCREEN.prg" "03-screen" ^
    -write "%PRG%\04-RASTER.prg" "04-raster" ^
    -write "%PRG%\05-ANIMATION.prg" "05-animation" ^
    -write "%PRG%\06-CUBE.prg" "06-cube" ^
    -write "%PRG%\07-MANDELBROT.prg" "07-mandelbrot" ^
    -write "%PRG%\08-SOUNDTEST.prg" "08-soundtest" ^
    -write "%PRG%\10-CPU816.prg" "10-cpu816" ^
    -write "%PRG%\11-SUPERRAM.prg" "11-superram" ^
    -write "%PRG%\12-RASTERIRQ.prg" "12-rasterirq" ^
    -write "%PRG%\13-VICBANKS.prg" "13-vicbanks" ^
    -write "%PRG%\14-SPRITEBALLS.prg" "14-spriteballs" ^
    -write "%PRG%\15-SCPU128PROBE.prg" "15-scpu128probe" ^
    -write "%PRG%\16-NMITIMING.prg" "16-nmitiming" ^
    -write "%PRG%\17-SUPERMON816.prg" "17-supermon816" ^
    -write "%PRG%\18-RAMSPEED.prg" "18-ramspeed"
if errorlevel 1 exit /b 1

echo.
echo Built PRG files in "%PRG%"
echo Built disk image "%DISK%"
exit /b 0

:basic
echo petcat  %~1
"%PETCAT%" -w2 -o "%PRG%\%~2.prg" -- "%BASIC%\%~1"
if errorlevel 1 (
    echo ERROR: petcat failed for %~1
    exit /b 1
)
exit /b 0

:asm
echo 64tass %~1
"%TASS%" --quiet --m65816 --cbm-prg -I "%ASM%" -L "%OBJ%\%~2.lst" -l "%OBJ%\%~2.lbl" -o "%PRG%\%~2.prg" "%ASM%\%~1"
if errorlevel 1 (
    echo ERROR: 64tass failed for %~1
    exit /b 1
)
exit /b 0
