@echo off
REM ===========================================================================
REM run_qemu.bat - Launch NDP sample in QEMU with e1000 networking
REM ===========================================================================
REM
REM Build the sample first:
REM   west build -b qemu_x86 -p auto samples/net/ndp ^
REM     -DEXTRA_CONF_FILE=overlay-e1000.conf ^
REM     -DDTC_OVERLAY_FILE=e1000.overlay
REM
REM Then run this script from the zephyr directory, or edit KERNEL_PATH
REM below to point to your built zephyr.elf.
REM ===========================================================================

setlocal enabledelayedexpansion

REM --- Configuration - edit these paths to match your environment ---

REM Path to your QEMU installation
set QEMU_PATH=C:Program Filesqemu

REM Path to your built zephyr.elf kernel
set KERNEL_PATH=C:\\github\\zephyr_project\\zephyr\\samples\\net\\ndp\\build\\zephyr\\zephyr.elf
set MEM_SIZE=32

REM --- Verify files exist ---

if not exist "%QEMU_PATH%qemu-system-i386.exe" (
    echo [ERROR] QEMU not found at: %QEMU_PATH%qemu-system-i386.exe
    echo   Update QEMU_PATH in this script to match your installation.
    pause
    exit /b 1
)

if not exist "%KERNEL_PATH%" (
    echo [ERROR] Kernel ELF not found at: %KERNEL_PATH%
    echo   Build the sample first or update KERNEL_PATH in this script.
    pause
    exit /b 1
)

echo ===========================================================================
echo  NDP Sample - QEMU Launcher
echo ===========================================================================
echo  QEMU:     %QEMU_PATH%qemu-system-i386.exe
echo  Kernel:   %KERNEL_PATH%
echo  Memory:   %MEM_SIZE% MB
echo  Network:  e1000 (guest IP: 10.0.2.15)
echo ---------------------------------------------------------------------------
echo  To send traffic, open a separate PowerShell window and run:
echo    ping 10.0.2.15
echo    ... or use the send_traffic.ps1 script
echo ---------------------------------------------------------------------------
echo  To exit QEMU: Press Ctrl+A, then X
echo ===========================================================================
echo.

REM --- Launch QEMU ---

"%QEMU_PATH%qemu-system-i386.exe" ^
    -m %MEM_SIZE% ^
    -cpu qemu32,+nx,+pae ^
    -machine q35 ^
    -device isa-debug-exit,iobase=0xf4,iosize=0x04 ^
    -no-reboot ^
    -machine acpi=off ^
    -netdev user,id=net0 ^
    -device e1000,netdev=net0 ^
    -chardev stdio,id=con,mux=on ^
    -serial chardev:con ^
    -mon chardev=con,mode=readline ^
    -nographic ^
    -kernel "%KERNEL_PATH%"

REM --- Post-exit ---

echo.
echo QEMU has exited.
pause
