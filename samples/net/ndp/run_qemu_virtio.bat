@echo off
REM ===========================================================================
REM  run_qemu_virtio.bat -- NDP Sample QEMU Launcher (virtio-net / e1000)
REM ===========================================================================
REM
REM  PURPOSE:
REM    Builds and launches the Zephyr NDP (Neural Data Pipeline) sample
REM    application inside QEMU with virtio-net or e1000 networking.
REM
REM  PREREQUISITES:
REM    1. Zephyr SDK installed (tested with zephyr-sdk-0.17.1)
REM    2. Python virtual environment with "west" installed
REM    3. WSL environment with ZEPHYR_BASE and ZEPHYR_SDK_INSTALL_DIR set
REM
REM  USAGE (from Windows Command Prompt or PowerShell):
REM    run_qemu_virtio.bat [MODE] [OPTIONS]
REM
REM    MODE:
REM      virtio       virtio-net-pci networking (default)
REM      e1000        Intel e1000 networking
REM      dummy        Dummy L2 (no real network - quick smoke tests)
REM      perf         Performance benchmark mode (dummy L2 + cycle counter)
REM
REM    OPTIONS:
REM      /build       Force rebuild before running
REM      /clean       Clean build directory before rebuild
REM      /trace       Enable QEMU serial port tracing (debug UART issues)
REM      /debug       Launch with GDB server on port 1234 (-s -S)
REM      /mem:N       Set VM memory in MB (default: 32)
REM      /help        Show this help message
REM
REM  EXAMPLES:
REM    run_qemu_virtio.bat                          Run with virtio-net (default)
REM    run_qemu_virtio.bat e1000 /build             Rebuild + run with e1000
REM    run_qemu_virtio.bat perf /clean /build       Clean build + perf mode
REM    run_qemu_virtio.bat virtio /trace /debug     Debug with GDB + UART trace
REM
REM  QUICK START (first time, in WSL):
REM    1. source /mnt/c/github/zephyr_project/.wsl_venv/bin/activate
REM    2. export ZEPHYR_BASE=/mnt/c/github/zephyr_project/zephyr
REM    3. export ZEPHYR_SDK_INSTALL_DIR=/mnt/c/mnt/zephyr_sdk/zephyr-sdk-0.17.1
REM    4. cd /mnt/c/github/zephyr_project/zephyr
REM    5. west build -b qemu_x86 samples/net/ndp -d samples/net/ndp/build_virtio -DDTC_OVERLAY_FILE=samples/net/ndp/virtio.overlay -DEXTRA_CONF_FILE=samples/net/ndp/overlay-virtio.conf
REM    6. Run this script: run_qemu_virtio.bat virtio
REM
REM  EXIT QEMU:
REM    Press Ctrl+A, then X   (when using -nographic)
REM
REM  SEND TEST TRAFFIC (from another PowerShell window):
REM    powershell -File send_traffic.ps1 -Count 100 -PacketSize 64
REM    ping 10.0.2.15
REM
REM ===========================================================================

setlocal enabledelayedexpansion

REM ===========================================================================
REM  STEP 1: Parse command-line arguments
REM ===========================================================================
set MODE=virtio
set DO_BUILD=0
set DO_CLEAN=0
set DO_TRACE=0
set DO_DEBUG=0
set MEM_SIZE=32

:parse_args
if "%~1"=="" goto :args_done
if /i "%~1"=="virtio"   set MODE=virtio   & goto :next_arg
if /i "%~1"=="e1000"    set MODE=e1000    & goto :next_arg
if /i "%~1"=="dummy"    set MODE=dummy    & goto :next_arg
if /i "%~1"=="perf"     set MODE=perf     & goto :next_arg
if /i "%~1"=="/build"   set DO_BUILD=1    & goto :next_arg
if /i "%~1"=="/clean"   set DO_CLEAN=1    & goto :next_arg
if /i "%~1"=="/trace"   set DO_TRACE=1    & goto :next_arg
if /i "%~1"=="/debug"   set DO_DEBUG=1    & goto :next_arg
if /i "%~1"=="/help"    goto :show_help
if /i "%~1"=="/h"       goto :show_help
if /i "%~1"=="/?"       goto :show_help

echo %~1 | findstr /r "^/mem:" >nul
if %ERRORLEVEL%==0 (
    for /f "tokens=2 delims=:" %%a in ("%~1") do set MEM_SIZE=%%a
    goto :next_arg
)

echo [WARNING] Unknown argument: %~1
:next_arg
shift
goto :parse_args

:args_done

REM ===========================================================================
REM  STEP 2: Show help if requested
REM ===========================================================================
:show_help
if /i "%~1"=="/help" goto :do_help
if /i "%~1"=="/h"    goto :do_help
if /i "%~1"=="/?"    goto :do_help
goto :skip_help

:do_help
echo.
echo ===========================================================================
echo  NDP QEMU Launcher - Help
echo ===========================================================================
echo.
echo  USAGE: run_qemu_virtio.bat [MODE] [OPTIONS]
echo.
echo  MODES:
echo    virtio     virtio-net-pci networking (guest IP: 10.0.2.15)  [DEFAULT]
echo    e1000      Intel e1000 PCI networking (guest IP: 10.0.2.15)
echo    dummy      Dummy L2 - no real networking (fast smoke tests)
echo    perf       Performance benchmark - dummy L2 + TSC cycle counter
echo.
echo  OPTIONS:
echo    /build     Rebuild the sample before launching QEMU
echo    /clean     Clean the build directory before rebuilding
echo    /trace     Enable QEMU serial port tracing (-trace enable=serial*)
echo    /debug     Start QEMU with GDB server on port 1234 (-s -S)
echo    /mem:N     Set VM memory to N MB (default: 32)
echo    /help      Show this help message
echo.
echo  EXAMPLES:
echo    run_qemu_virtio.bat                         Default virtio-net mode
echo    run_qemu_virtio.bat e1000 /build            Rebuild + run with e1000
echo    run_qemu_virtio.bat perf /clean /build      Clean build + perf mode
echo    run_qemu_virtio.bat /trace /debug           Debug with GDB + UART trace
echo    run_qemu_virtio.bat dummy /mem:64           Dummy mode, 64 MB RAM
echo.
echo  FILES USED:
echo    overlay-virtio.conf    Kconfig overlay for virtio-net
echo    overlay-e1000.conf     Kconfig overlay for e1000
echo    virtio.overlay         Devicetree overlay for virtio-net-pci
echo    e1000.overlay          Devicetree overlay for e1000
echo    send_traffic.ps1       PowerShell script to send test UDP packets
echo.
echo  REQUIRED ENVIRONMENT (WSL):
echo    ZEPHYR_BASE          = /mnt/c/github/zephyr_project/zephyr
echo    ZEPHYR_SDK_INSTALL_DIR = /mnt/c/mnt/zephyr_sdk/zephyr-sdk-0.17.1
echo    west                 = in PATH (from Python venv)
echo.
pause
exit /b 0

:skip_help

REM ===========================================================================
REM  STEP 3: Path configuration
REM ===========================================================================

REM --- Zephyr project paths (WSL format for west build) ---
set "ZEPHYR_BASE_WSL=/mnt/c/github/zephyr_project/zephyr"
set "SAMPLE_DIR_WSL=/mnt/c/github/zephyr_project/zephyr/samples/net/ndp"
set "BUILD_DIR_WSL=%SAMPLE_DIR_WSL%/build_%MODE%"
set "KERNEL_WSL=%BUILD_DIR_WSL%/zephyr/zephyr.elf"

REM --- Zephyr SDK paths ---
set "SDK_DIR=/mnt/c/mnt/zephyr_sdk/zephyr-sdk-0.17.1"
set "QEMU_BIN_WSL=%SDK_DIR%/sysroots/x86_64-pokysdk-linux/usr/bin/qemu-system-i386"

REM --- Python venv ---
set "VENV_DIR=/mnt/c/github/zephyr_project/.wsl_venv"

REM --- Overlay files (relative to sample dir) ---
if /i "%MODE%"=="virtio" (
    set "EXTRA_CONF=overlay-virtio.conf"
    set "DTC_OVERLAY=virtio.overlay"
    set "QEMU_NETDEV=-netdev user,id=net0 -device virtio-net-pci,netdev=net0"
    set "NET_LABEL=virtio-net-pci"
)
if /i "%MODE%"=="e1000" (
    set "EXTRA_CONF=overlay-e1000.conf"
    set "DTC_OVERLAY=e1000.overlay"
    set "QEMU_NETDEV=-netdev user,id=net0 -device e1000,netdev=net0"
    set "NET_LABEL=Intel e1000"
)
if /i "%MODE%"=="dummy" (
    set "EXTRA_CONF=NONE"
    set "DTC_OVERLAY=NONE"
    set "QEMU_NETDEV="
    set "NET_LABEL=Dummy L2 (no real network)"
)
if /i "%MODE%"=="perf" (
    set "EXTRA_CONF=NONE"
    set "DTC_OVERLAY=NONE"
    set "QEMU_NETDEV="
    set "NET_LABEL=Dummy L2 (performance benchmark)"
)

REM ===========================================================================
REM  STEP 4: Build (if requested)
REM ===========================================================================
if %DO_BUILD%==0 goto :skip_build

echo.
echo ===========================================================================
echo  STEP 4: Building NDP sample [MODE=%MODE%]
echo ===========================================================================
echo  Zephyr base:   %ZEPHYR_BASE_WSL%
echo  Sample dir:    %SAMPLE_DIR_WSL%
echo  Build dir:     %BUILD_DIR_WSL%
echo  Extra conf:    %EXTRA_CONF%
echo  DTC overlay:   %DTC_OVERLAY%
echo  Network:       %NET_LABEL%
echo ---------------------------------------------------------------------------

REM --- Build the WSL command ---
set "BUILD_CMD=export ZEPHYR_BASE=%ZEPHYR_BASE_WSL% && export ZEPHYR_SDK_INSTALL_DIR=%SDK_DIR% && export ZEPHYR_TOOLCHAIN_VARIANT=zephyr && cd %SAMPLE_DIR_WSL%"

if %DO_CLEAN%==1 (
    set "BUILD_CMD=!BUILD_CMD! && rm -rf %BUILD_DIR_WSL%"
    echo  Cleaning build directory...
)

set "BUILD_CMD=!BUILD_CMD! && west build -p auto -b qemu_x86 -d %BUILD_DIR_WSL%"

if not "%EXTRA_CONF%"=="NONE" (
    set "BUILD_CMD=!BUILD_CMD! -- -DEXTRA_CONF_FILE=%EXTRA_CONF%"
)
if not "%DTC_OVERLAY%"=="NONE" (
    set "BUILD_CMD=!BUILD_CMD! -DDTC_OVERLAY_FILE=%DTC_OVERLAY%"
)

set "BUILD_CMD=!BUILD_CMD! 2>&1"

echo  Running: !BUILD_CMD!
echo.
wsl -e bash -c "!BUILD_CMD!"
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Build failed! Check output above for details.
    echo   Common issues:
    echo     - Is ZEPHYR_BASE set correctly?
    echo     - Is the Zephyr SDK installed at %SDK_DIR%?
    echo     - Is the Python venv activated?
    echo     - Run with /clean if stale build artifacts exist.
    pause
    exit /b 1
)
echo  Build succeeded.
:skip_build

REM ===========================================================================
REM  STEP 5: Verify files exist (via WSL)
REM ===========================================================================
echo.
echo ===========================================================================
echo  STEP 5: Verifying build artifacts
echo ===========================================================================

wsl -e bash -c "test -f %QEMU_BIN_WSL%" 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] QEMU binary not found: %QEMU_BIN_WSL%
    echo   Check ZEPHYR_SDK_INSTALL_DIR in this script.
    pause
    exit /b 1
)
echo  [OK] QEMU: %QEMU_BIN_WSL%

wsl -e bash -c "test -f %KERNEL_WSL%" 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Kernel ELF not found: %KERNEL_WSL%
    echo   Build the sample first with: run_qemu_virtio.bat %MODE% /build
    pause
    exit /b 1
)
echo  [OK] Kernel: %KERNEL_WSL%

REM ===========================================================================
REM  STEP 6: Display run configuration
REM ===========================================================================
echo.
echo ===========================================================================
echo  STEP 6: Launching QEMU
echo ===========================================================================
echo  Mode:        %MODE% (%NET_LABEL%)
echo  Memory:      %MEM_SIZE% MB
echo  Debug:       %DO_DEBUG% (GDB port 1234)
echo  UART Trace:  %DO_TRACE%
echo ---------------------------------------------------------------------------
echo  Guest IP:    10.0.2.15 (QEMU user-mode network default)
echo  Exit QEMU:   Press Ctrl+A, then X
echo ---------------------------------------------------------------------------
echo  Send test traffic from another terminal:
echo    powershell -File send_traffic.ps1 -Count 100
echo    ping 10.0.2.15
echo ===========================================================================
echo.

REM ===========================================================================
REM  STEP 7: Build QEMU command line
REM ===========================================================================

REM --- Base QEMU flags ---
set QEMU_FLAGS=-m %MEM_SIZE%
set QEMU_FLAGS=%QEMU_FLAGS% -cpu qemu32,+nx,+pae
set QEMU_FLAGS=%QEMU_FLAGS% -machine q35
set QEMU_FLAGS=%QEMU_FLAGS% -device isa-debug-exit,iobase=0xf4,iosize=0x04
set QEMU_FLAGS=%QEMU_FLAGS% -no-reboot
set QEMU_FLAGS=%QEMU_FLAGS% -machine acpi=off

REM --- Network device (mode-specific) ---
if not "%QEMU_NETDEV%"=="" (
    set QEMU_FLAGS=%QEMU_FLAGS% %QEMU_NETDEV%
)

REM --- Display and serial ---
set QEMU_FLAGS=%QEMU_FLAGS% -chardev stdio,id=con,mux=on
set QEMU_FLAGS=%QEMU_FLAGS% -serial chardev:con
set QEMU_FLAGS=%QEMU_FLAGS% -mon chardev=con,mode=readline
set QEMU_FLAGS=%QEMU_FLAGS% -nographic

REM --- Debug mode: add GDB server ---
if %DO_DEBUG%==1 (
    set QEMU_FLAGS=%QEMU_FLAGS% -s -S
    echo.
    echo  *** GDB SERVER ACTIVE on tcp::1234 ***
    echo  Connect with: x86_64-zephyr-elf-gdb -ex "target remote :1234" %KERNEL_WSL%
    echo  QEMU is PAUSED - resume with (gdb) continue
    echo.
)

REM --- Trace mode: add serial trace ---
if %DO_TRACE%==1 (
    set QEMU_FLAGS=%QEMU_FLAGS% -trace enable=serial*
    echo  [TRACE] Serial port tracing enabled (output will be very verbose!)
    echo.
)

REM --- Kernel ---
set QEMU_FLAGS=%QEMU_FLAGS% -kernel %KERNEL_WSL%

REM ===========================================================================
REM  STEP 8: Launch QEMU
REM ===========================================================================

echo Full command:
echo   wsl -e %QEMU_BIN_WSL% %QEMU_FLAGS%
echo.

wsl -e %QEMU_BIN_WSL% %QEMU_FLAGS%

REM ===========================================================================
REM  STEP 9: Post-run troubleshooting
REM ===========================================================================
echo.
echo ===========================================================================
echo  QEMU has exited.
echo ===========================================================================
echo.
echo  TROUBLESHOOTING GUIDE:
echo  ----------------------
echo  No serial output?
echo    1. Run with /trace to see UART register activity
echo    2. Check that MCR OUT2 bit is set (early_serial.c fix applied)
echo    3. Verify CONFIG_X86_VERY_EARLY_CONSOLE=y in prj.conf
echo.
echo  Kernel panic or hang?
echo    1. Run with /debug and connect GDB:
echo       x86_64-zephyr-elf-gdb -ex "target remote :1234" ^
echo         -ex "hbreak z_fatal_error" -ex "continue" %KERNEL_WSL%
echo    2. Check /d guest_errors in QEMU flags for faults
echo.
echo  No network connectivity?
echo    1. Verify QEMU netdev matches the devicetree overlay:
echo       virtio mode needs: -device virtio-net-pci,netdev=net0
echo       e1000 mode needs:  -device e1000,netdev=net0
echo    2. Guest IP is 10.0.2.15 (QEMU user-mode default)
echo    3. Test with: ping 10.0.2.15
echo.
echo  Build fails?
echo    1. Run with /clean to remove stale build artifacts
echo    2. Verify Zephyr SDK path: %SDK_DIR%
echo    3. Check that west is in WSL PATH
echo.
echo  Wrong SDK version?
echo    Update SDK_DIR in this script. Current: %SDK_DIR%
echo.
echo  RELATED FILES:
echo    ndp_vs_dpdk_benchmark.html - Performance comparison with DPDK
echo    send_traffic.ps1           - PowerShell UDP packet generator
echo    overlay-virtio.conf        - Kconfig for virtio-net mode
echo    overlay-e1000.conf         - Kconfig for e1000 mode
echo    virtio.overlay             - Devicetree for virtio-net-pci
echo    e1000.overlay              - Devicetree for e1000
echo.
pause
endlocal