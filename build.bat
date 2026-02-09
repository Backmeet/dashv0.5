@echo off
setlocal EnableDelayedExpansion

REM ================= USER CONFIG =================
set SRC=main.c++
set OUT=auto
set LIBS=-static
REM ===============================================

REM ================= ARGS =================
set FORCE_BUILD=0
if /I "%1"=="force" set FORCE_BUILD=1
if /I "%2"=="force" set FORCE_BUILD=1

REM Normalize lists
set SRC_LIST=%SRC%
set OUT_LIST=%OUT%

REM Count SRC items
set COUNT=0
for %%A in (%SRC_LIST%) do set /a COUNT+=1

set /a LAST=COUNT-1

REM Convert SRC to indexed variables
set IDX=0
for %%A in (%SRC_LIST%) do (
    set SRC_!IDX!=%%A
    set /a IDX+=1
)

REM Convert OUT to indexed variables
set IDX=0
for %%A in (%OUT_LIST%) do (
    set OUT_!IDX!=%%A
    set /a IDX+=1
)

REM ================= BUILD =================
set BUILD_ERRORS=0

for /L %%I in (0,1,%LAST%) do (
    set SRC_FILE=!SRC_%%I!
    set OUT_FILE=!OUT_%%I!

    REM Auto output name
    if /I "!OUT_FILE!"=="auto" (
        for %%X in ("!SRC_FILE!") do set OUT_FILE=%%~nX.exe
    )

    REM Detect compiler
    for %%X in ("!SRC_FILE!") do set EXT=%%~xX
    if /I "!EXT!"==".c" (
        set COMPILER=gcc
    ) else (
        set COMPILER=g++
    )

    REM Timestamp check
    set SKIP_BUILD=0
    if "!FORCE_BUILD!"=="0" (
        if exist "!OUT_FILE!" (
            for %%S in ("!SRC_FILE!") do set SRCTIME=%%~tS
            for %%O in ("!OUT_FILE!") do set OUTTIME=%%~tO
            if "!SRCTIME!" LEQ "!OUTTIME!" set SKIP_BUILD=1
        )
    )

    if "!SKIP_BUILD!"=="1" (
        echo [BUILD]: Nothing changed for !SRC_FILE!, skipping.
    ) else (
        echo [BUILD]: Building !SRC_FILE! -> !OUT_FILE!
        "!COMPILER!" "!SRC_FILE!" %LIBS% -o "!OUT_FILE!"

        if errorlevel 1 (
            echo "[BUILD]: Build failed for !SRC_FILE!, with error(s)"
            set /a BUILD_ERRORS+=1
        )
    )
)

if %BUILD_ERRORS% GTR 0 (
    exit /b 1
)

echo [BUILD]: Build succeeded!

REM ================= RUN =================
if /I "%1" NEQ "run" if /I "%2" NEQ "run" goto :eof

set RUN_TARGET=
if /I "%1"=="run" set RUN_TARGET=%2
if /I "%2"=="run" set RUN_TARGET=%1

REM Default: first OUT
if "%RUN_TARGET%"=="" (
    if /I "!OUT_0!"=="auto" (
        for %%X in ("!SRC_0!") do set RUN_FILE=%%~nX.exe
    ) else (
        set RUN_FILE=!OUT_0!
    )
    goto do_run
)

REM Find matching SRC
set FOUND=0
for /L %%I in (0,1,%LAST%) do (
    for %%X in ("!SRC_%%I!") do (
        if /I "%%~nX"=="%RUN_TARGET%" (
            set FOUND=1
            if /I "!OUT_%%I!"=="auto" (
                set RUN_FILE=%%~nX.exe
            ) else (
                set RUN_FILE=!OUT_%%I!
            )
        )
    )
)

if "%FOUND%"=="0" (
    echo [RUN]: Error cannot find %RUN_TARGET% in [%SRC_LIST%]
    exit /b 1
)

:do_run
echo [RUN]: Running !RUN_FILE!
"!RUN_FILE!"
