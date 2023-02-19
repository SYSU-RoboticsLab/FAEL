@echo off
setlocal
REM This simple wrapper allowing us to pass a set of
REM environment variables to be sourced prior to running
REM another command. Used in the launch file for setting
REM robot configurations prior to xacro.

set ENVVARS_FILE=%1

REM get arguments starting from %2
shift
set RUNNER_COMMAND=
:getrunnerloop
if "%1"=="" goto after_loop
if "%RUNNER_COMMAND%" == "" (
    set RUNNER_COMMAND=%1
) else (
    set RUNNER_COMMAND=%RUNNER_COMMAND% %1
)
shift
goto getrunnerloop

:after_loop
call %ENVVARS_FILE%
call %RUNNER_COMMAND%
