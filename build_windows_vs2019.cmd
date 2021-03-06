@echo off
submodules\bx\tools\bin\windows\genie vs2019

pause

echo Check Visual Studio version
IF EXIST "c:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\IDE" (
echo Using Visual Studio 2019 Professional Path
set "VISUALSTUDIO19PATH=c:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\IDE"
) else (
echo Using Visual Studio 2019 Community Path
set "VISUALSTUDIO19PATH=c:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE"
)

IF EXIST "%VISUALSTUDIO19PATH%" (
echo Building physx Debug in Visual Studio 2019
"%VISUALSTUDIO19PATH%\devenv" ".build\projects\vs2019\physx.sln" /Build "Debug|x64"
echo ErrorLevel:%ERRORLEVEL%
IF %ERRORLEVEL% EQU 0 (
   echo Build successful!
) else (
   echo Build failed!
)
echo Building physx Release in Visual Studio 2019
"%VISUALSTUDIO19PATH%\devenv" ".build\projects\vs2019\physx.sln" /Build "Release|x64"
echo ErrorLevel:%ERRORLEVEL%
IF %ERRORLEVEL% EQU 0 (
   echo Build successful!
) else (
   echo Build failed!
)
) else (
echo Visual Studio 2019 not found! Open 'submodules\physx\.build\projects\vs2019\physx.sln' yourself and build it with your own version (NOTE you'll need to change vs2019 above to your installed version)
)

pause