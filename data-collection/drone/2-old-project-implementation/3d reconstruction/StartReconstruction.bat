@echo off

if [%3]==[] goto :usage

:: Convert video to images at original video quality
echo Converting video %1 into images at 1/5 Frames Per Second
echo Converting...
:: python .\videotoimg.py %1
echo Converted %1 to images

echo Starting Reconstruction with images in Images/%2 and output in Builds/%3
:: C:/Meshroom-2021.1.0/meshroom_batch.exe --input Images/%2 --output Builds/%3
goto :end

:usage
echo Usage: StartReconstruction.bat [VideoName] [InputImagesFolder(Same as video name)] [OutputModelFolder]
exit /B 1

:end