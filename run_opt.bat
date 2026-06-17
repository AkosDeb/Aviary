@echo off
cd /d c:\Software\Repository\Aviary_clean

echo Starting optimization...
python aviary/models/aircraft/horizontal_small_uav/run_horizontal_small_uav.py

echo.
echo Opening output folder...
start explorer outputs\run_horizontal_small_uav_try_v1_v1.19.0_out

echo.
echo Launching dashboard...
cd /d c:\Software\Repository\Aviary_clean\outputs
aviary dashboard run_horizontal_small_uav_try_v1_v1.19.0
