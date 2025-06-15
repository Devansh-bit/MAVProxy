cd ..\
python.exe -m pip install --upgrade build .
python.exe .\MAVProxy\mavproxy.py --master=0.0.0.0:14550 --console
pause
