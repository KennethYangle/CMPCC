SET PSP_PATH=C:\PX4PSP
start cmd.exe /k "echo Python3.8 environment has been set with openCV+pymavlink+numpy+pyulog etc. && echo You can use pip or pip3 command to install other libraries && echo Put your python scripts 'XXX.py' into the folder '%CD%' && echo Use the command: 'python XXX.py' to run the script with Python && SET PATH=%PSP_PATH%\Python38;%CD%"
