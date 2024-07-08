1. run client_ue4_SITL.bat to start a SITL simulation (or run client_ue4_HITL.bat to start HIL simulation, where the Pixhawk should be well configured)

2. Run Python38Run.bat and input "python client_ue4.py" to run the client program, which captures screens of RflySim3Ds and sends them to the network through UDP. 

3. Use VS Code to open server_ue4.py and run it in this computer to receive images from local network and process it with CV algorithms.

4. You can copy all of the files in this folder to another computer (Raspberry Pi, TX2, or any compuer with Linux and ROS environment), use command "python3 server_ue4.py" to receive images from the previous computer via UDP and process it with face recognition algorithms.

5. server_ue4_ROS.py is the ROS version of "server_ue4.py". The only different from them is the sentence "import PX4MavCtrlV4 as PX4MavCtrl" and "import PX4MavCtrlV4ROS as PX4MavCtrl", where PX4MavCtrlV4.py is the Mavlink API through pymavlink and PX4MavCtrlV4ROS.py is the Mavlink API through mavros.

6. In this demo, the broadcast UDP is adopted, where 
1) the "SET IS_BROADCAST=1" or (SET IS_BROADCAST=255.255.255.255) in client_ue4SITL.bat and client_ue4HITL.bat;  
2) the "TargetIP='255.255.255.255'" in client_ue4.py;
3) the "mav = PX4MavCtrl.PX4MavCtrler(20100,'255.255.255.255')" in server_ue4.py and server_ue4_ROS.py.

7. Use IP mode to improve the communication performance. Assuming that the master computer's IP is 192.168.1.20 (run client_ue4.py) and the target embedded computer's IP is 192.168.1.25 (run server_ue4.py). The following changes will enable IP communication  mode.
1) the "SET IS_BROADCAST=192.168.1.25" (the IP of target embedded computer) in client_ue4_SITL.bat and client_ue4_HITL.bat;  
2) the "TargetIP='192.168.1.25'" in client_ue4.py; (the IP of target embedded computer)
3) the "mav = PX4MavCtrl.PX4MavCtrler(20100,'192.168.1.20')" in server_ue4.py and server_ue4_ROS.py. (the IP of master computer)

8. PX4MavCtrlV4.py is the comunication API to Pixhawk through Mavlink (and to UE4 via UDP). and PX4MavCtrlV4ROS.py is the mavros version.

9. ScreenCapApiV4.py is the screen caputre API. The value "isNewUE=False" will enable old API for RflySim3D screen capture, whose speed is faster but is not compatible with UE4.23+; The value "isNewUE=False" will enable new API for RflySim3D screen capture, whose speed is a little slower but is compatible for all UE4 versions.

10. RflyVisionAPI.py is the image transmission API