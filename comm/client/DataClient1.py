# DataClient2.py

from tcpcom import TCPClient
import time

IP_ADDRESS = "10.3.141.4"
IP_PORT = 22000

def onStateChanged(state, msg):
    global isConnected
    if state == "CONNECTING":
       print ("Client:-- Waiting for connection...")
    elif state == "CONNECTED":
       print ("Client:-- Connection estabished.")
    elif state == "DISCONNECTED":
       print ("Client:-- Connection lost.")
       isConnected = False
    elif state == "MESSAGE":
       print ("Client:-- Received data:", msg)

client = TCPClient(IP_ADDRESS, IP_PORT, stateChanged = onStateChanged)
rc = client.connect()
if rc:
    isConnected = True
    while isConnected:
        print ("Client:-- Sending command: go...")
        client.sendMessage("coucou depuis mon ordi")
        time.sleep(2)
    print ("Done")
else:
    print ("Client:-- Connection failed")
