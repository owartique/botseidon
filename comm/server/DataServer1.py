# DataServer2.py

from tcpcom import TCPServer
import time

IP_PORT = 22000

def onStateChanged(state, msg):
    if state == "LISTENING":
        print("Server:-- Listening...")
    elif state == "CONNECTED":
        print ("Server:-- Connected to", msg)
    elif state == "MESSAGE":
        print ("Server:-- Message received:", msg)
        if msg == "go":
            server.sendMessage("Hello from server")



server = TCPServer(IP_PORT, stateChanged = onStateChanged)
