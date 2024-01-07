#!usr/bin/python
import threading
import time
import sys
from ardSerial import *

class Controller:
    # Class that manages the movement commands and sends them to the NyBoard

    def __init__(self):
        self.stopped = False
        self.command = ""  
        self.key = 255  
        self.goodPorts = {}
        connectPort(self.goodPorts) 
        self.start('')

    def start(self, keyPressed):
        # start the thread
        threading.Thread(target=keepCheckingPort, args=(goodPorts,)).start()
        # threading.Thread(target=self.remoteControl, args=(keyPressed,)).start()
        threading.Thread(target=self.move_forward).start()
        return self
    

    def move_forward(self):
        self.command = "k" + "wk" + "B"
        while True:
            send(self.goodPorts, [self.command, "\n"])

    def rest(self):
        self.command = "d" 
        send(self.goodPorts, [self.command, 1])

    def stop(self):
        # shut down servos of the mIDOG
        self.rest()
        time.sleep(1)
        # stop thread
        self.stopped = True
        # close serial communication with the NyBoard
        time.sleep(0.2)
        closeAllSerial(self.goodPorts)


if __name__ == "__main__":
    controller = Controller()
    # controller.move_forward()
    # sudo chmod 666 /dev/ttyS0 
    # sudo chmod 666 /dev/ttyAMA0 
