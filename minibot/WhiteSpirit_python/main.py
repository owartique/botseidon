from threading import Thread
from theServer import *


def runTheMainLoop(q):
    print("##############################################################################################################\n")
    print("\t\t\tWelcome to the Minibot project of the ELEME2002 class :)")
    print("##############################################################################################################\n")
    print("\t\t I'm White Spirit, please take care of me !\n")
    print("\t\t Please do not interchange the chips on my tower/motor PWM boards !\n")
    print("\t\t Try to respect the C-file interface when programming me because\n\t   \tit will be the same in the robotic project (Q2) !\n")
    print("\t\t As a piece of advice: encapsulate C-code, this will be much faster to be executed !\n")
    print("\t\t Before any tests on the motors, be sure that my wheels are not\n\t   \ton the ground so that I cannot go away dangerously !\n\n")
    print("\t\t\t(Press 'q' to end the program)\n")
    while True:
        try:
            value = q.get(False)
            if value == "POST":
                print("\n\t Someone pressed the beautiful button on my Web interface ;)\n\n")
        except Empty:
            pass

def quit():
    while True:
        cmd = input('')
        if cmd == 'q':
            exit()

if __name__ == '__main__':
    q = Queue()
    ender = Thread(target=quit)
    web = Thread(target=runServer, args=(q,), daemon=True)
    control = Thread(target=runTheMainLoop, args=(q,), daemon=True)
    control.start()
    web.start()
    ender.start()

