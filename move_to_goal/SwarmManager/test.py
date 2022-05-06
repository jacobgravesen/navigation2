from time import sleep
import threading


def coffee():
    while True:
        print("coffee")
        sleep(1)

def tea():
    while True:
        print("tea")
        sleep(1)

tea_thread = threading.Thread(target=tea)
tea_thread.start()

coffe_thread = threading.Thread(target=coffee)
coffe_thread.start()