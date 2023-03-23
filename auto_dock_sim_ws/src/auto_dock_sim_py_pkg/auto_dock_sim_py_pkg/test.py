import logging
import threading
import time

def thread_function(name):
    while(True):
        logging.info("Thread %s: starting", name)
        time.sleep(10)
        logging.info("Thread %s: finishing", name)

def two_function(name):
    while(True):
        logging.info("Thread %s: starting", name)
        time.sleep(2)
        logging.info("Thread %s: finishing", name)

if __name__ == "__main__":
    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%S")

    logging.info("Main    : before creating thread")
    x = threading.Thread(target=thread_function, args=(1,))
    x.start()

    y = threading.Thread(target=two_function, args=(2,))
    y.start()