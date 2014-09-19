
from manualControl import py_to_joy

class handler(object):

    #__socket=None
    def __init__(self,socket):
        self.socket=socket
   
    def handle(self):
        s=self.socket
        #diabase thn katastash sthn opoia 8elei na metabei o xrhsths
        while(True):
            try:
                state=int(s.recv(32))
                print(state)
            except ValueError:
                    print("Not a valid number")
                    
            #metabash sthn katastash automatou xeirismou an state=1
            if (state==1):
                print("Pame py_to_joys(s)")
                x=py_to_joy(s)
                x.controller()
            #elif
                #######
                #######
            #elif (state=0):
            
                