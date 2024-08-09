import serial
import time

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

if (__name__ == "__main__"):
    #open the serial channel
    arduino = serial.Serial(port='COM11',  baudrate=115200, timeout=.1)

    #instantiate the Readline object
    rl = ReadLine(arduino)

    #create a pipeline loop for testing. Note that the Arduino code contains the latch
    while True:
        #create a variable for benchmarking cycle speed
        tstart = time.time()
        
        #send to arduino channel
        arduino.write(bytes(str(911),  'utf-8'))

        #read from arduino channel
        data = rl.readline()

        #report cycle speed
        try:
            print(str(1/(time.time()-tstart)))
        except:
            print('Too fast to track')