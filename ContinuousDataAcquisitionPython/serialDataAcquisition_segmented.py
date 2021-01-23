import serial
import datetime;
from time import sleep, time
import csv
import pandas as pd
import numpy as np

columnName = ['flex_1', 'flex_2', 'flex_3', 'flex_4', 'flex_5',
              'Qw', 'Qx', 'Qy', 'Qz',
              'GYRx', 'GYRy','GYRz',
              'ACCx', 'ACCy', 'ACCz',
              'ACCx_real', 'ACCy_real', 'ACCz_real',
              'ACCx_world', 'ACCy_world', 'ACCz_world',
              'GRAx', 'GRAy', 'GRAz',
              'ACCx_raw', 'ACCy_raw', 'ACCz_raw',
              'GYRx_raw', 'GYRy_raw', 'GYRz_raw']

data = []

recorditeration = 2
segmentLength = 180
gestureName = 'deaf'
port = '/dev/ttyUSB0'

ser = serial.Serial(port=port,
                    baudrate=115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1,
                    xonxoff=0,
                    rtscts=0)

ser.dtr = False
sleep(1)
ser.reset_input_buffer()
ser.dtr = True

count = 0
recordData = True
interruptToken = False

print("Preparing Device... Please Wait")


while True:
    try:
        current_time = datetime.datetime.now()
        values = ser.readline().decode('utf-8').rstrip().split(',')
        values = list(map(float, values))
        # values = list(current_time) + values

        if(recordData == True):
            interruptToken = False
            data.append(values)
            print("\nData Received... index: ", len(data)-1, end='')

        if ((len(data) % segmentLength) == 0) and (recordData == True) and (interruptToken == False):
            if recordData == True:
                recordData = False
                interruptToken = True
                count += 1
                print('\tSegment - ', count, ' finished')
            if count > recorditeration:
                df = pd.DataFrame(data,columns=columnName)
                df.to_csv(gestureName + '.csv', index=False)
                print("\nData Writing Done... shape: ", df.shape)
                break


    except(KeyboardInterrupt):
        if recordData == False:
            print("\n\nStart")
            recordData = True

    except(ValueError):
        print(end='.')

    except:
        print("Something's Wrong... CHECK!!!")
