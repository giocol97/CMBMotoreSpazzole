import serial
import json
import eel

serialPort = "COM18"
connect = True
serialStarted = False


@eel.expose
def start_drive():
    ser.write(b'start\n')


@eel.expose
def reset_drive():
    ser.write(b'reset\n')


@eel.expose
def send_set_packet(txt):
    ser.write(txt.encode('utf-8')+'\n'.encode('utf-8'))

# connect to serial port


@eel.expose
def connect_serial(port=""):
    global connect, serialPort
    connect = True
    #serialPort = port

# disconnect from serial port


@eel.expose
def disconnect_serial():
    global connect
    connect = False


def serial_thread():
    global ser
    global serialPort, connect, serialStarted
    ser = serial.Serial(serialPort, 115200, timeout=1)

    serialStarted = True

    while True:
        eel.sleep(0.001)

        if connect:
            if(not serialStarted):
                ser.open()
                serialStarted = True

            line = ser.readline()
            if line:
                jsonData = line.decode('utf-8')
                print(line.decode('utf-8'))
                try:
                    y = json.loads(jsonData)  # check if valid json

                    eel.showData(jsonData)
                    #plt.plot(xpoints, ypoints)
                    # plt.show()

                except:
                    # print(jsonData)
                    eel.appendToLog(jsonData)
        else:
            if(serialStarted):
                ser.close()
                serialStarted = False


eel.init('web')

eel.spawn(serial_thread)
eel.start('index.html', size=(1536, 800))
