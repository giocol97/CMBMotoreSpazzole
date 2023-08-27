import serial
import json
import eel
import sys
import glob


serialPort = ""
connect = False
serialStarted = False


@eel.expose
def start_drive():
    ser.write(b'start\n')

@eel.expose
def stop_drive():
    ser.write(b'stop\n')

@eel.expose
def start_config():
    ser.write(b'config0\n')

@eel.expose
def end_config():
    ser.write(b'config1\n')

@eel.expose
def start_contTest():
    ser.write(b'contTest0\n')

@eel.expose
def end_contTest():
    ser.write(b'contTest1\n')

@eel.expose
def reset_drive():
    ser.write(b'reset\n')


@eel.expose
def send_set_packet(txt):
    ser.write(txt.encode('utf-8')+'\n'.encode('utf-8'))

@eel.expose
def get_data():
    ser.write(b'Get;\n')

@eel.expose
def connect_serial(port=""):
    global connect, serialPort
    serialPort = port
    connect = True

# disconnect from serial port


@eel.expose
def disconnect_serial():
    global connect
    connect = False


@eel.expose
def get_available_ports():
    available_ports = serial_ports()
    return json.dumps(available_ports)


def serial_ports():
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def serial_thread():
    global ser
    global serialPort, connect, serialStarted

    ports = serial_ports()

    if len(ports) > 0:
        serialPort = ports[0]

    #ser = serial.Serial(serialPort, 115200, timeout=1)

    #serialStarted = True

    while True:
        eel.sleep(0.001)

        if connect:
            if(not serialStarted):
                ser = serial.Serial(serialPort, 115200, timeout=1)
                # ser.open()
                ser.write(b'TYPE;\n')
                eel.showConnected(serialPort)
                serialStarted = True

            line = ser.readline()
            if line:

                try:
                    jsonData = line.decode('utf-8')
                    print(line.decode('utf-8'))
                except:
                    jsonData = ""
                    print("decode error")

                try:
                    y = json.loads(jsonData)  # check if valid json

                    if 'type' in y:
                        eel.setType(y['type'])
                        eel.sleep(0.05)
                        ser.write(b'sendData1;\n')
                    else:
                        eel.showData(jsonData)

                except:
                    eel.appendToLog(jsonData)
        else:
            if(serialStarted):
                ser.close()
                eel.showDisconnected()
                serialStarted = False


eel.init('web')

eel.spawn(serial_thread)

eel.start('index.html', size=(1536, 800))
