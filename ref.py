# NOTE: only work on micropython 1.12.x 
from m5stack import *
from m5ui import *
from uiflow import *
from libs.m5_espnow import M5ESPNOW
from kalman_filter import *
import unit

setattr(unit, 'PORT_ABC_A', (25, 21))
setattr(unit, 'PORT_ABC_B', (23, 33))
setattr(unit, 'PORT_ABC_C', (22, 19))

color_map = {
    'red': 0xFF0000,
    'green': 0x00FF00,
    'blue': 0x0000FF,
    'white': 0xFFFFFF,
    'black': 0x000000,
    'yellow': 0xFFFF00,
    'cyan': 0x00FFFF,
    'magenta': 0xFF00FF,
    'gray': 0x808080,
    'orange': 0xFF6E09,
}

DEVICE_MODE = ""

TICK_100MS = 100
TICK_200MS = 200
TICK_500MS = 500
TICK_1S = 1000
TICK_2S = 2000
TICK_12S = 12000
TICK_30S = 30000

FSM_TICKS = TICK_500MS  # 500ms
SENSOR_TIMEOUT = 6*TICK_1S   # 6s

uart = None
now = M5ESPNOW()

# Setup
def setup_connect():
    global uart, now
    now.espnow_init(1)
    uart = machine.UART(2, tx=1, rx=3)
    uart.init(115200, bits=8, parity=None, stop=1)

def printf(*args, sep='', end="\n", mode=""):
    global uart, now, DEVICE_MODE
    output = ''
    for arg in args:
        output += str(arg) + sep
    output += end
    # Write the output to the UART

    if DEVICE_MODE == "master":
        uart.write(output)
    else:
        now.espnow_broadcast_data(output)

class State:
    IDLE = 300
    DETECTED = 301
    COLLECTING = 302
    FILTERING = 303
    PROCESSING = 304
    ERROR = 305

class Sensor:
    def __init__(self):
        self.device = None
        self.key = self.__class__.__name__[0]
        self.collect_timeout = SENSOR_TIMEOUT
        self.collecting_value = [[]]
        self.idle_value = [0,0,0,0]
        self.state = State.IDLE
    
    def set_state(self, state):
        self.state = state

    def get_state(self):
        return self.state

    def _create(self):
        pass

    def create(self):
        try:
            self._create()
            self.set_state(State.IDLE)
            return True
        except Exception as e:
            return False

    def idle_collect(self):
        self.collecting_value = []
        self.collect_timeout = SENSOR_TIMEOUT

    def pre_collect(self):
        self.idle_value = [0,0,0,0]
        self.collect_timeout = SENSOR_TIMEOUT/4

    def detect_collect(self):
        pass

    def collect_data(self):
        pass

    def filter_data(self):
        value = kalman_filter(self.collecting_value)
        return value

    def check_available(self):
        """
        Try to create the device, then check if the device is available
        Returns:
            bool: True if the device is available, otherwise False"""
        if self.device is None:
            response = self.create()
            if not response:
                return False
        try:       
            self.device._available()
        except Exception as error:
            return False
        return True
        
    def stop(self):
        self.set_state(State.FILTERING)

class HeartRateSpo2(Sensor):
    """
    setLedCurrent(red, Ir) 0/4.4/7.6/11/14.2/17.4/20.8/24/27.1/30.6/33.8/37/40.2/43.6/46.8/50
    setPlusWidth() 0-3, 200/400/800/1600us
    setSamplingRate() 0-8, 50/100/167/200/400/600/800/1000Hz
    Mode: 0x02: Heart Rate Only, 0x03: +SpO2 mode
    """
    IR = RED = {
    "0": 0x00, "4.4": 0x01,
    "7.6": 0x02, "11": 0x03,
    "14.2": 0x04, "17.4": 0x05,
    "20.8": 0x06, "24": 0x07,
    "27.1": 0x08, "30.6": 0x09,
    "33.8": 0x0A, "37": 0x0B,
    "40.2": 0x0C, "43.6": 0x0D,
    "46.8": 0x0E, "50": 0x0F
    }

    def __init__(self):
        super().__init__()
        self.threshold = 2000

    def _create(self):
        self.device = unit.get(unit.HEART, unit.PORTA)
        self.device.setLedCurrent(self.RED['4.4'], self.IR['4.4'])
        self.device.setMode(0x03)
        
    def idle_collect(self):
        self.collecting_value = []
        self.device.getSpO2()
        value = self.get_ir()
        if value > self.threshold and value < 5000:
            self.device.setLedCurrent(self.RED['50'], self.IR['50'])
        else:
            self.device.setLedCurrent(self.RED['0'], self.IR['4.4'])
        self.idle_value.insert(0, value)
        self.idle_value = self.idle_value[:-1]
        last_value = self.idle_value[-1]
        if last_value != 0:
            if self.idle_value[0] - last_value > self.threshold - last_value and self.idle_value[1] - last_value > self.threshold and self.idle_value[2] > self.threshold:
                printf("!{}:DETECTED#".format(self.key),end="")
                self.set_state(State.DETECTED)

    def pre_collect(self):
        self.idle_value = [0,0,0,0]
        self.collect_timeout = 20*TICK_1S

    def collect_data(self):
        if not self.check_available():
            self.set_state(State.ERROR)
            return
        value = self.get_ir()
        if value > self.threshold:
            heart_rate = self.device.getHeartRate()
            spo2 = self.device.getSpO2()
            printf(heart_rate, spo2)
            if 88 < spo2 <= 100 and 50 < heart_rate <= 130:
                self.collecting_value.append([heart_rate, spo2])

    def filter_data(self):
        value = super().filter_data()
        if len(value) > 0 and 50 <= value[0] <= 130 and 94 <= value[1] <= 100:
            pass
        else:
            value = [None, None]
        return value

    def check_available(self):
        return super().check_available()

    def get_ir(self):
        return self.device.getIr()

    def get_red(self):
        return self.device.getRed()
    
    def stop(self):
        super().stop()

class TempNCIR(Sensor):
    
    def _create(self):
        self.device = unit.get(unit.NCIR, unit.PORTA)
        
    def idle_collect(self):
        super().idle_collect()
        try:
            value = self.device.temperature
            self.idle_value.insert(0, round(value,3))
            threshold = 3   
            self.idle_value = self.idle_value[:-1]
            last_value = self.idle_value[-1]
            if last_value != 0:
                if self.idle_value[0] - last_value > threshold and self.idle_value[1] - last_value > threshold and self.idle_value[2] - last_value > threshold:
                    printf("!{}:DETECTED#".format(self.key),end="")
                    self.set_state(State.DETECTED)
        except Exception as e:
            self.set_state(State.ERROR)
            pass

    def collect_data(self):
        if not self.check_available():
            self.set_state(State.ERROR)
            return
        value = self.device.temperature
        if 20 <= value <= 50:
            self.collecting_value.append([value])

    def check_available(self):
        '''
        Try to create the device, then check if the device is available
        Returns:
            bool: True if the device is available, otherwise False
        '''     
        return super().check_available()

    def stop(self):
        super().stop()

class Weight(Sensor):

    def _create(self):
        self.device = unit.get(unit.WEIGHT, unit.PORTA)
        wait(1)
        self.zero()  
            
    def check_available(self):
        '''
        Try to create the device, then check if the device is available
        Returns:
            bool: True if the device is available, otherwise False
        ''' 
        if self.device is None:
            response = self.create()
            if not response:
                return False    
        a = self.device.rawData
        b = self.device.rawData
        c = self.device.rawData
        if a == b == c:
            return False
        return True

    def idle_collect(self):
        super().idle_collect()
        try:
            value = self.device.weight / 100
            self.idle_value.insert(0, value)
            self.idle_value = self.idle_value[:-1]
            last_value = self.idle_value[-1]
            threshold = 2
            if last_value != 0:
                if self.idle_value[0] - last_value > threshold and self.idle_value[1] - last_value > threshold and self.idle_value[2] - last_value > threshold:
                    printf("!{}:DETECTED#".format(self.key),end="")
                    self.set_state(State.DETECTED)
        except Exception as e:
            self.set_state(State.ERROR)

    def collect_data(self):
        if not self.check_available():
            self.set_state(State.ERROR)
            return
        value = self.device.weight / 100    
        if 0 <= value <= 200:
            self.collecting_value.append([value])

    def filter_data(self):
        # self.collecting_value = [[1],[2],[3],[4],[5]]
        sum_value = sum(element[0] for element in self.collecting_value)
        list_length = len(self.collecting_value) if len(self.collecting_value) != 0 else 1
        value = [sum_value/list_length]
        return value

    def zero(self):
        self.device.zero()

class HeightSonar(Sensor):
    # default {'echo': Pin(32), 'trigger': Pin(26), 'echo_timeout_us': 1000000}
    
    def __init__(self):
        self.height_length = 214
        super().__init__()

    def _create(self):
        self.device = unit.get(unit.SONIC_IO, unit.PORT_ABC_A)

    # def idle_collect(self):
    #     super().idle_collect()
    #     try:
    #         value = self.height_length - self.device.get_distance(2)
    #         self.idle_value.insert(0, round(value,3))
    #         threshold = 10
    #         self.idle_value = self.idle_value[:-1]
    #         last_value = self.idle_value[-1]
    #         if last_value != 0:
    #             if self.idle_value[0] - last_value > threshold and self.idle_value[1] - last_value > threshold and self.idle_value[2] - last_value > threshold:
    #                 printf("!{}:DETECTED#".format(self.key),end="")
    #                 self.set_state(State.DETECTED)
    #     except Exception as e:
    #         self.set_state(State.ERROR)
    #         pass

    def collect_data(self):
        if not self.check_available():
            self.set_state(State.ERROR)
            return
        value = self.height_length - self.device.get_distance(2)
        if 20 <= value <= self.height_length:
            self.collecting_value.append([round(value,3)])

    def detect_collect(self):
        printf("!{}:DETECTED#".format(self.key),end="")
        self.set_state(State.DETECTED)

    def check_available(self):
        '''
        Try to create the device, then check if the device is available
        Returns:
            bool: True if the device is available, otherwise False
        ''' 
        # because there is not _available method in sonar 
        if self.device is None:
            response = self.create()
            if not response:
                return False
        for i in range(5):
            if self.device.get_distance(2) <= 0:
                return False
        return True

class HumanDetectPIR(Sensor):

    def __init__(self):
        self.old_value = 0
        self.rgb = None
        super().__init__()

    def _create(self):
        self.device = unit.get(unit.PIR, (22, 19))
        self.rgb = unit.get(unit.NEOPIXEL, (23, 33), 37)
        self.set_color('orange')
        self.turn_on = False
        self.set_brightness(0)

    def check_available(self):
        if self.device is None:
            return self.create()
        return True

    def idle_collect(self):
        super().idle_collect()
        value = self.device.state
        if self.turn_on:
            self.set_brightness(0)
            self.turn_on = False
        if value == 1:
            self.turn_on = True
            self.set_brightness(120)
            self.set_state(State.PROCESSING)

    def set_color(self, color_key):
        global color_map
        try:
            color = color_map.get(color_key, 0xFF0000)
            self.rgb.setColorAll(color)
        except Exception as e:
            printf(e)

    def set_brightness(self, brightness):
        self.rgb.setBrightness(brightness)

    def stop(self):
        self.set_state(State.IDLE)

class ECG(Sensor):
    pass

class BloodPressure(Sensor):
    pass

class ThermalMap(Sensor):
    pass

@timerSch.event('_heartbeat_timer') 
def t_heartbeat_timer():
    '''
    Send heartbeat to the server
    '''
    global sensor_data
    printf("!{}:live#".format("".join(opcode_dict.keys())), end="")
    for sensor in sensor_data:
        response = sensor.check_available()
        if response and sensor.get_state() == State.ERROR:
            printf("!{}:OK#".format(sensor.key),end="")
            sensor.set_state(State.IDLE)
        if not response:
            printf("!{}:ERR#".format(sensor.key),end="")
            sensor.set_state(State.ERROR)

@timerSch.event('_listen_uart')
def uart_call_back():
    global opcode_dict, uart
    uart_packet = uart.readline()
    if uart_packet:
        now.espnow_broadcast_data(uart_packet)
        decode_packet(uart_packet)

def decode_packet(packet):
    global DEVICE_MODE, opcode_dict
    packet = str(packet, 'utf-8')
    if packet.startswith("!") and packet.endswith("#"):
        opcode = packet[1:-1].split(":")[0]
        value = packet[1:-1].split(":")[-1]
        if opcode in opcode_dict and opcode_dict[opcode].get_state() != State.ERROR:
            if value == "START":
                opcode_dict[opcode].pre_collect()
                opcode_dict[opcode].state = State.COLLECTING
            elif value == "STOP":
                opcode_dict[opcode].state = State.FILTERING
            elif opcode == "LEAVE":
                for sensor in sensor_data:
                    sensor.set_state(State.IDLE)
            elif opcode == "MASTER" or value == "MASTER":           
                if DEVICE_MODE is "master":
                    DEVICE_MODE = "slave"
                    color_device_mode(DEVICE_MODE)
        elif value == "DETECTED":
            for sensor in sensor_data:
                if sensor.get_state != State.ERROR:
                    sensor.detect_collect()
        else:
            pass

def recv_cb(dummy):
    mac, data = now.espnow_recv_str()
    if DEVICE_MODE == "master":
        uart.write(data)
    decode_packet(data)
    
last_button_value = False
last_last_button_value = False
@timerSch.event('_button_reading')
def button_reading():
    global last_button_value, last_last_button_value
    button_value = btnA.isPressed()
    if button_value and last_button_value and last_last_button_value:
        buttonA_LongPress()
    last_last_button_value = last_button_value
    last_button_value = button_value

def buttonA_LongPress():
    global DEVICE_MODE
    if DEVICE_MODE is "master":
        DEVICE_MODE = "slave"
    else:
        DEVICE_MODE = "master"
        now.espnow_broadcast_data("!MASTER#")
    color_device_mode(DEVICE_MODE)

def color_device_mode(mode):
    if mode == "master":
        rgb.setColorAll(color_map.get('yellow', 0xffffff))
    else:
        rgb.setColorAll(color_map.get('magneta', 0xffffff))

@timerSch.event('_fsm_run') 
def fsm_run():
    for sensor in sensor_data:
        execute_device_state(sensor)

def execute_device_state(sensor):
    state = sensor.get_state()
    if state == State.IDLE:
        sensor.idle_collect()
    elif state == State.DETECTED:
        sensor.pre_collect()
        sensor.set_state(State.COLLECTING)
    elif state == State.COLLECTING:
        if sensor.collect_timeout <= 0:
            sensor.set_state(State.FILTERING)
        elif sensor.collect_timeout >= SENSOR_TIMEOUT/2:
            if sensor.collect_timeout % (2*FSM_TICKS) == 0:
                sensor.collect_data()
        else:
            sensor.collect_data()
        sensor.collect_timeout -= FSM_TICKS
    elif state == State.PROCESSING:
        if sensor.collect_timeout <= 0:
            sensor.stop()
        sensor.collect_timeout -= FSM_TICKS
    elif state == State.FILTERING:
        filtered_value = sensor.filter_data()
        printf("!{}:{}#".format(sensor.key, str(filtered_value)),end="")
        sensor.set_state(State.IDLE)
    elif state == State.ERROR:
        pass
    else:
        sensor.set_state(State.IDLE)

# ------------------ Main -------------------
def execute_program(*args, **kwargs):
    setup_connect()
    if sensor_data:
        for sensor in sensor_data:
            opcode_dict.update({sensor.key: sensor})
    else:
        printf("Empty Sensor Data List")

    printf("Starting...")

    #  delay 100MS then initialize the device
    timerSch.run('_heartbeat_timer', TICK_100MS, 0x01) 
    timerSch.run('_listen_uart', TICK_200MS, 0x00)
    timerSch.run('_fsm_run', FSM_TICKS, 0x00)
    timerSch.run('_button_reading', TICK_1S, 0x00)
    timerSch.setTimer('_heartbeat_timer', TICK_30S, 0x00) 
    now.espnow_recv_cb(recv_cb)

    color_device_mode(DEVICE_MODE)

heart_rate = HeartRateSpo2()
temp = TempNCIR()
weight = Weight()
lenght = HeightSonar()
pir = HumanDetectPIR()

opcode_dict = {}
sensor_data = [
    # heart_rate,
    # temp,
    # weight,
    # lenght,
    # pir,
]
# Add more sensor data list as needed
# DEVICE_MODE = "master"
execute_program(sensor_data)