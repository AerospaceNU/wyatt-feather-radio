import struct
import time
import serial

import serial
import serial.tools.list_ports

from PyQt6.QtWidgets import QApplication, QMainWindow
import sys
import time
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QLabel, QPushButton, QWidget, QLineEdit 
from PyQt6.QtCore import Qt, QTimer

from config_manager import ConfigManager

motor_config = ConfigManager("motor_config.json")

CM_LENGTH_TO_DEGREES = -76.0
CAMERA_ON = 5
CAMERA_OFF = 6

def motor_inches_to_absolute_degrees(inches: float, motor_idx: int):
    offset_key = f"motor_offset_{motor_idx}"
    set_length_in_degrees = (inches * CM_LENGTH_TO_DEGREES)
    return set_length_in_degrees

def motor_degrees_to_absolute_inches(degrees: float, motor_idx: int):
    offset_key = f"motor_offset_{motor_idx}"
    set_length_degrees_minus_offset = degrees
    return set_length_degrees_minus_offset / CM_LENGTH_TO_DEGREES

def set_current_degrees_as_offset(degrees: float, motor_idx: int):
    offset_key = f"motor_offset_{motor_idx}"
    motor_config.set(offset_key, motor_config.get(offset_key, 0) + degrees)


DEFAULT_LEN = 15

class MotorControl(QWidget):
    def __init__(self, motor_control_callback, pyro_control_callback, parent_widget: QWidget = None ):
        super().__init__(parent_widget)

        self.motor_control_callback = motor_control_callback
        self.pyro_control_callback = pyro_control_callback
        self.motor_run = False
        self.increment_box = QLineEdit()
        self.increment = 0.5
        self.increment_box.setText("0.5")
        self.increment_box.textChanged.connect(self.setIncrement)
        self.camera_on = False

        self.turn_setting = 0
        self.motor1Position = DEFAULT_LEN
        self.motor2Position = DEFAULT_LEN

        self.increment_label = QLabel()
        self.increment_label.setText("Set Increment:")

        self.controls_box = QLineEdit()
        self.controls_box.setText("")
        self.controls_box.keyPressEvent = self.keyPressEvent


        self.controls_label = QLabel()
        self.controls_label.setText("Click in the below box to be able to use arrow keys")
        self.motor1_position_label = QLabel()
        self.motor1_position_label.setText(f"Motor 1: {self.motor1Position}")
        self.motor2_position_label = QLabel()
        self.motor2_position_label.setText(f"Motor 2: {self.motor2Position}")        
        self.motor1_abs_position_label = QLabel()
        self.motor1_abs_position_label.setText(f"Degrees: {motor_inches_to_absolute_degrees(self.motor1Position, 0)}")
        self.motor2_abs_position_label = QLabel()
        self.motor2_abs_position_label.setText(f"Degrees: {motor_inches_to_absolute_degrees(self.motor2Position, 1)}")
        self.turn_label = QLabel()
        self.turn_label.setText(f"Overall turn setting: {self.turn_setting}")
        self.camera_button = QPushButton();
        self.camera_button.setText("Turn on camera")
        self.camera_button.clicked.connect(self.cameraOn)
        self.camera_off_button = QPushButton();
        self.camera_off_button.setText("Turn off camera")
        self.camera_off_button.clicked.connect(self.cameraOff)


        self.motor1_absolute_label = QLabel()
        self.motor1_absolute_label.setText("Motor 1 Manual Degrees Control")
        self.motor1_degrees_edit = QLineEdit()
        self.motor1_degrees_edit.setText(f"{motor_inches_to_absolute_degrees(self.motor1Position, 0)}")
        self.motor1_degrees_edit.textChanged.connect(self.setMotor1AbsoluteDesired)
        self.motor1AbsoluteDesired = motor_inches_to_absolute_degrees(self.motor1Position, 0)
        self.setMotor1AbsoluteButton = QPushButton()
        self.setMotor1AbsoluteButton.setText("Manual Set Motor 1")
        self.setMotor1AbsoluteButton.clicked.connect(self.setMotor1Absolute)
        self.motor1ZeroButton = QPushButton()
        self.motor1ZeroButton.setText("Set Current Motor 1 Position as Zero Position")
        self.motor1ZeroButton.clicked.connect(self.zeroMotor1)

        self.motor2_absolute_label = QLabel()
        self.motor2_absolute_label.setText("Motor 2 Manual Degrees Control")
        self.motor2_degrees_edit = QLineEdit()
        self.motor2_degrees_edit.setText(f"{motor_inches_to_absolute_degrees(self.motor2Position, 1)}")
        self.motor2_degrees_edit.textChanged.connect(self.setMotor2AbsoluteDesired)
        self.motor2AbsoluteDesired = motor_inches_to_absolute_degrees(self.motor2Position, 1)
        self.setMotor2AbsoluteButton = QPushButton()
        self.setMotor2AbsoluteButton.setText("Manual Set Motor 2")
        self.setMotor2AbsoluteButton.clicked.connect(self.setMotor2Absolute)
        self.motor2ZeroButton = QPushButton()
        self.motor2ZeroButton.setText("Set Current Motor 2 Position as Zero Position")
        self.motor2ZeroButton.clicked.connect(self.zeroMotor2)

        self.title_label = QLabel()
        self.title_label.setText("Dynamixel Motor Control")
        self.enable_motors_button = QPushButton()
        self.enable_motors_button.setText("Enable Live Control")

        self.enable_pyro_control_button = QPushButton()
        self.enable_pyro_control_button.setText("Enable pyro control")
        self.pyro_enabled = False
        self.pyro_fire_button = QPushButton()
        self.pyro_fire_button.setText("Fire pyro! ")
        self.pyro_fire_button.setDisabled(True)
        self.pyro_fire_button.clicked.connect(self.start_pyro_countdown)
        self.enable_pyro_control_button.clicked.connect(self.enable_pyro_control)

        self.enable_motors_button.clicked.connect(self.toggleMotorEnabled)

        self.timer = QTimer()
        self.timer.timeout.connect(self.tick_enabled)  # Connect to the slot
        self.timer.start(100)  # Interval in milliseconds

        self.pyro_activate_timer = QTimer(self)
        self.pyro_activate_timer.setSingleShot(True)  # Ensures it runs only once
        self.pyro_activate_timer.timeout.connect(self.activatePyro)

        layout = QGridLayout()

        data_view_layout = QGridLayout()
        data_view_layout.addWidget(self.title_label, 1, 2)
        data_view_layout.addWidget(self.motor1_position_label, 2, 1)
        data_view_layout.addWidget(self.motor2_position_label, 2, 2)
        data_view_layout.addWidget(self.motor1_abs_position_label, 3, 1)
        data_view_layout.addWidget(self.motor2_abs_position_label, 3, 2)
        data_view_layout.addWidget(self.turn_label, 2, 3)
        data_view_layout.addWidget(self.increment_label, 4, 1)
        data_view_layout.addWidget(self.increment_box, 4, 2)
        data_view_layout.addWidget(self.controls_label, 5, 2)
        data_view_layout.addWidget(self.controls_box, 6, 2)
        data_view_layout.addWidget(self.enable_motors_button, 7, 2)
        data_view_layout.addWidget(self.motor1_absolute_label, 8, 1)
        data_view_layout.addWidget(self.motor1_degrees_edit, 9, 1)
        data_view_layout.addWidget(self.setMotor1AbsoluteButton, 10, 1)
        data_view_layout.addWidget(self.motor1ZeroButton, 11, 1)
        data_view_layout.addWidget(self.motor2_absolute_label, 8, 2)
        data_view_layout.addWidget(self.motor2_degrees_edit, 9, 2)
        data_view_layout.addWidget(self.setMotor2AbsoluteButton, 10, 2)
        data_view_layout.addWidget(self.motor2ZeroButton, 11, 2)
        data_view_layout.addWidget(self.camera_button, 12, 1)
        data_view_layout.addWidget(self.camera_off_button, 12, 2)
        data_view_layout.addWidget(self.enable_pyro_control_button, 13, 1)
        data_view_layout.addWidget(self.pyro_fire_button, 13, 2)
        layout.addLayout(data_view_layout, 1, 1)


        self.setLayout(layout)

        self.setMinimumWidth(250)

    def start_pyro_countdown(self):
        self.pyro_activate_timer.start(3000)

    def enable_pyro_control(self):
        if self.pyro_enabled:
            if self.pyro_activate_timer.isActive():
                self.pyro_activate_timer.stop()
            self.pyro_enabled = False
            self.pyro_fire_button.setDisabled(True)
        else:
            self.pyro_enabled = True
            self.pyro_fire_button.setDisabled(False)
            self.enable_pyro_control_button.setText("Disable pyro control")

    

    def cameraOn(self):
        self.pyro_control_callback(CAMERA_ON)
    def cameraOff(self):
        self.pyro_control_callback(CAMERA_OFF)

    def activatePyro(self):
        self.pyro_control_callback(12)


    def zeroMotor1(self):
        current_degrees = motor_inches_to_absolute_degrees(self.motor1Position, 0)
        set_current_degrees_as_offset(current_degrees, 0)
        self.motor1Position = motor_degrees_to_absolute_inches(current_degrees, 0)
        self.update()

    def zeroMotor2(self):
        current_degrees = motor_inches_to_absolute_degrees(self.motor2Position, 1)
        set_current_degrees_as_offset(current_degrees, 1)
        self.motor2Position = motor_degrees_to_absolute_inches(current_degrees, 1)
        self.update()

    def toggleMotorEnabled(self):
        if self.motor_run:
            self.motor_run = False
            self.enable_motors_button.setText("Enable Live Control")
        else:
            self.motor_run = True
            self.enable_motors_button.setText("Disable Live Control")

    def tick_enabled(self):
        if not self.motor_run:
            return
        
        self.motor1Position = max(0, min(DEFAULT_LEN, DEFAULT_LEN + self.turn_setting))
        self.motor2Position = max(0, min(DEFAULT_LEN, DEFAULT_LEN - self.turn_setting))
        self.motor_control_callback(motor_inches_to_absolute_degrees(self.motor1Position, 0), motor_inches_to_absolute_degrees(self.motor2Position, 1))
        self.update()
        

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Right:
            self.turn_setting += self.increment
        elif event.key() == Qt.Key.Key_Left:
            self.turn_setting -= self.increment
        self.update()


    def update(self):
        self.motor1_position_label.setText(f"Motor 1: {self.motor1Position}")
        self.motor2_position_label.setText(f"Motor 2: {self.motor2Position}")
        self.motor1_abs_position_label.setText(f"Degrees: {motor_inches_to_absolute_degrees(self.motor1Position, 0)}")
        self.motor2_abs_position_label.setText(f"Degrees: {motor_inches_to_absolute_degrees(self.motor2Position, 1)}")
        self.turn_label.setText(f"Overall turn setting: {self.turn_setting}")

        super().update()

    def decrementMotor(self, motor_index):
        self.setSliderPosition(motor_index, self.slider_values[motor_index]-self.increment)

        
    def incrementMotor(self, motor_index):
        self.setSliderPosition(motor_index, self.slider_values[motor_index]+self.increment)


    def setIncrement(self, increment):
        try:
            increment = float(increment)
        except:
            return
        self.increment = increment

    def setMotor1Absolute(self):
        try:
            degrees = self.motor1AbsoluteDesired
            self.motor1Position = motor_degrees_to_absolute_inches(degrees, 0)
            self.motor_control_callback(self.motor1AbsoluteDesired, motor_inches_to_absolute_degrees(self.motor2Position, 1))
            self.update()
        except:
            return
        
    def setMotor2Absolute(self):
        try:
            degrees = self.motor2AbsoluteDesired
            self.motor2Position = motor_degrees_to_absolute_inches(degrees, 1)
            self.motor_control_callback(motor_inches_to_absolute_degrees(self.motor1Position, 0), self.motor2AbsoluteDesired)
            self.update()
        except:
            return
    

        
    def setMotor2AbsoluteDesired(self, degrees):
        try:
            self.motor2AbsoluteDesired = float(degrees)
        except:
            return
    
    def setMotor1AbsoluteDesired(self, degrees):
        try:
            self.motor1AbsoluteDesired = float(degrees)
        except:
            return
        

    def setSliderPosition(self, slider_index, value):
        try:
            value = int(value)
        except:
            return
        self.slider_values[slider_index] = value
        self.motor_sliders[slider_index].setValue(value)
        self.slider_labels[slider_index].setText(f"Motor {slider_index}")
        self.degrees_box[slider_index].setText(str(value))
        self.update()

    def updateData(self, vehicle_data, updated_data):
        self.update()

    def buttonPressed(self, button_number):
        self.motor_control_callback(self.motor1Position, self.motor2Position)
        self.update()


class MainWindow(QMainWindow):
    def __init__(self, port, motor_control_callback, pyro_control_callback):
        super().__init__()
        self.setWindowTitle("WYATTTTTTT")
        self.motorcontrol = MotorControl(motor_control_callback, pyro_control_callback, self)
        self.setCentralWidget(self.motorcontrol)
        
        # Timer for updating
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_scene)
        self.timer.start(16)  # Approx 60 FPS
        self.port = port
        self.last_sent_time = time.time()
        self.show()
    
    def update_scene(self):
        """Update the 3D scene and perform other tasks."""

        motor_val = 0
        receive_packet(ser)
        self.motorcontrol.update()
        time.sleep(0.05)  # Short delay to avoid excessive CPU usage

            


def get_crc(crc_accum: int, data_blk: bytes) -> int:
    """Computes CRC-16 using the lookup table method, matching the Arduino implementation."""
    crc_table = [
      0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033,
      0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066,
      0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F,
      0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
      0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB,
      0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE,
      0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087,
      0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0,
      0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6,
      0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
      0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
      0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138,
      0x813D, 0x8137, 0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
      0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317,
      0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353,
      0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
      0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3, 0x03F6, 0x03FC,
      0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
      0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B,
      0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E,
      0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7,
      0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
      0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275,
      0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261, 0x0220, 0x8225, 0x822F,
      0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
      0x0208, 0x820D, 0x8207, 0x0202]
    for byte in data_blk:
        i = ((0xFFFF & (crc_accum >> 8)) ^ byte) & 0xFF
        crc_accum = (crc_accum << 8) ^ crc_table[i]
    
    return crc_accum & 0xFFFF

def list_serial_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def select_serial_port(ports):
    """Ask user to select a serial port."""
    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port}")

    while True:
        try:
            index = int(input("Select a port by number: "))
            if 0 <= index < len(ports):
                return ports[index]
        except ValueError:
            pass
        print("Invalid selection. Try again.")

def read_serial_data(port, baudrate=115200):
    """Read data continuously from the selected serial port."""
    try:
        with serial.Serial(port, baudrate) as ser:
            print(f"Reading from {port}... (Press Ctrl+C to stop)")
            data = ser.read(16)
            print(len(data))
            x, y, z, w = struct.unpack('ffff', data)
            return (x,y,z, w)
    except serial.SerialException as e:
        print(f"Serial error: {e}")

PYRO_DUP_COUNT = 10

# Fixed struct formats
RADIO_PACKET_FORMAT = "<IIB40sH"  # timestamp_ms, payload_length, packet_type, payload (40 bytes), crc
GROUNDSTATION_PACKET_FORMAT = "<Id51s"  # timestamp_groundstation, rssi, payload (51 bytes)

ECHO_PACKET_FORMAT = "<ff"  # 2 floats
MOTOR_CONTROL_PACKET_FORMAT = "<ff"  # 2 floats
PYRO_TRIGGER_PACKET_FORMAT = "<" + "I" * PYRO_DUP_COUNT  # 10 uint32_t

PACKET_TYPE_MAP = {0: "Echo", 1: "MotorControl", 2: "PyroTrigger"}

def parse_payload(packet_type, payload_data):
    """Parses the payload based on the packet type."""
    if packet_type == 0:  # EchoPacket
        motor1, motor2 = struct.unpack(ECHO_PACKET_FORMAT, payload_data[:8])
        return {"currentMotor1SettingAbsoluteDegrees": motor1, "currentMotor2SettingAbsoluteDegrees": motor2}
    
    elif packet_type == 1:  # MotorControlPacket
        motor1, motor2 = struct.unpack(MOTOR_CONTROL_PACKET_FORMAT, payload_data[:8])
        return {"motor1SettingAbsoluteDegrees": motor1, "motor2SettingAbsoluteDegrees": motor2}
    
    elif packet_type == 2:  # PyroTriggerPacket
        pyro_values = struct.unpack(PYRO_TRIGGER_PACKET_FORMAT, payload_data[:40])
        return {"pyroToTrigger": list(pyro_values)}

    return {"raw_data": payload_data}

def parse_ground_station_packet(data):
    """Parses the full GroundStationPacket from binary data."""
    if len(data) < struct.calcsize(GROUNDSTATION_PACKET_FORMAT):
        raise ValueError("Incomplete packet received.")

    timestamp_groundstation, rssi, radio_payload = struct.unpack(GROUNDSTATION_PACKET_FORMAT, data)

    timestamp_ms, payload_length, packet_type, payload_data, crc = struct.unpack(RADIO_PACKET_FORMAT, radio_payload)

    parsed_payload = parse_payload(packet_type, payload_data) if packet_type in PACKET_TYPE_MAP else {"unknown": True}

    return {
        "timestamp_groundstation": timestamp_groundstation,
        "rssi": rssi,
        "radio_packet": {
            "timestamp_ms": timestamp_ms,
            "payload_length": payload_length,
            "packet_type": PACKET_TYPE_MAP.get(packet_type, "Unknown"),
            "payload": parsed_payload,
            "crc": crc,
        }
    }

def receive_packet(ser):
    """Non-blocking function to receive a GroundStationPacket if available."""
    packet_size = struct.calcsize(GROUNDSTATION_PACKET_FORMAT)
    
    if ser.in_waiting >= packet_size:  # Check if enough data is available
        raw_data = ser.read(packet_size)
        try:
            parsed_packet = parse_ground_station_packet(raw_data)
            print("Received Packet:", parsed_packet)
        except ValueError:
            print("Error parsing packet.")

def send_motor_command(ser, motor1, motor2):
    """Send a MotorControlPacket to the serial port."""
    packet_type = 1  # MotorControl
    offset_key_0 = f"motor_offset_{0}"
    offset_key_1 = f"motor_offset_{1}"
    degrees_offset_0 = motor_config.get(offset_key_0, default=0)
    motor1 = degrees_offset_0 + motor1
    degrees_offset_1 = motor_config.get(offset_key_1, default=0)
    motor2 = degrees_offset_1 + motor2
    payload = struct.pack(MOTOR_CONTROL_PACKET_FORMAT, motor1, motor2)
    
    timestamp_ms = 300
    payload_length = len(payload)
    crc = 0  # Placeholder, replace with actual CRC calculation if needed
    
    radio_packet = struct.pack(RADIO_PACKET_FORMAT, timestamp_ms, payload_length, packet_type, payload, crc)
    crc = get_crc(0, radio_packet[:struct.calcsize(RADIO_PACKET_FORMAT) - 2])
    radio_packet = struct.pack(RADIO_PACKET_FORMAT, timestamp_ms, payload_length, packet_type, payload, crc)

    
    timestamp_groundstation = 300
    rssi = -50.0  # Example RSSI value
    ground_station_packet = struct.pack(GROUNDSTATION_PACKET_FORMAT, timestamp_groundstation, rssi, radio_packet)
    
    ser.write(ground_station_packet)
    print(f"Sent MotorControlPacket: motor1={motor1}, motor2={motor1}")

def send_pyro_command(ser, pyro_pin=12):
    """Send a PyroTrigger to the serial port."""
    packet_type = 2  # PyroTrigger
    vals = (pyro_pin,) * (PYRO_DUP_COUNT)
    payload = struct.pack(PYRO_TRIGGER_PACKET_FORMAT, *vals)
    
    timestamp_ms = 300
    payload_length = len(payload)
    crc = 0  # Placeholder, replace with actual CRC calculation if needed
    
    radio_packet = struct.pack(RADIO_PACKET_FORMAT, timestamp_ms, payload_length, packet_type, payload, crc)
    crc = get_crc(0, radio_packet[:struct.calcsize(RADIO_PACKET_FORMAT) - 2])
    radio_packet = struct.pack(RADIO_PACKET_FORMAT, timestamp_ms, payload_length, packet_type, payload, crc)

    
    timestamp_groundstation = 300
    rssi = -50.0  # Example RSSI value
    ground_station_packet = struct.pack(GROUNDSTATION_PACKET_FORMAT, timestamp_groundstation, rssi, radio_packet)
    
    ser.write(ground_station_packet)
    print(f"Sent PyroTrigger: pin = {pyro_pin}")


if __name__ == "__main__":
    ports = list_serial_ports()
    if not ports:
        print("No serial devices found.")
    else:
        selected_port = select_serial_port(ports)
    
    ser = serial.Serial(selected_port, baudrate=115200, timeout=0.1)
    app = QApplication(sys.argv)
    window = MainWindow(ser, lambda x, y: send_motor_command(ser, x, y), lambda p: send_pyro_command(ser, p))
    sys.exit(app.exec())
    