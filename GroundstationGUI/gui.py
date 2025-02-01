from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QComboBox, QGridLayout, QLabel, QPushButton, QWidget, QSlider, QLineEdit 
from PyQt6.QtGui import QMatrix4x4, QVector3D, QQuaternion
from PyQt6.QtCore import Qt, QTimer


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
        self.turn_label = QLabel()
        self.turn_label.setText(f"Overall turn setting: {self.turn_setting}")

        self.title_label = QLabel()
        self.title_label.setText("Dynamixel Motor Control")
        self.enable_motors_button = QPushButton()
        self.enable_motors_button.setText("ENABLE MOTORS")

        self.enable_motors_button.clicked.connect(self.toggleMotorEnabled)

        self.timer = QTimer()
        self.timer.timeout.connect(self.tick_enabled)  # Connect to the slot
        self.timer.start(100)  # Interval in milliseconds

        layout = QGridLayout()

        data_view_layout = QGridLayout()
        data_view_layout.addWidget(self.title_label, 1, 2)
        data_view_layout.addWidget(self.motor1_position_label, 2, 1)
        data_view_layout.addWidget(self.motor2_position_label, 2, 2)
        data_view_layout.addWidget(self.turn_label, 2, 3)
        data_view_layout.addWidget(self.increment_label, 3, 1)
        data_view_layout.addWidget(self.increment_box, 3, 2)
        data_view_layout.addWidget(self.controls_label, 4, 2)
        data_view_layout.addWidget(self.controls_box, 5, 2)
        data_view_layout.addWidget(self.enable_motors_button, 6, 2)

        layout.addLayout(data_view_layout, 1, 1)


        self.setLayout(layout)

        self.setMinimumWidth(250)
    def toggleMotorEnabled(self):
        if self.motor_run:
            self.motor_run = False
            self.enable_motors_button.setText("ENABLE MOTORS")
        else:
            self.motor_run = True
            self.enable_motors_button.setText("DISABLE MOTORS")

    def tick_enabled(self):
        if not self.motor_run:
            return
        
        self.motor1Position = max(0, min(DEFAULT_LEN, DEFAULT_LEN + self.turn_setting))
        self.motor2Position = max(0, min(DEFAULT_LEN, DEFAULT_LEN - self.turn_setting))
        self.motor_control_callback(self.motor1Position, self.motor2Position)
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
        self.show()
    
    def update_scene(self):
        """Update the 3D scene and perform other tasks."""

        x,y,z, w = read_serial_data(self.port)
        self.navball.set_orientation(
          q = (w, x, y, z)
        )
        pass