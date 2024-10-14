import time
import os
import sys
sys.path.insert(0, '.')
import subprocess
import psutil

from robot_hat import Pin, ADC, PWM, Servo, Ultrasonic, fileDB, utils, Grayscale_Module


def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))


class Picarx(object):
    _instance = None
    
    
    CONFIG = '/opt/picar-x/picar-x.conf'
    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]
    DIR_MIN = -35
    DIR_MAX = 35
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 100
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65
    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    DIRECITONS = ['W', 'N', 'E', 'S']
    FORWARD_SPEED = 8
    FORWARD_TIME = 0.4
    TURN_SPEED = 30
    TURN_TIME = 2

    

    # singleton design pattern
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(Picarx, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance


    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: trig, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P13', 'P12'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG):

        if self._initialized:
            return
        self._initialized = True

        # reset robot_hat
        utils.reset_mcu()
        time.sleep(0.2)
        self.dist_trav = 0  # in cm
        self.direction = 'S'
        self.status = 'stop'
        self.move_start_time = None

        # --------- config_flie ---------
        self.config_flie = fileDB(config, 777, os.getlogin())

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

       
        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        trig, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(trig), Pin(echo, mode=Pin.IN, pull=Pin.PULL_DOWN))
    
         # Compatibility layer with picar-4wd
    def get_distance_at(self,angle):
        print("Angle: ", angle)
        self.set_cam_pan_angle(angle)
        time.sleep(0.1)
        distance = self.get_distance()        
        return distance


    def update_distance(self):
        if self.status in ['forward', 'backward']:
            self.dist_trav += round(10*(time.time() - self.move_start_time)/self.FORWARD_TIME, 2)
            self.move_start_time = time.time()
        return self.dist_trav
    

    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # print(f"direction: {direction}, speed: {speed}")
        if speed != 0:
            speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def backward(self, speed=FORWARD_SPEED):
        self.set_dir_servo_angle(0)
        current_angle = self.dir_current_angle
        if self.status == 'stop':
            self.move_start_time = time.time()
        self.status = 'backward'
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = (100 - abs_current_angle) / 100.0 
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * power_scale)
            else:
                self.set_motor_speed(1, -1*speed * power_scale)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self, speed=FORWARD_SPEED):
        self.set_dir_servo_angle(0)
        if self.status == 'stop':
            self.move_start_time = time.time()
        self.status = 'forward'
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = (100 - abs_current_angle) / 100.0
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, 1*speed * power_scale)
                self.set_motor_speed(2, -speed) 
            else:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * power_scale)
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)                  

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)
        self.update_distance()
        self.move_start_time = None
        self.status = 'stop'

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def reset(self):
        self.stop()
        self.set_dir_servo_angle(0)
        self.set_cam_tilt_angle(0)
        self.set_cam_pan_angle(0)

    def turnright(self, power=TURN_SPEED):
        self.stop()
        self.set_dir_servo_angle(30)
        self.status = 'turning'
        time.sleep(0.2)
        self.set_motor_speed(2, power)
        self.set_motor_speed(1, power)
        time.sleep(self.TURN_TIME)
        self.stop()
        cur_dir_idx = self.DIRECITONS.index(self.direction)
        self.direction = self.DIRECITONS[(cur_dir_idx+1)%4]

    def turnleft(self, power=TURN_SPEED):
        self.stop()
        self.set_dir_servo_angle(-30)
        self.status = 'turning'
        time.sleep(0.2)
        self.set_motor_speed(2, -power)
        self.set_motor_speed(1, -1*power)
        time.sleep(self.TURN_TIME)
        self.stop()
        cur_dir_idx = self.DIRECITONS.index(self.direction)
        self.direction = self.DIRECITONS[(cur_dir_idx-1)%4]
    

    def turnaround(self, power=TURN_SPEED):
        self.stop()
        self.set_dir_servo_angle(30)
        self.status = 'turning'
        time.sleep(0.2)
        self.set_motor_speed(2, power)
        self.set_motor_speed(1, power)
        time.sleep(self.TURN_TIME * 2)
        self.stop()
        cur_dir_idx = self.DIRECITONS.index(self.direction)
        self.direction = self.DIRECITONS[(cur_dir_idx+1)%4]

    
    def cpu_temperature(self):          # cpu_temperature
        raw_cpu_temperature = subprocess.getoutput("cat /sys/class/thermal/thermal_zone0/temp")
        cpu_temperature = round(float(raw_cpu_temperature)/1000,2)               # convert unit
        #cpu_temperature = 'Cpu temperature : ' + str(cpu_temperature)
        return cpu_temperature

    def gpu_temperature(self):          # gpu_temperature(
        raw_gpu_temperature = subprocess.getoutput( 'vcgencmd measure_temp' )
        gpu_temperature = round(float(raw_gpu_temperature.replace( 'temp=', '' ).replace( '\'C', '' )), 2)
        #gpu_temperature = 'Gpu temperature : ' + str(gpu_temperature)
        return gpu_temperature


    def disk_space(self):               # disk_space
        p = os.popen("df -h /")
        i = 0
        while 1:
            i = i +1
            line = p.readline()         
            if i==2:
                return line.split()[1:5]    

    def ram_info(self):
        p = os.popen('free')
        i = 0
        while 1:
            i = i + 1
            line = p.readline()
            if i==2:
                return list(map(lambda x:round(int(x) / 1000,1), line.split()[1:4]))
    
    def power_read(self):
        power_read_pin = ADC('A4')
        power_val = power_read_pin.read()
        power_val = power_val / 4095.0 * 3.3
        # print(power_val)
        power_val = power_val * 3
        power_val = round(power_val, 2)
        return power_val

    def pi_read(self):
        result = {
            "cpu_temperature": self.cpu_temperature(), 
            "gpu_temperature": self.gpu_temperature(),
            "cpu_usage": psutil.cpu_percent()/100, 
            "disk": self.disk_space(), 
            "ram": self.ram_info(), 
            "battery": self.power_read(), 
        }
        return result
    def get_pi_data(self):
        return self.pi_read()

if __name__ == "__main__":
    px = Picarx()
    px.forward(px.FORWARD_SPEED, px.FORWARD_TIME)
    time.sleep(1)
    px.stop()
