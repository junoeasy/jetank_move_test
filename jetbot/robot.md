robot.py 공부
================================================

```python
import time
import traitlets
```
> traitlets : 주피터 노트북에서 나온 라이브러리.   
> 파이썬의 변수를 스마트하게 만들어 준답니다...   
> 근데 그냥 일반 환경에서도 사용 가능함!!
---------------------------------------

```python
from traitlets.config.configurable import SingletonConfigurable
import qwiic
from Adafruit_MotorHAT import Adafruit_MotorHAT
from .motor import Motor

# Scan for devices on I2C bus
addresses = qwiic.scan()
```
> qwiic : SparkFun에서 만든 I2C통신 기반 간편 연결 시스템   
> addresses : 모터드라이버 종류 받아오기   
> jetbot 모터 드라이버에는 Adafruit회사 제품과, SparkFun회사의 드라이버가 있다.   
> 내(준호)가 조사했을 때 Jetank의 모터 드라이브는 SparkFun의 TB6612FNG 모터드라이버 => 아마도 address에 93이 들어가는 거 같음
---------------------------------------
``` python
class Robot(SingletonConfigurable):
```
#### SingletonConfigurable: 싱글톤이고, 설정가능하다
> 임베디드는 물리적으로 하나뿐인 자원이니까 `robot=new Robot()` 이런거 하면 안됨
---------------------------------------
``` py
#SingletonConfigurable 

    left_motor = traitlets.Instance(Motor)
    right_motor = traitlets.Instance(Motor)
    
    
    # config
    i2c_bus = traitlets.Integer(default_value=1).tag(config=True)
    left_motor_channel = traitlets.Integer(default_value=1).tag(config=True)
    left_motor_alpha = traitlets.Float(default_value=1.0).tag(config=True)
    right_motor_channel = traitlets.Integer(default_value=2).tag(config=True)
    right_motor_alpha = traitlets.Float(default_value=1.0).tag(config=True)
```
>### traitlets
> - Instance : (객체 타입) 변수에 특정 타입만 들어올 수 있다고 선언  
>     - ex) left_motor = traitlets.Instance(Motor) => left_motor에 `Motor` 클래스만 들어올 수 있음      
>   
> - Integer,Float : (기초 데이터 타입) 어떤 값이 들어올지 선언함
>
> - tag : (메타 데이터 or 꼬리표) 이 변수가 어떤 특성을 갖고 있는지 나타냄. 나중에 이걸로 분류 하거나 불러올 수 있음
>     1. config : 설정 파일로 저장할 변수만 골라낼 때 (`config=True`) -> 이걸 true로 하면 다른 함수에서 수정 가능 `robot1.Robot.right_motor_channel=1` 처럼 다른 코드에서 접근 및 수정 가능
>     2. sync : UI와 동기화할 변수만 표시할 때. (`sync=True`)
>     3. custom : 나만의 분류가 필요할 때. (`sensor=True` 등)

---------------------------------------

``` python
    # Adafruit Hardware
    # 우리는 이거 필요없음
    if 96 in addresses:

            def __init__(self, *args, **kwargs):
                super(Robot, self).__init__(*args, **kwargs)
                self.motor_driver = Adafruit_MotorHAT(i2c_bus=self.i2c_bus)
                self.left_motor = Motor(self.motor_driver, channel=self.left_motor_channel, alpha=self.left_motor_alpha)
                self.right_motor = Motor(self.motor_driver, channel=self.right_motor_channel, alpha=self.right_motor_alpha)
                
            def set_motors(self, left_speed, right_speed):
                self.left_motor.value = left_speed
                self.right_motor.value = right_speed
                
            def forward(self, speed=1.0, duration=None):
                self.left_motor.value = speed
                self.right_motor.value = speed

            def backward(self, speed=1.0):
                self.left_motor.value = -speed
                self.right_motor.value = -speed

            def left(self, speed=1.0):
                self.left_motor.value = -speed
                self.right_motor.value = speed

            def right(self, speed=1.0):
                self.left_motor.value = speed
                self.right_motor.value = -speed

            def stop(self):
                self.left_motor.value = 0
                self.right_motor.value = 0
```
> 우리는 이거 필요없음 -> address에 93이 들어가기 때문에(SparkFun 모터드라이버기 때문에)
--------------------
``` py
    # SparkFun Hardware
    elif 93 in addresses:
                
            def __init__(self, *args, **kwargs):
                super(Robot, self).__init__(*args, **kwargs)
                
                self.motor_driver = qwiic.QwiicScmd()
                self.left_motor = Motor(self.motor_driver, channel=self.left_motor_channel, alpha=self.left_motor_alpha)
                self.right_motor = Motor(self.motor_driver, channel=self.right_motor_channel, alpha=self.right_motor_alpha)
                self.motor_driver.enable()
                
            def set_motors(self, left_speed, right_speed):
                self.left_motor.value = left_speed
                self.right_motor.value = right_speed
                self.motor_driver.enable()
        
            # Set Motor Controls: .set_drive( motor number, direction, speed)
            # Motor Number: A = 0, B = 1
            # Direction: FWD = 0, BACK = 1
            # Speed: (-255) - 255 (neg. values reverse direction of motor)
            
            def forward(self, speed=1.0, duration=None):
                speed = int(speed*255)
                self.motor_driver.set_drive(0, 0, speed)
                self.motor_driver.set_drive(1, 0, speed)
                self.motor_driver.enable()

            def backward(self, speed=1.0):
                speed = int(speed*255)
                self.motor_driver.set_drive(0, 1, speed)
                self.motor_driver.set_drive(1, 1, speed)
                self.motor_driver.enable()

            def left(self, speed=1.0):
                speed = int(speed*255)
                self.motor_driver.set_drive(0, 1, speed)
                self.motor_driver.set_drive(1, 0, speed)
                self.motor_driver.enable()

            def right(self, speed=1.0):
                speed = int(speed*255)
                self.motor_driver.set_drive(0, 0, speed)
                self.motor_driver.set_drive(1, 1, speed)
                self.motor_driver.enable()

            def stop(self):
                self.motor_driver.set_drive(0, 0, 0)
                self.motor_driver.set_drive(1, 1, 0)
                self.motor_driver.disable()
```