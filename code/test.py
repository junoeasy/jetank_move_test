# test_driver.py 파일을 만들어 아래 내용을 넣으세요
print("1. 드라이버 임포트 시작")
from Adafruit_MotorHAT import Adafruit_MotorHAT
print("2. 드라이버 임포트 성공, 객체 생성 시도")# Jetson Nano는 무조건 i2c_bus=1 입니다.
mh = Adafruit_MotorHAT(addr=0x60, i2c_bus=1)
print("3. 객체 생성 성공! 하드웨어 제어 가능")