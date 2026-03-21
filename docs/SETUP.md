# Setup

## 도커 실행 예시
```bash
sudo docker run -it --rm \
  --name=my_bot \
  --net=host --ipc=host --pid=host \
  --privileged \
  -v /dev:/dev \
  -v ~/ros2_ws:/root/ros2_ws \
  ros:humble-ros-base
```

## 의존성 설치(컨테이너 내부)
```bash
apt update && apt install python3-pip -y
pip3 install numpy pyserial traitlets Adafruit-MotorHAT
apt install python3-opencv i2c-tools -y
```

## 레포 설치
```bash
cd /root/ros2_ws/src/jetank_move_test
. install.sh
```
