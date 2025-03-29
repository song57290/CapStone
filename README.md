# CapStone

sudo apt install ros-noetic-joy ros-noetic-rospy
pip install pygame
-> 조이스틱 값 받아오기 위한 패키지 추가

***************************************************
joystick.py 실행시키고나서 joy_display.py 실행 시킬 것

joy_display.py는 영어로 나와서 보기 힘들어서 한글화 시킨거임

***************************************************


PC1이 조이스틱 연결된 컴퓨터
PC2가 Gazebo깔린 컴퓨터

> 직장 환경:
ws://172.22.156.216:8080 줄의 주석을 해제하고, ws://192.168.35.2:8080 줄을 주석 처리.
TURN 서버 설정을 활성화(필요 시).

> 집 환경:
ws://192.168.35.2:8080 줄의 주석을 해제하고, ws://172.22.156.216:8080 줄을 주석 처리.
TURN 서버 설정은 비활성화(집에서는 필요 없음).

1.1. 직장 환경과 집 환경의 차이
> 직장 환경:
PC1: 172.22.156.216/20
PC2: 172.17.3.215/20
서로 다른 네트워크 → STUN/TURN 서버 필요, 시그널링 서버를 공용 네트워크에서 접근 가능하도록 설정.
PC2 접속: ssh song@172.17.3.215

> 집 환경:
PC1: 192.168.35.2/24
PC2: 192.168.35.25/24
동일한 네트워크 → STUN 서버만으로도 연결 가능, TURN 서버는 선택 사항.
PC2 접속: 동일 네트워크이므로 SSH 불필요.
목표:
집 환경에서도 직장 환경과 동일한 설정(STUN/TURN 서버 사용, 시그널링 서버 설정, SSH 접속 방식 등)을 사용.
script.js와 실행 명령어를 직장 환경 기준으로 통일하고, IP 주소만 환경에 따라 주석 처리로 변경.


# 직장 환경 (PC1: 172.22.156.216, PC2: 172.17.3.215)

# PC1에서 실행
# 1. joystick.py 실행
source /opt/ros/humble/setup.bash
cd ~/CapStone/
python3 joystick.py

# 2. rosbridge 실행
source /opt/ros/humble/setup.bash
ros2 run rosbridge_server rosbridge_websocket

# 3. 시그널링 서버 실행
cd ~/CapStone/
node server.js

# 4. PC1 웹 서버 실행
cd ~/CapStone/webrtc-robot-arm/client/pc1/
npx http-server -p 3000
# 브라우저에서 http://127.0.0.1:3000 접속

# PC2에서 실행
# 1. PC2 가상서버 접속
ssh song@172.17.3.215

# 2. PC2 웹 서버 실행
cd ~/CapStone/webrtc-robot-arm/client/pc2/
npx http-server -p 3001
# 브라우저에서 http://172.17.3.215:3001 접속

# 집 환경 (PC1: 192.168.35.2, PC2: 192.168.35.25)
# 직장 환경과 동일한 방식으로 실행

# PC1에서 실행
# 1. joystick.py 실행
source /opt/ros/humble/setup.bash
cd ~/CapStone/
python3 joystick.py

# 2. rosbridge 실행
source /opt/ros/humble/setup.bash
ros2 run rosbridge_server rosbridge_websocket

# 3. 시그널링 서버 실행
cd ~/CapStone/
node server.js

# 4. PC1 웹 서버 실행
cd ~/CapStone/webrtc-robot-arm/client/pc1/
npx http-server -p 3000
# 브라우저에서 http://127.0.0.1:3000 접속

# PC2에서 실행
# 1. PC2 접속 (SSH 사용)
ssh song@192.168.35.25

# 2. PC2 웹 서버 실행
cd ~/CapStone/webrtc-robot-arm/client/pc2/
npx http-server -p 3001
# 브라우저에서 http://192.168.35.25:3001 접속


공인 IP 관련은 Grok에서
song@song-950XCJ-951XCJ-950XCR:~$ listening-port=3478
external-ip=<PC1의 공인 IP>
user=user:pass
realm=yourdomain.com
listening-port=3478: 명령을 찾을 수 없습니다
-bash: 예기치 않은 토큰 `newline' 근처에서 문법 오류
위 문장을 검색하고 아래 나오는 것들을 차근차근 입력하면 됨