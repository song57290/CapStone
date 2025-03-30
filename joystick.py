import pygame
import asyncio
import json
import websockets

async def send_joystick_data():
    # pygame 및 조이스틱 초기화
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    uri = "ws://127.0.0.1:8080"
    async with websockets.connect(uri) as websocket:
        while True:
            pygame.event.pump()  # 이벤트 큐 업데이트

            # 축 값 읽기 (예: 좌우, 상하, 기타)
            x = joystick.get_axis(0)
            y = -joystick.get_axis(1)
            z = -joystick.get_axis(4) if joystick.get_numaxes() > 2 else 0
            
            # LB와 RB 버튼 상태 읽기 (버튼 인덱스: 4와 5, 버튼 수 확인)
            lb = joystick.get_button(4) if joystick.get_numbuttons() > 4 else 0
            rb = joystick.get_button(5) if joystick.get_numbuttons() > 5 else 0
            
            data = {
                "x": round(x, 2),
                "y": round(y, 2),
                "z": round(z, 2),
                "lb": lb,
                "rb": rb
            }
            await websocket.send(json.dumps(data))
            print("전송 데이터:", data)
            await asyncio.sleep(0.1)

asyncio.run(send_joystick_data())
