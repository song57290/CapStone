import pygame

pygame.init()
pygame.joystick.init()
print(f"연결된 조이스틱 수: {pygame.joystick.get_count()}")

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"조이스틱 이름: {joystick.get_name()}")
    print(f"축 개수: {joystick.get_numaxes()}")
    print(f"버튼 개수: {joystick.get_numbuttons()}")
else:
    print("조이스틱이 연결되지 않았습니다.")
