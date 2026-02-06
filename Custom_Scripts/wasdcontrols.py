import time
import pygame
import keyboard  # pip install keyboard
from Common.sara_library import SaraRobot, ColorLed


def set_police_leds(robot, color_left, color_right):
    """Set alternating LEDs for police effect instantly (no blinking)."""
    robot.left_arm.led.setcolor(color=color_left, blink=ColorLed.LED_ON)
    robot.right_arm.led.setcolor(color=color_right, blink=ColorLed.LED_ON)
    robot.base.led.setcolor(color=color_left, blink=ColorLed.LED_ON)
    robot.head.left_led.setcolor(color=color_left, blink=ColorLed.LED_ON)
    robot.head.right_led.setcolor(color=color_right, blink=ColorLed.LED_ON)


def main():
    # Init audio + pygame display for keyboard input
    pygame.mixer.init()
    pygame.init()
    # create a tiny window so pygame can capture keyboard focus
    pygame.display.set_mode((200, 100))
    pygame.display.set_caption("Sara Control (focus this window)")

    reverse_sound = pygame.mixer.Sound("audioFiles/truckReverse.mp3")
    reverse_channel = None
    reverse_playing = False

    robot = SaraRobot(logging=False)
    time.sleep(1.0)

    robot.head.getversion()
    robot.body.getversion()

    lamp_on = False
    police_mode = False
    last_police_toggle = 0
    police_interval = 0.5  # seconds
    police_state = False  # alternate states

    robot.head.lamp.set_lamp(lamp_on=False)

    print(
        "Controls:\n"
        "W/S: forward/back\n"
        "A/D: rotate left/right\n"
        "Q/E: strafe left/wright\n"
        "1: toggle lamp\n"
        "2: toggle police lights\n"
        "ESC: quit\n"
    )

    speed = 50
    rotation_speed = 60
    refresh_rate = 0.1

    # helper to debounce toggle keys
    key_pressed_last = {}

    try:
        while True:
            # pump events so pygame updates internal state (must be called)
            pygame.event.pump()
            keys = pygame.key.get_pressed()

            forward = 0
            sideways = 0
            rotation = 0

            # Movement
            if keys[pygame.K_w]:
                forward = speed
            elif keys[pygame.K_s]:
                forward = -speed
                if not reverse_playing:
                    reverse_channel = reverse_sound.play(loops=-1)
                    reverse_playing = True
            else:
                if reverse_playing:
                    if reverse_channel:
                        reverse_channel.stop()
                    reverse_playing = False

            if keys[pygame.K_d]:
                rotation = -rotation_speed
            elif keys[pygame.K_a]:
                rotation = rotation_speed

            if keys[pygame.K_q]:
                sideways = -speed
            elif keys[pygame.K_e]:
                sideways = speed

            # toggle helper function
            def check_toggle(pygame_key, name):
                pressed = keys[pygame_key]
                was = key_pressed_last.get(name, False)
                toggled = False
                if pressed and not was:
                    toggled = True  # key down event
                key_pressed_last[name] = pressed
                return toggled

            # Lamp toggle (1)
            if check_toggle(pygame.K_1, "lamp"):
                lamp_on = not lamp_on
                robot.head.lamp.set_lamp(lamp_on=lamp_on)
                print(f"Lamp {'ON' if lamp_on else 'OFF'}")

            # Police toggle (2)
            if check_toggle(pygame.K_2, "police"):
                police_mode = not police_mode
                print(f"Police mode {'ON' if police_mode else 'OFF'}")
                if not police_mode:
                    # Reset LEDs to white
                    robot.left_arm.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
                    robot.right_arm.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
                    robot.base.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
                    robot.head.left_led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
                    robot.head.right_led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)

            # Police flashing logic
            if police_mode:
                now = time.time()
                if now - last_police_toggle >= police_interval:
                    last_police_toggle = now
                    police_state = not police_state
                if police_state:
                    set_police_leds(robot, ColorLed.RED, ColorLed.BLUE)
                else:
                    set_police_leds(robot, ColorLed.BLUE, ColorLed.RED)

            # Movement commands
            if not any([forward, sideways, rotation]):
                robot.base.move_stop()
            else:
                robot.base.move(
                    Sideways_Velocity=sideways,
                    Forward_Velocity=forward,
                    Rotation_Velocity=rotation,
                )

            # ESC to quit
            if keys[pygame.K_ESCAPE]:
                print("Exiting...")
                break

            time.sleep(refresh_rate)

    except KeyboardInterrupt:
        pass
    finally:
        robot.base.move_stop()
        robot.head.lamp.set_lamp(False)
        # Reset LEDs
        robot.left_arm.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
        robot.right_arm.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
        robot.base.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
        robot.head.left_led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
        robot.head.right_led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
        robot.base.brake(ApplyBrake=False)
        pygame.mixer.stop()
        pygame.mixer.quit()
        pygame.quit()
        robot.stop()



if __name__ == "__main__":
    main()