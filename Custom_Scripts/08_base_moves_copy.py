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
    # Init audio
    pygame.mixer.init()
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

    try:
        while True:
            forward = 0
            sideways = 0
            rotation = 0

            # Movement
            if keyboard.is_pressed("w"):
                forward = speed
            elif keyboard.is_pressed("s"):
                forward = -speed
                if not reverse_playing:
                    reverse_channel = reverse_sound.play(loops=-1)
                    reverse_playing = True
            else:
                if reverse_playing:
                    if reverse_channel:
                        reverse_channel.stop()
                    reverse_playing = False
            if keyboard.is_pressed("d"):
                rotation = -rotation_speed
            elif keyboard.is_pressed("a"):
                rotation = rotation_speed
            if keyboard.is_pressed("q"):
                sideways = -speed
            elif keyboard.is_pressed("e"):
                sideways = speed

            # Lamp toggle
            if keyboard.is_pressed("1"):
                lamp_on = not lamp_on
                robot.head.lamp.set_lamp(lamp_on=lamp_on)
                print(f"Lamp {'ON' if lamp_on else 'OFF'}")
                while keyboard.is_pressed("1"):
                    time.sleep(0.1)

            # Police toggle
            if keyboard.is_pressed("2"):
                police_mode = not police_mode
                print(f"Police mode {'ON' if police_mode else 'OFF'}")
                if not police_mode:
                    # Reset LEDs to white
                    robot.left_arm.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
                    robot.right_arm.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
                    robot.base.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
                    robot.head.left_led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
                    robot.head.right_led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
                while keyboard.is_pressed("2"):
                    time.sleep(0.1)

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

            if keyboard.is_pressed("esc"):
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
        robot.stop()


if __name__ == "__main__":
    main()
