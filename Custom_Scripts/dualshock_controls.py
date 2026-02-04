import time
import pygame
from Common.sara_library import SaraRobot, ColorLed


DEADZONE = 0.15


def apply_deadzone(value, dz=DEADZONE):
    return 0 if abs(value) < dz else value


def set_police_leds(robot, color_left, color_right):
    robot.left_arm.led.setcolor(color=color_left, blink=ColorLed.LED_ON)
    robot.right_arm.led.setcolor(color=color_right, blink=ColorLed.LED_ON)
    robot.base.led.setcolor(color=color_left, blink=ColorLed.LED_ON)
    robot.head.left_led.setcolor(color=color_left, blink=ColorLed.LED_ON)
    robot.head.right_led.setcolor(color=color_right, blink=ColorLed.LED_ON)


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No controller detected")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller connected: {joystick.get_name()}")

    robot = SaraRobot(logging=False)
    time.sleep(1.0)

    lamp_on = False
    police_mode = False
    police_state = False
    last_police_toggle = 0
    police_interval = 0.5

    speed = 60
    rotation_speed = 70
    refresh_rate = 0.05

    button_last = {}

    def check_button(btn, name):
        pressed = joystick.get_button(btn)
        was = button_last.get(name, False)
        button_last[name] = pressed
        return pressed and not was

    robot.head.lamp.set_lamp(False)

    try:
        while True:
            pygame.event.pump()

            # Sticks
            lx = apply_deadzone(joystick.get_axis(0))   # strafe
            rx = apply_deadzone(joystick.get_axis(2))   # rotation

            # Triggers (DS4: -1.0 -> 1.0)
            l2 = (joystick.get_axis(4) + 1) / 2  # reverse
            r2 = (joystick.get_axis(5) + 1) / 2  # forward

            forward = int((r2 - l2) * speed)
            sideways = int(lx * speed)
            rotation = int(rx * rotation_speed)

            # R1 → flashlight
            if check_button(5, "lamp"):  # R1
                lamp_on = not lamp_on
                robot.head.lamp.set_lamp(lamp_on)
                print(f"Lamp {'ON' if lamp_on else 'OFF'}")

            # L1 → police lights
            if check_button(4, "police"):  # L1
                police_mode = not police_mode
                print(f"Police mode {'ON' if police_mode else 'OFF'}")
                if not police_mode:
                    for led in [
                        robot.left_arm.led,
                        robot.right_arm.led,
                        robot.base.led,
                        robot.head.left_led,
                        robot.head.right_led,
                    ]:
                        led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)

            # Police flashing
            if police_mode:
                now = time.time()
                if now - last_police_toggle >= police_interval:
                    last_police_toggle = now
                    police_state = not police_state

                if police_state:
                    set_police_leds(robot, ColorLed.RED, ColorLed.BLUE)
                else:
                    set_police_leds(robot, ColorLed.BLUE, ColorLed.RED)

            # Movement
            if forward == 0 and sideways == 0 and rotation == 0:
                robot.base.move_stop()
            else:
                robot.base.move(
                    Sideways_Velocity=sideways,
                    Forward_Velocity=forward,
                    Rotation_Velocity=rotation,
                )

            # Share → quit
            if joystick.get_button(8):
                print("Exiting...")
                break

            time.sleep(refresh_rate)

    finally:
        robot.base.move_stop()
        robot.head.lamp.set_lamp(False)

        for led in [
            robot.left_arm.led,
            robot.right_arm.led,
            robot.base.led,
            robot.head.left_led,
            robot.head.right_led,
        ]:
            led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)

        robot.base.brake(ApplyBrake=False)
        pygame.quit()
        robot.stop()


if __name__ == "__main__":
    main()
