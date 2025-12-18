import os
import platform
import numpy as np
from datetime import datetime

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands

class ColorLed:
    """
    Klasse voor het aansturen van een RGB-led op de robot.

    De led kan verschillende kleuren en knipperpatronen aannemen, afhankelijk van het onderdeel waarop deze is gemonteerd.
    """
    
    NOCOLOR = 0
    '''Geen kleur.'''

    RED = 1
    '''Rood.'''

    GREEN = 2
    '''Groen.'''

    BLUE = 3
    '''Blauw.'''

    WHITE = 4
    '''Wit.'''

    REDGREEN = 5
    '''Rood en groen gecombineerd.'''

    LED_NONE = 0
    '''Geen status.'''

    LED_OFF = 1
    '''Led uit.'''

    LED_ON = 2
    '''Led aan (continu licht).'''

    LED_BLINK_OFF = 3
    '''Led knippert met meer 'uit' dan 'aan'.'''

    LED_BLINK_SLOW = 4
    '''Led knippert langzaam.'''

    LED_BLINK_FAST = 5
    '''Led knippert snel.'''

    LED_BLINK_VERYFAST = 6
    '''Led knippert zeer snel.'''

    bridge_manager: object
    '''De BridgeManager voor communicatie met de robot.'''

    parent_name: str
    '''Naam van het bovenliggende robotonderdeel.'''

    instance_ENUM: int
    '''Enum die aanduidt welk led-onderdeel dit is.'''

    instance_name: str
    '''Samengestelde naam zoals "robot.head.left_led".'''
    
    
    '''NOCOLOR = 0      UITGECOMMENT DOOR DEFINITIES HIERBOVEN. VOOR NU LATEN STAAN
    RED = 1
    GREEN = 2
    BLUE = 3
    WHITE = 4
    REDGREEN = 5

    LED_NONE = 0
    LED_OFF = 1
    LED_ON = 2
    LED_BLINK_OFF = 3
    LED_BLINK_SLOW = 4
    LED_BLINK_FAST = 5
    LED_BLINK_VERYFAST = 6'''

    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """
        Initialiseert een ColorLed-object.

        Args:
            bridge_manager (object): Interface naar de robothardware.
            parent_name (str): Naam van het bovenliggende onderdeel.
            instance_ENUM (int): Enumwaarde die het led-onderdeel aanduidt.
        """
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)


    def setcolor(self, color=0, blink=0) -> None:
        """
        Zet de kleur en het knipperpatroon van de led.

        Args:
            color (int): Eén van de kleureigenschappen, zoals `ColorLed.RED`, `ColorLed.BLUE`, etc.
            blink (int): Knipperinstelling, zoals `ColorLed.LED_BLINK_SLOW` of `ColorLed.LED_ON`.
        """
        if self.instance_ENUM == SaraRobotPartNames.LEFT_ARM_LED:
            Parameters = np.array([SaraRobotCommands.CMD_LA_COLOR, color, blink])

        if self.instance_ENUM == SaraRobotPartNames.RIGHT_ARM_LED:
            Parameters = np.array([SaraRobotCommands.CMD_RA_COLOR, color, blink])

        if self.instance_ENUM == SaraRobotPartNames.BASE_LED:
            Parameters = np.array([SaraRobotCommands.CMD_BASE_COLOR, color, blink])

        if self.instance_ENUM == SaraRobotPartNames.HEAD_LEFT_LED:
            Parameters = np.array([SaraRobotCommands.CMD_HEAD_LEFT_COLOR, color, blink])

        if self.instance_ENUM == SaraRobotPartNames.HEAD_RIGHT_LED:
            Parameters = np.array([SaraRobotCommands.CMD_HEAD_RIGHT_COLOR, color, blink])

        self.bridge_manager.cmd_Generic(Parameters[0], 2, np.array(Parameters[1:]))

        return
