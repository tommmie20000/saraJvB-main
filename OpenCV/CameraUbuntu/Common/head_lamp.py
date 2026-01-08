import numpy as np

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands


class HeadLamp:
    """
    Klasse voor het aansturen van de hoofdlamp van de robot.

    De hoofdlamp kan aan- of uitgezet worden met behulp van een commando.
    """

    bridge_manager: object
    '''Interface naar de robot via BridgeManager.'''

    parent_name: str
    '''Naam van het bovenliggende robotonderdeel.'''

    instance_enum: int
    '''Enum die aanduidt welk onderdeel dit is (bijv. HEAD_LAMP).'''

    instance_name: str
    '''Volledige naam van het onderdeel, zoals "robot.head.lamp".'''

    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """
        Initialiseert een HeadLamp-object.

        Args:
            bridge_manager (object): Communicatiebrug met de robot.
            parent_name (str): Naam van het bovenliggende hardwareonderdeel.
            instance_ENUM (int): Enum die de hoofdlamp aanduidt.
        """
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_enum = instance_ENUM
        self.instance_name = f"{self.parent_name}.{bodypart_to_string(self.instance_enum)}"

        print(f"Adding {self.instance_name}")

    def set_lamp(self, lamp_on=False) -> None:
        """
        Zet de hoofdlamp aan of uit.

        Args:
            lamp_on (bool): True om de lamp aan te zetten, False om uit te zetten.
        """
        parameters = np.array([SaraRobotCommands.CMD_HEAD_LAMP, lamp_on])
        self.bridge_manager.cmd_Generic(parameters[0], 1, np.array(parameters[1:]))
        return
