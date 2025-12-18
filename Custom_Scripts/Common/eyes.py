import numpy as np

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands


class HeadEyes:
    """
    Klasse voor het aansturen van de ogen van de robotkop.

    De ogen kunnen in verschillende standen gezet worden, afhankelijk van de gewenste expressie.
    """
    
    bridge_manager: object
    '''Interface naar de robot via BridgeManager.'''

    parent_name: str
    '''Naam van het bovenliggende robotonderdeel.'''

    instance_enum: int
    '''Enum die aanduidt welk onderdeel dit is (bijv. HEAD_EYES).'''

    instance_name: str
    '''Volledige naam van het onderdeel, zoals "robot.head.eyes".'''
    
    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """
        Initialiseert een HeadEyes-object.

        Args:
            bridge_manager (object): Communicatiebrug met de robot.
            parent_name (str): Naam van het bovenliggende hardwareonderdeel.
            instance_ENUM (int): Enum die het ogenonderdeel aanduidt.
        """
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_enum = instance_ENUM
        self.instance_name = f"{self.parent_name}.{bodypart_to_string(self.instance_enum)}"

        print(f"Adding {self.instance_name}")

    def set_eyes(self, left_eye=None, right_eye=0):
        """
        Stelt de posities van de linker- en rechteroog in.

        Args:
            left_eye (int): Waarde voor het linkeroog (0–255). Mag ook None zijn.
            right_eye (int): Waarde voor het rechteroog (0–255).
        """
        parameters = np.array([SaraRobotCommands.CMD_HEAD_EYES, left_eye, right_eye])
        self.bridge_manager.cmd_Generic(parameters[0], 2, np.array(parameters[1:]))
        return
