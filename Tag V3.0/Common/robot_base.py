import os
import platform
import numpy as np
from datetime import datetime
import math

from Common.colorled import ColorLed
from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands


class RobotBase:
    """
    Klasse voor het aansturen van de onderkant (basis) van de robot.

    Hiermee kan de robot rijden in zijwaartse, voorwaartse en draaiende richting.
    """

    bridge_manager: object
    '''De communicatie-interface met de robot.'''

    parent_name: str
    '''Naam van het bovenliggende hardwareonderdeel.'''

    instance_ENUM: int
    '''Enum die aanduidt welk onderdeel dit is.'''

    instance_name: str
    '''Volledige naam zoals "robot.base".'''

    led: object
    '''Het LED-onderdeel van de robotbasis (bijv. RGB-status-led).'''

    motors: object
    '''Motoraansturing van de robotbasis.'''

    UP = 500
    '''Waarde voor volledig opgetild.'''

    FORWARD = 350
    '''Waarde voor neutrale voorwaartse positie.'''

    DOWN = 100
    '''Waarde voor volledig omlaag.'''
    
    #UP = 500
    #FORWARD = 350
    #DOWN = 100

    # def __init__(self, bridge_manager, bodypart):
    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None: 
        """
        Initialiseert een RobotBase-object.

        Args:
            bridge_manager (object): Communicatie-interface naar de robot.
            parent_name (str): Naam van het bovenliggende robotonderdeel.
            instance_ENUM (int): Enumwaarde die de basis aanduidt.
        """       
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        self.led = ColorLed(self.bridge_manager,
                    parent_name = self.instance_name, 
                    instance_ENUM= SaraRobotPartNames.BASE_LED
                    )

        self.motors = BaseMotors(self.bridge_manager,
                                 parent_name = self.instance_name, 
                                instance_ENUM= SaraRobotPartNames.BASE_MOTORS
                                )   

    def move_stop(self) -> None:
        """
        Stopt alle bewegingen van de robotbasis.
        """
        self.bridge_manager.cmd_Generic(
            SaraRobotCommands.CMD_BASE_MOVE, 3, np.array([0, 0, 0])
        )

    def move(self, Sideways_Velocity=0, Forward_Velocity=0, Rotation_Velocity=0) -> None:
        """
        Beweegt de robotbasis in zijwaartse, voorwaartse en/of roterende richting.

        Args:
            Sideways_Velocity (int): Snelheid naar links/rechts (–100 tot 100).
            Forward_Velocity (int): Snelheid naar voren/achter (–100 tot 100).
            Rotation_Velocity (int): Rotatiesnelheid (–100 tot 100).
        """
        assert abs(Sideways_Velocity <= 100), "Abs(Sideways) velocity too high"
        assert abs(Forward_Velocity <= 100), "Abs(Forward) velocity too high"
        assert abs(Rotation_Velocity <= 100), "Abs(Rotation) velocity too high"

        self.bridge_manager.cmd_Generic(
            SaraRobotCommands.CMD_BASE_MOVE,
            4,
            np.array([Sideways_Velocity, Forward_Velocity, Rotation_Velocity, 0]),
        )

    # Not sure if robot has brakes on base motors!!!
    def brake(self, ApplyBrake=False) -> None:
        """
        Activeert of deactiveert de rem op de robotbasis.

        Args:
            ApplyBrake (bool): True om de rem te activeren, False om te deactiveren.
        """
        Brake = 1 if ApplyBrake else 0
        self.bridge_manager.cmd_Generic(
            SaraRobotCommands.CMD_BASE_BRAKE, 1, np.array([Brake])
        )


class BaseMotors:
    """
    Klasse voor het verwerken van data van de motorencoders van de robotbasis.

    Deze klasse bevat de interne encoderwaardes van de basismotoren. Er worden momenteel geen actieve commando's naar motoren gestuurd vanuit deze klasse.
    """

    bridge_manager: object
    '''De communicatie-interface naar de robot.'''

    parent_name: str
    '''Naam van het bovenliggende robotonderdeel.'''

    instance_ENUM: int
    '''Enum die aanduidt welk motordeel dit is (bijv. BASE_MOTORS).'''

    instance_name: str
    '''Volledige naam zoals "robot.base.motors".'''

    encoders: np.ndarray
    '''Encoderwaarden van de basismotoren.'''

    # def __init__(self, bridge_manager, bodypart):
    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """
        Initialiseert een BaseMotors-object.

        Args:
            bridge_manager (object): Interface naar de robot.
            parent_name (str): Naam van het bovenliggende onderdeel.
            instance_ENUM (int): Enumwaarde voor het motordeel.
        """
        # self.bridge_manager = bridge_manager
        # self.full_bodypart_name = bodypart_to_string(bodypart) + ".motors"
        # print("Adding " + "robot." + self.full_bodypart_name)

        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        self.encoders = np.zeros(3)

    def new_data(self, data) -> None:
        """
        (Optioneel) verwerken van motorfeedback.

        Deze methode is gedefinieerd maar bevat geen actieve implementatie, omdat het dataschema niet gebruikt wordt.
        """
        # try:
        #     datalength = data[2]
        #     assert datalength == 22, self.full_bodypart_name + " data length not correct!"

        #     for i in range(11):
        #         new_byte_array = data[3 + i * 2 : -2]

        #         uint8_array = np.frombuffer(new_byte_array, dtype=">u1")
        #         combined_int = int.from_bytes(new_byte_array[0:2], byteorder="big")
        #         self.sensors[i] = float(combined_int)

        #     self.valid_data = True
        #     self.error_counter = 0
        #     # print(datetime.now())
        # except:
        #     print("Distance sensors data processing error")

        #     self.error_counter += 1

        #     if self.error_counter > 3:
        #         self.valid_data = False
        return
