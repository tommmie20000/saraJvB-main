import os
import platform
import numpy as np
import math
import time

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands


class Compass:
    """
    Klasse voor het uitlezen van de absolute rotatiehoek van de robot met behulp van een kompas.

    Kan ook een rotatieopdracht uitvoeren naar een absolute hoek en controleren of deze succesvol is afgerond.
    """

    bridge_manager: object
    '''Interface naar de robot via BridgeManager.'''

    parent_name: str
    '''Naam van het bovenliggende hardwareonderdeel.'''

    instance_ENUM: int
    '''Enum die het specifieke kompasonderdeel aanduidt.'''

    instance_name: str
    '''Volledige naam van het onderdeel, zoals "robot.body.compass".'''

    sensors: np.ndarray
    '''Sensorbuffer, momenteel 1-element array.'''

    abs_angle: float
    '''Laatst gemeten absolute hoek in graden.'''

    valid_data: bool
    '''True als de laatste data-invoer correct verwerkt kon worden.'''

    error_counter: int
    '''Aantal opeenvolgende verwerkingsfouten.'''

    target_rotation: int
    '''Doelhoek voor rotatie in graden.'''

    rotation_tmo_counter: int
    '''Huidige timeout-teller bij het roteren.'''

    rotation_tmo_threshold: int
    '''Maximaal aantal iteraties tijdens wachttijd voor rotatie.'''

    callback: callable
    '''Callback die wordt aangeroepen na nieuwe compassdata.'''

    rotate_ready: bool
    '''True zodra de rotatie als voltooid wordt gemeld.'''

    rotate_result: bool
    '''True als de rotatie succesvol is afgerond.'''
    
    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """
        Initialiseert het Compass-object.

        Args:
            bridge_manager (object): Interface naar de robot.
            parent_name (str): Naam van het bovenliggende onderdeel.
            instance_ENUM (int): Enum die dit compassonderdeel aanduidt.
        """
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        self.sensors = np.zeros(1)
        self.abs_angle = 0
        self.valid_data = False
        self.error_counter = 0

        self.target_rotation = 0
        self.rotation_tmo_counter = 0
        self.rotation_tmo_threshold = 100
        self.callback = None


    def new_data(self, data) -> None:        
        """
        Verwerkt binnenkomende compassdata.

        Args:
            data (bytes): Inkomend bericht.
        """
        try:
            # print("--> compass: " + data.hex())

            datalength = data[2]

            assert datalength == 6, (
                self.full_bodypart_name + " data length not correct!"
            )

            new_byte_array = data[3 : 3+6]

            int16_array_5 = np.frombuffer(new_byte_array, dtype=">i2")
            compass_angle, angle_degrees = self.calculate_angle(
                int16_array_5[0], int16_array_5[1]
            )

            self.abs_angle = float(compass_angle)
            self.valid_data = True
            self.error_counter = 0

            if self.callback is not None:
                self.callback()            

        except:
            print("Compass data processing error")

            self.error_counter += 1

            if self.error_counter > 3:
                self.valid_data = False

    def read_abs_angle(self) -> float:
        """
        Geeft de laatst gemeten absolute rotatiehoek.

        Returns:
            float: Hoek in graden.
        """
        return self.abs_angle

    def print_values(self) -> None:
        """
        Print de huidige absolute rotatiehoek naar de console.
        """
        print(f"Compass    : {self.abs_angle:.0f} Degree")

    # Function to calculate the angle
    def calculate_angle(self, x, y) -> tuple[float, float]:
        """
        Berekent de hoek in graden op basis van x- en y-waarden.

        Args:
            x (int): X-waarde uit de sensor.
            y (int): Y-waarde uit de sensor.

        Returns:
            tuple[float, float]: (kompashoek [0–360], ruwe hoek in graden)
        """
        # atan2(y, x) returns the angle in radians
        angle_radians = math.atan2(y, x)
        # Convert radians to degrees (optional)
        angle_degrees = math.degrees(angle_radians)

        #  convert to compass angles
        compass_degree = -1 * angle_degrees

        if compass_degree < 0:
            compass_degree += 360

        return compass_degree, angle_degrees

    # Rotate to an absolute angle using the compass
    def rotate_absolute(
        self, abs_rotation_angle=0, wait_for_finish=True, rotation_tmo_threshold=10
    ) -> None:
        """
        Laat de robot draaien naar een absolute hoek (0–360 graden).

        Args:
            abs_rotation_angle (int): Doelhoek.
            wait_for_finish (bool): True om te wachten op afronding.
            rotation_tmo_threshold (int): Maximaal aantal iteraties (x 0.1 sec).
        """
        assert (
            abs_rotation_angle < 360
        ), "Invalid abs_rotation_angle (>360 Deg is not allowed)!"
        assert (
            abs_rotation_angle >= 0
        ), "Invalid abs_rotation_angle (<0 Deg is not allowed)!"

        self.target_rotation = int(abs_rotation_angle)
        self.rotate_ready = False
        self.rotate_result = False
        self.rotation_tmo_counter = 0

        print(f"Compass rotation to {self.target_rotation :.0f} Deg ", end="")

        # Send command
        self.bridge_manager.cmd_createCompassMoveCommand(
            SaraRobotCommands.CMD_COMP_MOVE,
            self.target_rotation,
            rotation_tmo_threshold,
        )

        if (wait_for_finish == False) or (rotation_tmo_threshold == 0):
            print("\n")
            return

        # Wait for the response message or a timeout of 20 seconds
        while self.rotate_ready == False:
            assert self.rotation_tmo_counter < (
                rotation_tmo_threshold * 10
            ), "Rotate absolute timeout!"

            time.sleep(0.1)
            self.rotation_tmo_counter += 1
            print("-", end="")  # Print without a newline

        # Check the rotation result
        if self.rotate_result == 1:
            print("> ready")
        else:
            print("> failed")

        return

    def rotate_absolute_ready(self, data) -> None:
        """
        Wordt aangeroepen door BridgeManager als antwoord op compass-rotatie.

        Args:
            data (bytes): Binnenkomende data met statusbit.
        """
        try:
            datalength = data[2]

            try:
                assert datalength == 1, (
                    self.full_bodypart_name + " data length not correct!"
                )

                self.rotate_result = data[3] == 1
            except AssertionError as e:
                self.rotate_result = 0

            self.rotate_ready = True
        except:
            print(self.full_bodypart_name + " data processing error")

        return
    
    def set_callback(self, callback) -> None:
        """
        Stelt een callback in die aangeroepen wordt bij nieuwe compassdata.

        Args:
            callback (callable): Functie zonder argumenten.
        """
        self.callback = callback
        return