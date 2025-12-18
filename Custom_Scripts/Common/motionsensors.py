import numpy as np
from Common.sara_common import bodypart_to_string


class MotionSensors:
    """
    Klasse voor het uitlezen van bewegingssensoren op de robot.

    Deze sensoren meten lineaire versnelling of beweging van het robotonderdeel.
    """

    bridge_manager: object
    '''Interface naar de robot via BridgeManager.'''

    parent_name: str
    '''Naam van het bovenliggende hardwareonderdeel.'''

    instance_ENUM: int
    '''Enum die aanduidt welk onderdeel dit is.'''

    instance_name: str
    '''Samengestelde naam van het onderdeel zoals "robot.body.motion_sensors".'''

    sensors: np.ndarray
    '''Tweedimensionale array met bewegingsdata (bijvoorbeeld X en Y-as).'''

    abs_angle: float
    '''Berekende absolute hoek (indien van toepassing).'''

    valid_data: bool
    '''True als de laatste meting succesvol verwerkt is.'''

    error_counter: int
    '''Aantal opeenvolgende fouten bij het verwerken van data.'''

    callback: callable
    '''Optionele functie die wordt aangeroepen na nieuwe data.'''

    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """
        Initialiseert een MotionSensors-object.

        Args:
            bridge_manager (object): Interface naar de robot.
            parent_name (str): Naam van het bovenliggende onderdeel.
            instance_ENUM (int): Enumwaarde voor dit sensoronderdeel.
        """
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        self.sensors = np.zeros(2)
        self.abs_angle = 0.0
        self.valid_data = False
        self.error_counter = 0
        self.callback = None

    def new_data(self, data) -> None:
        """
        Verwerkt binnenkomende data van de bewegingssensoren.

        Args:
            data (bytes): Byte-array met sensorwaarden.
        """
        try:
            datalength = data[2]

            assert (
                datalength == 2
            ), f"{self.full_bodypart_name} data length not correct!"

            new_byte_array = data[3 : 3 + datalength]
            uint8_array = np.frombuffer(new_byte_array, dtype=">u1")

            self.sensors[0] = float(uint8_array[0])
            self.sensors[1] = float(uint8_array[1])

            self.valid_data = True
            self.error_counter = 0

            if self.callback is not None:
                self.callback()

        except Exception as e:
            print(f"Motion sensors data processing error: {e}")

            self.error_counter += 1
            if self.error_counter > 3:
                self.valid_data = False

    def print_values(self) -> None:
        """
        Print de laatste bewegingswaarden naar de console.
        """
        print("Motion     :", [int(value) for value in self.sensors])

    def get_all_values(self) -> np.ndarray:
        """
        Retourneert de laatste bewegingsmetingen.

        Returns:
            np.ndarray: Array van sensorwaarden.
        """
        return self.sensors

    def set_callback(self, callback) -> None:
        """
        Stelt een callback in die wordt aangeroepen bij nieuwe data.

        Args:
            callback (callable): Functie zonder argumenten.
        """
        self.callback = callback
        return