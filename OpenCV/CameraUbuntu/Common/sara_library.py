import platform
import numpy as np

from Common.distance_sensors import DistanceSensors
from Common.compass import Compass
from Common.battery import Battery
from Common.colorled import ColorLed
from Common.robot_base import RobotBase
from Common.encoders import Encoders
from Common.motionsensors import MotionSensors
from Common.robotarmmotor import RobotArmMotor
from Common.eyes import HeadEyes
from Common.head_lamp import HeadLamp
from Common.touch_sensors import TouchSensorsHead
from Common.touch_sensors import TouchSensorsBody

from Common.bridge_manager import BridgeManager
from Common.sara_ports import SaraRobotPorts

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands
from Common.sara_common import RobotArmPositions
from Common.sara_common import RobotHeadPositions


class SaraRobot:
    """
    Centrale klasse die de volledige robot initialiseert en aanstuurt.

    Deze klasse verbindt met de hardware, maakt de juiste onderdelen aan (armen, hoofd, basis, lichaam),
    en handelt alle binnenkomende data af via `process_callback`.
    """

    parent_name: str
    '''Naam van de root-node van de robot (altijd "robot").'''

    bridge_manager: object
    '''De centrale communicatiebrug met de hardwaremodules.'''

    logging: bool
    '''Geeft aan of statusinformatie wordt geprint naar de console.'''

    com_windows1: str
    '''COM-poort voor het hoofd (Windows).'''

    com_windows2: str
    '''COM-poort voor het lichaam (Windows).'''

    com_linux1: str
    '''COM-poort voor het hoofd (Linux).'''

    com_linux2: str
    '''COM-poort voor het lichaam (Linux).'''

    left_arm: object
    '''Object dat de linkerarm bestuurt (type: RobotArm).'''

    right_arm: object
    '''Object dat de rechterarm bestuurt (type: RobotArm).'''

    base: object
    '''Object dat de robotbasis bestuurt (type: RobotBase).'''

    body: object
    '''Object dat de sensoren en hardware van het lichaam beheert (type: Body).'''

    head: object
    '''Object dat de onderdelen van het hoofd beheert (type: Head).'''

    def __init__(
        self,
        logging=True,
    ) -> None:
        """Initialiseert de robot, maakt alle onderdelen aan en start de communicatie."""
        self.parent_name = "robot"
        self.bridge_manager = None
        
        print("-" * 80)
        self.com_windows1 = SaraRobotPorts.COM_HEAD_WINDOWS
        self.com_windows2 = SaraRobotPorts.COM_BODY_WINDOWS
        self.com_linux1 = SaraRobotPorts.COM_HEAD_LINUX
        self.com_linux2 = SaraRobotPorts.COM_BODY_LINUX
        self.logging = logging

        print("Starting robot")

        self.start()

        self.left_arm = RobotArm(self.bridge_manager, 
                                 parent_name=self.parent_name, 
                                 instance_ENUM=SaraRobotPartNames.LEFT_ARM)

        self.right_arm = RobotArm(self.bridge_manager, 
                                 parent_name=self.parent_name, 
                                 instance_ENUM=SaraRobotPartNames.RIGHT_ARM)

        self.base = RobotBase(self.bridge_manager, 
                              parent_name=self.parent_name,
                              instance_ENUM=SaraRobotPartNames.BASE)

        self.body = Body(self.bridge_manager,
                         parent_name=self.parent_name, 
                         instance_ENUM=SaraRobotPartNames.BODY)

        self.head = Head(self.bridge_manager, 
                         parent_name=self.parent_name, 
                         instance_ENUM=SaraRobotPartNames.HEAD)
        
        print("-" * 80)

        self.bridge_manager.connect()


    def start(self) -> None:
        """Selecteert COM-poorten op basis van OS en initialiseert BridgeManager."""

        if "LINUX" in platform.system().upper():
            print("Linux detected")
            self.bridge_manager = BridgeManager(
                port1=self.com_linux1, port2=self.com_linux2, baudrate=115200
            )
        else:
            print("Windows detected")
            self.bridge_manager = BridgeManager(
                port1=self.com_windows1, port2=self.com_windows2, baudrate=115200
            )

        self.bridge_manager.set_receive_callback_body(self.process_callback)
        self.bridge_manager.set_receive_callback_head(self.process_callback)

        print("-" * 80)

        return

    def stop(self) -> None:
        """Zet de rem uit en verbreekt de verbinding met de robot."""
        self.base.brake(ApplyBrake=False)
        self.bridge_manager.disconnect()

    def noodstop(self) -> None:
        """Stopt abrupt alle bewegingen van hoofd en lichaam."""
        self.bridge_manager.cmd_Generic(
            SaraRobotCommands.CMD_BODY_STOP, 0, 0, SaraRobotPartNames.BODY)
        self.bridge_manager.cmd_Generic(
            SaraRobotCommands.CMD_HEAD_STOP, 0, 0, SaraRobotPartNames.HEAD)

    def process_callback(self, data) -> None:
        """
        Verwerkt binnenkomende data van de robot en routeert deze naar het juiste onderdeel.

        Args:
            data (bytes): De ontvangen data inclusief header, payload en CRC.
        """
        response = data[1]

        if response == (SaraRobotCommands.CMD_VERSION_BODY | SaraRobotCommands.RESP_BIT):
            self.body.new_version_data(data)
            return

        if response == (SaraRobotCommands.CMD_VERSION_HEAD | SaraRobotCommands.RESP_BIT):
            self.head.new_version_data(data)
            return

        if response == (SaraRobotCommands.CMD_GET_BATTERY | SaraRobotCommands.RESP_BIT):
            self.body.battery.new_data(data)
            if self.logging:
                self.body.battery.print_state()
            return

        if response == (
            SaraRobotCommands.CMD_GET_DISTANCESENSORS | SaraRobotCommands.RESP_BIT
        ):
            self.body.distance_sensors.new_data(data)
            if self.logging:
                self.body.distance_sensors.print_values()
            return

        if response == (SaraRobotCommands.CMD_GET_COMPASS | SaraRobotCommands.RESP_BIT):
            self.body.compass.new_data(data)
            if self.logging:
                self.body.compass.print_values()
            return

        if response == (SaraRobotCommands.CMD_COMP_MOVE | SaraRobotCommands.RESP_BIT):
            self.body.compass.rotate_absolute_ready(data)
            return

        if response == (
            SaraRobotCommands.CMD_GET_ENCODERS | SaraRobotCommands.RESP_BIT
        ):
            self.body.encoders.new_data(data)
            if self.logging:
                self.body.encoders.print_values()
            return

        if response == (
            SaraRobotCommands.CMD_GET_MOTIONSENSORS | SaraRobotCommands.RESP_BIT
        ):
            self.body.motion_sensors.new_data(data)
            if self.logging:
                self.body.motion_sensors.print_values()
            return


        if response == (
            SaraRobotCommands.CMD_HEAD_TOUCHSENSORS | SaraRobotCommands.RESP_BIT
        ):
            self.head.touch_sensors.new_data(data)
            if self.logging:
                self.head.touch_sensors.print_values()
            return

        if response == (
            SaraRobotCommands.CMD_BODY_TOUCHSENSORS | SaraRobotCommands.RESP_BIT
        ):
            self.body.touch_sensors.new_data(data)
            if self.logging:
                self.body.touch_sensors.print_values()
            return

        # If not decoded, print the data
        hex_values = " ".join([format(x, "02X") for x in data])
        print("< NOT DECODED: " + hex_values)


class Body:
    """
    Klasse die de componenten van het lichaam groepeert en aanstuurt.
    """

    bridge_manager: object
    '''De interface naar de robot via BridgeManager.'''

    parent_name: str
    '''Naam van het bovenliggende robotonderdeel, meestal "robot".'''

    instance_ENUM: int
    '''Enum die het lichaam aanduidt (SaraRobotPartNames.BODY).'''

    instance_name: str
    '''Volledige naam zoals "robot.body".'''

    battery: object
    '''Component voor uitlezen van de batterijstatus (type: Battery).'''

    compass: object
    '''Kompasmodule voor het meten van de rotatie (type: Compass).'''

    encoders: object
    '''Encodermodule die de beweging van de wielen meet (type: Encoders).'''

    distance_sensors: object
    '''Afstandssensoren rondom het lichaam (type: DistanceSensors).'''

    motion_sensors: object
    '''Versnellingsmeter en gyroscoop (type: MotionSensors).'''
    
    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """Initialiseert alle sensoren en actuatoren in het lichaam (battery, compass, etc.)."""
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        self.distance_sensors = DistanceSensors(self.bridge_manager, 
                                                parent_name=self.instance_name, 
                                                instance_ENUM=SaraRobotPartNames.BODYDISTANCESENSORS)
        
        self.encoders = Encoders(self.bridge_manager,
                                                parent_name=self.instance_name, 
                                                instance_ENUM=SaraRobotPartNames.ENCODERS)
        
        self.compass = Compass(self.bridge_manager,
                                                parent_name=self.instance_name, 
                                                instance_ENUM=SaraRobotPartNames.COMPASS)
        
        self.battery = Battery(self.bridge_manager,
                                                parent_name=self.instance_name, 
                                                instance_ENUM=SaraRobotPartNames.BATTERY)

        self.motion_sensors = MotionSensors(self.bridge_manager, 
                                                parent_name=self.instance_name, 
                                                instance_ENUM=SaraRobotPartNames.MOTIONSENSORS)
    
        self.touch_sensors = TouchSensorsBody(self.bridge_manager, 
                                        parent_name=self.instance_name, 
                                        instance_ENUM=SaraRobotPartNames.BODY_TOUCHSENSORS
                                        )



    def getversion(self) -> None:
        """Vraagt de softwareversie van de body op via een commando."""
        self.bridge_manager.cmd_Generic(
            SaraRobotCommands.CMD_VERSION_BODY, 0, 0, SaraRobotPartNames.BODY
        )

    def new_version_data(self, data) -> None:
        """
        Print de ontvangen versie-informatie van het lichaam.

        Args:
            data (bytes): De response met de versie.
        """
        try:
            string_from_bytearray = data[3:-2].decode("utf-8")
            print("-" * 80)
            print("Software version : " + string_from_bytearray)
        except:
            print("Version bytes error")

        print("-" * 80)
        return
    

class Head:
    """
    Klasse die alle componenten van het hoofd groepeert en aanstuurt.
    """

    bridge_manager: object
    '''De interface naar de robot via BridgeManager.'''

    parent_name: str
    '''Naam van het bovenliggende robotonderdeel, meestal "robot".'''

    instance_ENUM: int
    '''Enum die het hoofd aanduidt (SaraRobotPartNames.HEAD).'''

    instance_name: str
    '''Volledige naam zoals "robot.head".'''

    pan_motor: object
    '''Motor voor zijwaartse rotatie (pan-beweging) van het hoofd.'''

    tilt_motor: object
    '''Motor voor op/af kantelen (tilt-beweging) van het hoofd.'''

    left_led: object
    '''Linker RGB-led op het hoofd.'''

    right_led: object
    '''Rechter RGB-led op het hoofd.'''

    eyes: object
    '''LED-oogcomponent voor weergave van expressies.'''

    lamp: object
    '''Koplamp van de robot.'''

    touch_sensors: object
    '''Aanraakgevoelige sensoren links/midden/rechts op het hoofd.'''
    
    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """Initialiseert motors, leds, ogen, lamp en touch-sensoren van het hoofd."""
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        self.pan_motor = RobotArmMotor(self.bridge_manager, 
                                        parent_name=self.instance_name, 
                                        instance_ENUM=SaraRobotPartNames.PAN_MOTOR,
                                        maximum_position=RobotHeadPositions.PAN_RIGHT,
                                        minimum_position=RobotHeadPositions.PAN_LEFT
                                        )

        self.tilt_motor = RobotArmMotor(self.bridge_manager, 
                                        parent_name=self.instance_name, 
                                        instance_ENUM=SaraRobotPartNames.TILT_MOTOR,
                                        maximum_position=RobotHeadPositions.TILT_UP,
                                        minimum_position=RobotHeadPositions.TILT_DOWN
                                        )

        self.left_led = ColorLed(self.bridge_manager, 
                                        parent_name=self.instance_name, 
                                        instance_ENUM=SaraRobotPartNames.HEAD_LEFT_LED
                                        )
        
        self.right_led = ColorLed(self.bridge_manager, 
                                        parent_name=self.instance_name, 
                                        instance_ENUM=SaraRobotPartNames.HEAD_RIGHT_LED
                                        )
        self.eyes = HeadEyes(self.bridge_manager, 
                                        parent_name=self.instance_name, 
                                        instance_ENUM=SaraRobotPartNames.HEAD_EYES
                                        )

        self.lamp = HeadLamp(self.bridge_manager, 
                                        parent_name=self.instance_name, 
                                        instance_ENUM=SaraRobotPartNames.HEAD_LAMP
                                        )

        self.touch_sensors = TouchSensorsHead(self.bridge_manager, 
                                        parent_name=self.instance_name, 
                                        instance_ENUM=SaraRobotPartNames.HEAD_TOUCHSENSORS
                                        )

    def getversion(self) -> None:
        """Vraagt de softwareversie van het hoofd op via een commando."""
        self.bridge_manager.cmd_Generic(
            SaraRobotCommands.CMD_VERSION_HEAD, 0, 0, SaraRobotPartNames.HEAD
        )

    def new_version_data(self, data) -> None:
        """
        Print de ontvangen versie-informatie van het hoofd.

        Args:
            data (bytes): De response met de versie.
        """
        try:
            string_from_bytearray = data[3:-2].decode("utf-8")
            print("-" * 80)
            print("Software version : " + string_from_bytearray)
        except:
            print("Version bytes error")

        print("-" * 80)
        return
    
class RobotArm:
    """
    Klasse die één arm van de robot groepeert (motor + led).
    """

    bridge_manager: object  
    '''De interface naar de robot via BridgeManager.'''

    parent_name: str  
    '''Naam van het bovenliggende robotonderdeel, meestal "robot".'''

    instance_ENUM: int  
    '''Enum die deze arm aanduidt (SaraRobotPartNames.LEFT_ARM of RIGHT_ARM).'''

    instance_name: str  
    '''Volledige naam zoals "robot.left_arm" of "robot.right_arm".'''

    motor: object  
    '''De armservo die deze arm beweegt (type: RobotArmMotor).'''

    led: object  
    '''De RGB-led op de betreffende arm (type: ColorLed).'''
    
    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """Initialiseert de motor en de LED van de linker- of rechterarm."""
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        if (instance_ENUM == SaraRobotPartNames.LEFT_ARM):
            self.motor = RobotArmMotor(self.bridge_manager, 
                                       parent_name=self.instance_name, 
                                       instance_ENUM= SaraRobotPartNames.LEFT_ARM_MOTOR)

            self.led = ColorLed(self.bridge_manager,
                               parent_name=self.instance_name, 
                               instance_ENUM= SaraRobotPartNames.LEFT_ARM_LED
                               )

        if (instance_ENUM == SaraRobotPartNames.RIGHT_ARM):
            self.motor = RobotArmMotor(self.bridge_manager, 
                                       parent_name=self.instance_name, 
                                       instance_ENUM= SaraRobotPartNames.RIGHT_ARM_MOTOR)
            
            self.led = ColorLed(self.bridge_manager,
                               parent_name=self.instance_name, 
                               instance_ENUM= SaraRobotPartNames.RIGHT_ARM_LED
                               )


