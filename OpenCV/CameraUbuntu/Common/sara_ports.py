class SaraRobotPorts:
    """
    Klasse die de standaard COM-poorten definieert voor verbinding met het hoofd en lichaam van de robot,
    op zowel Windows als Linux.
    """

    COM_HEAD_WINDOWS: str = "COM11"
    '''Standaard COM-poort voor het hoofd van de robot op Windows.'''

    COM_BODY_WINDOWS: str = "COM10"
    '''Standaard COM-poort voor het lichaam van de robot op Windows.'''

    COM_HEAD_LINUX: str = "/dev/ttyACM1"
    '''Standaard COM-poort voor het hoofd van de robot op Linux.'''

    COM_BODY_LINUX: str = "/dev/ttyACM0"
    '''Standaard COM-poort voor het lichaam van de robot op Linux.'''




'''class SaraRobotPorts:
    """
    Klasse die de COM-poorten definieert voor het hoofd en lichaam van de robot, op Windows en Linux.

    Deze klasse bevat standaardwaarden voor de communicatiepoorten.
    """
    COM_HEAD_WINDOWS = "COM11"
    COM_BODY_WINDOWS = "COM10"

    COM_HEAD_LINUX = "/dev/ttyACM1"
    COM_BODY_LINUX = "/dev/ttyACM0"
'''
