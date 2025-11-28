This is documentation of the Sara Robot, also known as the QIHAN S1-W1.

Our robot has been modified with a custom script to run python scripts, which are under [Tag V3.0](https://github.com/tommmie20000/saraJvB-main/tree/main/Tag%20V3.0).

There is also a camera on the robot, which can be viewed using the python scripts under [OpenCV](https://github.com/tommmie20000/saraJvB-main/tree/main/OpenCV)
The camera does need AstraSDK drivers, which can be found [here](https://www.orbbec.com/developers/astra-sdk/)

# Ssh verbinden met PI 5
## Windows
Wij hebben voor ssh de pi 5 verbonden met de hotspot van de laptop. 
Je kan bij instelling van de hotspot verbonden apparaten zien en hun ip adressen erbij.  
Om te verbinden met ssh moet je in je terminal of in de terminal:
```bat
ssh sarajvb*@(ip)
```
en dan word je geprompt om het wachtwoord in te voeren.  
Wanneer verbind de ssh.

## linux (ubuntu)
Verbind de PI 5 met de hotspot van de laptop met linux.  
Om het ip te vinden van de PI voer je dit in je terminal:
```
arp -a
```
Het ip dat hieruit komt begint waarschijnlijk met `10.`  
Dan doe je hetzelfde als voor windows in een terminal:
```bat
ssh sarajvb*@(ip)
```
Dan wordt je geprompt om het wachtwoord in te voeren. Als correct ingevoerd wordt je vervonden met ssh.