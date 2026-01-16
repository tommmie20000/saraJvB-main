This is documentation of the Sara Robot, also known as the QIHAN S1-W1.

Our robot has been modified with a custom script to run python scripts, which are under [Tag V3.0](https://github.com/tommmie20000/saraJvB-main/tree/main/Tag%20V3.0).

There is also a camera on the robot, which can be viewed using the python scripts under [OpenCV](https://github.com/tommmie20000/saraJvB-main/tree/main/OpenCV)
The camera does need AstraSDK drivers, which can be found [here](https://www.orbbec.com/developers/astra-sdk/)
This documentation is mostly dutch, so good luck!

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
Het ip dat hieruit komt begint waarschijnlijk met `10.` of een ander nummer, dit ligt vooral aan je configuratie. 
Dan doe je hetzelfde als voor windows in een terminal:
```bat
ssh sarajvb*@(ip)
```
Dan wordt je geprompt om het wachtwoord in te voeren. Als correct ingevoerd wordt je vervonden met ssh.

# Het installeren van Git
Ha! Wat toevallig, je hebt deze repo al geopend, dus je bent al bijna klaar.
Git werkt best simpel. En je hebt verschillende soorten commando’s.

Natuurlijk kom je nergens, zonder dat je Git installeert. Dit doe je met de volgende commando's:

## Linux (Pi)
```
sudo apt update            #  update alle packages
sudo apt install -y git    #  installeer git
```

## Windows
Dit kan handig zijn voor het ontwerpen van scripts nadat je de files kopieert
```
winget install --id Git.Git -e --source winget
```
of
[Direct van de website
](https://git-scm.com/install/windows)

# Forken van de repo

Om je eigen versie van de Sara Robot code te maken, is het handig om de repo te **forken** naar je eigen GitHub account. Dit geeft je de vrijheid om wijzigingen te maken zonder het centrale project te beïnvloeden.

1. Open de repo in je browser: [https://github.com/tommmie20000/saraJvB-main](https://github.com/tommmie20000/saraJvB-main)
2. Log in op je GitHub account.
3. Klik rechtsboven op de **Fork** knop.
4. Selecteer je eigen account als bestemming.
5. Je hebt nu een kopie van de repo in je eigen GitHub account.

## Je fork lokaal klonen

Na het forken, kan je je eigen fork lokaal werken met:

```bash
git clone https://github.com/<jouw-gebruikersnaam>/saraJvB-main.git /pad/naar/jouw/folder
```

Vervang `<jouw-gebruikersnaam>` door je eigen GitHub handle.

## Wijzigingen maken

1. Ga naar de folder van je clone:

```bash
cd /pad/naar/jouw/folder
```

2. Maak je wijzigingen in de code.
3. Voeg de wijzigingen toe:

```bash
git add .
```

4. Commit je wijzigingen:

```bash
git commit -m "Beschrijf je wijzigingen"
```

5. Push naar je eigen fork:

```bash
git push origin main
```

Persoonlijk ben ik meer fan van alles coderen op vscode die gelinked is met vscode, en dan alles pushen via de simpele UI en dan pullen op die pi. dit is vrij simpel en ook handig om te doen. Maar dit ligt compleet aan je eigen voorkeur, dus ik zou zeggen, google away!

# Up-to-date blijven met het centrale project

Om te zorgen dat je fork up-to-date blijft met de originele repo:

1. Voeg de originele repo toe als upstream:

```bash
git remote add upstream https://github.com/tommmie20000/saraJvB-main.git
```

2. Haal de laatste wijzigingen op van de centrale repo:

```bash
git fetch upstream
```

3. Merge deze wijzigingen in je lokale branch:

```bash
git merge upstream/main
```

4. Push de bijgewerkte code naar je eigen fork:

```bash
git push origin main
```

Door dit proces te volgen, kan iedereen hun eigen folder hebben, wijzigingen maken, en toch eenvoudig up-to-date blijven met het centrale project.

# Remote desktop met de PI

## real VNC server op de PI activeren

ealvnc staat standaard geinstaleerd op rasberrypios (het OS op de PI) die moet je alleen nog activeren via de terminal. Hierin voer je het command uit:
    sudo raspi-config
navigeer naar Interface Options slecteer VNC en druk enter. Selecteer dan yes en click enter.  
Dan kan je dit scherm sluiten en is realVNC geactiveerd

## realVNC viewer (client side)
ga naar de realVNC website [www.realvnc.com/en/connect/download/viewer/](www.realvnc.com/en/connect/download/viewer/) daan dowload je de RealVNC viewer .exe  
DIe kan je openen en dan instaleerd die de viewer.

## Verbinden met de PI 
Verbind beide de PI en client met de wifi van de tp-link router

Open RealVNC viewer op client (laptop) in de searchbar bovenin typ je `raspberrypi.local` druk enter.  
Dan krijg je een login scherm. Voer de login van de PI zelf in (die je ook gebruikt op de pi zelf).  
Wanneer je al eerder hebt verbonden kan je dubbel clicken op de preview die er staat waaronder `raspberrypi.local` staat