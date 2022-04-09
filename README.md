# **Bachelorarbeit**

## **Steuerung eines Kleinfahrzeugs mit LiDAR, Bodenmessung und Kamera**
**Betreuer (Owner): Prof. Dr. Hartmut Gimpel (Fakultät MA) <br/>**
**Student (Developer): James Kibii, EIB <br/>**

### **Projektbeschreibung**
In diesem Projekt soll ein Kleinfahrzeug mit mehreren Sensoren gesteuert werden: 
einem industriellen Laserscanner, einer Spiegel-Anordnung mit der dieser 
Laserscanner auch den Boden erkennt, und mit einer kleinen Kamera. Das verwendete 
Fahrzeug ist mit einem Raspberry Pi Kleinstrechner ausgestattet („Pi-Car“) und 
bereits einmal vorhanden.

### **Gliederung der Bachelorarbeit**
* Entwurf eines Algorithmus in Python 
zur Kalibration des vorhandenen 
Aufbaus zur Boden-Erkennung (Laserscanner mit zwei Spiegeln)
* Auswertung von Lidar-Messdaten (in Matlab oder Python) um die Performance des Aufbaus zur 
Boden-Erkennung (Laserscanner mit Spiegeln) detailliert zu quantifizieren
  * (falls nötig auch eigene Messdaten-Aufnahme, sobald die vorige Projektarbeit beendet ist)
  * Falls nötig, Aufbau eines identischen zweiten Exemplars des Pi-Cars mit Lidar und zwei 
  Spiegeln (falls die vorauslaufende Projektarbeit länger andauert).
* Mitarbeit an einem Journal Paper zu diesem Boden-Erkennungsaufbau
* Ergänzung des Python-Codes im Kleinfahrzeug, sodass der Boden-Erkennungsaufbau zur 
Absturz-Vermeidung beim Fahren genutzt wird
* Ergänzung des Python-Codes, sodass die Lidar-Daten aus dem fahrenden Fahrzeug exportiert 
und visualisiert werden können (z.B. mit einem Webserver via WLAN)
* Auswahl, Beschaffung und Inbetriebnahme eines Kameramoduls für den Raspberry Pi im Fahrzeug
* Verwendung einer Bildverarbeitungsbibliothek (vermutlich Open CV) über Python mit dem Ziel speziell 
angefertigte Markierungscodes in der Umgebung des Fahrzeugs zu erkennen und anzusteuern
* Erarbeitung von Vorschlägen um mit den vorhandenen Teilen (Pi-Car mit Lidar, Tor mit Radar, 
Markierungsaufkleber für das Pi-Car) im Messtechnik-Labor ein Parcours für Fahrten dieses 
Fahrzeugs aufgebaut werden könnte. (Der Aufbau selbst ist nicht Teil dieser Abschlussarbeit.) 

Das Projekt wird mit der Programmiersprache Python durchgeführt. Der Kern dieses Projekts ist die Implementierung der
Bibliothek pyusb [PyPi](https://pypi.org/project/pyusb/), [Github](https://github.com/pyusb/pyusb) zum
Auslesen der Daten über Endpoints.<br/>
Zum besseren Verständnis des 2D-LiDar kann man sich das folgende Youtube-Video [LiDar Tutorial](https://www.youtube.com/watch?v=wKrJ0fx648A&ab_channel=SICKSensorIntelligence.)
der Firma SICK ansehen. <br/>
Ein Tutorial zu Data-Telegrammen finden Sie auch hier 
[Telegramms Tutorial](https://www.youtube.com/watch?v=cTy66J6B8WY&ab_channel=SICKSensorIntelligence.). <br/>

### **Nützliche Links**
* [Operating Instructions](https://www.sick.com/media/pdf/4/04/604/IM0044604.PDF)
* [TiM-Serie, TiM3xx, Produktfamilienübersicht](https://cdn.sick.com/media/familyoverview/1/51/751/familyOverview_TiM3xx_g205751_de.pdf)
* [SOPAS Software](https://www.sick.com/de/de/sopas-engineering-tool-2020/p/p367244)
