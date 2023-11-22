# Turtech

## Modulverantwortliche

- Ashkan Haghighi Fashi (act595, Ashkan.HaghighiFashi@haw-hamburg.de)
- Hauke S. Rösler (acx988, Hauke.Roesler@haw-hamburg.de)

## Problemstellung

Immer mehr, vorallem ältere Menschen, leben einen großen Teil ihrer
Zeit Zuhause und vorallem **alleine** Zuhause. Dies führt natürlich
zu Komplikationen, wenn diesen Menschen mal was passiert. Falls diese
unglücklich stürzen sollten, passiert dies meist leise und selbst die
Nachbarn kriegen davon nichts mit. Genau soetwas möchten wir frühzeitig
erkennen und so schnell es geht Hilfe in Form einer menschlischen 
Person rufen. Um dieses Problem möchten wir uns als gesamtes 
"Living Place Lab" Team kümmern.

Wobei der Anfang dieses Prozesses, also die Auffindung des Menschen 
mit Hilfe eines Roboters die Aufgabe von unserem "Turtech" Team ist.

## Modulziel

### WHAT
Unsere Teilgruppe arbeitet an einer Smarthome Lösung, welche die Aufgabe 
hat einen eventuell verunfallten Menschen in seiner Wohnung:
- mit einem Roboter ausfindig zu machen
- ~~über eine Touchdisplay Schnittstelle simpel mit dem System interagieren zu lassen~~
- zu fotografieren, um diese Aufnahmen Helfern zur Diagnose zur Verfügung zu stellen
- die Position der Person zu erfassen und in eine verständliche Form zu bringen

Die Auffindung, Positionserfassung und Aufnahme (Fotos) sind der Primäre-Fokus von Turtech.

### WHY
Die Fotos sind die beste Option um den Notfallkontakten sowie dem Notdienst 
eine möglichst akkurate Einschätzung der Situation zu ermöglichen, damit erstere 
Fehl-Alarme besser vermeiden können und letztere schon vor der Ankunft erste 
Behandlungspläne erstellen können. Die Positionsdaten sorgen hingegen dafür,
dass die Person nicht erst vom Notdienst gesucht werden muss, was vor allem in 
größeren Wohnungen an - in Notsituationen besonders wertvoller - Zeit spart.

- ~~Zur Kommunikation mit dem Menschen ist es notwendig diesen erstmal ausfindig zu 
machen und die passende Technologie zu ihm zu bringen, da dieser sich im schlimmsten Fall nicht mehr bewegen kann.~~
- ~~Die simpelste Art und Weise dem Menschen eine Möglichkeit zu geben seinen aktuelle OK-Status übermitteln zu lassen ist unserer Meinung nach eine "Hast du dich eben verletzt?" Frage auf einem Display mit einem "Nein" Knopf, damit der Mensch antworten kann.~~
- ~~Falls der Mensch nicht antworten kann, sind die Fotos für den Rettungsdienst sicherlich hilfreich zur Diagnose des Zustands und der Dringlichkeit.~~

## Konzept

Ablauf:
- Karte der Wohnung erstellen
  - Manuell, turtlebot3_slam
- Pfad erstellen
  - doc/path-concept.md
  - einmalig pro Karte
- Suche
  - Pfad abfahren
  - Neue Objekte auffinden
    - Vergleich Laserscannerdaten mit Kartendaten
  - Objekt (als Person) identifizieren
    - Bildverarbeitung
  - Fotos aufnehmen
  - Position auf Bild der Karte einzeichnen

### Systemkontext

Unser Suchroboter steht fast ganz am Anfang des "Menschen-Retten"- Use Case 
und ist dafür da den Menschen ausfindig zu machen, Fotos von diesem zu machen,
damit sowohl die Position sowie die Fotos vom Team Second Human Factor genutzt 
und an den Notdienst weitergeleitet werden können.

## Entwicklung

\<Was wird Implementiert? Welche Komponenten werden implementiert?\>

### Komponenten

- setup
- search
  - move
  - scan
  - see

Setup generates a file/files later used by Search

### Schnittstellen

- MQTT
  - Input: Smartwatch Signal
  - Output: Dateipfad zu den Bildern
- Bilder im Dateisystem der VM abgelegt

### Verwendete Technologien

- TurtleBot3 Waffle Pi
- Robot Operating System (ROS)
    - Face Recognition Package
    - Camera Package
    - Navigation-Stack
- Python 3

## Referenzen

https://wiki.csti.haw-hamburg.de/WiSe_2122_LP
https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview

## Präsentation

### Live Vorstellung

Ablauf wie in echtem Scenario.
<br>Fahrt über manuell verkleinerten Pfad, wahrscheinlich auf Schlafzimmer reduziert
<br>Notwendiges Equipment: Turtlebot, LivingPlace-Rechner

### Einzel Vorstellung

Video in Dauerschleife aus Turtle-Kamera-Feed, Kamera aufnahmen vom Roboter, 
Screen-Capture von rviz mit navigation, laserscanner und costmap overlays und Logs, 
sowie Terminal Aufnahme als Intro.
<br>Notwendiges Equipment: Monitor um Video zu zeigen

