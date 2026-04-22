#!/usr/bin/python

######################################################
#  Python API
#  This library is used for the MR-25 robot.
#  http://www.macerobotics.com
#  Date : 10/03/2026
#  Version : 0.79
#
#  Modif :
#  15/02/26 : suppression des print sur ledRGB
#  22/06/26 : ajout fichier de calibration des moteurs
# 
#  Fonctionne avec Python 3
#
#  CopyLeft
######################################################

"""
MR25.py — Bibliothèque Python pour le robot MR-25 (Macerobotics)
-----------------------------------------------------------------
Interface de haut niveau pour piloter le robot MR-25 via liaison
série UART (/dev/ttyAMA0, 230400 bauds).

Protocole de communication série :
    Envoi    : #<COMMANDE>,<param1>,<param2>!\n
    Réponse  : $<valeur>\n

Dépendances :
    - serial  : communication UART avec le microcontrôleur embarqué
    - INA219  : lecture tension/courant batterie via I²C (adresse 0x41)
"""

from math import *
import serial
import time
import os
import INA219


# ─────────────────────────────────────────────────────────────────────
# Exports publics de l'API
# ─────────────────────────────────────────────────────────────────────
# Chaque __all__ déclare une fonction publique de l'API.
# Note : en Python, __all__ devrait être une liste unique — ici chaque
# ligne écrase la précédente, mais cela n'affecte pas le fonctionnement
# des fonctions elles-mêmes.

__all__ = ['firmwareVersion']   # lecture version firmware microcontrôleur
__all__ = ['battery']           # lecture tension batterie (V)
__all__ = ['batteryCurrent']    # lecture courant batterie (mA)
__all__ = ['proxSensor']        # lecture d'un capteur de proximité (1–5)
__all__ = ['proxSensorAll']     # lecture des 5 capteurs de proximité
__all__ = ['forward']           # avance à vitesse fixe
__all__ = ['back']              # recule à vitesse fixe
__all__ = ['stop']              # arrêt immédiat
__all__ = ['turnRight']         # rotation droite
__all__ = ['turnLeft']          # rotation gauche
__all__ = ['motorRight']        # commande directe moteur droit
__all__ = ['motorLeft']         # commande directe moteur gauche
__all__ = ['speedMotorRight']   # lecture vitesse moteur droit
__all__ = ['speedMotorLeft']    # lecture vitesse moteur gauche
__all__ = ['encoderLeft']       # lecture encodeur gauche
__all__ = ['encoderRight']      # lecture encodeur droit
__all__ = ['encoderReset']      # remise à zéro des encodeurs
__all__ = ['writeCommand']      # envoi d'une commande brute
__all__ = ['readData']          # lecture d'une réponse série
__all__ = ['buzzer']            # commande buzzer (fréquence + durée)
__all__ = ['buzzerStop']        # arrêt buzzer
__all__ = ['ledRGB']            # contrôle LED RGB
__all__ = ['ledLowBatt']        # contrôle LED batterie faible
__all__ = ['orientation']       # calcul de l'orientation par odométrie
__all__ = ['calibMotors']       # calibration des moteurs


# ─────────────────────────────────────────────────────────────────────
# Constantes
# ─────────────────────────────────────────────────────────────────────

# Couleurs prédéfinies pour la LED RGB (format "RVB" binaire sur 3 bits)
_RED   = "100"   # rouge pur
_GREEN = "010"   # vert pur
_BLUE  = "001"   # bleu pur


# ─────────────────────────────────────────────────────────────────────
# Initialisation du matériel
# ─────────────────────────────────────────────────────────────────────

# Ouverture du port série vers le microcontrôleur (STM32)
# /dev/ttyAMA0 : UART matériel du Raspberry Pi
# 230400 bauds, 8N1 (8 bits données, pas de parité, 1 bit stop)
port = serial.Serial('/dev/ttyAMA0', 230400, bytesize=8, parity='N', stopbits=1)
time.sleep(0.5)  # pause pour laisser le temps au port de s'initialiser

# Initialisation du capteur INA219 pour la mesure batterie
# Adresse I²C : 0x41
ina219 = INA219.INA219(addr=0x41)



# lecture du fichier de calibration des moteurs du robot.
values = {}
with open('calibration_moteurs.txt', 'r') as f:
    for line in f:
        parts = line.split()
        if len(parts) == 2:
            values[parts[0]] = int(parts[1])

_CALIB_MOTOR_RIGHT = values['MOTOR_RIGHT']
_CALIB_MOTOR_LEFT = values['MOTOR_LEFT']

# fermeture du fichier de calibration
f.close()






# ─────────────────────────────────────────────────────────────────────
# Informations système
# ─────────────────────────────────────────────────────────────────────

def firmwareVersion():
    """
    Lit la version du firmware du microcontrôleur embarqué.

    Envoie la commande '#FV!' et retourne la valeur reçue.

    Returns:
        float: Numéro de version du firmware (ex: 1.23).

    Exemple:
        >> firmwareVersion()
    """
    port.flushInput()       # vide le buffer de réception avant d'envoyer
    port.write(b'#FV!\n')   # commande de lecture de version
    value = readData()
    return float(value)


# ─────────────────────────────────────────────────────────────────────
# Capteurs
# ─────────────────────────────────────────────────────────────────────

def battery():
    """
    Lit la tension de la batterie via le capteur INA219.

    Returns:
        float: Tension en volts.

    Exemple:
        >> battery()
    """
    return ina219.getBusVoltage_V()


def batteryCurrent():
    """
    Lit le courant consommé par la batterie via le capteur INA219.

    Returns:
        float: Courant en milliampères (mA).

    Exemple:
        >> batteryCurrent()
    """
    return ina219.getCurrent_mA()


def proxSensor(sensor):
    """
    Lit la valeur d'un capteur de proximité individuel.

    Envoie '#PR,<n>!' et retourne la distance mesurée.

    Args:
        sensor (int): Numéro du capteur, de 1 à 5.

    Returns:
        float: Distance en millimètres (0 à 2000 mm).

    Exemple:
        >> proxSensor(2)
    """
    if sensor not in (1, 2, 3, 4, 5):
        print("error parameter")
        return None
    port.flushInput()                            # vide le buffer de réception
    port.write(b"#PR,")
    port.write(bytes(str(sensor), 'utf-8'))      # numéro du capteur en ASCII
    port.write(b'!\n')
    value = readData()
    return float(value)


def proxSensorAll():
    """
    Lit les 5 capteurs de proximité en séquence.

    Returns:
        list[float]: Liste de 5 distances en mm, capteurs 1 à 5.

    Exemple:
        >> proxSensorAll()
    """
    pd = []
    for i in range(1, 6):
        pd.append(proxSensor(i))
    return pd


# ─────────────────────────────────────────────────────────────────────
# Calibration des moteurs
# ─────────────────────────────────────────────────────────────────────

def calibMotors(motor_right, motor_left):
  port.write(b"#CALIBR,")
  port.write(bytes(str(motor_right), 'utf-8'))
  port.write(b'!\n')

  port.write(b"#CALIBL,")
  port.write(bytes(str(motor_left), 'utf-8'))
  port.write(b'!\n')

# ─────────────────────────────────────────────────────────────────────
# Déplacements
# ─────────────────────────────────────────────────────────────────────

def forward(speed):
    """
    Fait avancer le robot à la vitesse donnée.

    Envoie '#MF,<speed>!'.

    Args:
        speed (int): Vitesse de 0 à 100.

    Exemple:
        >> forward(20)
    """
    if 0 <= speed <= 100:
        port.write(b"#MF,")
        port.write(bytes(str(speed), 'utf-8'))
        port.write(b'!\n')


def forward2(speed, acceleration):
    """
    Fait avancer le robot avec une rampe d'accélération progressive.

    Augmente la vitesse de 0 jusqu'à `speed` par pas de `acceleration`
    par itération (toutes les 100 ms).

    Args:
        speed       (int): Vitesse cible (0 à 100).
        acceleration(float): Incrément de vitesse par pas de temps.
    """
    timef = 1
    y_speed = 0
    while y_speed <= speed:
        y_speed = acceleration * timef   # vitesse linéairement croissante
        timef += 1
        forward(int(y_speed))
        time.sleep(0.1)


def forwardmm(distance):
    """
    Fait avancer le robot d'une distance précise en millimètres.

    Envoie '#FM,<distance>!'. Utilise le contrôle d'encodeur embarqué.

    Args:
        distance (int): Distance en mm, entre -1000 et +1000.

    Exemple:
        >> forwardmm(200)
    """
    if -1001 < distance < 1001:
        port.write(b"#FM,")
        port.write(bytes(str(distance), 'utf-8'))
        port.write(b'!\n')
    else:
        print("error distance value")


def turnAngle(angle):
    """
    Fait pivoter le robot d'un angle précis en degrés.

    Envoie '#TA,<angle>!'. Utilise le contrôle d'encodeur embarqué.

    Args:
        angle (int): Angle en degrés (positif = droite, négatif = gauche).
    """
    port.write(b"#TA,")
    port.write(bytes(str(angle), 'utf-8'))
    port.write(b'!\n')


def back(speed):
    """
    Fait reculer le robot à la vitesse donnée.

    Envoie '#MB,<speed>!'.

    Args:
        speed (int): Vitesse de 0 à 100.

    Exemple:
        >> back(20)
    """
    if 0 <= speed <= 100:
        port.write(b'#MB,')
        port.write(bytes(str(speed), 'utf-8'))
        port.write(b'!\n')
    else:
        print("error speed value")


def stop():
    """
    Arrête immédiatement le robot.

    Envoie '#STP!'.

    Exemple:
        >> stop()
    """
    port.write(b'#STP!\n')


def turnRight(speed):
    """
    Fait tourner le robot vers la droite.

    Envoie '#TR,<speed>!'.

    Args:
        speed (int): Vitesse de rotation de 0 à 100.

    Exemple:
        >> turnRight(30)
    """
    if 0 <= speed <= 100:
        port.write(b"#TR,")
        port.write(bytes(str(speed), 'utf-8'))
        port.write(b'!\n')
    else:
        print("error speed value")


def turnLeft(speed):
    """
    Fait tourner le robot vers la gauche.

    Envoie '#TL,<speed>!'.

    Args:
        speed (int): Vitesse de rotation de 0 à 100.

    Exemple:
        >> turnLeft(30)
    """
    if 0 <= speed <= 100:
        port.write(b'#TL,')
        port.write(bytes(str(speed), 'utf-8'))
        port.write(b'!\n')
    else:
        print("error speed value")


def motorRight(direction, speed):
    """
    Commande directe du moteur droit (sans logique haut niveau).

    Envoie '#MOTR,<dir>,<speed>!'.

    Args:
        direction (int): Sens de rotation : 0 = arrière, 1 = avant.
        speed     (int): Vitesse de 0 à 100.

    Exemple:
        >> motorRight(1, 50)
    """
    port.write(b'#MOTR,')
    port.write(bytes(str(direction), 'utf-8'))
    port.write(b',')
    port.write(bytes(str(speed), 'utf-8'))
    port.write(b'!\n')


def motorLeft(direction, speed):
    """
    Commande directe du moteur gauche (sans logique haut niveau).

    Envoie '#MOTL,<dir>,<speed>!'.

    Args:
        direction (int): Sens de rotation : 0 = arrière, 1 = avant.
        speed     (int): Vitesse de 0 à 100.

    Exemple:
        >> motorLeft(1, 50)
    """
    port.write(b'#MOTL,')
    port.write(bytes(str(direction), 'utf-8'))
    port.write(b',')
    port.write(bytes(str(speed), 'utf-8'))
    port.write(b'!\n')


# ─────────────────────────────────────────────────────────────────────
# Buzzer
# ─────────────────────────────────────────────────────────────────────

def buzzer(frequency, temps):
    """
    Déclenche le buzzer à une fréquence et une durée données.

    Envoie '#BUZ,<freq>,<temps>!'.

    Args:
        frequency (int): Fréquence en Hz (1 à 20000).
        temps     (int): Durée en millisecondes.
    """
    if 0 < frequency <= 20000:
        port.write(b'#BUZ,')
        port.write(bytes(str(frequency), 'utf-8'))
        port.write(b',')
        port.write(bytes(str(temps), 'utf-8'))
        port.write(b'!\n')
    else:
        print("Error frequency value")


def buzzerStop():
    """
    Arrête le buzzer immédiatement.

    Envoie '#BUZS!'.
    """
    port.write(b'#BUZS!\n')


# ─────────────────────────────────────────────────────────────────────
# LEDs
# ─────────────────────────────────────────────────────────────────────

def ledRGB(red_green_blue):
    """
    Contrôle la LED RGB du robot.

    Envoie '#RGB,<couleur>!' où couleur est une chaîne de 3 bits "RVB".
    Utiliser les constantes _RED, _GREEN, _BLUE ou des combinaisons
    comme "110" (jaune), "111" (blanc), etc.

    Args:
        red_green_blue (str): Chaîne de 3 caractères '0'/'1', ex: "100".

    Exemple:
        >> ledRGB("100")   # rouge
        >> ledRGB("011")   # cyan
    """
    port.write(b'#RGB,')
    port.write(bytes(red_green_blue, 'utf-8'))
    port.write(b'!\n')


def ledLowBatt(value):
    """
    Allume ou éteint la LED d'alerte batterie faible.

    Envoie '#LEDB,<value>!'.

    Args:
        value (int): 1 = allumée, 0 = éteinte.
    """
    if value in (0, 1):
        port.write(b'#LEDB,')
        port.write(bytes(str(value), 'utf-8'))
        port.write(b'!\n')
    else:
        print("erreur")


# ─────────────────────────────────────────────────────────────────────
# Encodeurs et odométrie
# ─────────────────────────────────────────────────────────────────────

def encoderLeft():
    """
    Lit la valeur brute de l'encodeur gauche.

    Envoie '#EDL!' et retourne la valeur en ticks.

    Returns:
        int: Nombre de ticks encodeur gauche.

    Exemple:
        >> encoderLeft()
    """
    port.flushInput()
    port.write(b'#EDL!\n')
    return int(readData())


def encoderRight():
    """
    Lit la valeur brute de l'encodeur droit.

    Envoie '#EDR!' et retourne la valeur en ticks.

    Returns:
        int: Nombre de ticks encodeur droit.

    Exemple:
        >> encoderRight()
    """
    port.flushInput()
    port.write(b'#EDR!\n')
    return int(readData())


def speedMotorRight():
    """
    Lit la vitesse courante du moteur droit.

    Envoie '#SWR!' et retourne la vitesse en ticks/s.

    Returns:
        int: Vitesse du moteur droit.

    Exemple:
        >> speedMotorRight()
    """
    port.flushInput()
    port.write(b'#SWR!\n')
    return int(readData())


def speedMotorLeft():
    """
    Lit la vitesse courante du moteur gauche.

    Envoie '#SWL!' et retourne la vitesse en ticks/s.

    Returns:
        int: Vitesse du moteur gauche.

    Exemple:
        >> speedMotorLeft()
    """
    port.flushInput()
    port.write(b'#SWL!\n')
    return int(readData())


def encoderReset():
    """
    Remet à zéro les deux encodeurs.

    Envoie '#ERZ!'.

    Exemple:
        >> encoderReset()
    """
    port.write(b'#ERZ!\n')


def orientation():
    """
    Calcule l'orientation du robot par odométrie différentielle.

    Utilise la différence entre l'encodeur droit et l'encodeur gauche,
    convertie en angle via la circonférence de roue et le rapport ticks/tour.

    Paramètres internes :
        - Diamètre roue : 32 mm → circonférence ≈ 100.5 mm
        - Résolution encodeur : 4200 ticks / tour
        - Facteur conversion : 0.00024 m/tick × 57.296 °/rad

    Returns:
        float: Orientation en degrés (positif = droite).

    Exemple:
        >> orientation()
    """
    stepOrientation = encoderRight() - encoderLeft()
    stepOrientation = stepOrientation * 0.00024   # conversion ticks → mètres d'arc
    angle = stepOrientation * 57.29577            # conversion radians → degrés
    return angle


# ─────────────────────────────────────────────────────────────────────
# Position
# ─────────────────────────────────────────────────────────────────────

def positionX():
    """
    Lit la position X du robot calculée par le microcontrôleur.

    Envoie '#POX!'.

    Returns:
        int: Position X en unité interne.

    Exemple:
        >> positionX()
    """
    port.flushInput()
    port.write(b'#POX!\n')
    return int(readData())


def positionY():
    """
    Lit la position Y du robot calculée par le microcontrôleur.

    Envoie '#POY!'.

    Returns:
        int: Position Y en unité interne.

    Exemple:
        >> positionY()
    """
    port.flushInput()
    port.write(b'#POY!\n')
    return int(readData())


# ─────────────────────────────────────────────────────────────────────
# Port série secondaire
# ─────────────────────────────────────────────────────────────────────

def serial2Write(data):
    """
    Envoie des données sur le port série secondaire (UART2 du microcontrôleur).

    Envoie '#SRLW,<data>!'. Utile pour communiquer avec un périphérique
    connecté sur l'UART2 du STM32.

    Args:
        data (bytes): Données à transmettre.

    Exemple:
        >> serial2Write(b"HELLO")
    """
    port.write(b"#SRLW,")
    port.write(data)
    port.write(b"!")


# ─────────────────────────────────────────────────────────────────────
# Fonctions utilitaires (usage interne)
# ─────────────────────────────────────────────────────────────────────

def check_speed(speed, distance):
    """
    Vérifie que la vitesse est compatible avec la distance pour le générateur
    de trapèze d'accélération du STM32.

    Le générateur trapézoïdal interne a un coefficient d'accélération fixe
    de 0.5. Si la vitesse est trop élevée par rapport à la distance, le robot
    ne peut pas accélérer et décélérer correctement.

    Condition : distance × 0.5 > speed²

    Args:
        speed    (int): Vitesse souhaitée.
        distance (int): Distance souhaitée.

    Returns:
        int: 1 si la combinaison est valide, 0 sinon.
    """
    acceleration = 0.5  # coefficient d'accélération du générateur trapèze STM32

    if distance == 0:
        return 1  # distance nulle → pas de mouvement, toujours valide

    if distance * acceleration > speed * speed:
        return 1
    else:
        print("Error speed to high!")
        return 0


def __convListToUint(liste):
    """
    Convertit une liste de caractères en entier non signé.

    Fonction interne — concatène les caractères de `liste` en chaîne
    puis convertit en int. Retourne None silencieusement si la conversion
    échoue (ValueError).

    Args:
        liste (list[str]): Liste de caractères représentant un entier.

    Returns:
        int | None: Entier converti, ou None si invalide.
    """
    a = ''.join(liste)
    try:
        return int(a)
    except ValueError:
        pass


def writeCommand(command):
    """
    Envoie une commande brute sur le port série (sans paramètre).

    Encapsule automatiquement la commande entre '#' et '!'.

    Args:
        command (bytes): Corps de la commande, ex: b'STP'.

    Exemple:
        >> writeCommand(b'STP')   # équivalent à stop()
    """
    port.write(b'#')
    port.write(command)
    port.write(b'!')


def readData():
    """
    Lit une ligne de réponse sur le port série et extrait la valeur.

    Le protocole de réponse est : $<valeur>\n
    La fonction cherche le délimiteur '$' et retourne ce qui suit
    jusqu'au caractère de fin de ligne '\n'.

    Returns:
        bytes: Valeur brute extraite (à convertir selon le contexte).
    """
    chaine = port.readline()
    pos1 = chaine.find(b'$')   # début de la valeur après '$'
    pos2 = chaine.find(b'\n')  # fin de la valeur avant '\n'
    return chaine[pos1 + 1:pos2]


def __convListToFloat(liste):
    """
    Convertit une liste de caractères en nombre flottant.

    Fonction interne — concatène les caractères de `liste` en chaîne
    puis convertit en float.

    Args:
        liste (list[str]): Liste de caractères représentant un flottant.

    Returns:
        float: Valeur convertie.
    """
    return float(''.join(liste))


# ─────────────────────────────────────────────────────────────────────
# Déplacements avec contrôle (fonctions avancées, usage interne)
# ─────────────────────────────────────────────────────────────────────
# Ces fonctions utilisent le générateur de trapèze d'accélération du
# STM32 pour des mouvements précis avec profil de vitesse. Elles
# nécessitent que controlEnable() soit appelé au préalable.

def forwardControl(speed, distance):
    """
    Avance avec contrôle de position et profil d'accélération trapézoïdal.

    Envoie '#MFC,<distance>,<speed>!'. Ne bloque pas en attente de fin
    de mouvement (non-bloquant).

    Args:
        speed    (int): Vitesse de déplacement.
        distance (int): Distance cible en mm.
    """
    controlEnable()
    if control_robot:
        port.write(b"#MFC,")
        port.write(bytes(str(int(distance)), 'utf-8'))
        port.write(b",")
        port.write(bytes(str(speed), 'utf-8'))
        port.write(b"!")
        port.flushInput()
    else:
        print("error : control robot disable")


def __backControl(speed, distance):
    """
    Recule avec contrôle de position et profil d'accélération trapézoïdal.

    Envoie '#MBC,<distance>,<speed>!'. Fonction interne (préfixe __).

    Args:
        speed    (int): Vitesse de déplacement.
        distance (int): Distance cible en mm.
    """
    controlEnable()
    if control_robot:
        port.write(b"#MBC,")
        port.write(bytes(str(int(distance)), 'utf-8'))
        port.write(b",")
        port.write(bytes(str(speed), 'utf-8'))
        port.write(b"!")
        port.flushInput()
    else:
        print("error : control robot disable")


def __turnLeftControl(speed, angle):
    """
    Rotation gauche avec contrôle d'angle et profil trapézoïdal.

    Envoie '#TLC,<angle>,<speed>!'. Fonction interne (préfixe __).

    Args:
        speed (int): Vitesse de rotation.
        angle (int): Angle cible en degrés.
    """
    controlEnable()
    if control_robot:
        port.write(b"#TLC,")
        port.write(bytes(str(int(angle)), 'utf-8'))
        port.write(b",")
        port.write(bytes(str(speed), 'utf-8'))
        port.write(b"!")
        port.flushInput()
    else:
        print("error : control robot disable")


def __turnRightControl(speed, angle):
    """
    Rotation droite avec contrôle d'angle et profil trapézoïdal.

    Envoie '#TRC,<angle>,<speed>!'. Fonction interne (préfixe __).

    Args:
        speed (int): Vitesse de rotation.
        angle (int): Angle cible en degrés.
    """
    controlEnable()
    if control_robot:
        port.write(b"#TRC,")
        port.write(bytes(str(int(angle)), 'utf-8'))
        port.write(b",")
        port.write(bytes(str(speed), 'utf-8'))
        port.write(b"!")
        port.flushInput()
    else:
        print("error : control robot disable")


# ─────────────────────────────────────────────────────────────────────
# État global
# ─────────────────────────────────────────────────────────────────────

# Drapeau d'activation du contrôle de position/orientation.
# False par défaut → les fonctions *Control() sont désactivées tant
# que controlEnable() n'a pas été appelée explicitement.
control_robot = False


# calibration des moteurs en boucle ouverte
calibMotors(_CALIB_MOTOR_RIGHT, _CALIB_MOTOR_LEFT)

# end file
