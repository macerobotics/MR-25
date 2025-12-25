import pygame  # Importation de la bibliothèque Pygame pour gérer une manette
from time import sleep  
import sys  
import socket


# Configuration du client
SERVER_HOST = '10.3.141.1'  # L'adresse IP du serveur (localhost ici)
SERVER_PORT = 12345        # Le port auquel le serveur écoute


# Création du socket UDP
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Initialisation de Pygame pour préparer à l'utilisation de ses modules.
pygame.init()

# Compte le nombre de manettes connectées à l'ordinateur.
joystick_count = pygame.joystick.get_count()  

# Vérification si aucun joystick n'est détecté
if joystick_count == 0:
    print("Erreur, pas de joystick détecté !")  # Affiche un message d'erreur si aucun joystick n'est connecté.
    pygame.quit()  # Arrête Pygame proprement.
    sys.exit()  # Quitte le programme.

else:
    # Si un joystick est détecté, initialisation de celui-ci.
    joystick = pygame.joystick.Joystick(0)  # Récupère le premier joystick (index 0).
    joystick.init()  # Initialise le joystick pour pouvoir l'utiliser.

# Récupération du nombre de "hat switches" (poignées de contrôle directionnelles, souvent utilisées pour la direction).
hats = joystick.get_numhats()



# Fonction pour traiter les mouvements de la "hat switch" (direction du joystick)
def getHat(number):

    
    # Si le "hat" n'est pas en position (0,0), il a été déplacé
    if joystick.get_hat(number) != (0, 0):

        # Si le mouvement sur l'axe Y du "hat" est vers le haut
        if joystick.get_hat(number)[1]:
            print("Avancer")  # Afficher "Avancer" lorsque le mouvement est vers le haut.
            message = "MF" # message a envoyer par UDP pour avancer
            client_socket.sendto(message.encode(), (SERVER_HOST, SERVER_PORT))


        # Si le mouvement sur l'axe Y du "hat" est vers le bas
        if joystick.get_hat(number)[1] == -1:
            print("reculer")  # Afficher "reculer" lorsque le mouvement est vers le bas.
            message = "MB" # message a envoyer par UDP pour avancer
            client_socket.sendto(message.encode(), (SERVER_HOST, SERVER_PORT))
        
        # Si le mouvement sur l'axe X du "hat" est vers la droite
        if joystick.get_hat(number)[0] == 1:
            print("tourner à droite")  # Afficher "tourner à droite" lorsque le mouvement est vers la droite.
            message = "TR" # message a envoyer par UDP pour avancer
            client_socket.sendto(message.encode(), (SERVER_HOST, SERVER_PORT))


        # Si le mouvement sur l'axe X du "hat" est vers la gauche
        if joystick.get_hat(number)[0] == -1:
            print("tourner à gauche")  # Afficher "tourner à gauche" lorsque le mouvement est vers la gauche.
            message = "TL" # message a envoyer par UDP pour avancer
            client_socket.sendto(message.encode(), (SERVER_HOST, SERVER_PORT))
    else:
        print("STOP")
        message = "STP" # message a envoyer par UDP pour avancer
        client_socket.sendto(message.encode(), (SERVER_HOST, SERVER_PORT))


# Boucle principale du programme
while True:
    for event in pygame.event.get():  # Écoute tous les événements (comme les mouvements du joystick).
        if event.type == pygame.QUIT:  # Si l'utilisateur ferme la fenêtre
            pygame.quit()  # Arrête Pygame proprement.
            sys.exit()  # Quitte le programme.
        elif event.type == pygame.JOYHATMOTION :
            print("Joystick button pressed.")
            print(hats)
            #  Si le joystick a des hats (directionnels)
            if hats != 0:
              for i in range(hats):  # Parcours chaque hat 
                getHat(i)  # Appelle la fonction getHat pour traiter les mouvements de chaque hat.'''

    
    sleep(0.3)  # Petite pause pour réduire l'utilisation du processeur.
