######################################################
#  
#  http://www.macerobotics.com
#  Date : 02/02/2026
#  Version : 0.1
# 
#
#  MIT Licence

######################################################


import socket
import MR25
import time


# Configuration du serveur
SERVER_HOST = '10.3.141.1'  # L'adresse IP locale (localhost)
SERVER_PORT = 12345         # Le port sur lequel le serveur va écouter

# Création du socket UDP
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Lier le socket au port et à l'adresse
server_socket.bind((SERVER_HOST, SERVER_PORT))
print(f"Serveur UDP en écoute sur {SERVER_HOST}:{SERVER_PORT}")

MR25.stop()

i=0

while True:

    # Attente d'un message
    message, client_address = server_socket.recvfrom(1024)  # Taille maximale du message: 1024 octets
    print(f"Message reçu de {client_address}: {message.decode()}")
    print("compteur = ", i)
    i = i + 1
    commande = message.decode()

    print("COMMANDE = ", commande)

    if(commande == "MF"):
      print("Avancer robot")
      MR25.forward(35)
    elif(commande == "MB"):
      print("Reculer robot")
      MR25.back(35)
    elif(commande == "TR"):
      print("tourner à droite")
      MR25.turnRight(35)
    elif(commande == "TL"):
      print("tourner à gauche")
      MR25.turnLeft(35)
    elif(commande == "STP"):
      print("stop")
      MR25.stop()



# end of file