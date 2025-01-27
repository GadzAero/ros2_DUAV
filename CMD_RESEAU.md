# *CMD UTILES POUR GESTION RESEAU"

## SSH
Commmande pour se connecter sous windows ou linux :
 - ssh user_name@192.168.1.100
Raspi server :
 - ssh duav@192.168.8.226

Avec VS Code : https://code.visualstudio.com/docs/remote/ssh
Installer l'extension remote-ssh
Puis ouvrir la palette de commande et trouver
 - Remote-SSH: Connect to Host...
Se connecter normalement ensuite

## CONNEXION ROUTEUR
Nom du routeur : GL-MT300N-V2-035
IP du routeur : 192.168.8.1
MDP admin : duav222
MDP de connexion : goodlife

Tuto pour se connecter avec ubuntu_server :
 - https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line
L'erreur concernant Open vSwitch ne semble pas avoir d'impact

## TESTS RESEAU
Pour vérifier à quoi est connecté chaque port :
 - ss -plant
Et pour choper un truc spécifique
 - ss -plant | grep 5760
Pour + :
 - ss -tulpn
(option -u = UDP ; -t = TCP ; -l = listening ; -p = processus ; -n = ne pas résoudre les noms)