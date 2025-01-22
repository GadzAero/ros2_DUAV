# *CMD UTILES POUR GESTION RESEAU"

Pour vérifier à quoi est connecté chaque port :
 - ss -plant
Et pour choper un truc spécifique
 - ss -plant | grep 5760
Pour + :
 - ss -tulpn
(option -u = UDP ; -t = TCP ; -l = listening ; -p = processus ; -n = ne pas résoudre les noms)