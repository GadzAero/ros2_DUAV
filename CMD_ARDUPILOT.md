# *CMD UTILES POUR ARDUPILOT*

## LANCER UNE SIMU
Lancer un SITL
 - sim_vehicle.py -v ArduCopter --map --console
Ou pour pas etre encombré :
 - sim_vehicle.py -v ArduCopter
Sans Mavproxy
 - sim_vehicle.py -v ArduCopter --no-mavproxy
Pour en lancer 2 :
 - sim_vehicle.py -v ArduCopter --no-mavproxy --instance=0 (5760 5762 5763)
 - sim_vehicle.py -v ArduCopter --no-mavproxy --instance=1 (5770 5772 5773)

=> Si vous lancer le sitl dans linux et que vous ouvrez QGS sur windows, la connexion est automatique (a condition d'avoir setup la connectionTCP sur un des pourt Mavlink du STIl en automatique dans les paramètre QGS)

## MAVProxy
Commande de lancement :
 - mavproxy.py --master=tcp:127.0.0.1:5760 --out=tcp:127.0.0.1:14550
(127.0.0.1 est l’adresse de boucle locale (localhost) : Si tu utilises 127.0.0.1, seules les applications locales (sur la même machine) peuvent se connecter)
(0.0.0.0 signifie « toutes les interfaces réseau » : Si tu configures un programme pour écouter ou émettre sur 0.0.0.0, il sera accessible depuis n’importe quelle machine qui peut atteindre ton ordinateur via le réseau.)
Pour rediriger vers l'hote windows :
 - mavproxy.py --master=tcp:127.0.0.1:5763 --out=udp:192.168.1.9:14550

