# *CMD UTILES POUR LA MISE EN PLACE*

## QGroundControl
Lien vers installation :
 - https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
Lancer le logiciel :
 - ./QGroundControl.AppImage
Sawrming
 - https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/swarming_ardupilot.md

## MAVROS
Installation (a faire depuis la source et pas depuis les binairies):
 - https://github.com/mavlink/mavros/blob/ros2/mavros/README.md
Si tu rencontre un pb lors de l'installation install_geographiclib_datasets.sh, exécute a la racine du ws ros2 :
 - wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
 - sudo bash install_geographiclib_datasets.sh
puis continue normalement

