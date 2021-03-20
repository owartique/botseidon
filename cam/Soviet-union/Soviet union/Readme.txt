Yo bg,
Ces fichiers contiennent tout ce qu'il faut pour calibrer et tester ta
détection N-S pour la compétition.
-L'Imggrab sert à ouvrir la caméra et à prendre 
des images nécessaires à la calibration (à l'aide d'un chessboard)
-Le calibrate chess permet d'analyser les images et de ressortir la matrice
de déformations, qu'il stock ensuite dans un fichier "Calibration.pclk"

Le fichier de calibration est employé dans l'initialisation de la détection
Le socket ne sert pas vraiment à change chose ici vu que ROS sera employé je suppose
Fais bien gaffe à installer opencv-contrib-python via pip3 (ou avec thonny).

Débizou
Bon chans pour votr progé
