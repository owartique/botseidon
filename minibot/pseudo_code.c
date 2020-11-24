/*
Pseudo code pour le minibot. C'est pas du tout correct niveau syntaxe/pointeurs/etc mais je voulais juste donner une idée d'algorithme qu'on pourrait faire.
Comme vous avez déja chipoté quelques heures dessus, dites moi si vous pensez que ça pourrait marcher.
J'ai considéré que le minibot avance tout droit tant qu'il y a pas d'obstacle devant lui et qu'il s'arrête si il y a un obstacle.
*/

// Déclaration de quelques variables utiles
const int DISTANCE = 300; //distance (en mm) pour laquelle on veut eviter des obstacle
const int ALPHA = 10; //angle (en degrés) pour lequel on s'intéresse
bool isStopped; // variable qui permet de savoir si le robot roule ou pas
bool obstacle; // variable qui permet de savoir si il y a un obstacle
bool firstTime = true; // variable qui permet d'executer qu'une fois le début du code
array tableau[][]; //jsp comment on déclare un tableau

int main(){
    while(!ctrl_c_pressed){ // boucle infinie tant qu'on appuie pas sur ctrl+c

        if(firstTime){ //exécuté qu'une seule fois au début
            // on allume le lidar
            lidar.start();
            // on enregistre les données du premier tour dans un tableau
            tableau[?][3] = lidar.getScanData();
            // on regarge si il y a pas une obstacle (voir fonction créée plus bas)
            obstacle = checkObstacle(tableau);
            if(!obstacle){
                //si il y a pas d'obstacle alors on peut lancer les moteurs
                motor.start();
                isStopped = false;
                obstacle = false;
            }
            else{
                //si il y a un obstacle alors les moteurs restent éteints
                isStopped = true;
                obstacle = true;
            }
            firstTime = false;
        }
        else{
            // on demande les dernières valeurs du lidar
            tableau[?][3] = lidar.getScanData();
            obstacle = checkObstacle(tableau);
            if(obstacle & !isStopped){
                //si il y a un obstacle et qu'il roule alors on le stoppe
                motor.stop();
                isStopped = true;
            }
            else if(obstacle & isStopped){
                //si il y a un osbtacle et qu'il est deja arreté alors on fait rien
                //else if inutile mais je l'ai mis pour que ce soit + compréhensible
            }
            else if(!obstacle & isStopped){
                //si il y a pas d'obstacle et qu'il est arreté alors on le fait avancer
                motor.start();
                isStopped = false;
            }
            else if(!obstacle & !isStopped){
                //si il y a pas d'obstacle et qu'il roule alors on fait rien
                // else if inutile mais je l'ai mis pour que ce soit + compréhensible
            }
        }
    }
}

// fonction qui parcoure les nodes du tour et qui renvoie true si au moins un node a une distance plus petite que DISTANCE et un angle compris entre +- ALPHA
bool checkObstacle(tableau){
    for(int i=0;i<tableau.length();i++){
        if(tableau[i][0]<ALPHA & tableau[i][0]>(360-ALPHA)){	// si l'angle nous intéresse
            if(tableau[i][1]<DISTANCE & tableau[i][2]>0){		// si moins de 30cm et que c'est pas une quality de merde
                    return true; // pas besoin de continuer à regarder d'autres nodes car il faut d'office s'arrêter
            }
        }
    }
    // si on arrive jusqu'ici ça veut dire qu'il y a aucun obstacle donc on renvoie false
    return false;
}


