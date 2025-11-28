#include <iostream>
#include "gcode.h"

using namespace std;

int main() {
    //di norma è 140.0 105.0 per gli A4 verticali

    //dobbiamo convertire il GCODE generato con Inkscape
    //da un file di testo .txt in una serie di chiamate di funzioni C++
    //es. convertire "G03 X97.505069 Y259.737213 Z-1.000000 I0.047291 J-60.384406 F400.000000"
    //a "G03(363.237, 42.4949, -1, -60.3844, -0.047291);"

    //(ignoriamo le informazioni sulla velocià perché non dobbiamo tagliare/fresare)

    //come vediamo dall'esempio di sopra, non è una semplice conversione di testo ma c'è da applicare
    // un cambio di sistema di rifermento da sistema di riferimento del disegno a quello del robot

    //il vettore nel sistema di riferimento del robot, che punta verso il sistema di riferimento del foglio
    //è: (103.5, 140)
    //il sistema di riferimento del foglio deve avere (0, 0) in basso a sinistra (NON in alto a sinistra)

    double sdr_foglio_x = 103.5;
    double sdr_foglio_y = 140;
    gcode oggetto("disegni/Smile.ngc", sdr_foglio_x, sdr_foglio_y);
    oggetto.leggi();
    oggetto.scrivi();

    return 0;
}
