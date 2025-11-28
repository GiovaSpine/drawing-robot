

#include "gcode.h"
#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;


//ASSENTE è un valore che non potrà mai essere scritto nel gcode, in quanto irragiungibile dal robot
//o perlomeno so che non si troverà mai nei codici G00, G01, G02 e G03
#define ASSENTE 10000.0


codiceG::codiceG() {  //COSTRUTTORE DEFAULT
    strcpy(str, "G0-\0");
    X = ASSENTE; Y = ASSENTE; Z = ASSENTE; I = ASSENTE; J = ASSENTE;
}


void Stampa(codiceG vett[], int len){
    for(int i=0; i<len; i++){
        cout << vett[i].str << ' ' << vett[i].X << ' ' << vett[i].Y << ' ' << vett[i].Z << ' ' << vett[i].I << ' ' << vett[i].J << endl;
    }
}


gcode::gcode(const char *str, double X, double Y, double width, double height) {

    directory = new char[strlen(str) + 1];
    strcpy(directory, str);

    //vettore trascinamento Vt = (Xt,Yt)
    //vettore nel SDR del robot, indica la posizione dell'origine (0,0) del foglio
    this->Xt = X;
    this->Yt = Y;

    //dimensioni del foglio, quelle default sono di un foglio A4
    this->width = width;
    this->height = height;

    //numero di codici del tipo G00, G01, G02 o G03
    this->n = 0;

}


gcode::~gcode() {
    delete[] directory;  // liberazione stringa allocata con new[]
    delete[] buffer;  // liberazione array di codiceG
}


void gcode::leggi() {

    ifstream iff;

    iff.open(directory);
    if (!iff) {
        cerr << "Il file non puo' essere aperto" << endl;
        exit(1); // funzione exit
    }

    char aux;
    int count = 0;

    while(iff){
        iff >> aux;
        if(aux == 'G'){
            //attenzione, ci possono essere parole come "Generated" oppure "G21"
            //leggo i caratteri succesivi a G, devono essere 00, 01, 02, o 03
            char A;
            iff >> A;
            char B;
            iff >> B;
            if(A == '0') count++;
        }
    }
    iff.close();

    //n numero di comandi che dobbiamo eseguire
    this->n = count;

    //memorizzo i comandi validi lì
    this->buffer = new codiceG[n];


    iff.open(directory);
    if (!iff) {
        cerr << "Il file non puo' essere aperto" << endl;
        exit(1); // funzione exit
    }

    bool controllore = true;
    aux = '-';
    int k = 0;

    while(iff){

        if(controllore) iff >> aux;
        else aux == 'G';

        if(aux == 'G'){
            //attenzione, ci possono essere frasi come "Generated" oppure G21
            //leggo il carattere succesivo a G, deve essere 0
            char A; iff >> A;

            if(A == '0'){
                //allora si ha un comando G0- (dove '-' può essere 0,1,2 o 3)
                char i;
                iff >> i;
                buffer[k].str[2] = i;
                //ci aspettiamo di trovare una coordinata tra queste: X,Y,Z,I o J

                for(int j=0; j<5; j++){
                    iff >> i;
                    if(i == 'G'){
                        controllore = false;
                        break;
                    }
                    if(i == 'X'){ double num; iff >> num; buffer[k].X = num;}
                    if(i == 'Y'){ double num; iff >> num; buffer[k].Y = num;}
                    if(i == 'Z'){ double num; iff >> num, buffer[k].Z = num;}
                    if(i == 'I'){ double num; iff >> num; buffer[k].I = num;}
                    if(i == 'J'){ double num; iff >> num; buffer[k].J = num;}
                    if(i != 'G') controllore = true;
                }

                k++;  //importante
            }

        }


    }
    iff.close();

}


void gcode::scrivi() {

    char *str = new char[strlen(directory) + 1];
    strcpy(str, directory);

    //devo modificare l'intestazione con txt
    str[strlen(directory) - 1] = 't';
    str[strlen(directory) - 2] = 'x';
    str[strlen(directory) - 3] = 't';


    ofstream off;
    off.open(str);
    if (!off) {
        cerr << "Il file non puo' essere aperto" << endl;
        exit(1); // funzione exit
    }

    for(int i=0; i < n; i++){
        //scrivo G00, G01, G02 o G03
        off << buffer[i].str;
        off << "( ";

        if(buffer[i].X == ASSENTE) off << "ASSENTE";
        else off << Xt + buffer[i].Y;
        off << ", ";
        if(buffer[i].Y == ASSENTE) off << "ASSENTE";
        else off << Yt - buffer[i].X;
        off << ", ";
        if(buffer[i].Z == ASSENTE) off << "ASSENTE";
        else off << buffer[i].Z;

        //ATTENZIONE! ci sono più versioni di G02 e G03
        //quella nostra è che I e J sono relativi al vettore posizione attuale! non vanno quindi sommati con il vettore trascinamento
        //vanno cambiati di direzione però
        if(buffer[i].str[2] == '2' || buffer[i].str[2] == '3'){
            //ci sono a prescindere I e J
            off << ", ";
            off << buffer[i].J;
            off << ", ";
            off << -1.0 * buffer[i].I;
        }
        off << " )";
        off << ';' <<'\n';

    }

}