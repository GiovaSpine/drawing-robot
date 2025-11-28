
//lunghezza link
#define d2 220.0
#define d4 190.0

//quanto deve alzarsi e abbbasarsi dal piano l'end effector ?
#define ALZATA 20.0
#define ABBASSATA 0.0

//======================================================================================================================

//INIZIALIZZAZIONE
Manipulator::Manipulator(double raggio) : raggioEndEffector(raggio), joint1('X', 65.0, 20, 4), joint2('Y', 90.0, 15, 4), joint3('Z', 180.0, 1, 1) {
  joint1.set_vincolo(-60.0, 135.0);
  joint2.set_vincolo(-60.0, 135.0);
  joint3.set_vincolo(0.0, ALZATA);
}

void Manipulator::inizializza(){
  joint1.inizializza(); joint2.inizializza(); joint3.inizializza();
  
  //il robot si trova nella posizione home, la posizione indicata dal diagramma cinematico
  alza_EF();
  delay(500);
  joint1.angle_configure(90.0, 20.0, 4.0);
  joint2.angle_configure(90.0, 20.0, 4.0);
  
  this->theta1 = 0.0;
  this->theta2 = 0.0;
  this->theta3 = ALZATA;  //theta3 in generale assume ABBASSATA oppure ALZATA

  this->current_x = d2 + d4 + this->raggioEndEffector;
  this->current_y = 0.0;
  this->current_z = false;
}

//TERMINAZIONE
void Manipulator::home(){

  delay(1000);
  joint3.angle_iter(ALZATA);  //importante che lo faccia per prima, sennò disegna mentre va verso la home
  joint1.angle_iter(0.0);
  joint2.angle_iter(0.0);
  delay(1000);
  joint2.angle_configure(-90.0, 30.0, 3.0);
  joint1.angle_configure(-90.0, 30.0, 3.0);
  joint3.angle_configure(ABBASSATA, 30.0, 3.0);

  //blocca programma
  while(true) Serial.println("->PROGRAMMA BLOCCATO");
}

//======================================================================================================================

void Manipulator::trigonometriaV1(double x0, double y0){
  
  if(x0 < 0){
    Serial.begin(9600);
    while(true) Serial.print("->Errore: valore x0 non ammissibile: "); Serial.print(x0); Serial.println(" < 0");
  }

  double D4 = d4 + this->raggioEndEffector;

  double ipotenusa = sqrt((x0*x0) + (y0*y0));

  double beta = atan(abs(y0) / x0);
  double alfa2 = acos( ((D4*D4)+(d2*d2)-(ipotenusa*ipotenusa)) / (2.0*d2*D4) );
  double alfa1 = acos( ((ipotenusa*ipotenusa)+(d2*d2)-(D4*D4)) / (2.0*ipotenusa*d2) );
  //conversione da radianti a gradi
  beta = (beta*180.0) / pi;
  alfa2 = (alfa2*180.0) / pi;
  alfa1 = (alfa1*180.0) / pi;
  
  //CASO 1)
  if(y0 >= 0.0){
    theta1 = beta - alfa1;
    theta2 = 180.0 - alfa2;
  }
  //CASO 2)
  if(y0 < 0.0){
    theta1 = -1.0 * (beta + alfa1);
    theta2 = 180.0 - alfa2;
  }

}


double Manipulator::norma(double x, double y){
  //norma di un vettore
  return sqrt( (x*x) + (y*y) );
}
//======================================================================================================================
//alza e abbassa EF (End Effector)
//--> si ha parametro "ALZATA" e "ABBASSATA"

//alza l'end effector dal piano
void Manipulator::alza_EF(){
  joint3.angle_iter(ALZATA);  
}
//abbassa l'end effector per toccare il piano
void Manipulator::abbassa_EF(){
  joint3.angle_iter(ABBASSATA); 
}

//======================================================================================================================
//PRIVATA
void Manipulator::raggiungi_punto_iter(double x, double y, bool z){
  //FUNZIONE PRIVATA, c'è differenza da quella pubblica

  trigonometriaV1(x, y);
  //il risultato è la modifica di theta1 e theta2

  //-- considero a parte il joint3:
  // z == true, l'end effector deve toccare il piano
  // z == false, l'end effector NON deve toccare il piano
  if(z == true) abbassa_EF();
  else alza_EF();

  joint2.angle_iter(theta2);
  joint1.angle_iter(theta1);
  
  this->current_x = x;
  this->current_y = y;
  this->current_z = z;
}

//======================================================================================================================
//METODI PUBBLICI PER MUOVERE IL ROBOT

void Manipulator::raggiungi_punto(double x, double y){
  //funzione che viene usata dall'utente, fare attenzione!
  //c'è differenza da quella privata

  trigonometriaV1(x, y);
  //il risultato è la modifica di theta1 e theta2

  //il joint3 non viene toccato dall'utente! è sempre in ALZATA e viene abbasato solo da alcune funzioni

  joint2.angle_iter_acc(theta2, 20.0, 2.5);
  joint1.angle_iter_acc(theta1, 20.0, 2.5);
  
  this->current_x = x;
  this->current_y = y;

  //ritardo per non far partire subito il robot
  delay(500);
}

void Manipulator::punto(){
  //funzione che disegna un punto nella posizione attuale
  abbassa_EF();
  delay(100);
  alza_EF();
}


void Manipulator::segmento(double x, double y){
  //->funzione che:
  //disegna un segmento tra posizione attuale e posizione inserita come argomento, tramite combinazione convessa
  //combinazione convessa: tutti i punti 'x', tali che x = a + t(b-a)   punti a,b   t appartiente a [0,1]

  //parte toccando il piano dalla posizione attuale (si presume il robot sia in ALZATA)
  abbassa_EF();

  double pos_A[2] = {this->current_x, this->current_y};
  double pos_B[2] = {x, y};

  //vorremmo che venisse raggiunto un punto ogni 1/3 millimetro (c'è il * 3)
  int passaggi = (int(norma(pos_B[0] - pos_A[0], pos_B[1] - pos_A[1]))) * 3;

  for(int t=0; t<=passaggi; t++){
    double pos_tBA[2] = { ( double(t)/double(passaggi) )*(pos_B[0] - pos_A[0]), ( double(t)/double(passaggi) )*(pos_B[1] - pos_A[1])};
    double pos_X = pos_A[0] + pos_tBA[0];
    double pos_Y = pos_A[1] + pos_tBA[1];
    this->raggiungi_punto_iter(pos_X, pos_Y, true);
  }
  this->raggiungi_punto_iter(x, y, true);

  //arrivato alla fine alza l'end effector
  alza_EF();
}


