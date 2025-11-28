
//COSTRUTTORE
CNCstepper::CNCstepper(char iden, double velocita, int ratio, int res){
    
  this->steps = 0;

  //----------------------------------------
  //ratio
  this->ratio = double(ratio);

  //----------------------------------------
  //tali numeri derivano dal tipo di Shield
  if(iden == 'X' || iden == 'x') {stepPin = 2; dirPin = 5;}
  if(iden == 'Y' || iden == 'y') {stepPin = 3; dirPin = 6;}
  if(iden == 'Z' || iden == 'z') {stepPin = 4; dirPin = 7;}

  //----------------------------------------

  this->omega = velocita;  //velocita angolare, es. 360 gradi/s
  double Periodo = ((360.0) / omega) * 1000000.0;  //Periodo T in microsecondi  (T = 2pi / w)
  this->time = Periodo / (400.0 * this->ratio);  //tempo in microsecondi tra un impulso e l'altro
  
  //sanificazione di velocita
  if(this->time <= 600.0){  //perde passi (funziona male) per delay minori a 600
    Serial.begin(9600);
    while(true) {Serial.print("->Errore: velocità angolare non raggiungibile, time =  "); Serial.print(time); Serial.println(" <= 600 !");}
  }

  //----------------------------------------
  //di default il motore (considerabile come un servomotore) può raggiungere un angolo appartenente a [-360, 360]
  this->minAngle = -360.0;
  this->maxAngle = 360.0;
  
  //----------------------------------------
  //resolution è un valore reale per cercare di evitare perdità di precisone
  if(res != 1 && res != 2 && res != 4 && res != 8 && res != 16){
    Serial.begin(9600);
    while(true) Serial.println("->Errore: risoluzione non valida");
  }
  else this->resolution = double(res);
  
}

//===================================================================================================================================

void CNCstepper::inizializza(){
  //-->funzione che:
  //setta come output i pin opportuni. DA INVOCARE nel setup
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}


//===================================================================================================================================

void CNCstepper::set_vincolo(double min, double max){
  if(min < -360.0 || min >= 360.0){
    Serial.begin(9600);
    while(true) Serial.print("->Errore: vincolo non valido");
  }

  if(max <= -360.0 || max > 360.0){
    Serial.begin(9600);
    while(true) Serial.print("->Errore: vincolo non valido");
  }

  this->minAngle = min;
  this->maxAngle = max;
}
//===================================================================================================================================

void CNCstepper::angle_iter(double angle){
  //-->funzione che:
  //blocca l'esecuzione del programma: fino a quando l'angolo non è raggiunto non verrà eseguito nessun altro codice

  if(angle < minAngle) angle = minAngle;
  if(angle > maxAngle) angle = maxAngle;

  //proporzione per ottenere gli steps che corrispondono all'angolo desiderato
  //int angle_steps = int( (angle * 20.0 * resolution / 36.0 );
  int angle_steps = int(  double(angle) / double( 36.0 / (20.0 * resolution * this->ratio) )  );
  
  if(angle_steps == this->steps){
    //angolo raggiunto
    return;
  }

  int local_steps = 0;  //steps da fare per raggiungere l'angolo

  if(this->steps < angle_steps){
    digitalWrite(dirPin, LOW);  //deve girare in senso antiorario per raggiungerlo
    local_steps = angle_steps - this->steps;
  }

  if(this->steps > angle_steps){
    digitalWrite(dirPin, HIGH);  //deve girare in senso orario per raggiungerlo
    local_steps = this->steps - angle_steps;
  }

  //importante questo
  this->steps = angle_steps;
  
  for(int i=0;  i < (local_steps);  i++){    
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(time);
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(time);    
  }

  //ritardo per non far partire subito gli altri motori
  //delay(1500);
  delayMicroseconds(time);
}

//===================================================================================================================================

void CNCstepper::angle_configure(double angle, double percentuale, double ritardo){
  //-->funzione che:
  //raggiunge l'angolo desiderato, bloccando il programma, in quanto si ha un for
  //non viene però modificato il membro privato "steps"
  //per cui ruota il motore di "angle" a prescindere se rispetto ad un sistema di riferimento

  //proporzione per ottenere gli steps che corrispondono all'angolo desiderato
  //int angle_steps = int( (angle * 20.0 * resolution) / 36.0 );
  int angle_steps = int(  double(angle) / double( 36.0 / (20.0 * resolution * this->ratio) )  );
  
  if(angle_steps == this->steps){
    //angolo raggiunto
    return;
  }

  int local_steps = 0;  //steps da fare per raggiungere l'angolo

  if(this->steps < angle_steps){
    digitalWrite(dirPin, LOW);  //deve girare in senso antiorario per raggiungerlo
    local_steps = angle_steps - this->steps;
  }

  if(this->steps > angle_steps){
    digitalWrite(dirPin, HIGH);  //deve girare in senso orario per raggiungerlo
    local_steps = this->steps - angle_steps;
  }

  for(int i=0;  i < (local_steps);  i++){
    double tempo = funzione_tempo(i, local_steps, time, percentuale, ritardo);
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(tempo);
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(tempo);    
  }

  //ritardo per non far partire subito gli altri motori
  delay(1000);
}

//===================================================================================================================================

double CNCstepper::funzione_tempo(double x, double steps, double time0, double percentuale, double ritardo){
  //-- x è a che passo è tra [0, steps]
  //-- calcola un ritardo per avere una rotazione accelerata e decelerata
  //-- suppondendo percentuale sia pari a 20%, ciò significa che verrà calcolato un ritardo che cala progressivamente fino alla velocità ordinaria
  //   per il primo 20% della rotazione e l'ultimo 80%
  //-- l'utente può inserire quanto ritardare, tipo ritardo = 2, allora il max ritardo sarà 2*time

  //ATTENZIONE: si può rendere piu efficiente, evitando di calcolare p1 e p2 ogni volta !

  if(percentuale <= 0.0 || percentuale > 50.0){
    Serial.begin(9600);
    while(true) Serial.println("->Errore: funzione_tempo");
  }
  if(ritardo < 1.0 || ritardo > 10.0){
    Serial.begin(9600);
    while(true) Serial.println("->Errore: funzione_tempo");
  }

  //double p1 = (  (4.0 * steps * steps)  /  ( ((25.0 * ritardo) - 25.0)*time0 )  );
  double p1 = -1.0 * (   ( ((percentuale*percentuale) - (100.0*percentuale))*(steps*steps) )  /  ((10000.0*ritardo - 10000.0) * time0)    );
  double p2 = (ritardo * time0) - ( (steps*steps) / (4.0*p1) );

  double tempo = (( (x - (steps/2.0))*(x - (steps/2.0)) ) / p1) + p2;  //la y nella funzione

  if(tempo <= time0) return time0;
  else return tempo;
}


void CNCstepper::angle_iter_acc(double angle, double percentuale, double ritardo){
  //-->funzione che:
  //blocca l'esecuzione del programma: fino a quando l'angolo non è raggiunto non verrà eseguito nessun altro codice

  if(angle < minAngle) angle = minAngle;
  if(angle > maxAngle) angle = maxAngle;

  //proporzione per ottenere gli steps che corrispondono all'angolo desiderato
  //int angle_steps = int( (angle * 20.0 * resolution) / 36.0 );  //PERDITA DI PRECISIONE (credo sia inevitabile)
  int angle_steps = int(  double(angle) / double( 36.0 / (20.0 * resolution * this->ratio) )  );

  if(angle_steps == this->steps){
    //angolo raggiunto
    return;
  }

  int local_steps = 0;  //steps da fare per raggiungere l'angolo

  if(this->steps < angle_steps){
    digitalWrite(dirPin, LOW);  //deve girare in senso antiorario per raggiungerlo
    local_steps = angle_steps - this->steps;
  }

  if(this->steps > angle_steps){
    digitalWrite(dirPin, HIGH);  //deve girare in senso orario per raggiungerlo
    local_steps = this->steps - angle_steps;
  }

  //importante questo
  this->steps = angle_steps;
  
  for(int i=0;  i < (local_steps);  i++){
    double tempo = funzione_tempo(i, local_steps, time, percentuale, ritardo);
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(tempo);
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(tempo);    
  }

  //ritardo per non far partire subito gli altri motori
  //delay(1500);
  delayMicroseconds(time);
}

