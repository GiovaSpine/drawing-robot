


void Manipulator::G00(double X, double Y, double Z){
  //è un po' come se fosse la nostra raggiungi_punto_iter privata

  //tratto prima il joint3
  if(Z != ASSENTE){
    //allora vogliamo modificare Z
    if(Z == -1.0) {abbassa_EF(); this->current_z = true;}
    if(Z != -1.0) {alza_EF(); this->current_z = false;}
  }

  if(X == ASSENTE && Y == ASSENTE){
    //allora vuol dire che andava modificato solo Z
    return;
  }

  if(X == ASSENTE) X = this->current_x;
  if(Y == ASSENTE) Y = this->current_y;

  trigonometriaV1(X, Y);
  //il risultato è la modifica di theta1 e theta2

  joint2.angle_iter_acc(theta2, 15.0, 3.0);
  delayMicroseconds(1000);
  joint1.angle_iter_acc(theta1, 15.0, 3.0);

  this->current_x = X;
  this->current_y = Y;

}

//=====================================================================================================================================

void Manipulator::G01(double X = ASSENTE, double Y = ASSENTE, double Z = ASSENTE){

  //tratto prima il joint3
  if(Z != ASSENTE){
    //se Z == ASSENTE allora vogliamo mantenere current_z
    //allora vogliamo modificare Z (ci sta non faccia nulla poi abbassa e alza)
    if(Z == -1.0) {abbassa_EF(); this->current_z = true;}
    if(Z != -1.0) {alza_EF(); this->current_z = false;}
  }

  if(X == ASSENTE && Y == ASSENTE) return;  //nel comando G01 non è possibile che ciò accada! (per noi non accade)

  if(X == ASSENTE) X = this->current_x;
  if(Y == ASSENTE) Y = this->current_y;

  double pos_A[2] = {this->current_x, this->current_y};
  double pos_B[2] = {X, Y};

  //vorremmo che venisse raggiunto un punto ogni 1/5 millimetro (c'è il * 5)
  int passaggi = (int(norma(pos_B[0] - pos_A[0], pos_B[1] - pos_A[1]))) * 5;

  for(int t=0; t<=passaggi; t++){
    double pos_tBA[2] = { ( double(t)/double(passaggi) )*(pos_B[0] - pos_A[0]), ( double(t)/double(passaggi) )*(pos_B[1] - pos_A[1])};
    double pos_X = pos_A[0] + pos_tBA[0];
    double pos_Y = pos_A[1] + pos_tBA[1];

    raggiungi_punto_iter(pos_X, pos_Y, this->current_z);
  }
  //a causa di perdità di precisione:
  raggiungi_punto_iter(X, Y, this->current_z);

}


//=====================================================================================================================================

void Manipulator::G02(double X, double Y, double Z, double I, double J){
  //ATTENZIONE! ci sono più versioni di G02 e G03
  //la nostra è quella dove I,J sono RELATIVI alla posizione attuale

  if(Z != ASSENTE){
    //se Z == ASSENTE allora vogliamo mantenere current_z
    //allora vogliamo modificare Z (ci sta non faccia nulla poi abbassa e alza)
    if(Z == -1.0) {abbassa_EF(); this->current_z = true;}  //<-- PARAMETRO!
    if(Z != -1.0) {alza_EF(); this->current_z = false;}
    delayMicroseconds(600);
  }

  //Interpolazione circolare in senso ORARIO

  double R = norma(I,J);
  {
    //--VERIFICO CHE IL COMANDO VALGA--
    //vettore relativo alla posizione attuale
    double X_primo = X - current_x;
    double Y_primo = Y - current_y;
    //tecnicamente la sua distanza da I,J dovrebbe essere uguale a R (distanza della posizione attuale da I,J)
    double d = norma(I - X_primo, J - Y_primo);

    if(abs(R - d) >= 0.5){
      //allora c'è una differenza notevole tra i due raggi, è assurdo/impossibile
      //ATTENZIONE! si possono avere cerchi MOLTO GRANDI, per cui la differenza tra i raggi cresce
      //ma poichè i raggi sono molto lunghi tale differenza è nulla in confronto
      Serial.begin(9600);
      while(true){Serial.println("->ERRORE: comando G02 non valido");}
    }
  }

  //coordinate globali del cerchio
  double Xc = current_x + I;
  double Yc = current_y + J;

  //coordinate della posizione attuale e X,Y relative al cerchio
  double X0 = current_x - Xc;
  double Y0 = current_y - Yc;
  double X1 = X - Xc;
  double Y1 = Y - Yc;

  //vogliamo andare da ti --> tf (tecnicamente ti > tf, per andare in senso ORARIO)
  //ATTENZIONE! la differenza tra tf e ti può essere piccolissima
  double ti = atan2(Y0 , X0);
  double tf = atan2(Y1 , X1);
  //atan2 resituisce una angolo tra ]-pi, pi] e ciò non ci piace
  //li converto a [0.0, 2*pi[
  if(ti < 0.0) ti = (2.0*pi + ti);
  if(tf < 0.0) tf = (2.0*pi + tf);
  
  //attenzione tf può essere maggiore di ti
  //dovremo percorrere un ARCO più lungo -->(2 possibili archi)

  //angolo che dobbiamo percorrere
  double angolo = abs(ti - tf);  //ottengo l'angolo COMPRESO
  if(tf > ti){  //MA se tf>ti dobbiamo percorrere "l'opposto"
    Serial.println("->ATTENZIONE tf > ti");
    angolo = 2*pi - angolo;  
  }

  double arco = R * angolo;  //dove abs(tf - ti) è l'angolo compreso tra i 2
  //ogni 1/10 di millimetro raggiungiamo un punto (c'è il * 10.0 e / 10.0)
  int passaggi = int(arco * 10.0);

  for(int i=0; i <= passaggi; i++){
    double de_angle = (double(i) / 10.0) / R;

    double pos_X = ( R * cos(ti - de_angle) ) + Xc;
    double pos_Y = ( R * sin(ti - de_angle) ) + Yc;
    
    raggiungi_punto_iter(pos_X, pos_Y, this->current_z); 
  }

  //per sicurezza è importante che current_x diventi X e current_y diventi Y
  raggiungi_punto_iter(X, Y, this->current_z);
  
}




void Manipulator::G03(double X, double Y, double Z, double I, double J){
  //ATTENZIONE! ci sono più versioni di G02 e G03
  //la nostra è quella dove I,J sono RELATIVI alla posizione attuale

  if(Z != ASSENTE){
    //se Z == ASSENTE allora vogliamo mantenere current_z
    //allora vogliamo modificare Z (ci sta non faccia nulla poi abbassa e alza)
    if(Z == -1.0) {abbassa_EF(); this->current_z = true;}  //<-- PARAMETRO!
    if(Z != -1.0) {alza_EF(); this->current_z = false;}
    delayMicroseconds(600);
  }
  
  //Interpolazione circolare in senso ANTIORARIO


  double R = norma(I,J);
  Serial.print("Raggio: "); Serial.println(R);
  {
    //--VERIFICO CHE IL COMANDO VALGA--
    //vettore relativo alla posizione attuale
    double X_primo = X - current_x;
    double Y_primo = Y - current_y;
    //tecnicamente la sua distanza da I,J dovrebbe essere uguale a R (distanza della posizione attuale da I,J)
    double d = norma(I - X_primo, J - Y_primo);
    Serial.print("distanza: "); Serial.println(d);
    if(abs(R - d) >= 0.5){
      //ATTENZIONE! si possono avere cerchi MOLTO GRANDI, per cui la differenza tra i raggi cresce
      //ma poichè i raggi sono molto lunghi tale differenza è nulla in confronto
      //allora c'è una differenza notevole tra i due raggi, è assurdo/impossibile
      Serial.begin(9600);
      while(true){Serial.println("->ERRORE: comando G03 non valido");}
    }
  }

  //coordinate globali del cerchio
  double Xc = current_x + I;
  double Yc = current_y + J;
  Serial.print("cerchio: "); Serial.print(Xc); Serial.print(" "); Serial.println(Yc);

  //coordinate della posizione attuale e X,Y relative al cerchio
  double X0 = current_x - Xc;
  double Y0 = current_y - Yc;
  double X1 = X - Xc;
  double Y1 = Y - Yc;

  //vogliamo andare da ti --> tf (tecnicamente tf > ti, per andare in senso ANTIORARIO)
  //ATTENZIONE! la differenza tra tf e ti può essere piccolissima
  double ti = atan2(Y0 , X0);
  double tf = atan2(Y1 , X1);
  //atan2 resituisce una angolo tra ]-pi, pi] e ciò non ci piace
  //li converto a [0.0, 2*pi[
  if(ti < 0.0) ti = (2.0*pi + ti);
  if(tf < 0.0) tf = (2.0*pi + tf);
  Serial.print("Angoli: "); Serial.print(ti); Serial.print(" "); Serial.println(tf);
  //attenzione ti può essere maggiore di tf
  //dovremo percorrere un ARCO più lungo -->(2 possibili archi)

  //angolo che dobbiamo percorrere
  double angolo = abs(tf - ti);  //ottengo l'angolo COMPRESO
  if(ti > tf){  //MA se ti>tf dobbiamo percorrere "l'opposto"
    Serial.println("->ATTENZIONE ti > tf");
    angolo = 2*pi - angolo;  
  }

  double arco = R * angolo;  //dove abs(tf - ti) è l'angolo compreso tra i 2
  //ogni 1/10 di millimetro raggiungiamo un punto (c'è il * 10.0 e / 10.0)
  int passaggi = int(arco * 10.0);
  Serial.print("arco: "); Serial.println(arco); 

  for(int i=0; i <= passaggi; i++){
    double de_angle = (double(i) / 10.0) / R;

    double pos_X = ( R * cos(ti + de_angle) ) + Xc;
    double pos_Y = ( R * sin(ti + de_angle) ) + Yc;
    
    raggiungi_punto_iter(pos_X, pos_Y, this->current_z); 
  }

  //per sicurezza è importante che current_x diventi X e current_y diventi Y
  raggiungi_punto_iter(X, Y, this->current_z);
}




//=====================================================================================================================================

void Manipulator::Draw(){

    //ora come ora non c'è il supporto per un scheda SD
    //ma è possibile convertire il gcode da .txt a chiamate di funzione valide
    //e incollare queste funzioni qui sotto
    // (il problema è la memoria limitata di arduino)
    //...

    // smile drawing :)
    G00( ASSENTE, ASSENTE, 5 );
    G00( 363.498, 36.9356, ASSENTE );
    G01( ASSENTE, ASSENTE, -1 );
    G03( 363.237, 42.4949, -1, -60.3844, -0.047291 );
    G03( 362.584, 46.7305, -1, -35.5588, -3.31595 );
    G03( 357.919, 59.2636, -1, -45.7661, -9.90109 );
    G03( 349.883, 70, -1, -41.3336, -22.5613 );
    G03( 336.653, 79.2845, -1, -33.1061, -33.1061 );
    G03( 320.979, 83.5527, -1, -20.0797, -42.8246 );
    G03( 315.374, 83.7533, -1, -4.54429, -48.579 );
    G03( 310.314, 83.3223, -1, 0.872585, -40.1564 );
    G03( 302.6, 81.5406, -1, 7.43409, -49.7812 );
    G03( 295.467, 78.6348, -1, 14.301, -45.3141 );
    G03( 278.219, 63.6539, -1, 21.0576, -41.6631 );
    G03( 270.15, 42.291, -1, 38.2765, -26.6622 );
    G03( 269.833, 36.8177, -1, 48.7273, -5.57149 );
    G03( 270.129, 31.6523, -1, 43.7063, -0.087459 );
    G03( 271.802, 23.4861, -1, 48.3784, 5.65928 );
    G03( 274.791, 15.9023, -1, 44.6706, 13.2233 );
    G03( 290.172, -1.7575, -1, 41.7679, 20.8501 );
    G03( 312.361, -9.83984, -1, 26.8282, 39.1539 );
    G03( 318.222, -10.0488, -1, 4.60704, 46.9118 );
    G03( 323.631, -9.54102, -1, -1.07908, 40.5485 );
    G03( 331.097, -7.76142, -1, -8.37584, 51.6915 );
    G03( 337.768, -5.05469, -1, -13.9515, 43.9557 );
    G03( 354.151, 8.5031, -1, -21.0619, 42.1288 );
    G03( 362.721, 27.6484, -1, -36.4341, 27.7996 );
    G03( 363.283, 31.7355, -1, -38.0594, 7.31922 );
    G03( 363.498, 36.9356, -1, -61.5009, 5.14679 );
    G01( 363.498, 36.9356, -1 );
    G00( ASSENTE, ASSENTE, 5 );
    G00( ASSENTE, ASSENTE, 5 );
    G00( 359.928, 38.5117, ASSENTE );
    G01( ASSENTE, ASSENTE, -1 );
    G02( 359.2, 28.7474, -1, -43.4205, -1.67461 );
    G02( 356.127, 18.9941, -1, -42.6726, 8.08585 );
    G02( 347.595, 6.53834, -1, -39.0219, 17.5791 );
    G02( 335.199, -2.25195, -1, -30.8994, 30.4381 );
    G02( 308.899, -5.67518, -1, -18.4693, 39.1559 );
    G02( 285.59, 6.87305, -1, 7.80008, 42.4131 );
    G02( 277.18, 19.4464, -1, 31.2494, 29.9992 );
    G02( 273.58, 34.2012, -1, 40.0079, 17.5793 );
    G02( 273.539, 38.9602, -1, 42.3452, 2.74273 );
    G02( 274.031, 43.6719, -1, 41.8948, -1.9929 );
    G02( 278.247, 56.4934, -1, 42.5669, -6.89324 );
    G02( 286.27, 67.4785, -1, 38.951, -20.0232 );
    G02( 289.459, 70.3772, -1, 32.7816, -32.8661 );
    G02( 292.635, 72.7461, -1, 25.0946, -30.329 );
    G02( 320.5, 79.9035, -1, 24.0778, -35.9186 );
    G02( 346.736, 67.9883, -1, -3.79935, -43.2077 );
    G02( 356.318, 54.2517, -1, -29.6554, -30.8968 );
    G02( 359.928, 38.5117, -1, -39.8201, -17.4149 );
    G01( 359.928, 38.5117, -1 );
    G00( ASSENTE, ASSENTE, 5 );
    G00( ASSENTE, ASSENTE, 5 );
    G00( 340.176, 48.5703, ASSENTE );
    G01( ASSENTE, ASSENTE, -1 );
    G03( 340.093, 49.1207, -1, -1.31608, 0.083175 );
    G03( 339.705, 49.8398, -1, -2.89443, -1.09668 );
    G03( 338.453, 50.9009, -1, -2.78476, -2.01727 );
    G03( 335.324, 52.0215, -1, -6.2934, -12.6427 );
    G03( 331.598, 52.5215, -1, -4.55366, -19.8025 );
    G03( 327.613, 52.3359, -1, -0.931548, -22.869 );
    G03( 322.862, 51.0514, -1, 2.5265, -18.7734 );
    G03( 321.178, 49.6582, -1, 1.43626, -3.45155 );
    G03( 320.933, 48.5184, -1, 1.33866, -0.882934 );
    G03( 321.566, 47.2637, -1, 2.30014, 0.373276 );
    G03( 324.681, 45.5042, -1, 4.58142, 4.47365 );
    G03( 330.613, 44.832, -1, 5.79486, 24.6248 );
    G03( 336.533, 45.5738, -1, -0.136297, 25.0739 );
    G03( 339.549, 47.3613, -1, -1.47519, 5.92749 );
    G03( 340.025, 48.035, -1, -2.35061, 2.16671 );
    G03( 340.176, 48.5703, -1, -1.17183, 0.618948 );
    G01( 340.176, 48.5703, -1 );
    G00( ASSENTE, ASSENTE, 5 );
    G00( ASSENTE, ASSENTE, 5 );
    G00( 340.168, 22.9102, ASSENTE );
    G01( ASSENTE, ASSENTE, -1 );
    G03( 339.981, 23.7712, -1, -1.48729, 0.128275 );
    G03( 339, 24.8984, -1, -3.22624, -1.8178 );
    G03( 336.498, 26.1109, -1, -4.24052, -5.56277 );
    G03( 331.359, 26.8594, -1, -6.06841, -23.6485 );
    G03( 324.421, 26.1145, -1, -0.91927, -24.1217 );
    G03( 321.42, 24.2988, -1, 1.51894, -5.89963 );
    G03( 320.925, 23.2015, -1, 1.45768, -1.31693 );
    G03( 321.172, 22.1035, -1, 1.71149, -0.192538 );
    G03( 323.512, 20.3006, -1, 3.55907, 2.19964 );
    G03( 330.518, 19.2578, -1, 7.00529, 23.0081 );
    G03( 338.654, 20.8543, -1, 0, 21.5326 );
    G03( 340.168, 22.9102, -1, -0.924904, 2.26614 );
    G01( 340.168, 22.9102, -1 );
    G00( ASSENTE, ASSENTE, 5 );
    G00( ASSENTE, ASSENTE, 5 );
    G00( 305.9, 62.9297, ASSENTE );
    G01( ASSENTE, ASSENTE, -1 );
    G03( 305.917, 63.037, -1, -0.081347, 0.067614 );
    G03( 305.801, 63.2461, -1, -0.904849, -0.366437 );
    G02( 304.913, 64.6095, -1, 12.303, 8.97967 );
    G02( 304.541, 65.4297, -1, 4.00144, 2.31045 );
    G03( 304.277, 66.0411, -1, -3.9796, -1.35604 );
    G03( 304.189, 66.0938, -1, -0.087516, -0.046341 );
    G03( 303.506, 65.9439, -1, 0, -1.63572 );
    G03( 300.369, 64.3359, -1, 17.393, -37.7884 );
    G03( 290.222, 55.0915, -1, 15.3297, -27.0164 );
    G03( 285.15, 42.5215, -1, 24.8515, -17.3361 );
    G03( 284.805, 37.5048, -1, 27.8494, -4.43575 );
    G03( 285.357, 32.4961, -1, 28.2833, 0.582765 );
    G03( 289.981, 21.4472, -1, 29.9402, 6.03693 );
    G03( 298.527, 12.9824, -1, 25.5312, 17.2306 );
    G03( 302.546, 10.6119, -1, 25.1691, 38.0771 );
    G03( 303.502, 10.3828, -1, 0.932873, 1.78401 );
    G03( 303.622, 10.4418, -1, -0.001659, 0.155199 );
    G03( 304.115, 11.1602, -1, -5.1304, 4.05038 );
    G02( 304.619, 11.9001, -1, 6.16962, -3.65571 );
    G02( 305.244, 12.6152, -1, 6.37562, -4.94595 );
    G03( 305.863, 13.2925, -1, -8.76309, 8.62792 );
    G01( 305.857, 13.3398, -1 );
    G03( 305.795, 13.3756, -1, -0.1849, -0.251292 );
    G03( 304.574, 13.8848, -1, -36.0384, -84.6693 );
    G02( 302.474, 14.8772, -1, 8.22599, 20.1246 );
    G02( 300.211, 16.2402, -1, 13.3114, 24.6635 );
    G02( 292.658, 23.9996, -1, 14.4565, 21.6279 );
    G02( 288.713, 34.4863, -1, 23.5719, 14.8529 );
    G02( 288.431, 38.0745, -1, 22.5342, 3.5786 );
    G02( 288.717, 41.832, -1, 24.9309, -0.010441 );
    G02( 292.607, 52.2386, -1, 26.8423, -4.1025 );
    G02( 300.19, 59.9824, -1, 21.851, -13.8127 );
    G02( 302.866, 61.56, -1, 16.4636, -24.874 );
    G02( 305.305, 62.6484, -1, 10.1785, -19.5283 );
    G03( 305.791, 62.847, -1, -3.22752, 8.59733 );
    G03( 305.9, 62.9297, -1, -0.121405, 0.274561 );
    G01( 305.9, 62.9297, -1 );
    G00( ASSENTE, ASSENTE, 5 );
    G00( 103.5, 140, ASSENTE );

}
