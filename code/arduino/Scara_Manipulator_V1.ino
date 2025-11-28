#include <math.h>
#define pi 3.141592


//in generale alcuni comandi gcode possono omettere X, Y o Z
//ciò significa che dobbiamo raggiungere un punto che ha le stesse coordinate di una delle x, y o z attuali
//"ASSENTE" è un valore che siamo sicuro non si può trovare nei comandi G, in quanto richiederebbe di avere una workspace molto grande
#define ASSENTE 100000.0


class Manipulator;


class CNCstepper {
private:
  int steps;  //steps correnti. Varia tra [0, 200*resolution*ratio]
  double ratio;

  double omega;  //velocità angolare con cui a prescindere ruota il joint
  double time;  //dipende da omega, è il delay in microsecondi da un passo all'altro

  double minAngle;  //vincolo MAX angolo raggiungibile <= 360
  double maxAngle;  //vincolo min angolo raggiungibile >= -360

  int stepPin;  //pin dei motori, derivano da dove si collega il motore allo shield (X, Y o Z)
  int dirPin;  //direzione "dir" indica in quale verso deve ruotare (orario (HIGH) o antiorario (LOW))
  
  double resolution;  //risoluzione degli step, di default è 1, quindi 1.8 gradi per step. Può valere 1, 2, 4, 8, 16

private:
  double funzione_tempo(double x, double steps, double time0, double percentuale = 20.0, double ritardo = 2.0);

public:
  CNCstepper(char identificatore, double velocita, int ratio, int resolution = 1);

  void inizializza();
  void set_vincolo(double min, double max);

  int steps_value();
  int ratio_value();

  void angle_iter(double angle);
  void angle_iter_acc(double angle, double percentuale = 20.0, double ritardo = 2.0);
  void angle_configure(double angle, double percentuale = 20.0, double ritardo = 2.0);

  friend class Manipulator;
};


class Manipulator{
private:
  const double raggioEndEffector;

  CNCstepper joint1;
  CNCstepper joint2;
  CNCstepper joint3;

  double theta1;
  double theta2;
  double theta3;
  
  double current_x;
  double current_y;
  bool current_z;

private:
  void trigonometriaV1(double x, double y);
  double norma(double x, double y);

  void alza_EF();
  void abbassa_EF();
    
  void raggiungi_punto_iter(double x, double y, bool z);

  //codici gcode
  void G00(double X, double Y, double Z);
  void G01(double X, double Y, double Z);
  void G02(double X, double Y, double Z, double I, double J);
  void G03(double X, double Y, double Z, double I, double J);

public:
  Manipulator(double raggio_EndEffector);
  void inizializza();

  void home();

  void raggiungi_punto(double x, double y);
  void punto();
  void segmento(double x, double y);

  void Draw();
};

//======================================================================================================================================================

//inizializziamo il manipolatore con il raggio del pennarello
//perché impatta nella distanza del link e quindi nella trigonometria
Manipulator Scara(8.33/2.0);




void setup() {
  Serial.begin(9600);

  delay(5000);

  Scara.inizializza();
}



void loop() {

  Serial.println("Test");


  delay(2000);
  Scara.Draw();

  Scara.home();  
}
