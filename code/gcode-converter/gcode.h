
struct codiceG{
    char str[3 + 1];
    double X;
    double Y;
    double Z;

    double I;
    double J;

    codiceG();  //COSTRUTTORE DEFAULT
};



class gcode {
private:
    char* directory;

    double Xt;
    double Yt;

    double width;
    double height;

    int n;

    codiceG* buffer;

public:
    gcode(const char str[], double X, double Y, double width = 210.0, double height = 297.0);
    ~gcode();
    void leggi();
    void scrivi();
};
