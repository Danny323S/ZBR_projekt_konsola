# ZBR_projekt_konsola
Wersja konsolowa projektu ZBR

#include <iostream>
#include <string>
#include <cmath>

//KLASY
//klasa układ współrzędnych
class coordinateSystem {
public:

    struct position {
        double x;
        double y;
        double z;
    }pos;

    //współrzędna maszynowa
    double fi;
    //cos fi
    double C;
    //sin fi
    double S;

    coordinateSystem(double x, double y, double z){
        pos.x = x;
        pos.y = y;
        pos.z = z;
        fi = 0;
        S = 0;
        C = 0;
    }

};

//STRUKTURY
struct POINT{
    double x = 0;
    double y = 0;
    double z = 0;
};

//struktura zawierająca parametry konstrukcyjne robota
struct  ROBOT_PARAMETERS {
    //dlugości poszczególnych członów
    double arm1 = 350;
    double arm2 = 500;
    double arm3 = 1350;
    double arm4 = 150;
    double arm5 = 200;
    double arm6 = 220;

    //pierwsza długość odsadzenia
    double d = 220;

    //parametr potrzebny do obliczenia drugiego odsadzenia
    //drugie odsadzenie = d - e
    double e = 60;

    //kąty podejścia narzędnia
    double OAngle = 45;
    double YAngle = 30;

    //parametry rozwiązujące położenia ....
    int delta1 = -1;
    int delta2 = -1;
    int delta5 = -1;
};

//struktura zawierająca współrzędne maszynowe robota
struct MACHINE_COORDS {
    double fi1;
    double fi2;
    double fi3;
    double fi4;
    double fi5;
    double fi23;
    double fi234;
};

//FUNKCJE
//funkcja obliczająca współrzędne maszynowe
MACHINE_COORDS calculateMachineCoords(ROBOT_PARAMETERS parameter, POINT startPoint, POINT endPoint, double step){

    //dlugości poszczególnych członów
    double arm1 = parameter.arm1;
    double arm2 = parameter.arm2;
    double arm3 = parameter.arm3;
    double arm4 = parameter.arm4;
    double arm5 = parameter.arm5;
    double arm6 = parameter.arm6;

    //pierwsza długość odsadzenia
    double d = parameter.d;

    //parametr potrzebny do obliczenia drugiego odsadzenia
    //drugie odsadzenie = d - e
    double e = parameter.e;

    //kąty podejścia narzędnia
    double OAngle = parameter.OAngle;
    double YAngle = parameter.YAngle;

    //parametry rozwiązujące położenia ....
    int delta1 = parameter.delta1;
    int delta2 = parameter.delta1;
    int delta5 = parameter.delta1;

    //zmienne do obliczeń
    double a = 0;
    double b = 0;

    //Układ bazowy O0
    coordinateSystem coord0(0, 0, 0);

    //deklaracja ukladu współrzędnych narzednia
    //pozycja bedzie później wprowadzana przez uzytkownika
    coordinateSystem TCPcoords(xt,yt,zt);
    double sinO = sin(OAngle*M_PI / 180);
    double cosO = cos(OAngle*M_PI / 180);
    double sinY = sin(YAngle*M_PI / 180);
    double cosY = cos(YAngle*M_PI / 180);


    //Zdefiniowanie układu O5, Obliczenie xp, yp, zp
    coordinateSystem coord5(0, 0, 0);
    coord5.pos.x = TCPcoords.pos.x - ((arm6 + arm5) * cosO * cosY);
    coord5.pos.y = TCPcoords.pos.y - ((arm6 + arm5) * cosO * sinY);
    coord5.pos.z = TCPcoords.pos.z - ((arm6 + arm5) * sinO);

    //Obliczenie S1 i C1
    coordinateSystem coord1(0 ,0 ,0);
    coord1.S = (1/(pow(coord5.pos.x,2) + pow(coord5.pos.y,2))) * (e*coord5.pos.x + delta1*coord5.pos.y*sqrt(pow(coord5.pos.x,2) + pow(coord5.pos.y,2) - pow(e,2)));
    coord1.C = (1/(pow(coord5.pos.x,2) + pow(coord5.pos.y,2))) * (-e*coord5.pos.x + delta1*coord5.pos.x*sqrt(pow(coord5.pos.x,2) + pow(coord5.pos.y,2) - pow(e,2)));

    //Obliczenie S5 i C5
    coord5.S = cosO * (sinY*coord1.C - cosO*coord1.S);
    coord5.C = delta5*sqrt(1 - pow(coord5.S, 2));

    coordinateSystem coord234(0, 0, 0);
    coord234.S = sinO/coord5.C;
    coord234.C = (cosO/coord5.C) * (cosY*coord1.C + sinY*coord1.S);

    //Zdefiniowanie układu O3, Obliczenie xr, yr, zr
    coordinateSystem coord3(0, 0, 0);
    coord3.pos.x = coord5.pos.x - arm4*coord1.C*coord234.C;
    coord3.pos.y = coord5.pos.y - arm4*coord1.S*coord234.C;
    coord3.pos.z = coord5.pos.z - arm4*coord234.S;

    a = (-arm1 + delta1*sqrt(pow(coord3.pos.x,2) + pow(coord3.pos.y,2) - pow(e,2)));
    b = ((1/(2*arm2)) * (pow(a,2) + pow(coord3.pos.z,2) + pow(arm2,2) - pow(arm3, 2)));

    //Zdefiniowanie układu O2, Obliczenie S2, C2
    coordinateSystem coord2(0,0,0);
    coord2.S = (1/(pow(a, 2) + pow(coord3.pos.z,2)) * (coord3.pos.z*b + delta2*a*sqrt(pow(a,2) + pow(coord3.pos.z,2) - pow(b,2))));
    coord2.C = (1/(pow(a, 2) + pow(coord3.pos.z,2)) * (a*b - delta2*coord3.pos.z*sqrt(pow(a,2) + pow(coord3.pos.z,2) - pow(b,2))));

    //Obliczenie S3, C3
    coord3.S = -(delta2/arm3) * sqrt(pow(a,2) + pow(coord3.pos.z,2) - pow(b,2));
    coord3.C = ((b-arm2)/arm3);

    //obliczenie sin(fi2 + fi3) cos(fi2 +fi3)
    coordinateSystem coord23(0,0,0);
    coord23.S = (1/arm3) * (coord3.pos.z -arm2*coord2.S);
    coord23.C = (1/arm3) * (a - arm2*coord2.C);

    //Zdefiniowanie O4, Obliczenie S4, C4
    coordinateSystem coord4(0,0,0);
    coord4.S = coord234.S*coord23.C - coord234.C*coord23.S;
    coord4.C = coord234.C*coord23.C - coord234.S*coord23.S;

    //obliczenie pozycji 01
    coord1.pos.x = arm1*coord1.C;
    coord1.pos.y = arm1*coord1.S;
    coord1.pos.z = 0;

    //obliczenie pozycji02
    coord2.pos.x = coord1.pos.x + d*coord1.S + arm2*coord2.C*coord1.C - (d-e)*coord1.S;
    coord2.pos.y = arm1*coord1.S -d*coord1.C + arm2*coord2.C*coord1.S + (d-e)*coord1.C;
    coord2.pos.z = arm2*coord2.S;

    MACHINE_COORDS actualCoords;

    actualCoords.fi1 = asin(coord1.S) * 180.0 / M_PI ;
    actualCoords.fi2 = asin(coord2.S) * 180.0 / M_PI;
    actualCoords.fi3 = asin(coord3.S) * 180.0 / M_PI;
    actualCoords.fi4 = asin(coord4.S) * 180.0 / M_PI;
    actualCoords.fi5 = asin(coord5.S) * 180.0 / M_PI;
    actualCoords.fi23 = asin(coord23.S) * 180.0 / M_PI;
    actualCoords.fi234 = asin(coord234.S) * 180.0 / M_PI;

    return actualCoords;
}

//funkcja wyświetlająca współrzędne maszynowe
void printMachineCoords(MACHINE_COORDS machineCoords) {
    std::cout << "Wspolrzedne maszynowe: " << std::endl;
    std::cout << "fi1 = " << machineCoords.fi1 << std::endl;
    std::cout << "fi2 = " << machineCoords.fi2 << std::endl;
    std::cout << "fi3 = " << machineCoords.fi3 << std::endl;
    std::cout << "fi4 = " << machineCoords.fi4 << std::endl;
    std::cout << "fi5 = " << machineCoords.fi5 << std::endl;
    std::cout << "fi23 = " << machineCoords.fi23 << std::endl;
    std::cout << "fi234 = " << machineCoords.fi234 << std::endl << std::endl;
}

//funkcja obliczająca współrzędne punktów między podanym punktem początkowym i końcowym
void interpolation(int step, double xs, double ys, double zs, double xe, double ye, double ze) {
    struct VECTOR{
        double x = 0;
        double y = 0;
        double z = 0;
    };

    //wektor między punktem początkowym i końcowym TCP
    VECTOR SE;
    SE.x = xe - xs;
    SE.y = ye - ys;
    SE.z = ze - zs;

    VECTOR temp;

    VECTOR actualpoint;

    for(double i = 0; i <= step; i++){

        temp.x = SE.x;
        temp.y = SE.y;
        temp.z = SE.z;

        if(i == 0){
            actualpoint.x = xs;
            actualpoint.y = ys;
            actualpoint.z = zs;
        } else {

            temp.x = (i/step)*temp.x;
            temp.y = (i/step)*temp.y;
            temp.z = (i/step)*temp.z;

            actualpoint.x = temp.x + xs;
            actualpoint.y = temp.y + ys;
            actualpoint.z = temp.z + zs;
        }

        //wyświetlenie aktualnego punkty
        std::cout << i << ". punkt" << std::endl;
        std::cout << "("<<actualpoint.x << "," << actualpoint.y << "," << actualpoint.z << ")" << std::endl;
    }
}

using namespace std;

int main(){
    
    
    return 0;
}
