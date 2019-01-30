#include <iostream>
#include <cmath>

using namespace std;

int main(){
    double distance = 0;
    double height = 0;
    double angle = 0;
    cout << "Initial Velocity Calculator for 5249S\nInput Firing Angle(degrees): ";
    cin >> angle;
    cout << "\nInput X distance(meters): ";
    cin >> distance;
    cout << "\nInput Y height(meters): ";
    cin >> height;
    angle *= (3.1415/180);
    double velocity = sqrt((-9.8*pow(distance, 2)/(2 * pow(cos(angle), 2) * (height - distance * tan(angle))));
    cout << "\n" << velocity;
    return 0;
}
