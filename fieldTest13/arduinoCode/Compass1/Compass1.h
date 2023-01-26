#ifndef Compass1_h
#define Compass1_h


class Compass1 {
public:
    Compass1(char address);
    float CMPS2_getHeading();
    void CMPS2_init();
private:
    void CMPS2_read_XYZ();
    char _address;
    float _data[3];
    unsigned int _raw[3];
    float _offset[3];
    int _declination;

};
#endif