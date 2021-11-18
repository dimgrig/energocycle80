#ifndef ENERGOCYCLE80_ENERGOCYCLESETTINGS_H
#define ENERGOCYCLE80_ENERGOCYCLESETTINGS_H

#include "LinxDevice.h"

struct PID_settings_s {
    PID_settings_s() :
        Kp(0),
        Ki(0),
        Kd(0)
    {};
    PID_settings_s(float p, float i, float d) :
        Kp(p),
        Ki(i),
        Kd(d)
    {};
    PID_settings_s(PID_settings_s &settings) :
            Kp(settings.Kp),
            Ki(settings.Ki),
            Kd(settings.Kd)
    {};
    double Kp;
    double Ki;
    double Kd;
};

struct EC_PID_settings {
	EC_PID_settings() :
            T_PID_settings(),
            U_PID_settings(),
            I_PID_settings()
    {};
    EC_PID_settings(PID_settings_s t, PID_settings_s u, PID_settings_s i) :
            T_PID_settings(t),
            U_PID_settings(u),
            I_PID_settings(i)
    {};
    PID_settings_s T_PID_settings;
    PID_settings_s U_PID_settings;
    PID_settings_s I_PID_settings;
};

struct CALIB_settings_s {
	CALIB_settings_s() :
        K(1),
        B(0)
    {};
	CALIB_settings_s(float k, float b) :
        K(k),
        B(b)
    {};
    CALIB_settings_s(CALIB_settings_s &settings) :
            K(settings.K),
            B(settings.B)
    {};
    double K;
    double B;
};

struct EC_CALIB_settings {
	EC_CALIB_settings() :
		T_CALIB_settings(),
		U_CALIB_settings(),
		I_CALIB_settings()
    {};
	EC_CALIB_settings(CALIB_settings_s t, CALIB_settings_s u, CALIB_settings_s i) :
		T_CALIB_settings(t),
		U_CALIB_settings(u),
		I_CALIB_settings(i)
    {};

	CALIB_settings_s T_CALIB_settings;
	CALIB_settings_s U_CALIB_settings;
	CALIB_settings_s I_CALIB_settings;
};

enum Energocycle_mode {
    E_mode_U,
    E_mode_I,
    E_mode_P
};

enum Energocycle_state {
    E_state_idle,
    E_state_heat,
    E_state_cool
};

enum Energocycle_error {
    E_no_error,
    E_error
};

struct Energocycle_settings_s {
    Energocycle_settings_s() :
            mode(E_mode_U),
            res1(0),
            res2(0),
            res3(0),
            Unom(0),
            Umax(0),
            Umin(0),
            Inom(0),
            Imax(0),
            Imin(0),
            Tmax(0),
            Tmin(0),
            Cycles(0)
    {};
    Energocycle_settings_s(Energocycle_mode m, float unom, float umax, float umin,
                           float inom, float imax, float imin,
                           float tmax, float tmin, unsigned int c) :
            mode(m),
            res1(0),
            res2(0),
            res3(0),
            Unom(unom),
            Umax(umax),
            Umin(umin),
            Inom(inom),
            Imax(imax),
            Imin(imin),
            Tmax(tmax),
            Tmin(tmin),
            Cycles(c)
    {};
    Energocycle_mode mode;
    unsigned char res1;
    unsigned char res2;
    unsigned char res3;

    float Unom;
    float Umax;
    float Umin;
    float Inom;
    float Imax;
    float Imin;
    float Tmax;
    float Tmin;

    unsigned int Cycles;
};

struct Energocycle_status_s {
    Energocycle_status_s() :
            status(L_OK),
            state(E_state_idle),
            error(E_no_error),
            res(0),
            U(0),
            I(0),
            T(0),
            Cycles(0)
    {};
    Energocycle_status_s(LinxStatus l_s, Energocycle_state s, Energocycle_error e,
                           float u, float i, float t, unsigned int c) :
            status(l_s),
            state(s),
            error(e),
            res(0),
            U(u),
            I(i),
            T(t),
            Cycles(c)
    {};

    LinxStatus status;
    Energocycle_state state;
    Energocycle_error error;
    unsigned char res;

    float U;
    float I;
    float T;
    unsigned int Cycles;
};

struct EC_FLASH_settings{
	EC_FLASH_settings():
		init(0),
		epid_settings(),
		ecalib_settings()
	{}

	long unsigned int init;
	EC_PID_settings epid_settings;
	EC_CALIB_settings ecalib_settings;
	// !!! Full size (bytes) must be a multiple of 4 !!!
};


//EC_FLASH_settings eflash_settings;
//#define MY_FLASH_PAGE_ADDR 0x800FC00
//#define SETTINGS_WORDS sizeof(eflash_settings)/4

#endif //ENERGOCYCLE80_ENERGOCYCLESETTINGS_H
