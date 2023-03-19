/** @file Trajectory.h
 *
 * Copyright (c) 2019 IACE
 */
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <cmath>
#include "define.h"
#include <cstdarg>

/**
 * @brief Base tractory class.
 */
class Trajectory {
public:
    /**
     * method the computes the trajectory output for the current time
     * @param dTime current time in milliseconds
     * @return trajectory output for the current time
     */
    virtual void compute(const unsigned long &lTime) = 0;

    double *getEta() {
        return this->dEta;
    }

    double *getTheta() {
        return this->dTheta;
    }

    double *getAlpha() {
        return this->dAlpha;
    }

    /**
     * @brief virtual destructor to make deleting derived classes
     * from a base class pointer work
     */
    virtual ~Trajectory() {}

protected:
    unsigned long lStartTime = 0;       ///< value of start time in ms
    double dStartValue = 0.0;           ///< value of start value in ms

    double dTheta[3] = {0., 0., 0.};
    double dAlpha[3] = {0., 0., 0.};
    double dEta[5] = {0., 0., 0., 0.};
};

class NoneTrajectory : public Trajectory {
public:
    void compute(const unsigned long &lTime) override {
        this->dEta[0]  = 0;
    };
};

/**
 * @brief Class that is derived from the Trajectory class and implements a Polynomial trajectory of order 2.
 */
class PolynomTrajectory : public Trajectory {
private:
    unsigned long lEndTime = 0;        ///< value sof final time
    double dEndValue = 0;              ///< value of final value
public:

    void compute(const unsigned long &lTime) override {

        if (lTime <= this->lStartTime) {
            this->dEta[0] = this->dStartValue;

            for (int i = 1; i < 5; ++i) {
                this->dEta[i] = 0.0;
            }
        } else if (lTime >= this->lEndTime) {
            this->dEta[0] = this->dEndValue;

            for (int i = 1; i < 5; ++i) {
                this->dEta[i] = 0.0;
            }
        } else {
            double dT = (this->lEndTime - this->lStartTime); // [ms]
            double dTau = (lTime - this->lStartTime) / dT;     // []

            double dTau2 = dTau * dTau;
            double dTau3 = dTau2 * dTau;
            double dTau4 = dTau3 * dTau;
            double dTau5 = dTau4 * dTau;
            double dTau6 = dTau5 * dTau;
            double dTau7 = dTau6 * dTau;
            double dTau8 = dTau7 * dTau;
            double dTau9 = dTau8 * dTau;

            double dDeltaY = (this->dEndValue - this->dStartValue);

            // Todo:: apply to AnPaMa model to calc x_ref
            this->dEta[0] =
                    this->dStartValue + dDeltaY * (70 * dTau9 - 315 * dTau8 + 540 * dTau7 - 420 * dTau6 + 126 * dTau5);
            this->dEta[1] = dDeltaY * (630 * dTau8 - 2520 * dTau7 + 3780 * dTau6 - 2520 * dTau5 + 630 * dTau4) / dT;
            this->dEta[2] =
                    dDeltaY * (5040 * dTau7 - 17640 * dTau6 + 22680 * dTau5 - 12600 * dTau4 + 2520 * dTau3) / (dT * dT);
            this->dEta[3] = dDeltaY * (35280 * dTau6 - 105840 * dTau5 + 113400 * dTau4 - 50400 * dTau3 + 7560 * dTau2) /
                            (dT * dT * dT);
            this->dEta[4] =
                    dDeltaY * (211680 * dTau5 - 529200 * dTau4 + 453600 * dTau3 - 201600 * dTau2 + 15120 * dTau) /
                    (dT * dT * dT * dT);


        }
    };

    void setTimesValues(const unsigned long &lStartTime,
                        const unsigned long &lEndTime,
                        const double &dStartValue,
                        const double &dEndValue) {
        this->lStartTime = lStartTime; // [ms] in PyWisp umgerechnet
        this->lEndTime = lEndTime;    // [ms] ni PyWisp umgerechnet
        this->dStartValue = dStartValue;
        this->dEndValue = dEndValue;
    };
};


/**
 * @brief Class that is derived from the Trajectory class and implements a Polynomial trajectory of order 2.
 */
class TrapezTrajectory : public Trajectory {
private:
    unsigned long lEndTime = 0;        ///< value sof final time
    unsigned long lMidStartTime = 0;        ///< value of mid point time
    unsigned long lMidEndTime = 0;    ///< value of mid point hold time
    double dEndValue = 0;              ///< value of final value
    double dMidValue = 0;
public:

    void compute(const unsigned long &lTime) override {

        if (lTime <= this->lStartTime) {
            this->dEta[0] = this->dStartValue;

        } else if (lTime <= this->lMidStartTime && lTime >= lStartTime) {
            // first linear slope
            double dT = (this->lMidStartTime - this->lStartTime); // [ms]
            double dTau = (lTime - this->lStartTime) / dT;     // []

            double dDeltaY = (this->dMidValue - this->dStartValue);

            this->dEta[0] = this->dStartValue + dDeltaY * dTau;


        } else if(lTime <= lMidEndTime && lTime >= lMidStartTime) {
            this->dEta[0] = this->dMidValue;

        } else if (lTime <= lEndTime && lTime >= lMidEndTime) {

           // second linear slope
            double dT = (this->lEndTime - this->lMidEndTime); // [ms]
            double dTau = (lTime - this->lMidEndTime) / dT;     // []

            double dDeltaY = (this->dEndValue - this->dMidValue);

            this->dEta[0] = this->dMidValue + dDeltaY * dTau;

        }else if (lTime >= this->lEndTime) {
                this->dEta[0] = dEndValue;

        }

    };

    void setTimesValues(const unsigned long &lStartTime,
                        const unsigned long &lMidStartTime,
                        const unsigned long &lMidEndTime,
                        const unsigned long &lEndTime,
                        const double &dStartValue,
                        const double &dMidValue,
                        const double &dEndValue) {
        this->lStartTime = lStartTime; // [ms] in PyWisp umgerechnet
        this->lEndTime = lEndTime;    // [ms] ni PyWisp umgerechnet
        this->lMidStartTime = lMidStartTime;
        this->lMidEndTime = lMidEndTime;
        this->dStartValue = dStartValue;
        this->dMidValue = dMidValue;
        this->dEndValue = dEndValue;
    };
};

#endif // TRAJECTORY_H