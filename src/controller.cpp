#include "controller.hpp"

// P-Regler: u(t) = Kp * e(t)
// Kp: Proportionalfaktor – bestimmt die Stärke der Reaktion auf den aktuellen Fehler
// e(t): Fehler zum aktuellen Zeitpunkt t (Sollwert - Istwert)
// u(t): Stellgröße (z. B. Heizleistung, Motorstrom, ...)
PController::PController(double kp) : mKp(kp) {}
double PController::update(double error, double /*dt*/)
{
    return mKp * error;
}

// PI-Regler: u(t) = Kp * e(t) + Ki * ∫e(t) dt
// Ki: Integralfaktor – verstärkt die kumulierte Summe aller vergangenen Fehler
// ∫e(t) dt: Zeitliche Integration des Fehlers – wie lange und wie stark der Fehler besteht
// → Vorteil: Korrigiert bleibende Regelabweichung
PIController::PIController(double kp, double ki) : mKp(kp), mKi(ki), mIntegral(0.0) {}
double PIController::update(double error, double dt)
{
    mIntegral += error * dt;
    return mKp * error + mKi * mIntegral;
}

// PID-Regler: u(t) = Kp * e(t) + Ki * ∫e(t) dt + Kd * de(t)/dt
// Kd: Differentialfaktor – reagiert auf die Änderungsgeschwindigkeit des Fehlers
// de(t)/dt: Ableitung des Fehlers – wie schnell sich der Fehler verändert
// → Vorteil: Dämpft schnelle Änderungen und verhindert Überschwingen
PIDController::PIDController(double kp, double ki, double kd) : mKp(kp), mKi(ki), mKd(kd), mIntegral(0.0), mPreviousError(0.0) {}
double PIDController::update(double error, double dt)
{
    mIntegral += error * dt;
    double derivative = (error - mPreviousError) / dt;
    mPreviousError = error;
    return mKp * error + mKi * mIntegral + mKd * derivative;
}
