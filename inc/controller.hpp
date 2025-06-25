class Controller
{
public:
    Controller() = default;
    virtual ~Controller() = default;

    virtual double update(double error, double dt) = 0;
};

class PController : public Controller
{
public:
    PController(double kp);
    double update(double error, double dt) override;

private:
    double mKp;
};

class PIController : public Controller
{
public:
    PIController(double kp, double ki);
    double update(double error, double dt) override;

private:
    double mKp;
    double mKi;
    double mIntegral;
};

class PIDController : public Controller
{
public:
    PIDController(double kp, double ki, double kd);
    double update(double error, double dt) override;

private:
    double mKp;
    double mKi;
    double mKd;
    double mIntegral;
    double mPreviousError;
};