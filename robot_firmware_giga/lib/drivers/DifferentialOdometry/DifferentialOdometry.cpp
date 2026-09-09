#include "DifferentialOdometry.h"
#include <math.h>

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------

DifferentialOdometry::DifferentialOdometry(float wheelRadiusMeters, float wheelBaseMeters)
    : _wheelRadiusMeters(wheelRadiusMeters)
    , _wheelBaseMeters(wheelBaseMeters)
    , _x(0.0f)
    , _y(0.0f)
    , _theta(0.0f)
    , _lastLinearDisplacement(0.0f)
    , _lastAngularDisplacement(0.0f)
{
}

// -----------------------------------------------------------------------
// Core integration
// -----------------------------------------------------------------------

void DifferentialOdometry::update(float deltaLeftMeters, float deltaRightMeters)
{
    // Classic differential-drive kinematics:
    //   linear displacement  = average of both wheels
    //   angular displacement = difference of both wheels / wheel base
    const float linearDisplacement  = (deltaLeftMeters + deltaRightMeters) / 2.0f;
    const float angularDisplacement = (deltaRightMeters - deltaLeftMeters) / _wheelBaseMeters;

    // First-order (Euler) integration in the global frame.
    // Valid as an approximation for small angularDisplacement per step;
    // call update() at a steady, sufficiently fast rate (e.g. same
    // period as your velocity control loop) to keep this approximation
    // accurate.
    _x     += linearDisplacement * cosf(_theta);
    _y     += linearDisplacement * sinf(_theta);
    _theta  = _normalizeAngle(_theta + angularDisplacement);

    _lastLinearDisplacement  = linearDisplacement;
    _lastAngularDisplacement = angularDisplacement;
}

void DifferentialOdometry::reset()
{
    _x = 0.0f;
    _y = 0.0f;
    _theta = 0.0f;
    _lastLinearDisplacement  = 0.0f;
    _lastAngularDisplacement = 0.0f;
}

void DifferentialOdometry::setPose(float x, float y, float thetaRadians)
{
    _x = x;
    _y = y;
    _theta = _normalizeAngle(thetaRadians);
}

// -----------------------------------------------------------------------
// Accessors
// -----------------------------------------------------------------------

float DifferentialOdometry::getX() const     { return _x; }
float DifferentialOdometry::getY() const     { return _y; }
float DifferentialOdometry::getTheta() const { return _theta; }

float DifferentialOdometry::getLastLinearDisplacement() const  { return _lastLinearDisplacement; }
float DifferentialOdometry::getLastAngularDisplacement() const { return _lastAngularDisplacement; }

float DifferentialOdometry::getWheelBase() const   { return _wheelBaseMeters; }
float DifferentialOdometry::getWheelRadius() const { return _wheelRadiusMeters; }

// -----------------------------------------------------------------------
// Private helper
// -----------------------------------------------------------------------

float DifferentialOdometry::_normalizeAngle(float angleRadians)
{
    // Bring angleRadians into [-pi, pi] regardless of how many full
    // turns it has accumulated.
    while (angleRadians > M_PI)  { angleRadians -= 2.0f * M_PI; }
    while (angleRadians < -M_PI) { angleRadians += 2.0f * M_PI; }
    return angleRadians;
}
