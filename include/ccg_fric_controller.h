#ifndef CCG_FRICTION_CONTROLLER_H
#define CCG_FRICTION_CONTROLLER_H

#include <vector>

class CCGFrictionController {
public:
    CCGFrictionController();
    void computeTau(const std::vector<double>& q, const std::vector<double>& q_dot, std::vector<double>& tau, 
        std::vector<double>& grav_gain, std::vector<double>& fric_gain);

private:
    std::vector<double> L;
    std::vector<double> beta;
    std::vector<double> friction_params;
};

#endif