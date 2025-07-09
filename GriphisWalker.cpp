#include <cnoid/SimpleController>
#include <cnoid/Joystick>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

using namespace cnoid;

class GriphisWalker : public SimpleController
{
    Joystick joystick;
    std::vector<Link*> groupA; // griphis_1,3
    std::vector<Link*> groupB; // griphis_2,4
    std::vector<Link*> fixedJoints;
    std::vector<double> fixedRef;
    std::vector<double> fixedPrev;
    double dt;
    bool is_control_started = false;

    // モーション制御用
    double phase = 0.0;
    const double motionFrequency = 0.5; // Hz
    const double motionAmplitude = 45.0;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        auto body = io->body();
        dt = io->timeStep();

        std::cout << "--- Initializing GriphisWalker ---" << std::endl;

        std::vector<std::string> groupA_names = {
            "griphis_1_pitch_1_link",
            "griphis_3_pitch_1_link"
        };
        std::vector<std::string> groupB_names = {
            "griphis_2_pitch_1_link",
            "griphis_4_pitch_1_link"
        };

        for(const auto& name : groupA_names){
            auto link = body->link(name);
            if(!link){
                std::cerr << "Joint not found: " << name << std::endl;
                return false;
            }
            link->setActuationMode(Link::JointTorque);
            io->enableIO(link);
            groupA.push_back(link);
        }

        for(const auto& name : groupB_names){
            auto link = body->link(name);
            if(!link){
                std::cerr << "Joint not found: " << name << std::endl;
                return false;
            }
            link->setActuationMode(Link::JointTorque);
            io->enableIO(link);
            groupB.push_back(link);
        }

        for(auto& link : body->links()){
            if(std::find(groupA.begin(), groupA.end(), link) == groupA.end() &&
               std::find(groupB.begin(), groupB.end(), link) == groupB.end() &&
               (link->isRotationalJoint() || link->isSlideJoint()) &&
               link->name().find("nail") == std::string::npos)
            {
                link->setActuationMode(Link::JointTorque);
                io->enableIO(link);
                fixedJoints.push_back(link);
                fixedRef.push_back(link->q());
                fixedPrev.push_back(link->q());
            }
        }

        joystick.readCurrentState();

        std::cout << "--- Initialization complete. " << fixedJoints.size()
                  << " joints fixed, " << groupA.size() + groupB.size()
                  << " joints moving. ---" << std::endl;

        return true;
    }

    virtual bool control() override
    {
        if(!is_control_started){
            std::cout << "--- Control loop started ---" << std::endl;
            is_control_started = true;
        }

        joystick.readCurrentState();
        double yAxis = joystick.getPosition(1);

        static const double P = 100.0;
        static const double D = 10.0;

        if(std::fabs(yAxis) > 0.2){
            phase += 2 * M_PI * motionFrequency * dt;
            phase = std::fmod(phase, 2 * M_PI);
            double a_pos = motionAmplitude * std::sin(phase);  // groupA
            double b_pos = motionAmplitude * std::sin(phase + M_PI);  // groupB（逆位相）

            for(auto joint : groupA){
                double q = joint->q();
                double dq = (q - q) / dt; // 実質0（簡略化）
                joint->u() = P * (a_pos - q) - D * dq;
            }

            for(auto joint : groupB){
                double q = joint->q();
                double dq = (q - q) / dt;
                joint->u() = P * (b_pos - q) - D * dq;
            }
        } else {
            for(auto joint : groupA) joint->u() = 0.0;
            for(auto joint : groupB) joint->u() = 0.0;
        }

        for(size_t i = 0; i < fixedJoints.size(); ++i){
            Link* joint = fixedJoints[i];
            double q = joint->q();
            double dq = (q - fixedPrev[i]) / dt;
            joint->u() = P * (fixedRef[i] - q) - D * dq;
            fixedPrev[i] = q;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(GriphisWalker)
