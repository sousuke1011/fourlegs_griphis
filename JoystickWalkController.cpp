#include <cnoid/SimpleController>
#include <cnoid/Joystick>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

using namespace cnoid;

class JoystickWalkController : public SimpleController
{
    Joystick joystick;
    Link* targetJoint;
    std::vector<Link*> fixedJoints;
    std::vector<double> fixedRef;
    std::vector<double> fixedPrev;
    double prevTargetQ = 0.0;
    double dt;
    bool is_control_started = false;

    // モーション制御用
    double phase = 0.0;
    const double motionFrequency = 0.5; // Hz
    const double motionAmplitude = 10.0; // ラジアンに変換

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        auto body = io->body();
        dt = io->timeStep();

        std::cout << "--- Initializing JoystickWalkController ---" << std::endl;

        targetJoint = body->link("griphis_1_pitch_1_link");
        if(!targetJoint){
            std::cerr << "Target joint 'griphis_1_yaw_1_link' not found!" << std::endl;
            return false;
        }

        targetJoint->setActuationMode(Link::JointTorque);
        io->enableIO(targetJoint);
        prevTargetQ = targetJoint->q();

        for(auto& link : body->links()){
            if(link != targetJoint &&
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

        joystick.readCurrentState(); // 初期読み取り

        std::cout << "--- Initialization complete. " << fixedJoints.size()
                  << " joints fixed, 1 joint moving. ---" << std::endl;

        return true;
    }

    virtual bool control() override
    {
        if(!is_control_started){
            std::cout << "--- Control loop started ---" << std::endl;
            is_control_started = true;
        }

        joystick.readCurrentState();
        double yAxis = joystick.getPosition(1); // 必要に応じて axis を変更

        static const double P = 200.0;
        static const double D = 30.0;

        // デバッグ出力（必要なら消してOK）
        std::cout << "Joystick Y Axis (2): " << yAxis << std::endl;
	bool ok = joystick.readCurrentState();
	std::cout << "Joystick read status: " << std::boolalpha << ok << std::endl;


        // 動かす関節の制御
        if(std::fabs(yAxis) > 0.2){
            phase += 2 * M_PI * motionFrequency * dt;
            phase = std::fmod(phase, 2 * M_PI); // オーバーフロー防止

            double q_desired = motionAmplitude * std::sin(phase);
            double q = targetJoint->q();
            double dq = (q - prevTargetQ) / dt;
            double dq_ref = 0.0;

            targetJoint->u() = P * (q_desired - q) + D * (dq_ref - dq);
            prevTargetQ = q;
        } else {
            // 停止：トルクゼロにして保持
            targetJoint->u() = 0.0;
        }

        // 固定関節のトルク制御
        for(size_t i = 0; i < fixedJoints.size(); ++i){
            Link* joint = fixedJoints[i];
            double q = joint->q();
            double dq = (q - fixedPrev[i]) / dt;
            double dq_ref = 0.0;

            joint->u() = P * (fixedRef[i] - q) + D * (dq_ref - dq);
            fixedPrev[i] = q;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JoystickWalkController)
