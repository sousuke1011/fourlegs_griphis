#include <cnoid/SimpleController>
#include <vector>
#include <string> // std::string::find を使うためにインクルード
#include <iostream>

using namespace cnoid;

class FixedJointController : public SimpleController
{
    std::vector<Link*> joints;
    std::vector<double> q_ref;
    std::vector<double> q_prev;
    double dt;
    bool is_control_started = false; // control関数が呼ばれたかどうかのフラグ

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        joints.clear();
        q_ref.clear();
        q_prev.clear();

        auto body = io->body();
        dt = io->timeStep();

        std::cout << "--- Initializing FixedJointController (Selective Mode) ---" << std::endl;

        for(auto& link : body->links()){
            // ★★★ 修正点 ★★★
            // 回転・直動関節、かつ、名前に "nail" を含まない関節のみを対象にする
            if((link->isRotationalJoint() || link->isSlideJoint()) &&
               (link->name().find("nail") == std::string::npos))
            {
                std::cout << "Registering joint: " << link->name() << std::endl;
                link->setActuationMode(Link::JointTorque);
                io->enableIO(link);
                
                joints.push_back(link);
                q_ref.push_back(link->q());
                q_prev.push_back(link->q());
            } else if (link->jointId() >= 0) {
                // 対象外の関節も一応表示してみる
                std::cout << "Skipping joint: " << link->name() << std::endl;
            }
        }

        std::cout << "--- Initialization complete. " << joints.size() << " joints registered. ---" << std::endl;
        return true;
    }

    virtual bool control() override
    {
        // 最初の1フレームだけメッセージを表示する
        if(!is_control_started){
            std::cout << "--- Control loop successfully started. ---" << std::endl;
            is_control_started = true;
        }
        
        static const double P = 200.0;
        static const double D = 50.0;

        for(size_t i = 0; i < joints.size(); ++i){
            Link* joint = joints[i];
            
            double q = joint->q();
            double dq = (q - q_prev[i]) / dt;
            double dq_ref = 0.0;

            joint->u() = P * (q_ref[i] - q) + D * (dq_ref - dq);
            
            q_prev[i] = q;
        }
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(FixedJointController)
