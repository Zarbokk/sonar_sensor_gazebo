//
// Created by tim on 05.03.21.
//

#ifndef SONAR_SENSOR_GAZEBO_ROTATINGSONARCONTROLPLUGIN_CC_H
#define SONAR_SENSOR_GAZEBO_ROTATINGSONARCONTROLPLUGIN_CC_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo {
    /// \brief A plugin to control a Velodyne sensor.
    class RotatingSonarControlPlugin : public ModelPlugin {
        /// \brief Constructor
    public:
        RotatingSonarControlPlugin() {}


        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            double velocity = 0;//default velocity this is rad/s
            if (_sdf->HasElement("velocity"))
                velocity = _sdf->Get<double>("velocity");
            // Safety check
            if (_model->GetJointCount() == 0) {
                std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
                return;
            }
            // Store the model pointer for convenience.
            this->model = _model;

            // Get the first joint. We are making an assumption about the model
            // having one joint that is the rotational joint.
            this->joint = _model->GetJoints()[0];

            // Setup a P-controller, with a gain of 0.1.
            this->pid = common::PID(0.1, 0, 0);

            // Apply the P-controller to the joint.
            this->model->GetJointController()->SetVelocityPID(
                    this->joint->GetScopedName(), this->pid);

            // Set the joint's target velocity. This target velocity is just
            // for demonstration purposes.
            this->model->GetJointController()->SetVelocityTarget(
                    this->joint->GetScopedName(), velocity);
            // Create the node
            
        }

    private:


        /// \brief Pointer to the model.
        physics::ModelPtr model;

        /// \brief Pointer to the joint.
        physics::JointPtr joint;

        /// \brief A PID controller for the joint.
        common::PID pid;
        /// \brief A node used for transport
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(RotatingSonarControlPlugin)
}
#endif