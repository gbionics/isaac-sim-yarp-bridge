// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause

#ifndef ISAACSIM_MAS_SENSORS_NWC_ROS2_H
#define ISAACSIM_MAS_SENSORS_NWC_ROS2_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IPreciselyTimed.h>

#include <rclcpp/node.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include "IsaacSimControlBoardNWCROS2_ParamsParser.h"

namespace yarp::dev
{
    class IsaacSimControlBoardNWCROS2;
}


class yarp::dev::IsaacSimControlBoardNWCROS2 :
    public yarp::dev::DeviceDriver,
    public yarp::dev::IPidControl,
    public yarp::dev::IPositionControl,
    public yarp::dev::IPositionDirect,
    public yarp::dev::IVelocityControl,
    public yarp::dev::IPWMControl,
    public yarp::dev::ICurrentControl,
    public yarp::dev::IEncodersTimed,
    public yarp::dev::IMotor,
    public yarp::dev::IMotorEncoders,
    public yarp::dev::IAmplifierControl,
    public yarp::dev::IControlLimits,
    public yarp::dev::IRemoteCalibrator,
    public yarp::dev::IControlCalibration,
    public yarp::dev::ITorqueControl,
    public yarp::dev::IImpedanceControl,
    public yarp::dev::IControlMode,
    public yarp::dev::IAxisInfo,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::IInteractionMode,
    public yarp::dev::IRemoteVariables,
    public yarp::dev::IJointFault
{

public:
    IsaacSimControlBoardNWCROS2() = default;
    ~IsaacSimControlBoardNWCROS2() override;

    /* DeviceDriver methods */
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    /* IPidControl */
    bool setPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, const yarp::dev::Pid& p) override;

    bool setPids(const yarp::dev::PidControlTypeEnum& pidtype, const yarp::dev::Pid* ps) override;

    bool setPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double ref) override;

    bool setPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, const double* refs) override;

    bool setPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double limit) override;

    bool setPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, const double* limits) override;

    bool getPidError(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* err) override;

    bool getPidErrors(const yarp::dev::PidControlTypeEnum& pidtype, double* errs) override;

    bool getPidOutput(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* out) override;

    bool getPidOutputs(const yarp::dev::PidControlTypeEnum& pidtype, double* outs) override;

    bool setPidOffset(const yarp::dev::PidControlTypeEnum& pidtype, int j, double v) override;

    bool getPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, yarp::dev::Pid* p) override;

    bool getPids(const yarp::dev::PidControlTypeEnum& pidtype, yarp::dev::Pid* pids) override;

    bool getPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* ref) override;

    bool getPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, double* refs) override;

    bool getPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* limit) override;

    bool getPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, double* limits) override;

    bool resetPid(const yarp::dev::PidControlTypeEnum& pidtype, int j) override;

    bool disablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j) override;

    bool enablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j) override;

    bool isPidEnabled(const yarp::dev::PidControlTypeEnum& pidtype, int j, bool* enabled) override;

    /* IPositionControl */
    bool getAxes(int* ax) override;

    bool positionMove(int j, double ref) override;

    bool positionMove(const double* refs) override;

    bool positionMove(const int n_joints, const int* joints, const double* refs) override;

    bool getTargetPosition(const int joint, double* ref) override;

    bool getTargetPositions(double* refs) override;

    bool getTargetPositions(const int n_joint, const int* joints, double* refs) override;

    bool relativeMove(int j, double delta) override;

    bool relativeMove(const double* deltas) override;

    bool relativeMove(const int n_joints, const int* joints, const double* deltas) override;

    bool checkMotionDone(int j, bool* flag) override;

    bool checkMotionDone(bool* flag) override;

    bool checkMotionDone(const int n_joints, const int* joints, bool* flags) override;

    bool setRefSpeed(int j, double sp) override;

    bool setRefSpeeds(const double* spds) override;

    bool setRefSpeeds(const int n_joints, const int* joints, const double* spds) override;

    bool setRefAcceleration(int j, double acc) override;

    bool setRefAccelerations(const double* accs) override;

    bool setRefAccelerations(const int n_joints, const int* joints, const double* accs) override;

    bool getRefSpeed(int j, double* ref) override;

    bool getRefSpeeds(double* spds) override;

    bool getRefSpeeds(const int n_joints, const int* joints, double* spds) override;

    bool getRefAcceleration(int j, double* acc) override;

    bool getRefAccelerations(double* accs) override;

    bool getRefAccelerations(const int n_joints, const int* joints, double* accs) override;

    bool stop(int j) override;

    bool stop() override;

    bool stop(const int n_joints, const int* joints) override;

    /* IJointFault */

    bool getLastJointFault(int j, int& fault, std::string& message) override;

    /* IVelocityControl */
    bool velocityMove(int j, double v) override;

    bool velocityMove(const double* v) override;

    /* IEncoders */
    bool resetEncoder(int j) override;

    bool resetEncoders() override;

    bool setEncoder(int j, double val) override;

    bool setEncoders(const double* vals) override;

    bool getEncoder(int j, double* v) override;

    bool getEncoders(double* encs) override;

    bool getEncodersTimed(double* encs, double* t) override;

    bool getEncoderTimed(int j, double* v, double* t) override;

    bool getEncoderSpeed(int j, double* sp) override;

    bool getEncoderSpeeds(double* spds) override;

    bool getEncoderAcceleration(int j, double* acc) override;

    bool getEncoderAccelerations(double* accs) override;

    /* IMotorEncoders */
    bool getNumberOfMotorEncoders(int* num) override;

    bool resetMotorEncoder(int m) override;

    bool resetMotorEncoders() override;

    bool setMotorEncoderCountsPerRevolution(int m, const double cpr) override;

    bool getMotorEncoderCountsPerRevolution(int m, double* cpr) override;

    bool setMotorEncoder(int m, const double val) override;

    bool setMotorEncoders(const double* vals) override;

    bool getMotorEncoder(int m, double* v) override;

    bool getMotorEncoders(double* encs) override;

    bool getMotorEncodersTimed(double* encs, double* t) override;

    bool getMotorEncoderTimed(int m, double* v, double* t) override;

    bool getMotorEncoderSpeed(int m, double* sp) override;

    bool getMotorEncoderSpeeds(double* spds) override;

    bool getMotorEncoderAcceleration(int m, double* acc) override;

    bool getMotorEncoderAccelerations(double* accs) override;

    /* IAmplifierControl */
    bool enableAmp(int j) override;

    bool disableAmp(int j) override;

    bool getAmpStatus(int* st) override;

    bool getAmpStatus(int j, int* v) override;

    bool setMaxCurrent(int j, double v) override;

    bool getMaxCurrent(int j, double* v) override;

    bool getNominalCurrent(int m, double* val) override;

    bool setNominalCurrent(int m, const double val) override;

    bool getPeakCurrent(int m, double* val) override;

    bool setPeakCurrent(int m, const double val) override;

    bool getPWM(int m, double* val) override;

    bool getPWMLimit(int m, double* val) override;

    bool setPWMLimit(int m, const double val) override;

    bool getPowerSupplyVoltage(int m, double* val) override;

    /* IControlLimits */
    bool setLimits(int j, double min, double max) override;

    bool getLimits(int j, double* min, double* max) override;

    bool setVelLimits(int j, double min, double max) override;

    bool getVelLimits(int j, double* min, double* max) override;

    /* IRemoteVariables */

    bool getRemoteVariable(std::string key, yarp::os::Bottle& val) override;

    bool setRemoteVariable(std::string key, const yarp::os::Bottle& val) override;

    bool getRemoteVariablesList(yarp::os::Bottle* listOfKeys) override;

    /* IRemoteCalibrator */

    bool isCalibratorDevicePresent(bool* isCalib) override;

    yarp::dev::IRemoteCalibrator* getCalibratorDevice() override;

    bool calibrateSingleJoint(int j) override;

    bool calibrateWholePart() override;

    bool homingSingleJoint(int j) override;

    bool homingWholePart() override;

    bool parkSingleJoint(int j, bool _wait = true) override;

    bool parkWholePart() override;

    bool quitCalibrate() override;

    bool quitPark() override;

    /* IControlCalibration */

    bool calibrateAxisWithParams(int j, unsigned int ui, double v1, double v2, double v3) override;

    bool setCalibrationParameters(int j, const yarp::dev::CalibrationParameters& params) override;

    bool calibrationDone(int j) override;

    bool abortPark() override;

    bool abortCalibration() override;

    /* IMotor */
    bool getNumberOfMotors(int* num) override;

    bool getTemperature(int m, double* val) override;

    bool getTemperatures(double* vals) override;

    bool getTemperatureLimit(int m, double* val) override;

    bool setTemperatureLimit(int m, const double val) override;

    bool getGearboxRatio(int m, double* val) override;

    bool setGearboxRatio(int m, const double val) override;

    /* IAxisInfo */
    bool getAxisName(int j, std::string& name) override;

    bool getJointType(int j, yarp::dev::JointTypeEnum& type) override;

    bool getRefTorques(double* refs) override;

    bool getRefTorque(int j, double* t) override;

    bool setRefTorques(const double* t) override;

    bool setRefTorque(int j, double t) override;

    bool setRefTorques(const int n_joint, const int* joints, const double* t) override;

    bool getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params) override;

    bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override;

    bool setImpedance(int j, double stiff, double damp) override;

    bool setImpedanceOffset(int j, double offset) override;

    bool getTorque(int j, double* t) override;

    bool getTorques(double* t) override;

    bool getTorqueRange(int j, double* min, double* max) override;

    bool getTorqueRanges(double* min, double* max) override;

    bool getImpedance(int j, double* stiff, double* damp) override;

    bool getImpedanceOffset(int j, double* offset) override;

    bool getCurrentImpedanceLimit(int j, double* min_stiff, double* max_stiff, double* min_damp, double* max_damp) override;

    bool getControlMode(int j, int* mode) override;

    bool getControlModes(int* modes) override;

    // IControlMode interface
    bool getControlModes(const int n_joint, const int* joints, int* modes) override;

    bool setControlMode(const int j, const int mode) override;

    bool setControlModes(const int n_joints, const int* joints, int* modes) override;

    bool setControlModes(int* modes) override;

    bool setPosition(int j, double ref) override;

    bool setPositions(const int n_joints, const int* joints, const double* dpos) override;

    bool setPositions(const double* refs) override;

    bool getRefPosition(const int joint, double* ref) override;

    bool getRefPositions(double* refs) override;

    bool getRefPositions(const int n_joint, const int* joints, double* refs) override;

    yarp::os::Stamp getLastInputStamp() override;

    // IVelocityControl interface
    bool velocityMove(const int n_joints, const int* joints, const double* spds) override;

    bool getRefVelocity(const int joint, double* vel) override;

    bool getRefVelocities(double* vels) override;

    bool getRefVelocities(const int n_joint, const int* joints, double* vels) override;

    bool getInteractionMode(int j, yarp::dev::InteractionModeEnum* mode) override;

    bool getInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes) override;

    bool getInteractionModes(yarp::dev::InteractionModeEnum* modes) override;

    bool setInteractionMode(int j, yarp::dev::InteractionModeEnum mode) override;

    bool setInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes) override;

    bool setInteractionModes(yarp::dev::InteractionModeEnum* modes) override;

    // IPWMControl
    bool setRefDutyCycle(int m, double ref) override;

    bool setRefDutyCycles(const double* refs) override;

    bool getRefDutyCycle(int m, double* ref) override;

    bool getRefDutyCycles(double* refs) override;

    bool getDutyCycle(int m, double* val) override;

    bool getDutyCycles(double* vals) override;

    // ICurrentControl
    bool getCurrent(int m, double* curr) override;

    bool getCurrents(double* currs) override;

    bool getCurrentRange(int m, double* min, double* max) override;

    bool getCurrentRanges(double* min, double* max) override;

    bool setRefCurrents(const double* currs) override;

    bool setRefCurrent(int m, double curr) override;

    bool setRefCurrents(const int n_motor, const int* motors, const double* currs) override;

    bool getRefCurrents(double* currs) override;

    bool getRefCurrent(int m, double* curr) override;

private:

    bool setup();

    void updateJointMeasurements(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

    void updateMotorMeasurements(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

    struct JointsState
    {
        std::vector<std::string> name;
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> effort;
        std::vector<int64_t> jointTypes;
        yarp::os::Stamp timestamp;
        std::atomic<bool> valid{ false };
        mutable std::mutex mutex;
        void convert_to_vectors(const sensor_msgs::msg::JointState::ConstSharedPtr& js);
        void convert_to_msg(sensor_msgs::msg::JointState& js) const;
        void resize();
        void invalidate();
    };

    class CBNode : public rclcpp::Node
    {
    public:

        using Parameters = std::vector<std::pair<std::string, uint8_t>>;
        explicit CBNode(const std::string& node_name,
                        const std::string& joint_state_topic_name,
                        const std::string& motor_state_topic_name,
                        const std::string& joint_references_topic_name,
                        const std::string& get_param_service_name,
                        const std::string& set_param_service_name,
                        double requests_timeout_sec,
                        IsaacSimControlBoardNWCROS2* parent);

        std::vector<rcl_interfaces::msg::ParameterValue> getParameters(const Parameters& parameters);

        std::vector<rcl_interfaces::msg::SetParametersResult> setParameters(const std::vector<rcl_interfaces::msg::Parameter>& params);

        void publishReferences(JointsState& msg);

        bool areServicesAvailable();

    private:
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_jointStateSubscription;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_motorStateSubscription;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_referencesPublisher;
        rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr m_getParamClient;
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr m_setParamClient;
        sensor_msgs::msg::JointState m_referencesMessageBuffer;
        std::chrono::duration<double> m_requestsTimeout;
    };

    std::shared_ptr<CBNode> m_node;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::mutex m_mutex;
    std::thread m_executorThread;
    IsaacSimControlBoardNWCROS2_ParamsParser m_paramsParser;
    JointsState m_jointState, m_motorState, m_jointReferences;
    std::vector<std::string> m_jointNames;
    std::atomic<bool> m_ready{ false };
};
#endif
