/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelocityProvider.hpp"
#include <uwv_kalman_filters/VelocityUKF.hpp>

using namespace uwv_kalman_filters;

VelocityProvider::VelocityProvider(std::string const& name)
    : VelocityProviderBase(name)
{
}

VelocityProvider::VelocityProvider(std::string const& name, RTT::ExecutionEngine* engine)
    : VelocityProviderBase(name, engine)
{
}

VelocityProvider::~VelocityProvider()
{
}

void VelocityProvider::orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_sample)
{
    current_orientation = orientation_sample;
    velocity_filter->updateOrientation(current_orientation.orientation);
}

void VelocityProvider::dvl_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dvl_velocity_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d dvlInBody;
    if (!_dvl2body.get(ts, dvlInBody))
    {
        RTT::log(RTT::Error) << "skip, couldn't receive a valid dvl-in-body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    if(dvl_velocity_samples_sample.hasValidVelocity() && dvl_velocity_samples_sample.hasValidVelocityCovariance())
    {
        base::Vector3d velocity = dvlInBody.rotation() * dvl_velocity_samples_sample.velocity;
        if(base::isnotnan(current_angular_velocity))
        {
            velocity -= current_angular_velocity.cross(dvlInBody.translation());
        }

        // apply new velocity measurement
        VelocityUKF::DVLMeasurement measurement;
        measurement.mu = velocity;
        measurement.cov = dvlInBody.rotation() * dvl_velocity_samples_sample.cov_velocity * dvlInBody.rotation().transpose();

        predictionStep(ts);
        try
        {
            velocity_filter->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add DVL measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Info) << "DVL measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void VelocityProvider::imu_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_sensor_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d imuInBody;
    if (!_imu2body.get(ts, imuInBody))
    {
        RTT::log(RTT::Error) << "skip, couldn't receive a valid imu-in-body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    if(base::isnotnan(imu_sensor_samples_sample.gyro))
    {
        // apply new gyro measurement
        VelocityUKF::GyroMeasurement measurement;
        current_angular_velocity = imuInBody.rotation() * imu_sensor_samples_sample.gyro;
        measurement.mu = current_angular_velocity;
        measurement.cov = _cov_angular_velocity.value();

        predictionStep(ts);
        try
        {
            velocity_filter->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add angular velocity measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Angular velocity measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void VelocityProvider::body_effortsTransformerCallback(const base::Time &ts, const ::base::commands::LinearAngular6DCommand &body_efforts_sample)
{
    if(!base::isnotnan(body_efforts_sample.linear) || !base::isnotnan(body_efforts_sample.angular))
    {
        RTT::log(RTT::Error) << "Body effort measurements contain NaN values! Measurement will be skipped." << RTT::endlog();
        return;
    }

    VelocityUKF::BodyEffortsMeasurement measurement;
    measurement.mu.resize(6);
    measurement.mu.block(0,0,3,1) = body_efforts_sample.linear;
    measurement.mu.block(3,0,3,1) = body_efforts_sample.angular;

    predictionStep(ts);
    // apply body efforts measurement
    try
    {
        velocity_filter->integrateMeasurement(measurement);
    }
    catch(const std::runtime_error& e)
    {
        RTT::log(RTT::Error) << "Failed to add thruster speed measurements: " << e.what() << RTT::endlog();
    }
}

void VelocityProvider::pressure_sensor_samplesTransformerCallback(const base::Time& ts, const base::samples::RigidBodyState& pressure_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d pressureSensorInBody;
    if (!_pressure_sensor2body.get(ts, pressureSensorInBody))
    {
        RTT::log(RTT::Error) << "skip, couldn't receive a valid pressure-sensor-in-body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    if(!base::isNaN(pressure_samples_sample.position.z()) && !base::isNaN(pressure_samples_sample.cov_position(2,2)))
    {
        Eigen::Matrix<double, 1, 1> depth;
        depth << pressure_samples_sample.position.z() - pressureSensorInBody.translation().z();

        // apply depth measurement
        VelocityUKF::PressureMeasurement measurement;
        measurement.mu = depth;
        measurement.cov = pressure_samples_sample.cov_position.block(2,2,1,1);

        predictionStep(ts);
        try
        {
            velocity_filter->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add depth measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Depth measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void VelocityProvider::predictionStep(const base::Time& sample_time)
{
    try
    {
        velocity_filter->predictionStepFromSampleTime(sample_time);
    }
    catch(const std::runtime_error& e)
    {
        RTT::log(RTT::Error) << "Failed to execute prediction step: " << e.what() << RTT::endlog();
        RTT::log(RTT::Error) << "Skipping prediction step." << RTT::endlog();
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelocityProvider.hpp for more detailed
// documentation about them.

bool VelocityProvider::configureHook()
{
    if (! VelocityProviderBase::configureHook())
        return false;

    // setup initial state
    VelocityUKF::State init_state;
    VelocityUKF::Covariance state_cov;
    // linear velocity
    init_state.velocity = VelocityType(MTK::vect<3, double>::Zero());
    init_state.z_position = ZPosType(MTK::vect<1, double>::Zero());
    state_cov = 0.1 * VelocityUKF::Covariance::Identity();
    state_cov(3,3) = 100.0;
    velocity_filter.reset(new VelocityUKF(init_state, state_cov));

    // set model parameters
    if (!velocity_filter->setupMotionModel(_model_parameters.value()))
    {
        RTT::log(RTT::Error) << "Failed to setup motion model!" << RTT::endlog();
        return false;
    }

    // setup process noise
    const VelocityProcessNoise& process_noise = _process_noise.value();
    VelocityUKF::Covariance process_noise_cov = VelocityUKF::Covariance::Zero();

    if(base::isnotnan(process_noise.velocity_noise))
        process_noise_cov.block(0,0,3,3) = process_noise.velocity_noise;
    else
        process_noise_cov.block(0,0,3,3) = 0.001 * base::Matrix3d::Identity();

    if(!base::isNaN(process_noise.depth_noise))
        process_noise_cov(3,3) = process_noise.depth_noise;
    else
        process_noise_cov(3,3) = 0.01;

    velocity_filter->setProcessNoiseCovariance(process_noise_cov);

    velocity_filter->setMaxTimeDelta(_max_time_delta.get());

    // setup stream alignment verifier
    verifier.reset(new pose_estimation::StreamAlignmentVerifier());
    verifier->setVerificationInterval(20.0);
    verifier->setDropRateWarningThreshold(0.5);
    verifier->setDropRateCriticalThreshold(1.0);
    streams_with_alignment_failures = 0;
    streams_with_critical_alignment_failures = 0;

    current_angular_velocity = Eigen::Vector3d::Zero();
    last_sample_time = base::Time();
    current_orientation.invalidate();

    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;

    return true;
}
bool VelocityProvider::startHook()
{
    if (! VelocityProviderBase::startHook())
        return false;
    return true;
}
void VelocityProvider::updateHook()
{
    new_state = RUNNING;
    VelocityProviderBase::updateHook();

    // check stream alignment status
    verifier->verifyStreamAlignerStatus(_transformer.getStreamAlignerStatus(), streams_with_alignment_failures, streams_with_critical_alignment_failures);
    if(streams_with_alignment_failures > 0)
        new_state = TRANSFORMATION_ALIGNMENT_FAILURES;
    if(streams_with_critical_alignment_failures > 0)
        exception(CRITICAL_ALIGNMENT_FAILURE);

    // write estimated body state
    VelocityUKF::State current_state;
    VelocityUKF::Covariance state_cov;
    base::Time current_sample_time = velocity_filter->getLastMeasurementTime();
    if(current_sample_time > last_sample_time && velocity_filter->getCurrentState(current_state, state_cov))
    {
        base::samples::RigidBodyState velocity_sample;
        velocity_sample.velocity = current_state.velocity;
        velocity_sample.position.z() = current_state.z_position(0);
        velocity_sample.angular_velocity = current_angular_velocity;
        if(current_orientation.hasValidOrientation())
            velocity_sample.orientation = current_orientation.orientation;
        else
            velocity_sample.orientation = base::Orientation::Identity();
        velocity_sample.cov_velocity = state_cov.block(0,0,3,3);
        velocity_sample.cov_position(2,2) = state_cov(3,3);
        velocity_sample.cov_angular_velocity = _cov_angular_velocity.value();
        velocity_sample.cov_orientation = current_orientation.cov_orientation;
        velocity_sample.time = current_sample_time;
        velocity_sample.targetFrame = _target_frame.value();
        velocity_sample.sourceFrame = _target_frame.value();
        _velocity_samples.write(velocity_sample);
        last_sample_time = current_sample_time;
    }

    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void VelocityProvider::errorHook()
{
    VelocityProviderBase::errorHook();
}
void VelocityProvider::stopHook()
{
    VelocityProviderBase::stopHook();
}
void VelocityProvider::cleanupHook()
{
    VelocityProviderBase::cleanupHook();
}
