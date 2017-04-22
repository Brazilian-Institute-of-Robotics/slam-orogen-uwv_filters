/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UWV_KALMAN_FILTERS_VELOCITYPROVIDER_TASK_HPP
#define UWV_KALMAN_FILTERS_VELOCITYPROVIDER_TASK_HPP

#include "uwv_kalman_filters/VelocityProviderBase.hpp"
#include <boost/shared_ptr.hpp>
#include <pose_estimation/StreamAlignmentVerifier.hpp>
#include <uwv_kalman_filters/uwv_kalman_filtersTypes.hpp>
#include <base/commands/LinearAngular6DCommand.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/IMUSensors.hpp>

namespace uwv_kalman_filters {

    class VelocityUKF;

    /*! \class VelocityProvider 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','uwv_kalman_filters::VelocityProvider')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class VelocityProvider : public VelocityProviderBase
    {
	friend class VelocityProviderBase;
    protected:
        boost::shared_ptr<uwv_kalman_filters::VelocityUKF> velocity_filter;
        boost::shared_ptr<pose_estimation::StreamAlignmentVerifier> verifier;
        unsigned streams_with_alignment_failures;
	unsigned streams_with_critical_alignment_failures;
        Eigen::Vector3d current_angular_velocity;
        base::Time last_sample_time;
        States last_state;
        States new_state;
        base::samples::RigidBodyState current_orientation;


        virtual void dvl_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dvl_velocity_samples_sample);

        virtual void imu_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_sensor_samples_sample);

        virtual void body_effortsTransformerCallback(const base::Time &ts, const ::base::commands::LinearAngular6DCommand &body_efforts);

        virtual void pressure_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pressure_samples_sample);
        
        virtual void orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_sample);

        void predictionStep(const base::Time& sample_time);

    public:
        /** TaskContext constructor for VelocityProvider
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        VelocityProvider(std::string const& name = "uwv_kalman_filters::VelocityProvider");

        /** TaskContext constructor for VelocityProvider 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        VelocityProvider(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of VelocityProvider
         */
	~VelocityProvider();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

