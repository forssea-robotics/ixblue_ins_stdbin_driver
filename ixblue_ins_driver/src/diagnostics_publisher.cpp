#include <ixblue_ins_driver/diagnostics_publisher.h>
#include <rclcpp/time.hpp>
#include <memory>
#include <bitset>

DiagnosticsPublisher::DiagnosticsPublisher(rclcpp::Node::SharedPtr n)
: diagnosticsUpdater(n, 1.0), node(n), steady_clock(RCL_STEADY_TIME)
{
    n->declare_parameter<double>("expected_frequency", 10.0);
    n->get_parameter("expected_frequency", expected_frequency);
    n->declare_parameter<double>("max_latency", 1.0);
    n->get_parameter("max_latency", max_latency);
    n->declare_parameter<double>("connection_lost_timeout", 10.0);
    n->get_parameter("connection_lost_timeout", connection_lost_timeout);

    RCLCPP_INFO(n->get_logger(), "Expected frequency for diagnostics : %.2f Hz", expected_frequency);
    RCLCPP_INFO(n->get_logger(), "Max latency acceptable for diagnostics : %.3f s", max_latency);
    RCLCPP_INFO(n->get_logger(), "Connection lost timeout : %.3f s", connection_lost_timeout);

    diagnosticsUpdater.add("status", this,
                           &DiagnosticsPublisher::produceStatusDiagnostics);
    stdImuTopicDiagnostic = std::make_unique<diagnostic_updater::TopicDiagnostic>(
        "imu topic", diagnosticsUpdater,
        diagnostic_updater::FrequencyStatusParam(&expected_frequency, &expected_frequency,
                                                 frequency_tolerance, 10),
        diagnostic_updater::TimeStampStatusParam(-max_latency, max_latency));
    using namespace std::chrono_literals;
    diagnosticsUpdater.setPeriod(0.1);
}

void DiagnosticsPublisher::setHardwareID(const std::string& hwId)
{
    diagnosticsUpdater.setHardwareID(hwId);
}

void DiagnosticsPublisher::stdImuTick(const rclcpp::Time& stamp)
{
    stdImuTopicDiagnostic->tick(stamp);
}

void DiagnosticsPublisher::updateStatus(
    const boost::optional<ixblue_stdbin_decoder::Data::INSSystemStatus>& systemStatus,
    const boost::optional<ixblue_stdbin_decoder::Data::INSAlgorithmStatus>&
        algorithmStatus)
{
    lastMessageReceivedStamp = steady_clock.now();
    lastSystemStatus = systemStatus;
    lastAlgorithmStatus = algorithmStatus;
}

void DiagnosticsPublisher::produceStatusDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& status)
{
    if(!lastAlgorithmStatus.is_initialized() || !lastSystemStatus.is_initialized())
    {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No data received yet");
    }
    else if((steady_clock.now() - lastMessageReceivedStamp).seconds() >
            connection_lost_timeout)
    {
        std::stringstream ss;
        ss << "No more data for more than " << connection_lost_timeout << " s";
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, ss.str());
    }
    else
    {
        const std::bitset<32> algoStatus1{lastAlgorithmStatus->status1};
        const std::bitset<32> algoStatus2{lastAlgorithmStatus->status2};
        const std::bitset<32> algoStatus3{lastAlgorithmStatus->status3};
        const std::bitset<32> algoStatus4{lastAlgorithmStatus->status4};

        const std::bitset<32> systemStatus1{lastSystemStatus->status1};
        const std::bitset<32> systemStatus2{lastSystemStatus->status2};
        const std::bitset<32> systemStatus3{lastSystemStatus->status3};

        status.add("algorithm_status_1", algoStatus1.to_string());
        status.add("algorithm_status_2", algoStatus2.to_string());
        status.add("algorithm_status_3", algoStatus3.to_string());
        status.add("algorithm_status_4", algoStatus4.to_string());

        status.add("system_status_1", systemStatus1.to_string());
        status.add("system_status_2", systemStatus2.to_string());
        status.add("system_status_3", systemStatus3.to_string());

        if(systemStatus1.test(
               ixblue_stdbin_decoder::Data::INSSystemStatus::Status1::SERIAL_IN_R_ERR))
        {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                           "Serial input error");
        }
        else if(systemStatus1.test(
                    ixblue_stdbin_decoder::Data::INSSystemStatus::Status1::INPUT_A_ERR))
        {
            // GNNS Input error on Atlans, Input A error on other systems
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                           "GNSS or Input A error");
        }
        // TODO other system status checks
        else if(systemStatus2.test(ixblue_stdbin_decoder::Data::INSSystemStatus::Status2::
                                       WAIT_FOR_POSITION))
        {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                           "System is waiting for position");
        }
        else if(algoStatus1.test(
                    ixblue_stdbin_decoder::Data::INSAlgorithmStatus::Status1::ALIGNMENT))
        {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           "System in alignment, do not move");
        }
        else if(algoStatus1.test(ixblue_stdbin_decoder::Data::INSAlgorithmStatus::
                                     Status1::FINE_ALIGNMENT))
        {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Fine alignment");
        }
        else if(algoStatus1.test(
                    ixblue_stdbin_decoder::Data::INSAlgorithmStatus::Status1::NAVIGATION))
        {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "System in navigation");
        }
        else
        {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
        }
    }
}
