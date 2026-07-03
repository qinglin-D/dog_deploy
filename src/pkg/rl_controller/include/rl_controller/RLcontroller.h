#pragma once

#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <thread>
#include <mutex>

#include "Types_.h"

#ifdef CONFIG_PATH
#define POLICY_CONFIG CONFIG_PATH "/policy.yaml"
#else
#define POLICY_CONFIG ""
#endif
#ifdef MODEL_PATH
#define ENCODER_1_PATH MODEL_PATH "/crawl_encoder.onnx"
#define POLICY_1_PATH MODEL_PATH "/crawl_policy.onnx"
#define ENCODER_2_PATH MODEL_PATH "/trot_encoder.onnx"
#define POLICY_2_PATH MODEL_PATH "/trot_policy.onnx"
#else
#define ENCODER_1_PATH ""
#define POLICY_1_PATH ""
#define ENCODER_2_PATH ""
#define POLICY_2_PATH ""
#endif

class Controller
{
using tensor_element_t = float;
public:
    Controller(std::string node_name="controller");
    ~Controller(){};

    void start();

private:
    void LoadModel();
    void LoadRLCfg();

    void RecCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void Publish();
    void ComputeActions();
    void ComputePolicyObservation();
    void ComputeEncoderActions();
    void ComputeHistoryObservation();
    void ButtonStateCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void PolicyIndexCallback(const std_msgs::msg::Int32::SharedPtr msg);

    std::string configPath_{POLICY_CONFIG};
    int active_model_index_{0};
    int num_models_{0};
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr button_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr policy_index_sub_;

    std::shared_ptr<Ort::Env> onnxEnvPrt_;

    std::vector<std::unique_ptr<Ort::Session>> encoderSessionPtrs_;
    std::vector<std::vector<const char *>> encoderInputNames_;
    std::vector<std::vector<const char *>> encoderOutputNames_;
    std::vector<std::vector<Ort::AllocatedStringPtr>> encoderInputNodeNameAllocatedStrings_;
    std::vector<std::vector<Ort::AllocatedStringPtr>> encoderOutputNodeNameAllocatedStrings_;
    std::vector<std::vector<std::vector<int64_t>>> encoderInputShapes_;
    std::vector<std::vector<std::vector<int64_t>>> encoderOutputShapes_;

    std::vector<std::unique_ptr<Ort::Session>> policySessionPtrs_;
    std::vector<std::vector<const char *>> policyInputNames_;
    std::vector<std::vector<const char *>> policyOutputNames_;
    std::vector<std::vector<Ort::AllocatedStringPtr>> policyInputNodeNameAllocatedStrings_;
    std::vector<std::vector<Ort::AllocatedStringPtr>> policyOutputNodeNameAllocatedStrings_;
    std::vector<std::vector<std::vector<int64_t>>> policyInputShapes_;
    std::vector<std::vector<std::vector<int64_t>>> policyOutputShapes_;

    std::vector<CommandRange> command_ranges_;

    Proprioception proprioception_;
    Command command_;
    vector_t lastActions_;
    vector_t defaultJointAngles_;
    std::vector<float> actions_;
    std::vector<float> encoderOutputs_;
    Actions actionsCfg_;
    std::vector<tensor_element_t> policyObservations_;
    std::vector<tensor_element_t> historyObservations_;

    float frequency_{50};
    float cycle_{0};

    int actionsSize_{12};
    int observationSize_{45};
    int stackSize_{4};
    int encoderOutputSize_{3};
    int input_size_;

    double phase{0};
    bool isfirstRecv_{true};

    Ort::MemoryInfo memoryInfo;
    Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
    Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> policyHistoryBuffer_;
    ThreadLock threadLock_;
    std::atomic_bool is_runing{true};

private:
    vector3_t temp_command_;
    vector_t temp_actions_;
};
