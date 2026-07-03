#include <iostream>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include "rl_controller/RLcontroller.h"

Controller::Controller(std::string node_name) : memoryInfo(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)){
    this->LoadRLCfg();
    this->LoadModel();
    this->input_size_ = observationSize_ * stackSize_;

    this->node_ = rclcpp::Node::make_shared(node_name);
    this->publisher_ = this->node_->create_publisher<std_msgs::msg::Float32MultiArray>("actions", 10);
    this->subscriber_ = this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "observation", 10, std::bind(&Controller::RecCallback, this, std::placeholders::_1));
    this->button_state_sub_ = this->node_->create_subscription<std_msgs::msg::Int32>(
        "button_state", 10, std::bind(&Controller::ButtonStateCallback, this, std::placeholders::_1));
    this->policy_index_sub_ = this->node_->create_subscription<std_msgs::msg::Int32>(
        "policy_index", 10, std::bind(&Controller::PolicyIndexCallback, this, std::placeholders::_1));

    int interval = 1000 / frequency_;
    this->timer_ = this->node_->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&Controller::Publish, this));
}


void Controller::start(){
    rclcpp::ExecutorOptions options;
    rclcpp::executors::MultiThreadedExecutor executor(options, 3);
    executor.add_node(this->node_);
    executor.spin();
    rclcpp::shutdown();
}


void Controller::LoadRLCfg(){
    try {
        YAML::Node config = YAML::LoadFile(configPath_);
        proprioception_.scale.baseAngVel = config["observations"]["scale"]["ang_vel"].as<scalar_t>();
        proprioception_.scale.jointPos = config["observations"]["scale"]["dof_pos"].as<scalar_t>();
        proprioception_.scale.jointVel = config["observations"]["scale"]["dof_vel"].as<scalar_t>();
        proprioception_.clipObs = config["observations"]["clip"].as<scalar_t>();

        frequency_ = config["robot"]["frequency"].as<float>();
        actionsSize_ = config["robot"]["actions"]["size"].as<int>();
        actionsCfg_.clipActions = config["robot"]["actions"]["clip"].as<scalar_t>();
        encoderOutputSize_ = config["observations"]["encoder_output"].as<int>();
        observationSize_ = config["observations"]["slice_size"].as<int>();
        stackSize_ = config["observations"]["history_length"].as<int>();
        cycle_ = config["observations"]["cycle"].as<float>();

        lastActions_.resize(actionsSize_);
        temp_actions_.resize(actionsSize_);
        actions_.resize(actionsSize_);
        encoderOutputs_.resize(encoderOutputSize_);
        actionsCfg_.actionScale.resize(actionsSize_);
        defaultJointAngles_.resize(actionsSize_);
        policyObservations_.resize(observationSize_ * stackSize_ + encoderOutputSize_);
        historyObservations_.resize(observationSize_ * stackSize_);
        proprioHistoryBuffer_.resize(observationSize_ * stackSize_);
        policyHistoryBuffer_.resize(observationSize_ * stackSize_ + encoderOutputSize_);
        proprioception_.jointPos.resize(actionsSize_);
        proprioception_.jointVel.resize(actionsSize_);
        std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
        std::fill(lastActions_.begin(), lastActions_.end(), 0.0f);
        std::fill(proprioHistoryBuffer_.begin(), proprioHistoryBuffer_.end(), 0.0f);

        std::vector<std::string> JointNames = config["robot"]["joints"]["joint_names"].as<std::vector<std::string>>();
        int idx = 0;
        for (const auto& item : JointNames) {
            actionsCfg_.actionScale[idx] = config["robot"]["actions"]["scale"][item].as<scalar_t>();
            idx++;
        }
        idx = 0;
        for (const auto& item : JointNames) {
            defaultJointAngles_[idx] = config["robot"]["joints"]["defualt_joint_pos"][item].as<scalar_t>();
            idx++;
        }

        if (config["command_range"]) {
            for (const auto& kv : config["command_range"]) {
                CommandRange cr;
                cr.x_min = kv.second["x"][0].as<double>();
                cr.x_max = kv.second["x"][1].as<double>();
                cr.y_min = kv.second["y"][0].as<double>();
                cr.y_max = kv.second["y"][1].as<double>();
                cr.yaw_min = kv.second["yaw"][0].as<double>();
                cr.yaw_max = kv.second["yaw"][1].as<double>();
                command_ranges_.push_back(cr);
            }
        }

    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        write_log("Failed to load config (controller package)", "ERROR");
        exit(EXIT_FAILURE);
    }
}


void Controller::LoadModel(){
    onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));

    Ort::SessionOptions sessionOptions;
    sessionOptions.SetInterOpNumThreads(1);

    std::vector<std::string> encoder_paths;
    std::vector<std::string> policy_paths;

    encoder_paths.push_back(ENCODER_1_PATH);
    policy_paths.push_back(POLICY_1_PATH);
    encoder_paths.push_back(ENCODER_2_PATH);
    policy_paths.push_back(POLICY_2_PATH);

    num_models_ = std::min(encoder_paths.size(), policy_paths.size());
    if (num_models_ == 0) {
        write_log("No encoder/policy model paths configured", "ERROR");
        exit(EXIT_FAILURE);
    }

    Ort::AllocatorWithDefaultOptions allocator;

    for (int m = 0; m < num_models_; m++) {
        auto encoder_ptr = std::make_unique<Ort::Session>(*onnxEnvPrt_, encoder_paths[m].c_str(), sessionOptions);
        auto policy_ptr = std::make_unique<Ort::Session>(*onnxEnvPrt_, policy_paths[m].c_str(), sessionOptions);

        std::vector<const char *> encInNames, encOutNames;
        std::vector<Ort::AllocatedStringPtr> encInAlloc, encOutAlloc;
        std::vector<std::vector<int64_t>> encInShapes, encOutShapes;

        for (size_t i = 0; i < encoder_ptr->GetInputCount(); i++) {
            auto p = encoder_ptr->GetInputNameAllocated(i, allocator);
            encInAlloc.push_back(std::move(p));
            encInNames.push_back(encInAlloc.back().get());
            encInShapes.push_back(encoder_ptr->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        }
        for (size_t i = 0; i < encoder_ptr->GetOutputCount(); i++) {
            auto p = encoder_ptr->GetOutputNameAllocated(i, allocator);
            encOutAlloc.push_back(std::move(p));
            encOutNames.push_back(encOutAlloc.back().get());
            encOutShapes.push_back(encoder_ptr->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        }

        std::vector<const char *> polInNames, polOutNames;
        std::vector<Ort::AllocatedStringPtr> polInAlloc, polOutAlloc;
        std::vector<std::vector<int64_t>> polInShapes, polOutShapes;

        for (size_t i = 0; i < policy_ptr->GetInputCount(); i++) {
            auto p = policy_ptr->GetInputNameAllocated(i, allocator);
            polInAlloc.push_back(std::move(p));
            polInNames.push_back(polInAlloc.back().get());
            polInShapes.push_back(policy_ptr->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        }
        for (size_t i = 0; i < policy_ptr->GetOutputCount(); i++) {
            auto p = policy_ptr->GetOutputNameAllocated(i, allocator);
            polOutAlloc.push_back(std::move(p));
            polOutNames.push_back(polOutAlloc.back().get());
            polOutShapes.push_back(policy_ptr->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        }

        encoderSessionPtrs_.push_back(std::move(encoder_ptr));
        encoderInputNames_.push_back(std::move(encInNames));
        encoderOutputNames_.push_back(std::move(encOutNames));
        encoderInputNodeNameAllocatedStrings_.push_back(std::move(encInAlloc));
        encoderOutputNodeNameAllocatedStrings_.push_back(std::move(encOutAlloc));
        encoderInputShapes_.push_back(std::move(encInShapes));
        encoderOutputShapes_.push_back(std::move(encOutShapes));

        policySessionPtrs_.push_back(std::move(policy_ptr));
        policyInputNames_.push_back(std::move(polInNames));
        policyOutputNames_.push_back(std::move(polOutNames));
        policyInputNodeNameAllocatedStrings_.push_back(std::move(polInAlloc));
        policyOutputNodeNameAllocatedStrings_.push_back(std::move(polOutAlloc));
        policyInputShapes_.push_back(std::move(polInShapes));
        policyOutputShapes_.push_back(std::move(polOutShapes));
    }

    RCLCPP_INFO(rclcpp::get_logger("controller"), "Loaded %d models", num_models_);
}


void Controller::Publish(){
    if (is_runing.load(std::memory_order_relaxed)){
        this->ComputeHistoryObservation();
        this->ComputeEncoderActions();
        this->ComputePolicyObservation();
        this->ComputeActions();
    }
    std_msgs::msg::Float32MultiArray msg;
    msg.data = actions_;
    publisher_->publish(msg);
}

void Controller::RecCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){

    if (msg->data.size() != static_cast<size_t>(10 + actionsSize_ * 2)) {
        return;
    }
    const double* data_ptr = msg->data.data();

    double raw_axes[4];
    for (int i = 0; i < 4; i++) {
        raw_axes[i] = *data_ptr;
        data_ptr++;
    }

    threadLock_.Lock();
    if (active_model_index_ < static_cast<int>(command_ranges_.size())) {
        const auto& cr = command_ranges_[active_model_index_];
        double x_raw = raw_axes[0];
        double y_raw = raw_axes[1];
        double yaw_stick = raw_axes[2];
        double trigger = raw_axes[3];

        command_.base_vel[0] = x_raw < 0.0 ? x_raw * (-cr.x_min) : x_raw * cr.x_max;
        command_.base_vel[1] = y_raw < 0.0 ? y_raw * (-cr.y_min) : y_raw * cr.y_max;

        if (trigger < 0.0) {
            command_.base_vel[2] = yaw_stick * (cr.yaw_min / 2.0) + (cr.yaw_min / 2.0);
        } else if (trigger > 0.0) {
            command_.base_vel[2] = yaw_stick * (cr.yaw_max / 2.0) + (cr.yaw_max / 2.0);
        } else {
            command_.base_vel[2] = 0.0;
        }
    }

    for(int i = 0; i < 3; i++){
        proprioception_.baseAngVel[i] = *data_ptr;
        data_ptr++;
    }

    for(int i = 0; i < 3; i++){
        proprioception_.projectedGravity[i] = *data_ptr;
        data_ptr++;
    }

    for(int i = 0; i < actionsSize_; i++){
        proprioception_.jointPos[i] = *data_ptr;
        data_ptr++;
    }

    for(int i = 0; i < actionsSize_; i++){
        proprioception_.jointVel[i] = *data_ptr;
        data_ptr++;
    }
    threadLock_.Unlock();
    isfirstRecv_ = false;
}


void Controller::ButtonStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (msg->data == 0) {
        is_runing.store(false, std::memory_order_release);
        RCLCPP_INFO(node_->get_logger(), "PAUSE");
    }
    else if (msg->data == 1) {
        is_runing.store(true, std::memory_order_release);
        RCLCPP_INFO(node_->get_logger(), "GO_ON");
    }
    // TERMINATE=2 and RESET=3 handled by real package
}


void Controller::PolicyIndexCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    static u_int32_t time = 0;
    int idx = msg->data;
    if (idx >= 0 && idx < num_models_ && idx != active_model_index_ && time >= 10) {
        active_model_index_ = idx;
        isfirstRecv_ = true;
        phase = 0;
        std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
        std::fill(proprioHistoryBuffer_.begin(), proprioHistoryBuffer_.end(), 0.0f);
        std::string msg = "Switched to model " + std::to_string(idx);
        RCLCPP_INFO(node_->get_logger(), "Switched to model %d", idx);
        write_log(msg.c_str(), "INFO");
        time = 0;
    }
    time++;
}

/*
***************************************************************************************
***************************************************************************************
*/

void Controller::ComputeHistoryObservation(){

    vector_t proprioObs(observationSize_);
    double phase_norm = std::fmod(phase * (1000.0 / frequency_), cycle_) / cycle_ * 6.283185307;
    proprioception_.sin[0] = std::sin(phase_norm);
    proprioception_.cos[0] = std::cos(phase_norm);
    threadLock_.Lock();
    proprioObs << proprioception_.sin, proprioception_.cos,
        command_.base_vel, // 3
        (proprioception_.baseAngVel * proprioception_.scale.baseAngVel),  // 3
        proprioception_.projectedGravity,  // 3
        ((proprioception_.jointPos - defaultJointAngles_) * proprioception_.scale.jointPos),  // 12
        (proprioception_.jointVel * proprioception_.scale.jointVel),  // 12
        lastActions_;  // 12
    threadLock_.Unlock();
    phase += 1;

    if (isfirstRecv_)
    {
        for (int i = 11; i < observationSize_; i++)
        {
            proprioObs(i,0) = 0.0;
        }

        for (int i = 0; i < stackSize_; i++)
        {
            proprioHistoryBuffer_.segment(i * observationSize_, observationSize_) = proprioObs.cast<tensor_element_t>();
        }
        isfirstRecv_ = false;

        std::fill(historyObservations_.begin(), historyObservations_.end(), 0.0f);
    }
    proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) = proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - observationSize_);
    proprioHistoryBuffer_.tail(observationSize_) = proprioObs.cast<tensor_element_t>();

    Eigen::VectorXf::Map(historyObservations_.data(), input_size_) = proprioHistoryBuffer_.head(input_size_);

    scalar_t obsMin = -proprioception_.clipObs;
    scalar_t obsMax = proprioception_.clipObs;
    std::transform(
        historyObservations_.begin(),
        historyObservations_.end(),
        historyObservations_.begin(),
        [obsMin, obsMax](scalar_t x){ return std::max(obsMin, std::min(obsMax, x)); }
    );
}


void Controller::ComputePolicyObservation(){

    Eigen::VectorXf::Map(policyObservations_.data() + encoderOutputSize_, input_size_) =
    Eigen::VectorXf::Map(historyObservations_.data(), input_size_);

    Eigen::VectorXf::Map(policyObservations_.data(), encoderOutputSize_) =
    Eigen::VectorXf::Map(encoderOutputs_.data(), encoderOutputSize_);
}


void Controller::ComputeEncoderActions(){

    int idx = active_model_index_;
    auto& activeSession = *encoderSessionPtrs_[idx];
    auto& activeInputNames = encoderInputNames_[idx];
    auto& activeOutputNames = encoderOutputNames_[idx];
    auto& activeInputShapes = encoderInputShapes_[idx];

    std::vector<Ort::Value> encoderInputValues;
    encoderInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(
        memoryInfo,
        historyObservations_.data(),
        historyObservations_.size(),
        activeInputShapes[0].data(),
        activeInputShapes[0].size())
    );

    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues;
    outputValues = activeSession.Run(
        runOptions,
        activeInputNames.data(),
        encoderInputValues.data(),
        activeSession.GetInputCount(),
        activeOutputNames.data(),
        activeSession.GetOutputCount()
    );

    auto output_values_ptr = outputValues[0].GetTensorMutableData<tensor_element_t>();
    for (int i = 0; i < encoderOutputSize_; i++)
    {
        encoderOutputs_[i] = *(output_values_ptr + i);
    }

}

void Controller::ComputeActions(){

    int idx = active_model_index_;
    auto& activeSession = *policySessionPtrs_[idx];
    auto& activeInputNames = policyInputNames_[idx];
    auto& activeOutputNames = policyOutputNames_[idx];
    auto& activeInputShapes = policyInputShapes_[idx];

    std::vector<Ort::Value> policyInputValues;
    policyInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(
        memoryInfo,
        policyObservations_.data(),
        policyObservations_.size(),
        activeInputShapes[0].data(),
        activeInputShapes[0].size())
    );

    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues;
    outputValues = activeSession.Run(
        runOptions,
        activeInputNames.data(),
        policyInputValues.data(),
        activeSession.GetInputCount(),
        activeOutputNames.data(),
        activeSession.GetOutputCount()
    );


    auto last_action = outputValues[0].GetTensorMutableData<scalar_t>();
    auto output_values_ptr = outputValues[0].GetTensorMutableData<tensor_element_t>();

        for (int i = 0; i < actionsSize_; i++)
        {
            lastActions_[i] = *(last_action + i);
            actions_[i] = *(output_values_ptr + i) * actionsCfg_.actionScale[i] + defaultJointAngles_[i];
        }
        scalar_t actMin = -actionsCfg_.clipActions;
        scalar_t actMax = actionsCfg_.clipActions;
        std::transform(
            actions_.begin(),
            actions_.end(),
            actions_.begin(),
            [actMin, actMax](scalar_t x){ return std::max(actMin, std::min(actMax, x)); }
        );
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    Controller controller = Controller();
    controller.start();
    return 0;
}
