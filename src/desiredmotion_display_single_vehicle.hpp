/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <tuple>
#include <unordered_map>
#include <boost/circular_buffer.hpp>
#include <geometry_msgs/Pose.h>

#include <rviz/display.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>

#include <string>

#include <automated_driving_msgs/DeltaTrajectory.h>
#include <automated_driving_msgs/MotionState.h>

#include <util_rviz/util_rviz.hpp>

#include "desiredmotion_visual.hpp"

namespace desired_motion_rviz_plugin_ros {

class DesiredMotionVisual;

class DesiredMotionDisplaySingleVehicle : public rviz::Display {
    Q_OBJECT
public:
    DesiredMotionDisplaySingleVehicle();
    virtual ~DesiredMotionDisplaySingleVehicle();

    // Overrides from Display
    virtual void onInitialize();
    virtual void reset();

protected:
    // overrides from Display
    virtual void onEnable();
    virtual void onDisable();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void fixedFrameChanged();

    // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
    void updateTopic();
    void updateTimeInterval();
    void updateMaxTime();
    void updateColorMaxTime();
    void updateObjectProperty();
    void updateRadiusProperty();

    // Function to handle an incoming ROS message.
private:
    // properties
    std::unique_ptr<rviz::FloatProperty> circleTimeIntervalProperty_;
    std::unique_ptr<rviz::FloatProperty> maxFrequencyProperty_;
    std::unique_ptr<rviz::FloatProperty> maxTimeProperty_;
    std::unique_ptr<rviz::FloatProperty> colorMaxTimeProperty_;
    std::unique_ptr<rviz::RosTopicProperty> topicDTProperty_;
    std::unique_ptr<rviz::RosTopicProperty> topicMSProperty_;
    std::unique_ptr<rviz::BoolProperty> plannerDebugModeProperty_;
    std::unique_ptr<rviz::FloatProperty> radiusProperty_;

    void incomingMessageMS(const automated_driving_msgs::MotionState::ConstPtr& msg);
    void incomingMessageDT(const automated_driving_msgs::DeltaTrajectory::ConstPtr& msg);

    void processMessageMS(const automated_driving_msgs::MotionState::ConstPtr& motionState);

    void createVisuals(const automated_driving_msgs::DeltaTrajectory::ConstPtr& deltaTrajectory,
                       const automated_driving_msgs::MotionState::ConstPtr &motionState);
    void reCalcVisual(const automated_driving_msgs::DeltaTrajectory::ConstPtr& deltaTrajectory,
                      const automated_driving_msgs::MotionState::ConstPtr &motionState,
                      const double currentAbsoluteTime,
                      std::shared_ptr<DesiredMotionVisual> visual);


    // Storage for the objectStates, the desiredmotions and the list of visuals.
    std::vector<std::shared_ptr<DesiredMotionVisual>> visuals_;
    automated_driving_msgs::MotionState::ConstPtr motionState_;
    automated_driving_msgs::DeltaTrajectory::ConstPtr deltaTrajectory_;

    // ros subscribers
    ros::Subscriber subMS_;
    ros::Subscriber subDT_;

    // message counter
    uint32_t messagesReceivedMS_ = 0;
    uint32_t messagesReceivedDT_ = 0;

    float timeInterval_;

    uint32_t maxNumberOfVisuals_ = 1;
    float maxTime_;

    ros::WallTime lastUpdate_{0.};
    float maxFrequency_;

    // Max Time for calculating the ColorCode. It is highly recommended to set it to the same value
    // in all instances of
    // the plugin.
    float colorMaxTime_;

    // Const and uniform prefix to make sure that Sender and Receiver Names for the QT-Signals and SLOTs are processed
    // without errors.
    const std::string propertyObjectPrefix = std::string("Object");

    std::string transformErrorMsg_ = "";
    bool transformOk_ = true;
};

} // namespace desired_motion_rviz_plugin_ros
