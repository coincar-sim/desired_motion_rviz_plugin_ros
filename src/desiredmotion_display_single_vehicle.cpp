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

#include <assert.h>
#include <util_automated_driving_msgs/util_automated_driving_msgs.hpp>
#include <util_geometry_msgs/util_geometry_msgs.hpp>
#include <util_simulation_only_msgs/util_simulation_only_msgs.hpp>

#include "desiredmotion_display_single_vehicle.hpp"

namespace desired_motion_rviz_plugin_ros {

DesiredMotionDisplaySingleVehicle::DesiredMotionDisplaySingleVehicle() {

    topicMSProperty_ = std::make_unique<rviz::RosTopicProperty>(
        "MotionState(MS) - Topic",
        "",
        QString::fromStdString(ros::message_traits::datatype<automated_driving_msgs::MotionState>()),
        "Topic where the motion state are published",
        this,
        SLOT(updateTopic()));

    topicDTProperty_ = std::make_unique<rviz::RosTopicProperty>(
        "DeltaTrajectory(DT) - Topic",
        "",
        QString::fromStdString(ros::message_traits::datatype<automated_driving_msgs::DeltaTrajectory>()),
        "Topic where the desired motion is published",
        this,
        SLOT(updateTopic()));

    circleTimeIntervalProperty_ = std::make_unique<rviz::FloatProperty>(
        "Circle Spacing", 0.3, "The spacing between circles. Interval in seconds", this, SLOT(updateTimeInterval()));

    maxTimeProperty_ = std::make_unique<rviz::FloatProperty>(
        "Maximum Planning Horizon",
        10.0,
        "The furthest desired motion (refering to time) that is possibly displayed [seconds]",
        this,
        SLOT(updateMaxTime()));

    maxFrequencyProperty_ = std::make_unique<rviz::FloatProperty>(
        "Maximum Frequency",
        20.0,
        "Maximum update frequency for delta-trajectory messages [walltime; Hz; min. 1 Hz]",
        this,
        SLOT(updateMaxTime()));

    colorMaxTimeProperty_ = std::make_unique<rviz::FloatProperty>(
        "ColorCode Planning Horizon",
        10.0,
        "Defines the spreading of the ColorCode."
        "A reasonable choice is to set the same value as for Maximum Planning Horizon. "
        "It is highly recommended (unless you have good reasons not to do so) to set it to the "
        "same value "
        "in all instances of the plugin [seconds]",
        this,
        SLOT(updateColorMaxTime()));

    radiusProperty_ = std::make_unique<rviz::FloatProperty>(
        "Radius", 1.0, "Set the radius of the displayed circles [m]", this, SLOT(updateRadiusProperty()));

    plannerDebugModeProperty_ =
        std::make_unique<rviz::BoolProperty>("PlannerDebugMode(!)",
                                             false,
                                             "If selected, the desired motion is always visualized at the latest "
                                             "object state, regardless of time differences (use with caution!)",
                                             this);
}


DesiredMotionDisplaySingleVehicle::~DesiredMotionDisplaySingleVehicle() {
    unsubscribe();
}

void DesiredMotionDisplaySingleVehicle::onInitialize() {
    updateTimeInterval();
}

void DesiredMotionDisplaySingleVehicle::reset() {
    Display::reset();

    visuals_.clear();
    deltaTrajectory_.reset();
    motionState_.reset();

    messagesReceivedMS_ = 0;
    messagesReceivedDT_ = 0;
}

void DesiredMotionDisplaySingleVehicle::onEnable() {
    updateTimeInterval();
    updateMaxTime();
    updateColorMaxTime();

    subscribe();
}

void DesiredMotionDisplaySingleVehicle::onDisable() {
    unsubscribe();
    reset();
}

void DesiredMotionDisplaySingleVehicle::unsubscribe() {
    subMS_.shutdown();
    subDT_.shutdown();
}

void DesiredMotionDisplaySingleVehicle::updateTopic() {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
}

void DesiredMotionDisplaySingleVehicle::fixedFrameChanged() {
    reset();
}


void DesiredMotionDisplaySingleVehicle::updateRadiusProperty() {
    bool allOk = true;

    if (sender()) {
        std::string senderName = sender()->objectName().toStdString();
        try {
            for (auto&& visual : visuals_) {
                visual->setRadius(radiusProperty_->getFloat());
            }
        } catch (const std::invalid_argument& e) {
            allOk = false;
            setStatus(rviz::StatusProperty::Error, "Radius", QString::fromStdString(e.what()));
        }

    } else {
        allOk = false;
        setStatus(
            rviz::StatusProperty::Error,
            "Radius",
            "DesiredMotionDisplaySingleVehicle::updateRadiusProperty(): Failed to receive objectId from QTSender.");
    }

    if (allOk)
        setStatus(rviz::StatusProperty::Ok, "Radius", "Radius settings ok");
}


void DesiredMotionDisplaySingleVehicle::subscribe() {
    if (!isEnabled()) {
        return;
    }
    if (!topicMSProperty_->getTopic().isEmpty()) {
        try {
            ROS_DEBUG("Subscribing to %s", topicMSProperty_->getTopicStd().c_str());
            subMS_ = update_nh_.subscribe(
                topicMSProperty_->getTopicStd(), 1, &DesiredMotionDisplaySingleVehicle::incomingMessageMS, this);
            setStatus(rviz::StatusProperty::Ok, "MS-Topic", "Subscribed, no message received");
        } catch (ros::Exception& e) {
            setStatus(rviz::StatusProperty::Error, "MS-Topic", QString("Error subscribing: ") + e.what());
        }
    } else {
        setStatus(rviz::StatusProperty::Warn, "MS-Topic", QString("Not set"));
    }
    if (!topicDTProperty_->getTopic().isEmpty()) {
        try {
            ROS_DEBUG("Subscribing to %s", topicDTProperty_->getTopicStd().c_str());
            subDT_ = update_nh_.subscribe(
                topicDTProperty_->getTopicStd(), 1, &DesiredMotionDisplaySingleVehicle::incomingMessageDT, this);
            setStatus(rviz::StatusProperty::Ok, "DT-Topic", "Subscribed, no message received");
        } catch (ros::Exception& e) {
            setStatus(rviz::StatusProperty::Error, "DT-Topic", QString("Error subscribing: ") + e.what());
        }
    } else {
        setStatus(rviz::StatusProperty::Warn, "DT-Topic", QString("Not set"));
    }
}

// Set the current time interval between circles.
void DesiredMotionDisplaySingleVehicle::updateTimeInterval() {
    timeInterval_ = circleTimeIntervalProperty_->getFloat();
    maxNumberOfVisuals_ = maxTime_ / timeInterval_;
}

// Set the furthest time.
void DesiredMotionDisplaySingleVehicle::updateMaxTime() {
    maxTime_ = maxTimeProperty_->getFloat();
    maxNumberOfVisuals_ = maxTime_ / timeInterval_;

    maxFrequency_ = maxFrequencyProperty_->getFloat();
    if (maxFrequency_ < 1.) {
        maxFrequency_ = 1.;
        maxFrequencyProperty_->setFloat(1.);
    }
}

void DesiredMotionDisplaySingleVehicle::updateColorMaxTime() {
    colorMaxTime_ = colorMaxTimeProperty_->getFloat();
}

void DesiredMotionDisplaySingleVehicle::incomingMessageMS(const automated_driving_msgs::MotionState::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    ++messagesReceivedMS_;
    setStatus(rviz::StatusProperty::Ok, "MS-Topic", QString::number(messagesReceivedMS_) + " messages received");
    motionState_ = msg;
    processMessageMS(msg);
}

void DesiredMotionDisplaySingleVehicle::incomingMessageDT(
    const automated_driving_msgs::DeltaTrajectory::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    ++messagesReceivedDT_;
    setStatus(rviz::StatusProperty::Ok, "DT-Topic", QString::number(messagesReceivedDT_) + " messages received");
    deltaTrajectory_ = msg;
    if (plannerDebugModeProperty_->getBool()) {
        if (motionState_) {
            processMessageMS(motionState_);
        } else {
            ROS_WARN_THROTTLE(1,
                              "desired_motion_rviz_plugin_ros: Received delta trajectory but not object state array "
                              "yet, not displaying anything!");
        }
    }
}

// Processes incoming ObjectStateArray message.
void DesiredMotionDisplaySingleVehicle::processMessageMS(
    const automated_driving_msgs::MotionState::ConstPtr& motionStateMsg) {

    if ((ros::WallTime::now() - lastUpdate_).toSec() < (1 / maxFrequency_)) {
        return;
    }

    if (!deltaTrajectory_) {
        return;
    }

    lastUpdate_ = ros::WallTime::now();

    transformOk_ = true;
    transformErrorMsg_ = "";

    createVisuals(deltaTrajectory_, motionStateMsg);


    // set status of TF transformations
    if (transformOk_) {
        setStatus(rviz::StatusProperty::Ok, "Frame", "No TF error");
    } else {
        setStatus(rviz::StatusProperty::Error, "Frame", QString::fromStdString(transformErrorMsg_));
    }
}


void DesiredMotionDisplaySingleVehicle::createVisuals(
    const automated_driving_msgs::DeltaTrajectory::ConstPtr& deltaTrajectory,
    const automated_driving_msgs::MotionState::ConstPtr& motionState) {

    // get the maximum available Delta Time (important if less than maxTime_)
    if (deltaTrajectory->delta_poses_with_delta_time.empty()) {
        ROS_WARN("desired_motion_rviz_plugin_ros: Received empty deltaTrajectory, discarding!");
        return;
    }
    const double maxDeltaTime = deltaTrajectory->delta_poses_with_delta_time.back().delta_time.toSec();

    // Recalculate and Reset number of visual Elements
    const uint32_t numberOfVisuals = ((maxDeltaTime < maxTime_) ? (maxDeltaTime / timeInterval_) : maxNumberOfVisuals_);
    if (visuals_.size() != numberOfVisuals) {
        visuals_.clear();
        visuals_.reserve(numberOfVisuals);
        for (size_t i = 0; i < numberOfVisuals; i++) {
            visuals_.emplace_back(std::make_shared<DesiredMotionVisual>(
                context_->getSceneManager(), scene_node_, radiusProperty_->getFloat()));
        }
    }

    // calculate and set Position and Color for Visuals for corresponding vehicle
    // currentDeltaTime is increased depending on the time interval selected by the user
    // circles are displayed at fixed absolute times for better visualization
    double absoluteTimeNow, firstDeltaDisplayTime;
    absoluteTimeNow = ros::Time::now().toSec();
    if (plannerDebugModeProperty_->getBool()) {
        // set time now to latest object state timestamp
        absoluteTimeNow = motionState->header.stamp.toSec();
        bool trajectoryCompletelyInPast =
            (absoluteTimeNow - (deltaTrajectory->header.stamp.toSec() + maxDeltaTime) > 0);
        if (trajectoryCompletelyInPast) {
            ROS_WARN("desired_motion_rviz_plugin_ros: Not displaying trajectory though in debug mode "
                     "because delta trajectory lies completely in past. You might want to stop the time.");
        }
    }
    firstDeltaDisplayTime = -fmod(absoluteTimeNow, timeInterval_) + timeInterval_;
    for (size_t i = 0; (i * timeInterval_ <= maxTime_ && i * timeInterval_ <= maxDeltaTime && i < visuals_.size());
         i++) {
        // createVisual(deltaTrajectory, objectState, objectState.object_id, absoluteTimeNow + currentDeltaTime);
        reCalcVisual(
            deltaTrajectory, motionState, absoluteTimeNow + firstDeltaDisplayTime + i * timeInterval_, visuals_[i]);
    }
}

void DesiredMotionDisplaySingleVehicle::reCalcVisual(
    const automated_driving_msgs::DeltaTrajectory::ConstPtr& deltaTrajectory,
    const automated_driving_msgs::MotionState::ConstPtr& motionState,
    const double currentAbsoluteTime,
    std::shared_ptr<DesiredMotionVisual> visual) {

    // Account for difference of objectStateMsg and desiredMotionMsg
    double startDt = motionState->header.stamp.toSec() - deltaTrajectory->header.stamp.toSec();
    if (startDt < deltaTrajectory->delta_poses_with_delta_time[0].delta_time.toSec() ||
        startDt > deltaTrajectory->delta_poses_with_delta_time.back().delta_time.toSec()) {
        if (plannerDebugModeProperty_->getBool()) {
            // set start time (time in deltaTrajectory list when object state arrived) to zero, if object state is older
            // than delta trajectory (otherwize it cannot be displayed)
            startDt = 0.0;
        } else {
            return;
        }
    }
    double currentDt = currentAbsoluteTime - deltaTrajectory->header.stamp.toSec();
    if (currentDt < deltaTrajectory->delta_poses_with_delta_time[0].delta_time.toSec() ||
        currentDt > deltaTrajectory->delta_poses_with_delta_time.back().delta_time.toSec()) {
        return;
    }

    simulation_only_msgs::DeltaTrajectoryWithID dtwid;
    dtwid.header = deltaTrajectory->header;
    dtwid.delta_poses_with_delta_time = deltaTrajectory->delta_poses_with_delta_time;

    // Interpolate Start Pose
    geometry_msgs::Pose newDeltaPoseStart;
    bool valid;
    std::string errMsg;
    util_simulation_only_msgs::interpolateDeltaPose(dtwid, motionState->header.stamp, newDeltaPoseStart, valid, errMsg);
    if (!valid) {
        transformOk_ = false;
        transformErrorMsg_ = "Latest Error: " + errMsg;
        visual->setVisible(false);
        return;
    }

    // Interpolate Current Pose
    geometry_msgs::Pose newDeltaPose;
    ros::Time currentAbsoluteTimestamp;
    currentAbsoluteTimestamp.fromSec(currentAbsoluteTime);
    util_simulation_only_msgs::interpolateDeltaPose(dtwid, currentAbsoluteTimestamp, newDeltaPose, valid, errMsg);
    if (!valid) {
        transformOk_ = false;
        transformErrorMsg_ = "Latest Error: " + errMsg;
        visual->setVisible(false);
        return;
    }

    // Calculate start Pose of delta trajectory
    geometry_msgs::Pose startPose =
        util_geometry_msgs::computations::subtractDeltaPose(motionState->pose.pose, newDeltaPoseStart);

    // Calculate final Pose
    geometry_msgs::Pose finalPose = util_geometry_msgs::computations::addDeltaPose(startPose, newDeltaPose);

    // Transform the pose into the fixed_frame
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->transform(motionState->header.frame_id,
                                                ros::Time(0), // using latest information
                                                finalPose,
                                                position,
                                                orientation)) {

        std::string error;
        context_->getFrameManager()->transformHasProblems(motionState->header.frame_id, ros::Time(0), error);

        transformOk_ = false;
        transformErrorMsg_ = "Latest Error: " + error;
        visual->setVisible(false);
        return;
    }

    // Now set or update the contents of the visual.
    visual->setVisible(true);
    util_rviz::setPositionSafely(visual, position);
    util_rviz::setOrientationSafely(visual, orientation);

    // calculate and set color
    double colorDeltaTime, hue;
    colorDeltaTime = fmod(currentAbsoluteTime, colorMaxTime_);
    hue = colorDeltaTime / colorMaxTime_;
    visual->setColor(hue, 1, 1, 1);
}


} // namespace desired_motion_rviz_plugin_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(desired_motion_rviz_plugin_ros::DesiredMotionDisplaySingleVehicle, rviz::Display)
