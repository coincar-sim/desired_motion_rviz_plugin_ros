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

    topicOSAProperty_ = std::make_unique<rviz::RosTopicProperty>(
        "ObjectStateArray (OSA) - Topic",
        "",
        QString::fromStdString(ros::message_traits::datatype<automated_driving_msgs::ObjectStateArray>()),
        "Topic where the object states are published.",
        this,
        SLOT(updateTopic()));

    topicDTProperty_ = std::make_unique<rviz::RosTopicProperty>(
        "DeltaTrajectoryWithID (DT) - Topic",
        "",
        QString::fromStdString(ros::message_traits::datatype<simulation_only_msgs::DeltaTrajectoryWithID>()),
        "Topic where the desired motions are published.",
        this,
        SLOT(updateTopic()));

    circleTimeIntervalProperty_ = std::make_unique<rviz::FloatProperty>(
        "Circle Spacing", 1.0, "The spacing between circles. Interval in seconds", this, SLOT(updateTimeInterval()));

    maxTimeProperty_ = std::make_unique<rviz::FloatProperty>(
        "Maximum Planning Horizon",
        10.0,
        "The furthest desired motion (refering to time) that is possibly displayed. [seconds]",
        this,
        SLOT(updateMaxTime()));

    maxFrequencyProperty_ = std::make_unique<rviz::FloatProperty>(
        "Maximum Frequency",
        20.0,
        "Maximum update frequency for delta-trajectory messages. [walltime; Hz; min. 1 Hz]",
        this,
        SLOT(updateMaxTime()));

    colorMaxTimeProperty_ = std::make_unique<rviz::FloatProperty>(
        "ColorCode Planning Horizon",
        10.0,
        "Defines the spreading of the ColorCode."
        "A reasonable choice is to set the same value as for Maximum Planning Horizon. "
        "It is highly recommended (unless you have good reasons not to do so) to set it to the "
        "same value "
        "in all instances of the plugin. [seconds]",
        this,
        SLOT(updateColorMaxTime()));

    objectAllProperty_ =
        std::make_unique<rviz::BoolProperty>("Display All Objects",
                                             false,
                                             "If selected the desired motions of all objects are "
                                             "displayed. If deselected you can choose for all objects individually.",
                                             this,
                                             SLOT(updateObjectProperty()));

    radiiProperty_ = std::make_unique<rviz::Property>(
        "Radii of Circles",
        QVariant(),
        "Set The Radii of the displayed Circles. For each object you can set the radius individually. Default = 1.0m",
        this);

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
    deltaTrajectorys_.clear();
    objectPropertys_.clear();

    messagesReceivedOSA_ = 0;
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
    subOSA_.shutdown();
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

void DesiredMotionDisplaySingleVehicle::updateObjectProperty() {
    if (objectAllProperty_->getBool() == false) {
        // if not all objects should be shown
        for (auto&& it : objectPropertys_) {

            const object_id_type& obj_id = it.first;

            if (deltaTrajectorys_.find(obj_id) != deltaTrajectorys_.end() && objectInObjectStates(obj_id)) {
                // if object could be shown, show its bool property
                it.second->setHidden(false);
            } else {
                // otherwize hide the latter and set it to false
                it.second->setBool(false);
                it.second->setHidden(true);
            }

            if (it.second->getBool()) {
                // if it should be shown, show its radius property
                radiusPropertys_[obj_id]->setHidden(false);
            } else {
                // otherwize hide the latter
                radiusPropertys_[obj_id]->setHidden(true);
            }
        }
    } else {
        // if all objects should be shown
        for (auto&& it : objectPropertys_) {

            const object_id_type& obj_id = it.first;

            // hide all single booleans
            it.second->setHidden(true);

            // display all radii of objects that could be displayed
            if (deltaTrajectorys_.find(obj_id) != deltaTrajectorys_.end() && objectInObjectStates(obj_id)) {
                radiusPropertys_[obj_id]->setHidden(false);
            } else {
                radiusPropertys_[obj_id]->setHidden(true);
            }
        }
    }
}


void DesiredMotionDisplaySingleVehicle::updateRadiusProperty() {
    bool allOk = true;

    if (sender()) {
        std::string senderName = sender()->objectName().toStdString();
        try {
            object_id_type id = std::stoi(senderName.substr(propertyObjectPrefix.length()));

            for (auto&& visual : visuals_[id]) {
                visual->setRadius(radiusPropertys_[id]->getFloat());
            }
        } catch (const std::invalid_argument& e) {
            allOk = false;
            setStatus(rviz::StatusProperty::Error, "Radius", QString::fromStdString(e.what()));
        }

    } else {
        allOk = false;
        setStatus(rviz::StatusProperty::Error,
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
    if (!topicOSAProperty_->getTopic().isEmpty()) {
        try {
            ROS_DEBUG("Subscribing to %s", topicOSAProperty_->getTopicStd().c_str());
            subOSA_ = update_nh_.subscribe(
                topicOSAProperty_->getTopicStd(), 1, &DesiredMotionDisplaySingleVehicle::incomingMessageOSA, this);
            setStatus(rviz::StatusProperty::Ok, "OSA-Topic", "Subscribed, no message received");
        } catch (ros::Exception& e) {
            setStatus(rviz::StatusProperty::Error, "OSA-Topic", QString("Error subscribing: ") + e.what());
        }
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

void DesiredMotionDisplaySingleVehicle::incomingMessageOSA(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    ++messagesReceivedOSA_;
    setStatus(rviz::StatusProperty::Ok, "OSA-Topic", QString::number(messagesReceivedOSA_) + " messages received");
    objectStates_ = msg;
    processMessageOSA(msg);
}

void DesiredMotionDisplaySingleVehicle::incomingMessageDT(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    ++messagesReceivedDT_;
    setStatus(rviz::StatusProperty::Ok, "DT-Topic", QString::number(messagesReceivedDT_) + " messages received");
    deltaTrajectorys_[msg->object_id] = msg;
    if (plannerDebugModeProperty_->getBool()) {
        if (objectStates_) {
            processMessageOSA(objectStates_);
        } else {
            ROS_WARN_THROTTLE(1,
                              "desired_motion_rviz_plugin_ros: Received delta trajectory but not object state array "
                              "yet, not displaying anything!");
        }
    }
}

// Processes incoming ObjectStateArray message.
void DesiredMotionDisplaySingleVehicle::processMessageOSA(
    const automated_driving_msgs::ObjectStateArray::ConstPtr& objectStateArrayMsg) {

    if ((ros::WallTime::now() - lastUpdate_).toSec() < (1 / maxFrequency_)) {
        return;
    }

    lastUpdate_ = ros::WallTime::now();

    transformOk_ = true;
    transformErrorMsg_ = "";
    size_t numberOfUpdatedVisuals = 0;

    // process all objects in the ObjectStateArray
    for (auto&& objectState : objectStateArrayMsg->objects) {
        if (deltaTrajectorys_.find(objectState.object_id) != deltaTrajectorys_.end()) {
            // if a delta trajectory exists for this object

            if (objectPropertys_.count(objectState.object_id) == 0) {
                // create new Bool property to choose if object is displayed if not yet there
                // TODO detect if object is no longer there (maybe wait some time?) and delete property
                createObjectProperty(objectState.object_id);
            }

            if (objectAllProperty_->getBool() || objectPropertys_[objectState.object_id]->getBool()) {
                // if object shall be visualized
                bool foundAndUnique;
                createVisuals(deltaTrajectorys_[objectState.object_id],
                              util_automated_driving_msgs::conversions::objectStateFromObjectStateArray(
                                  objectStates_, objectState.object_id, foundAndUnique));
                assert(foundAndUnique);
                numberOfUpdatedVisuals++;
            } else {
                // else delete its visual
                visuals_[objectState.object_id].clear();
                visuals_.erase(objectState.object_id);
            }
        }
    }

    // delete visuals and properties of objects that are no longer present
    if (numberOfUpdatedVisuals != visuals_.size()) {

        std::vector<int> idsToDelete;
        for (auto it : visuals_) {
            int id = it.first;
            if (!objectInObjectStates(id)) {
                idsToDelete.push_back(id);
            }
        }
        for (int id : idsToDelete) {
            visuals_[id].clear();
            visuals_.erase(id);
        }
        updateObjectProperty();
    }

    // set status of TF transformations
    if (transformOk_) {
        setStatus(rviz::StatusProperty::Ok, "Frame", "No TF error");
    } else {
        setStatus(rviz::StatusProperty::Error, "Frame", QString::fromStdString(transformErrorMsg_));
    }
}


void DesiredMotionDisplaySingleVehicle::createVisuals(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& deltaTrajectory,
                                         const automated_driving_msgs::ObjectState& objectState) {

    // get the maximum available Delta Time (important if less than maxTime_)
    if (deltaTrajectory->delta_poses_with_delta_time.empty()) {
        ROS_WARN("desired_motion_rviz_plugin_ros: Received empty deltaTrajectory for ID %i, discarding!",
                 deltaTrajectory->object_id);
        return;
    }
    const double maxDeltaTime = deltaTrajectory->delta_poses_with_delta_time.back().delta_time.toSec();

    // Recalculate and Reset number of visual Elements
    const uint32_t numberOfVisuals = ((maxDeltaTime < maxTime_) ? (maxDeltaTime / timeInterval_) : maxNumberOfVisuals_);
    if (visuals_[deltaTrajectory->object_id].size() != numberOfVisuals) {
        visuals_[deltaTrajectory->object_id].clear();
        visuals_[deltaTrajectory->object_id].reserve(numberOfVisuals);
        for (size_t i = 0; i < numberOfVisuals; i++) {
            visuals_[deltaTrajectory->object_id].emplace_back(std::make_shared<DesiredMotionVisual>(
                context_->getSceneManager(), scene_node_, radiusPropertys_[deltaTrajectory->object_id]->getFloat()));
        }
    }

    // calculate and set Position and Color for Visuals for corresponding vehicle
    // currentDeltaTime is increased depending on the time interval selected by the user
    // circles are displayed at fixed absolute times for better visualization
    double absoluteTimeNow, firstDeltaDisplayTime;
    absoluteTimeNow = ros::Time::now().toSec();
    if (plannerDebugModeProperty_->getBool()) {
        // set time now to latest object state timestamp
        absoluteTimeNow = objectState.header.stamp.toSec();
        bool trajectoryCompletelyInPast =
            (absoluteTimeNow - (deltaTrajectory->header.stamp.toSec() + maxDeltaTime) > 0);
        if (trajectoryCompletelyInPast) {
            ROS_WARN("desired_motion_rviz_plugin_ros: Not displaying trajectory of object %i though in debug mode "
                     "because delta trajectory lies completely in past. You might want to stop the time.",
                     deltaTrajectory->object_id);
        }
    }
    firstDeltaDisplayTime = -fmod(absoluteTimeNow, timeInterval_) + timeInterval_;
    for (size_t i = 0; (i * timeInterval_ <= maxTime_ && i * timeInterval_ <= maxDeltaTime &&
                        i < visuals_[deltaTrajectory->object_id].size());
         i++) {
        // createVisual(deltaTrajectory, objectState, objectState.object_id, absoluteTimeNow + currentDeltaTime);
        reCalcVisual(deltaTrajectory,
                     objectState,
                     objectState.object_id,
                     absoluteTimeNow + firstDeltaDisplayTime + i * timeInterval_,
                     visuals_[deltaTrajectory->object_id][i]);
    }
}

void DesiredMotionDisplaySingleVehicle::reCalcVisual(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& deltaTrajectory,
                                        const automated_driving_msgs::ObjectState& objectState,
                                        object_id_type objectId,
                                        const double currentAbsoluteTime,
                                        std::shared_ptr<DesiredMotionVisual> visual) {

    // Account for difference of objectStateMsg and desiredMotionMsg
    double startDt = objectState.header.stamp.toSec() - deltaTrajectory->header.stamp.toSec();
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

    // Interpolate Start Pose
    geometry_msgs::Pose newDeltaPoseStart;
    bool valid;
    std::string errMsg;
    util_simulation_only_msgs::interpolateDeltaPose(
        deltaTrajectorys_[objectId], objectState.header.stamp, newDeltaPoseStart, valid, errMsg);
    if (!valid) {
        transformOk_ = false;
        transformErrorMsg_ = "Latest Error: Obj:" + std::to_string(objectState.object_id) + ": " + errMsg;
        visual->setVisible(false);
        return;
    }

    // Interpolate Current Pose
    geometry_msgs::Pose newDeltaPose;
    ros::Time currentAbsoluteTimestamp;
    currentAbsoluteTimestamp.fromSec(currentAbsoluteTime);
    util_simulation_only_msgs::interpolateDeltaPose(
        deltaTrajectorys_[objectId], currentAbsoluteTimestamp, newDeltaPose, valid, errMsg);
    if (!valid) {
        transformOk_ = false;
        transformErrorMsg_ = "Latest Error: Obj:" + std::to_string(objectState.object_id) + ": " + errMsg;
        visual->setVisible(false);
        return;
    }

    // Calculate start Pose of delta trajectory
    geometry_msgs::Pose startPose =
        util_geometry_msgs::computations::subtractDeltaPose(objectState.motion_state.pose.pose, newDeltaPoseStart);

    // Calculate final Pose
    geometry_msgs::Pose finalPose = util_geometry_msgs::computations::addDeltaPose(startPose, newDeltaPose);

    // Transform the pose into the fixed_frame
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->transform(objectState.motion_state.header.frame_id,
                                                ros::Time(0), // using latest information
                                                finalPose,
                                                position,
                                                orientation)) {

        std::string error;
        context_->getFrameManager()->transformHasProblems(
            objectState.motion_state.header.frame_id, ros::Time(0), error);

        transformOk_ = false;
        transformErrorMsg_ = "Latest Error: Obj:" + std::to_string(objectState.object_id) + ": " + error;
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

void DesiredMotionDisplaySingleVehicle::createObjectProperty(object_id_type objectId) {
    // temporary std::strings for property name and description creation
    const std::string pName = propertyObjectPrefix + boost::lexical_cast<std::string>(objectId);
    const std::string pDescription = std::string("Select to display the desired motion of Object") +
                                     boost::lexical_cast<std::string>(objectId) +
                                     "Only Objects received on the set OSA-Topic are selectable.";

    /* Make sure to use an unique and reasonable naming here (for rName).
     * Since std::stoi() is used to extract the objectId from the QObjectsName
     * make sure to have the objectID in the beginning of the name
     * It is used later to identify the sender in the SLOT.
     */
    const std::string rName = propertyObjectPrefix + boost::lexical_cast<std::string>(objectId);
    const std::string rDescription =
        std::string("The Circle Radius for Object ") + boost::lexical_cast<std::string>(objectId);
    // create new property
    objectPropertys_[objectId] = std::make_unique<rviz::BoolProperty>(QString::fromStdString(pName),
                                                                      false,
                                                                      QString::fromStdString(pDescription),
                                                                      objectAllProperty_.get(),
                                                                      SLOT(updateObjectProperty()),
                                                                      this);
    radiusPropertys_[objectId] = std::make_unique<rviz::FloatProperty>(QString::fromStdString(rName),
                                                                       1.0,
                                                                       QString::fromStdString(rDescription),
                                                                       radiiProperty_.get(),
                                                                       SLOT(updateRadiusProperty()),
                                                                       this);
    // The data is not worth saving, because its existence is highly dependend on the received
    // messages.
    objectPropertys_[objectId]->setShouldBeSaved(false);
    radiusPropertys_[objectId]->setShouldBeSaved(false);
}

bool DesiredMotionDisplaySingleVehicle::objectInObjectStates(object_id_type objectId) {
    // todo: move to utils
    for (auto&& objectState : objectStates_->objects) {
        if (objectState.object_id == objectId)
            return true;
    }
    return false;
}

} // namespace desired_motion_rviz_plugin_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(desired_motion_rviz_plugin_ros::DesiredMotionDisplaySingleVehicle, rviz::Display)
