#include "desiredmotion_display.hpp"

namespace desired_motion_rviz_plugin_ros {

DesiredMotionDisplay::DesiredMotionDisplay() {

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
}


DesiredMotionDisplay::~DesiredMotionDisplay() {
    unsubscribe();
}

void DesiredMotionDisplay::onInitialize() {
}

void DesiredMotionDisplay::reset() {
    Display::reset();

    visuals_.clear();
    deltaTrajectorys_.clear();
    objectPropertys_.clear();

    messagesReceivedOSA_ = 0;
    messagesReceivedDT_ = 0;
}

void DesiredMotionDisplay::onEnable() {
    updateTimeInterval();
    updateMaxTime();
    updateColorMaxTime();

    subscribe();
}

void DesiredMotionDisplay::onDisable() {
    unsubscribe();
    reset();
}

void DesiredMotionDisplay::unsubscribe() {
    subOSA_.shutdown();
    subDT_.shutdown();
}

void DesiredMotionDisplay::updateTopic() {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
}

void DesiredMotionDisplay::fixedFrameChanged() {
    reset();
}

void DesiredMotionDisplay::updateObjectProperty() {
    if (objectAllProperty_->getBool() == false) {
        for (auto&& it : objectPropertys_) {
            if (!it.second->getBool()) {
                visuals_[it.first].clear();
            }
            it.second->setHidden(false);
        }
    } else {
        for (auto&& it : objectPropertys_) {
            it.second->setHidden(true);
        }
    }
}


void DesiredMotionDisplay::updateRadiusProperty() {
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
                  "DesiredMotionDisplay::updateRadiusProperty(): Failed to receive objectId from QTSender.");
    }

    if (allOk)
        setStatus(rviz::StatusProperty::Ok, "Radius", "Radius settings ok");
}


void DesiredMotionDisplay::subscribe() {
    if (!isEnabled()) {
        return;
    }
    if (!topicOSAProperty_->getTopic().isEmpty()) {
        try {
            ROS_DEBUG("Subscribing to %s", topicOSAProperty_->getTopicStd().c_str());
            subOSA_ = update_nh_.subscribe(
                topicOSAProperty_->getTopicStd(), 1, &DesiredMotionDisplay::incomingMessageOSA, this);
            setStatus(rviz::StatusProperty::Ok, "OSA-Topic", "Subscribed, no message received");
        } catch (ros::Exception& e) {
            setStatus(rviz::StatusProperty::Error, "OSA-Topic", QString("Error subscribing: ") + e.what());
        }
    }
    if (!topicDTProperty_->getTopic().isEmpty()) {
        try {
            ROS_DEBUG("Subscribing to %s", topicDTProperty_->getTopicStd().c_str());
            subDT_ = update_nh_.subscribe(
                topicDTProperty_->getTopicStd(), 1, &DesiredMotionDisplay::incomingMessageDT, this);
            setStatus(rviz::StatusProperty::Ok, "DT-Topic", "Subscribed, no message received");
        } catch (ros::Exception& e) {
            setStatus(rviz::StatusProperty::Error, "DT-Topic", QString("Error subscribing: ") + e.what());
        }
    }
}

// Set the current time interval between circles.
void DesiredMotionDisplay::updateTimeInterval() {
    timeInterval_ = circleTimeIntervalProperty_->getFloat();
    maxNumberOfVisuals_ = maxTime_ / timeInterval_;
}

// Set the furthest time.
void DesiredMotionDisplay::updateMaxTime() {
    maxTime_ = maxTimeProperty_->getFloat();
    maxNumberOfVisuals_ = maxTime_ / timeInterval_;
}

void DesiredMotionDisplay::updateColorMaxTime() {
    colorMaxTime_ = colorMaxTimeProperty_->getFloat();
}

void DesiredMotionDisplay::incomingMessageOSA(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    ++messagesReceivedOSA_;
    setStatus(rviz::StatusProperty::Ok, "OSA-Topic", QString::number(messagesReceivedOSA_) + " messages received");
    objectStates_ = msg;
    processMessageOSA(msg);
}

void DesiredMotionDisplay::incomingMessageDT(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    ++messagesReceivedDT_;
    setStatus(rviz::StatusProperty::Ok, "DT-Topic", QString::number(messagesReceivedDT_) + " messages received");
    deltaTrajectorys_[msg->object_id] = msg;
}

// Processes incoming ObjectStateArray message.
void DesiredMotionDisplay::processMessageOSA(
    const automated_driving_msgs::ObjectStateArray::ConstPtr& objectStateArrayMsg) {

    transformOk_ = true;
    transformErrorMsg_ = "";

    // process all objects in the ObjectStateArray
    for (auto&& objectState : objectStateArrayMsg->objects) {
        // create new Bool property to choose if object is displayed
        // TODO detect if object is no longer there (maybe wait some time?) and delete property
        if (objectPropertys_.count(objectState.object_id) == 0) {
            createObjectProperty(objectState.object_id);
        }

        if (objectAllProperty_->getBool() || objectPropertys_[objectState.object_id]->getBool()) {
            // if object shall be visualized

            if (deltaTrajectorys_.find(objectState.object_id) != deltaTrajectorys_.end()) {
                // if a delta trajectory exists for this object
                createVisuals(deltaTrajectorys_[objectState.object_id],
                              util_perception::ObjectStateFromObjectStateArray(objectStates_, objectState.object_id));
            }
        }
    }

    // set status of TF transformations
    if (transformOk_) {
        setStatus(rviz::StatusProperty::Ok, "Frame", "No TF error");
    } else {
        setStatus(rviz::StatusProperty::Error, "Frame", QString::fromStdString(transformErrorMsg_));
    }
}


void DesiredMotionDisplay::createVisuals(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& deltaTrajectory,
                                         const automated_driving_msgs::ObjectState& objectState) {

    // get the maximum available Delta Time (important if less than maxTime_)
    const uint32_t maxDeltaTime = deltaTrajectory->delta_poses_with_delta_time.back().delta_time.toSec();

    // Recalculate and Reset number of visual Elements
    const uint32_t numberOfVisuals = ((maxDeltaTime < maxTime_) ? (maxDeltaTime / timeInterval_) : maxNumberOfVisuals_);
    if (visuals_[deltaTrajectory->object_id].size() != numberOfVisuals) {
        visuals_[deltaTrajectory->object_id].clear();
        visuals_[deltaTrajectory->object_id].reserve(numberOfVisuals);
        for (int i = 0; i < numberOfVisuals; i++) {
            visuals_[deltaTrajectory->object_id].emplace_back(std::make_shared<DesiredMotionVisual>(
                context_->getSceneManager(), scene_node_, radiusPropertys_[deltaTrajectory->object_id]->getFloat()));
        }
    }

    // calculate and set Position and Color for Visuals for corresponding vehicle
    // currentDeltaTime is increased depending on the time interval selected by the user
    // circles are displayed at fixed absolute times for better visualization
    double absoluteTimeNow, firstDeltaDisplayTime;
    absoluteTimeNow = ros::Time::now().toSec();
    firstDeltaDisplayTime = -fmod(absoluteTimeNow, timeInterval_) + timeInterval_;
    for (int i = 0; (i * timeInterval_ <= maxTime_ && i * timeInterval_ <= maxDeltaTime &&
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

void DesiredMotionDisplay::reCalcVisual(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& deltaTrajectory,
                                        const automated_driving_msgs::ObjectState& objectState,
                                        object_id_type objectId,
                                        const double currentAbsoluteTime,
                                        std::shared_ptr<DesiredMotionVisual> visual) {

    // Account for difference of objectStateMsg and desiredMotionMsg
    double startDt = objectState.header.stamp.toSec() - deltaTrajectory->header.stamp.toSec();
    if (startDt < deltaTrajectory->delta_poses_with_delta_time[0].delta_time.toSec() ||
        startDt > deltaTrajectory->delta_poses_with_delta_time.back().delta_time.toSec()) {
        return;
    }
    double currentDt = currentAbsoluteTime - deltaTrajectory->header.stamp.toSec();
    if (currentDt < deltaTrajectory->delta_poses_with_delta_time[0].delta_time.toSec() ||
        currentDt > deltaTrajectory->delta_poses_with_delta_time.back().delta_time.toSec()) {
        return;
    }

    // Interpolate Start Pose
    double scale = 0.0;
    size_t j = 0;
    std::tie(j, scale) = util_localization_mgmt::getInterpolationIndexAndScale(deltaTrajectorys_[objectId], startDt);
    geometry_msgs::Pose newDeltaPoseStart =
        util_localization_mgmt::interpolatePose(deltaTrajectory->delta_poses_with_delta_time[j].delta_pose,
                                                deltaTrajectory->delta_poses_with_delta_time[j + 1].delta_pose,
                                                scale);

    // Interpolate Current Pose
    scale = 0.0;
    j = 0;
    std::tie(j, scale) = util_localization_mgmt::getInterpolationIndexAndScale(deltaTrajectorys_[objectId], currentDt);
    geometry_msgs::Pose newDeltaPose =
        util_localization_mgmt::interpolatePose(deltaTrajectory->delta_poses_with_delta_time[j].delta_pose,
                                                deltaTrajectory->delta_poses_with_delta_time[j + 1].delta_pose,
                                                scale);

    // Calculate start Pose of delta trajectory
    geometry_msgs::Pose startPose =
        util_localization_mgmt::subtractDeltaPose(objectState.motion_state.pose.pose, newDeltaPoseStart);

    // Calculate final Pose
    geometry_msgs::Pose finalPose = util_localization_mgmt::addDeltaPose(startPose, newDeltaPose);

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
    util_rviz::setSafePosition(visual, position);
    util_rviz::setSafeOrientation(visual, orientation);

    // calculate and set color
    double colorDeltaTime, hue;
    colorDeltaTime = fmod(currentAbsoluteTime, colorMaxTime_);
    hue = colorDeltaTime / colorMaxTime_;
    visual->setColor(hue, 1, 1, 1);
}

void DesiredMotionDisplay::createObjectProperty(object_id_type objectId) {
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

} // namespace desired_motion_rviz_plugin_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(desired_motion_rviz_plugin_ros::DesiredMotionDisplay, rviz::Display)
