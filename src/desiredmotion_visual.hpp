#pragma once

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include <math.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <ros/ros.h>

#include <automated_driving_msgs/ObjectStateArray.h>
#include <simulation_utils/util_rviz.hpp>

namespace desired_motion_rviz_plugin_ros {

class DesiredMotionVisual {
public:
    DesiredMotionVisual(Ogre::SceneManager* scene_manager,
                        Ogre::SceneNode* parent_node,
                        const float radius = 1.0,
                        const float lineWidth = 0.1,
                        const uint8_t nCircleElements = 32);
    virtual ~DesiredMotionVisual();

    void createCircle(const float radius, const uint8_t nCircleElements, const float lineWidth);
    void setPosition(const Ogre::Vector3& position);
    void setOrientation(const Ogre::Quaternion& orientation);
    void setRadius(const float radius);
    void setRadiusHelper(const float radius);
    void setNCircleElements(const uint8_t n);
    void setLineWidth(const float lineWidth);
    void setLineWidthHelper(const float lineWidth);
    void setColor(const float hue, const float saturation, const float brightness, const float alpha);
    void setVisible(const bool visible);

private:
    // The object implementing the actual shape
    std::shared_ptr<rviz::BillboardLine> circle_;

    // A SceneNode whose pose is set to match the coordinate frame of
    // the DesiredMotion message header.
    Ogre::SceneNode* frame_node_;

    // The SceneManager, kept here only so the destructor can ask it to
    // destroy the ``frame_node_``.
    Ogre::SceneManager* scene_manager_;

    // circle parameters
    float radius_;
    float lineWidth_;
    // The higher nCircleElements the "rounder" the circle. Less elements for better performance.
    uint8_t nCircleElements_;
};

} // namespace desired_motion_rviz_plugin_ros
