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

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include <math.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <ros/ros.h>

#include <automated_driving_msgs/ObjectStateArray.h>
#include <util_rviz/util_rviz.hpp>

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
