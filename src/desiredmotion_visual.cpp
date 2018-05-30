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

#include "desiredmotion_visual.hpp"

namespace desired_motion_rviz_plugin_ros {

DesiredMotionVisual::DesiredMotionVisual(Ogre::SceneManager* scene_manager,
                                         Ogre::SceneNode* parent_node,
                                         float radius,
                                         const float lineWidth,
                                         const uint8_t nCircleElements) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    // set Radius
    setRadiusHelper(radius);
    // set LineWidth
    setLineWidthHelper(lineWidth);

    // Circle Elements must be at least 3
    nCircleElements_ = (nCircleElements >= 3 ? nCircleElements : 3);
    circle_.reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    createCircle(radius_, nCircleElements_, lineWidth_);
}

DesiredMotionVisual::~DesiredMotionVisual() {
    scene_manager_->destroySceneNode(frame_node_);
}

void DesiredMotionVisual::createCircle(const float radius, const uint8_t nCircleElements, const float lineWidth) {
    circle_->clear();
    circle_->setLineWidth(lineWidth);
    const double PIESLICE = M_PI / nCircleElements;
    for (uint8_t i = 0; i <= 2 * nCircleElements; i += 2) {
        circle_->addPoint(Ogre::Vector3(radius * cos(i * PIESLICE), radius * sin(i * PIESLICE), 0));
    }
}

// Position and orientation are passed through to the SceneNode.
void DesiredMotionVisual::setPosition(const Ogre::Vector3& position) {
    util_rviz::setPositionSafely(frame_node_, position);
}

void DesiredMotionVisual::setOrientation(const Ogre::Quaternion& orientation) {
    util_rviz::setOrientationSafely(frame_node_, orientation);
}

// Set Circle Size. This results in creating a new Circle
void DesiredMotionVisual::setRadius(const float radius) {
    setRadiusHelper(radius);
    createCircle(radius_, nCircleElements_, lineWidth_);
}

void DesiredMotionVisual::setRadiusHelper(const float radius) {
    if (radius > 0) {
        radius_ = radius;
    } else {
        ROS_WARN_THROTTLE(1, "Radius must be greater than zero. Set Default Value 1.0");
        radius_ = 1.0;
    }
}

// The higher nCircleElements the "rounder" the circle. Less elements for better performance.
void DesiredMotionVisual::setNCircleElements(const uint8_t n) {
    // Circle Elements must be at least 3
    nCircleElements_ = (n >= 3 ? n : 3);
}

void DesiredMotionVisual::setLineWidth(const float lineWidth) {
    setLineWidthHelper(lineWidth_);
    circle_->setLineWidth(lineWidth_);
}

void DesiredMotionVisual::setLineWidthHelper(const float lineWidth) {
    if (lineWidth > 0) {
        lineWidth_ = lineWidth;
    } else {
        ROS_WARN_THROTTLE(1, "LineWidth must be greater than zero. Set Default Value 0.1");
        lineWidth_ = 0.1;
    }
}

// Color is passed through to the Shape object.
void DesiredMotionVisual::setColor(const float hue, const float saturation, const float brightness, const float alpha) {
    Ogre::ColourValue rgb_color;
    rgb_color.setHSB(hue, saturation, brightness);
    circle_->setColor(rgb_color.r, rgb_color.g, rgb_color.b, alpha);
}

void DesiredMotionVisual::setVisible(const bool visible) {
    frame_node_->setVisible(visible);
}

} // namespace desired_motion_rviz_plugin_ros
