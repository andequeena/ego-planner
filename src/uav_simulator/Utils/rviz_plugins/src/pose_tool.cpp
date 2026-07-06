// 文件说明：pose_tool.cpp 属于rviz_plugins：提供在 RViz 中交互发布目标位姿的工具。
// 注释原则：说明接口意图、数据单位、坐标系及关键算法步骤；保持原有行为不变。

/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

#include "rviz/geometry.h"
#include "rviz/load_resource.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/render_panel.h"
#include "rviz/viewport_mouse_event.h"

#include "pose_tool.h"

namespace rviz
{

Pose3DTool::Pose3DTool()
  : Tool()
  , arrow_(NULL)
{
}

Pose3DTool::~Pose3DTool()
{
  delete arrow_;
}

void
Pose3DTool::onInitialize()
{
  arrow_ = new Arrow(scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void
Pose3DTool::activate()
{
  setStatus("Left-drag: set yaw. Shift + left-drag vertically: set height.");
  state_ = Position;
}

void
Pose3DTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}

int
Pose3DTool::processMouseEvent(ViewportMouseEvent& event)
{
  int                  flags = 0;
  static Ogre::Vector3 ang_pos;
  static double        initz;
  static double        prevz;
  static double        prevangle;
  const double         z_scale    = 50;
  const double         z_interval = 0.5;
  if (event.leftDown())
  {
    ROS_ASSERT(state_ == Position);
    Ogre::Vector3 intersection;
    Ogre::Plane   ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    if (getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x,
                                    event.y, intersection))
    {
      pos_ = intersection;
      arrow_->setPosition(pos_);
      state_ = Orientation;
      flags |= Render;
    }
  }
  else if (event.type == QEvent::MouseMove && event.left())
  {
    if (state_ == Orientation)
    {
      // compute angle in x-y plane
      Ogre::Vector3 cur_pos;
      Ogre::Plane   ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if (getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x,
                                      event.y, cur_pos))
      {
        double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);
        arrow_->getSceneNode()->setVisible(true);
        arrow_->setDirection(Ogre::Vector3(cos(angle), sin(angle), 0.0));
        if (event.shift())
          state_  = Height;
        initz     = pos_.z;
        prevz     = event.y;
        prevangle = angle;
        flags |= Render;
      }
    }
    if (state_ == Height)
    {
      double z  = event.y;
      double dz = z - prevz;
      prevz     = z;
      pos_.z -= dz / z_scale;
      arrow_->setPosition(pos_);
      // Create a list of arrows
      for (int k = 0; k < arrow_array.size(); k++)
        delete arrow_array[k];
      arrow_array.clear();
      int cnt = ceil(fabs(initz - pos_.z) / z_interval);
      for (int k = 0; k < cnt; k++)
      {
        Arrow* arrow__;
        arrow__ = new Arrow(scene_manager_, NULL, 0.5f, 0.1f, 0.0f, 0.1f);
        arrow__->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        arrow__->getSceneNode()->setVisible(true);
        Ogre::Vector3 arr_pos = pos_;
        arr_pos.z = initz - ((initz - pos_.z > 0) ? 1 : -1) * k * z_interval;
        arrow__->setPosition(arr_pos);
        arrow__->setDirection(
          Ogre::Vector3(cos(prevangle), sin(prevangle), 0.0));
        arrow_array.push_back(arrow__);
      }
      flags |= Render;
    }
  }
  else if (event.leftUp())
  {
    if (state_ == Orientation || state_ == Height)
    {
      // Create a list of arrows
      for (int k = 0; k < arrow_array.size(); k++)
        delete arrow_array[k];
      arrow_array.clear();
      onPoseSet(pos_.x, pos_.y, pos_.z, prevangle);
      flags |= (Finished | Render);
    }
  }

  return flags;
}
}
