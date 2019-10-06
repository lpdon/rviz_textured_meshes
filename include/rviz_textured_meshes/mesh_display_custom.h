/* Copyright (c) 2013-2015 Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
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

#ifndef RVIZ_TEXTURED_QUADS_MESH_DISPLAY_CUSTOM_H
#define RVIZ_TEXTURED_QUADS_MESH_DISPLAY_CUSTOM_H

#include <QObject>
// kinetic compatibility http://answers.ros.org/question/233786/parse-error-at-boost_join/
#ifndef Q_MOC_RUN

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRenderQueueListener.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreWindowEventUtilities.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <map>
#include <message_filters/subscriber.h>
#include <rviz/display.h>
#include <rviz/frame_manager.h>
#include <rviz/image/image_display_base.h>
#include <rviz/image/ros_image_texture.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <shape_msgs/Mesh.h>
#include <std_msgs/Float64.h>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <vector>

#endif  // Q_MOC_RUN

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace cv
{
class Mat;
}

namespace rviz
{
class FloatProperty;
class RenderPanel;
class RosTopicProperty;
class TfFrameProperty;
class VectorProperty;

class MeshDisplayCustom: public rviz::Display,  public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
  Q_OBJECT
public:
  MeshDisplayCustom();
  virtual ~MeshDisplayCustom();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

private Q_SLOTS:
  void updateDisplayImages();
  void updateDisplayMesh();
  void updateMeshScale();

protected:
  virtual void load();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  // This is called by incomingMessage().
  void processImage(int index, const sensor_msgs::Image& msg);

  virtual void subscribe();
  virtual void unsubscribe();

private:
  void clear();
  void writeCvToTex(cv::Mat& img, Ogre::TexturePtr tex);
  void updateImage(const sensor_msgs::Image::ConstPtr& image);
  geometry_msgs::Pose getMeshOrigin();

  RosTopicProperty* image_topic_property_;
  TfFrameProperty* tf_frame_property_;
  StringProperty* mesh_location_;
  VectorProperty* scale_property_;

  ros::Subscriber image_sub_;

  ros::NodeHandle nh_;

  bool new_image_;
  sensor_msgs::Image::ConstPtr cur_image_;

  RenderPanel* render_panel_;  // this is the active render panel

  boost::mutex image_mutex_;

  Ogre::Entity* custom_mesh_entity_;
  Ogre::SceneNode* custom_mesh_node_;
};

}  // namespace rviz

#endif  // RVIZ_TEXTURED_MESHES_MESH_DISPLAY_CUSTOM_H


