/* Copyright (c) 2013-2015 Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 * MeshDisplayCustom class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on the rviz image display class.
 *
 * Latest changes (12/11/2012):
 * - fixed segfault issues
 */
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
 * LIABLE FOR ANY DImesh, INDImesh, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreFrustum.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OgreException.h>
#include <OgreHardwarePixelBuffer.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_common.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rviz/display_context.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/render_panel.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/tf_link_updater.h>
#include <rviz/validate_floats.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz_textured_meshes/mesh_display_custom.h>
#include <sensor_msgs/image_encodings.h> 
#include <string>
#include <vector>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <shape_msgs/Mesh.h>

namespace rviz
{

MeshDisplayCustom::MeshDisplayCustom()
  : Display()
  , new_image_(false)
  , custom_mesh_entity_(nullptr)
  , custom_mesh_node_(nullptr)
{
  image_topic_property_ = new RosTopicProperty("Image Topic", "",
      QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
      "Image topic to subscribe to.",
      this, SLOT(updateDisplayImages()));

  tf_frame_property_ = new TfFrameProperty("Quad Frame", "map",
      "Align the image quad to the xy plane of this tf frame",
      this, 0, true, SLOT(updateDisplayMesh()));

  mesh_location_ = new StringProperty("Mesh location", "", "Desired mesh location", this, SLOT(updateDisplayMesh()));

  scale_property_ = new VectorProperty("Mesh scale", Ogre::Vector3::UNIT_SCALE, "Desired mesh scale", this, SLOT(updateMeshScale()));
}

MeshDisplayCustom::~MeshDisplayCustom()
{
  unsubscribe();
}

void MeshDisplayCustom::onInitialize()
{
  tf_frame_property_->setFrameManager(context_->getFrameManager());
  Display::onInitialize();
}

void MeshDisplayCustom::updateImage(const sensor_msgs::Image::ConstPtr& image)
{
  image_mutex_.lock(); 

  cur_image_ = image;
  new_image_ = true;

  image_mutex_.unlock();
}

void MeshDisplayCustom::updateDisplayImages()
{
  unsubscribe();
  subscribe();
}

void MeshDisplayCustom::updateMeshScale()
{
  if (custom_mesh_node_)
  {
    custom_mesh_node_->setScale(scale_property_->getVector());
  }
}

geometry_msgs::Pose MeshDisplayCustom::getMeshOrigin()
{
  const auto frame = tf_frame_property_->getFrameStd();

  geometry_msgs::Pose mesh_origin;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  if (!context_->getFrameManager()->getTransform(frame, ros::Time(0),
      position, orientation))
  {
    std::stringstream ss;
    ss << "Error transforming from fixed frame to frame " << frame;
    throw std::runtime_error(ss.str());
  }

  mesh_origin.position.x = position[0];
  mesh_origin.position.y = position[1];
  mesh_origin.position.z = position[2];
  mesh_origin.orientation.w = orientation[0];
  mesh_origin.orientation.x = orientation[1];
  mesh_origin.orientation.y = orientation[2];
  mesh_origin.orientation.z = orientation[3];

  return mesh_origin;
}

void MeshDisplayCustom::updateDisplayMesh()
{
  namespace fs = boost::filesystem;

  if (mesh_location_->getStdString().empty())
    return;

  const auto location = mesh_location_->getStdString();
  fs::path mesh_path(location);
  fs::path mesh_dir = mesh_path.parent_path();
  fs::path mesh_filename = mesh_path.filename();

  geometry_msgs::Pose mesh_origin;

  try 
  {
    mesh_origin =  getMeshOrigin();
  }
  catch(std::runtime_error ex)
  {
    ROS_ERROR("%s",ex.what());
    return;
  }

  if (mesh_filename.has_filename() && fs::exists(mesh_path)) 
  {
    try
    {
      Ogre::String resource_group_name = "custom_mesh";
      Ogre::ResourceGroupManager& resource_manager = Ogre::ResourceGroupManager::getSingleton();

      if (custom_mesh_entity_)
      {
        scene_manager_->destroySceneNode(custom_mesh_node_);
      }

      resource_manager.addResourceLocation(mesh_dir.string(), "FileSystem", resource_group_name, false);
      custom_mesh_entity_ = scene_manager_->createEntity(mesh_filename.string());
      custom_mesh_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
      
      custom_mesh_node_->setPosition(mesh_origin.position.x, mesh_origin.position.y, mesh_origin.position.z);
      custom_mesh_node_->setOrientation(mesh_origin.orientation.w, mesh_origin.orientation.x, mesh_origin.orientation.y, mesh_origin.orientation.z);
      custom_mesh_node_->attachObject(custom_mesh_entity_);
      custom_mesh_node_->setScale(scale_property_->getVector());

      Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual(
            "RttTex", 
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            Ogre::TEX_TYPE_2D,      
            240, 240,         
            0,                
            Ogre::PF_BYTE_BGRA,
            Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

      Ogre::MaterialPtr render_material = 
        Ogre::MaterialManager::getSingleton().create(
          "RttMat", 
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

      render_material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
      render_material->getTechnique(0)->getPass(0)->createTextureUnitState("RttTex");

      custom_mesh_entity_->setMaterial(render_material);

    }
    catch(std::exception ex)
    {
      ROS_ERROR("Cannot load mesh file");
    }
  }
}

void MeshDisplayCustom::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  if (!image_topic_property_->getTopic().isEmpty())
  {
    try
    {
      image_sub_ = nh_.subscribe(image_topic_property_->getTopicStd(),
          1, &MeshDisplayCustom::updateImage, this);
      setStatus(StatusProperty::Ok, "Display Images Topic", "ok");
    }
    catch (ros::Exception& e)
    {
      setStatus(StatusProperty::Error, "Display Images Topic", QString("Error subscribing: ") + e.what());
    }
  }
}

void MeshDisplayCustom::unsubscribe()
{
  image_sub_.shutdown();
}

void MeshDisplayCustom::load()
{
  return;
}

void MeshDisplayCustom::onEnable()
{
  subscribe();
}

void MeshDisplayCustom::onDisable()
{
  unsubscribe();
}

void MeshDisplayCustom::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  return;
}

//from: https://forums.ogre3d.org/viewtopic.php?t=81095
void MeshDisplayCustom::writeCvToTex(cv::Mat& img, Ogre::TexturePtr tex)
{
  using namespace Ogre;

  PixelFormat pf = tex->getFormat();
  int pixelStep = PixelUtil::getNumElemBytes(pf);
  
  Ogre::uint8* src = img.ptr();
  
  Ogre::Image::Box region(0,0, tex->getWidth(), tex->getHeight());
  const PixelBox& pb = tex->getBuffer()->lock(region, HardwareBuffer::HBL_DISCARD);
  
  Ogre::uint8* dest = static_cast<Ogre::uint8*>(pb.data);
  int numChannels = img.channels();
  
  for(int w=region.left; w<region.right; w++)
  {
    for(int h=region.top; h<region.bottom; h++)
    {
      ColourValue cv;
      PixelUtil::unpackColour(&cv, pf, img.ptr(0) + (h * img.cols + w) * numChannels);
      PixelUtil::packColour(cv, pf, dest);
      
      dest += pixelStep;
    }
  }

  tex->getBuffer()->unlock();
}

void MeshDisplayCustom::update(float wall_dt, float ros_dt)
{
  image_mutex_.lock();

  if (cur_image_)
  {
    if (custom_mesh_entity_)
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(cur_image_, sensor_msgs::image_encodings::BGR8);

      Ogre::TexturePtr texture = Ogre::TextureManager::getSingletonPtr()->getByName("RttTex");

      cv::Mat resized_img(texture->getHeight(), texture->getWidth(), cv_ptr->image.type());
      cv::resize(cv_ptr->image, resized_img, resized_img.size());
      cv::transpose(resized_img, resized_img);

      writeCvToTex(resized_img, texture);
    }

    new_image_ = false;
  }
  
  image_mutex_.unlock();

  setStatus(StatusProperty::Ok, "Display Image", "ok");
}

void MeshDisplayCustom::reset()
{
  Display::reset();
}

}  // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::MeshDisplayCustom, rviz::Display)
