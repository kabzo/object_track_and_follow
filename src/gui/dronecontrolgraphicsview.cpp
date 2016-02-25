/*  Copyright 2012 UdeS University of Sherbrooke
 *
 *   This file is part of ROS_OpenTLD.
 *
 *   ROS_OpenTLD is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ROS_OpenTLD is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with ROS_OpenTLD. If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 * base_frame_graphics_view.cpp
 *
 *  Created on: May 17, 2012
 *      Author: Ronan Chauvin
 */

#include <iostream>
#include <ros/ros.h>

#include "dronecontrolgraphicsview.h"

DroneControlGraphicsView::DroneControlGraphicsView(QWidget * parent) : QGraphicsView(parent)
{
  m_scene = new QGraphicsScene(this);
  this->setScene(m_scene);

  m_pen = new QPen();
  m_brush = new QBrush(Qt::SolidPattern);
  m_item_pixmap = m_scene->addPixmap(QPixmap("../../res/opentld.png"));

  m_pen->setColor(QColor(0,0,255));
  m_brush->setColor(QColor(255,255,255,64));

  m_item_rect = new QGraphicsRectItem();
  m_item_rect->setPen(*m_pen);
  m_item_rect->setBrush(*m_brush);
  m_scene->addItem(m_item_rect);

  target_drone_view = new QTarget();
  target_tracked_object = new QTarget();
  target_drone_view->add_to_scene(m_scene);
  target_tracked_object->add_to_scene(m_scene);

  cmd_arrow = new QArrow();
  cmd_arrow->add_to_scene(m_scene);

  reference_object_followed = new QGraphicsRectItem();
  m_pen->setColor((QColor(0,255,0)));
  m_pen->setWidth(5);
  reference_object_followed->setPen(*m_pen);
  reference_object_followed->setBrush(*m_brush);
  m_scene->addItem(reference_object_followed);
  reference_object_followed->setRect(0,0,0,0);


  this->setCursor(Qt::ArrowCursor);

  _firts_init_lines = true;
}



DroneControlGraphicsView::~DroneControlGraphicsView() 
{
  delete m_pen;
  delete m_brush;
}

QGraphicsRectItem * DroneControlGraphicsView::get_bb() const
{
  return m_item_rect;
}

QPen * DroneControlGraphicsView::get_pen() const
{
  return m_pen;
}

QBrush * DroneControlGraphicsView::get_brush() const
{
  return m_brush;
}

void DroneControlGraphicsView::image_received(const QImage & img)
{
  m_item_pixmap->setPixmap(QPixmap::fromImage(img));
}

void DroneControlGraphicsView::tracked_objet_changed(const QRectF & rect)
{
  //The tracker node sent a bounding box
  if(rect.width()>1 || rect.height()>1)
    {
      if(!m_item_rect->isVisible() || !target_tracked_object->isVisible()|| !reference_object_followed->isVisible() ){
          m_item_rect->show();
          target_tracked_object->show();
          reference_object_followed->show();
        }
      m_item_rect->setRect(rect);
      target_tracked_object->draw_target(QPoint(rect.x() + rect.width()/2,rect.y() + rect.height()/2));

      int referenc_object_x = target_tracked_object->center.x()-reference_object_followed->rect().width()/2;
      int referenc_object_y = target_tracked_object->center.y()-reference_object_followed->rect().height()/2;

      reference_object_followed->setRect(referenc_object_x,referenc_object_y,reference_object_followed->rect().width(),reference_object_followed->rect().height());

    }else{
      m_item_rect->hide();
      target_tracked_object->hide();
      reference_object_followed->hide();
    }

}

void DroneControlGraphicsView::center_view_changed(const QPoint & p){
  target_drone_view->draw_target(p);
}

void DroneControlGraphicsView::cmd_received(const geometry_msgs::Twist &cmd){

  if(cmd.angular.z==0 && cmd.linear.z==0){
      cmd_arrow->hide();
    }else{
      if(!cmd_arrow->isVisible())
        cmd_arrow->show();

      int factor = 100;
      QPoint error_pid;
      error_pid.setX(-cmd.angular.z*factor);
      error_pid.setY(-cmd.linear.z*factor);

      QPoint center = target_drone_view->center;
      cmd_arrow->drawArrow(center,QPoint(center.x()+error_pid.x(),center.y()+error_pid.y()));
    }
}



void DroneControlGraphicsView::resizeEvent(QResizeEvent * event)
{
  fitInView(this->sceneRect(), Qt::KeepAspectRatio);
  QGraphicsView::resizeEvent(event);
}

void DroneControlGraphicsView::followed_object_size(const int width,const int height){

  reference_object_followed->setRect(reference_object_followed->rect().x(),reference_object_followed->rect().y(),width,height);
}

