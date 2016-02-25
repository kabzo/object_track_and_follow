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
 * base_frame_graphics_view.hpp
 *
 *  Created on: May 17, 2012
 *      Author: Ronan Chauvin
 */

#ifndef DRONE_CONTROL_GRAPHICS_VIEW_H
#define DRONE_CONTROL_GRAPHICS_VIEW_H

#include <QtGui/QApplication>

#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsView>
#include <QtGui/QGraphicsPixmapItem>
#include <QtGui/QGraphicsRectItem>
#include <QtGui/QGraphicsItem>

#include <QtGui/QMouseEvent>
#include <QtGui/QResizeEvent>

#include <QtGui/QImage>
#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QColor>
#include <QtCore/QPoint>
#include <QtCore/QRectF>
#include <QtCore/QDebug>

#include <geometry_msgs/Twist.h>

struct QTarget
{

    QTarget()
    {
        line1 = new QGraphicsLineItem();
        line2 = new QGraphicsLineItem();
        ellipse = new QGraphicsEllipseItem();

        pen = new QPen();
        pen->setColor(QColor(0,0,0));

        line1->setPen(*pen);
        line1->setPen(*pen);

        pen->setColor(QColor(255,0,0));
        pen->setWidth(10);

        ellipse->setPen(*pen);
    }

    void add_to_scene(QGraphicsScene * s)
    {
        s->addItem(line1);
        s->addItem(line2);
        s->addItem(ellipse);
    }

    void draw_target(QPoint p)
    {
        center = p;

        if(p.x() != 0 && p.y() != 0)
            {
                int size_line = 50;
                line1->setLine(p.x()-size_line/2,p.y(),p.x()+size_line/2,p.y());
                line2->setLine(p.x(),p.y()-size_line/2,p.x(),p.y()+size_line/2);

                int size = 10;
                ellipse->setRect(p.x()-size/2,p.y()-size/2,size,size);
            }
        else
            {
                line1->setLine(0,0,0,0);
                line2->setLine(0,0,0,0);
                ellipse->setRect(0,0,0,0);
            }
    }
    void hide(){
      line1->hide();
      line2->hide();
      ellipse->hide();
    }

    void show(){
      line1->show();
      line2->show();
      ellipse->show();
    }

    bool isVisible(){
      return line1->isVisible() && line2->isVisible() && ellipse->isVisible();
    }

    QPoint center;
    QGraphicsLineItem *line1;
    QGraphicsLineItem *line2;
    QGraphicsEllipseItem *ellipse;
    QPen *pen;

};

struct QArrow{
    QArrow(){
        line_arrow1 = new QGraphicsLineItem();
        line_arrow2 = new QGraphicsLineItem();
        line = new QGraphicsLineItem();

        pen = new QPen();
        pen->setColor(QColor(0,0,0));
        pen->setWidth(3);

        line_arrow1->setPen(*pen);
        line_arrow2->setPen(*pen);
        line->setPen(*pen);
    }

    void add_to_scene(QGraphicsScene * s)
    {
        s->addItem(line);
        s->addItem(line_arrow1);
        s->addItem(line_arrow2);
    }

    void drawArrow(QPoint p, QPoint q, int arrowMagnitude = 9)
    {
        //Draw the principle line
        line->setLine(p.x(),p.y(), q.x(),q.y());
        //compute the angle alpha
        double angle = atan2((double)p.y()-q.y(), (double)p.x()-q.x());
        //compute the coordinates of the first segment
        p.setX((int) ( q.x() +  arrowMagnitude * cos(angle + M_PI/4)));
        p.setY((int) ( q.y() +  arrowMagnitude * sin(angle + M_PI/4)));
        //Draw the first segment
        line_arrow1->setLine(p.x(),p.y(), q.x(),q.y());
        //compute the coordinates of the second segment
        p.setX((int) ( q.x() +  arrowMagnitude * cos(angle - M_PI/4)));
        p.setY((int) ( q.y() +  arrowMagnitude * sin(angle - M_PI/4)));
        //Draw the second segment
        line_arrow2->setLine(p.x(),p.y(), q.x(),q.y());
    }

    void hide(){
      line->hide();
      line_arrow1->hide();
      line_arrow2->hide();
    }

    void show(){
      line->show();
      line_arrow1->show();
      line_arrow2->show();
    }

    bool isVisible(){
      return line->isVisible() && line_arrow1->isVisible() && line_arrow2->isVisible();
    }


    QPoint start;
    QPoint end;

    QGraphicsLineItem *line;
    QGraphicsLineItem *line_arrow1;
    QGraphicsLineItem *line_arrow2;

    QPen *pen;


};

class DroneControlGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    DroneControlGraphicsView(QWidget * parent);
    ~DroneControlGraphicsView();
    QGraphicsRectItem * get_bb() const;
    bool get_correct_bb();
    QPen * get_pen() const;
    QBrush * get_brush() const;
    QTarget *get_target_drone_view(){
        return target_drone_view;
    }
    QGraphicsRectItem *get_followed_object(){
        return reference_object_followed;
    }

protected:
    void resizeEvent(QResizeEvent * event);

private:

    QGraphicsRectItem *reference_object_followed;
    QGraphicsPixmapItem * m_item_pixmap;
    QGraphicsRectItem * m_item_rect;
    QTarget *target_drone_view;
    QTarget *target_tracked_object;
    QArrow *cmd_arrow;

    QGraphicsScene * m_scene;
    QPen * m_pen;
    QBrush * m_brush;

    bool _firts_init_lines;

public slots:
    void image_received(const QImage & img);
    void tracked_objet_changed(const QRectF & rect);
    void center_view_changed(const QPoint & p);
    void cmd_received(const geometry_msgs::Twist &cmd);
    void followed_object_size(const int width,const int height);

signals:
    void sig_bb_set(QRectF rect);
};

#endif
