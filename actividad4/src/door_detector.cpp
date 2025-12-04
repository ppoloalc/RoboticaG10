//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>
#include <cppitertools/enumerate.hpp>


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    //Devolver vector de puertas. Cada puerta es una estructura formada por 2 puntos con sus angulos
    //Recorrer lidar con sliding window.
    Peaks peaks;
    for (const auto &p: points | iter::sliding_window(2))
    {
        const auto &p1 = p[0];
        const auto &p2 = p[1];
        if (abs(p1.distance2d -p2.distance2d) > 1000.f )
        {
            //Imprimir que picos cumplen la condicion
            //qInfo() << "Pico: " << p1.distance2d << " " << p2.distance2d;
            if (p1.distance2d > p2.distance2d)
                peaks.emplace_back(Eigen::Vector2f{p2.x, p2.y}, p2.phi);
            else
                peaks.emplace_back(Eigen::Vector2f{p1.x, p1.y}, p1.phi);
        }
    }


    ///////////////////////////////////////////////////////////////////////
    // non-maximum suppression of peaks: remove peaks closer than 500mm
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
        if (const bool too_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2) { return (p - std::get<0>(p2)).norm() < 500.f; }); not too_close)
            nms_peaks.emplace_back(p, a);
    peaks = nms_peaks;

    static std::vector<QGraphicsItem*> items;
    for (const auto &i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();
    for (const auto &[p, _] : peaks)
    {
        auto item = scene->addEllipse(-100, -100, 200, 200, QPen(QColor("yellow")), QBrush(QColor("yellow")));
        item->setPos(p.x(), p.y());
        items.push_back(item);
    }

    Doors doors;
    for (const auto &pe : peaks | iter::combinations(2))
    {
        const auto &[p1, a1] = pe[0];
        const auto &[p2, a2] = pe[1];
        float distance = (p1 - p2).norm();
        if ( distance > 700.f and distance < 1300.f )
        {

            // Pintar linea entre los 2 puntos de la puerta
            doors.emplace_back(p2,  a2 ,p1, a1);
            const auto item = scene->addLine(p2.x(), p2.y(), p1.x(), p1.y(), QPen(QColor("red"), 60));
            items.push_back(item);
        }
    }
    return doors;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);

    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    for(const auto &p : points)
    {
        for(const auto &[i,d] : doors | iter::enumerate)
        {
            const float dist_to_door = d.center().norm();
            // Check if the angular range wraps around the -π/+π boundary
            const bool angle_wraps = d.p2_angle < d.p1_angle;

            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) or (p.phi < d.p2_angle);
            }
            else
            {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) and (p.phi < d.p2_angle);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if(point_in_angular_range and p.distance2d >= dist_to_door)
                break;

            if (i==doors.size() -1)
                filtered.emplace_back(p);
        }
    }
    return filtered;
}
