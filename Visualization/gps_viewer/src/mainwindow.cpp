/***************************************************************************
 * QGeoView is a Qt / C ++ widget for visualizing geographic data.
 * Copyright (C) 2018-2023 Andrey Yaroshenko.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, see https://www.gnu.org/licenses.
 ****************************************************************************/

#include "gps_viewer/mainwindow.hpp"

#include <QRandomGenerator>
#include <QTimer>

#include <gps_viewer/utils/helpers.h>

#include <QGeoView/QGVLayerOSM.h>
#include <rclcpp/logging.hpp>

MainWindow::MainWindow()
{
    setWindowTitle("ROS-GPSViewer");

    mMap = new QGVMap(this);
    setCentralWidget(mMap);

    Helpers::setupCachedNetworkAccessManager(this);

    // Background layer
    auto osmLayer = new QGVLayerOSM();
    mMap->addItem(osmLayer);

    // Custom layer
    auto customLayer = new QGVLayer();
    mMap->addItem(customLayer);

    // Add moving item in custom layer
    circleMarker = new PlacemarkCircle(QGV::GeoPos(0,0), 0, Qt::red);
    customLayer->addItem(circleMarker);

    // Show whole world
    QTimer::singleShot(100, this, [this]() {
        auto target = mMap->getProjection()->boundaryGeoRect();
        mMap->cameraTo(QGVCameraActions(mMap).scaleTo(target));
    });
}

MainWindow::~MainWindow()
{
}

void MainWindow::DrawCircleAt(double latitude, double longitude, double radius)
{
    if(!circleMarker)
    {
        RCLCPP_ERROR(rclcpp::get_logger("gps_viewer"), "Tried to move the circle, but it was never created!");
        return;
    }

    circleMarker->setRadius(radius);
    QGV::GeoPos newPos = QGV::GeoPos(latitude, longitude);
    circleMarker->setCenter(newPos);
}
