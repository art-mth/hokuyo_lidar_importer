#include "hokuyo_lidar_importer.h"

#include <math.h>

namespace {
const char* kDeviceFile = "/dev/ttyACM0";
const long kBaudrate = 115200;
}

bool HokuyoLidarImporter::initialize() {
    data = writeChannel<lms::math::PointCloud2f>("HOKUYO_LIDAR_DATA");
    configsChanged();

    if (!m_lidar.open(kDeviceFile, kBaudrate, qrk::Urg_driver::Serial)) {
        logger.error() << "Failed opening Hokuyo Lidar at " << kDeviceFile
                       << " error: " << m_lidar.what();
        return false;
    }
    m_lidar.set_scanning_parameter(m_lidar.deg2step(m_startAtDeg),
                                   m_lidar.deg2step(m_stopAtDeg), 0);
    m_lidar.start_measurement();

    return true;
}

bool HokuyoLidarImporter::deinitialize() {
    m_lidar.stop_measurement();

    return true;
}

void HokuyoLidarImporter::configsChanged() {
    m_offsetFromOrigin.x = config().get<float>("xOffsetFromOrigin", 0);
    m_offsetFromOrigin.y = config().get<float>("yOffsetFromOrigin", 0);
    m_startAtDeg = config().get<double>("startAtDeg", -90);
    m_stopAtDeg = config().get<double>("stopAtDeg", 90);
}

bool HokuyoLidarImporter::cycle() {
    std::vector<long> rawDataPoints;
    if (!m_lidar.get_distance(rawDataPoints)) {
        logger.error() << "Failed getting distance from Hokuyo Lidar error: "
                       << m_lidar.what();
        return false;
    }

    data->points().clear();
    data->points().resize(rawDataPoints.size());
    for (std::vector<long>::size_type i = 0; i < rawDataPoints.size(); i++) {
        long l = rawDataPoints[i];
        double radian = m_lidar.index2rad(i);
        long x = l * cos(radian);
        long y = l * sin(radian);
        data->points().push_back(lms::math::vertex2f(x, y) / 1000 +
                                 m_offsetFromOrigin);
    }

    return true;
}

void HokuyoLidarImporter::printLidarData(const std::vector<long>& data) {
    for (std::vector<long>::size_type i = 0; i < data.size(); i++) {
        long l = data[i];
        double radian = m_lidar.index2rad(i);
        long x = l * cos(radian);
        long y = l * sin(radian);
        logger.info("LidarData") << "(" << x << ", " << y << ")[mm]";
    }
}
