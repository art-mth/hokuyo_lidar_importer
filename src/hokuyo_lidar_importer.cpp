#include "hokuyo_lidar_importer.h"

#include <math.h>

namespace {
const char* kDeviceFile = "/dev/ttyACM0";
const long kBaudrate = 115200;
const double kStartDegree = -90;
const double kEndDegree = 90;
}

bool HokuyoLidarImporter::initialize() {
    if (!m_lidar.open(kDeviceFile, kBaudrate, qrk::Urg_driver::Serial)) {
        logger.error() << "Failed opening Hokuyo Lidar at " << kDeviceFile
                       << " error: " << m_lidar.what();
        return false;
    }
    m_lidar.set_scanning_parameter(m_lidar.deg2step(kStartDegree),
                                   m_lidar.deg2step(kEndDegree), 0);
    m_lidar.start_measurement();

    return true;
}

bool HokuyoLidarImporter::deinitialize() {
    m_lidar.stop_measurement();

    return true;
}

void HokuyoLidarImporter::configsChanged() {}

bool HokuyoLidarImporter::cycle() {
    std::vector<long> data;
    if (!m_lidar.get_distance(data)) {
        logger.error() << "Failed getting distance from Hokuyo Lidar error: "
                       << m_lidar.what();
        return false;
    }
    printLidarData(data);

    return true;
}

void HokuyoLidarImporter::printLidarData(const std::vector<long>& data) {
    for (std::vector<long>::size_type i = 0; i < 1; i++) {
        long l = data[i];
        double radian = m_lidar.index2rad(i);
        long x = l * cos(radian);
        long y = l * sin(radian);
        logger.info("LidarData") << "(" << x << ", " << y << ")[mm]";
    }
}
