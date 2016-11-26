#include "hokuyo_lidar_importer.h"

#include <math.h>
#include <utility>

namespace {
const char* kDeviceFile = "/dev/ttyACM0";
const long kBaudrate = 115200;
}

bool HokuyoLidarImporter::initialize() {
    data = writeChannel<lms::math::PointCloud2f>("HOKUYO_LIDAR_DATA");
    newData = writeChannel<bool>("NEW_DATA");

    configsChanged();
    m_configsChanged = false;

    if (!m_lidar.open(kDeviceFile, kBaudrate, qrk::Urg_driver::Serial)) {
        logger.error() << "Failed opening Hokuyo Lidar at " << kDeviceFile
                       << " error: " << m_lidar.what();
        return false;
    }
    m_lidar.set_scanning_parameter(m_lidar.deg2step(m_startAtDeg),
                                   m_lidar.deg2step(m_stopAtDeg), 0);
    m_lidar.start_measurement();
    startImporterThread();
    return true;
}

bool HokuyoLidarImporter::deinitialize() {
    a_importerRunning = false;
    m_importer.join();
    m_lidar.stop_measurement();
    m_lidarMutex.unlock();
    return true;
}

void HokuyoLidarImporter::configsChanged() {
    m_offsetFromOrigin.x = config().get<float>("xOffsetFromOrigin", 0);
    m_offsetFromOrigin.y = config().get<float>("yOffsetFromOrigin", 0);
    m_startAtDeg = config().get<double>("startAtDeg", -90);
    m_stopAtDeg = config().get<double>("stopAtDeg", 90);
    m_configsChanged = true;
}

bool HokuyoLidarImporter::cycle() {
    std::lock_guard<std::mutex> lock(m_dataMutex);
    if (!m_newImport) {
        *newData = false;
        return true;
    }
    m_newImport = false;
    *newData = true;

    prepRawData();

    if (m_configsChanged) {
        m_lidar.set_scanning_parameter(m_lidar.deg2step(m_startAtDeg),
                                       m_lidar.deg2step(m_stopAtDeg), 0);
    }
    m_lidarMutex.unlock();
    return true;
}

void HokuyoLidarImporter::startImporterThread() {
    m_newImport = false;
    a_importerRunning = true;
    m_importer = std::thread([this]() {
        while (a_importerRunning.load()) {
            std::vector<long> measurement;
            m_lidarMutex.lock();
            if (!m_lidar.get_distance(measurement)) {
                logger.error()
                    << "Failed getting distance from Hokuyo Lidar error: "
                    << m_lidar.what();
                m_lidarMutex.unlock();
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(m_dataMutex);
                m_rawDataPoints = std::move(measurement);
                m_newImport = true;
            }
        }
    });
}

void HokuyoLidarImporter::prepRawData() {
    data->points().clear();
    data->points().resize(m_rawDataPoints.size());
    for (std::vector<long>::size_type i = 0; i < m_rawDataPoints.size(); i++) {
        long l = m_rawDataPoints[i];
        double radian = m_lidar.index2rad(i);
        long x = l * cos(radian);
        long y = l * sin(radian);
        data->points().push_back(lms::math::vertex2f(x, y) / 1000 +
                                 m_offsetFromOrigin);
    }
}
