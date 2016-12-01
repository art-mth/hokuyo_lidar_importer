#include "hokuyo_lidar_importer.h"

#include <math.h>
#include <chrono>
#include <utility>

namespace {
const char* kDeviceFile = "/dev/ttyACM0";
const long kBaudrate = 115200;

template <typename R>
bool isReady(const std::future<R>& f) {
    return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}
}

bool HokuyoLidarImporter::initialize() {
    data = writeChannel<lms::math::PointCloud2f>("HOKUYO_LIDAR_DATA");
    newData = writeChannel<bool>("NEW_DATA");

    setConfigMembers();
    m_configsChanged = false;

    if (!m_lidar.open(kDeviceFile, kBaudrate, qrk::Urg_driver::Serial)) {
        logger.error() << "Failed opening Hokuyo Lidar at " << kDeviceFile
                       << " error: " << m_lidar.what();
        return false;
    }
    m_lidar.set_scanning_parameter(m_lidar.deg2step(m_startAtDeg),
                                   m_lidar.deg2step(m_stopAtDeg), 0);
    m_lidar.start_measurement();

    m_rawDataFuture = std::async(std::launch::async,
                                 &HokuyoLidarImporter::importRawData, this);
    return true;
}

bool HokuyoLidarImporter::deinitialize() {
    m_rawDataFuture.wait();
    m_lidar.stop_measurement();
    return true;
}

void HokuyoLidarImporter::configsChanged() {
    setConfigMembers();
    m_configsChanged = true;
}

bool HokuyoLidarImporter::cycle() {
    if (!isReady(m_rawDataFuture)) {
        *newData = false;
        return true;
    }
    *newData = true;
    data->points(prepRawData(m_rawDataFuture.get()));

    if (m_configsChanged) {
        m_lidar.set_scanning_parameter(m_lidar.deg2step(m_startAtDeg),
                                       m_lidar.deg2step(m_stopAtDeg), 0);
    }
    m_rawDataFuture = std::async(std::launch::async,
                                 &HokuyoLidarImporter::importRawData, this);
    return true;
}

std::vector<long> HokuyoLidarImporter::importRawData() {
    std::vector<long> measurement;
    if (!m_lidar.get_distance(measurement)) {
        logger.error() << "Failed getting distance from Hokuyo Lidar error: "
                       << m_lidar.what();
    }
    return measurement;
}

std::vector<lms::math::vertex2f> HokuyoLidarImporter::prepRawData(
    const std::vector<long>& rawData) {
    std::vector<lms::math::vertex2f> data;
    for (std::vector<long>::size_type i = 0; i < rawData.size(); i++) {
        long distance = rawData[i];
        double radian = m_lidar.index2rad(i);
        float x = static_cast<float>(distance * cos(radian));
        float y = static_cast<float>(distance * sin(radian));
        data.push_back(lms::math::vertex2f(x, y) / 1000 + m_offsetFromOrigin);
    }
    return data;
}

void HokuyoLidarImporter::setConfigMembers() {
    m_offsetFromOrigin.x = config().get<float>("xOffsetFromOriginMeter", 0);
    m_offsetFromOrigin.y = config().get<float>("yOffsetFromOriginMeter", 0);
    m_startAtDeg = config().get<double>("startAtDeg", -90);
    m_stopAtDeg = config().get<double>("stopAtDeg", 90);
}
