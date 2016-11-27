#ifndef HOKUYO_LIDAR_IMPORTER_H
#define HOKUYO_LIDAR_IMPORTER_H

#include <future>
#include <vector>

#include <cpp/Urg_driver.h>
#include <lms/math/point_cloud.h>
#include <lms/math/vertex.h>
#include <lms/module.h>

/**
 * @brief LMS module HokuyoLidarImporter
 * Imports data from a Hokuyo URG-04LX-UG01 LIDAR. The data points are
 * transformed into vertices in the car cartesian plane and written to a data
 * channel as a 2d point cloud. The import is running asynchronously and cycles
 * without new data are NOP.
 */
class HokuyoLidarImporter : public lms::Module {
   public:
    bool initialize();
    bool deinitialize();
    void configsChanged() override;
    bool cycle();

   private:
    std::vector<long> importRawData();
    std::vector<lms::math::vertex2f> prepRawData(
        const std::vector<long>& rawData);
    void setConfigMembers();

    qrk::Urg_driver m_lidar;
    std::future<std::vector<long>> m_rawDataFuture;

    ////////////////////////////// Data Channels ///////////////////////////////
    lms::WriteDataChannel<lms::math::PointCloud2f> data;
    lms::WriteDataChannel<bool> newData;

    /////////////////////////////// Config Values //////////////////////////////
    bool m_configsChanged;
    lms::math::vertex2f m_offsetFromOrigin;
    double m_startAtDeg;
    double m_stopAtDeg;
};

#endif  // HOKUYO_LIDAR_IMPORTER_H
