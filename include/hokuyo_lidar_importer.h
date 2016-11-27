#ifndef HOKUYO_LIDAR_IMPORTER_H
#define HOKUYO_LIDAR_IMPORTER_H

#include <vector>
#include <future>

#include <cpp/Urg_driver.h>
#include <lms/math/point_cloud.h>
#include <lms/math/vertex.h>
#include <lms/module.h>

class HokuyoLidarImporter : public lms::Module {
   public:
    bool initialize();
    bool deinitialize();
    void configsChanged() override;
    bool cycle();

   private:
    std::vector<long> importRawData();
    std::vector<lms::math::vertex2f> prepRawData(const std::vector<long>& rawData);
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
