#ifndef HOKUYO_LIDAR_IMPORTER_H
#define HOKUYO_LIDAR_IMPORTER_H

#include <atomic>
#include <thread>
#include <vector>

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
    void startImporterThread();
    void prepRawData();

    qrk::Urg_driver m_lidar;
    std::vector<long> m_rawDataPoints;
    bool m_newImport;

    ////////////////////////////// Data Channels ///////////////////////////////
    lms::WriteDataChannel<lms::math::PointCloud2f> data;
    lms::WriteDataChannel<bool> newData;

    /////////////////////////////// Config Values //////////////////////////////
    bool m_configsChanged;
    lms::math::vertex2f m_offsetFromOrigin;
    double m_startAtDeg;
    double m_stopAtDeg;

    /////////////////////////////// Thread & Mutex /////////////////////////////
    std::mutex m_dataMutex;
    std::mutex m_lidarMutex;
    std::thread m_importer;
    std::atomic_bool a_importerRunning;
};

#endif  // HOKUYO_LIDAR_IMPORTER_H
