#ifndef HOKUYO_LIDAR_IMPORTER_H
#define HOKUYO_LIDAR_IMPORTER_H

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
    void printLidarData(const std::vector<long>& data);

    lms::WriteDataChannel<lms::math::PointCloud2f> data;

    qrk::Urg_driver m_lidar;

    lms::math::vertex2f m_offsetFromOrigin;
    double m_startAtDeg;
    double m_stopAtDeg;
};

#endif  // HOKUYO_LIDAR_IMPORTER_H
