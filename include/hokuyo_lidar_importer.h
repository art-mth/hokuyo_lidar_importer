#ifndef HOKUYO_LIDAR_IMPORTER_H
#define HOKUYO_LIDAR_IMPORTER_H

#include <vector>

#include <cpp/Urg_driver.h>
#include <lms/module.h>

class HokuyoLidarImporter : public lms::Module {
   public:
    bool initialize();
    bool deinitialize();
    void configsChanged() override;
    bool cycle();

   private:
    void printLidarData(const std::vector<long>& data);

    qrk::Urg_driver m_lidar;
};

#endif  // HOKUYO_LIDAR_IMPORTER_H
