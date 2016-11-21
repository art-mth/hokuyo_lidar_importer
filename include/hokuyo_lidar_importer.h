#ifndef HOKUYO_LIDAR_IMPORTER_H
#define HOKUYO_LIDAR_IMPORTER_H

#include "lms/module.h"

class HokuyoLidarImporter : public lms::Module {
   public:
    bool initialize();
    bool deinitialize();
    void configsChanged() override;
    bool cycle();
};

#endif  // HOKUYO_LIDAR_IMPORTER_H
