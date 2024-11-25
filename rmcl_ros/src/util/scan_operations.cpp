#include "rmcl_ros/util/scan_operations.h"
#include <rmcl_ros/util/conversions.h>

namespace rmcl
{

void fill(
    rmcl_msgs::msg::Scan& scan, 
    const rmagine::Memory<float, rmagine::RAM>& ranges)
{
    rmagine::SphericalModel model;
    convert(scan.info, model);
    
    scan.data.ranges.resize(ranges.size());
    for(unsigned int vid = 0; vid < model.getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < model.getWidth(); hid++)
        {
            const unsigned int loc_id = model.getBufferId(vid, hid);
            scan.data.ranges[loc_id] = ranges[loc_id];
        }
    }
}

void fillEmpty(rmcl_msgs::msg::Scan& scan)
{
    rmagine::SphericalModel model;
    convert(scan.info, model);
    
    scan.data.ranges.resize(model.size());
    for(unsigned int vid = 0; vid < model.getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < model.getWidth(); hid++)
        {
            const unsigned int loc_id = model.getBufferId(vid, hid);
            scan.data.ranges[loc_id] = scan.info.range_max + 1.0;
        }
    }
}

} // namespace rmcl