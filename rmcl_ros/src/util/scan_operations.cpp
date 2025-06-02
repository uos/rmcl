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

void filter(
  rmcl_msgs::msg::O1Dn& o1dn_out,
  const rmcl_msgs::msg::O1Dn& o1dn_in,
  const FilterOptions2D& options)
{
  o1dn_out.info.width = (o1dn_in.info.width - options.width.skip_begin - options.width.skip_end) / options.width.increment;
  o1dn_out.info.height = (o1dn_in.info.height - options.height.skip_begin - options.height.skip_end) / options.height.increment;


  o1dn_out.info.orig = o1dn_in.info.orig;
  o1dn_out.info.dirs.resize(o1dn_in.info.width * o1dn_in.info.height);

  o1dn_out.info.range_min = std::max(o1dn_in.info.range_min, options.range_min);
  o1dn_out.info.range_max = std::min(o1dn_in.info.range_max, options.range_max);


  o1dn_out.data.ranges.resize(o1dn_out.info.width * o1dn_out.info.height);

  const bool has_mask = !o1dn_in.data.mask.empty();
  const bool has_normals = !o1dn_in.data.normals.empty();
  const bool has_colors = !o1dn_in.data.colors.empty();
  const bool has_stamps = !o1dn_in.data.stamps.empty();
  const bool has_intensities = !o1dn_in.data.intensities.empty();
  const bool has_labels = !o1dn_in.data.labels.empty();

  if(has_mask)
  {
    o1dn_out.data.mask.resize(o1dn_out.info.width * o1dn_out.info.height);
  }

  if(has_normals)
  {
    o1dn_out.data.normals.resize(o1dn_out.info.width * o1dn_out.info.height);
  }

  if(has_colors)
  {
    o1dn_out.data.colors.resize(o1dn_out.info.width * o1dn_out.info.height);
  }

  if(has_stamps)
  {
    o1dn_out.data.stamps.resize(o1dn_out.info.width * o1dn_out.info.height);
  }

  if(has_intensities)
  {
    o1dn_out.data.intensities.resize(o1dn_out.info.width * o1dn_out.info.height);
  }

  if(has_labels)
  {
    o1dn_out.data.labels.resize(o1dn_out.info.width * o1dn_out.info.height);
  }
  
  for(size_t tgt_i = 0; tgt_i < o1dn_out.info.height; tgt_i++)
  {
    const size_t src_i = tgt_i * options.height.increment + options.height.skip_begin;
    // const uint8_t* row = &pcd->data[src_i * pcd->row_step];

    for(size_t tgt_j = 0; tgt_j < o1dn_out.info.width; tgt_j++)
    {
      const size_t src_j = tgt_j * options.width.increment + options.width.skip_begin;
      // const uint8_t* data_ptr = &row[src_j * pcd->point_step];

      const size_t tgt_buf_id = tgt_i * o1dn_out.info.width + tgt_j;
      const size_t src_buf_id = src_i * o1dn_in.info.width  + src_j;

      o1dn_out.info.dirs[tgt_buf_id] = o1dn_in.info.dirs[src_buf_id];
      o1dn_out.data.ranges[tgt_buf_id] = o1dn_in.data.ranges[src_buf_id];

      if(has_mask)
      {
        o1dn_out.data.mask[tgt_buf_id] = o1dn_in.data.mask[src_buf_id];
      }

      if(has_normals)
      {
        o1dn_out.data.normals[tgt_buf_id] = o1dn_in.data.normals[src_buf_id];
      }

      if(has_colors)
      {
        o1dn_out.data.colors[tgt_buf_id] = o1dn_in.data.colors[src_buf_id];
      }

      if(has_stamps)
      {
        o1dn_out.data.stamps[tgt_buf_id] = o1dn_in.data.stamps[src_buf_id];
      }

      if(has_intensities)
      {
        o1dn_out.data.intensities[tgt_buf_id] = o1dn_in.data.intensities[src_buf_id];
      }

      if(has_labels)
      {
        o1dn_out.data.labels[tgt_buf_id] = o1dn_in.data.labels[src_buf_id];
      }
    }
  }
}

} // namespace rmcl