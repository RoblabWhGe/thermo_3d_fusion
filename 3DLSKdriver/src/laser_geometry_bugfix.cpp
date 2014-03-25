#ifndef laser_geometry_bugfix
#define laser_geometry_bugfix
using namespace laser_geometry;

void projectLaser_bugfix (const sensor_msgs::LaserScan& scan_in,
                                      sensor_msgs::PointCloud2 &cloud_out,
                                      double range_cutoff,
                                      int channel_options);

void transformLaserScanToPointCloud_bugfix (const std::string &target_frame,
                                                        const sensor_msgs::LaserScan &scan_in,
                                                        sensor_msgs::PointCloud2 &cloud_out,
                                                        tf::Transformer &tf,
                                                        double range_cutoff,
                                                        int channel_options)
  {
    //check if the user has requested the index field
    bool requested_index = false;
    if ((channel_options & channel_option::Index))
      requested_index = true;

    //we'll enforce that we get index values for the laser scan so that we
    //ensure that we use the correct timestamps
    channel_options |= channel_option::Index;

    projectLaser_bugfix(scan_in, cloud_out, -1.0, channel_options);

    //ROS_INFO("Cloud has size: %d", cloud_out.width);

    //we'll assume no associated viewpoint by default
    bool has_viewpoint = false;
    uint32_t vp_x_offset = 0;

    //we need to find the offset of the intensity field in the point cloud
    //we also know that the index field is guaranteed to exist since we
    //set the channel option above. To be really safe, it might be worth
    //putting in a check at some point, but I'm just going to put in an
    //assert for now
    uint32_t index_offset = 0;
    for(unsigned int i = 0; i < cloud_out.fields.size(); ++i)
    {
      if(cloud_out.fields[i].name == "index")
      {
        index_offset = cloud_out.fields[i].offset;
      }

      //we want to check if the cloud has a viewpoint associated with it
      //checking vp_x should be sufficient since vp_x, vp_y, and vp_z all
      //get put in together
      if(cloud_out.fields[i].name == "vp_x")
      {
        has_viewpoint = true;
        vp_x_offset = cloud_out.fields[i].offset;
      }
    }

    ROS_ASSERT(index_offset > 0);

    cloud_out.header.frame_id = target_frame;

    // Extract transforms for the beginning and end of the laser scan
    ros::Time start_time = scan_in.header.stamp;
    ros::Time end_time   = scan_in.header.stamp + ros::Duration ().fromSec (scan_in.ranges.size () * scan_in.time_increment);

    tf::StampedTransform start_transform, end_transform, cur_transform ;

    tf.waitForTransform(target_frame, scan_in.header.frame_id, start_time, ros::Duration(1.0));
    tf.lookupTransform (target_frame, scan_in.header.frame_id, start_time, start_transform);
    tf.waitForTransform(target_frame, scan_in.header.frame_id, end_time, ros::Duration(1.0));
    tf.lookupTransform (target_frame, scan_in.header.frame_id, end_time, end_transform);

    double ranges_norm = 1 / ((double) scan_in.ranges.size () - 1.0);

    //we want to loop through all the points in the cloud
    for(size_t i = 0; i < cloud_out.width; ++i)
    {
      // Apply the transform to the current point
      float *pstep = (float*)&cloud_out.data[i * cloud_out.point_step + 0];

      //find the index of the point
      uint32_t pt_index;
      memcpy(&pt_index, &cloud_out.data[i * cloud_out.point_step + index_offset], sizeof(uint32_t));

      // Assume constant motion during the laser-scan, and use slerp to compute intermediate transforms
      tfScalar ratio = pt_index * ranges_norm;

      //! \todo Make a function that performs both the slerp and linear interpolation needed to interpolate a Full Transform (Quaternion + Vector)
      // Interpolate translation
      tf::Vector3 v (0, 0, 0);
      v.setInterpolate3 (start_transform.getOrigin (), end_transform.getOrigin (), ratio);
      cur_transform.setOrigin (v);

      // Interpolate rotation
      tf::Quaternion q1, q2;
      start_transform.getBasis ().getRotation (q1);
      end_transform.getBasis ().getRotation (q2);

      // Compute the slerp-ed rotation
      cur_transform.setRotation (slerp (q1, q2 , ratio));

      tf::Vector3 point_in (pstep[0], pstep[1], pstep[2]);
      tf::Vector3 point_out = cur_transform * point_in;

      // Copy transformed point into cloud
      pstep[0] = point_out.x ();
      pstep[1] = point_out.y ();
      pstep[2] = point_out.z ();

      // Convert the viewpoint as well
      if(has_viewpoint)
      {
        float *vpstep = (float*)&cloud_out.data[i * cloud_out.point_step + vp_x_offset];
        point_in = tf::Vector3 (vpstep[0], vpstep[1], vpstep[2]);
        point_out = cur_transform * point_in;

        // Copy transformed point into cloud
        vpstep[0] = point_out.x ();
        vpstep[1] = point_out.y ();
        vpstep[2] = point_out.z ();
      }
    }

    //if the user didn't request the index field, then we need to copy the PointCloud and drop it
    if(!requested_index)
    {
      sensor_msgs::PointCloud2 cloud_without_index;

      //copy basic meta data
      cloud_without_index.header = cloud_out.header;
      cloud_without_index.width = cloud_out.width;
      cloud_without_index.height = cloud_out.height;
      cloud_without_index.is_bigendian = cloud_out.is_bigendian;
      cloud_without_index.is_dense = cloud_out.is_dense;

      //copy the fields
      cloud_without_index.fields.resize(cloud_out.fields.size());
      unsigned int field_count = 0;
      unsigned int offset_shift = 0;
      for(unsigned int i = 0; i < cloud_out.fields.size(); ++i)
      {
        if(cloud_out.fields[i].name != "index")
        {
          cloud_without_index.fields[field_count] = cloud_out.fields[i];
          cloud_without_index.fields[field_count].offset -= offset_shift;
          ++field_count;
        }
        else
        {
          //once we hit the index, we'll set the shift
          offset_shift = 4;
        }
      }

      //resize the fields
      cloud_without_index.fields.resize(field_count);

      //compute the size of the new data
      cloud_without_index.point_step = cloud_out.point_step - offset_shift;
      cloud_without_index.row_step   = cloud_without_index.point_step * cloud_without_index.width;
      cloud_without_index.data.resize (cloud_without_index.row_step   * cloud_without_index.height);

      uint32_t i = 0;
      uint32_t j = 0;
      //copy over the data from one cloud to the other
      while (i < cloud_out.data.size())
      {
        if((i % cloud_out.point_step) < index_offset || (i % cloud_out.point_step) >= (index_offset + 4))
        {
          cloud_without_index.data[j++] = cloud_out.data[i];
        }
        i++;
      }

      //make sure to actually set the output
      cloud_out = cloud_without_index;
    }
  }

void projectLaser_bugfix (const sensor_msgs::LaserScan& scan_in,
                                      sensor_msgs::PointCloud2 &cloud_out,
                                      double range_cutoff,
                                      int channel_options)
  {
    size_t n_pts = scan_in.ranges.size ();
    Eigen::ArrayXXd ranges (n_pts, 2);
    Eigen::ArrayXXd output (n_pts, 2);

    // Get the ranges into Eigen format
    for (size_t i = 0; i < n_pts; ++i)
    {
      ranges (i, 0) = (double) scan_in.ranges[i];
      ranges (i, 1) = (double) scan_in.ranges[i];
    }

    static Eigen::ArrayXXd co_sine_map_; //Workaround because of bugfix is member of the class
    static float angle_min_ = 0;
    static float angle_max_ = 0;

    // Check if our existing co_sine_map is valid
    if (co_sine_map_.rows () != (int)n_pts || angle_min_ != scan_in.angle_min || angle_max_ != scan_in.angle_max )
    {
      ROS_DEBUG ("[projectLaser] No precomputed map given. Computing one.");
      co_sine_map_ = Eigen::ArrayXXd (n_pts, 2);
      angle_min_ = scan_in.angle_min;
      angle_max_ = scan_in.angle_max;
      // Spherical->Cartesian projection
      for (size_t i = 0; i < n_pts; ++i)
      {
        co_sine_map_ (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
        co_sine_map_ (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
      }
    }

    output = ranges * co_sine_map_;

    // Set the output cloud accordingly
    cloud_out.header = scan_in.header;
    cloud_out.height = 1;
    cloud_out.width  = scan_in.ranges.size ();
    cloud_out.fields.resize (3);
    cloud_out.fields[0].name = "x";
    cloud_out.fields[0].offset = 0;
    cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_out.fields[1].name = "y";
    cloud_out.fields[1].offset = 4;
    cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_out.fields[2].name = "z";
    cloud_out.fields[2].offset = 8;
    cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;

    // Define 4 indices in the channel array for each possible value type
    int idx_intensity = -1, idx_index = -1, idx_distance = -1, idx_timestamp = -1, idx_vpx = -1, idx_vpy = -1, idx_vpz = -1;

    //now, we need to check what fields we need to store
    int offset = 12;
    if ((channel_options & channel_option::Intensity) && scan_in.intensities.size() > 0)
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "intensity";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      offset += 4;
      idx_intensity = field_size;
    }

    if ((channel_options & channel_option::Index))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "index";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::INT32;
      cloud_out.fields[field_size].offset = offset;
      offset += 4;
      idx_index = field_size;
    }

    if ((channel_options & channel_option::Distance))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "distances";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      offset += 4;
      idx_distance = field_size;
    }

    if ((channel_options & channel_option::Timestamp))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "stamps";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      offset += 4;
      idx_timestamp = field_size;
    }

    if ((channel_options & channel_option::Viewpoint))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 3);

      cloud_out.fields[field_size].name = "vp_x";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      offset += 4;

      cloud_out.fields[field_size + 1].name = "vp_y";
      cloud_out.fields[field_size + 1].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size + 1].offset = offset;
      offset += 4;

      cloud_out.fields[field_size + 2].name = "vp_z";
      cloud_out.fields[field_size + 2].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size + 2].offset = offset;
      offset += 4;

      idx_vpx = field_size;
      idx_vpy = field_size + 1;
      idx_vpz = field_size + 2;
    }

    cloud_out.point_step = offset;
    cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
    cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
    cloud_out.is_dense = false;

    //TODO: Find out why this was needed
    //float bad_point = std::numeric_limits<float>::quiet_NaN ();

    if (range_cutoff < 0)
      range_cutoff = scan_in.range_max;
    else
      range_cutoff = std::min(range_cutoff, (double)scan_in.range_max);

    unsigned int count = 0;
    for (size_t i = 0; i < n_pts; ++i)
    {
      //check to see if we want to keep the point
      if (scan_in.ranges[i] <= range_cutoff && (scan_in.ranges[i] >= scan_in.range_min || true)) //Just a test "true" NEEDS TO BE REMOVED
      {
        float *pstep = (float*)&cloud_out.data[count * cloud_out.point_step];

        // Copy XYZ
        pstep[0] = output (i, 0);
        pstep[1] = output (i, 1);
        pstep[2] = 0;

        // Copy intensity
        if(idx_intensity != -1)
          pstep[idx_intensity] = scan_in.intensities[i];

        //Copy index
        if(idx_index != -1)
          ((int*)(pstep))[idx_index] = i;

        // Copy distance
        if(idx_distance != -1)
          pstep[idx_distance] = scan_in.ranges[i];

        // Copy timestamp
        if(idx_timestamp != -1)
          pstep[idx_timestamp] = i * scan_in.time_increment;

        // Copy viewpoint (0, 0, 0)
        if(idx_vpx != -1 && idx_vpy != -1 && idx_vpz != -1)
        {
          pstep[idx_vpx] = 0;
          pstep[idx_vpy] = 0;
          pstep[idx_vpz] = 0;
        }

        //make sure to increment count
        ++count;
      }

      /* TODO: Why was this done in this way, I don't get this at all, you end up with a ton of points with NaN values
       * why can't you just leave them out?
       *
      // Invalid measurement?
      if (scan_in.ranges[i] >= range_cutoff || scan_in.ranges[i] <= scan_in.range_min)
      {
        if (scan_in.ranges[i] != LASER_SCAN_MAX_RANGE)
        {
          for (size_t s = 0; s < cloud_out.fields.size (); ++s)
            pstep[s] = bad_point;
        }
        else
        {
          // Kind of nasty thing:
          //   We keep the oringinal point information for max ranges but set x to NAN to mark the point as invalid.
          //   Since we still might need the x value we store it in the distance field
          pstep[0] = bad_point;           // X -> NAN to mark a bad point
          pstep[1] = co_sine_map (i, 1);  // Y
          pstep[2] = 0;                   // Z

          if (store_intensity)
          {
            pstep[3] = bad_point;           // Intensity -> NAN to mark a bad point
            pstep[4] = co_sine_map (i, 0);  // Distance -> Misused to store the originnal X
          }
          else
            pstep[3] = co_sine_map (i, 0);  // Distance -> Misused to store the originnal X
        }
      }
      */
    }

    //resize if necessary
    cloud_out.width = count;
    cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
    cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
  }

#endif
