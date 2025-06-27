#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <limits>
#include <string>
#include <vector>

int main() {
    libfreenect2::Freenect2 freenect2;
    int deviceCount = freenect2.enumerateDevices();
    if (deviceCount == 0) return -1;

    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    libfreenect2::Freenect2Device* dev = freenect2.openDevice(serial);
    if (!dev) return -1;

    libfreenect2::Freenect2Device::Config config;
    config.EnableBilateralFilter = true;
    config.EnableEdgeAwareFilter = true;
    config.MinDepth = 0.5f;
    config.MaxDepth = 1.2f;
    dev->setConfiguration(config);

    auto pipeline = new libfreenect2::CpuPacketPipeline();
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
    dev->setIrAndDepthFrameListener(&listener);

    if (!dev->startStreams(false, true)) return -1;

    auto irp = dev->getIrCameraParams();
    int numFrames = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accumulated(new pcl::PointCloud<pcl::PointXYZ>());

    for (int f = 0; f < numFrames; ++f) {
        libfreenect2::FrameMap frames;
        listener.waitForNewFrame(frames);
        auto* depth = frames[libfreenect2::Frame::Depth];
        const float* depth_data = reinterpret_cast<float*>(depth->data);

        for (int y = 0; y < 424; ++y) {
            for (int x = 0; x < 512; ++x) {
                size_t idx = y * 512 + x;
                float z = depth_data[idx];
                if (z == 0) continue;

                pcl::PointXYZ pt;
                pt.x = (x - irp.cx) * z / irp.fx;
                pt.y = (y - irp.cy) * z / irp.fy;
                pt.z = z;
                cloud_accumulated->points.push_back(pt);
            }
        }

        listener.release(frames);
    }

    cloud_accumulated->width = cloud_accumulated->points.size();
    cloud_accumulated->height = 1;
    cloud_accumulated->is_dense = false;
    pcl::io::savePCDFileBinary("kinect_accumulated_30frames.pcd", *cloud_accumulated);

    dev->stop();
    dev->close();
    delete pipeline;

    return 0;
}
