#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/cpu_packet_pipeline.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <limits>
#include <string>
#include <vector>
#include <memory>

int main() {
    libfreenect2::Freenect2 freenect2;

    if (freenect2.enumerateDevices() == 0) {
        std::cerr << "No Kinect device found." << std::endl;
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    // Use CPU pipeline
    std::unique_ptr<libfreenect2::PacketPipeline> pipeline(new libfreenect2::CpuPacketPipeline());

    libfreenect2::Freenect2Device* dev = freenect2.openDevice(serial, pipeline.get());
    if (!dev) {
        std::cerr << "Failed to open device." << std::endl;
        return -1;
    }

    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
    dev->setIrAndDepthFrameListener(&listener);

    if (!dev->startStreams(false, true)) {
        std::cerr << "Failed to start streams." << std::endl;
        return -1;
    }

    libfreenect2::Freenect2Device::IrCameraParams irp = dev->getIrCameraParams();
    int numFrames = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accumulated(new pcl::PointCloud<pcl::PointXYZ>());

    for (int f = 0; f < numFrames; ++f) {
        libfreenect2::FrameMap frames;
        listener.waitForNewFrame(frames);

        libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];
        const float* depth_data = reinterpret_cast<float*>(depth->data);

        for (int y = 0; y < 424; ++y) {
            for (int x = 0; x < 512; ++x) {
                size_t idx = y * 512 + x;
                float z = depth_data[idx];
                if (z <= 0.0f || z > 12.0f) continue;  // ignore invalid depth

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

    pcl::io::savePCDFileBinary("kinect_accumulated_1frame.pcd", *cloud_accumulated);

    dev->stop();
    dev->close();

    return 0;
}
