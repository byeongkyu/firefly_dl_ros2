#include <thread>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <sensor_msgs/fill_image.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>


class FireflyDLDriver: public rclcpp::Node
{
    public:
        FireflyDLDriver(): Node("firefly_dl_driver")
        {
            system_ = Spinnaker::System::GetInstance();
            const Spinnaker::LibraryVersion spinnakerLibraryVersion = system_->GetLibraryVersion();
            RCLCPP_INFO(this->get_logger(), "Spinnaker library version: %d.%d.%d.%d",
                    spinnakerLibraryVersion.major, spinnakerLibraryVersion.minor,
                    spinnakerLibraryVersion.type, spinnakerLibraryVersion.build);

            int param_serial_number = 0;
            Spinnaker::CameraList cameraList = system_->GetCameras();
            unsigned int numCameras = cameraList.GetSize();
            RCLCPP_INFO(this->get_logger(), "Found %d camera(s)", numCameras);
            assert(numCameras != 0);

            // Now, use first index camera only
            if(param_serial_number !=0)
                camera_ = cameraList.GetBySerial(std::to_string(param_serial_number));
            else
                camera_ = cameraList.GetByIndex(0);

            // Spinnaker::GenApi::INodeMap& nodeMapTLDevice = camera_->GetTLDeviceNodeMap();
            // int result = printDeviceInfo(nodeMapTLDevice);

            serial_num_ = this->declare_parameter<int>("serial_number", 0);
            camera_name_ = this->declare_parameter<std::string>("camera_name", "camera");
            frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_link");


            // ROS2 related
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
            camera_info_pub_ = image_transport::create_camera_publisher(this, camera_name_ + "/image_raw", custom_qos_profile);

            cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
            auto camera_calibration_file_param_ = this->declare_parameter("camera_calibration_file", "file:///home/byeongkyu/config/"+ camera_name_ + ".yaml");
            cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);

            camera_->Init();

            // Get Camera Info
            node_map_ = &camera_->GetNodeMap();
            Spinnaker::GenApi::CStringPtr model_name = node_map_->GetNode("DeviceModelName");
            RCLCPP_INFO(this->get_logger(), "Device Model Name: %s", model_name->ToString().c_str());
            Spinnaker::GenApi::CStringPtr serial_number = node_map_->GetNode("DeviceSerialNumber");
            RCLCPP_INFO(this->get_logger(), "Device Serial Number: %s", serial_number->ToString().c_str());


            // Set Camera Parameter
            Spinnaker::GenApi::CEnumerationPtr ptrGainAuto = camera_->GetNodeMap().GetNode("GainAuto");
            Spinnaker::GenApi::CEnumEntryPtr ptrGainAutoOff = ptrGainAuto->GetEntryByName("Off");
            ptrGainAuto->SetIntValue(ptrGainAutoOff->GetValue());

            Spinnaker::GenApi::CFloatPtr ptrGain = camera_->GetNodeMap().GetNode("Gain");
            ptrGain->SetValue(2.0);

            Spinnaker::GenApi::CEnumerationPtr ptrBalanceWhiteAuto = camera_->GetNodeMap().GetNode("BalanceWhiteAuto");
            Spinnaker::GenApi::CEnumEntryPtr ptrBalanceWhiteAutoOff = ptrBalanceWhiteAuto->GetEntryByName("Off");
            ptrBalanceWhiteAuto->SetIntValue(ptrBalanceWhiteAutoOff->GetValue());

            Spinnaker::GenApi::CFloatPtr ptrBalanceRatio = camera_->GetNodeMap().GetNode("BalanceRatio");
            ptrBalanceRatio->SetValue(1.8);

            Spinnaker::GenApi::CEnumerationPtr ptrExposureAuto = camera_->GetNodeMap().GetNode("ExposureAuto");
            Spinnaker::GenApi::CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
            ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());

            Spinnaker::GenApi::CFloatPtr ptrExposureTime = camera_->GetNodeMap().GetNode("ExposureTime");
            ptrExposureTime->SetValue(16000);

            try
            {
                camera_->BeginAcquisition();
            }
            catch (const Spinnaker::Exception& e)
            {
                throw std::runtime_error("[SpinnakerCamera::start] Failed to start capture with error: " + std::string(e.what()));
            }

            thread_camera_ = std::make_shared<std::thread>(std::bind(&FireflyDLDriver::thread_camera_func, this));
            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }
        ~FireflyDLDriver()
        {
            camera_->EndAcquisition();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            system_->ReleaseInstance();
            thread_camera_->join();
        }

    private:
        void thread_camera_func(void)
        {
            while(rclcpp::ok())
            {
                Spinnaker::ImagePtr image_ptr = camera_->GetNextImage();
                Spinnaker::ImageStatus imageStatus = image_ptr->GetImageStatus();

                if(imageStatus != Spinnaker::IMAGE_NO_ERROR)
                    RCLCPP_WARN(this->get_logger(),  "Image Error");;

                //?? image_ptr -> OpenCV Image
                // image transport publish

                sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image());

                msg->header.stamp.sec = image_ptr->GetTimeStamp() * 1e-9;
                msg->header.stamp.nanosec = image_ptr->GetTimeStamp();

                size_t bitsPerPixel = image_ptr->GetBitsPerPixel();
                std::string imageEncoding = sensor_msgs::image_encodings::MONO8;

                Spinnaker::GenApi::CEnumerationPtr color_filter_ptr =
                    static_cast<Spinnaker::GenApi::CEnumerationPtr>(node_map_->GetNode("PixelColorFilter"));

                Spinnaker::GenICam::gcstring color_filter_str = color_filter_ptr->ToString();
                Spinnaker::GenICam::gcstring bayer_rg_str = "BayerRG";
                Spinnaker::GenICam::gcstring bayer_gr_str = "BayerGR";
                Spinnaker::GenICam::gcstring bayer_gb_str = "BayerGB";
                Spinnaker::GenICam::gcstring bayer_bg_str = "BayerBG";

                if (color_filter_ptr->GetCurrentEntry() != color_filter_ptr->GetEntryByName("None"))
                {
                    if (bitsPerPixel == 16)
                    {
                        // 16 Bits per Pixel
                        if (color_filter_str.compare(bayer_rg_str) == 0)
                            imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB16;
                        else if (color_filter_str.compare(bayer_gr_str) == 0)
                            imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG16;
                        else if (color_filter_str.compare(bayer_gb_str) == 0)
                            imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG16;
                        else if (color_filter_str.compare(bayer_bg_str) == 0)
                            imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR16;
                        else
                            throw std::runtime_error("[SpinnakerCamera::grabImage] Bayer format not recognized for 16-bit format.");
                    }
                    else
                    {
                        // 8 Bits per Pixel
                        if (color_filter_str.compare(bayer_rg_str) == 0)
                            imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
                        else if (color_filter_str.compare(bayer_gr_str) == 0)
                            imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG8;
                        else if (color_filter_str.compare(bayer_gb_str) == 0)
                            imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;
                        else if (color_filter_str.compare(bayer_bg_str) == 0)
                            imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR8;
                        else
                            throw std::runtime_error("[SpinnakerCamera::grabImage] Bayer format not recognized for 8-bit format.");
                    }
                }
                else  // Mono camera or in pixel binned mode.
                {
                    if (bitsPerPixel == 16)
                        imageEncoding = sensor_msgs::image_encodings::MONO16;
                    else if (bitsPerPixel == 24)
                        imageEncoding = sensor_msgs::image_encodings::RGB8;
                    else
                        imageEncoding = sensor_msgs::image_encodings::MONO8;
                }

                int width = image_ptr->GetWidth();
                int height = image_ptr->GetHeight();
                int stride = image_ptr->GetStride();

                sensor_msgs::fillImage(*msg, imageEncoding, height, width, stride, image_ptr->GetData());
                msg->header.frame_id = frame_id_;

                sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_(new sensor_msgs::msg::CameraInfo(cinfo_manager_->getCameraInfo()));
                camera_info_msg_->header.stamp = msg->header.stamp;

                camera_info_pub_.publish(msg, camera_info_msg_);
            }
        }

    private:
        int printDeviceInfo(Spinnaker::GenApi::INodeMap& node)
        {
            int result = 0;
            try
            {
                Spinnaker::GenApi::FeatureList_t features;
                Spinnaker::GenApi::CCategoryPtr category = node.GetNode("DeviceInformation");
                if (Spinnaker::GenApi::IsAvailable(category) && Spinnaker::GenApi::IsReadable(category))
                {
                    category->GetFeatures(features);

                    Spinnaker::GenApi::FeatureList_t::const_iterator it;
                    for (it = features.begin(); it != features.end(); ++it)
                    {
                        Spinnaker::GenApi::CNodePtr pfeatureNode = *it;
                        std::cout << pfeatureNode->GetName() << " : ";
                        Spinnaker::GenApi::CValuePtr pValue = (Spinnaker::GenApi::CValuePtr)pfeatureNode;
                        std::cout << (Spinnaker::GenApi::IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                        std::cout << std::endl;
                    }
                }
                else
                {
                    std::cout << "Device control information not available." << std::endl;
                }
            }
            catch (Spinnaker::Exception& e)
            {
                std::cout << "Error: " << e.what() << std::endl;
                result = -1;
            }

            return result;
        }


    private:
        Spinnaker::SystemPtr system_;
        Spinnaker::CameraPtr camera_;
        Spinnaker::GenApi::INodeMap* node_map_;
        std::shared_ptr<std::thread> thread_camera_;
        std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
        image_transport::CameraPublisher camera_info_pub_;

        int serial_num_;
        std::string camera_name_;
        std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FireflyDLDriver>());
    rclcpp::shutdown();
    return 0;
}