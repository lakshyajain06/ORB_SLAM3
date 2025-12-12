#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include "ndarray_converter.h"
#include <opencv2/core/core.hpp>
#include "System.h"
#include "ImuTypes.h"

namespace py = pybind11;

// --- HELPERS: CONVERT NUMPY <-> CV::MAT ---

// 1. Python Image (Numpy) -> C++ Image (cv::Mat)
cv::Mat numpy_to_cvmat(py::array_t<unsigned char>& input) {
    py::buffer_info buf = input.request();
    cv::Mat mat(buf.shape[0], buf.shape[1], CV_8UC3, (unsigned char*)buf.ptr);
    return mat.clone();
}

// 2. C++ Pose (cv::Mat) -> Python Matrix (Numpy)
py::array_t<float> cvmat_to_numpy(cv::Mat& mat) {
    if(mat.empty()) return py::array_t<float>(); 
    mat.convertTo(mat, CV_32F); 
    return py::array_t<float>(
        {mat.rows, mat.cols},
        {mat.cols * sizeof(float), sizeof(float)},
        (float*)mat.data
    );
}

// 3. Python List -> ORB_SLAM3 IMU Point
ORB_SLAM3::IMU::Point list_to_imupoint(const std::vector<double>& d) {
    // Expects [ax, ay, az, gx, gy, gz, timestamp]
    return ORB_SLAM3::IMU::Point((float)d[0], (float)d[1], (float)d[2], 
                                 (float)d[3], (float)d[4], (float)d[5], d[6]);
}

// --- MAIN PYTHON MODULE ---

PYBIND11_MODULE(orbslam, m) {
    m.doc() = "ORB-SLAM3 Python Wrapper";

    // Expose Sensor Enum
    py::enum_<ORB_SLAM3::System::eSensor>(m, "Sensor")
        .value("MONOCULAR", ORB_SLAM3::System::MONOCULAR)
        .value("IMU_MONOCULAR", ORB_SLAM3::System::IMU_MONOCULAR)
        .export_values();

    // Expose System Class
    py::class_<ORB_SLAM3::System>(m, "System")
        .def(py::init<const std::string&, const std::string&, ORB_SLAM3::System::eSensor, bool>(),
             py::arg("vocab_file"), py::arg("settings_file"), py::arg("sensor"), py::arg("bUseViewer") = true)

        // //TRACKING: Pure Monocular (No IMU)
        // .def("track_monocular", [](ORB_SLAM3::System& self, py::array_t<unsigned char>& img, double time) {
        //     cv::Mat cv_img = numpy_to_cvmat(img);
        //     // cv::Mat pose = self.TrackMonocular(cv_img, time); // Call C++
        //     // Capture as Sophus object
        //     Sophus::SE3f T_cw = self.TrackMonocular(cv_img, time);

        //     // Convert to Eigen Matrix (Pybind11 converts this to NumPy automatically)
        //     Eigen::Matrix4f pose = T_cw.matrix(); 

        //     return pose;
        //     // return cvmat_to_numpy(pose); // Return Numpy
        // })

        .def("TrackMonocular", [](ORB_SLAM3::System& self, const cv::Mat &im, const double &timestamp) {
            std::vector<ORB_SLAM3::IMU::Point> vImuMeas; 
            std::string filename = ""; 

            // 1. Get the sophisticated object
            Sophus::SE3f T_cw = self.TrackMonocular(im, timestamp, vImuMeas, filename);
            
            auto t = T_cw.translation();
            
            // std::cout << "--------------------------------" << std::endl << std::endl;
            // std::cout << std::fixed << std::setprecision(4); // Formatting
            // std::cout << "[C++ DEBUG] TS: " << timestamp 
            //           << " | Pos: [" << t.x() << ", " << t.y() << ", " << t.z() << "]";

            // std::cout << "--------------------------------" << std::endl << std::endl;

            // 2. Convert manually to a flat vector (Safe & Simple)
            Eigen::Matrix4f m = T_cw.matrix();
            std::vector<float> pose_data;
            pose_data.reserve(16);
            
            // Row-Major flattened order
            for(int r=0; r<4; r++){
                for(int c=0; c<4; c++){
                    pose_data.push_back(m(r,c));
                }
        }

        // Return simple vector (Python list)
        return pose_data; 
    })

        // TRACKING: Monocular-Inertial
        .def("TrackMonocularInertial", [](ORB_SLAM3::System& self, py::array_t<unsigned char>& img, double time, const std::vector<std::vector<double>>& imu_data) {
            std::vector<ORB_SLAM3::IMU::Point> vImu;
            for(auto& row : imu_data) vImu.push_back(list_to_imupoint(row));
            
            cv::Mat cv_img = numpy_to_cvmat(img);
            // ORB-SLAM3 usually overloads TrackMonocular to accept IMU data
            // cv::Mat pose = self.TrackMonocular(cv_img, time, vImu); 
            // Capture as Sophus object
            Sophus::SE3f T_cw = self.TrackMonocular(cv_img, time, vImu);

            // Convert to Eigen Matrix (Pybind11 converts this to NumPy automatically)
            Eigen::Matrix4f pose = T_cw.matrix(); 

            return pose;
            // return cvmat_to_numpy(pose);
        })

        // SAVING: save pointclud
        .def("SavePointCloud", [](ORB_SLAM3::System& self, const std::string &folder) {
            self.SavePointCloud(folder);
        })

        // // MAP MANAGEMENT
        // .def("save_atlas", [](ORB_SLAM3::System& self, int type) {
        //     self.SaveAtlas(type); // Saves as 'Atlas.osa' usually
        // })
        // .def("load_atlas", [](ORB_SLAM3::System& self, const std::string& filename, int type) {
        //     self.LoadAtlas(filename, type);
        // })

        // MODES & SHUTDOWN
        .def("ActivateLocalization", &ORB_SLAM3::System::ActivateLocalizationMode)
        .def("DeactivateLocalization", &ORB_SLAM3::System::DeactivateLocalizationMode)
        .def("Shutdown", &ORB_SLAM3::System::Shutdown)
        .def("GetTrackingState", &ORB_SLAM3::System::GetTrackingState);
}