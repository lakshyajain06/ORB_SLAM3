#ifndef NDARRAY_CONVERTER_H
#define NDARRAY_CONVERTER_H

#include <opencv2/core/core.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

// This class converts Python NumPy arrays to C++ cv::Mat
class NDArrayConverter {
public:
    static bool toMat(py::object o, cv::Mat &m) {
        // Check if the input is actually a numpy array
        if(!py::isinstance<py::array>(o)) return false;
        
        // Get the array information (pointers, shape, etc)
        py::array a = py::reinterpret_borrow<py::array>(o);
        py::buffer_info info = a.request();
        
        int type = -1;
        // Determine OpenCV type based on numpy format
        if (info.format == py::format_descriptor<float>::format()) {
            if (info.ndim == 3) type = CV_32FC3;
            else if (info.ndim == 2) type = CV_32FC1;
        } else if (info.format == py::format_descriptor<unsigned char>::format()) {
            if (info.ndim == 3) type = CV_8UC3;
            else if (info.ndim == 2) type = CV_8UC1;
        }
        
        if (type == -1) {
            // If unknown type, throw error or return false
            return false; 
        }
        
        // Construct the cv::Mat using the pointer to the numpy memory
        m = cv::Mat(info.shape[0], info.shape[1], type, info.ptr, info.strides[0]);
        return true;
    }
};

// This tells Pybind11: "Whenever you see a cv::Mat in C++, look for a Numpy array in Python"
namespace pybind11 { namespace detail {
template <> struct type_caster<cv::Mat> {
public:
    PYBIND11_TYPE_CASTER(cv::Mat, _("numpy.ndarray"));

    // Conversion Python -> C++
    bool load(handle src, bool) {
        return NDArrayConverter::toMat(reinterpret_borrow<object>(src), value);
    }

    // Conversion C++ -> Python (not strictly needed for input only, but good to have)
    static handle cast(const cv::Mat &m, return_value_policy, handle) {
        return py::none(); // Placeholder
    }
};
}} 
#endif