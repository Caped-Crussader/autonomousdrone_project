#include <memory>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>

namespace py = pybind11;

#include "SdsShutterControl.h"

namespace Sds {
    
    class PyShutterControl : public IShutterControl {
    public:
        void updateShutter(uint8_t currentShutter, const std::vector<OpticalPeak>& peaks) override {
            PYBIND11_OVERRIDE_PURE(void, IShutterControl, updateShutter, currentShutter, peaks);
        };
    };

    PYBIND11_MODULE(pyShutterControl, g) {

        py::class_<OpticalPeak>(g, "OpticalPeak")
            .def(py::init<>())
            .def_readwrite("has_id", &OpticalPeak::has_id)
            .def_readwrite("id_duplicate", &OpticalPeak::id_duplicate)
            .def_readwrite("id", &OpticalPeak::id)
            .def_readwrite("height", &OpticalPeak::height)
            .def_readwrite("background", &OpticalPeak::background)
            .def_readwrite("carnea", &OpticalPeak::carnea);

        py::class_<IShutterControl, PyShutterControl, std::shared_ptr<IShutterControl>>(g, "IShutterControl")
            .def(py::init<>())
            .def("updateShutter", &IShutterControl::updateShutter)
            .def("setShutterAuto", &IShutterControl::setShutterAuto)
            .def("setShutterFixed", &IShutterControl::setShutterFixed)
            .def("registerSetShutterListener", &IShutterControl::registerSetShutterListener);
    }
}
