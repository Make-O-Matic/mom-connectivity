#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include "glove.h"

PYBIND11_PLUGIN(glove) {
    pybind11::module m("glove", "Glove communication and DB write");

    pybind11::class_<Glove>(m, "Glove", pybind11::dynamic_attr())
            .def(pybind11::init<const std::string&, const std::string&,
                 const std::function<void(int)>&,
                 const std::function<bool()>&, 
                 const std::string&, const std::string&>())
            .def("connect", &Glove::connect)
            .def("disconnect", &Glove::disconnect)   
            .def("now", &Glove::now)                        
            .def("setTrainsetExercise", &Glove::setTrainsetExercise);

    return m.ptr();
}
