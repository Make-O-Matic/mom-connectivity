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
            .def("connect", [](Glove &glove){
				pybind11::gil_scoped_release release;
				glove.connect();
			})
            .def("disconnect", [](Glove &glove){
				pybind11::gil_scoped_release release;
				glove.disconnect();
			})   
            .def("now", &Glove::now)                        
            .def("setTrainsetExercise", [](Glove &glove, const std::string &trainset,
        const int step, const std::string &mutation, const std::string &mutationIndex){
				pybind11::gil_scoped_release release;
				glove.setTrainsetExercise(trainset, step, mutation, mutationIndex);
			});

    return m.ptr();
}
