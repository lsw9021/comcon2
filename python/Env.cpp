#include "Env.h"
PYBIND11_MODULE(pycomcon, m){
	py::class_<Environment>(m, "env")
		.def(py::init<>())
		.def("get_dim_state",&Environment::getDimState)
		.def("get_dim_state_amp",&Environment::getDimStateAMP)
		.def("get_dim_action",&Environment::getDimAction)
		.def("reset",&Environment::reset)
		.def("step",&Environment::step)
		.def("eoe",&Environment::eoe)
		.def("verbose",&Environment::verbose)
		.def("get_state_amp_experts",&Environment::getStateAMPExperts)
		.def("get_reward_position",&Environment::getRewardPosition)
		.def("get_state",&Environment::getState)
		.def("get_state_amp",&Environment::getStateAMP);
}