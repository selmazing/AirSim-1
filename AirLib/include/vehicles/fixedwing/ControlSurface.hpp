#ifndef msr_airlib_controlsurface_hpp
#define msr_airlib_controlsurface_hpp

#include "common/Common.hpp"
#include <limits>
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "common/FirstOrderFilter.hpp"
#include "physics/PhysicsBodyVertex.hpp"
#include "AircraftParams.hpp"

namespace msr
{
	namespace airlib
	{
		/* Aircraft gets control deflection signal as input which changes the balance of forces on the aircraft*/
		class ControlSurface
		{
		public: //types
			struct Output
			{
				// control terms
				real_T control_signal_filtered; 
				real_T control_signal_input;
				real_T control_speed;
				real_T control_deflection;
			};
		public: //methods

			ControlSurface()
			{
				initialize();
			}
			void initialize()
			{
				air_density_sea_level_ = EarthUtils::getAirDensity(0.0f);

				// hardcoded time-constant for now, should probably have another hpp class with the control surface properties
				control_signal_filter_.initialize(0.005f, 0, 0);
			}

			// set elevator signal from -1 to 1 for full deflection at limit
			void setControlSignal(real_T control_signal)
			{
				// control_signal_filter_.setInput(Utils::clip(control_signal, -1.0f, 1.0f));
				output_.control_deflection = (control_signal - 1500.0f) / 500.0f;
			}

			Output getOutput() const
			{
				return output_;
			}

			/* Start: update state implementation*/
			void resetImplementation()
			{
				control_signal_filter_.reset();
				setOutput(output_, control_signal_filter_);
			}

			void update()
			{
				// control_signal_filter_.update();
				// setOutput(output_, control_signal_filter_);
			}

			void reportState(StateReporter& reporter)
			{
				reporter.writeValue("Ctrl-in", output_.control_signal_input);
				reporter.writeValue("Ctrl-fl", output_.control_signal_filtered);
				reporter.writeValue("Control-Deflection", output_.control_deflection);
				reporter.writeValue("Control-Speed", output_.control_speed);
			}
			/* End: update state implementation*/
			
		private: // methods

			//calculates all the forces
			void setOutput(Output& output, const FirstOrderFilter<real_T>& control_signal_filter)
			{
				output.control_signal_input = control_signal_filter_.getInput();
				output.control_signal_filtered = control_signal_filter_.getOutput();
			}

		private: //fields
			FirstOrderFilter<real_T> control_signal_filter_;
			const Kinematics* kinematics_ = nullptr;
			real_T air_density_sea_level_, air_density_ratio_, dyn_pressure_, forward_velocity_;
			Output output_;
			
			
		};
	}
}

#endif

