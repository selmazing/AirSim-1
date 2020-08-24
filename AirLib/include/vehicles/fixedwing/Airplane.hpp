#pragma once
#ifndef msr_airlib_airplane_hpp
#define msr_airlib_airplane_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "physics/PhysicsBodyVertex.hpp"
#include "AircraftParams.hpp"

namespace msr
{
	namespace airlib
	{
		class Airplane : PhysicsBodyVertex
		{
		public:
			// types, declare variables in struct?

		public:
			Airplane()
			{
				// default constructor
			}

			Airplane(const Vector3r& position, const Vector3r& normal, const LinearAeroDerivatives& derivative)
			{
				initialize(position, normal, derivative);
			}
			void initialize(const Vector3r& position, const Vector3r& normal, const LinearAeroDerivatives& derivative)
			{
				PhysicsBodyVertex::initialize(position, normal); // call base initializer
			}
		};
		
	}
}
#endif