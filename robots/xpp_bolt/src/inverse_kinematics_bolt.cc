/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <xpp_bolt/inverse_kinematics_bolt.h>

#include <cmath>
#include <iostream>

#include <xpp_states/endeffector_mappings.h>


namespace xpp {

Joints
InverseKinematicsBolt::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  using namespace biped;
  std::vector<Eigen::VectorXd> q_vec;

  // make sure always exactly 2 elements
  auto x_biped_B = x_B.ToImpl();
  x_biped_B.resize(2, x_biped_B.front());

  Eigen::Vector3d offset_base_to_hip_L(0.0,-0.04, 0.065);
  Eigen::Vector3d offset_base_to_hip_R(0.0, 0.04, 0.065);


  q_vec.push_back(leg.GetJointAngles(x_biped_B.at(L) + offset_base_to_hip_L));
  q_vec.push_back(leg.GetJointAngles(x_biped_B.at(R) + offset_base_to_hip_R));


  return Joints(q_vec);
}

} /* namespace xpp */


/*
    const double z_nominal_b = -0.31;
    const double y_nominal_b =  0.1235;

    nominal_stance_.at(L) << 0.0,  y_nominal_b, z_nominal_b;
    nominal_stance_.at(R) << 0.0, -y_nominal_b, z_nominal_b;
    */