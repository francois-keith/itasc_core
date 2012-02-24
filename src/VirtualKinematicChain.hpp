/******************************************************************************
 *                 This file is part of the iTaSC project                      *
 *                                                                             *
 *                        (C) 2011 Dominick Vanthienen
 *                    dominick.vanthienen@mech.kuleuven.be,
 *                    Department of Mechanical Engineering,                    *
 *                   Katholieke Universiteit Leuven, Belgium.                  *
 *                   http://www.orocos.org/itasc                               *
 *                                                                             *
 *       You may redistribute this software and/or modify it under either the  *
 *       terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  *
 *       <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your *
 *       discretion) of the Modified BSD License:                              *
 *       Redistribution and use in source and binary forms, with or without    *
 *       modification, are permitted provided that the following conditions    *
 *       are met:                                                              *
 *       1. Redistributions of source code must retain the above copyright     *
 *       notice, this list of conditions and the following disclaimer.         *
 *       2. Redistributions in binary form must reproduce the above copyright  *
 *       notice, this list of conditions and the following disclaimer in the   *
 *       documentation and/or other materials provided with the distribution.  *
 *       3. The name of the author may not be used to endorse or promote       *
 *       products derived from this software without specific prior written    *
 *       permission.                                                           *
 *       THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  *
 *       IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        *
 *       WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    *
 *       ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,*
 *       INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
 *       (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       *
 *       OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
 *       HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   *
 *       STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING *
 *       IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    *
 *       POSSIBILITY OF SUCH DAMAGE.                                           *
 *                                                                             *
 *******************************************************************************/

#ifndef _ITASC_VIRTUALKINEMATICCHAIN_HPP_
#define _ITASC_VIRTUALKINEMATICCHAIN_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Core>

namespace iTaSC {

class VirtualKinematicChain: public RTT::TaskContext {
public:
	VirtualKinematicChain(std::string name, TaskState initial_state = Stopped) :
		TaskContext(name, initial_state) {
		//INPUT
		this->ports()->addPort("T_o1_o2_in", T_o1_o2_in_port);
		this->ports()->addPort("Jq_qdot", Jq_qdot_port);
		//OUTPUT
		this->ports()->addPort("T_o1_o2_out", T_o1_o2_out_port);
		this->ports()->addPort("Chif", Chif_port);
		this->ports()->addPort("JuXudot", JuXudot_port);
		this->ports()->addPort("Jf", Jf_port);
		this->ports()->addPort("initialized", initialized);

		this->provides()->addAttribute("nfc", nfc);

        this->addOperation("updateVKCE", &VirtualKinematicChain::updateVKCE, this)
            .doc("updateVKCE of "+this->getName());

	};

	virtual ~VirtualKinematicChain() {};

	virtual bool configureHook()=0;
	virtual bool startHook()=0;
	virtual void updateHook()=0;
	virtual void stopHook()=0;
	virtual void cleanupHook()=0;
    virtual void updateVKCE()=0;
protected:
	///input: RelPose(o2|o2,o1|o1,o1) (pose of o2 on body o2 wrt. o1 on body o1 expressed in o1)
	RTT::InputPort<KDL::Frame> T_o1_o2_in_port;
	///input: task robots twist = RelTwist(o1|o1,o1,o2)  (ref.point o1 on object o1 expressed in o1)
	RTT::InputPort<KDL::Twist> Jq_qdot_port;

	///output: RelPose(o2|o2,o1|o1,o1) (pose of o2 on body o2 wrt. o1 on body o1 expressed in o1)
	RTT::OutputPort<KDL::Frame> T_o1_o2_out_port;
	///output: Feature coordinates
	RTT::OutputPort<KDL::JntArray> 	Chif_port;
	///output: modified constraint (at velocity level)
	RTT::OutputPort<KDL::JntArray> ydot_port;
	///output: geometric uncertainty of the task=RelTwist(b2|ee2, b2, b2) (ref.point b12 on object ee2 expressed in b2)
	RTT::OutputPort<KDL::Twist> JuXudot_port;
	///output:feature selection matrix
	RTT::OutputPort<Eigen::MatrixXd> Cf_port;
	///output: Jf=RelJacobian=cfr.RelTwist(o1|o2,o1,o1) (ref.point o1 on object o2 expressed in o1)
	RTT::OutputPort<KDL::Jacobian> Jf_port;
	///output: constraint weighting matrix
	RTT::OutputPort<Eigen::MatrixXd> Wy_port;
	///output: flag: task initialized?
	RTT::OutputPort<bool> initialized;

	unsigned int nfc;
};

}//end of namespace

#endif

