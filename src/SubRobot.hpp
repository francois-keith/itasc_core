/*******************************************************************************
 *                 This file is part of the iTaSC project                      *
 *                        													   *
 *                        (C) 2011 Dominick Vanthienen                         *
 *                        (C) 2010 Ruben Smits                                 *
 *                    dominick.vanthienen@mech.kuleuven.be,                    *
 *                        ruben.smits@mech.kuleuven.be                         *
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
#ifndef _ITASC_SUBROBOT_HPP_
#define _ITASC_SUBROBOT_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Core>
#include "eigen_toolkit.hpp"

namespace iTaSC {
using namespace Eigen;


class SubRobot: public RTT::TaskContext {
public:
	SubRobot(const std::string& name, TaskState initial_state = Stopped) :
		TaskContext(name, initial_state)
	{
		this->ports()->addPort("qdot", qdot_port);
		this->ports()->addPort("objectFramesPort", objectFrames_port);
		this->ports()->addPort("q", q_port);
		this->ports()->addPort("JuXudot", JuXudot_port);
		this->ports()->addPort("Wq", Wq_port);
		this->ports()->addPort("Jq", Jq_port);
		this->ports()->addPort("T_b_e", T_b_e_port);

		this->provides()->addAttribute("nq", nq);

		this->addOperation("updateRobotState", &SubRobot::updateRobotState, this,RTT::ClientThread) .doc("updateRobotState ");
		this->addOperation("sendToRobot", &SubRobot::sendToRobot, this,RTT::ClientThread) .doc("sendToRobot ");
	}
	;

	~SubRobot() {
	}
	;

	virtual bool configureHook()=0;
	virtual bool startHook()=0;
	virtual void updateHook()=0;
	virtual void stopHook()=0;
	virtual void cleanupHook()=0;
	virtual void updateRobotState(){};
	virtual void sendToRobot(){};

protected:
	//Input
	RTT::InputPort<KDL::JntArray> qdot_port;
	///input: vector of the names of object frames to broadcast
	RTT::InputPort<std::vector<std::string> > objectFrames_port;
	//Output
	RTT::OutputPort<KDL::JntArray> q_port;
	RTT::OutputPort<KDL::Twist> JuXudot_port;
	RTT::OutputPort<MatrixXd> Wq_port;
	RTT::OutputPort<KDL::Jacobian> Jq_port;
	RTT::OutputPort<KDL::Frame> T_b_e_port;

	///input: measured joint position (to read from robot)
	RTT::InputPort<std::vector<double> > q_from_robot;



	//Constant Attribute
	unsigned int nq;

};
}
#endif
