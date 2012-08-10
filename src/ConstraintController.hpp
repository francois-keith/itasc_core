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

#ifndef _ITASC_CONSTRAINTCONTROLLER_HPP_
#define _ITASC_CONSTRAINTCONTROLLER_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/extras/Properties.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Core>

namespace iTaSC {

class ConstraintController: public RTT::TaskContext {

protected:
	///input: robot joint values
	RTT::InputPort<KDL::JntArray> q_port;
	///input: Feature coordinates (from a Virtual Kinematic Chain or an Interaction Model)
	RTT::InputPort<KDL::JntArray> Chif_port;

	///output: modified constraint (at velocity level)
	RTT::OutputPort<KDL::JntArray> ydot_port;
	///output:feature selection matrix
	RTT::OutputPort<Eigen::MatrixXd> Cf_port;
	///output:joint selection matrix
	RTT::OutputPort<Eigen::MatrixXd> Cq_port;
	///output: constraint weighting matrix
	RTT::OutputPort<Eigen::MatrixXd> Wy_port;
	///output: flag: task initialized?
	RTT::OutputPort<bool> initialized;

	//number of constraints
	unsigned int nc;
	//number of feature coordinates
	unsigned int nfc;
	//number of joints of robot 1 and 2
	unsigned int nq;
	//feature coordinates
	KDL::JntArray Chif;
	//joint values
	KDL::JntArray q;
	//feature selection matrix
	Eigen::MatrixXd Cf;
	//joint selection matrix
	Eigen::MatrixXd Cq;
	//Constraint weight matrix
	Eigen::MatrixXd Wy;
	std::vector<double> Wy_prop;
	std::vector<double,Eigen::aligned_allocator<double> > Wy_prop_aligned;

public:
	virtual bool configureHook()=0;
	virtual bool startHook()=0;
	virtual void updateHook()=0;
	virtual void stopHook()=0;
	virtual void cleanupHook()=0;

	ConstraintController(std::string name, TaskState initial_state = Stopped) :
		TaskContext(name, initial_state),
		Cf(Eigen::Matrix<double, 6, 6>::Zero()),
		Cq(Eigen::Matrix<double, 6, 6>::Zero()),
		Wy_prop(std::vector<double>(6, 1.0))
	{
		//INPUT
		this->ports()->addPort("q", q_port);
		this->ports()->addPort("Chif", Chif_port);
		//OUTPUT
		this->ports()->addPort("ydot", ydot_port);
		this->ports()->addPort("Cf", Cf_port);
		this->ports()->addPort("Cq", Cq_port);
		this->ports()->addPort("Wy", Wy_port);
		this->ports()->addPort("initialized", initialized);
		//PROPERTIES
		this->provides()->addProperty("W", Wy_prop).doc(
					"diagonal elements of weighting matrix (do not fill with negative values)");
		//ATTRIBUTES
		this->provides()->addProperty("nc", nc);
	};
	virtual ~ConstraintController() {};
};

}//end of namespace

#endif

