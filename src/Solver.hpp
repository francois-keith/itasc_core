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
#ifndef _ITASC_SOLVER_HPP_
#define _ITASC_SOLVER_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>

#include <Eigen/Core>

namespace iTaSC {

class Solver: public RTT::TaskContext {
protected:
	RTT::InputPort<Eigen::MatrixXd> A_port;
	RTT::InputPort<Eigen::MatrixXd> Wy_port;
	RTT::InputPort<Eigen::MatrixXd> Wq_port;
	RTT::InputPort<Eigen::VectorXd> ydot_port;
	RTT::InputPort<Eigen::VectorXd> ydot_max_port;
	RTT::InputPort<Eigen::VectorXd> inEqualities;
	RTT::OutputPort<Eigen::VectorXd> qdot_port;

	unsigned int nc;
	unsigned int nq;
	unsigned int inequalityProvisions;

	virtual bool solve()=0;

public:
	Solver(const std::string& name, unsigned int ineq) :
		TaskContext(name, Stopped), inequalityProvisions(ineq), A_port("A"), Wy_port("Wy"), Wq_port("Wq"),
				ydot_port("ydot"), ydot_max_port("ydot_max"), inEqualities("inequalities"), qdot_port("qdot")
	{
		this->ports()->addPort(A_port);
		this->ports()->addPort(Wy_port);
		this->ports()->addPort(Wq_port);
		this->ports()->addPort(ydot_port);
		this->ports()->addPort(ydot_max_port);
		this->ports()->addPort(inEqualities);
		this->ports()->addPort(qdot_port);

		this->provides()->addAttribute("inEqualityProvisions", inequalityProvisions);
		this->provides()->addAttribute("nc",nc);
		this->provides()->addAttribute("nq",nq);

		this->addOperation("solve", &Solver::solve, this, RTT::ClientThread).doc(
				"Solve the Scene's constraints");
	}
	;
	virtual ~Solver() {
	}
	;

	virtual bool configureHook()=0;
	virtual bool startHook()=0;
	virtual void updateHook()=0;
	virtual void stopHook()=0;
	virtual void cleanupHook()=0;
};
}
#endif
