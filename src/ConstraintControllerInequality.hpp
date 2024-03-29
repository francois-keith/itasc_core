/*******************************************************************************
 *                 This file is part of the iTaSC project                      *
 *                                                                             *
 *			  (C) 2012 Pieterjan Bartels			       *
 *                        (C) 2011 Dominick Vanthienen			       *
 *                    dominick.vanthienen@mech.kuleuven.be,		       *
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
#ifndef _ITASC_CONSTRAINTCONTROLLER_INEQUALITY_HPP_
#define _ITASC_CONSTRAINTCONTROLLER_INEQUALITY_HPP_

#include "ConstraintController.hpp"
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/extras/Properties.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Core>

namespace iTaSC {
//subclass of ConstraintController, containing ports for output of the minimum and maximum variables of the constraint.
class ConstraintControllerInequality: public ConstraintController{

protected:
	///output: modified minimum level of constraint (at velocity level)
	RTT::OutputPort<KDL::JntArray> ydot_max_port;
public:
	ConstraintControllerInequality(const std::string & name, TaskState initial_state = Stopped) :
		ConstraintController(name, initial_state)
	{
		//OUTPUT
		this->ports()->addPort("ydot_max", ydot_max_port);
	}
	virtual ~ConstraintControllerInequality() {}
	virtual bool isEqualityConstraint() const { return false; }
};


}//end of namespace

#endif
