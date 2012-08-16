/*******************************************************************************
 *                 This file is part of the iTaSC project                      *
 *                        						       *
 *			  (C) 2012 Pieterjan Bartels   			       *
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
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chain.hpp>


#include <Eigen/Core>

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

	///input: measured joint position (to read from robot)
	RTT::InputPort<std::vector<double> > q_from_robot;



	//Constant Attribute
	unsigned int nq;


//object frames
struct ObjectFrame{
public:
	std::string objectFrameName;
	unsigned int nq_chain;

	///Output Pose_<segment name>_base
	RTT::OutputPort<KDL::Frame> T_b_e_port;
	//jacobian of <segment frame> wrt root
	RTT::OutputPort<KDL::Jacobian> Jq_port;
	
	//forward kinematics solver
	KDL::ChainFkSolverPos_recursive* fk;
	//jacobian solver
	KDL::ChainJntToJacSolver* jnt2jac;


	//the jacobian of the objectframe
	KDL::Jacobian Jq_local;
	KDL::Jacobian Jq_refpee_local;
	//the joints of the chain to the objectframe
	KDL::JntArray q_local;
	
	//the pose of the objectframe
	KDL::Frame T_b_e_local;

	//indices of the joint positions q of_this_chain in the all-joint-vector of the robot
	std::vector<unsigned int> q_indices;
	//names of the joint belonging to this chain 
	std::vector<std::string> jnt_names;
	//RelJacobian
	RTT::OutputPort<KDL::Jacobian> Jq_refpee_port;
	//joints of chain from root to <segment_frame>
	RTT::OutputPort<KDL::JntArray> q_port;
	//names of the joint of this chain
	RTT::OutputPort<std::vector<std::string> > jnt_names_port;
	
	//constructor
	ObjectFrame( const std::string& segmentName, KDL::Chain segmentChain, std::vector<unsigned int> q_indices_in, std::vector<std::string> jnt_names_in):
		objectFrameName(segmentName),
		jnt_names(jnt_names_in),
		objectFrameChain(segmentChain){
			q_indices = q_indices_in;
			nq_chain = objectFrameChain.getNrOfJoints();
			if(nq_chain > 0){
				fk = new KDL::ChainFkSolverPos_recursive(objectFrameChain);
				jnt2jac = new KDL::ChainJntToJacSolver(objectFrameChain);
					Jq_local = KDL::Jacobian(nq_chain);
					Jq_local.data.setZero(6, nq_chain);
					
					Jq_refpee_local=KDL::Jacobian(nq_chain);
					Jq_refpee_local.data.setZero(6, nq_chain);
					
						
				q_local.resize(nq_chain);
				q_port.write(q_local);
			}
		}
		
	~ObjectFrame(){
			delete fk;
			delete jnt2jac;
	}
private: 
	//a chain from root to <segment_frame>	
	KDL::Chain objectFrameChain;

};

//create object frame map
typedef std::map<std::string, ObjectFrame*> ObjectFrameMap;
ObjectFrameMap OFmap;




};
}
#endif
