/*******************************************************************************
 *                 This file is part of the iTaSC project                      *
 *                        	
 *			  (C) 2012 Pieterjan Bartels			       *
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

#ifndef _ITASC_SCENE_HPP
#define _ITASC_SCENE_HPP

#include <rtt/TaskContext.hpp>
#include <map>
#include "SubRobot.hpp"
#include "ConstraintController.hpp"
#include "ConstraintControllerInequality.hpp"
#include "ConstraintController.hpp"
#include "VirtualKinematicChain.hpp"
#include "Solver.hpp"
#include "eigen_toolkit.hpp"

#include <string>
#include <sstream>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Core>

#include <vector>

//define variables that make sense, but allow quick comparison
#define ROBOT 0
#define VIRTUAL_KINEMATIC_CHAIN 1
#define INTERACTION_MODEL 2

namespace iTaSC {

class Scene: public RTT::TaskContext {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Scene(const std::string& name);
	~Scene() {};

	virtual bool configureHook();
	virtual bool startHook() { return true;	}
	virtual void updateHook(){};
	virtual void stopHook() {};
	virtual void cleanupHook();

	bool orderPriorities();
	bool addRobot(const std::string& PeerName, const KDL::Frame& T_w_b);
	bool addObjectFrame(const std::string& objectFrameName,const std::string& segmentName, const std::string& PeerName);
	bool addConstraintController(const std::string& PeerName, const std::string& Object1, const std::string& Object2, const std::string& virtualLinkName, const int PriorityFromBag);
	bool addVirtualKinematicChain(const std::string& PeerName, const std::string& Object1, const std::string& Object2 );
	bool addSolver(const std::string& PeerName);
	bool broadcastObjectFrames();
	bool connectScene2Robots();
	bool connectScene2Solver();
	void calculatePoses();
	void calculateA();
	void handOut();

private:
	std::string iTaSC_configuration;
	///Output: weighting matrix on the joints of all Robots (used by solver)
	RTT::OutputPort<Eigen::MatrixXd> Wq_port;
	///Output: vector with the number of constraint per priority
	RTT::OutputPort<std::vector<int>  > nc_priorities_port;
	///Input: desired joint position
	RTT::InputPort<Eigen::VectorXd> qdot_port;
	int taskPriority;

	//declaring ObjectFrame and creating map here, so that Robot knows the map
	struct ObjectFrame;
	typedef std::map<std::string, ObjectFrame*> ObjectFrameMap;

	struct Robot {
	public:
		TaskContext* peer;
		///Output: desired joint velocity
		RTT::OutputPort<KDL::JntArray> qdot_port;
		///Output: Object Frames
		RTT::OutputPort<std::vector<std::string> > objectFrames_port;
		///Input: current joint position
		RTT::InputPort<KDL::JntArray> q_port;
		///Input: uncertainty twist: RelTwist(b1|ee1, b1, b1) (ref.point b1 on object ee1 expressed in b1)
		RTT::InputPort<KDL::Twist> JuXudot_port;
		///Input: weighting matrix on the joints of a Robot
		RTT::InputPort<Eigen::MatrixXd> Wq_port;
		///Input:global weighting matrix on the joints of a Robot
		RTT::InputPort<Eigen::MatrixXd> Wq_global_port;
		///Property: RelPose(b|b,w|w,w) = pose of b on body b wrt. w on body w expressed in w
		///This is initially read from the scene configuration
		///can this be dynamically changed???=> should be tested, I see no problems
		RTT::Property<KDL::Frame> T_w_b;
		//Joint velocities of a Robot
		KDL::JntArray qdot;
		//Joint positions of a Robot
		KDL::JntArray q;
		//weighting matrix on the joints of a Robot
		Eigen::MatrixXd Wq_local;
		//global weighting matrix on the joints of a Robot
		Eigen::MatrixXd Wq_global;
		//vector to keep object frame names in, to be send to robot component property objectFrames
		std::vector<std::string> objectFramesToRobot;
		//flag whether the robot component has a property objectFrames
		bool robotHasObjectFrameProp;

		unsigned int start_index, nq;

		ObjectFrameMap ObjectFrames;

		Robot(TaskContext* peer_in, const KDL::Frame& T_w_b_in,
				unsigned int nq_in, unsigned int start_index_in) :
			peer(peer_in),
			qdot_port(peer->getName() + "_qdot"),
			objectFrames_port(peer->getName()+"_objectFrames"),
			q_port(peer->getName() + "_q"),
			JuXudot_port(peer->getName() + "_JuXudot"),
			Wq_port(peer->getName()	+ "_Wq_local"),
			Wq_global_port(peer->getName() + "_Wq_global"),
			T_w_b(peer->getName() + "_T_w_b","Pose of the base of robot " + peer->getName() + " wrt world", T_w_b_in),
			robotHasObjectFrameProp(false),
			start_index(start_index_in),
			nq(nq_in)
		{
			if (nq > 0)
			{
				qdot.resize(nq);
				q.resize(nq);
				SetToZero(qdot);
				SetToZero(q);
				qdot_port.write(qdot);
				Wq_local = MatrixXd::Identity(nq, nq);
				Wq_global = MatrixXd::Identity(nq, nq);
				objectFramesToRobot.reserve(16);//does not break RT if more space is needed: automatic reallocation at configuring
			}
		}

	};//end of Robot

	struct ObjectFrame {
		public:
			Robot* robotPeer;
			std::string segmentName;
			std::string objectFrameName;
			///Input: RelJacobian=cfr.RelTwist(b|ee,b,b) = ref.point b on object ee expressed in b
			RTT::InputPort<KDL::Jacobian> Jq_port;
			///Input: RelPose(ee|ee,b|b,b) = pose of ee on body ee wrt. b on body b expressed in b
			RTT::InputPort<KDL::Frame> T_b_e_port;

			///Output: RelJacobian=cfr.RelTwist(w|ee,w,b) = ref.point w on object ee expressed in w
			RTT::OutputPort<KDL::Jacobian> Jq_w_port;

			//Robot jacobian
			KDL::Jacobian Jq;

			ObjectFrame(Robot* robotPeer_in, const std::string& segmentName_in, const std::string& objectFrameName_in) :
				robotPeer(robotPeer_in),
				segmentName(segmentName_in),
				objectFrameName(objectFrameName_in),
				Jq_port(robotPeer_in->peer->getName()+"_"+segmentName_in + "_Jq"),
				T_b_e_port(robotPeer_in->peer->getName()+"_"+segmentName_in+ "_T_b_e"),
				Jq_w_port(robotPeer_in->peer->getName()+"_"+segmentName_in + "_Jq_w")
			{
				if (robotPeer_in->nq > 0)
				{
					Jq.resize(robotPeer_in->nq);
					SetToZero(Jq);
				}
			}

	};//end of ObjectFrame

	struct VirtualLink {
	public:
		TaskContext* peer;

		///Input: geometric uncertainty of the task=RelTwist(b2|ee2, b2, b2) (ref.point b12 on object ee2 expressed in b2)
		RTT::InputPort<KDL::Twist> JuXudot_port;
		///Output: RelTwist(o1|o1,o2,o1)  (ref.point w on object ee1 expressed in w)
		RTT::OutputPort<KDL::Twist> Jq_qdot_port;
		///Output: RelPose(o2|o2,o1|o1,o1) (pose of o2 on body o2 wrt. o1 on body o1 expressed in w)
		RTT::OutputPort<KDL::Frame> T_o1_o2_port;

		//inverse feature jacobian Jf^(-1) or interaction model
		Eigen::MatrixXd H;

		unsigned int start_index;
		//number of feature coordinates
		unsigned int nfc;
		ObjectFrame* objectFrame1;
		ObjectFrame* objectFrame2;
		Robot* robot1;
		Robot* robot2;

		KDL::Frame T_b_ee1;
		KDL::Frame T_b_ee2;
		KDL::Frame T_w_o1;
		KDL::Frame T_w_o2;
		KDL::Frame T_o1_o2;

		VirtualLink(TaskContext* peer_in, ObjectFrame* objectFrame1_in, Robot* robot1_in, ObjectFrame* objectFrame2_in, Robot* robot2_in,
				unsigned int nFeatureCoordinates_in, unsigned int start_index_in) :
					peer(peer_in),
					JuXudot_port(peer->getName() + "_JuXudot"),
					Jq_qdot_port(peer->getName() + "_Jq_qdot"),
					T_o1_o2_port(peer->getName() + "_T_o1_o2"),
					start_index(start_index_in),
					nfc(nFeatureCoordinates_in),
					objectFrame1(objectFrame1_in),
					objectFrame2(objectFrame2_in),
					robot1(robot1_in),
					robot2(robot2_in),
					T_b_ee1(KDL::Frame::Identity()),
					T_b_ee2(KDL::Frame::Identity()),
					T_w_o1(KDL::Frame::Identity()),
					T_w_o2(KDL::Frame::Identity()),
					T_o1_o2(KDL::Frame::Identity())
		{}
	};

	struct VirtualKinematicChainStruct: public VirtualLink {
	public:
		///Input: feature jacobian
		RTT::InputPort<KDL::Jacobian> Jf_port;
		KDL::Jacobian Jf;
		Eigen::MatrixXd Uf, Vf;
		Eigen::VectorXd Sf;

		VirtualKinematicChainStruct(TaskContext* peer_in, ObjectFrame* objectFrame1_in, Robot* robot1_in, ObjectFrame* objectFrame2_in, Robot* robot2_in,
				unsigned int nFeatureCoordinates_in, unsigned int start_index_in) :
					VirtualLink(peer_in, objectFrame1_in,  robot1_in, objectFrame2_in, robot2_in, nFeatureCoordinates_in, start_index_in),
					Jf_port(peer->getName() + "_Jf"),
					Jf(6)
		{
			SetToZero(Jf);
		}
	};

	//TODO struct InteractionModel: public VirtualLink {};

	struct ConstraintControllerStruct {
	public:
		TaskContext* peer;
		///Input: modified constraint (at velocity level)
		RTT::InputPort<KDL::JntArray> ydot_port;
		///Input: feature selection matrix
		RTT::InputPort<Eigen::MatrixXd> Cf_port;
		///Input: joint selection matrix
		RTT::InputPort<Eigen::MatrixXd> Cq_port;
		///Input: constraint weighting matrix
		RTT::InputPort<Eigen::MatrixXd> Wy_port;
		///Input: global weight on this task
		RTT::InputPort<double> Wy_global_port;

		///Output: joint positions
		RTT::OutputPort<KDL::JntArray> q12_port;

		///Property: priority of this task
		RTT::Property<int> priority;

		VirtualLink* constrainedLink;
		ObjectFrame* objectFrame1;
		ObjectFrame* objectFrame2;
		Robot* robot1;
		Robot* robot2;
		//flag whether objectFrames are on same robot
		bool sameRobot;

		unsigned int start_index;
		//number of constraints
		unsigned int nc;
		KDL::JntArray y_dot_local;
		KDL::JntArray q12_local;
		Eigen::MatrixXd Cf_local, Cq_local, Wy_local;
		double Wy_global;
		//type of constrained instance (VKC, IM or only the robots)
		int constrainedInstanceType;

		ConstraintControllerStruct(TaskContext* peer_in, ObjectFrame* objectFrame1_in, Robot* robot1_in, ObjectFrame* objectFrame2_in, Robot* robot2_in, VirtualLink* constrainedLink_in, const int constrainedInstanceType_in, unsigned int nc_in, unsigned int start_index_in) :
					peer(peer_in),
					ydot_port(peer->getName() + "_ydot"),
					Cf_port(peer->getName() + "_Cf"),
					Cq_port(peer->getName() + "_Cq"),
					Wy_port(peer->getName() + "_Wy_local"),
					Wy_global_port(peer->getName() + "_Wy_global"),
					q12_port(peer->getName() + "_q12"),
					//TODO use everywhere blocks of matrices iso matrices itself: then you can change priority dynamically
					priority(peer->getName() + "_priority","priority of " + peer->getName() + " DO NOT CHANGE VALUE", 1),
					constrainedLink(constrainedLink_in),
					objectFrame1(objectFrame1_in),
					objectFrame2(objectFrame2_in),
					robot1(robot1_in),
					robot2(robot2_in),
					sameRobot(false),
					start_index(start_index_in),
					nc(nc_in),
					y_dot_local(nc),
					q12_local(objectFrame1_in->robotPeer->nq+objectFrame2_in->robotPeer->nq),
					//Cf_local default constructor is NULL matrix
					Cq_local(Eigen::MatrixXd::Zero(nc, objectFrame1_in->robotPeer->nq+objectFrame2_in->robotPeer->nq)),
					Wy_local(Eigen::MatrixXd::Zero(nc, nc)),
					Wy_global(0),
					constrainedInstanceType(constrainedInstanceType_in)
		{
			if(constrainedLink!=NULL)
			{
				Cf_local = Eigen::MatrixXd::Zero(nc, constrainedLink_in->nfc);
			}
		}
	};

	struct ConstraintControllerInequalityStruct : ConstraintControllerStruct {
	//this is a struct containing two extra ports, in case inequality constraints are used. One port is for the maximum of the interval, the other one is 		for a vector containing booleans. for the minimum of the interval, the ydot_port of the superclass is used.
	public: 
		///Input: modified constraint, the maximum level (at velocity level)
		RTT::InputPort<KDL::JntArray> ydot_max_port;
		KDL::JntArray y_max_local;
		
		ConstraintControllerInequalityStruct(TaskContext* peer_in, ObjectFrame* objectFrame1_in, Robot* robot1_in, ObjectFrame* objectFrame2_in, Robot* robot2_in, VirtualLink* constrainedLink_in, const int constrainedInstanceType_in, unsigned int nc_in, unsigned int start_index_in) :
			ConstraintControllerStruct(peer_in, objectFrame1_in, robot1_in, objectFrame2_in, robot2_in, constrainedLink_in, constrainedInstanceType_in, nc_in, start_index_in),
			ydot_max_port(peer->getName() + "_ydot_max"),
			y_max_local(nc){
		}
	};

	struct SolverStruct{
	public:
		TaskContext* peer;
		bool priorityProvisions;
		bool inequalityProvisions;

		SolverStruct(TaskContext* peer_in, bool priorityProvisions_in, bool ineqProv_in = false ):
			peer(peer_in),
			priorityProvisions(priorityProvisions_in),
			inequalityProvisions(ineqProv_in)
		{
		}
	};

	typedef std::map<std::string, Robot*> RobotMap;
	typedef std::map<std::string, ConstraintControllerStruct*> ConstraintControllerMap;
	typedef std::map<std::string, bool> InequalityMap;
	typedef std::map<std::string, VirtualKinematicChainStruct*> VKCMap;

	RobotMap robots;
	VKCMap VKCs;
	SolverStruct* the_solverp;

	//std::vector priorities contains structs of the "Priority" type, which contains TaskMaps with tasks
	struct Priority
	{
	public:
		ConstraintControllerMap constraints;
		InequalityMap inequalities;
		unsigned int nc_priority;
		RTT::OutputPort<Eigen::MatrixXd> A_port;
		RTT::OutputPort<Eigen::MatrixXd> Wy_port;
		RTT::OutputPort<Eigen::VectorXd> ydot_port;
		RTT::OutputPort<Eigen::VectorXd> ydot_max_port; //only necessary if the constraintController has inequality provisions
		RTT::OutputPort<Eigen::VectorXd> inequalities_port;//only necessary if the constraintcontroller has inequality provisions

		Eigen::MatrixXd A_priority, Wy_priority, tmpCfJf_priority, CfJfinvJq_priority;
		Eigen::VectorXd ydot_priority;
		Eigen::VectorXd ydotmax_priority;
		Eigen::VectorXd ydot_inequalities_priority;

		Priority() :
			//initialisations
			nc_priority(0)
		{

		}
	};


	std::vector<Priority*> priorities;

	//total number of robot joints
	unsigned int nq_total;
	//total number of constraints
	unsigned int nc_total;
	//vector containing the number of constraints per priority
	std::vector<int> nc_priorities;
	//total number of feature coordinates
	unsigned int nFeatureCoordinates_total;

	//Eigen::MatrixXd A_total, Wy_total, tmpCfJf,
	Eigen::MatrixXd  Wq_total, tmpJq;
	//Eigen::VectorXd ydot_total,
	Eigen::VectorXd qdot_total, temp;
	//Uncertainty Jacobian * derivative of uncertainty coordinates (=twist)
	Eigen::VectorXd JuXudot;

	//kdl version of JuXudot = twist
	KDL::Twist JuXudot_kdl;
	//
	KDL::Twist t1;
	KDL::Twist t2;
	//screw twist to contain t1-t2: in reference frame and point o1
	KDL::Twist Jq_qdot;
	//screw twist to contain t1-t2: in reference frame and point w
	KDL::Twist Jq_qdot_w;

	//flag to say that there are inequalityconstraints
	bool inequalitiesPresent;
	//flag to say that you've completed priority ordering
	bool prioritiesOrdered;

	std::string externalName;
	std::stringstream ssName;
};
}
#endif
