/*******************************************************************************
 *                 This file is part of the OROCOS iTaSC project               *
 *                        			
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

#include "Scene.hpp"
#include "Solver.hpp"
#include "eigen_toolkit.hpp"

#include <rtt/marsh/PropertyDemarshaller.hpp>
#include <ocl/Component.hpp>

#include <kdl/frames_io.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>

ORO_CREATE_COMPONENT( iTaSC::Scene )
;

namespace iTaSC {

using namespace RTT;
using namespace std;
using namespace KDL;
using namespace Eigen;

Scene::Scene(const string& name) :
	TaskContext(name, PreOperational),
	iTaSC_configuration("iTaSC_configuration.xml"),
	taskPriority(1),
	priorities(std::vector<Priority*>(1,new Priority)),
	nq_total(0),
	nc_total(0),
	nc_priorities(std::vector<int>(1,0)),
	nFeatureCoordinates_total(0),
	JuXudot(Eigen::VectorXd::Zero(6)),
	JuXudot_kdl(Twist::Zero()),
	t1(Twist::Zero()),
	t2(Twist::Zero()),
	Jq_qdot(Twist::Zero()),
	prioritiesOrdered(false),
	inequalitiesPresent(false)
{
	this->properties()->addProperty("iTaSC_configuration", iTaSC_configuration).doc(
			"file containing the configuration of robots and virtual linkages");

	this->ports()->addPort("Wq", Wq_port);
	this->ports()->addPort("nc_priorities", nc_priorities_port);
	this->ports()->addPort("qdot", qdot_port);

	this->addOperation("addRobot", &Scene::addRobot, this,
		    ClientThread) .doc("add a robot model to the Scene");
	this->addOperation("addObjectFrame", &Scene::addObjectFrame, this,
			    ClientThread) .doc("add a object frame to a robot");
	this->addOperation("addConstraintController",&Scene::addConstraintController, this,
		    ClientThread) .doc("add a Constraint/Controller to a task");
	this->addOperation("addVirtualKinematicChain", &Scene::addVirtualKinematicChain, this,
		    ClientThread) .doc("add a Virtual Kinematic Chain to a task");
	this->addOperation("addSolver", &Scene::addSolver, this,
			    ClientThread) .doc("add a Solver to iTaSC");
	this->addOperation("broadcastObjectFrames", &Scene::broadcastObjectFrames, this,
			    ClientThread) .doc("broadcast ObjectFrames (to Robots) ");
	this->addOperation("connectScene2Robots", &Scene::connectScene2Robots, this,
			    ClientThread) .doc("connect Scene to Robots ");
	this->addOperation("connectScene2Solver", &Scene::connectScene2Solver, this,
			    ClientThread) .doc("connect Scene to Solver ");
	this->addOperation("calculatePoses", &Scene::calculatePoses, this,
			    ClientThread) .doc("calculate robot and object poses");
	this->addOperation("calculateA", &Scene::calculateA, this,
			    ClientThread) .doc("calculate the pseudo-Jacobian A");
	this->addOperation("handOut", &Scene::handOut, this,
			    ClientThread) .doc("handOut updated values to all components connected to the Scene");
}

void Scene::cleanupHook() {
	//clear the RobotMap
	robots.clear();
	//clear the ConstraintControllerMaps
	for (unsigned int i = 0; i < priorities.size(); i++) {
		priorities[i]->constraints.clear();
	}
	//clear the priorities vector
	priorities.clear();
}

bool Scene::configureHook() {
	Logger::In in(this->getName());

	Wq_total = MatrixXd((int) nq_total, (int) nq_total);
	Wq_total.setZero();
	qdot_total = VectorXd((int) nq_total);
	qdot_total.setZero();
	tmpJq = MatrixXd(6, (int) nq_total);
	tmpJq.setZero();
	temp = VectorXd(6);
	temp.setZero();
	//initialize priority specific variables
	for (unsigned int i = 0; i < priorities.size(); i++) {
		priorities[i]->A_priority = MatrixXd((int) priorities[i]->nc_priority,
				(int) nq_total);
		priorities[i]->A_priority.setZero();
		priorities[i]->Wy_priority = MatrixXd((int) priorities[i]->nc_priority,
				(int) priorities[i]->nc_priority);
		priorities[i]->Wy_priority.setZero();
		priorities[i]->ydot_priority = VectorXd(
				(int) priorities[i]->nc_priority);
		priorities[i]->ydot_priority.setZero();
		priorities[i]->ydotmax_priority = VectorXd(
				(int) priorities[i]->nc_priority);
		priorities[i]->ydotmax_priority.setZero();
		priorities[i]->ydot_inequalities_priority = VectorXd(
				(int) priorities[i]->nc_priority);
		priorities[i]->ydot_inequalities_priority.setZero();
		priorities[i]->tmpCfJf_priority = MatrixXd(
				(int) priorities[i]->nc_priority, 6);
		priorities[i]->tmpCfJf_priority.setZero();
		priorities[i]->CfJfinvJq_priority = MatrixXd(
						(int) priorities[i]->nc_priority, (int) nq_total);
		priorities[i]->CfJfinvJq_priority.setZero();
	}
	return true;
}

bool Scene::orderPriorities() {
	Logger::In in(this->getName());
	//Check for non-consecutive priorities eg. 1,2,5
	//If that case, delete empty entries
	//WARNING: when doing so, changing priorities at runtime will crash the Scene (anyway,because matrix sizes will change too)
	for (int k = priorities.size() - 1; k >= 0; k--) {
		if (priorities[k]->constraints.empty()) {
			priorities.erase(priorities.begin() + k);
		}
	}
	//raise a flag that you've completed this ordering
	prioritiesOrdered = true;
	return true;
}

bool Scene::addRobot(const string& PeerName, const Frame& T_w_b) {

	Logger::In in(this->getName());
	log(Info) << "Adding SubRobot '" << PeerName << "', located at " << T_w_b
			<< endlog();

	//Check if the Scene is connected to this robot
	if (!this->hasPeer(PeerName)) {
		log(Error) << "Scene '" << this->getName()
				<< "' does not have a peer named '" << PeerName << "'."
				<< endlog();
		return false;
	}
	TaskContext* peer = this->getPeer(PeerName);
	//check if peer is a valid SubRobot class
	SubRobot* robotp = dynamic_cast<SubRobot*> (peer);
	if (robotp == NULL) {
		//if(!peer.properties()->hasAttribute("nq")||
		//   NULL==peer.ports()->getPort("qdot")||
		//   NULL==peer.ports()->getPort("Jq")||
		//   NULL==peer.ports()->getPort("T_w_ee"))
		log(Error) << "Component '" << PeerName
				<< "'is not a valid SubRobot component." << endlog();
		return false;
	}

	//Get the number of joints and configure the jointrange
	Attribute<unsigned int> nq = robotp->attributes()->getAttribute("nq");
	if (!nq.ready()) {
		log(Error) << "Component '" << PeerName
				<< "' does not have an necessary attribute nq" << endlog();
		return false;
	}

	//Insert new robot structure in the Scene's robotmap
	Robot* robot = new Robot(robotp, T_w_b, nq.get(), nq_total);
	pair<RobotMap::iterator, bool> result = robots.insert(RobotMap::value_type(
			PeerName, robot));
	if (!result.second) {
		log(Error) << "SubRobot with name '" << PeerName << "' already exists."
				<< endlog();
		//If inserting the robot did not succeed, delete robot and return
		delete robot;
		return false;
	}

	//check if the Robot has a objectFrames port
	RTT::base::PortInterface* objectFramesRobotPortp = peer->provides()->getPort("objectFramesPort");
	if (!objectFramesRobotPortp)
	{
		log(Error) << "Robot '" << PeerName <<"' has no port objectFrames." << endlog();
		//TODO handle this: assume end effector if no frame is given (and it is not a tree)
		//If it did not succeed, remove Robot from RobotMap
		delete robot;
		robots.erase(PeerName);
		return false;
	}else
	{
		robot->robotHasObjectFrameProp=true;
	}

	//Connect the ports of the Scene to the ports of the robot
	bool connected = true;
	this->ports()->addPort(robot->qdot_port);
	connected &= robot->qdot_port.connectTo(robotp->ports()->getPort("qdot"));
	this->ports()->addPort(robot->q_port);
	connected &= robot->q_port.connectTo(robotp->ports()->getPort("q"));
	this->ports()->addPort(robot->Wq_port);
	connected &= robot->Wq_port.connectTo(robotp->ports()->getPort("Wq"));
	this->ports()->addPort(robot->JuXudot_port);
	connected &= robot->JuXudot_port.connectTo(robotp->ports()->getPort("JuXudot"));
	this->ports()->addPort(robot->Wq_global_port);
	connected &= robot->objectFrames_port.connectTo(objectFramesRobotPortp);
	this->ports()->addPort(robot->objectFrames_port);

	if (!connected) {
		log(Error) << "Could not connect to SubRobot '" << PeerName
				<< "' ports." << endlog();
		//If connecting did not succeed, remove Robot from RobotMap
		delete robot;
		robots.erase(PeerName);
		return false;
	}

	this->properties()->addProperty(robot->T_w_b);

	nq_total += nq.get();
	return true;
}

bool Scene::addObjectFrame(const std::string& objectFrameName, const std::string& segmentName, const std::string& PeerName)
{
		Logger::In in(this->getName());
		log(Info) << "Adding an object frame '" << objectFrameName << "', to segment '"<< segmentName <<"' of robot " << PeerName << endlog();

		//Check if the Robot exists
		RobotMap::const_iterator robotIt = robots.find(PeerName);
		if (robotIt == robots.end()) {
			log(Error) << "Robot '" << PeerName
					<< "' does not exist in the Scene." << endlog();
			return false;
		}
		Robot* robotp = robotIt->second;

		//Insert new objectFrame structure in the Scene's  object frame map
		ObjectFrame* objectFrame = new ObjectFrame(robotp, segmentName, objectFrameName);
		pair<ObjectFrameMap::iterator, bool> result = robotp->ObjectFrames.insert(ObjectFrameMap::value_type(
				objectFrameName, objectFrame));
		if (!result.second) {
			log(Error) << "ObjectFrame with name '" << objectFrameName << "' already exists."
					<< endlog();
			//If inserting the ObjectFrame did not succeed, delete it and return
			delete objectFrame;
			return false;
		}

		//add segmentName to vector that will be put in objectFrames prop of robot
		robotp->objectFramesToRobot.push_back(segmentName);

		//Connect the ports of the Scene to the ports of the robot after ports are created at robot side
		this->ports()->addPort(objectFrame->Jq_port);
		this->ports()->addPort(objectFrame->T_b_e_port);
		this->ports()->addPort(objectFrame->Jq_w_port);

	return true;
}//end addObjectFrame

bool Scene::addConstraintController(const string& PeerName, const string& Object1, const string& Object2, const string& virtualLinkName, const int PriorityFromBag) {

	Logger::In in(this->getName());
	log(Info)<< "Adding Constraint/Controller '" << PeerName << "' on '" << virtualLinkName
			<< "' between objectFrame '"<< Object1
			<<"' and objectFrame '" << Object2
			<<"' with priority " << PriorityFromBag
			<< endlog();
	//Check if we are connected to this task
	if (!this->hasPeer(PeerName)) {
		log(Error) << "Scene '" << this->getName()
				<< "' does not have a peer named '" << PeerName << "'."
				<< endlog();
		return false;
	}

	TaskContext* peer = this->getPeer(PeerName);
	//check if peer is a valid ConstraintController class
	ConstraintController* constraintClassp = dynamic_cast<ConstraintController*> (peer);
	if (constraintClassp == NULL) {
		log(Error) << "Component '" << PeerName
				<< "'is not a valid ConstraintController component." << endlog();
		return false;
	}
	
	//determine whether or not the given ConstraintController is an inequality constraint controller
	bool hasInequalities = false;
	ConstraintControllerInequality* constraintClassEq = dynamic_cast<ConstraintControllerInequality*> (peer);
	if (constraintClassEq == NULL) {
		hasInequalities = false; //the given constraint controller has only equalities
	}
	else{
		hasInequalities = true; //the given ConstraintController is an instance of ConstraintControllerInequality
		inequalitiesPresent = true;
	}


	RobotMap::iterator robotp;
	Robot* robot1p;
	Robot* robot2p;
	ObjectFrameMap::iterator objectFrame1p;
	ObjectFrameMap::iterator objectFrame2p;
	ObjectFrame* objectFrameStruct1p;
	ObjectFrame* objectFrameStruct2p;
	bool OF1found = false;
	bool OF2found = false;
	for(robotp = robots.begin(); robotp != robots.end(); robotp++)
	{
		Robot* robot = robotp->second;

		//Check if ObjectFrame1 exists
		objectFrame1p = robot->ObjectFrames.find(Object1);
		if (objectFrame1p != robot->ObjectFrames.end())
		{
			OF1found = true;
			objectFrameStruct1p = objectFrame1p->second;
			robot1p = robot;
		}

		//Check if ObjectFrame2 exists
		objectFrame2p = robot->ObjectFrames.find(Object2);
		if (objectFrame2p != robot->ObjectFrames.end())
		{
			OF2found = true;
			objectFrameStruct2p = objectFrame2p->second;
			robot2p = robot;
		}
	}

	if (!OF1found)
	{
		log(Error) << "Object Frame '" << Object1
				<< "' does not exist in the Scene." << endlog();
		return false;
	}
	if (!OF2found)
	{
		log(Error) << "Object Frame '" << Object2
				<< "' does not exist in the Scene." << endlog();
		return false;
	}

	//Check whether object frames are on same robot
	bool sameRobot = false;
	if(robot1p==robot2p)
	{
		log(Info) << "Robots '" << robot1p->peer->getName()
				<<"' and '" << robot2p->peer->getName()
				<< " are the same" << endlog();
		sameRobot = true;
	}

	//Check if priority is feasible
	if (PriorityFromBag < 1) {
		log(Error) << "The priority for Constraint '" << PeerName << "' = "
				<< PriorityFromBag << "is not valid (should be (int)>0 )"
				<< endlog();
		return false;
	}

	//Get the number of constraints and configure the constraint range
	Property<unsigned int> nc = constraintClassp->provides()->getProperty("nc");
	if (!nc.ready()) {
		log(Error) << "Component '" << PeerName
				<< "' does not have an necessary Property nc" << endlog();
		return false;
	}
#ifndef NDEBUG
	else{
		log(Debug) << "Component '" << PeerName
				<< "' has " << nc.get() << " constraints" <<endlog();
	}
#endif


	//Check if there is already another ConstraintControllerStruct with the same priority
	//if not create extra priorities in the priorities vector
	while (priorities.size() < (unsigned int) PriorityFromBag) {
		Priority* priority = new Priority;
		priorities.push_back(priority);
	}

	ConstraintControllerStruct* constraintStructp;
	//check what type it is and if constrainedInstance exists
	//	1st check: is there a virtualLink?
	if(virtualLinkName == "")
	{
		log(Info) << "There is no virtual link constrained, so no feature coordinates." << endlog();
		//Check whether nc = nq+nfc (with nfc = 0)
		if(sameRobot)
		{
			if(nc.get() != (robot1p->nq))
			{
				log(Error) << "The number of constraints ("<<nc.get()<<") does not match the number of joints ("<<(robot1p->nq)
						<<"). \n Did you forget to mention the VKC or IM?" << endlog();
				return false;
			}
		}else
		{
			if(nc.get() != (robot1p->nq+robot2p->nq))
			{
				log(Error) << "The number of constraints ("<<nc.get()<<") does not match the number of joints ("<<(robot1p->nq+robot2p->nq)
						<<"). \n Did you forget to mention the VKC or IM?" << endlog();
				return false;
			}
		}
		//Add the ConstraintControllerStruct to the ConstraintControllerMap of the right priority of the Scene
		if(hasInequalities)
			constraintStructp = new ConstraintControllerInequalityStruct(constraintClassp,objectFrameStruct1p, robot1p, objectFrameStruct2p, robot2p, NULL, ROBOT, nc.get(),	priorities[PriorityFromBag - 1]->nc_priority);
		else
		constraintStructp = new ConstraintControllerStruct(constraintClassp,objectFrameStruct1p, robot1p, objectFrameStruct2p, robot2p, NULL, ROBOT, nc.get(),	priorities[PriorityFromBag - 1]->nc_priority);
	}else
	{
		//2nd check: is it a VirtualKinematicChainStruct?
		TaskContext* constrainedInstance = this->getPeer(virtualLinkName);
		VirtualKinematicChain* VKCclassp = dynamic_cast<VirtualKinematicChain*> (constrainedInstance);
		if (VKCclassp == NULL)
		{
			//TODO: 3th check: is it an IM?
			log(Error) << "The constrained instance '" << virtualLinkName << "' is of an unrecognized type." << endlog();
			return false;
		}else{
			log(Info) << "The constrained instance '"<< virtualLinkName << "' is a VirtualKinematicChainStruct" << endlog();
			VKCMap::const_iterator VKCit = VKCs.find(virtualLinkName);
			if (VKCit == VKCs.end())
			{
				log(Error) << "The constrained instance '" << virtualLinkName
						<< "' does not exist in the VKCMap of the Scene." << endlog();
				return false;
			}else
			{
				//Get a pointer to the VKC struct
				VirtualKinematicChainStruct* VKC = VKCit->second;

				//Check whether the VKC is defined between the same robots
				if(VKC->objectFrame1 != objectFrameStruct1p || VKC->objectFrame2 != objectFrameStruct2p)
				{
					log(Error) << "The Virtual Kinematic Chain '"<< virtualLinkName
							   << "' is not defined between the same object frames as the constraint/controller '"
							   << PeerName << "'. \n Check your iTaSC configuration file. " << endlog();
					return false;
				}

				//TODO what to do with the following?
				//Check whether nc = nq+nfc
				//if(nc.get() != (VKC->nfc + (objectFrameStruct1p->nq+objectFrameStruct2p->nq)))
				//{
				//	log(Error) << "The number of constraints does not match the number of joints + the number of feature coordinates. \n With nfc = "
				//			   << VKC->nfc << ", and sum of nq's = " << (objectFrameStruct1p->nq+objectFrameStruct2p->nq) << " while nc = " << nc.get()
				//			   << endlog();
				//	return false;
				//}
				//Add the ConstraintControllerStruct to the ConstraintControllerMap of the right priority of the Scene

				if(hasInequalities) 				
				constraintStructp = new ConstraintControllerInequalityStruct(constraintClassp,objectFrameStruct1p, robot1p, objectFrameStruct2p, robot2p, VKC, VIRTUAL_KINEMATIC_CHAIN, nc.get(),	priorities[PriorityFromBag - 1]->nc_priority);
				else
				constraintStructp = new ConstraintControllerStruct(constraintClassp,objectFrameStruct1p, robot1p, objectFrameStruct2p, robot2p, VKC, VIRTUAL_KINEMATIC_CHAIN, nc.get(),	priorities[PriorityFromBag - 1]->nc_priority);
#ifndef NDEBUG
				log(Debug) << "ConstraintControllerStruct created for constraint/controller '"
					<< PeerName << "' with " << constraintStructp->nc << " constraints" <<endlog();
#endif
			}
		}
	}

	//try to put the constraintStructp in the ConstraintControllerMap
	pair<ConstraintControllerMap::iterator, bool> result =
			priorities[PriorityFromBag - 1]->constraints.insert(ConstraintControllerMap::value_type(PeerName, constraintStructp));
	if (!result.second)
	{
		log(Error) << "ConstraintControllerStruct with name '" << PeerName << "' already exists."	<< endlog();
		//delete task if insertion did not succeed
		delete constraintStructp;
		return false;
	}
	else{
		priorities[PriorityFromBag - 1]->inequalities.insert(InequalityMap::value_type(PeerName, hasInequalities));
	}

	//put the sameRobot flag in the struct
	constraintStructp->sameRobot = sameRobot;

	//port connections
#ifndef NDEBUG
	log(Debug) << "Starting to create and connect ConstraintControllerStruct ports" <<endlog();
#endif
	bool connected = true;
	//Connect ConstraintControllerStruct to the Scene
	this->ports()->addPort(constraintStructp->ydot_port);
	connected &= constraintStructp->ydot_port.connectTo(constraintClassp->ports()->getPort("ydot"));
	if(hasInequalities){
		this->ports()->addPort(((ConstraintControllerInequalityStruct*) constraintStructp)->ydotmax_port);
		connected &= ((ConstraintControllerInequalityStruct*) constraintStructp) -> 
									ydotmax_port.connectTo(constraintClassEq->ports()->getPort("ydot_max"));
	}
	this->ports()->addPort(constraintStructp->Cf_port);
	connected &= constraintStructp->Cf_port.connectTo(constraintClassp->ports()->getPort("Cf"));
	this->ports()->addPort(constraintStructp->Cq_port);
	connected &= constraintStructp->Cq_port.connectTo(constraintClassp->ports()->getPort("Cq"));
	this->ports()->addPort(constraintStructp->Wy_port);
	connected &= constraintStructp->Wy_port.connectTo(constraintClassp->ports()->getPort("Wy"));
	this->ports()->addPort(constraintStructp->Wy_global_port);
	this->ports()->addPort(constraintStructp->q12_port);
	connected &= constraintStructp->q12_port.connectTo(constraintClassp->ports()->getPort("q"));
	//give a data sample to the q12_port to enable the CC component to configure matrix sizes
	if(sameRobot)
	{
		constraintStructp->Cq_local.setZero(nc, constraintStructp->robot1->nq);
		constraintStructp->q12_local.resize(constraintStructp->robot1->nq);

	}else
	{
		constraintStructp->q12_local.resize(constraintStructp->robot1->nq+constraintStructp->robot2->nq);
	}
	constraintStructp->q12_port.write(constraintStructp->q12_local);

	//connect the ConstraintController with the
	connected &= constraintStructp->Wy_port.connectTo(constraintClassp->ports()->getPort("Wy"));

	//Connect ConstraintControllerStruct to a VKC or IM if needed
	if(constraintStructp->constrainedInstanceType == VIRTUAL_KINEMATIC_CHAIN)
	{
		connected &= constraintStructp->constrainedLink->peer->ports()->getPort("Chif")->connectTo(constraintClassp->ports()->getPort("Chif"));
	}
	//TODO: add here connection for IM case!

	//add properties
	this->provides()->addProperty(constraintStructp->priority);
	constraintStructp->priority.set(PriorityFromBag);

	if (!connected) {
		log(Error) << "Could not connect to ConstraintControllerStruct '" << PeerName
				<< "' ports." << endlog();
		//delete constraintStructp and remove from map
		delete constraintStructp;
		priorities[PriorityFromBag - 1]->constraints.erase(PeerName);
		return false;
	}

	//put nc in right priority
	nc_total += nc.get();
#ifndef NDEBUG
	log(Debug) << "adding " << nc.get() << " constraints to priorities[" << (PriorityFromBag - 1) <<
		"], which puts it at a total of "<< priorities[PriorityFromBag - 1]->nc_priority + nc.get() << endlog();
#endif
	priorities[PriorityFromBag - 1]->nc_priority += nc.get();

	return true;
}//end of addConstraintController: implementation with VKC/IM

bool Scene::addVirtualKinematicChain(const string& PeerName, const string& Object1, const string& Object2)
{
	Logger::In in(this->getName());
	log(Info) << "Adding the Virtual Kinematic Chain '" << PeerName << "' between '"
			<< Object1 <<"'  and '" << Object2 << endlog();
	//Check if we are connected to this task
	if (!this->hasPeer(PeerName)) {
		log(Error) << "Scene '" << this->getName()
				<< "' does not have a peer named '" << PeerName << "'."
				<< endlog();
		return false;
	}
	TaskContext* peer = this->getPeer(PeerName);
	//check if peer is a valid VirtualKinematicChainStruct class
	VirtualKinematicChain* VKCclassp = dynamic_cast<VirtualKinematicChain*> (peer);
	if (VKCclassp == NULL) {
		log(Error) << "Component '" << PeerName
				<< "'is not a valid VirtualKinematicChainStruct component." << endlog();
		return false;
	}

	RobotMap::iterator robotp;
	Robot* robot1p;
	Robot* robot2p;
	ObjectFrameMap::iterator objectFrame1p;
	ObjectFrameMap::iterator objectFrame2p;
	ObjectFrame* objectFrameStruct1p;
	ObjectFrame* objectFrameStruct2p;
	bool OF1found = false;
	bool OF2found = false;
	for(robotp = robots.begin(); robotp != robots.end(); robotp++)
	{
		Robot* robot = robotp->second;

		//Check if ObjectFrame1 exists
		objectFrame1p = robot->ObjectFrames.find(Object1);
		if (objectFrame1p != robot->ObjectFrames.end())
		{
			OF1found = true;
			objectFrameStruct1p = objectFrame1p->second;
			robot1p = robot;
		}

		//Check if ObjectFrame2 exists
		objectFrame2p = robot->ObjectFrames.find(Object2);
		if (objectFrame2p != robot->ObjectFrames.end())
		{
			OF2found = true;
			objectFrameStruct2p = objectFrame2p->second;
			robot2p = robot;
		}
	}

	if (!OF1found)
	{
		log(Error) << "Object Frame '" << Object1
				<< "' does not exist in the Scene." << endlog();
		return false;
	}
	if (!OF2found)
	{
		log(Error) << "Object Frame '" << Object2
				<< "' does not exist in the Scene." << endlog();
		return false;
	}

	//Get the number of constraints and configure the VKC range
	Attribute<unsigned int> nFeatureCoordinates = VKCclassp->provides()->getAttribute("nfc");
	if (!nFeatureCoordinates.ready()) {
		log(Error) << "Component '" << PeerName
				<< "' does not have an necessary Attribute nFeatureCoordinates" << endlog();
		return false;
	}

	//Add the task to the VKCMap of the Scene
	VirtualKinematicChainStruct* VKCstructp = new VirtualKinematicChainStruct(VKCclassp, objectFrameStruct1p, robot1p, objectFrameStruct2p, robot2p, nFeatureCoordinates.get(),
			nFeatureCoordinates_total);
	pair<VKCMap::iterator, bool> result = VKCs.insert(VKCMap::value_type(PeerName, VKCstructp));
	if (!result.second) {
		log(Error) << "VirtualKinematicChainStruct with name '" << PeerName << "' already exists."
				<< endlog();
		//delete VKC if insertion did not succeed
		delete VKCstructp;
		return false;
	}
#ifndef NDEBUG
	log(Debug) << "Starting to create and connect VKC ports" <<endlog();
#endif
	bool connected = true;
	//Connect VKC to the Scene
	this->ports()->addPort(VKCstructp->Jq_qdot_port);
	connected &= VKCstructp->Jq_qdot_port.connectTo(VKCclassp->ports()->getPort("Jq_qdot"));
	this->ports()->addPort(VKCstructp->Jf_port);
	connected &= VKCstructp->Jf_port.connectTo(VKCclassp->ports()->getPort("Jf"));
	this->ports()->addPort(VKCstructp->T_o1_o2_port);
	connected &= VKCstructp->T_o1_o2_port.connectTo(VKCclassp->ports()->getPort("T_o1_o2_in"));

	if (!connected) {
		log(Error) << "Could not connect to VirtualKinematicChain '" << PeerName
				<< "' ports." << endlog();
		//delete VKCstructp and remove from map
		delete VKCstructp;
		VKCs.erase(PeerName);
		return false;
	}

	//initialisation of VKCstructp specific parameters
	VKCstructp->H.resize(nFeatureCoordinates.get(),6);
	VKCstructp->H.setZero();
	VKCstructp->Uf.resize(6, nFeatureCoordinates.get());
	VKCstructp->Uf.setIdentity();
	VKCstructp->Vf.resize(nFeatureCoordinates.get(), nFeatureCoordinates.get());
	VKCstructp->Vf.setIdentity();
	VKCstructp->Sf.resize(nFeatureCoordinates.get());
	VKCstructp->Sf.setZero();

	//augment the total number of VKC with this VKC
	nFeatureCoordinates_total += nFeatureCoordinates.get();

	return true;
}// end of addVirtualKinematicChain

bool Scene::addSolver(const string& PeerName) {
	Logger::In in(this->getName());
	log(Info) << "Adding Solver '" << PeerName << "'." << endlog();

	//eliminate non-existing priorities
	if(!prioritiesOrdered){this->orderPriorities();}

	//Check if the Scene is connected to this robot
	if (!this->hasPeer(PeerName)) {
		log(Error) << "Scene '" << this->getName()
				<< "' does not have a peer named '" << PeerName << "'."
				<< endlog();
		return false;
	}
	TaskContext* peer = this->getPeer(PeerName);
	//check if peer is a valid Solver class
	Solver* solverp = dynamic_cast<Solver*> (peer);
	if (solverp == NULL) {
		//if(!peer.provides()->hasAttribute("nq")||
		//   NULL==peer.ports()->getPort("qdot")||
		//   NULL==peer.ports()->getPort("Jq")||
		//   NULL==peer.ports()->getPort("T_w_ee"))
		log(Error) << "Component '" << PeerName
				<< "'is not a valid Solver component." << endlog();
		return false;
	}
	
	//Set nc and nq attributes of the Solver
	Attribute<unsigned int> nq = peer->provides()->getAttribute("nq");
	nq.set(nq_total);
	Attribute<unsigned int> priority_solver = peer->provides()->getAttribute("priorityNo");
	//	check whether the Solver has provisions for task priorities
	if (!priority_solver.ready()) {
		log(Warning)
				<< "The proposed Solver has no provisions for task priorities. \n The program will take only primary constraints into account."
				<< endlog();
		//Set the nc and niq attributes of the Solver
		Attribute<unsigned int> nc = peer->provides()->getAttribute("nc");
		nc.set(priorities[0]->nc_priority);
		//Create a SolverStruct
		the_solverp = new SolverStruct(solverp, false);
	} else {
		//the Solver has provisions for task priorities in this case:
		priority_solver.set(priorities.size());
#ifndef NDEBUG
		log(Debug) << "Attribute priorityNo of Solver successfully set."<< endlog();
#endif
		//Create a SolverStruct
		the_solverp = new SolverStruct(solverp, true);
	}
	Attribute<bool> iep = peer->provides()->getAttribute("inEqualityProvisions");
	the_solverp->inequalityProvisions = iep.get();
	if (the_solverp->priorityProvisions)
	{
		//create a port
		if(!nc_priorities_port.connectTo(peer->ports()->getPort("nc_priorities"))){log(Error) << "[[addSolver]] unable to connect to nc_priorities" << endlog();}
		//put the constraints per priority in a vector
		nc_priorities.resize(priorities.size()); // else a vector sized 1 is sent
		for (unsigned int i = 0; i < priorities.size(); i++)
		{
			nc_priorities[i]=priorities[i]->nc_priority;
#ifndef NDEBUG
			log(Debug) << "nc_priorities["<< i<<"] = "<< nc_priorities[i] << endlog();
#endif
		}
		//put that vector on the created port
		nc_priorities_port.write(nc_priorities);
		log(Info)<<"Number of constraints written to nc_priorities"<< endlog();


	}

	return true;
}

bool Scene::broadcastObjectFrames()
{
	Logger::In in(this->getName());
	RobotMap::iterator robotp;
	for (robotp = robots.begin(); robotp != robots.end(); ++robotp)
	{
		Robot* robot = robotp->second;
		robot->objectFrames_port.write(robot->objectFramesToRobot);
	}
	return true;
}

//connect the ports of the Scene with the ports of the robots
bool Scene::connectScene2Robots()
{
	Logger::In in(this->getName());
	bool connected = true;
	//iterate over all objectFrames of all robots
	RobotMap::iterator Rit;
	ObjectFrameMap::iterator OFit;
	for(Rit = robots.begin(); Rit != robots.end(); Rit++)
	{
		Robot* robot = Rit->second;
		//if(robot->nq != 0)
		//{
			for (OFit = robot->ObjectFrames.begin(); OFit != robot->ObjectFrames.end(); ++OFit)
			{
				ObjectFrame* objectFrame = OFit->second;
				if(robot->nq != 0)
				{
					//give the port at the Scene side a name ...
					externalName= "Jq_" + objectFrame->segmentName +"_base";
					//... and connect with it's counterpart at the Robot side
					connected &= objectFrame->Jq_port.connectTo(robot->peer->ports()->getPort(externalName));
				}
				externalName= "Pose_" + objectFrame->segmentName +"_base";
				connected &= objectFrame->T_b_e_port.connectTo(robot->peer->ports()->getPort(externalName));
			}
		//}
	}
	return connected;
}
//connect the ports of the Scene with the ports of the solver
bool Scene::connectScene2Solver()
{
	Logger::In in(this->getName());
	//ports independent of the fact that there are priority provisions or not
	if(!Wq_port.connectTo(the_solverp->peer->ports()->getPort("Wq"))){log(Error) << "[[addSolver]] unable to connect to Wq" << endlog();}
	if(!qdot_port.connectTo(the_solverp->peer->ports()->getPort("qdot"))){log(Error) << "[[addSolver]] unable to connect to qdot" << endlog();}

	if((!the_solverp->inequalityProvisions) && inequalitiesPresent) { //the solver doesn't have inequalityProvisions, but there are inequalities present
		log(Error) << "[[ConnectScene2Solver] error: Scene has inequalities, but solver can't handle inequalities" << endlog();
	}

	//ports dependent of the fact that there are priority provisions or not
	if (the_solverp->priorityProvisions)
	{
		//connect every port
		for (unsigned int i = 0; i < priorities.size(); i++)
		{
			//give the port at the Scene side a name and connect with it's counterpart at the Solver side
			ssName.clear();
			ssName << "A_" << i + 1;
			ssName >> externalName;
			this->ports()->addPort(externalName, priorities[i]->A_port).doc(
					"Generalized Jacobian with priority of the index");
			if(!priorities[i]->A_port.connectTo(the_solverp->peer->ports()->getPort(
					externalName))){log(Error) << "[[addSolver]] unable to connect to A_port of priority " << i << endlog();}
			ssName.clear();
			ssName << "Wy_" << i + 1;
			ssName >> externalName;
			this->ports()->addPort(externalName, priorities[i]->Wy_port).doc(
					"Output weight matrix with priority of the index");
			if(!priorities[i]->Wy_port.connectTo(the_solverp->peer->ports()->getPort(
					externalName))){log(Error) << "[[addSolver]] unable to connect to Wy_port of priority " << i << endlog();}
			ssName.clear();
			ssName << "ydot_" << i + 1;
			ssName >> externalName;
			this->ports()->addPort(externalName, priorities[i]->ydot_port).doc(
					"Desired output velocity with priority of the index");
			if(!priorities[i]->ydot_port.connectTo(the_solverp->peer->ports()->getPort(
					externalName))){log(Error) << "[[addSolver]] unable to connect to ydot_port of priority " << i << endlog();}

			if(inequalitiesPresent){
				ssName.clear();
				ssName << "ydot_max_" << i + 1;
				ssName >> externalName;
				this->ports()->addPort(externalName, priorities[i]->ymax_port);
				if(!priorities[i]->ymax_port.connectTo(the_solverp->peer->ports()->getPort(
						externalName))){log(Error) << "[[addSolver]] unable to connect to ydot_max_port of priority " << i << endlog();}
				ssName.clear();
				ssName << "inequalities_" << i + 1;
				ssName >> externalName;
				this->ports()->addPort(externalName, priorities[i]->Wy_port);
				if(!priorities[i]->inequality_port.connectTo(the_solverp->peer->ports()->getPort(
						externalName))){log(Error) << "[[addSolver]] unable to connect to inequalities_port of priority " << i << endlog();}
			}
		}
	} else
	{
		//give the port at the Scene side a name
		this->ports()->addPort("A_1", priorities[0]->A_port).doc(
				"Generalized Jacobian with priority of the index");
		this->ports()->addPort("Wy_1", priorities[0]->Wy_port).doc(
				"Output weight matrix with priority of the index");
		this->ports()->addPort("ydot_1", priorities[0]->ydot_port).doc(
				"Desired output velocity with priority of the index");
		//connect with it's counterpart at the Solver side
		priorities[0]->A_port.connectTo(the_solverp->peer->ports()->getPort("A"));
		priorities[0]->Wy_port.connectTo(the_solverp->peer->ports()->getPort("Wy"));
		priorities[0]->ydot_port.connectTo(the_solverp->peer->ports()->getPort("ydot"));
		if(inequalitiesPresent){
			this->ports()->addPort("ydot_max_1", priorities[0]->ymax_port).doc(
					"Desired maximum output velocity with priority of the index, in case of equality constraints");
			this->ports()->addPort("inequalities_1", priorities[0]->inequality_port).doc(
					"port sending out information about whether a certain constraint is an inequality");
			priorities[0]->inequality_port.connectTo(the_solverp->peer->ports()->getPort("inequalities"));
			priorities[0]->ymax_port.connectTo(the_solverp->peer->ports()->getPort("ydot_max"));
		}
	}
	return true;
}

// ******************************************************************
// Part of the code used when RUNNING (as in the previous updateHook)
// ******************************************************************

void Scene::calculatePoses()
{
	Logger::In in(this->getName());

	//iterate over all objectFrames of all robots
	RobotMap::iterator robotp;
	ObjectFrameMap::iterator objectFramep;
	for(robotp = robots.begin(); robotp != robots.end(); robotp++)
	{
		Robot* robot = robotp->second;
		if (robot->nq != 0)
		{
			//Get all jacobians
			for (objectFramep = robot->ObjectFrames.begin(); objectFramep != robot->ObjectFrames.end(); ++objectFramep)
			{
				ObjectFrame* objectFrame = objectFramep->second;
#ifndef NDEBUG
				//log(Debug) << "For '" << objectFrame->peer->getName()<< "', get jacobian and weighting matrix" << endlog();
#endif
				//Jq=RelJacobian=cfr.RelTwist(b|ee,b,b) (ref.point b on object ee expressed in b)
				objectFrame->Jq_port.read(objectFrame->Jq);
				//Jq=RelJacobian=cfr.RelTwist(w|ee,b,w) (ref.point w on object ee expressed in w)
				objectFrame->Jq.changeRefFrame(robot->T_w_b.get());
				//write this transformed Jq to a port
				objectFrame->Jq_w_port.write(objectFrame->Jq);

			}
			//Get all joint weights and create Wq_total
#ifndef NDEBUG
			//	log(Debug) << "For '" << objectFrame->peer->getName()<< "', get jacobian and weighting matrix" << endlog();
#endif
			robot->Wq_port.read(robot->Wq_local);
			if(NoData==robot->Wq_global_port.read(robot->Wq_global))
			{
				robot->Wq_global.setIdentity(); //default: objectFrame is switched on
			}
			Wq_total.block(robot->start_index, robot->start_index, robot->nq, robot->nq).noalias()
					= (robot->Wq_global * robot->Wq_local);
		}
	}
	//Calculate robot poses
	//loop all VKCs and IMs
	//TODO operations with VirtualLink pointer instead of VKC or IM pointer! (see notitions projects V, p.1)
	VKCMap::iterator VKCp;
	for (VKCp = VKCs.begin(); VKCp != VKCs.end(); ++VKCp)
	{
		VirtualKinematicChainStruct* VKC = VKCp->second;

		//initializations
		JuXudot = VectorXd::Zero(6);
		SetToZero(JuXudot_kdl);

		//Calculate robot poses:
		//	T_b_e=RelPose(ee|ee,b1|b1,b1) (pose of ee on body ee wrt. b1 on body b1 expressed in b1)
		VKC->objectFrame1->T_b_e_port.read(VKC->T_b_ee1);
		//	T_w_o1=RelPose(o1|o1,w|w,w) (pose of o1 on body o1 wrt. w on body w expressed in w)
		VKC->T_w_o1 = VKC->robot1->T_w_b.get() * VKC->T_b_ee1;
		//	T_b_e=RelPose(ee|ee,b2|b2,b2) (pose of ee on body ee wrt. b2 on body b2 expressed in b2)
		VKC->objectFrame2->T_b_e_port.read(VKC->T_b_ee2);
		//	T_w_o2=RelPose(o2|o2,w|w,w) (pose of o2 on body o2 wrt. w on body w expressed in w)
		VKC->T_w_o2 = VKC->robot2->T_w_b.get() * VKC->T_b_ee2;
		//	T_o1_o2=RelPose(o2|o2,o1|o1,o1) (pose of o2 on body o2 wrt. o1 on body o1 expressed in o1)
		VKC->T_o1_o2 = VKC->T_w_o1.Inverse() * VKC->T_w_o2;
		VKC->T_o1_o2_port.write(VKC->T_o1_o2);
	}

	//loop all priorities
	for (unsigned int m = 0; m < priorities.size(); m++)
	{
		//FOR EACH CONSTRAINT: loop over each constraint of a priority
		ConstraintControllerMap::iterator constraintp;
		for (constraintp = priorities[m]->constraints.begin(); constraintp
				!= priorities[m]->constraints.end(); ++constraintp)
		{
			ConstraintControllerStruct* constraint = constraintp->second;
			//read joint positions
			constraint->robot1->q_port.read(constraint->robot1->q);
			if(!(constraint->sameRobot))
			{
				constraint->robot2->q_port.read(constraint->robot2->q);
			//hand it out
				constraint->q12_local.data.head(constraint->robot1->nq) = constraint->robot1->q.data;
				constraint->q12_local.data.tail(constraint->robot2->nq) = constraint->robot2->q.data;
			}else
			{
				constraint->q12_local.data = constraint->robot1->q.data;
			}
			//put it on the ports
			constraint->q12_port.write(constraint->q12_local);
		}
	}
}

void Scene::calculateA()
{
	Logger::In in(this->getName());
	//loop all VKC
	VKCMap::iterator VKCp;
	for (VKCp = VKCs.begin(); VKCp != VKCs.end(); ++VKCp)
	{
		VirtualKinematicChainStruct* VKC = VKCp->second;
		//Get Jf, transform it to the base and invert it.
		//	Jf=RelJacobian=cfr.RelTwist(o1|o2,o1,o1) (ref.point o1 on object o2 expressed in o1)
		VKC->Jf_port.read(VKC->Jf);
		//	Jf=RelJacobian=cfr.RelTwist(w|o2,o1,w) (ref.point w on object o2 expressed in w)
		VKC->Jf.changeRefFrame(VKC->T_w_o1);
		if (0 != svd_eigen_HH(VKC->Jf.data, VKC->Uf, VKC->Sf, VKC->Vf, temp))
		{
			log(Error)<< "Could not invert virtual linkage jacobian of VirtualKinematicChainStruct "
					<< VKC->peer->getName();
			this->fatal();
		}

#ifndef NDEBUG
		log(Debug) << "Jf inverse of task '" << VKC->peer->getName()<< endlog();
#endif
#ifndef NDEBUG
		log(Debug) << "Jf: " << VKC->Jf.data << endlog();
#endif
#ifndef NDEBUG
		log(Debug) << "Uf: " << VKC->Uf << endlog();
#endif
#ifndef NDEBUG
		log(Debug) << "Sf: " << VKC->Sf << endlog();
#endif
#ifndef NDEBUG
		log(Debug) << "Vf: " << VKC->Vf << endlog();
#endif

		for (unsigned int j = 0; j < (unsigned int) VKC->Sf.size(); j++)
		{
			if (VKC->Sf(j) > 0)
				{VKC->Uf.col(j) *= 1 / VKC->Sf(j);}
			else
				{VKC->Uf.col(j).setZero();}
		}
		VKC->H.noalias() = (VKC->Vf * VKC->Uf.transpose());
#ifndef NDEBUG
		log(Debug) << "H = \n" << VKC->H<< endlog();
#endif
	}

	//loop all priorities
	for (unsigned int m = 0; m < priorities.size(); m++) {
		//FOR EACH CONSTRAINT: loop over each constraint of a priority
		ConstraintControllerMap::iterator constraintp;
		for (constraintp = priorities[m]->constraints.begin(); constraintp
				!= priorities[m]->constraints.end(); ++constraintp)
		{
			ConstraintControllerStruct* constraint = constraintp->second;
			//reset old values
			priorities[m]->CfJfinvJq_priority.setZero(priorities[m]->CfJfinvJq_priority.rows(),priorities[m]->CfJfinvJq_priority.cols());


			// *** YDOT ***
			// get ydot and store in ydot_priority
#ifndef NDEBUG
			//log(Debug) << "For '" << constraint->peer->getName()<< "', get ydot and store in ydot_total" << endlog();
#endif
			//hier nog y_max_local uitlezen, indien nodig! 
			constraint->ydot_port.read(constraint->y_dot_local);
			priorities[m]->ydot_priority.segment(constraint->start_index, constraint->nc) = constraint->y_dot_local.data;

			if(inequalitiesPresent){//scene has inequalities!
				bool hasinequalities = (priorities[m]->inequalities.find(constraintp->first))->second;

				if(hasinequalities){//constraintController has inequalities
					//therefor we are sure we can cast to CCInequalityStruct a this point.
					((ConstraintControllerInequalityStruct*) constraint) -> 
								  	ydotmax_port.read( ((ConstraintControllerInequalityStruct*) constraint)->y_max_local);
					priorities[m]->ydotmax_priority.segment(constraint->start_index, constraint->nc) = 
									((ConstraintControllerInequalityStruct*) constraint)->y_max_local.data;
					//priorities[m]->ydot_inequalities_priority.segment(constraint->start_index, constraint->nc) = 1; 
				}
				else{//constraintController doesn't have inequalities
					//put same values on ydot_max as on regular ydot
					priorities[m]->ydotmax_priority.segment(constraint->start_index, constraint->nc) = constraint->y_dot_local.data;
					//set corresponding segment of inequalities vector to zero, since it's isn't a constraintcontroller
					priorities[m]->ydot_inequalities_priority.segment(constraint->start_index, constraint->nc).setZero();
				}
			}
			// *** A ***

			//get Cq
			constraint->Cq_port.read(constraint->Cq_local);

			if(constraint->constrainedLink == NULL)
			{
#ifndef NDEBUG
				log(Debug) << "There is no virtual link for this task=> A=Cq" << endlog();
#endif
				//put Cq parts in right place determined by the robot (objectFrame1 and objectFrame2 are not necessarily right behind each other in A matrix!)
				if (constraint->robot1->nq != 0)
				{
					priorities[m]->A_priority.block(constraint->start_index,
							constraint->robot1->start_index, constraint->nc, constraint->robot1->nq)
							= constraint->Cq_local.block(0, 0, constraint->nc, constraint->robot1->nq);
				}
				if((!(constraint->sameRobot)) && (constraint->robot2->nq != 0) )
				{
					priorities[m]->A_priority.block(constraint->start_index,
							constraint->robot2->start_index, constraint->nc, constraint->robot2->nq)
							= constraint->Cq_local.block(0, constraint->robot1->nq, constraint->nc, constraint->robot2->nq);
				}
			}else{
				//get Cf
				constraint->Cf_port.read(constraint->Cf_local);
#ifndef NDEBUG
				log(Debug) << "priority = " << m+1 << endlog();
#endif
#ifndef NDEBUG
				log(Debug) << "Cf = \n" << constraint->Cf_local << endlog();
#endif

				//tmpCfJf_priority = Cf*Jf^-1
				priorities[m]->tmpCfJf_priority.block(constraint->start_index, 0,
						constraint->nc, 6).noalias() = -(constraint->Cf_local * constraint->constrainedLink->H);
#ifndef NDEBUG
				//log(Debug) << "CfJf: " << priorities[m]->tmpCfJf_priority.block(constraint->start_index, 0, constraint->nc,6) << endlog();
#endif

				//UNCERTAINTY (TODO does not work for objectFrames on same robot yet(uncertainty should be split out))
				if(!(constraint->sameRobot))
				{
					//Add Cf*Jf_inv*JuXudot=B*Xudot to ydot
					//	Part of objectFrame1
					//		JuXudot_kdl=RelTwist(b1|ee1, b1, b1) (ref.point b1 on object ee1 expressed in b1)
					constraint->robot1->JuXudot_port.read(JuXudot_kdl);
					//	Transform to world reference frame
					//		JuXudot(_kdl)=RelTwist(w|ee1,b1,w)  (ref.point w on object ee1 expressed in w)
					JuXudot_kdl = constraint->robot1->T_w_b.get() * JuXudot_kdl;
					JuXudot << Map<Vector3d> (JuXudot_kdl.vel.data), Map<Vector3d> (
							JuXudot_kdl.rot.data);
					priorities[m]->ydot_priority.segment(constraint->start_index, constraint->nc).noalias()
							+= (priorities[m]->tmpCfJf_priority.block(
									constraint->start_index, 0, constraint->nc, 6) * JuXudot);
					//	Part of objectFrame2
					//		JuXudot_kdl=RelTwist(b2|ee2, b2, b2) (ref.point b12 on object ee2 expressed in b2)
					constraint->robot2->JuXudot_port.read(JuXudot_kdl);
					//	Transform to world reference frame
					//		JuXudot(_kdl)=RelTwist(w|ee2,b2,w)  (ref.point w on object ee2 expressed in w)
					JuXudot_kdl = constraint->robot2->T_w_b.get() * JuXudot_kdl;
					JuXudot << Map<Vector3d> (JuXudot_kdl.vel.data), Map<Vector3d> (
							JuXudot_kdl.rot.data);
					priorities[m]->ydot_priority.segment(constraint->start_index, constraint->nc).noalias()
							-= (priorities[m]->tmpCfJf_priority.block(
									constraint->start_index, 0, constraint->nc, 6) * JuXudot);
				}
				// multiply -Cf*Jf^-1 with Jq
				//		the rows of CfJfinvJq_priority are empty at this stage
				if (constraint->robot1->nq != 0)
					priorities[m]->CfJfinvJq_priority.block(constraint->start_index, constraint->robot1->start_index, constraint->nc, constraint->robot1->nq).noalias()
							= (priorities[m]->tmpCfJf_priority.block(
									constraint->start_index, 0, constraint->nc, 6)
									* constraint->objectFrame1->Jq.data);
				//		the rows of CfJfinvJq_priority are NOT empty at this stage if the objectFrames are on the same robot => substract from current value
				if (constraint->robot2->nq != 0)
					priorities[m]->CfJfinvJq_priority.block(constraint->start_index, constraint->robot2->start_index, constraint->nc, constraint->robot2->nq).noalias()
					=
					priorities[m]->CfJfinvJq_priority.block(constraint->start_index, constraint->robot2->start_index, constraint->nc, constraint->robot2->nq)
					-(priorities[m]->tmpCfJf_priority.block(constraint->start_index, 0, constraint->nc, 6)* constraint->objectFrame2->Jq.data);

				//add Cq to -Cf*Jf^-1*Jq, and store Cq-Cf*Jf^-1*Jq in A
				// A=RelJac=cfr.RelTwist(w|?,?,w) (ref.point w expressed in w)
#ifndef NDEBUG
				//log(Debug) << "For '" << constraint->peer->getName()<< "', store -Cf*Jf^-1*Jq in A" << endlog();
#endif
				if (constraint->robot1->nq != 0)
				{
					priorities[m]->A_priority.block(
							constraint->start_index,constraint->robot1->start_index, constraint->nc, constraint->robot1->nq)
							= priorities[m]->CfJfinvJq_priority.block(
									constraint->start_index, constraint->robot1->start_index, constraint->nc, constraint->robot1->nq)
							  +constraint->Cq_local.block(0, 0, constraint->nc, constraint->robot1->nq);
				}
				if((!(constraint->sameRobot)) && (constraint->robot2->nq != 0) )
				{
					priorities[m]->A_priority.block(
							constraint->start_index, constraint->robot2->start_index, constraint->nc, constraint->robot2->nq)
							= priorities[m]->CfJfinvJq_priority.block(
									constraint->start_index, constraint->robot2->start_index, constraint->nc, constraint->robot2->nq)
							  +constraint->Cq_local.block(0, constraint->robot1->nq, constraint->nc, constraint->robot2->nq);
				}
			}

			// *** Wy ***
			//store weight for this constraint in priorities[m]->Wy_priority
#ifndef NDEBUG
			//log(Debug) << "For '" << constraint->peer->getName()<< "', store weight for this constraint in priorities[m]->Wy_priority" << endlog();
#endif
			constraint->Wy_port.read(constraint->Wy_local);
			if(NoData==constraint->Wy_global_port.read(constraint->Wy_global))
			{
				constraint->Wy_global=0.0; //default: constraint is switched off
			}

			priorities[m]->Wy_priority.block(constraint->start_index,
					constraint->start_index, constraint->nc, constraint->nc).noalias()
					= (constraint->Wy_global * constraint->Wy_local);
		}

		priorities[m]->A_port.write(priorities[m]->A_priority);
		priorities[m]->Wy_port.write(priorities[m]->Wy_priority);
		priorities[m]->ydot_port.write(priorities[m]->ydot_priority);
		if(inequalitiesPresent){//scene has inequalities, also write the ymax and inequalities ports to the solver
			priorities[m]->ymax_port.write(priorities[m]->ydotmax_priority);
			priorities[m]->inequality_port.write(priorities[m]->ydot_inequalities_priority);
		}
#ifndef NDEBUG
		//log(Debug) << "A_" << m << " = " << priorities[m]->A_priority << endlog();
#endif
		//log(Info) << "Wy written for priority " << m << " = " << priorities[m]->Wy_priority << endlog();
	}
	Wq_port.write(Wq_total);
}

void Scene::handOut()
{
	Logger::In in(this->getName());

	qdot_port.read(qdot_total);
#ifndef NDEBUG
	//log(Debug) << "qdot returned from Solver = \n" << qdot_total << endlog();
#endif

	//Send result to robots:
	RobotMap::iterator robotp;
	for (robotp = robots.begin(); robotp != robots.end(); ++robotp) {
		Robot* robot = robotp->second;
#ifndef NDEBUG
		//log(Debug) << "For '" << robot->peer->getName()<< "', send resulting qdot to robot" << endlog();
#endif
		if (robot->nq != 0) {
			robot->qdot.data
					= qdot_total.segment(robot->start_index, robot->nq);
			robot->qdot_port.write(robot->qdot);
		}
	}

	//Send result to VKC's and IM's:
	//loop all VKC's and IM's
	//TODO loop also over IM's
	VKCMap::iterator VKCit;
	for (VKCit = VKCs.begin(); VKCit!= VKCs.end(); ++VKCit)
	{
		VirtualKinematicChainStruct* VKCstructp = VKCit->second;
#ifndef NDEBUG
		//log(Debug) << "For '" << constraint->peer->getName()<< "', send resulting Jq_qdot to robot" << endlog();
#endif
		t1 = Twist::Zero();
		t2 = Twist::Zero();
		//t1=RelTwist(w|ee1, b1, w) (ref.point w on object ee1 expressed in w)
		if (VKCstructp->robot1->nq != 0)
			MultiplyJacobian(VKCstructp->objectFrame1->Jq, VKCstructp->robot1->qdot, t1);
		//t2=RelTwist(w|ee2, b2, w) (ref.point w on object ee2 expressed in w)
		if (VKCstructp->robot2->nq != 0)
			MultiplyJacobian(VKCstructp->objectFrame2->Jq, VKCstructp->robot2->qdot, t2);
		//log(Info) << "priority = " << m+1 << endlog();
		//log(Info) << "Jq_qdot = t1-t2 = \n"<< t1 << "\n - \n" << t2 << endlog();
		//WARNING: hidden assumption that t_b2_b1=0=RelTwist(w|b1,b2,w)
		//Jq_qdot=t1-t2=RelTwist(w|ee1,ee2,w)  (ref.point w on object ee1 expressed in w)
		Jq_qdot_w = t1-t2;
		//change refFrame = (ref Base and ref Point change) from w to o1=T_w_o1.Inverse()*Jq_qdot_w
		Jq_qdot = VKCstructp->T_w_o1.Inverse(Jq_qdot_w);
		//Jq_qdot=RelTwist(o1|o1,o2,o1)  (ref.point o1 on object o1 expressed in o1) o1=ee1 and o2=ee2
		VKCstructp->Jq_qdot_port.write(Jq_qdot);
	}
}

}//End of namespace


