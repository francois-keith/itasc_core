--           This file is part of the iTaSC project                      
--                                                                       
--                  (C) 2011 Dominick Vanthienen                         
--              dominick.vanthienen@mech.kuleuven.be,                    
--              Department of Mechanical Engineering,                    
--             Katholieke Universiteit Leuven, Belgium.                  
--                    http://www.orocos.org/itasc                  
--                                                                       
-- You may redistribute this software and/or modify it under either the  
-- terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  
-- <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your 
-- discretion) of the Modified BSD License:                              
-- Redistribution and use in source and binary forms, with or without    
-- modification, are permitted provided that the following conditions    
-- are met:                                                              
-- 1. Redistributions of source code must retain the above copyright     
-- notice, this list of conditions and the following disclaimer.         
-- 2. Redistributions in binary form must reproduce the above copyright  
-- notice, this list of conditions and the following disclaimer in the   
-- documentation and/or other materials provided with the distribution.  
-- 3. The name of the author may not be used to endorse or promote       
-- products derived from this software without specific prior written    
-- permission.                                                           
-- THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  
-- IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        
-- WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    
-- ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
-- INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    
-- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       
-- OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
-- HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   
-- STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
-- IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    
-- POSSIBILITY OF SUCH DAMAGE.                                           

require "rttlib"
require "rfsm"
require "rfsm_emem" --needed for event memory
require "rfsm_rtt"
require "rfsm_ext"  --needed for the sequential-AND state
require "rfsmpp"    --needed for the sequential-AND state 
require "kdlpp"     --kdl pritty print (should be included in lua path!)
require "rttros"    --needed for 'find_rospack'

tc=rtt.getTC()
local common_events_in, priority_events_in
local timer_id_in_fs ,timer_id_in

taskTable      = {}
robotTable     = {}
solverTable    = {}
updateRobotStateTable = {}
sendToRobotTable      = {}

local prev_timertrigger_time={}
local current_timertrigger_time={}

function configureHook() 
	
	-- peer table (to enable smaller code to request operations)
	peertable = rttlib.mappeers(function (tc) return tc end, tc)
	--print("[itascSuperVisor.lua] SuperVisor has following peers:")
	--for K,V in pairs(peertable) do print( K) end
	
	-- PROPERTIES
	-- create a timer id property
	application_timer_id=rtt.Property("int", "application_timer_id", "Timer ID of the application timer")
	tc:addProperty(application_timer_id)
    -- location of the itasc_configuration
    itasc_configuration_package_prop=rtt.Property("string","itasc_configuration_package","package where to find the itasc_configuration, if any")
    tc:addProperty(itasc_configuration_package_prop)
    itasc_configuration_prop=rtt.Property("string", "itasc_configuration", "path and name to the itasc_configuration, starting from the package (start with a slash), if any")
    tc:addProperty(itasc_configuration_prop)
	-- location of the itasc level FSMs
    itasc_fsm_package_prop=rtt.Property("string","itasc_fsm_package","package where to find the itasc_fsm, if any")
    tc:addProperty(itasc_fsm_package_prop)
	itasc_fsm_prop=rtt.Property("string", "itasc_fsm", "path and name to the itasc_fsm file, starting from the package (start with a slash), if any")
	tc:addProperty(itasc_fsm_prop)
	-- location of the composite_task_fsm
    composite_task_fsm_package_prop=rtt.Property("string","composite_task_fsm_package","package where to find the composite_task_fsm, if any")
    tc:addProperty(composite_task_fsm_package_prop)
	composite_task_fsm_prop=rtt.Property("string", "composite_task_fsm", "path and name to the composite_task_fsm file, starting from the package (start with a slash), if any")
	tc:addProperty(composite_task_fsm_prop)
	-- location of the running_itasc_coordination
    running_itasc_coordination_package_prop=rtt.Property("string","running_itasc_coordination_package","package where to find the running_itasc_coordination, if any")
    tc:addProperty(running_itasc_coordination_package_prop)
	running_itasc_coordination_prop=rtt.Property("string", "running_itasc_coordination", "path and name to the composite_task_fsm_prop file, starting from the package (start with a slash), if any")
	tc:addProperty(running_itasc_coordination_prop)

    -- fill in standard values for the properties
    application_timer_id:set(1)
    itasc_fsm_package_prop:set("")
    composite_task_fsm_package_prop:set("")
    running_itasc_coordination_package_prop:set("")
	itasc_fsm_prop:set(rttros.find_rospack("itasc_core") .. "/scripts/itasc_fsm.lua")
    running_itasc_coordination_prop:set(rttros.find_rospack("itasc_core") .."/scripts/running_itasc_coordination.lua")
    composite_task_fsm_prop:set(rttros.find_rospack("itasc_core") .."/scripts/composite_task_fsm.lua")
	
	-- INPUT PORTS 

	-- Port to recieve trigger from a timer
	time_trigger = rtt.InputPort("int")
	tc:addEventPort(time_trigger,"trigger","Port to recieve trigger from a timer")

	-- COMMON events: the following creates a string input port, adds it as a 
	-- port to the Taskcontext. 
	common_events_in = rtt.InputPort("string")
	tc:addPort(common_events_in, "itasc_common_events_in", "rFSM common_event input port")

	-- PRIORITY events: the following creates a string input port, adds it as an event
	-- driven port to the Taskcontext.    
	priority_events_in = rtt.InputPort("string")
	tc:addEventPort(priority_events_in, "itasc_priority_events_in", "rFSM priority_event input port")

	-- TRIGGER events: the following creates a string input port, adds it as an event
	-- driven port to the Taskcontext. 
	trigger_events_in = rtt.InputPort("string")
	tc:addEventPort(trigger_events_in, "itasc_trigger_events_in", "itasc trigger_event input port")

	--OUTPUT PORTS

	-- create a string port with which the current common events are send
	common_events_out = rtt.OutputPort("string")
	tc:addPort(common_events_out, "itasc_common_events_out", "current common_events in iTaSCFSM")
	   
	-- create a string port with which the current PRIORITY events are send	   
	priority_events_out = rtt.OutputPort("string")
	tc:addPort(priority_events_out, "itasc_priority_events_out", "current priority_events in iTaSCFSM")

	-- create a string port with which the current TRIGGER events are send
	trigger_events_out = rtt.OutputPort("string")
	tc:addPort(trigger_events_out, "itasc_trigger_events_out", "current trigger_events in iTaSCFSM")

	return true	
end

function startHook()
    print("[itasc_supervisor.lua] starting") 
    -- getting the file locations 
    if(itasc_configuration_package_prop:get()=="")
    then
        print("[itasc_supervisor.lua] No itasc_configuration_package specified, will look for configuration file on location specified by itasc_configuration property")    
        itasc_configuration_file = itasc_configuration_prop:get()
    else
        itasc_configuration_file = rttros.find_rospack(itasc_configuration_package_prop:get()) .. itasc_configuration_prop:get()
    end
    if(composite_task_fsm_package_prop:get()=="")
    then
        print("[itasc_supervisor.lua] No composite_task_fsm_package specified, will look for fsm file on location specified by composite_task_fsm property")    
        composite_task_fsm_file = composite_task_fsm_prop:get()
    else
        composite_task_fsm_file = rttros.find_rospack(composite_task_fsm_package_prop:get()) .. composite_task_fsm_prop:get()
    end
    if(running_itasc_coordination_package_prop:get()=="")
    then
        print("[itasc_supervisor.lua] No running_itasc_coordination_package specified, will look for coordination file on location specified by running_itasc_coordination property")    
        running_itasc_coordination_file = running_itasc_coordination_prop:get()
    else
        running_itasc_coordination_file = rttros.find_rospack(running_itasc_coordination_package_prop:get()) .. running_itasc_coordination_prop:get()
    end
    if(itasc_fsm_package_prop:get()=="")
    then
        print("[itasc_supervisor.lua] No itasc_fsm_package specified, will look for fsm file on location specified by itasc_fsm property")    
        itasc_fsm_file = itasc_fsm_prop:get()
    else
        itasc_fsm_file = rttros.find_rospack(itasc_fsm_package_prop:get()) .. itasc_fsm_prop:get()
    end

	-- load state machine
	print("[itasc_supervisor.lua] loading FSM: " .. itasc_fsm_file )
	fsm = rfsm.init(rfsm.load(itasc_fsm_file))

	-- get all events from the all input ports
	-- getevents function, which returns all data on the current port as
	-- events. This function is called by the rFSM core to check for new events.
	fsm.getevents = rfsm_rtt.gen_read_events(common_events_in, priority_events_in, trigger_events_in)

	-- optional: create a string port to which the currently active
	-- state of the FSM will be written. gen_write_fqn generates a
	-- function suitable to be added to the rFSM step hook to do this.
	fqn_out = rtt.OutputPort("string")
	tc:addPort(fqn_out, "currentState", "current active rFSM state")
	fsm.step_hook=rfsm_rtt.gen_write_fqn(fqn_out)
	
	--raise event functions
	raise_common_event=gen_raise_event(common_events_out, fsm)		
	raise_priority_event=gen_raise_event(priority_events_out, fsm)
	raise_trigger_event=gen_raise_event(trigger_events_out, fsm)
	
	-- Functions containing RTT specific info to request operations
	SceneCalculatePoses = peertable.Scene:getOperation("calculatePoses")
	SceneCalculateA = peertable.Scene:getOperation("calculateA")
	SceneSolvers_solve = peertable.Solver:getOperation("solve")
	SceneHandOut = peertable.Scene:getOperation("handOut")

    return true
end

function updateHook() 
	timer_id_in_fs, timer_id_in = time_trigger:read()
	
	if timer_id_in_fs=="NewData" then
		if timer_id_in==application_timer_id:get()then
			rfsm.send_events(fsm, 'e_TimerTrigger')
			rfsm.run(fsm)
		end
	else 
		rfsm.run(fsm)
	end 
end

function cleanupHook()
	-- cleanup the created ports.
	tc:removePort(common_events_in:info().name)
	tc:removePort(trigger_events_in:info().name)     
	-- cleanup created variables
	common_events_in:delete()
	priority_events_in:delete()
	trigger_events_in:delete()   
	timer_id_in_fs:delete()
	timer_id_in:delete()
	-- cleanup created properties
	tc:removeProperty("application_timer_id")   
end

--- Generate an event raising function.
-- The generated function accepts zero to many arguments and writes
-- them to the given port and optionally to the internal queue of fsm.
-- @param port outport to write events to
-- @param fsm events are sent to this fsm's internal queue (optional)
-- @return function to send events to the port
function gen_raise_event(port, fsm)
	return function (...)
	     	for _,e in ipairs{...} do port:write(e) end
		if fsm then rfsm.send_events(fsm, ...) end
	end
end

--- Function containing RTT specific info to add a robot to the Scene and configure it
function addRobot(robotName, robotLocation)
	-- configure the Robot/Object
	if peertable[robotName]:configure() 
	then print("   " .. robotName .." configured") 
	else 
		print("   unable to configure " .. robotName) 
		raise_common_event("e_emergency") 
	end
	-- add the Robot/Object
	if peertable.Scene:addRobot(robotName, robotLocation)  then print("   [iTaSC SuperVisor]" .. robotName .. " added") else raise_common_event("e_emergency") end
	robotTable[#robotTable+1] = robotName --we'll start counting from 1 !!!
	-- create table with updateRobotState operations
	updateRobotStateTable[#updateRobotStateTable+1] = peertable[robotName]:getOperation("updateRobotState")
	-- create table with sendToRobot operations
	sendToRobotTable[#sendToRobotTable+1] = peertable[robotName]:getOperation("sendToRobot")
	
end

--- Function containing RTT specific info to add an object to the Scene
function addObjectFrame(objectFrameName, segmentName, robotName)
	if peertable.Scene:addObjectFrame(objectFrameName, segmentName, robotName)  then print("   [iTaSC SuperVisor] object frame " .. objectFrameName .. " on robot " .. robotName .. " added") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to add a Constraint/Controller to the Scene
function addConstraintController(taskName, objectFrameName1, objectFrameName2, VirtualLinkName , priority)
  	vecToSend = {}
  	vecToSend[1] = objectFrameName1
  	vecToSend[2] = robot1
  	if peertable.Scene:addConstraintController(taskName, objectFrameName1, objectFrameName2, VirtualLinkName , priority)  then print("   [iTaSC SuperVisor]" .. taskName .. " added") else raise_common_event("e_emergency") end
	local croppedTaskName = string.gsub(taskName,"CC_","") --cut the CC_ part from the taskName
	taskTable[#taskTable+1] = croppedTaskName --we'll start counting from 1 !!!
end

--- Function containing RTT specific info to add a VirtualKinematicChain to the Scene
function addVirtualKinematicChain(taskName, objectFrameName1, objectFrameName2)
	if peertable.Scene:addVirtualKinematicChain(taskName, objectFrameName1, objectFrameName2)  then print("   [iTaSC SuperVisor]" .. taskName .. " added") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to add a Solver to the Scene
function addSolver(solverName)
  	if peertable.Scene:addSolver(solverName) then print("   [iTaSC SuperVisor]" .. solverName .. " added") else raise_common_event("e_emergency") end
	solverTable[#solverTable+1] = solverName --we'll start counting from 1 !!!
end

--- Function containing RTT specific info to configure the Solvers
function configureSolvers()
	for i=1,#solverTable do
		if peertable[solverTable[i]]:configure() then
			print("   " .. solverTable[i] .. " configured") 
		else
			print("   unable to configure " .. solverTable[i])
			raise_common_event("e_emergency") 
		end
	end
end

--- Function containing RTT specific info to configure the objectFrames of robots and objects
function configureObjectFrames()
	for i=1,#robotTable do
		if peertable[robotTable[i]]:configureObjectFrames() then
			print("   " .. robotTable[i] .. "-objectframes configured") 
		else 
			print("   unable to configure " .. robotTable[i] .. "-objectframes")
			raise_common_event("e_emergency") 
		end
	end
end

--- Function containing RTT specific info to broadcastObjectFrames
function broadcastObjectFrames()
	if peertable.Scene:broadcastObjectFrames() then print ("   Scene broadcasted object frames") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to connectScene2Solver
function connectScene2Solver()
	if peertable.Scene:connectScene2Solver() then print("   Scene connected to Solver") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to connectScene2Robots
function connectScene2Robots()
	if peertable.Scene:connectScene2Robots() then print("   Scene connected to Robots") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to configure the Scene
function configureScene()
	if peertable.Scene:configure() then print("   Scene configured") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to configure the Reporter
function configureReporter()
	if peertable.Reporter:configure() then print("   Reporter configured") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to stop all itasc level components
function startRobots()
	for i=1,#robotTable do
		if peertable[robotTable[i]]:start() then
			print("   " .. robotTable[i] .. " started") 
		else
			print("   unable to start " .. robotTable[i]) 		
			raise_common_event("e_emergency") 
		end
	end	
end

--- Function containing RTT specific info to stop all itasc level components
function stopAllComponents()
	peertable.Scene:stop()
	for i=1,#robotTable do
		if peertable[robotTable[i]]:stop() then
			print("   " .. robotTable[i] .. " stopped") 
		else 
			print("   unable to stop " .. robotTable[i])	
			raise_common_event("e_emergency") 
		end
	end	
end

--- Functions to create the necessary functions for the coordination of the iTaSC algorithm
function updateRobots()
	for l, opURS in ipairs(updateRobotStateTable) do
		opURS()
	end
end

function sendToRobot()
	for k, opSTR in ipairs(sendToRobotTable) do
		opSTR()
	end
end
