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
unlockRobotTable      = {}

local Scene
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
    -- tables
	robotTableProp=rtt.Property("string[]", "robotTable", "Table of robot component names")
	tc:addProperty(robotTableProp)
    taskTableProp=rtt.Property("string[]", "taskTable", "Table of task names")
    tc:addProperty(taskTableProp)
    solverTableProp=rtt.Property("string[]", "solverTable", "Table of solver component names")
    tc:addProperty(solverTableProp)
    -- scene
    sceneProp=rtt.Property("string", "scene", "name of the scene component")
    tc:addProperty(sceneProp)

    -- fill in standard values for the properties
    application_timer_id:set(1)
    itasc_fsm_package_prop:set("")
    composite_task_fsm_package_prop:set("")
    running_itasc_coordination_package_prop:set("")
	itasc_fsm_prop:set(rttros.find_rospack("itasc_core") .. "/scripts/reduced_itasc_fsm.lua")
    running_itasc_coordination_prop:set(rttros.find_rospack("itasc_core") .."/scripts/running_itasc_coordination.lua")
    composite_task_fsm_prop:set(rttros.find_rospack("itasc_core") .."/scripts/reduced_composite_task_fsm.lua")
    sceneProp:set("Scene")
	
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
    rtt.logl("Info"," starting") 
    --get tables from properties
    if not robotTableProp:get() then
      rtt.logl("Error","No robotTable property set!")
    end
    if not taskTableProp:get() then
      rtt.logl("Error","No taskTable property set!")
    end
    if not solverTableProp:get() then
      rtt.logl("Error","No solverTable property set!")
    end
    if not sceneProp:get() then
      rtt.logl("Error","No scene property set!")
    end
    local rttRobotTable = robotTableProp:get()
    local rttTaskTable  = taskTableProp:get()
    local rttSolverTable= solverTableProp:get()
    Scene = sceneProp:get() 
    -- rtt tables start from 0, lua tables from 1!
    for i=0,rttRobotTable.size-1 do
      robotTable[#robotTable+1] = rttRobotTable[i]
    end
    for i=0,rttTaskTable.size-1 do
      taskTable[#taskTable+1] = rttTaskTable[i]
    end
    for i=0,rttSolverTable.size-1 do
      solverTable[#solverTable+1] = rttSolverTable[i]
    end
    if(composite_task_fsm_package_prop:get()=="")
    then
        rtt.logl("Warning"," No composite_task_fsm_package specified, will look for fsm file on location specified by composite_task_fsm property")    
        composite_task_fsm_file = composite_task_fsm_prop:get()
    else
        composite_task_fsm_file = rttros.find_rospack(composite_task_fsm_package_prop:get()) .. composite_task_fsm_prop:get()
    end
    if(running_itasc_coordination_package_prop:get()=="")
    then
        rtt.logl("Warning"," No running_itasc_coordination_package specified, will look for coordination file on location specified by running_itasc_coordination property")    
        running_itasc_coordination_file = running_itasc_coordination_prop:get()
    else
        running_itasc_coordination_file = rttros.find_rospack(running_itasc_coordination_package_prop:get()) .. running_itasc_coordination_prop:get()
    end
    if(itasc_fsm_package_prop:get()=="")
    then
        rtt.logl("Warning"," No itasc_fsm_package specified, will look for fsm file on location specified by itasc_fsm property")    
        itasc_fsm_file = itasc_fsm_prop:get()
    else
        itasc_fsm_file = rttros.find_rospack(itasc_fsm_package_prop:get()) .. itasc_fsm_prop:get()
    end

	-- load state machine
	rtt.logl("Info"," loading FSM: " .. itasc_fsm_file )
	fsm = rfsm.init(rfsm.load(itasc_fsm_file))

	-- get all events from the all input ports
	-- getevents function, which returns all data on the current port as
	-- events. This function is called by the rFSM core to check for new events.
	fsm.getevents = rfsm_rtt.gen_read_str_events(common_events_in, priority_events_in, trigger_events_in)
	--fsm.dbg=rfsmpp.gen_dbgcolor("itasc_fsm",{STATE_ENTER=true, STATE_EXIT=true,EFFECT=true,EXEC_PATH=true},false)
	-- optional: create a string port to which the currently active
	-- state of the FSM will be written. gen_write_fqn generates a
	-- function suitable to be added to the rFSM step hook to do this.
	fqn_out = rtt.OutputPort("string")
	tc:addPort(fqn_out, "currentState", "current active rFSM state")
	rfsm.post_step_hook_add(fsm,rfsm_rtt.gen_write_fqn(fqn_out))
	
	--raise event functions
	raise_common_event=rfsm_rtt.gen_raise_event(common_events_out, fsm)		
	raise_priority_event=rfsm_rtt.gen_raise_event(priority_events_out, fsm)
	raise_trigger_event=rfsm_rtt.gen_raise_event(trigger_events_out)
	
	-- Functions containing RTT specific info to request operations
	SceneCalculatePoses = peertable[Scene]:getOperation("calculatePoses")
	SceneCalculateA = peertable[Scene]:getOperation("calculateA")
	SceneSolvers_solve = peertable.Solver:getOperation("solve")
	SceneHandOut = peertable[Scene]:getOperation("handOut")

    return true
end

local e_TimerTrigger = 'e_TimerTrigger'
timer_id_in = rtt.Variable('int')
function updateHook() 

	timer_id_in_fs = time_trigger:read(timer_id_in)
	if timer_id_in_fs=="NewData" then
	   if timer_id_in:tolua()==application_timer_id:get()then
	      rfsm.send_events(fsm, e_TimerTrigger)
	      rfsm.run(fsm)
	   end
	else 
	   rfsm.run(fsm)
	end 
end

function cleanupHook()
    rttlib.tc_cleanup()
end

function createOperationTables()
    for i,v in pairs(robotTable) do
      if peertable[v]:hasOperation("updateRobotState") then
	  updateRobotStateTable[#updateRobotStateTable+1] = peertable[v]:getOperation("updateRobotState")
      else
        rtt.logl("Error",v.." has no required operation updateRobotState")
		raise_common_event("e_emergency") 
      end
	  -- create table with sendToRobot operations
      if peertable[v]:hasOperation("sendToRobot") then
	  sendToRobotTable[#sendToRobotTable+1] = peertable[v]:getOperation("sendToRobot")
      else
        rtt.logl("Error",v.." has no required operation sendToRobot")
		raise_common_event("e_emergency") 
      end
      if peertable[v]:hasOperation("unlockRobotAxes") then
        unlockRobotTable[#unlockRobotTable+1] = peertable[v]:getOperation("unlockRobotAxes")
      else
        rtt.logl("Warning",v.." has no operation unlockRobotAxes")
		--raise_common_event("e_emergency") 
      end
      if peertable[v]:hasOperation("lockRobotAxes") then
        lockRobotTable[#lockRobotTable+1] = peertable[v]:getOperation("lockRobotAxes")
      else
        rtt.logl("Warning",v.." has no operation lockRobotAxes")
		--raise_common_event("e_emergency") 
      end
    end
end

--- Function containing RTT specific info to configure the Solvers
function configureSolvers()
	for i=1,#solverTable do
		if peertable[solverTable[i]]:configure() then
			rtt.logl("Info","   " .. solverTable[i] .. " configured") 
		else
			rtt.logl("Error","   unable to configure " .. solverTable[i])
			raise_common_event("e_emergency") 
		end
	end
end

--- Function containing RTT specific info to configure the objectFrames of robots and objects
function configureObjectFrames()
	for i=1,#robotTable do
        rtt.logl("Info","Trying to configure the object frames of "..robotTable[i])
		if peertable[robotTable[i]]:configureObjectFrames() then
			rtt.logl("Info","   " .. robotTable[i] .. "-objectframes configured") 
		else 
			rtt.logl("Error","   unable to configure " .. robotTable[i] .. "-objectframes")
			raise_common_event("e_emergency") 
		end
	end
end

--- Function containing RTT specific info to broadcastObjectFrames
function broadcastObjectFrames()
	if peertable[Scene]:broadcastObjectFrames() then print ("   Scene broadcasted object frames") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to connectScene2Solver
function connectScene2Solver()
	if peertable[Scene]:connectScene2Solver() then rtt.logl("Info","   Scene connected to Solver") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to connectScene2Robots
function connectScene2Robots()
	if peertable[Scene]:connectScene2Robots() then rtt.logl("Info","   Scene connected to Robots") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to configure the Scene
function configureScene()
	if peertable[Scene]:configure() then rtt.logl("Info","   Scene configured") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to stop all itasc level components
function startRobots()
	for i=1,#robotTable do
		if peertable[robotTable[i]]:start() then
			rtt.logl("Info","   " .. robotTable[i] .. " started") 
		else
			rtt.logl("Error","   unable to start " .. robotTable[i]) 		
			raise_common_event("e_emergency") 
		end
	end	
end

--- Function containing RTT specific info to stop all itasc level components
function stopAllComponents()
	peertable[Scene]:stop()
	for i=1,#robotTable do
		if peertable[robotTable[i]]:stop() then
			rtt.logl("Info","   " .. robotTable[i] .. " stopped") 
		else 
			rtt.logl("Error","   unable to stop " .. robotTable[i])	
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

--unlock axes
--- Function containing RTT specific info
function unlockRobotAxes()
    if #unlockRobotTable > 0 then
	  for k, unlockRob in ipairs(unlockRobotTable) do
	  	if not unlockRob() then
            rtt.logl("Error","unable to unlock the axes of "..robotTable[k])
            raise_common_event("e_emergency") 
          end
	  end
    end
end

--lock axes
--- Function containing RTT specific info
function lockRobotAxes()
	for k, lockRob in ipairs(lockRobotTable) do
		if not lockRob() then
          rtt.logl("Error","unable to lock the axes of "..robotTable[k])
          raise_common_event("e_emergency") 
        end
	end
end
