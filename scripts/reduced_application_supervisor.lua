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

-- tc=Task Context of the component we are in ( in this case ApplicationSuperVisor)
tc=rtt.getTC()
local common_events_in, priority_events_in
local timer_id_in_fs

setpointGeneratorTable = {}
driverTable = {}
compositeTable = {}
defaultCompositeTable = rtt.Variable('strings')
defaultCompositeTable:resize(1)
defaultCompositeTable:fromtab{"ITASC"}

function configureHook()  
	-- Peer table (to enable smaller code to request operations)
	-- Mappeers does an in dept search (also looks for peers of peers...) 
	ApplicationSupPeertable = rttlib.mappeers(function (tc) return tc end, tc)
	--print("[ApplicationSuperVisor.lua] ApplicationSuperVisor has following peers:")
	--for K,V in pairs(ApplicationSupPeertable) do print( K) end
	
	-- PROPERTIES
	-- create a timer id property
	application_timer_id=rtt.Property("int", "application_timer_id", "Timer ID of the application timer")
	tc:addProperty(application_timer_id)
    application_fsm_package_prop=rtt.Property("string","application_fsm_package","package where to find the application_fsm, if any")
    tc:addProperty(application_fsm_package_prop)
	application_fsm_prop=rtt.Property("string", "application_fsm", "path and name to the FSM file of the application, starting from the package (start with a slash), if any")
	tc:addProperty(application_fsm_prop)
    -- tables
    setpointGeneratorTableProp=rtt.Property("string[]", "setpointGeneratorTable", "Table of trajectory generator names")
    tc:addProperty(setpointGeneratorTableProp)
    driverTableProp=rtt.Property("string[]", "driverTable", "Table of driver names")
    tc:addProperty(driverTableProp)
    compositeTableProp=rtt.Property("string[]", "compositeTable", "Table of names of supervisors at the composite/itasc (=application-1) level")
    tc:addProperty(compositeTableProp)

    -- fill in standard values for the properties (this will cause it to work as in previous versions)
    application_timer_id:set(1)
    application_fsm_package_prop:set("")
	application_fsm_prop:set(rttros.find_rospack("itasc_core") .. "/scripts/reduced_application_fsm.lua")
    compositeTableProp:set(defaultCompositeTable) 

	-- INPUT PORTS 

	-- Port to receive trigger from a timer
	time_trigger = rtt.InputPort("int")
	tc:addEventPort(time_trigger,"trigger","Port to recieve trigger from a timer")

	-- the following creates a string input port, adds it as a 
	-- port to the Taskcontext. The third line generates a
	-- getevents function, which returns all data on the current port as
	-- events. This function is called by the rFSM core to check for
	-- new events.
	common_events_in = rtt.InputPort("string")
	tc:addPort(common_events_in, "application_common_events_in", "rFSM common_event input port")

	-- the following creates a string input port, adds it as an event
	-- driven port to the Taskcontext. The third line generates a
	-- getevents function, which returns all data on the current port as
	-- events. This function is called by the rFSM core to check for
	-- new events. (usefull for eg. e_stop event)
	priority_events_in = rtt.InputPort("string")
	tc:addEventPort(priority_events_in, "application_priority_events_in", "rFSM priority_event input port")

	-- TRIGGER events: the following creates a string input port, adds it as an event
	-- driven port to the Taskcontext. 
	trigger_events_in = rtt.InputPort("string")
	tc:addEventPort(trigger_events_in, "application_trigger_events_in", "application trigger_event input port")

	-- OUTPUT PORTS

	-- create a string port with which the current common events are send
	common_events_out = rtt.OutputPort("string")
	tc:addPort(common_events_out, "application_common_events_out", "current common events in ApplicationFSM")

	-- create an event driven string port with which the current priority events are send
	priority_events_out = rtt.OutputPort("string")
	tc:addPort(priority_events_out, "application_priority_events_out", "current priority events in ApplicationFSM")

	-- create a string port with which the current TRIGGER events are send
	trigger_events_out = rtt.OutputPort("string")
	tc:addPort(trigger_events_out, "application_trigger_events_out", "current trigger_events in ApplicationFSM")

	return true	
end

function startHook()
    print("[application_supervisor.lua] starting") 
    if not setpointGeneratorTableProp:get() then
      rtt.logl("Error","No setpointGeneratorTable property set!")
    end
    local rttTrajectoryGeneratorTable = setpointGeneratorTableProp:get()
    if not driverTableProp:get() then
      rtt.logl("Error","No driverTable property set!")
    end
    local rttDriverTable = driverTableProp:get()
    local rttCompositeTable = compositeTableProp:get()
    -- rtt tables start from 0, lua tables from 1!
    for i=0,rttTrajectoryGeneratorTable.size-1 do
      setpointGeneratorTable[#setpointGeneratorTable+1] = rttTrajectoryGeneratorTable[i]
    end
    for i=0,rttDriverTable.size-1 do
      driverTable[#driverTable+1] = rttDriverTable[i]
    end
    for i=0,rttCompositeTable.size-1 do
      compositeTable[#compositeTable+1] = rttCompositeTable[i]
    end
    -- getting the file locations 
    if(application_fsm_package_prop:get()=="")
    then
        print("[application_supervisor.lua] No application_fsm_package specified, will look for fsm file on location specified by application_fsm property")    
        application_fsm_file = application_fsm_prop:get()
    else
        application_fsm_file = rttros.find_rospack(application_fsm_package_prop:get()) .. application_fsm_prop:get()
    end

	-- FSM
	-- load state machine
	fsm = rfsm.init(rfsm.load(application_fsm_file))
    --rfsm.pre_step_hook_add(fsm, function(fsm, events)
    --  if #events > 1 then print("application_supervisor received: "..utils.tab2str(events)) end
    --end)

	-- get all events from the all input ports
	fsm.getevents = rfsm_rtt.gen_read_str_events(common_events_in, priority_events_in, trigger_events_in)

	-- optional: create a string port to which the currently active
	-- state of the FSM will be written. gen_write_fqn generates a
	-- function suitable to be added to the rFSM step hook to do this.
	fqn_out = rtt.OutputPort("string")
	tc:addPort(fqn_out, "currentState", "current active rFSM state")
	rfsm.post_step_hook_add(fsm,rfsm_rtt.gen_write_fqn(fqn_out))

	--raise event functions
	raise_common_event=rfsm_rtt.gen_raise_event(common_events_out, fsm)
	raise_priority_event=rfsm_rtt.gen_raise_event(priority_events_out, fsm)
	raise_trigger_event=rfsm_rtt.gen_raise_event(trigger_events_out, fsm)

    return true
end

timer_id_in = rtt.Variable('int')
function updateHook() 
	timer_id_in_fs = time_trigger:read(timer_id_in)
	
	--check whether this component is triggered by a timer and if so, whether it is the correct timer
	if timer_id_in_fs=="NewData" then
	   if timer_id_in:tolua()==application_timer_id:get() then
	            rfsm.run(fsm)
		 end
	else 
		rfsm.run(fsm)
	end
end

function cleanupHook()
    rttlib.tc_cleanup()
end

-- CONFIGURE
--- Function containing RTT specific info to configure TrajectoryGenerators
function configureTrajectoryGenerators()
    for i=1,#setpointGeneratorTable do
      if not ApplicationSupPeertable[setpointGeneratorTable[i]]:configure() then
        rtt.logl("Error","unable to configure "..setpointGeneratorTable[i])
        raise_common_event("e_emergency") 
      end
    end
end

-- configure drivers
function configureDrivers()
    for i=1,#driverTable do
      if ApplicationSupPeertable[driverTable[i]]:configure() then
        rtt.logl("Error","unable to configure "..driverTable[i])
        raise_common_event("e_emergency") 
      end
    end
end

-- START
--- Function containing RTT specific info to start TrajectoryGenerators
function startTrajectoryGenerators()
    for i=1,#setpointGeneratorTable do
      if ApplicationSupPeertable[setpointGeneratorTable[i]]:getState()~='Running' then
        if not ApplicationSupPeertable[setpointGeneratorTable[i]]:start() then
          rtt.logl("Error","unable to start "..setpointGeneratorTable[i])
          raise_common_event("e_emergency") 
        else
          rtt.logl("Info",setpointGeneratorTable[i].." started")
        end
      else
        rtt.logl("Info", setpointGeneratorTable[i].." was already started")
      end
    end
end

-- start drivers
function startDrivers()
    for i=1,#driverTable do
      if ApplicationSupPeertable[driverTable[i]]:start() then
        rtt.logl("Error","unable to start "..driverTable[i])
        raise_common_event("e_emergency") 
      end
    end
end

--STOP
--- Function containing RTT specific info to stop TrajectoryGenerators
function stopTrajectoryGenerators()
    for i=1,#setpointGeneratorTable do
      if ApplicationSupPeertable[setpointGeneratorTable[i]]:getState()~='Stopped' then
        if not ApplicationSupPeertable[setpointGeneratorTable[i]]:stop() then
          rtt.logl("Error","unable to stop "..setpointGeneratorTable[i])
          raise_common_event("e_emergency") 
        end
      end
    end
end

-- stop drivers
function stopDrivers()
    for i=1,#driverTable do
      if ApplicationSupPeertable[driverTable[i]]:stop() then
        rtt.logl("Error","unable to stop "..driverTable[i])
        raise_common_event("e_emergency") 
      end
    end
end

--- Function to kill the application
function exitApplication(status)
    os.exit(status)
end
