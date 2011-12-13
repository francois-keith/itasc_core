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
require "rfsm_ext" --needed for the sequential-AND state                                                                                                                                                     
require "rfsmpp"    --needed for the sequential-AND state 
require "kdlpp"    --kdl pritty print (should be included in lua path!)

-- tc=Task Context of the compent we are in ( in this case ApplicationSuperVisor)
tc=rtt.getTC()
local common_events_in, priority_events_in
local timer_id_in_fs ,timer_id_in

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

    -- fill in standard values for the properties (this will cause it to work as in previous versions)
    task_fsm_package_prop:set("")
	task_fsm_prop:set(rttros.find_rospack("itasc_core") .. "/scripts/application_fsm.lua")
	
    
	-- INPUT PORTS 

	-- Port to recieve trigger from a timer
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

	-- get all events from the all input ports
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
end

function updateHook() 
	timer_id_in_fs, timer_id_in = time_trigger:read()

	--check whether this component is triggered by a timer and if so, whether it is the correct timer
	if timer_id_in_fs=="NewData" then
		if timer_id_in==application_timer_id:get() then
			 rfsm.run(fsm)
		 end
	else 
		rfsm.run(fsm)
	end
end

function cleanupHook()
	-- cleanup the created ports.
	tc:removePort(common_events_in:info().name)
	-- cleanup created variables
	common_events_in:delete()
	priority_events_in:delete()
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

-- CONFIGURE
--- Function containing RTT specific info to configure TrajectoryGenerators
function configureTrajectoryGenerators()
	if ApplicationSupPeertable.lissajous_generator:configure() then print("   lissajous_generator configured") else raise_common_event("e_emergency") end
	if ApplicationSupPeertable.cartesian_generator:configure() then print("   cartesian_generator configured") else raise_common_event("e_emergency") end
end

-- START
--- Function containing RTT specific info to start TrajectoryGenerators
function startTrajectoryGenerators()
	if ApplicationSupPeertable.lissajous_generator:start() then print("   lissajous_generator started") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to start TrajectoryGenerators
function startLissajousGenerator()
	if ApplicationSupPeertable.lissajous_generator:start() then print("   lissajous_generator started") else raise_common_event("e_emergency") end
	if ApplicationSupPeertable.CC_cartesian_tracing:getProperty("use_pose"):set(0) then print("   CC_cartesian_tracing use_pose updated") else raise_common_event("e_emergency") end
	if ApplicationSupPeertable.lissajous_generator:move() then print("   lissajous_generator is moving") else raise_common_event("e_emergency") end

end

--- Function containing RTT specific info to start TrajectoryGenerators
function startCartesianGenerator()
	if ApplicationSupPeertable.cartesian_generator:start() then print("   cartesian_generator started") else raise_common_event("e_emergency") end
end

--STOP
--- Function containing RTT specific info to stop TrajectoryGenerators
function stopTrajectoryGenerators()
	if ApplicationSupPeertable.lissajous_generator:stop() then print("   lissajous_generator stopped") else raise_common_event("e_emergency") end
	if ApplicationSupPeertable.cartesian_generator:stop() then print("   cartesian_generator stopped") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to start TrajectoryGenerators
function stopLissajousGenerator()
	if ApplicationSupPeertable.lissajous_generator:stop() then print("   lissajous_generator stopped") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to start TrajectoryGenerators
function stopCartesianGenerator()
	if ApplicationSupPeertable.cartesian_generator:stop() then print("   cartesian_generator stopped") else raise_common_event("e_emergency") end
end

--- Function containing RTT specific info to start the Robot Drivers
function startDrivers()
	if ApplicationSupPeertable.youbot_driver:start() then 
    print("   youbot_driver started") 
  else 
    print("   unable to start youbot_driver ") 
    raise_priority_event("e_emergency") 
  end
end

--- Function containing RTT specific info to unlock the Robot axes
function unlockAxes()
    --2=Velocity, defined in youbot_master_rtt/src/youbot_types.hpp
    if ApplicationSupPeertable.youbot_driver:provides("Arm1"):setControllerMode(2)
    then
        print("   unlocked successfully the youbot axes")
    else
        print("   unable to unlock youbot axes")
        raise_priority_event("e_emergency")
    end
end
