--           This file is part of the iTaSC project                      
--                                                                       
--                  (C) 2012 Dominick Vanthienen                         
--              dominick.vanthienen@mech.kuleuven.be,                    
--              Department of Mechanical Engineering,                    
--                        KU Leuven, Belgium.                  
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
require "rfsm_rtt"
require "rfsm_ext" --needed for the sequential-AND state                                                                                                                                                     
local fsc, eCommonIn 
local fsp, ePriorityIn
local ITASCcommonEventOutPort, ApplicationcommonEventOutPort, ITASCpriorityEventOutPort, ApplicationpriorityEventOutPort

-- FOR MANUALLY FIRING EVENTS
tc=rtt.getTC()

function configureHook()  
	-- peer table (to enable smaller code to request operations)
	peertableEF={}
	-- ipairs: i=> iterate over array part
	for i,p in ipairs(tc:getPeers()) do peertableEF[p]=tc:getPeer(p) end
	--print("	content of peertableEF: ")
	-- K= key=name of the peer
	-- V=value= taskcontext where he's pointing to
	--for K,V in pairs(peertableEF) do print( K) end
	--print("	end of eventFirer peers")
	
	-- input ports 
    ROScommonEventInPort = rtt.InputPort("/std_msgs/String", "ros_common_events_in", "port to receive common events towards itasc and application level, through a ROS topic")
    tc:addPort(ROScommonEventInPort)
    ROSpriorityEventInPort = rtt.InputPort("/std_msgs/String", "ros_priority_events_in", "port to receive priority events towards itasc and application level, through a ROS topic")
    tc:addEventPort(ROSpriorityEventInPort)

    -- output ports
    -- should be created and connected on the fly depending on the application
	return true	
end

function startHook()
    ITASCcommonEventOutPort = tc:getPort("itasc_common_events_in")
    ApplicationcommonEventOutPort = tc:getPort("application_common_events_in")
    ITASCpriorityEventOutPort = tc:getPort("itasc_priority_events_in")
    ApplicationpriorityEventOutPort = tc:getPort("application_priority_events_in")
    if( ITASCcommonEventOutPort and ApplicationcommonEventOutPort and ITASCpriorityEventOutPort and ApplicationpriorityEventOutPort )
    then return true
    end 
    return false
end

function updateHook() 
    --read events from ROS topics
    fsc, eCommonIn = ROScommonEventInPort:read()
    fsp, ePriorityIn = ROSpriorityEventInPort:read()
    --send event out
    if fsc=='NewData' then
      if ITASCcommonEventOutPort then
        ITASCcommonEventOutPort:write(eCommonIn.data)
      else
        rtt.logl("Error","no itasc_common_events_in port found")
      end
      if ApplicationcommonEventOutPort then
        ApplicationcommonEventOutPort:write(eCommonIn.data)
      else
        rtt.logl("Error","no application_common_events_in port found")
      end
    end
    if fsp=='NewData' then
      if ITASCpriorityEventOutPort then
        ITASCpriorityEventOutPort:write(ePriorityIn.data)
      else
        rtt.logl("Error","no itasc_priority_events_in port found")
      end
      if ApplicationpriorityEventOutPort then
        ApplicationpriorityEventOutPort:write(ePriorityIn.data)
      else
        rtt.logl("Error","no application_priority_events_in port found")
      end
    end
end

function cleanupHook()
	print("executing eventFirer cleanup")
	ApplicationpriorityEventOutPort:write("e_stop")
	-- cleanup the created ports.
    rttlib.tc_cleanup()
end
