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
require "rfsm_rtt"
require "rfsm_ext" --needed for the sequential-AND state                                                                                                                                                     

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
	
	-- input port to write events to 
	ITASCcommonEventOutPort = rttlib.port_clone_conn(peertableEF.itasc_supervisor:getPort("itasc_common_events_in")) 
	tc:addPort(ITASCcommonEventOutPort, "ITASCcommonEventOut", "ITASCcommonEvent output port")
	ApplicationcommonEventOutPort = rttlib.port_clone_conn(peertableEF.application_supervisor:getPort("application_common_events_in"))
	tc:addPort(ApplicationcommonEventOutPort, "ApplicationcommonEventOut", "ApplicationcommonEvent output port")   
	ITASCpriorityEventOutPort = rttlib.port_clone_conn(peertableEF.itasc_supervisor:getPort("itasc_priority_events_in")) 
	tc:addPort(ITASCpriorityEventOutPort, "ITASCpriorityEventOut", "ITASCpriorityEvent output port")
	ApplicationpriorityEventOutPort = rttlib.port_clone_conn(peertableEF.application_supervisor:getPort("application_priority_events_in"))
	tc:addPort(ApplicationpriorityEventOutPort, "ApplicationpriorityEventOut", "ApplicationpriorityEvent output port")  
        
	return true	
end

function updateHook() 

end

function cleanupHook()
	print("executing eventFirer cleanup")
	ApplicationpriorityEventOutPort:write("e_stop")
	-- cleanup the created ports.
	tc:removePort(ITASCcommonEventOut:info().name)
	tc:removePort(ApplicationcommonEventOut:info().name)
	tc:removePort(ITASCpriorityEventOut:info().name)
	tc:removePort(ApplicationpriorityEventOut:info().name)
end
