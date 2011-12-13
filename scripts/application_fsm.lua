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

-- ApplicationFSM
--require("ansicolors")

return rfsm.composite_state{
	--dbg = fsmpp.gen_dbgcolor3({["STATE_ENTER"]=true, ["STATE_EXIT"]=true, ["HIBERNATING"]=false, ["EXEC_PATH"]=true, ["EFFECT"]=false, ["DOO"]=false, ["CHECKING"]=false, ["RAISED"]=true},  false,   ansicolors.yellow .. ansicolors.bright ..  "APPLICATIONfsm" ..  ansicolors.reset),

NONemergency = rfsm.composite_state{

	ConfiguringApplication = rfsm.simple_state{
		entry=function()
			print("=>ApplicationFSM->ConfiguringApplication state entry")
			configureTrajectoryGenerators()
			raise_trigger_event("e_configITASC")
		end,

	},
	
	ConfiguredApplication = rfsm.simple_state{
		entry=function()
			print("=>ApplicationFSM->ConfiguredApplication state entry")
			raise_common_event("e_ApplicationConfigured")
		end,
	},
	
	StartingApplication = rfsm.simple_state{
		entry=function()
			print("=>ApplicationFSM->StartingApplication state")
      startDrivers()    
			raise_trigger_event("e_startITASC")
		end,
	},
	
	StartedApplication = rfsm.simple_state{
		entry=function()
			print("=>ApplicationFSM->StartedApplication state entry")
			-- start trajectory generator here so it has something on his input port
			startTrajectoryGenerators()
			raise_common_event("e_ApplicationStarted")
		end,
	},
	
	RunningApplication = rfsm.simple_state{
		entry=function()
			print("=>ApplicationFSM->RunningApplication state")
			raise_trigger_event("e_runITASC")
		end,
	},
	
	StoppingApplication = rfsm.simple_state{
		entry=function()
			print("=>ApplicationFSM->StoppingApplication state")
			raise_trigger_event("e_stopITASC")
			stopTrajectoryGenerators()
		end,
	},
	
	StoppedApplication = rfsm.simple_state{
		entry=function()
			print("=>ApplicationFSM->StoppedApplication state")
			raise_common_event("e_ApplicationStopped")
		end,
	},
	
	rfsm.transition { src='initial', tgt='ConfiguringApplication' },
	rfsm.transition { src='ConfiguringApplication', tgt='ConfiguredApplication',
		guard=function (tr)
			local x = tr.src.emem.e_ITASCConfigured
			if x and x > 0 then return true end
			return false
		end },
	rfsm.transition { src='ConfiguringApplication', tgt='StoppingApplication', events={ 'e_stop' } },
	rfsm.transition { src='ConfiguredApplication', tgt='StartingApplication', events={ 'e_done' } },
	rfsm.transition { src='ConfiguredApplication', tgt='StoppingApplication', events={ 'e_stop' } },
	rfsm.transition { src='StartingApplication', tgt='StartedApplication',
		guard=function (tr)
			local x = tr.src.emem.e_ITASCStarted
			if x and x > 0 then return true end
			return false
		end },
	rfsm.transition { src='StartingApplication', tgt='StoppingApplication', events={ 'e_stop' } },
	rfsm.transition { src='StartedApplication', tgt='RunningApplication', events={ 'e_done' } },
	rfsm.transition { src='StartedApplication', tgt='StoppingApplication', events={ 'e_stop' } },
	rfsm.transition { src='RunningApplication', tgt='StoppingApplication', events={ 'e_stop' } },
	rfsm.transition { src='StoppingApplication', tgt='StoppedApplication',
		guard=function (tr)
			local x = tr.src.emem.e_ITASCStopped
			if x and x > 0 then return true end
			return false
		end },
},

ApplicationEmergency = rfsm.simple_state{
	entry = function ()
		raise_priority_event("e_emergency")
		print("[EMERGENCY] =>Application application in emergency state!")
		print("[EMERGENCY] =>ApplicationFSM has therefore raised a priority emergency event")
	end,
},

rfsm.transition { src='initial', tgt='NONemergency' },
rfsm.transition { src='NONemergency', tgt='ApplicationEmergency', events={'e_emergency'} },

}

