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

-- statemachine
--require("ansicolors")

function guardMultipleEvents(tr, taskTable, prefix, appendix)
	local x=false
	for i=1,#taskTable do
		local x_temp_name = prefix .. taskTable[i] .. appendix
		local x_temp = tr.src.emem[x_temp_name]
		if i>1 then
			x = x and x_temp and x_temp > 0
		else
			x = x_temp and x_temp > 0
		end
	end
	if x then return true end
	return false
end
		
return rfsm.composite_state{
	--dbg = fsmpp.gen_dbgcolor3({["STATE_ENTER"]=true, ["STATE_EXIT"]=true, ["HIBERNATING"]=false, ["EXEC_PATH"]=false, ["EFFECT"]=false, ["DOO"]=false, ["CHECKING"]=false, ["RAISED"]=true},  false, ansicolors.yellow .. ansicolors.bright .. "ITASCfsm" ..  ansicolors.reset),

	
NONemergency = rfsm.composite_state{
	PreOperational = rfsm.simple_state{
	},

	ConfiguringITASC = rfsm.simple_state{	
	    entry=function(fsm)
	    	print("=>iTaSCFSM=>ConfiguringITaSC->ConfiguringFixedPart state entry")
	    	-- broadcast object frames
	    	broadcastObjectFrames()
	    	-- configure solvers
	    	configureSolvers()
	    	-- create connection of the Scene
	    	connectScene2Solver()
	    	-- configure object frames (MUST BE after object frames are broadcasted)
	    	configureObjectFrames()
	    	-- create connection between Robots and Scene (MUST BE after robots are configured)
	    	connectScene2Robots()
	    	-- configure Scene
	    	configureScene()
	    	raise_trigger_event("e_configTasks")
	    end
	},

	ConfiguredITASC = rfsm.simple_state{
		entry=function()
			print("=>iTaSCFSM->ConfiguredITASC state entry")
            -- sent out two events for event name backward compability
			raise_common_event("e_ITASCConfigured")
			raise_common_event("e_ITASC_configured")
		end,
	},

	StartingITASC = rfsm.simple_state{
		entry=function(fsm)
			print("=>iTaSCFSM->StartingITASC state entry")
			startRobots()
			raise_trigger_event("e_startTasks")
		end,
	},

	StartedITASC = rfsm.simple_state{
		entry=function()
			print("=>iTaSCFSM->StartedITASC state entry")
            unlockRobotAxes()
			raise_common_event("e_ITASCStarted")
			raise_common_event("e_ITASC_started")
		end,
	},

	-- sequential AND state
	RunningITASC = rfsm.composite_state {
	   Initializing =rfsm.simple_state{
	      entry=function(fsm)
		       --print("=>iTaSCFSM=>Running=>CompositeTaskFSM->Initializing State")
		    end,
	      doo=function()
		     while true do
			    raise_trigger_event("e_runTasks")
			    rfsm.yield()
		     end
		  end,
	      exit=function()
		      --print("=>iTaSCFSM=>Running=>CompositeTaskFSM->Initialized State")
		      --print("===Application up and running!===")
		   end
	   },

	   Running = rfsm_ext.seqand {
	      order = {'CoordinatingITASC','CompositeTaskFSMa'},
	      CoordinatingITASC = rfsm.init(rfsm.load(running_itasc_coordination_file)),
	      CompositeTaskFSMa = rfsm.init(rfsm.load(composite_task_fsm_file)),
	   },

	   rfsm.transition { src='initial', tgt='Initializing' },
	   rfsm.transition { src='Initializing', tgt='Running',
			     guard = function (tr)
					return guardMultipleEvents(tr, taskTable, 'e_running', '_coordinationInitialized')
				     end
			  },
	},

	StoppingITASC = rfsm.simple_state{
		entry = function ()
			print("=>iTaSCFSM->StoppingITASC state entry")
			stopAllComponents()
            lockRobotAxes()
			raise_trigger_event("e_stopTasks")
		end,
	},

	StoppedITASC = rfsm.simple_state{
		entry=function()
			print("=>iTaSCFSM->StoppedITASC state")
			raise_common_event("e_ITASCStopped")
			raise_common_event("e_ITASC_stopped")
		end,
	},

	rfsm.transition { src='initial', tgt='PreOperational' },
	rfsm.transition { src='PreOperational', tgt='ConfiguringITASC', events={'e_configITASC'}, effect=function () print("=>iTaSCFSM->transition to Configuring ITASC state") end },
	rfsm.transition { src='ConfiguringITASC', tgt='ConfiguredITASC',
					guard = function (tr)
						return guardMultipleEvents(tr, taskTable, 'e_', 'Configured')
					end
		 			},
	rfsm.transition { src='ConfiguringITASC', tgt='StoppingITASC', events={'e_stopITASC'} },
	rfsm.transition { src='ConfiguredITASC', tgt='StartingITASC', events={'e_startITASC'} },
	rfsm.transition { src='ConfiguredITASC', tgt='StoppingITASC', events={'e_stopITASC'} },
	rfsm.transition { src='StartingITASC', tgt='StartedITASC',
					guard = function (tr)
						return guardMultipleEvents(tr, taskTable, 'e_', 'Started')
					end	
					},
	rfsm.transition { src='StartingITASC', tgt='StoppingITASC', events={ 'e_stopITASC' } },
	rfsm.transition { src='StartedITASC', tgt='RunningITASC', events={'e_runITASC'} },
	rfsm.transition { src='StartedITASC', tgt='StoppingITASC', events={ 'e_stopITASC' } },
	rfsm.transition { src='RunningITASC', tgt='StoppingITASC', events={ 'e_stopITASC' } },
	rfsm.transition { src='StoppingITASC', tgt='StoppedITASC',
					guard = function (tr)
						return guardMultipleEvents(tr, taskTable, 'e_', 'Stopped')
					end
					},
},

ITASCEmergency = rfsm.simple_state{
	entry = function ()
		stopAllComponents()
		raise_priority_event("e_emergency")
		print("[EMERGENCY] =>iTaSC application in emergency state!")
		print("[EMERGENCY] =>iTaSC has therefore raised an priority emergency event")
	end,
},

rfsm.transition { src='initial', tgt='NONemergency' },
rfsm.transition { src='NONemergency', tgt='ITASCEmergency', events={'e_emergency'} },
}
