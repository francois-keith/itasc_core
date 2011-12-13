--           This file is part of the iTaSC project                      
--                                                                       
--                  (C) 2011 Dominick Vanthienen                         
--                  (C) 2011 Tinne De Laet                                 
--              dominick.vanthienen@mech.kuleuven.be,                    
--                  tinne.delaet@mech.kuleuven.be,                    
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

-- CompositeTaskFSM
require("ansicolors")

return rfsm.composite_state{
--	dbg = fsmpp.gen_dbgcolor3({["STATE_ENTER"]=true, ["STATE_EXIT"]=true, ["HIBERNATING"]=false, ["EXEC_PATH"]=true, ["EFFECT"]=false, ["DOO"]=false, ["CHECKING"]=false, ["RAISED"]=true},  false, ansicolors.yellow .. "CompositeTaskFSM".. ansicolors.reset),
	
NONemergency = rfsm.composite_state{	
	Initializing =rfsm.simple_state{
		entry=function(fsm)
			print("=>iTaSCFSM=>Running=>CompositeTaskFSM->Initializing State")
		end,
		doo=function()
			while true do
				raise_trigger_event("e_runTasks")
				rfsm.yield()
			end
		end,
	},
	
	Initialized =rfsm.simple_state{
		entry=function(fsm)
			print("=>iTaSCFSM=>Running=>CompositeTaskFSM->Initialized State")
			print("===Application up and running!===")
		end,
	},

    -- CREATE HERE YOUR OWN STATES

	rfsm.transition { src='initial', tgt='Initializing' },
	rfsm.transition { src='Initializing', tgt='Initialized',
		guard = function (tr)
			return guardMultipleEvents(tr, taskTable, 'e_running', '_coordinationInitialized')
		end
	},
	
    -- CREATE HERE YOUR OWN DEFINED TRANSITIONS BETWEEN THE STATES OF THIS FSM
},

TaskFSMemergency = rfsm.simple_state{
	entry = function ()
		raise_priority_event("e_emergency")
		print("[EMERGENCY] =>TaskFSM encountered an emergency!")
		print("[EMERGENCY] =>TaskFSM will therefore raise an TaskFSMemergency event")
	end,
	},
	
rfsm.transition { src='initial', tgt='NONemergency' },
rfsm.transition { src='NONemergency', tgt='TaskFSMemergency', events={'e_emergency'} },
}		

