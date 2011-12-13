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

-- ConfiguringVariationalPart state
-- INITIALIZATION OF PARAMETERS
-- example:
-- local robotLoc = rtt.Variable("KDL.Frame")

return rfsm.simple_state{
	entry=function()
		print("=>iTaSCFSM=>ConfiguringITaSC->ConfiguringVariationalPart state entry")
		--
		-- PARAMETERS
		--
		-- define the location of the base of the robots
        -- example
		-- robotLoc:fromtab( {M={X_x=1,Y_x=0,Z_x=0,X_y=0,Y_y=1,Z_y=0,X_z=0,Y_z=0,Z_z=1},p={X=0.0,Y=0.0,Z=0.0}} )

		--
		-- COMPOSING THE SCENE: KEEP THE ORDER! (robots=>VKC's/IM's=>CC's=>Solvers)
		--

		-- add ROBOTS and OBJECTS
		-- addRobot("<robot component name>", <kdl.Frame with location of the base in the scene, eg. robotLoc>)

		-- add OBJECT FRAMES to the robots
		-- addObjectFrame("<objectFrameName you choose for this objectFrame>", "<segment of robot where it is attached to>", "<robot component name>")

		-- add TASKS
		-- add VirtualKinematicChains or InteractionModels
		-- addVirtualKinematicChain("<name of the VKC component>","<1st objectFrameName as chosen above>","<2nd objectFrameName as chosen above>")
		
		-- add Constraints/Controllers
		-- add ConstraintController(	"<name of the CC component>","<1st objectFrameName as chosen above>","<2nd objectFrameName as chosen above>", 
		--				"<name of the VKC component that you want to constrain>",<priority number>)


		-- add Solvers
		addSolver("Solver")
	end,
	exit=function()
		print("=>iTaSCFSM=>ConfiguringITaSC->ConfiguringVariationalPart state exit")
	end,
}

