----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 04/20/2017 09:28:51 AM
-- Design Name: 
-- Module Name: template_vhdl - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;


entity template_vhdl is
port(	
clk: in std_logic;
res: in std_logic;
res_state: in std_logic;
#$ENTITY_INPUT_PORTS$#
#$ENTITY_OUTPUT_PORTS$#
);
end template_vhdl;  


architecture template_vhdl_behavioral of template_vhdl_entity is

	
    signal current_state, next_state: std_logic_vector(#$noBddvarState$# downto 0);
          process(clk,res)
          begin
          if res = '1' then
          current_state <=res_state;
        
        else if clk'event and clk='1' then
        current_state <= next_state;     
        end if;
        end if;
         
            begin 
          
	  #$BEHAVIORAL_OUTPUT_FUNCTIONS$#
          
            end template_vhdl_behavioral;


	
	
	
	
	
