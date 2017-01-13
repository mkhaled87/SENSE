classdef ncsFIFOController < handle 
    
    properties (SetAccess=private)
        id        % id of the currently open controller
    end
    
    methods
        
        function obj=ncsFIFOController(filename, varargin)
            if(isstr(filename) ~= 1)
                error('filname is not a string');
            end 
                        
            obj.id = mexNcsController('open', filename);
        end
        
        function u = getInputs(obj, qValues, xValues)
            u = mexNcsController('getInput', obj.id, qValues, xValues)';
        end
        
        
    end
end