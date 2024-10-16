classdef Workspace < LoadObjectSTL
    properties
        fileName = "workspace";
        scaleX = 1.0;
        scaleY = 1.0;
        scaleZ = 1.0;
        transformation = SE3(0,0,0);
    end
    
    methods
        function self = Workspace()
            self@LoadObjectSTL();
        end
            
    end
end