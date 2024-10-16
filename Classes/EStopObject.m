classdef EStopObject < LoadObjectSTL
    properties
        fileName = "estop";
        scaleX = 1.0;
        scaleY = 1.0;
        scaleZ = 1.0;
        transformation = SE3(0,0.15,0);
        faceColour = [0.6,0.3,0];
        edgeColour = [0.65,0.35,0.05];
    end
    
    methods
        function self = EStopObject()
            self@LoadObjectSTL();
        end
            
    end
end