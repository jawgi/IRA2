classdef Door < LoadObjectSTL
    properties
        fileName = "door";
        scaleX = 1.0;
        scaleY = 0.8;
        scaleZ = 1.0;
        transformation = SE3(2.01,-1.39,0);
        faceColour = [0.68, 0.85, 0.90];
        edgeColour = [0.63,0.8,0.85];
        identifier = "door";
    end
    
    methods
        function self = Door()
            self@LoadObjectSTL();
        end
            
    end
end