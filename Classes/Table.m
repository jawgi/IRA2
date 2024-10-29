classdef Table < LoadObjectSTL
    properties
        fileName = "table2";
        scaleX = 1.0;
        scaleY = 0.6;
        scaleZ = 1.0;
        transformation = SE3(0,-0.25,0);
        faceColour = [0.6 0.3 0];
        edgeColour = '#663300';
        identifier = "table2";
    end
    
    methods
        function self = Table()
            self@LoadObjectSTL();
        end
            
    end
end