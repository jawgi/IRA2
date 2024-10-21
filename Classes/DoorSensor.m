classdef DoorSensor < LoadObjectSTL
    properties
        fileName = "doorsensor";
        scaleX = 1.0;
        scaleY = 2.0;
        scaleZ = 1.5;
        transformation = SE3(1.915,-0.898,1.5);
        faceColour = [0, 0, 0];
        edgeColour = [0.1,0.1,0.1];
        identifier = "doorsensor";
    end
    
    methods
        function self = DoorSensor()
            self@LoadObjectSTL();
        end
            
    end
end