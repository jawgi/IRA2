classdef CameraObject < LoadObjectSTL
    properties
        fileName = "camera";
        scaleX = 1.0;
        scaleY = 1.0;
        scaleZ = 0.7;
        transformation = SE3(0,-0.55,0.7);
        faceColour = [0.2,0.2,0.2];
        edgeColour = [0.5,0.5,0.5];
        identifier = "camera";
    end
    
    methods
        function self = CameraObject()
            self@LoadObjectSTL();
        end
            
    end
end