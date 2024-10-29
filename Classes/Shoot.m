classdef Shoot < LoadObjectSTL
    properties
        fileName = "shoot2";
        scaleX = 1.0;
        scaleY = 1.0;
        scaleZ = 0.7;
        transformation = SE3(0,-0.9,0.4);
        faceColour = [0,0,0];
        edgeColour = [0.5,0.5,0.5];
        identifier = "shoot2";
    end
    
    methods
        function self = Shoot()
            self@LoadObjectSTL();
        end
            
    end
end