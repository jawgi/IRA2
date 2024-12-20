classdef Enclosure < LoadObjectSTL
    properties
        fileName = "enclosure";
        scaleX = 0.8;
        scaleY = 0.6;
        scaleZ = 1.0;
        transformation = SE3(0.34,-0.4,0);
        faceColour = [0.68, 0.85, 0.90];
        edgeColour = [0.63,0.8,0.85];
        identifier = "enclosure";
    end
    
    methods
        function self = Enclosure()
            self@LoadObjectSTL();
        end
            
    end
end