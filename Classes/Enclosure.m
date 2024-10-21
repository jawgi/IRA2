classdef Enclosure < LoadObjectSTL
    properties
        fileName = "enclosure";
        scaleX = 1.0;
        scaleY = 1.0;
        scaleZ = 1.0;
        transformation = SE3(0.34,-0.1,0);
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