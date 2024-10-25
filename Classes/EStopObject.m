classdef EStopObject < LoadObjectSTL
    properties
        fileName = "estop";
        scaleX = 1.0;
        scaleY = 1.0;
        scaleZ = 1.2;
        transformation = SE3(0,0,0);
        faceColour = [1,0,0];
        edgeColour = [0.9,0.1,0.1];
        identifier = "estop";
    end
    
    methods
        function self = EStopObject(location)
            if self.identifier == "estop"
                self.deleteAll();
            end
            if isempty(location)
                location = "doorstop";
            end
            if location == "doorstop"
                rotation = troty(pi/2) % 90-degree rotation around Z
                translation = [2.08; -0.7; 1.5] % Translation vector
                T = rotation;
                T(1:3,4) = translation;
                self.transformation = SE3(T)
            elseif location == "robot1"
                self.transformation = SE3(0.67,-0.58,0.81);
            elseif location == "robot2"
                self.transformation = SE3(0.67,0.82,0.81);
            elseif location == "collection"
                rotation = trotx(-pi/2) % 90-degree rotation around Z
                translation = [-0.75;1.34881;1.5] % Translation vector
                T = rotation;
                T(1:3,4) = translation;
                self.transformation = SE3(T);
            end

            self.identifier = location;
            self.plotFile();
        end
    end
end