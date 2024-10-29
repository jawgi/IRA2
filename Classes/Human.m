classdef Human < LoadObjectSTL
    properties
        fileName = "scientist";
        scaleX = 1.0;
        scaleY = 1.0;
        scaleZ = 1.0;
        transformation = SE3(1.25,-1,0);
        faceColour = '#FF6666';
        edgeColour = '#FF6666';
        identifier = "human";
    end
    
    methods
        function self = Human(location)
            self@LoadObjectSTL();
            if nargin == 1
                delete(findobj('Tag', 'human'));
                self.transformation = SE3(location);
                % self.delete();
                self.plotFile();
                drawnow();
            end
        end
            
    end
end