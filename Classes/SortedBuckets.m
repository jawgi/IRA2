classdef SortedBuckets < LoadObjectSTL
    properties
        scaleX = 1.0;
        scaleY = 0.9;
        scaleZ = 1.0;
        fileName = "finalBuckets";
        transformation = SE3(0,1.05,0);
        faceColour = [0.95,0.95,0.95];
        edgeColour = [0.9,0.9,0.9];
        identifier = "finalBuckets";
    end

    methods
        function self = SortedBuckets()
            self@LoadObjectSTL();
        end %Bucket

    end %methods
end %classDef