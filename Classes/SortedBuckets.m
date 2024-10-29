classdef SortedBuckets < LoadObjectSTL
    properties
        scaleX = 1.0;
        scaleY = 0.9;
        scaleZ = 1.0;
        fileName = "finalBuckets2";
        transformation = SE3(0,1.05,0);
        faceColour = [0.95,0.95,0.95];
        edgeColour = '#A0A0A0';
        identifier = "finalBuckets2";
    end

    methods
        function self = SortedBuckets()
            self@LoadObjectSTL();
        end %Bucket

    end %methods
end %classDef