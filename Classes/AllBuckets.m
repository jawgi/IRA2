classdef AllBuckets < LoadObjectSTL
    properties
        scaleX = 1.0;
        scaleY = 0.7;
        scaleZ = 1.0;
        fileName = "allBuckets";
        transformation = SE3(0,0.16,0.8);
        faceColour = [0.5,0.5,0.5];
        edgeColour = '#606060';
        identifier = "allBuckets";
    end

    methods
        function self = AllBuckets()
            self@LoadObjectSTL();
        end %Bucket

    end %methods
end %classDef