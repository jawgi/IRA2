classdef Environment < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        area;
        subarea;
    end

    methods
        function self = Environment(area,subarea)
            if isempty(area)
                area = "floor";
            end
            if area == "floor"
                floor(self,subarea);
            elseif area == "wall"
                wall(self,subarea);
            end
        end
        function floor(self,subarea)
            if subarea == "floor"
                surf([-4,-4;4,4] ...
                    ,[-4,4;-4,4] ...
                    ,[0.01,0.01;0.01,0.01] ...
                    ,'CData',imread('floor.jpg') ...
                    ,'FaceColor','texturemap');
            elseif subarea == "ceiling"
                hold on;
                surf([-4,-4;4,4] ...
                    ,[-0.95,4;-0.95,4] ...
                    ,[2.4,2.4;2.4,2.4] ...
                    ,'CData',imread('floor.jpg') ...
                    ,'FaceColor','texturemap');
                surf([-4,-4;4,4] ...
                    ,[-1.9,-4;-1.9,-4] ...
                    ,[2.4,2.4;2.4,2.4] ...
                    ,'CData',imread('floor.jpg') ...
                    ,'FaceColor','texturemap');
                 surf([-4,-4;-0.48,-.048] ...
                    ,[-4,4;-4,4] ...
                    ,[2.4,2.4;2.4,2.4] ...
                    ,'CData',imread('floor.jpg') ...
                    ,'FaceColor','texturemap');
                 surf([0.45,0.45;4,4] ...
                    ,[-4,4;-4,4] ...
                    ,[2.4,2.4;2.4,2.4] ...
                    ,'CData',imread('floor.jpg') ...
                    ,'FaceColor','texturemap');
                hold off;
            end
        end

        function wall(self,subarea)
            z = [0,2.5;0,2.5];

            if subarea == "rear" 
                x = [4,4;4,4];
                y = [-4,-4;4,4];
            elseif subarea == "collection"
                x = [-4,-4;4,4];
                y = [4,4;4,4];
            elseif subarea == "shoot"
                x = [-4,-4;4,4];
                y = [-4,-4;-4,-4];
            end
           
            hold on;
             surf(x,y,z ...
                ,'CData',imread('wall.jpg') ...
                ,'FaceColor','texturemap');
             hold off;
        end
    end
end

