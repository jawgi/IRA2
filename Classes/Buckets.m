classdef Buckets < LoadObjectSTL
    properties
        scaleX = 1.0;
        scaleY = 0.7;
        scaleZ = 1.0;
        fileName = "buckets2";
        transformation = SE3(0,-0.15,0);
        faceColour = [0.5,0.5,0.5];
        edgeColour = '#606060';
        identifier = "buckets2";
    end

    methods
        function self = Buckets()
            self@LoadObjectSTL();
        end %Bucket

        % % function setBucketStage(self,stage,row,col)
        %     if stage == 1
        %         self.fileName = "bucketStage1"
        %         disp(self.fileName)
        %         self.transformation = SE3(0,-1.5,1.0)
        % 
        %     end
        % 
        %     if stage == 2
        %         self.fileName = "bucketStage2";
        %         for i = 1:row 
        %             position = -0.2 + (0.2 * (row-1));
        %             self.transformation = self.transformation * transl(position,0,0);
        %         end
        %     end
        % 
        %     if stage == 3
        %         self.fileName = "bucketStage3"
        %         for i = 1:col
        %             positionVertical = 1.0 + (0.2 * (column-1));
        %             self.transformation = self.transformation * transl(0,0,positionVertical)
        %             for j = 1:row
        %                 positionHorizontal = -0.2 + (0.2 * (row-1));
        %                 self.transformation = self.transformation * transl(positionHorizontal,0,0)
        %             end
        %         end
        %     end
        %     disp(self.fileName);
        % end
    end %methods
end %classDef