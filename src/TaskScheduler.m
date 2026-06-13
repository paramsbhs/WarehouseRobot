classdef TaskScheduler < handle
    %TASKSCHEDULER Greedy nearest-task assignment.

    properties
        Tasks % K x 2 array of [row col] pick locations remaining
    end

    methods
        function obj = TaskScheduler(tasks)
            obj.Tasks = tasks;
        end

        function tf = hasTasks(obj)
            tf = ~isempty(obj.Tasks);
        end

        function task = assignNearest(obj, robotPos)
            % Pop and return the task closest to robotPos (Manhattan)
            d = abs(obj.Tasks(:,1) - robotPos(1)) + ...
                abs(obj.Tasks(:,2) - robotPos(2));
            [~, i] = min(d);
            task = obj.Tasks(i, :);
            obj.Tasks(i, :) = [];
        end
    end
end
