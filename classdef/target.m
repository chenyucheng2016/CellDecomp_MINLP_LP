classdef target < points
    properties
        init_cueLev
        crt_cueLev
        id  % final id 
        y_ans % classification
    end
    methods
        % constructor
        function target = target(loc,init_cueLev,id,y_ans)
            if nargin > 3
                super_arg{1} = loc;

            else
                super_arg{1} = NaN;

            end
            target@points(super_arg{:});
            if nargin > 3
                target.init_cueLev = init_cueLev;
                target.crt_cueLev = init_cueLev;
                target.id = id;
                target.y_ans = y_ans;
            else
                target.init_cueLev = NaN;
                target.crt_cueLev = NaN;
                target.id = NaN;
                target.y_ans = NaN;
            end
            
        end
    end
end