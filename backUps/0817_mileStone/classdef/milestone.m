classdef milestone < points
    properties
        type
        EER
    end
    methods
        % constructor
        function milestone = milestone(loc,type,EER)
            if nargin > 2
                super_arg{1} = loc;

            else
                super_arg{1} = NaN;

            end
            milestone@points(super_arg{:});
            if nargin > 2
                milestone.type = type;
                milestone.EER = EER;
            else
                milestone.type = NaN;
                milestone.EER = NaN;
            end
            
        end
    end
end