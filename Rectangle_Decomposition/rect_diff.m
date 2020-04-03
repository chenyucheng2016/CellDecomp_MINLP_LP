function [new_rect, change]=rect_diff(R1,R2)
%R1-R2, the outputs are rects

R1=round(R1*10)/10;
R2=round(R2*10)/10;

new_rect={};
change=0;
if length(R1)==0
    return;
end
    
[out] =rect_relation(R1, R2);
%out=-1, R1 is in R2
%out=-2, R2 is in R1
%out=0, intersection, with intersected area >0
%out=1, not intersected with intersected area=0

min_x_1=round(min(R1(1,:))*10)/10;
max_x_1=round(max(R1(1,:))*10)/10;
min_y_1=round(min(R1(2,:))*10)/10;
max_y_1=round(max(R1(2,:))*10)/10;

min_x_2=round(min(R2(1,:))*10)/10;
max_x_2=round(max(R2(1,:))*10)/10;
min_y_2=round(min(R2(2,:))*10)/10;
max_y_2=round(max(R2(2,:))*10)/10;
R1_back=[[min_x_1;min_y_1] [max_x_1;min_y_1] [max_x_1;max_y_1] [min_x_1;max_y_1]];
R2_back=[[min_x_2;min_y_2] [max_x_2;min_y_2] [max_x_2;max_y_2] [min_x_2;max_y_2]];

i=0;
if out==1
    i=i+1;
    new_rect{i}=R1_back;
    %i=i+1;
    %new_rect{i}=R2_back;
    return;
elseif out==-1
    new_rect={};
    change=1;
    return;
elseif out==-2
    R_new=[[min_x_1;min_y_1] [min_x_2;min_y_1] [min_x_2;max_y_1] [min_x_1;max_y_1]];
    if rect_area(R_new)>0.005
        i=i+1;
        new_rect{i}=R_new;
        change=1;
    end
    R_new=[[min_x_2;min_y_1] [max_x_2;min_y_1] [max_x_2;min_y_2] [min_x_2;min_y_2]];
    if rect_area(R_new)>0.005
        i=i+1;
        new_rect{i}=R_new;
        change=1;
    end
    R_new=[[min_x_2;max_y_2] [max_x_2;max_y_2] [max_x_2;max_y_1] [min_x_2;max_y_1]];
    if rect_area(R_new)>0.005
        i=i+1;
        new_rect{i}=R_new;
        change=1;
    end
    R_new=[[max_x_2;min_y_1] [max_x_1;min_y_1] [max_x_1;max_y_1] [max_x_2;max_y_1]];
    if rect_area(R_new)>0.005
        i=i+1;
        new_rect{i}=R_new;
        change=1;
    end
    return;
end

%R1~R2
if min_x_1<=min_x_2 & max_x_1>=max_x_2
    
    if min_y_1>=min_y_2 & max_y_1<=max_y_2
       R_new=[[min_x_1;min_y_1] [min_x_2;min_y_1] [min_x_2;max_y_1] [min_x_1;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
       R_new=[[max_x_2;min_y_1] [max_x_1;min_y_1] [max_x_1;max_y_1] [max_x_2;max_y_1]  ]; 
       if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
    elseif min_y_1<min_y_2 & max_y_1<max_y_2
        R_new=[[min_x_1;min_y_1] [min_x_2;min_y_1] [min_x_2;max_y_1] [min_x_1;max_y_1]  ];
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[min_x_2;min_y_1] [max_x_2;min_y_1] [max_x_2;min_y_2] [min_x_2;min_y_2] ];
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[max_x_2;min_y_1] [max_x_1;min_y_1] [max_x_1;max_y_1] [max_x_2;max_y_1]  ];
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        
    elseif min_y_1>min_y_2 & max_y_1>max_y_2
         R_new=[[min_x_1;min_y_1] [min_x_2;min_y_1] [min_x_2;max_y_1] [min_x_1;max_y_1]  ];
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[min_x_2;max_y_2] [max_x_2;max_y_2] [max_x_2;max_y_1] [min_x_2;max_y_1] ];
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[max_x_2;min_y_1] [max_x_1;min_y_1] [max_x_1;max_y_1] [max_x_2;max_y_1]  ];
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        
    end
elseif min_x_1>min_x_2 & max_x_1<max_x_2
    
    if min_y_1<=min_y_2 & max_y_1>=max_y_2
        R_new=[[min_x_1;min_y_1] [max_x_1;min_y_1] [max_x_1;min_y_2] [min_x_1;min_y_2]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[min_x_1;max_y_2] [max_x_1;max_y_2] [max_x_1;max_y_1] [min_x_1;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        
    elseif min_y_1<=min_y_2 & max_y_1<max_y_2
       R_new=[[min_x_1;min_y_1] [max_x_1;min_y_1] [max_x_1;min_y_2] [min_x_1;min_y_2]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        
    elseif min_y_1>min_y_2 & max_y_1>=max_y_2
       R_new=[[min_x_1;max_y_2] [max_x_1;max_y_2] [max_x_1;max_y_1] [min_x_1;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
       
    end    
elseif min_x_1<=min_x_2 & max_x_1<max_x_2
    
    if min_y_1<=min_y_2 & max_y_1>=max_y_2
       R_new=[[min_x_1;min_y_1] [min_x_2;min_y_1] [min_x_2;max_y_1] [min_x_1;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[min_x_2;min_y_1] [max_x_1;min_y_1] [max_x_1;min_y_2]  [min_x_2;min_y_2]   ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[min_x_2;max_y_2] [max_x_1;max_y_2] [max_x_1;max_y_1]  [min_x_2;max_y_1]   ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end         

    elseif min_y_1>min_y_2 & max_y_1<max_y_2
       R_new=[[min_x_1;min_y_1] [min_x_2;min_y_1] [min_x_2;max_y_1] [min_x_1;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
    elseif min_y_1<=min_y_2 & max_y_1<max_y_2
       R_new=[[min_x_1;min_y_1] [min_x_2;min_y_1] [min_x_2;max_y_1] [min_x_1;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[min_x_2;min_y_1] [max_x_1;min_y_1] [max_x_1;min_y_2]  [min_x_2;min_y_2]   ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
    elseif min_y_1>min_y_2 & max_y_1>=max_y_2
        R_new=[[min_x_1;min_y_1] [min_x_2;min_y_1] [min_x_2;max_y_1] [min_x_1;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
       
        R_new=[[min_x_2;max_y_2] [max_x_1;max_y_2] [max_x_1;max_y_1]  [min_x_2;max_y_1]   ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end    
    end
elseif min_x_1>min_x_2 & max_x_1>=max_x_2
    
    if min_y_1<=min_y_2 & max_y_1>=max_y_2
       R_new=[[min_x_1;min_y_1] [max_x_2;min_y_1] [max_x_2;min_y_2] [min_x_1;min_y_2]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[max_x_2;min_y_1] [max_x_1;min_y_1]  [max_x_1;max_y_1]  [max_x_2;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[min_x_1;max_y_2] [max_x_2;max_y_2] [max_x_2;max_y_1] [min_x_1;max_y_1] ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end   
        
    elseif min_y_1>min_y_2 & max_y_1<max_y_2
        R_new=[[max_x_2;min_y_1] [max_x_1;min_y_1]  [max_x_1;max_y_1]  [max_x_2;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end

    elseif min_y_1<=min_y_2 & max_y_1<max_y_2
        R_new=[[min_x_1;min_y_1] [max_x_2;min_y_1] [max_x_2;min_y_2] [min_x_1;min_y_2]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[max_x_2;min_y_1] [max_x_1;min_y_1]  [max_x_1;max_y_1]  [max_x_2;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end

    elseif min_y_1>min_y_2 & max_y_1>=max_y_2
       
        R_new=[[max_x_2;min_y_1] [max_x_1;min_y_1]  [max_x_1;max_y_1]  [max_x_2;max_y_1]  ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end
        R_new=[[min_x_1;max_y_2] [max_x_2;max_y_2] [max_x_2;max_y_1] [min_x_1;max_y_1] ]; 
        if rect_area(R_new)>0.005
            i=i+1;
            new_rect{i}=R_new;
            change=1;
        end   

    end    
end
