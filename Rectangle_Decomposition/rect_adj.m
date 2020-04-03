function if_adj=rect_adj(R1, R2)
% tell two rects adjacent or not, i.e., there edge overlap

if_adj=0;
[out, area] =rect_relation(R1, R2);
min_x_a=min(R1(1,:));
max_x_a=max(R1(1,:));
min_y_a=min(R1(2,:));
max_y_a=max(R1(2,:));
min_x_b=min(R2(1,:));
max_x_b=max(R2(1,:));
min_y_b=min(R2(2,:));
max_y_b=max(R2(2,:));
%R1_back=[[min_x_a;min_y_a] [max_x_a;min_y_a] [max_x_a;max_y_a] [min_x_a;max_y_a]];
%edgeR1=[min_x_a min_y_a max_x_a max_y_a];
%R2_back=[[min_x_b;min_y_b] [max_x_b;min_y_b] [max_x_b;max_y_b] [min_x_b;max_y_b]];
%edgeR2=[min_x_b min_y_b max_x_b max_y_b];
if out~=1
    if_adj=1;
    return;
end
if out==1
    if round(min_x_a*10)==round(max_x_b*10) | round(max_x_a*10)==round(min_x_b*10)
        if (round(min_y_a*10)==round(min_y_b*10) & round(max_y_a*10)==round(max_y_b*10)) 
            if_adj=1;
            return;
        end        
    end
    if round(min_x_a*10)==round(max_x_b*10) | round(max_x_a*10)==round(min_x_b*10)
        if (round(min_y_a*10)>round(min_y_b*10) & round(min_y_a*10)<round(max_y_b*10)) | (round(max_y_a*10)>round(min_y_b*10) & round(max_y_a*10)<round(max_y_b*10)) | (round(min_y_b*10)>round(min_y_a*10) & round(min_y_b*10)<round(max_y_a*10)) | (round(max_y_b*10)>round(min_y_a*10) & round(max_y_b*10)<round(max_y_a*10)) 
            if_adj=1;
            return;
        end        
    end
    if round(min_y_a*10)==round(max_y_b*10) | round(max_y_a*10)==round(min_y_b*10)
        if (round(min_x_a*10)==round(min_x_b*10) & round(max_x_a*10)==round(max_x_b*10)) 
            if_adj=1;
            return;
        end 
    end
    if round(min_y_a*10)==round(max_y_b*10) | round(max_y_a*10)==round(min_y_b*10)
        if (round(min_x_a*10)>round(min_x_b*10) & round(min_x_a*10)<round(max_x_b*10)) | (round(max_x_a*10)>round(min_x_b*10) & round(max_x_a*10)<round(max_x_b*10)) | (round(min_x_b*10)>round(min_x_a*10) & round(min_x_b*10)<round(max_x_a*10)) | (round(max_x_b*10)>round(min_x_a*10) & round(max_x_b*10)<round(max_x_a*10)) 
            if_adj=1;
            return;
        end 
    end

end