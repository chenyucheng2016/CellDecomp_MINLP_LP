function out = cons_region(tours, t1,s1,t2,s2,t4,s4)
% checks if same region entry and exit exists in the same subtour
% t3 = 110; 
% t1 = 65; s1 = 98;
% t2 = 75; s2 = 111;
% t4 = 141; s4 = 63;
out = true;
for i=1:length(tours)
   if ~isempty(tours{i})
       nodes = tours{i};
   end
   flags = zeros(6,1);
   for j = 1:length(nodes)
       n = nodes(j);
       if n==t1
           flags(1) = 1;
       end
       if n==s1
           flags(2) = 1;
       end
       if n==t2
           flags(3) = 1;
       end
       if n==s2
           flags(4) = 1;
       end
       if n==t4
           flags(5) = 1;
       end
       if n==s4
           flags(6) = 1;
       end
   end
   
   
   
   if flags(1)==1 && flags(2)==1
       out = false;
   end
   if flags(3)==1 && flags(4)==1
       out = false;
   end
   if flags(5)==1 && flags(6)==1
       out = false;
   end
end




end

