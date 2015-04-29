function [M] = getMoments(img)

M=zeros(2,2);
M;
for x=1:size(img,1)
  for y=1:size(img,2)
    M(1,1) = M(1,1) +  x^0 * y^0 *img(x,y);
    M(2,1) = M(2,1) +  x^1 * y^0 *img(x,y);
    M(1,2) = M(1,2) +  x^0 * y^1 *img(x,y);
    M(2,2) = M(2,2) +  x^1 * y^1 *img(x,y);
  endfor    
endfor

endfunction