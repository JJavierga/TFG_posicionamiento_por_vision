function [img_out]=screenshot2marker(img_in,edge)

 [f,c]=size(img_in);

 
 % Encuentra primera columna
 fin=0;
 for j=1:c
   if(fin~=0)
    break
   end
   for i=2:f-1
      if(img_in(i,j)==0 && img_in(i+1,j)==1)
        fin=i;
        break;
      elseif(img_in(i,j)==0 && img_in(i-1,j)==1)
        inicio=i;
        col_0=j;
      end
    end
  end
  
  tam=fin-inicio+1;
  
  img_prev=zeros(tam,tam);
  
  for i=inicio:fin
    for j=col_0:col_0+tam-1;
      if(img_in(i,j)==1)
        img_prev(i-inicio+1,j-col_0+1)=1;
      end
    end
  end
         
  img_out=ones(tam+2*edge,tam+2*edge);
  
  img_out(edge+1:edge+tam,edge+1:edge+tam)=img_prev;
      
 
end 
