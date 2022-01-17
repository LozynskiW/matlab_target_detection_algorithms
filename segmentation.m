function [segmentacja]=segmentation(akwizycja)

akwizycja=uint8(akwizycja);
klatka=edge(akwizycja,'Prewitt');
szer = size(klatka,2);
wys = size(klatka,1);
contour=zeros(wys,szer);
for y=1:1:wys
    for x=1:1:szer
        if klatka(y,x)>0
            contour(y,x)=x;
        end
    end
end

for y=1:1:wys
    line=contour(y,:);
    max_lim=max(line);
    for x=1:1:szer
        if klatka(y,x)==0
            contour(y,x)=max_lim-1;
        end
    end
end

segmentacja=zeros(wys,szer);

for y=1:1:wys
    line=contour(y,:);
    min_lim=min(line);
    max_lim=max(line);

    for x=min_lim+1:1:max_lim
        segmentacja(y,x)=255;
    end
    
    for x=1:1:min_lim+1
        segmentacja(y,x)=0;
    end        
    
end
%{
segmentacja=uint8(segmentacja);
figure()
subplot(2,1,1)
imshow(segmentacja)
subplot(2,1,2)
imshow(klatka)
%}
end


