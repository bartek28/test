% Projekt Przetwarzanie i Rozpoznawanie Obrazów
% Wykonawcy: Bartosz Kurzyñski, Adam ¯ychliñski

clear all
close all
% wczytanie i kadrowanie obrazu referencyjnego
o_ref = imread('z13_ref.bmp');
o_ref_kadr = o_ref(1:41,:,:);
% imshow(o_ref); impixelinfo;       % do odczytu kolorow
[o_ref_kadr,itab]=rgb2ind(o_ref_kadr,2);      % konwersja rgb

o_ref_kadr = imbinarize(o_ref_kadr);          % binaryzacja
o_ref_kadr = imcomplement(o_ref_kadr);        % negatyw
% figure
% imshow(o_ref)

% etykietyzacja 
[o, num] = bwlabel(o_ref_kadr);
% wyznaczenie cech wzorcowych
[cechyWzor, centroidWzor, areaWzor] = cechowanie(o);

% RED 
rR = 130/255;
gR = 46/255;
bR = 43/255;
pR = 0.2;
% PINK 
rP = 226/255;
gP = 41/255;
bP = 229/255;
pP = 0.3;
% GREEN
rG = 46/255;
gG = 227/255;
bG = 51/255;
pG = 0.3;
% BLACK
rB = 46/255;
gB = 45/255;
bB = 51/255;
pB = 0.25;

r = [rR; rP; rG; rB];
g = [gR; gP; gG; gB];
b = [bR; bP; bG; bB];
p = [pR; pP; pG; pB];

% petla obrazow
% for j=1:12
    j=1;
    o2=['z13_',num2str(j),'.bmp'];
    o1 = imread(o2);
    o1 = im2double(o1);             % konwersja na double
    obj1 = [];                      % objekty - samochody poprawne
    obj2 = [];                      % objekty - choinki poprawne
    wynik = zeros(4,7);             % macierz wynikow
    
    for i=1:4
        o_segm = segmentacja(o1,r(i,1),g(i,1),b(i,1),p(i,1));
        figure
        imshow(o_segm) 
        o_segm = medfilt2(o_segm);                  % filtracja - filtr medianowy
        
        % usuwanie objektow na krawedziach
        o_kraw = o_segm;
        o_segm = imclearborder(o_segm);
        o_kraw = o_kraw - o_segm;
        
%         figure
%         imshow(o_kraw)                              % obiekty na krawedziach
%         figure
%         imshow(o_segm)                              % obraz bez obiektow na krawedziach
        
        [ob_kraw,kraw] = bwlabel(o_kraw);             % zliczenie obiektów na krawedziach
        wynik(i,7) = wynik(i,7) + kraw;
        
        % etykietyzacja i wyznaczenie cech
        [o_obj, elementy] = bwlabel(o_segm);
        [cechyObr, centroid, area] = cechowanie(o_obj);
        
        for k = 1:1:elementy
            if (area(k,1) > 100)
                aktualny = cechyObr(k,:);
                d = sqrt(sum(abs(cechyWzor - repmat(aktualny,num,1)).^2,2));        % wyznaczenie odleglosci
                klasyfikacja = find (d == min(d));                                  % klasyfikacja

                if(klasyfikacja == 1)
                    wynik(i,1) = wynik(i,1) + 1;
                    obj1 = [obj1; centroid(k,:)];
                elseif(klasyfikacja == 2)
                    wynik(i,2) = wynik(i,2) + 1;
                elseif(klasyfikacja == 3)
                    wynik(i,3) = wynik(i,3) + 1;
                elseif(klasyfikacja == 4)
                    wynik(i,4) = wynik(i,4) + 1;
                    obj2 = [obj2; centroid(k,:)];
                elseif(klasyfikacja == 5)
                    wynik(i,5) = wynik(i,5) + 1;
                elseif(klasyfikacja == 6)
                    wynik(i,6) = wynik(i,6) + 1;
                end
            end
        end
    end
    
    figure
    imshow(imread(o2))
    hold on
    viscircles(obj1,32*ones(size(obj1,1),1),'EdgeColor','r');       % zaznaczanie obiektów
    viscircles(obj2,32*ones(size(obj2,1),1),'EdgeColor','b');
    title(['Zestaw ',num2str(j)])
    hold off
    disp(wynik)
% end


function [cechy, centroid, area] = cechowanie(obr)
    cechy =regionprops(obr,'EulerNumber','Area','Solidity','Centroid');
    euler = cat(1,cechy.EulerNumber);
    solidity = cat(1,cechy.Solidity);
    centroid = cat(1,cechy.Centroid);
    area = cat(1,cechy.Area); 
    cechy = [euler,solidity];
end

function segm = segmentacja(o1,r,g,b,p)
    [m n n1] = size(o1);
    rgb = cat(3,r*ones(m,n),g*ones(m,n),b*ones(m,n));
    d1 = (o1 - rgb) .* (o1 - rgb);
    dist = sqrt ( d1(:,:,1) + d1(:,:,2) + d1(:,:,3));   % odleglosc euklidesowa
    segm = dist < p;
end











