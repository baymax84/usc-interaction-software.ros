i = 0;
C = [];
ids = [];
while i < 7
   fileID = fopen(strcat(num2str(i),'.pose'));
   cells = textscan(fileID,'%f%f%f%f%f%f%f%f','Delimiter',' ');
   nC = cell2mat(cells);
   [u v] = size(nC);
   ids = [ids;ones(u,1)*i];
   C = [C;nC];
   fclose(fileID);
   i = i+1;
end


for i=0:6
    i
end