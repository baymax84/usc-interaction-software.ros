i = 0;
C = [];
D = [];
ids = [];
while i < 6
   fileID = fopen(strcat(num2str(i),'.pose'));
   cells = textscan(fileID,'%f%f%f%f%f%f%f%f','Delimiter',' ');
   nC = cell2mat(cells);
   [u v] = size(nC);
   ids = [ids;ones(u,1)*i];
   C = [C;nC];
   fclose(fileID);
   for j=1:u
      D = [D;nC(j,:)-nC(1,:)];
   end
   i = i+1;
end

