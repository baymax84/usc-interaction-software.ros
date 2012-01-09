function plot_GMM(Data,trues,num_clusters,ids)

[num q] = size(Data)
tot = q*(q-1)/2;
xx = 1;
options = statset('Display','iter','MaxIter',1000);

colors = [ [0 0 0] ; [0.8 0 0] ; [0 0.8 0] ; [0 0 0.8] ; 
					 [0.8 0.8 0] ; [0.8 0 0.8] ; [0 0.8 0.8] ; [0.4 0.4 0.4] ];
mcolor=[];
%obj=gmdistribution.fit(Data,num_clusters,'Options',options,'Replicates',5,'SharedCov',true);
for j=1:q
    for k=j+1:q
				[j k]
                figure
        %subplot(tot, 1, xx);
        %plotGMM([obj.mu(:,j) obj.mu(:,k)]', obj.Sigma([j,k],[j,k],:), [.8 0 0], 1);
        %hold on;
        xx = xx+1;
				
				for i=1:num
					mcolor(i,:) = colors(trues(i),:);
                    plot(Data(i,j), Data(i,k), 'x', 'markerSize',4,'color',mcolor(i,:));
                    hold on;
				end


        xlabel(ids(j),'fontsize',16); ylabel(ids(k),'fontsize',16);
    end
end

% histogram filter
