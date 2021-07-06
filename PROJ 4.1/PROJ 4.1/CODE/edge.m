[row col]=size(edges)
nodes=csvread('nodes.csv');
nodes=nodes([9:20],:);

edges=csvread('edges.csv');
edges=edges([7:24],:);

edgesfinal=zeros(12,12);
for r=1:row
  edgesfinal(edges(r,1),edges(r,2))=edges(r,3);
endfor
nodesfinal=zeros(12,1)
for r=1:12
  nodesfinal(r,1)=nodes(r,4);
endfor
