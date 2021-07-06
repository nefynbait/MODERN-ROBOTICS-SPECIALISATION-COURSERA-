function [path dist]=astar(nodes,edges)
  open=[];
  close=[];
  n=size(nodes);
  n=n(1,1);
  past_cost=zeros(n,1);
  parent=zeros(n,1);
  past_cost=Inf+past_cost;
  past_cost(1,1)=0;
  open=[open; 1 0];
  while(~isempty(open))
    %%fprintf("o");
    open';
    current=open(1,1);
    %%fprintf("c")
    close';
    %%fprintf("P");
    past_cost';
    close=[close;current];
    dist=open(1,2);
    
    if size(open)==1
      open=[];
    else
      open=open(2:end,:);
    endif

    if current==n

      break;
    endif
    for cc=1:n
        nbr=edges(cc,current);
        if nbr>0
          close';
          cc=cc;
          if sum(ismember(close,cc))==0
            tent_past_cost=past_cost(current,1)+nbr;
            
            if tent_past_cost<past_cost(cc,1)
            past_cost(cc,1)=min(tent_past_cost,past_cost(cc,1));
            parent(cc,1) = current;
            endif
          
            if sum(ismember(open,cc))==0
              open=[open;cc past_cost(cc,1)];
            else
              for rr=1:size(open,1)
                if open(rr,1)==cc
                  open(rr,2)=past_cost(cc,1);
                endif
              endfor
            endif    
          endif
        endif
    endfor
    open=sortrows(open,2);
  endwhile
  current=n;
  path=[];
  while current~=0
    path=[current path];
    current=parent(current,1);
  endwhile
  csvwrite('path.csv',path);
endfunction
