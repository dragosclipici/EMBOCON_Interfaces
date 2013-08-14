TX = textread('TX.txt', '');

% initial state values
state = TX(1,2:end);

for i = 1:length(state)
   state(i) = state(i) + 0.01*state(i)*(2*rand-1); 
end