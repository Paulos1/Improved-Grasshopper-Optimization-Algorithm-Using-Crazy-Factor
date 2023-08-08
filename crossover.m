function offspring = crossover(P,pc,etac)

[N,D]= size(P);
index = randperm(N);
P = P(index,:);
offspring = NaN(N,D);
for i = 1:2:N
    r = rand;
    if r < pc
        for j = 1:D
            r =rand;
            if r < 0.5
                beta =(2*r)^(1/(etac+1));
            else
                beta =(1/(2*(1-r)))^(1/(etac+1));
            end
            offspring(i,j) = 0.5*(((1+beta)*P(i,j)) + (1-beta)*P(i+1,j));
            offspring(i+1,j) = 0.5*(((1-beta)*P(i,j)) + (1+beta)*P(i+1,j));
        end
    else
        offspring(i,:) = P(i,:);
        offspring(i+1,:) = P(i+1,:);
    end
end