%% MATLAB code for PSO.
% Edited code. Orignal code taken from Yarpiz.

function [BestCost,BestSol,Convergence_curve,Trajectories,fitness_history, position_history]=pso(N, Max_iter, lb,ub, dim, fobj)
disp('PSO Itration.....');
CostFunction=@(x) fobj(x);        % Cost Function


nVar=dim;     % Number of Decision Variables

VarSize=[1 nVar];   % Size of Decision Variables Matrix

VarMin=lb;         % Lower Bound of Variables
VarMax=ub;         % Upper Bound of Variables

%% PSO Parameters

MaxIt=Max_iter;      % Maximum Number of Iterations

nPop=N;        % Population Size (Swarm Size)

% PSO Parameters
w=1;            % Inertia Weight
wdamp=0.99;     % Inertia Weight Damping Ratio
c1=1.5;         % Personal Learning Coefficient
c2=2.0;         % Global Learning Coefficient

% Velocity Limits
VelMax=0.1*(VarMax-VarMin);
VelMin=-VelMax;

%% Initialization
Convergence_curve=zeros(1,Max_iter);
fitness_history=zeros(N,Max_iter);
position_history=zeros(N,Max_iter,dim);
Trajectories=zeros(N,Max_iter);

empty_particle.Position=[];
empty_particle.Cost=[];
empty_particle.Velocity=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];

particle=repmat(empty_particle,nPop,1);

GlobalBest.Cost=inf;

for i=1:nPop
    
    % Initialize Position
    particle(i).Position=unifrnd(VarMin,VarMax,VarSize);
    
    % Initialize Velocity
    particle(i).Velocity=zeros(VarSize);
    
    % Evaluation
    particle(i).Cost=CostFunction(particle(i).Position);
    
    % Update Personal Best
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost<GlobalBest.Cost
        
        GlobalBest=particle(i).Best;
        
    end
    
end

BestCost=zeros(MaxIt,1);

%% PSO Main Loop

for it=1:MaxIt
    
    for i=1:nPop
        
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
            +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
        
        % Apply Velocity Limits
        particle(i).Velocity = max(particle(i).Velocity,VelMin);
        particle(i).Velocity = min(particle(i).Velocity,VelMax);
        
        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Velocity Mirror Effect
        IsOutside=(particle(i).Position<VarMin | particle(i).Position>VarMax);
        particle(i).Velocity(IsOutside)=-particle(i).Velocity(IsOutside);
        
        % Apply Position Limits
        particle(i).Position = max(particle(i).Position,VarMin);
        particle(i).Position = min(particle(i).Position,VarMax);
        
        % Evaluation
        particle(i).Cost = CostFunction(particle(i).Position);
        
        % Update Personal Best
        if particle(i).Cost<particle(i).Best.Cost
            
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            
            % Update Global Best
            if particle(i).Best.Cost<GlobalBest.Cost
                
                GlobalBest=particle(i).Best;
                
            end
            
        end
        
    end
    
    BestCost(it)=GlobalBest.Cost;
    Convergence_curve(it)=BestCost(it);
   % disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
    
    w=w*wdamp;
    %w= wMax - t .* ((wMax - wMin) / maxIter);
    
end
 BestSol=GlobalBest.Position;
 BestCost = BestCost(end);
% 
% dim=size(data,2);
% k=size(BestSol,2)/dim;
% clusterCenter=reshape(BestSol,[],k)';
% 
% d = pdist2(data, clusterCenter);
% [dmin, idx] = min(d, [], 2);
% 
% cc=clusterCenter;
% toc;
