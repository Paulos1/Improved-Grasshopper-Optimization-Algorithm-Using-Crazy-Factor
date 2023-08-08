
% IGOA(crazy factor)
clc;
clear;
close all;

SearchAgents_no=30; % Number of search agents

Function_name='F4'; % Name of the test function that can be from F1 to F23 (Table 1,2,3 in the paper)

Max_iteration=100; % Maximum numbef of iterations

% Load details of the selected benchmark function
[lb,ub,dim,fobj]=Get_Functions_details(Function_name);
[cTarget_score,Target_pos,cGOA_cg_curve, cTrajectories,cfitness_history, cposition_history]=Crazy_GOA(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
[gTarget_score,Target_pos,GOA_cg_curve,  Trajectories,fitness_history, position_history]=GOA(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
[aTarget_score,Target_pos,aGOA_cg_curve, Trajectories,fitness_history, position_history]=abc(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
[pTarget_score,Target_pos,pGOA_cg_curve, Trajectories,fitness_history, position_history]=pso(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
[iTarget_score,Target_pos,iGOA_cg_curve, Trajectories,fitness_history, position_history]=fa(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);


semilogy(cGOA_cg_curve,'r')
hold on
semilogy(GOA_cg_curve,'g')
hold on
semilogy(aGOA_cg_curve,'b')
hold on
semilogy(pGOA_cg_curve,'k')
hold on 
semilogy(iGOA_cg_curve,'m')

title('Convergence curve')
xlabel('Iteration#');
ylabel('Best score obtained so far');
legend('Crazy-GOA','GOA','ABC','PSO','FA')
box on
axis tight
% 
display(['The best solution obtained by Crazy_GOA is : ', num2str(cTarget_score)]);
display(['The best solution obtained by GOA is : ', num2str(gTarget_score)]);
display(['The best solution obtained by ABC is : ', num2str(aTarget_score)]);
display(['The best solution obtained by PSO is : ', num2str(pTarget_score)]);
display(['The best solution obtained by FA is : ', num2str(iTarget_score)]);

