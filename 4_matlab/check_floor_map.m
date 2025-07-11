% Assumed environment
%            8
%            |
% 1 -------- 2 - 3 ----- 4
% |              |       |
% 5 ------------ 6 ----- 7
% "-" : length 1
% "|" : length 2
clear
clc
P = [[0,4];[20,4];[22,4];[40,4];[0,0];[22,0];[40,0];[20,6];[18,6]];
M = [[0, 1, 0, 0, 1, 0, 0, 0, 0]
    [1, 0, 1, 0, 0, 0, 0, 1, 0]
    [0, 1, 0, 1, 0, 1, 0, 0, 0]
    [0, 0, 1, 0, 0, 0, 1, 0, 0]
    [1, 0, 0, 0, 0, 1, 0, 0, 0]
    [0, 0, 1, 0, 1, 0, 1, 0, 0]
    [0, 0, 0, 1, 0, 1, 0, 0, 0]
    [0, 1, 0, 0, 0, 0, 0, 0, 1]
    [0, 0, 0, 0, 0, 0, 0, 1, 0]]; % 各ノードからいけるノードを表す。
%% 2-5 F
M = [
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0];
    [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0];
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0];
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0]];

P = [[0,5.8];[2,5.8];[12,5.8];[16,5.8];[24,5.8];[31.7,5.8];[38.5,5.8];[41,5.8];[51.5,5.8];[53.5,5.8];[64.5,5.8];[66.5,5.8];[77,5.8];[79,5.8];[0,0];[2,0];[12.5,0];[16.5,0];[24.5,0];[28,0];[38.5,0];[39,0];[41,0];[51.5,0];[53.5,0];[64.5,0];[66.5,0];[77,0];[79,0];[31.7,8.5];[28.5,8.5]];
T = ["elevator_hall",30;
    "elevator",31;
    "401",15;
    "402",16;
    "403",17;
    "404",18;
    "405",19;
    "406",21;
    "407",22;
    "408",23;
    "409",24;
    "410",25;
    "411",26;
    "412",27;
    "413",28;
    "414",29;
    "415",13;
    "416",12;
    "417",11;
    "418",10;
    "419",9;
    "420",8;
    "421",5;
    "422",4;
    "423",3;
    "424",2];

%%
P= [[0,5.8];[2,5.8];[12,5.8];[16,5.8];[24,5.8];[31.7,5.8];[38.5,5.8];[40.5,5.8];[51.5,5.8];[53.5,5.8];[64.5,5.8];[66.5,5.8];[77,5.8];[79,5.8];[-4,0];[6,0];[12.5,0];[16.5,0];[24.5,0];[28,0];[38.5,0];[39,0];[41,0];[49.5,0];[64.5,0];[79,0];[31.7,8.5];[28.5,8.5]];
M = [
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0];
    [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0];
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0];
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0]];
T=["elevator_hall",[27];
    "elevator",[28];
    "101",[14];
    "102",[15];
    "103",[16];
    "104",[17];
    "105",[18];
    "106",[19];
    "107",[20];
    "108",[23];
    "109",[24];
    "110",[25];
    "111",[12];
    "112",[11];
    "113",[10];
    "114",[9];
    "115",[8];
    "116",[7];
    "117",[4];
    "118",[3];
    "119",[2];
    "120",[1];
    "Faculty Office",[1];
    "Creative Center",[11]];
%%

for i = 1:size(M,1)
i1=zeros(size(M,1),1);
i1(i) = 1;
Mi=find(M*i1);
M(Mi,i) = vecnorm(P(Mi,:)-P(i,:),2,2);
end
%%

bld10_4F=imread("1_launcher\launch_slam_toolbox\bld10_4F.pgm");
imshow(bld10_4F)
hold on
%th = 0.0647;%    0.0595
th = 0.067;
R = [cos(th),-sin(th);sin(th),cos(th)];
[r,c] = find(M);
origin=[-5.8, -3.58];
offset=[0.6,0];
r401=[0.3339691162109375,  0.6438026428222656]; 
Pn = 20*[1,-1].*(R*(P+offset)')'+[0,471]+20*[1,-1].*(-origin);
x = [Pn(r,1),Pn(c,1),nan(size(r))]';
y = [Pn(r,2),Pn(c,2),nan(size(r))]';

plot(Pn(:,1),Pn(:,2),'ro',x,y,'r');
%text(Pn(:,1),Pn(:,2)-1.5,string((1:length(Pn))),'Color','red');
text(Pn(str2double(T(:,2)),1),Pn(str2double(T(:,2)),2)+20*0.5,T(:,1))

daspect([1 1 1])
% ids = data(:,2)>13;
% offseta = [-7.5,37];
% Pa = 20*[1,-1].*(R*([-1,1].*data(ids,[1,3]))')'+[1800,350]+20*[1,-1].*(offseta-origin);
% plot(Pa(:,1),Pa(:,2),'.');


%%
% example 
% M(:,2) : [8 0 1 0 0 0 0 2]' 
% これは node 2 からいける nodeである1,3,8に値を持つ行列
% 各値はnode間距離（上図参照）

%% initialize lists
I = 1:size(M,1);% list of indices
V = zeros(size(I)); % value list
IV = [I;V]'; % valued index list : value is a minimum cost to reach the node

%% calc setting
start = 27; %15; % nearest node : TODO : consider how to decide it
goal = 1;%30;  % goal node : TODO : consider how to decide it
[seq,res] = gen_sequence(start,goal,M,IV);
disp(seq); % show the result
function [P,H]=gen_sequence(start,goal,M,IV)
ID = 1; % id
GEN= 2; % generation = tree depth
PN = 3; % parent number in generateion
NID= 4; % node id
VAL= 5; % value
ai = start; % current position
H = zeros(size(M,1)^2,5); % history
% H = [id, generation, parent number in generation, node index, value]
H(1,:) = [1,1,0,ai,0]; % initial
for i = 2:size(M,1) % generation loop
    disp(["Generation : ",i]); % for debug
    ids = H(:,GEN)==i-1; % previous gen ids (logical vector)
    v0 = H(ids,NID); % previous vertices
    V0 = H(ids,VAL); % previous vertices' value 
    Vi = M(:,v0); % value from v0 to next vertices
    for col = 1:size(Vi,2) % loop for each previous vertex
        % col : focused vertex
        disp(["col : ",col]); % for debug
        lH = find(H(:,1)==0,1)-1; % number of history
        V = Vi(:,col); % value from v0(col) to next generation vi
        vi = find(V); % node connected from v0(col)
        ei = V~=0; % edge indices 
        V(ei) = V(ei) + V0(col); % value from start to vi 
        TV = IV(:,2); % Minimum vlue to each node
        
        % subs. large value(=1000) to 0 to compare
        TV(TV==0) = 1000;
        V(V==0) = 1000;

        mini=find(TV>V); % index to be replace the value
        IV(mini,2) = V(mini); % set/update value
        ids = (lH+1:lH+length(mini))' % indices for set/update nodes
        H(ids,:) = [ids,repmat([i,col],length(ids),1),mini,V(mini)]; % log histroy
    end
    H(1:find(H==0,1),:)
    ngen = length(H(:,GEN)==i); % number of this generation
    if ngen == 0 % break if there is no new node
        break;
    end
end
%% Find path from start to goal
nid = goal; % node index
id = H(:,NID)==nid; % id = row in H
gen = H(id,GEN); % generation to reach the goal with minimum path
P = zeros(1,gen); % path array
P(end) = goal; % set goal
for g = gen-1:-1:1 % from goal to start
    gid = H(:,GEN)==g; % indices of g-th generation
    tmp = H(gid,:);    % g-th gen history
    tmp = tmp(H(id,PN),:); % find the parent of current node
    nid = tmp(NID); % parent's node index
    id = tmp(ID); % parent's id in H
    P(g) = nid; % set parent as a node on the path
end
end
