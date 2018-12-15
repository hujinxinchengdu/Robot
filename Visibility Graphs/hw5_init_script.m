% HW5 Script file to initialize variables and run the template code

clear

% Define an array of polytopes
p1 = Polygon([0,10; 10,10; 10,0; 12,0; 12,12; 0,12]');
p2 = Polygon([15,-5; 17,-5; 17,8; 15,8]');
P = [p1 p2];

% Define start and goal
start = [0 0];
goal = [20 8];

% Construct graph 
% The template file ignores the polygones and creates a trivial graph with 
% only the start and goal nodes
[G,vs,vg] = HW5VisGraph_ASUID(P,start,goal);

% Get the shortest path from start to goal
path = G.Astar(vs, vg)

% Plot the environment and the visibility graph
figure
plot(P);
plot(G);
G.highlight_path(path)
