%function [x] = Truss_Calculator(b)

% x y coordinates of each node
tic
% assignment nodes
% nodes = [1 0 0; 2 1 0; 3 2 0; 4 3 0; 5 3 1; 6 2 1; 7 1 1; 8 0 1];
% connections = zeros(size(nodes,1));
% assignment connections
% connections = [0 1 0 0 0 0 1 1; 1 0 1 0 0 1 1 0; 0 1 0 1 0 1 0 0; 0 0 1 0 1 1 0 0; 0 0 0 1 0 1 0 0; 0 1 1 1 1 0 1 0; 1 1 0 0 0 1 0 1; 1 0 0 0 0 0 1 0];

nodes= [1 -50, 0;
          2 -25, 0;
          3 25, 0;
          4 69.07, 0;
          5 119.47, 0;
          6 169.87, 0;
          7 220.27, 0;
          8 270.67, 0;
          9 287.15, 0;
          10 312.5, 16.8;
          11 287.5, 16.8;
          12 273, 18.6;
          13 223.4, 24.9;
          14 173.8, 31.2;
          15 124.2, 37.5;
          16 74.6, 43.7;
          17 25, 50;
          18 -25, 50];
          
connections = [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1; % 01
               1 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1; % 02
               0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0 1 1; % 03
               0 0 1 0 1 0 0 0 0 0 0 0 0 0 0 1 1 0; % 04
               0 0 0 1 0 1 0 0 0 0 0 0 0 0 1 1 0 0; % 05
               0 0 0 0 1 0 1 0 0 0 0 0 0 1 1 0 0 0; % 06
               0 0 0 0 0 1 0 1 0 0 0 0 1 1 0 0 0 0; % 07
               0 0 0 0 0 0 1 0 1 0 0 1 1 0 0 0 0 0; % 08
               0 0 0 0 0 0 0 1 0 1 1 1 0 0 0 0 0 0; % 09
               0 0 0 0 0 0 0 0 1 0 1 0 0 0 0 0 0 0; % 10
               0 0 0 0 0 0 0 0 1 1 0 1 0 0 0 0 0 0; % 11
               0 0 0 0 0 0 0 1 1 0 1 0 1 0 0 0 0 0; % 12
               0 0 0 0 0 0 1 1 0 0 0 1 0 1 0 0 0 0; % 13
               0 0 0 0 0 1 1 0 0 0 0 0 1 0 1 0 0 0; % 14
               0 0 0 0 1 1 0 0 0 0 0 0 0 1 0 1 0 0; % 15
               0 0 0 1 1 0 0 0 0 0 0 0 0 0 1 0 1 0; % 16
               0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 1 0 1; % 17
               1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0]; % 18
           
% Figure out how many members we need to solve:
upper_connections = triu(connections,1);
% x-y becomes a pair that indicates the member force variable
[force_indices_x, force_indices_y] = ind2sub(size(connections),find(upper_connections));

A = zeros(2*size(nodes,1),size(force_indices_x,1));

% each node is represented by 2 rows, one for x, and one for y
% negative since we move all the forces to the other side of the equation

% assignment b vector:
%b = -[0; 35/3; 0; -10; 0; -15; 0; 40/3; 0; 0; 0; 0; 0; 0; 0; 0];

% b for all external forces
%{
b = -[0; 0; % node 1
    0; -25.0155; % node 2
    0; 29.9205; % node 3
    0; 0; % node 4
    0; 0; % node 5
    0; 0; % node 6
    0; 0; % node 7
    0; 0; % node 8
    0; 0; % node 9
    0; -9.81/4; % node 10
    0; -9.81/4; % node 11
    0; 0; % node 12
    0; 0; % node 13
    0; 0; % node 14
    0; 0; % node 15
    0; 0; % node 16
    0; 0; % node 17
    0; 0]; % node 18
%}
% b for virtual force of 1N where redundant member is
%{
b = -[0; 0; % node 1
    0; 0; % node 2
    0; 0; % node 3
    0; 0; % node 4
    0; 0; % node 5
    0; 0; % node 6
    0.9430; 0.3327; % node 7
    0; 0; % node 8
    0; 0; % node 9
    0; 0; % node 10
    0; 0; % node 11
    -0.9430; -0.3327; % node 12
    0; 0; % node 13
    0; 0; % node 14
    0; 0; % node 15
    0; 0; % node 16
    0; 0; % node 17
    0; 0]; % node 18
%}
% b for virtual force of 1N vertical at tip to find deflection
%
b = -[0; 0; % node 1
    0; -5.75; % node 2
    0; 6.75; % node 3
    0; 0; % node 4
    0; 0; % node 5
    0; 0; % node 6
    0; 0; % node 7
    0; 0; % node 8
    0; 0; % node 9
    0; -1; % node 10
    0; 0; % node 11
    0; 0; % node 12
    0; 0; % node 13
    0; 0; % node 14
    0; 0; % node 15
    0; 0; % node 16
    0; 0; % node 17
    0; 0]; % node 18
%}

% @ node 1:
% connections(1,:)
% find all non-zero elements in this row, and save their column #
% use those indices and get their x,y coordinates from the nodes matrix row
% subtract each one from your  node 1 (x,y), to get vector directions
% normalize each one
% do a sum of x components, and sum of y components
% store them in the right format in your final A & b matrices
% make sure to know which variables multiply, and which are zero

for current_node = 1:size(nodes,1)
node_connections = connections(current_node,:);
node_xy_connections_indices = find(node_connections)';
node_xy_connections = nodes(node_xy_connections_indices,:);
node_xy_conn_values = node_xy_connections(:,2:3);

node_vectors = node_xy_conn_values - nodes(current_node,2:3);

% normalize each row of the matrix i.e. each vector from the node
node_vectors_normalized = bsxfun(@rdivide,node_vectors,vecnorm(node_vectors')');

% now need to put the x components into one equation
% 1*T12 + 0.7071*T17 + 0*T18 = - Force in x
% Then y components into equation
% 0*T12 + 0.7071*T17 + 1*T18 = - Force in y

% Force in x and y can be accounted for at the very end in b of Ax = b
% Just need to negate them and put them in the right row (each row
% corresponds to one node equation



% if node == x && connected_joint == y OR node == y && connected_joint == x
% then put the x component of that connected_joint into current A row column y
% of A
% & put y component of that connected_joint into next current A row column y

% find which member forces are acting on the current node
matching_x = find(force_indices_x == current_node);

% check the reverse force subscript
matching_x2 = find(force_indices_y == current_node);
% concatenate the two and re-order to make sure ascending order to match
% with x,y data
matching_x = sort([matching_x; matching_x2]);
    
% store x components
column_entry = matching_x;
x_components = node_vectors_normalized(:,1);
y_components = node_vectors_normalized(:,2);
for i = 1:size(find(matching_x),1)
    % jump by 2 since each node is responsible for two rows of A:
    % one row for sum of x equation
    % one row for sum of y equation
    A(2*current_node-1,column_entry(i)) = x_components(i);
    A(2*current_node,column_entry(i)) = y_components(i);
end    
end
% rows 1 and 2 of A will be for node 1 x-y
% rows 3 and 4 of A will be for node 2 x-y
% rows 5 and 6 of A will be for node 3 x-y
% rows 7 and 8 of A will be for node 4 x-y
% ... A should have 2*size(nodes,1) rows

% Solve system of equations:
x = A\b
toc