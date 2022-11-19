%% Represent set Polytopes
F = randn(10,2)
f = ones(10,1)

% {x | F*x <= f};
P = Polyhedron(F,f)
P.plot

% Vertices of polyhedron (row-wise)
P.V
% Normals of constraints (row-wise)
P.A
% Offsets of constraints (row-wise)
P.b

%% Intersection
P = Polyhedron(randn(10,2),ones(10,1)) + randn(2,1);
Q = Polyhedron(randn(10,2),ones(10,1)) + randn(2,1);
plot(P, 'alpha', 0.1, Q, 'alpha', 0.1);
%%
I = intersect(P, Q);

% I.H = cat(P.H, Q.H)
I.H
P.H
Q.H

% Not all elements of H are necessary
I.minHRep
hold on; I.plot('alpha', 0.3, 'color', 'g'); hold off

%% Pre-set computation
P = Polyhedron(randn(30,3), ones(30,1)) + [0;0;1]
plot(P, 'alpha', 0.1)
%%
p = P.projection(1:2)
hold on; plot(p, 'clor', 'g'); hold off

%% Equality test
P = Polyhedron(randn(10,2), ones(10,1))
Q = Polyhedron(randn(10,2), ones(10,1))
plot(P, 'alpha', 0.1, 'color', 'g', Q, 'alpha', 0.1);
%%
[P.support(Q.A') Q.b]

Q <= P
Q == P
Q >= P