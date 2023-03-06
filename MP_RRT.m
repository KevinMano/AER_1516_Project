%Kevin Mano
%AER1516: Robot Motion Planning
%27 Feb 2023
%MP-RRT

%Declare world variables
world_x = [0 500];
world_y = [0 500];
time = 300;
q_goal = [randi(max(world_x)) randi(max(world_y))];
q_init = [randi(max(world_x)) randi(max(world_y))];

%Declare static obstacle variables
num_static_obs = 5;
obs_rad = 15;
static_obs_pos = zeros(num_static_obs,2);
for i=1:num_static_obs
    static_obs_pos(i,1) = randi(max(world_x));
    static_obs_pos(i,2) = randi(max(world_y));
end

%Declare dynamic obstacle variables. Store these in a struct instead.
num_dynamic_obs = 5;
obs_pos = struct();
obs_pos.x = 0;
obs_pos.y = 0;
obs_pos.vel_x = 0;
obs_pos.vel_x = 0;
dynamic_obs_pos = repmat(obs_pos, num_dynamic_obs, 1);
for i=1:num_static_obs
    dynamic_obs_pos(i).x = zeros(time, 1); 
    dynamic_obs_pos(i).y = zeros(time,1); 
    dynamic_obs_pos(i).vel_x = randi(5);
    dynamic_obs_pos(i).vel_y = randi(5);
    dynamic_obs_pos(i).x(1,1) = randi(max(world_x));
    dynamic_obs_pos(i).y(1,1) = randi(max(world_y));
end

for i=1:num_dynamic_obs
    for j=2:time
        new_x_pos = dynamic_obs_pos(i).x(j-1,1) + dynamic_obs_pos(i).vel_x*(j/100);
        if(or( (new_x_pos) >= max(world_x), (new_x_pos) <= min(world_x)) )
            dynamic_obs_pos(i).vel_x = -1*dynamic_obs_pos(i).vel_x;
        end
        dynamic_obs_pos(i).x(j,1) = new_x_pos;
        new_y_pos = dynamic_obs_pos(i).y(j-1,1) + dynamic_obs_pos(i).vel_y*(j/100);
        if( or( new_y_pos >= max(world_y), new_y_pos <= min(world_y) ) )
            dynamic_obs_pos(i).vel_y = -1*dynamic_obs_pos(i).vel_y;
        end
        dynamic_obs_pos(i).y(j,1) = new_y_pos;
    end
end

%Declare Tree variables.
Tree = tree(q_goal, time);
Forest = [];
p_goal = 0.01;
p_forest = 0.1;
for i=1:1:time
    %Select node randomly.
    new_node = SelectSample(Forest, p_goal, p_forest, world_x, world_y, q_init)

    distance = 100000000;
    %Find nearest neighbour.
    for k=1:1:length(Tree.tree_mat)
        new_distance = norm(new_node - [Tree.tree_mat(k).Value_x, Tree.tree_mat(k).Value_y] );
        if new_distance<distance
            distance = new_distance;
            nearest_node = [Tree.tree_mat(k).Value_x, Tree.tree_mat(k).Value_y];
            nearest_node_id = Tree.tree_mat(k).ID;
        end
    end

    new_node_level = Tree.tree_mat(k).Level + 1;
    %Add node.
    Tree = add_node(Tree, new_node, new_node_level, nearest_node_id, static_obs_pos, dynamic_obs_pos, obs_rad);
end

%Create animation
%Declare required variables.
for animation_time = 1:time
    clf;
    %Plot static obstacles.
    plot(static_obs_pos(:,1),static_obs_pos(:,2),'.','MarkerSize',obs_rad,'Color','Black');
    hold on
    plot(q_init(1,1), q_init(1,2), '.', 'MarkerSize', 15, 'Color', 'Blue');
    plot(q_goal(1,1), q_goal(1,2), '.', 'MarkerSize', 15, 'Color', 'Green');
    plot(Tree.tree_mat(:).Value_x, Tree.tree_mat(:).Value_y, '.','MarkerSize',obs_rad,'Color','magenta');
    %Plot dynamic obstacles.
    for obs=1:num_dynamic_obs
        plot(dynamic_obs_pos(obs).x(animation_time,1), dynamic_obs_pos(obs).y(animation_time,1),'.','MarkerSize',obs_rad,'Color','Red');
    end
    %Plot tree
    xlim([-5 500])
    ylim([-5 500])
    xlabel('X')
    ylabel('Y')
    legend('Static obstacles', 'Initial Configuration', 'Goal Configuration', 'Sampled points', 'Dynamic Obstacles')
    drawnow
    
    %Store each image into a struct.
    frames(animation_time) = getframe; 
end

%Function SelectSample
function q_new = SelectSample(forest, p_goal, p_forest, world_x, world_y, q_goal) 
    p = rand();
    if p<p_goal
        q_new = q_goal;
    elseif( and((p<(p_goal + p_forest)), not(isempty(forest)==true) ) )
        subtreeroots = [];
        subtreeroot_index = randi(size(subtreeroots,1));
        q_new = [forest(subtreeroot_index).x forest(subtreeroot_index).y];
    else
        q_new = [randi(max(world_x)) randi(max(world_y))];
    end
end