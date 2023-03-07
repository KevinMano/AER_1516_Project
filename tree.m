%Kevin Mano
%AER1516: Robot Motion Planning
%27 Feb 2023
%MP-RRT: Tree class

classdef tree
    properties
        %Root node.
        root = struct('ID', 0, 'Level', 0, 'Value_x', 0, 'Value_y', 0);
        nodes = 0;
        tree_mat = [];
    end
    methods
        %Constructor
        function obj = tree(q_root, size)
           %Populate Root.
           obj.root.Value_x = q_root(1,1);
           obj.root.Value_y = q_root(1,2);

           %Unique identifier for each root. 
           obj.root.ID = size;
           
           %Update node count.
           obj.nodes = 1;

           %Array of structs.
           obj.tree_mat = repmat(obj.root, obj.root.ID, 1);
       end

        %Add a node to the tree.
        function [success, obj] = add_node(obj, q_coords, level, parent_id, static_obs, dynamic_obs, rad)
            if parent_id > obj.root.ID
                disp('Error, parent non-existent!')
                success = false;
                return
            else
                no_collision = collision_check(obj, parent_id, q_coords, static_obs, dynamic_obs, rad);
                if no_collision == true
                    Node_ID = obj.root.ID - obj.nodes;
                    obj.nodes = obj.nodes + 1;
                    %node = struct('ID', Node_ID, 'Level', level, 'Value', q_coords, 'Parent', parent_id)
                    obj.tree_mat(obj.nodes).ID = Node_ID;
                    obj.tree_mat(obj.nodes).Level = level;
                    obj.tree_mat(obj.nodes).Value_x = q_coords(1,1);
                    obj.tree_mat(obj.nodes).Value_y = q_coords(1,2);
                    obj.tree_mat(obj.nodes).Parent = parent_id;
                    success = true;
                    return
                else
                   disp('Collision present');
                   success = false;
                   return 
                end
            end
        end

        %Delete node if no longer valid.
        function obj = delete_node(obj, node_id)
            %Delete node struct.
            obj.tree_mat(node_id) = [];

            %Update number of nodes.
            Num_nodes = obj.nodes - 1;
            obj.nodes = Num_nodes;

            %Update parents of nodes.

        end

        %Delete edge if no longer valid.
        %function obj = delete_edge(obj, )
            
        %end

        %If no collision present, draw edge. 
        function success = collision_check(obj, parent_id, coords, static_obstacles, dynamic_obstacles, rad)
            %Check against static obstacles.
            for i=1:1:length(static_obstacles)
                %Form quadratic equation of the line between node and its parent and the obstacle
                %circle.
                m = (coords(1,2) - obj.tree_mat(parent_id).Value_y)/(coords(1,1)- obj.tree_mat(parent_id).Value_x);
                c = coords(1,2) - m*coords(1,1);
                %Check the perpendicular distance of line to circle.
                perp_distance = (static_obstacles(i,1)*m + static_obstacles(i,2)*(-1) + c)/(sqrt(m^2 + 1));
                if abs(perp_distance) < rad
                    success = false;
                    return
                end
            end

            %Check against dynamic obstacles.
%             for i=1:1:length(dynamic_obstacles)
%                 %Form quadratic equation of the line between node and its parent and the obstacle
%                 %circle.
%                 m = (coords(1,2) - obj.tree_mat(parent_id).Value_y)/(coords(1,1)- obj.tree_mat(parent_id).Value_x);
%                 c = coords(1,2) - m*coords(1,1);
%                 %Check the perpendicular distance of line to circle.
%                 perp_distance = (dynamic_obstacles(i).x(end,1)*m + dynamic_obstacles(i).y(end,1)*(-1) + c)/(sqrt(m^2 + 1));
%                 if perp_distance < rad
%                     success = false;
%                     return
%                 end
%             end
            success = true;
        end
    end
end
