%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "Jinxin", "Hu", "1207744664"

classdef RRTgoal < Navigation

    properties
        npoints         % number of points to find
        graph           % graph Object representing random nodes

        simtime         % path simulation time

        xrange          % range of x coordinates
        yrange          % range of y coordinates
        
        speed           % speed of vehicle
        vehicle         % Vehicle class object describes kinematics
        
        revcost         % penalty for going backwards
        
        root            % coordinate of the root of the tree (3x1)
    end

    methods

        function rrt = RRTgoal(vehicle, varargin)
        %RRTmap.RRT Create an RRT navigation object
        %
        % R = RRTmap.RRT(VEH, OPTIONS) is a rapidly exploring tree navigation
        % object  for a vehicle kinematic model given by a Vehicle subclass object VEH.
        %
        % R = RRTmap.RRT(VEH, MAP, OPTIONS) as above but for a region with obstacles
        % defined by the occupancy grid MAP.
        %
        % Options::
        % 'npoints',N    Number of nodes in the tree (default 500)
        % 'simtime',T    Interval over which to simulate kinematic model toward 
        %                random point (default 0.5s)
        % 'goal',P       Goal position (1x2) or pose (1x3) in workspace
        % 'speed',S      Speed of vehicle [m/s] (default 1)
        % 'root',R       Configuration of tree root (3x1) (default [0,0,0])
        % 'revcost',C    Cost penalty for going backwards (default 1)
        % 'range',R      Specify rectangular bounds of robot's workspace:
        %                - R scalar; X: -R to +R, Y: -R to +R
        %                - R (1x2); X: -R(1) to +R(1), Y: -R(2) to +R(2)
        %                - R (1x4); X: R(1) to R(2), Y: R(3) to R(4)
        %
        % Other options are provided by the Navigation superclass.
        %
        % Notes::
        % - 'range' option is ignored if an occupacy grid is provided.
        %
        % Reference::
        % - Robotics, Vision & Control
        %   Peter Corke, Springer 2011.  p102.
        %
        % See also Vehicle, Bicycle, Unicycle.

            % invoke the superclass constructor, it handles some options
            rrt = rrt@Navigation(varargin{:});

            rrt.vehicle = vehicle;

            % handle the options not done by Navigation superclass
            opt.npoints = 500;
            opt.simtime = 0.5;
            opt.speed = vehicle.speedmax;
            opt.revcost = 1;
            opt.root = [0 0 0];
            
            [rrt,args] = tb_optparse(opt, varargin, rrt);
            
            if isempty(rrt.occgrid)
                opt = [];
                opt.range = 5;
                [opt,args] = tb_optparse(opt, args);
                
                % range can be specified as scalar, min/max, different min/max per
                % direction
                switch length(opt.range)
                    case 1
                        rrt.xrange = [-opt.range opt.range];
                        rrt.yrange = [-opt.range opt.range];
                    case 2
                        rrt.xrange = [-opt.range(1) opt.range(1)];
                        rrt.yrange = [-opt.range(2) opt.range(2)];
                    case 4
                        rrt.xrange = [opt.range(1) opt.range(2)];
                        rrt.yrange = [opt.range(3) opt.range(4)];
                    otherwise
                        error('bad range specified');
                end
            else
                rrt.xrange = [1 numcols(rrt.occgrid)];
                rrt.yrange = [1 numrows(rrt.occgrid)];
            end

            rrt.graph = PGraph(3, 'distance', 'SE2', ...
                'dweight', 2*pi/norm(sum([rrt.xrange; rrt.yrange])) );  % graph of points in SE(2)

        end

        function plan(rrt, varargin)
        %RRTmap.plan Create a rapidly exploring tree
        %
        % R.plan(OPTIONS) creates the tree roadmap by driving the vehicle
        % model toward random goal points.  The resulting graph is kept
        % within the object.
        %
        % Options::
        % 'goal',P        Goal pose (1x3)
        % 'ntrials',N     Number of path trials (default 50)
        % 'noprogress'    Don't show the progress bar
        % 'samples'       Show progress in a plot of the workspace
        %                 - '.' for each random point x_rand
        %                 - 'o' for the nearest point which is added to the tree
        %                 - red line for the best path
        %
        % Notes::
        % - At each iteration we need to find a vehicle path/control that moves it
        %   from a random point towards a point on the graph.  We sample ntrials of
        %   random steer angles and velocities and choose the one that gets us
        %   closest (computationally slow, since each path has to be integrated
        %   over time).

            opt.progress = true;
            opt.samples = false;
            opt.goal = [];
            opt.ntrials = 50;
            
            opt = tb_optparse(opt, varargin);

            if ~isempty(opt.goal)
                rrt.goal = opt.goal;
            end

            % build a graph over the free space
            rrt.message('create the graph');
            rrt.graph.clear();

            if rrt.verbose
                clf
                %idisp(1-rrt.occgrid, 'ynormal', 'nogui');
                hold on
            end

            % check root node sanity
            if isempty(rrt.root)
                error('no root node specified');
            end
            if ~isvec(rrt.root, 3)
                error('root must be 3-vector');
            end
            assert( ~rrt.isoccupied(rrt.root(1:2)), 'root node cell is occupied')

            % check goal node sanity
            if isempty(rrt.goal)
                error('no goal node specified');
            end
            if ~isvec(rrt.goal, 3)
                error('goal must be 3-vector');
            end
            assert( ~rrt.isoccupied(rrt.goal(1:2)), 'goal node cell is occupied')
            
            % add the root point as the first node
            vroot = rrt.graph.add_node(rrt.root);
            data.vel = 0;
            data.path = [];
            rrt.graph.setvdata(vroot, data);

            % graphics setup
            if opt.progress
                h = Navigation.progress_init('RRTmap planning...');
            end
            if opt.samples
                clf
                hold on
                xlabel('x'); ylabel('y');
            end

            npoints = 0;
            %v = rrt.root;
            while npoints < rrt.npoints       % build the tree

                % Step 3
                % find random state x,y

                % pick a point not in obstacle
                while true
                    %xy = rrt.randxy();  % get random coordinate (x,y)
                    %x = v(1) + (rrt.goal(1)-v(1))*rand();
                    %y = v(2) + (rrt.goal(2)-v(2))*rand();
                    x = rrt.root(1) + (rrt.goal(1)-rrt.root(1))*rand();
                    y = rrt.root(2) + (rrt.goal(2)-rrt.root(2))*rand();
                    xy=[x,y];
                    
                    if isempty(rrt.occgrid)
                        break
                    else
                        
                        % test if lies in the obstacle map
                        try
                            if ~( rrt.isoccupied(floor(xy(1:2))) || rrt.isoccupied(ceil(xy(1:2))) || rrt.isoccupied([ceil(xy(1)) floor(xy(2))]) || rrt.isoccupied([floor(xy(1)) ceil(xy(2))]))
                                xy = round(xy);
                                break;
                            end
                        catch
                            % index error, point must be off the map
                            continue;
                        end
                    end
                end
                theta = rrt.rand*2*pi;
                xrand = [xy, theta]';
                if opt.samples
                    plot(xy(1), xy(2), '.')
                end

                % Step 4
                % find the existing node closest in state space

                vnear = rrt.graph.closest(xrand);   % nearest vertex
                xnear = rrt.graph.coord(vnear);     % coord of nearest vertex
%                 if rrt.graph.distance_metric(xnear, xrand) < 0.25
%                     continue;
%                 end

                rrt.message('xrand (%g, %g) node %d', xy, vnear);

                % Step 5
                % figure how to drive the robot from xnear to xrand
                                
                best = rrt.bestpath(xnear, xrand, opt.ntrials);
                
                % Added by GF: ignore nodes that were not able to connect
                % due to collisions
                if best.d < inf
                
                    xnew = best.path(:,best.k);
                    if opt.samples
                        plot(xnew(1), xnew(2), 'o');
                        plot2(best.path', 'r');
                        drawnow
                    end

                    % Step 7,8
                    % add xnew to the graph, with an edge from xnear
                    vnew = rrt.graph.add_node(xnew);


                    if rrt.graph.vdata(vnear).vel * best.vel < 0
                        % we changed direction, penalise that
                        cost = rrt.revcost;
                    else
                        cost = 1;
                    end
                    rrt.graph.add_edge(vnear, vnew, cost);

                    rrt.graph.setvdata(vnew, best);
                end
                %v = xrand;
                npoints = npoints + 1;
                if opt.progress
                    Navigation.progress(h, npoints / rrt.npoints);
                end
                
            end

            if opt.progress
                Navigation.progress_delete(h)
            end
            rrt.message('graph create done');
        end

        function p_ = query(rrt, xstart, xgoal)
        %RRTmap.query Find a path between two points
        %
        % X = R.query(START, GOAL) finds a path (Nx3) from pose START (1x3) 
        % to pose GOAL (1x3).  The pose is expressed as [X,Y,THETA]. 
        %
        % R.query(START, GOAL) as above but plots the path in 3D, where the vertical
        % axis is vehicle heading angle.  The nodes are shown as circles and the
        % line segments are blue for forward motion and red for backward motion.
        %
        % Notes::
        % - The path starts at the vertex closest to the START state, and ends
        %   at the vertex closest to the GOAL state.  If the tree is sparse this
        %   might be a poor approximation to the desired start and end.
        %
        % See also RRTmap.plot.

            assert(rrt.graph.n > 0, 'RTB:RRTmap: there is no plan');
            rrt.checkquery(xstart, xgoal);
            
            g = rrt.graph;
            vstart = g.closest(xstart);
            vgoal = g.closest(xgoal);

            % find path through the graph using A* search
            [path,cost] = g.Astar(vstart, vgoal);
            
            fprintf('A* path cost %g\n', cost);
          
            % concatenate the vehicle motion segments
            cpath = [];
            for i = 1:length(path)
                p = path(i);
                data = g.vdata(p);
                if ~isempty(data.path)
                    if i >= length(path) || g.edgedir(p, path(i+1)) > 0
                        cpath = [cpath data.path];
                    else
                        cpath = [cpath data.path(:,end:-1:1)];
                    end
                end
            end

            if nargout == 0
                % plot the path
                clf; hold on

                plot2(g.coord(path)', 'o');     % plot the node coordinates
                
                for i = 1:length(path)
                    p = path(i);
                    b = g.vdata(p);            % get path data for segment
                    
                    % draw segment with direction dependent color
                    if ~isempty(b.path)
                        % if the vertex has a path leading to it
                        
                        if i >= length(path) || g.edgedir(p, path(i+1)) > 0
                            % positive edge
                            %  draw from prev vertex to end of path
                            seg = [g.coord(path(i-1)) b.path]';
                        else
                            % negative edge
                            %  draw reverse path to next next vertex
                            seg = [  b.path(:,end:-1:1)  g.coord(path(i+1))]';
                        end
                        
                        if b.vel > 0
                            plot2(seg, 'b');
                        else
                            plot2(seg, 'r');
                        end
                    end
                end

                xlabel('x'); ylabel('y'); zlabel('\theta');
                grid
            else
                p_ = cpath';
            end
        end

        function plot(rrt, varargin)
        %RRTmap.plot Visualize navigation environment
        %
        % R.plot() displays the navigation tree in 3D, where the vertical axis is
        % vehicle heading angle.  If an occupancy grid was provided this is also
        % displayed.


            % display the occgrid background
            rrt.plot_bg(varargin{:});
            
            % display the graph
            %rrt.graph.plot('noedges', 'NodeSize', 3, 'NodeFaceColor', 'm', 'NodeEdgeColor', 'm', 'edges');
            
            rrt.graph.plot('noedges', 'nocomponentcolor', 'NodeSize', 3, 'NodeFaceColor', 'b', 'NodeEdgeColor', 'b', 'edges');
hold on
            
            % display the occgrid background
            rrt.plot_fg(varargin{:});
            axis([rrt.xrange rrt.yrange])
            xlabel('x'); ylabel('y'); zlabel('\theta');
            grid on; hold off
            view(0,90);
            axis equal
            rotate3d
        end

        % required by abstract superclass
        function next(rrt)
        end

        function s = char(rrt)
        %RRTmap.char  Convert to string
        %
        % R.char() is a string representing the state of the RRTmap
        % object in human-readable form.
        %
        
            % invoke the superclass char() method
            s = char@Navigation(rrt);

            % add RRTmap specific stuff information
            s = char(s, sprintf('  region: X %f : %f; Y %f : %f', rrt.xrange, rrt.yrange));
            s = char(s, sprintf('  sim time: %f', rrt.simtime));
            s = char(s, sprintf('  speed: %f', rrt.speed));
            s = char(s, sprintf(' Graph:'));
            s = char(s, char(rrt.graph) );
            if ~isempty(rrt.vehicle)
                s = char(s, char(rrt.vehicle) );
            end
        end
        

    end % methods


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    P R I V A T E    M E T H O D S
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods (Access='protected')

        function best = bestpath(rrt, x0, xg, N)

            % initial and final state as column vectors
            x0 = x0(:); xg = xg(:);

            best.d = Inf;
            for i=1:N   % for multiple trials 
            
                %choose random direction of motion and random steer angle
                if rand > 0.5
                    vel = rrt.speed;
                else
                    vel = -rrt.speed;
                end
                steer = (2*rrt.rand - 1) * rrt.vehicle.steermax;    % uniformly distributed
                
                % simulate motion of vehicle for this speed and steer angle which 
                % results in a path
                x = rrt.vehicle.run2(rrt.simtime, x0, vel, steer)';
                
                %% find point on the path closest to xg
                % distance of all path points from goal
                d = colnorm( [bsxfun(@minus, x(1:2,:), xg(1:2)); angdiff(x(3,:), xg(3))] );
                % the closest one
                [dmin,k] = min(d);
                
                % Added by GF: is x a collision state
                if isempty(rrt.occgrid)
                    xCollision = 0;
                else
                    % test if lies in the obstacle map
                    if rrt.isoccupied(floor(x(1:2))) || rrt.isoccupied(ceil(x(1:2))) || rrt.isoccupied([ceil(x(1)) floor(x(2))]) || rrt.isoccupied([floor(x(1)) ceil(x(2))])
                        xCollision = 1;
                    else
                        xCollision = 0;
                    end
                end
                
                % is it the best so far?
                % Added by GF: as long as x is not a collision state
                if dmin < best.d && ~xCollision
                    % yes it is!  save it and the inputs that led to it
                    best.d = dmin;
                    best.path = x;
                    best.steer = steer;
                    best.vel = vel;
                    best.k = k;
                end
            end 
        end 

        % generate a random coordinate within the working region
        function xy = randxy(rrt)
            xy = rrt.rand(1,2) .* [rrt.xrange(2)-rrt.xrange(1) rrt.yrange(2)-rrt.yrange(1)] + ...
                [rrt.xrange(1) rrt.yrange(1)];
        end

        % test if a path is obstacle free
        function c = clearpath(rrt, xy)
            if isempty(rrt.occgrid)
                c = true;
                return;
            end

            xy = round(xy);
            try
                % test that all points along the path do not lie within an obstacle
                for pp=xy'
                    if rrt.isoccupied(pp) > 0
                        c = false;
                        return;
                    end
                end
                c = true;
            catch
                % come here if we index out of bounds
                c = false;
                return;
            end
        end


    end % private methods
end % class