#include <world_explorer_cdt/graph_planner.h>
#include <std_msgs/Int32.h>


GraphPlanner::GraphPlanner(){}

GraphPlanner::~GraphPlanner(){}

void GraphPlanner::setGraph(const cdt_msgs::Graph& graph)
{
    graph_ = graph;
    graph_size = graph.nodes.size();
}

int GraphPlanner::getGraphID(double x , double y)
{
    int id = -1;

    for(int i=0; i<graph_.nodes.size(); i++)
    {
        if( graph_.nodes.at(i).pose.position.x == x && graph_.nodes.at(i).pose.position.y == y ) 
            id = graph_.nodes.at(i).id.data;
    }

    return id;
}

void GraphPlanner::findClosestNodes(const double& robot_x, const double& robot_y , const double& robot_theta, 
                             Eigen::Vector2d goal_pose,
                             std::vector<geometry_msgs::Pose>& nodes)
{
    double dist2goal = 1e5;
    double dist2robot = 1e5;
    nodes.resize(2);

    // go through all loops
    for(int i =0; i<graph_.nodes.size(); i++)
    {
        geometry_msgs::Pose graph_pose = graph_.nodes.at(i).pose;

        double new_dist2robot = hypot(robot_x - graph_pose.position.x,
                                        robot_y - graph_pose.position.y);
        double new_dist2goal = hypot(goal_pose.x() - graph_pose.position.x,
                                     goal_pose.y() - graph_pose.position.y);     

        if(new_dist2goal < dist2goal)
        {
            dist2goal = new_dist2goal;
            nodes.at(0) = graph_pose;            
        }
            
        if(new_dist2robot < dist2robot)
        {
            dist2robot = new_dist2robot;      
            nodes.at(1) = graph_pose;                                                 
        }            
    }
}

void GraphPlanner::generateGraphFromMsg(Eigen::MatrixXd &graph)
{
    for(cdt_msgs::GraphNode n : graph_.nodes){
        for(std_msgs::Int32 neighbour_id : n.neighbors_id){
            double distance = sqrt(pow(graph_.nodes[neighbour_id.data].pose.position.x - n.pose.position.x,2) + pow(graph_.nodes[neighbour_id.data].pose.position.y - n.pose.position.y,2));
            graph(n.id.data, neighbour_id.data) = distance;
            graph(neighbour_id.data, n.id.data) = distance;
        }
    }
    std::cout << "Generated the following graph from Msg:\n";
    std::cout << graph;
    std::cout << "\n\n";

}

bool GraphPlanner::planPath(const double& robot_x, 
                            const double& robot_y , 
                            const double& robot_theta, 
                            Eigen::Vector2d goal_pose,
                            std::vector<Eigen::Vector2d>& route)
{
       
    std::vector<geometry_msgs::Pose> graph_nodes;
    
    findClosestNodes(robot_x, robot_y, robot_theta, goal_pose, graph_nodes);    

    Eigen::Vector2d goal(graph_nodes.at(0).position.x, graph_nodes.at(0).position.y);
    Eigen::Vector2d start(graph_nodes.at(1).position.x, graph_nodes.at(1).position.y);    

    int goal_id = getGraphID(goal.x(), goal.y());
    int start_id = getGraphID(start.x(), start.y());
    printf("Start id: %d \t Goal id: %d \n", start_id, goal_id);

    int no_vertices = graph_.nodes.size();

    Eigen::MatrixXd graph(no_vertices, no_vertices);

    generateGraphFromMsg(graph);

    dijkstra(graph, start_id, goal_id, route);

    std::cout << "Dijkstra produced the following plan:\n";
    for(Eigen::Vector2d node : route){
        std::cout << node;
        std::cout << "\n";
    }

    return true;
}   

double GraphPlanner::distance(int id_1, int id_2)
{
    geometry_msgs::Pose pose_1 = graph_.nodes.at(id_1).pose;
    geometry_msgs::Pose pose_2 = graph_.nodes.at(id_2).pose;

    return hypot(pose_1.position.x - pose_2.position.x,
                pose_1.position.y - pose_2.position.y);
}

void GraphPlanner::dijkstra(const Eigen::MatrixXd& graph, int start_id, int goal_id, std::vector<Eigen::Vector2d>& route) /*Method to implement shortest path algorithm*/
{
    // TODO think and implement other planner...
    // Code adapted from https://www.includehelp.com/cpp-tutorial/dijkstras-algorithm.aspx

    int vertex = graph.size();

	double dist[vertex];                             
	bool Dset[vertex];
    int path[vertex];

	for(int i=0;i<vertex;i++) /*Initialize distance of all the vertex to INFINITY and Dset as false*/  
	{
		dist[i]=1e5;
		Dset[i]=false;	
        path[i] = -1;
	}
	dist[start_id]=0; /*Initialize the distance of the source vertec to zero*/

	for(int c=0;c<vertex;c++)                           
	{
		int u = minimumDist(dist,Dset);  /*u is any vertex that is not yet included in Dset and has minimum distance*/
		Dset[u]=true;                    /*If the vertex with minimum distance found include it to Dset*/ 

		for(int v=0;v<vertex;v++)                  
		/*Update dist[v] if not in Dset and their is a path from src to v through u that has distance minimum than current value of dist[v]*/
		{
			if(!Dset[v] && graph(u,v) && graph(u,v)>1e-4 && dist[u]!=1e5 && dist[u]+graph(u,v)<dist[v])
            {
                dist[v] = dist[u]+graph(u,v);
                path[v] = u;
            }		
		}
	}

    // Add goal pose to route
    route.clear();
    Eigen::Vector2d goal(graph_.nodes.at(goal_id).pose.position.x, graph_.nodes.at(goal_id).pose.position.y);
    route.push_back(goal);

    // TODO - DONE extract the final route
    int next_id = goal_id;
    while(next_id!=start_id){
        next_id = path[next_id];
        route.push(goal(graph_.nodes.at(next_id).pose.position.x, graph_.nodes.at(next_id).pose.position.y)
    }
}

int GraphPlanner::minimumDist(double dist[], bool Dset[]) 
{
	double min = 1e5;
    int index;                 /*initialize min with the maximum possible value as infinity does not exist */
	for(int v=0;v<graph_size;v++)
	{
		if(Dset[v]==false && dist[v]<=min)      
		{
			min=dist[v];
			index=v;
		}
	}
	return index;    
}
