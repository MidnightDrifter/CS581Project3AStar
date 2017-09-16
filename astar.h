#ifndef ASTAR
#define ASTAR

//callback object for Astar
template <typename GraphType, typename AstarType>
class Callback {
    protected:
        GraphType const& g;
    public:
        Callback( GraphType const& _g) : g(_g) {}
        virtual ~Callback() {}
        virtual void OnIteration( AstarType const& ) { }
        virtual void OnFinish( AstarType const& )    { }
};

template <typename GraphType, typename Heuristic> 
class Astar {
    public:
        ////////////////////////////////////////////////////////////
        Astar( GraphType const& _graph, Callback<GraphType,Astar> & cb ) : 
            graph(_graph),
            openList(),
            closedlist(),
            solution(),
            callback(cb),
            start_id(0),
            goal_id(0)
        {}
        ////////////////////////////////////////////////////////////
        // this function should not be used in the actual code
        void sample_function(size_t s, size_t g) {
            start_id = s;
            goal_id  = g;
            openList.clear();
            closedlist.clear();
            solution.clear();
            Heuristic heuristic;
            // note "const&", since Graph returns const references, we save a 
            // temporary
            typename GraphType::Vertex const& vertex_start = graph.GetVertex(start_id);
            typename GraphType::Vertex const& vertex_goal  = graph.GetVertex(goal_id);
            //heuristic from start to goal
            typename Heuristic::ReturnType h = heuristic( graph, vertex_start, vertex_goal );
            std::cout << "Heuristic at start " << h << std::endl;


            // note "const&", since Graph returns const references, we save a 
            // temporary
			std::vector<typename GraphType::Edge> const& outedges = graph.GetOutEdges( vertex_goal );
			size_t outedges_size = outedges.size();
			for (size_t i = 0; i < outedges_size; ++i) {
				std::cout << "goal has a neighbor " << outedges[i].GetID2() << " at distance " << outedges[i].GetWeight() << std::endl;
			}

        }
        ////////////////////////////////////////////////////////////
        std::vector<typename GraphType::Edge> search(size_t s, size_t g) {
            start_id = s;
            goal_id  = g;
            openList.clear();
            closedList.clear();
            solution.clear();
            Heuristic heuristic;
            //heuristic from start to goal
            typename Heuristic::ReturnType h = heuristic( graph,graph.GetVertex(start_id),graph.GetVertex(goal_id) );
			//Heuristic--graph, ID of first vertex to check, ID of second vertex to check


			/*Edges seem to have:  getID1(), getID2()  --  from vertex w/ ID1 to vertex w/ ID2
			getWeight()  -- self explanatory




			Vertices seem to have:   getID(), getName(), -- self explanatory
			some flavor of container / list that is  outgoing_edges[],

			some kind of operator= op. -- implementation seems to depend on the ID



			Graphs seem to have:  
			
			
			// maps vertex to a vector of outgoing edges,
			// adjacency list
			std::map< VertexType, std::vector<EdgeType> > outgoining_edges;
			
			*/

			//Push starting node onto openList (presumably priority queue) w/ priority h + g
			// h = heuristic val, g =0


            while ( openList.size() > 0 ) {
                callback.OnIteration( *this );

				//Pop node off of openList
				//Check if it's the goal
				
				
				//If it is,  step back through the solution until you get to the starting node, then push that path into the solution, then break out & return

				//Else, expand node
				//For each child, calculate its current cost: g = (parent's g + the edge it took to get here) +  h = (its heuristic value)
				//Check the closed list:  if a child is on it, check if the current node's g  (or g + h, potentially)  is <= than the one on the closed list
				//If the closed list is greater, simply discard & ignore that child.   Or is it push it onto the closed list?   CHECK THIS STEP AAAAA
				//Otherwise, remove that node from the closed list, and push the current node onto the openList

				//If a child is NOT on it, push it onto the openList w/ priority g + h

				//RINSE AND REPEAT


            }




            callback.OnFinish( *this );
            return solution;
        }
        ////////////////////////////////////////////////////////////////////////
    private:
        // do not modify the next 2 lines
        const GraphType &            graph;
        Callback<GraphType,Astar>  & callback;
        // the next 4 lines are just sugestions
        // OpenListContainer, ClosedListContainer, SolutionContainer are typedefed
        OpenListContainer            openList;
        ClosedListContainer          closedList;
        SolutionContainer            solution;
        size_t                       start_id,goal_id;
};

#endif
