#ifndef ASTAR
#define ASTAR

#include <list>
#include <vector>
#include <queue>
#include <limits>



template<typename NodeType, typename EdgeType>
class Node
{
private:
	Node*  parentNode;
	NodeType* node;
	typename Heuristic::ReturnType h;
	typename Heuristic::ReturnType g;   //Check this, might just be float??
	//float g;

	

public:
	Node(NodeType& n, Heuristic::ReturnType h1, float g1) : node(&n), g(g1), h(h1), p(NULL) {}
	Node(NodeType& n, Heuristic::ReturnType h1, float g1, Node& p) : node(&n), g(g1), h(h1), parentNode(&p) {}
	Node(Node& other) : node(other.getNode()), parentNode(other.getParentNode()), g(other.getG()), h(other.getH()) {}
	
	typename Heuristic::ReturnType getScore() { return h + g; }
	size_t GetID() const { return node->GetID(); }
	typename Heuristic::ReturnType getG() const{ return g; }
	typename Heuristic::ReturnType getH() const { return h; }
	NodeType* getNode() const { return node; }
	Node* getParentNode() const { return parentNode; }

	const Node& operator=(const Node& other)
	{
		if (*this != other)
		{
			node = other.getNode();
			g = other.getG();
			h = other.getH();
			parentNode = other.getParentNode();
		}

		return *this;
	}

	bool operator<(const Node& other) { return  this->getScore() < other.getScore(); }
	bool operator>(const Node& other) { return this->getScore > other.getScore(); }
	bool operator==(const Node& other) { return this->GetID() == other.GetID(); }
	
	
	Node& operator=(const Node& other) 
	{
		if (*this != other)
		{
			node = other.getNode();
			parentNode = other.getParentNode();
			g = other.getG();
			h = other.getH();
		}
		return *this
	}

};


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
			typename GraphType::Vertex const& vertex_start = graph.GetVertex(start_id);
			typename GraphType::Vertex const& vertex_goal = graph.GetVertex(goal_id);

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



//openList.push(Node(vertex_start));
//typename GraphType::Vertex currentNode;
Node<GraphType::Vertex, GraphType::Edge> currentNode(vertex_start, h, 0), childNode(vertex_start,h,0);
openList.push(vertex_start);
while (openList.size() > 0) {
	callback.OnIteration(*this);

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
	currentNode = openList.top();
	openList.pop();
	//currentNode = openList.pop();
	if (currentNode.GetID() == vertex_goal)
	{
		while (currentNode.GetID() != vertex_start)
		{
			GraphType::Edge e;
			std::vector<GraphType::Edge> const& outedges = graph.GetOutEdges(currentNode.GetID());
			for (int i = 0; i < outedges.size(); i++)
			{
				if ((outedges[i].GetID1() == currentNode.GetID() && outedges[i].GetID2() == currentNode.getParentNode()->GetID()) || (outedges[i].GetID2() == currentNode.GetID() && outedges[i].GetID1() == currentNode.getParentNode()->GetID()))
				{
					solution.push_back(outedges[i]);
					i = outedges.size();
				}

			}
			//solution.push_back(currentNode));
			currentNode = currentNode->getParentNode();  //???
		}
		break;
	}

	//End solution pushing





	//Is the 'g' value just the edge length + the parent's 'g value, or the edge length + the parent's 'g + h' value?
	//Think just the edge + g
	std::vector<GraphType::Edge> const& outedges = graph.GetOutEdges(currentNode.GetID());
	size_t childID;
	//float gHolder;
	//Heuristic::ReturnType hHolder;
//	bool poppedFromClosedList = false, isOnClosedList=false;



	//Expand node!
	bool isOnClosedList = false;
	for (int i = 0; i < outedges.size();i++)
	{
	//	gHolder = currentNode.getG() + outedges[i].GetWeight();
		isOnClosedList = false;

		childID = outedges[i].getID1();
		
		
		if (outedges[i].GetID1() == currentNode.GetID())
		{
			childID = outedges[i].GetID2();
		}


		childNode = Node(graph.GetVertex(childID), heuristic(graph, graph.GetVertex(childID), vertex_goal), currentNode.getG() + outedges[i].GetWeight(), currentNode)
		//hHolder = Heuristic(graph, graph.GetVertex(otherID), vertex_goal);

		//Check the closed list:  if a child is on it, check if the current node's g  (or g + h, potentially)  is <= than the one on the closed list
		//If the closed list is greater, simply discard & ignore that child.   Or is it push it onto the closed list?   CHECK THIS STEP AAAAA
		//Otherwise, remove that node from the closed list, and push the current node onto the openList

		for (auto j = closedList.begin(); j!= closedList.end();j++)
		{
			if (childID == j->GetID())
			{
				isOnClosedList = true;
				if (childNode.getG() <= j->getG())
				{
					openList.push(childNode);
					//openList.push(Node(currentNode->getNode(), hHolder, gHolder, currentNode));
					closedList.remove(*j);  //Remove at iterator j?   Also check this
					//Push current node, or child node?   CHECK THIS!!
				
					
				//	poppedFromClosedList = true;
				}
				j = closedList.end();
			}
		}
	
		if (!isOnClosedList)
		{
			openList.push(childNode);
		}



					//openList.push(Node(graph.GetVertex(childID, heuristic(graph, graph.GetVertex(childID), vertex_goal), currentNode.getG() + outedges[i].GetWeight(), &currentNode);
				}

				
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
      std::priority_queue<Node>  openList;  //OpenListContainer             
	  std::list<Node>         closedList;  //ClosedListContainer   
        std::vector<GraphType::Edge>           solution;    //SolutionContainer
        size_t                       start_id,goal_id;
};

#endif
