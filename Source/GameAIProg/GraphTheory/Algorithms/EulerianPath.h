#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		// TODO If the graph is not connected, there can be no Eulerian Trail

		// TODO Count nodes with odd degree 

		// TODO A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian

		// TODO A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// TODO An Euler trail can be made, but only starting and ending in these 2 nodes

		// TODO A connected graph with no odd nodes is Eulerian
		
		if (!IsConnected())
			return Eulerianity::notEulerian;

		int oddCount = 0;
		for (Node* node : m_pGraph->GetActiveNodes())
		{
			int degree = static_cast<int>(m_pGraph->FindConnectionsFrom(node->GetId()).size());
			if (degree % 2 != 0)
				oddCount++;
		}

		if (oddCount > 2)
			return Eulerianity::notEulerian;
		if (oddCount == 2)
			return Eulerianity::semiEulerian;
		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		eulerianity = IsEulerian();
		if (eulerianity == Eulerianity::notEulerian)
			return {};

		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> nodes = graphCopy.GetActiveNodes();
		std::vector<Node*> path;

		// Determine start node
		int startId = nodes[0]->GetId();

		if (eulerianity == Eulerianity::semiEulerian)
		{
			for (Node* n : nodes)
			{
				int degree = static_cast<int>(graphCopy.FindConnectionsFrom(n->GetId()).size());
				if (degree % 2 != 0)
				{
					startId = n->GetId();
					break;
				}
			}
		}

		
		// TODO Check if there can be an Euler path
		// TODO If this graph is not eulerian, return the empty path
		
		// TODO Start algorithm loop
		std::stack<int> stack;
		stack.push(startId);

		while (!stack.empty())
		{
			int v = stack.top();
			auto conns = graphCopy.FindConnectionsFrom(v);

			if (!conns.empty())
			{
				Connection* c = conns.front();
				int to = c->GetToId();

				graphCopy.RemoveConnection(c);
				stack.push(to);
			}
			else
			{
				stack.pop();
				path.push_back(m_pGraph->GetNode(v).get()); 
			}
		}


		std::reverse(path.begin(), path.end());
		return path;

	}

	inline void EulerianPath::VisitAllNodesDFS(
		const std::vector<Node*>& nodes,
		std::vector<bool>& visited,
		int index) const
	{
		visited[index] = true;
		int nodeId = nodes[index]->GetId();

		for (Connection* c : m_pGraph->FindConnectionsFrom(nodeId))
		{
			int toId = c->GetToId();

			// Find index of the target node
			for (int i = 0; i < nodes.size(); i++)
			{
				if (nodes[i]->GetId() == toId && !visited[i])
				{
					VisitAllNodesDFS(nodes, visited, i);
				}
			}
		}
	}


	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> nodes = m_pGraph->GetActiveNodes();
		int n = static_cast<int>(nodes.size());
		if (n == 0)
			return false;

		// Find a node with at least one connection
		int startIndex = -1;
		for (int i = 0; i < n; i++)
		{
			if (!m_pGraph->FindConnectionsFrom(nodes[i]->GetId()).empty())
			{
				startIndex = i;
				break;
			}
		}

		// If no edges exist, graph is trivially Eulerian
		if (startIndex == -1)
			return true;

		std::vector<bool> visited(n, false);
		VisitAllNodesDFS(nodes, visited, startIndex);

		// Check if all nodes with edges were visited
		for (int i = 0; i < n; i++)
		{
			bool hasEdges = !m_pGraph->FindConnectionsFrom(nodes[i]->GetId()).empty();
			if (hasEdges && !visited[i])
				return false;
		}

		return true;

		// TODO choose a starting node
		
		// TODO start a depth-first-search traversal from the node that has at least one connection
		
		// TODO if a node was never visited, this graph is not connected
	}
}