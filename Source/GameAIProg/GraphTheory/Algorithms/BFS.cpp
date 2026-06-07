#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
    std::vector<Node*> path;

    if (pStartNode == pDestinationNode)
    {
        path.push_back(pStartNode);
        return path;
    }

    std::queue<Node*> openQueue{};
    std::map<Node*, Node*> cameFrom{};  // child -> parent

    openQueue.push(pStartNode);
    cameFrom[pStartNode] = nullptr;

    while (!openQueue.empty())
    {
        Node* pCurrent = openQueue.front();
        openQueue.pop();

        if (pCurrent == pDestinationNode)
            break;

        for (Connection* pConnection : pGraph->FindConnectionsFrom(pCurrent->GetId()))
        {
            Node* pNext = pGraph->GetNode(pConnection->GetToId()).get();

            // Skip water terrain
            if (TerrainNode* pTerrain = dynamic_cast<TerrainNode*>(pNext))
            {
                if (pTerrain->GetType() == TerrainNode::Type::Water)
                    continue;
            }

            // Only visit unvisited nodes
            if (cameFrom.find(pNext) == cameFrom.end())
            {
                openQueue.push(pNext);
                cameFrom[pNext] = pCurrent;
            }
        }
    }

    // If destination was never reached
    if (cameFrom.find(pDestinationNode) == cameFrom.end())
        return path;

    // Reconstruct path
    Node* pCurrent = pDestinationNode;
    while (pCurrent != nullptr)
    {
        path.push_back(pCurrent);
        pCurrent = cameFrom[pCurrent];
    }

    std::reverse(path.begin(), path.end());
    return path;
}