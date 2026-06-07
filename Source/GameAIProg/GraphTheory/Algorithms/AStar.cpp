#pragma once

#include "AStar.h"
#include <Shared/Graph/GridGraph/GridGraph.h>

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
    : pGraph(pGraph)
    , HeuristicFunction(hFunction)
{
    UE_LOG(LogTemp, Warning, TEXT("A* initialized with heuristic at address: %p"), HeuristicFunction);
}

std::vector<Node*> AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
    std::vector<NodeRecord> openList{};
    std::vector<NodeRecord> closedList{};

    NodeRecord startRecord{};
    startRecord.pNode = pStartNode;
    startRecord.pConnection = nullptr;
    startRecord.costSoFar = 0.f;
    startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
    openList.push_back(startRecord);

    NodeRecord currentRecord{};

    while (!openList.empty())
    {
        //get lowest cost
        currentRecord = *std::min_element(
            openList.begin(), openList.end(),
            [](auto const& a, auto const& b)
            {
                return a.estimatedTotalCost < b.estimatedTotalCost;
            });

        //if goal = reconstruct
        if (currentRecord.pNode == pGoalNode)
            return ReconstructPath(currentRecord, closedList, pStartNode);

		//loop through connections
        for (Connection* pConnection : pGraph->FindConnectionsFrom(currentRecord.pNode->GetId()))
        {
            Node* pNextNode = pGraph->GetNode(pConnection->GetToId()).get();
            float newCostSoFar = currentRecord.costSoFar + pConnection->GetWeight();

            auto closedIt = std::find_if(
                closedList.begin(), closedList.end(),
                [pNextNode](auto const& nr) { return nr.pNode == pNextNode; });

            if (closedIt != closedList.end())
            {
                if (closedIt->costSoFar <= newCostSoFar)
                    continue;

                closedList.erase(closedIt);
            }

            auto openIt = std::find_if(openList.begin(), openList.end(),[pNextNode](auto const& nr) { return nr.pNode == pNextNode; });
           
            if (openIt != openList.end())
            {
                if (openIt->costSoFar <= newCostSoFar)
                    continue;

                openList.erase(openIt);
            }

            NodeRecord newRecord{};
            newRecord.pNode = pNextNode;
            newRecord.pConnection = pConnection;
            newRecord.costSoFar = newCostSoFar;
            newRecord.estimatedTotalCost = newCostSoFar + GetHeuristicCost(pNextNode, pGoalNode);

            openList.push_back(newRecord);
        }

        auto it = std::find_if(openList.begin(), openList.end(),[&](auto const& nr) { return nr.pNode == currentRecord.pNode; });

        if (it != openList.end())
            openList.erase(it);

        
        closedList.push_back(currentRecord);
    }

  
    NodeRecord* best = nullptr;
    float bestH = FLT_MAX;

    for (auto& rec : closedList)
    {
        float h = GetHeuristicCost(rec.pNode, pGoalNode);
        if (h < bestH)
        {
            bestH = h;
            best = &rec;
        }
    }

    if (best)
    {
        UE_LOG(LogTemp, Warning, TEXT("Goal unreachable — fallback to node %d"), best->pNode->GetId());
        return ReconstructPath(*best, closedList, pStartNode);
    }

	//unreachable
    return { pStartNode };
}



std::vector<Node*> AStar::ReconstructPath(
    NodeRecord endRecord,
    std::vector<NodeRecord> const& closedList,
    Node* const pStartNode) const
{
    std::vector<Node*> path;
    NodeRecord current = endRecord;

    while (current.pNode != pStartNode)
    {
        path.push_back(current.pNode);
        Node* prev = pGraph->GetNode(current.pConnection->GetFromId()).get();

        current = *std::find_if(
            closedList.begin(), closedList.end(),
            [prev](auto const& nr) { return nr.pNode == prev; });
    }

    path.push_back(pStartNode);
    std::reverse(path.begin(), path.end());
    return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
    FVector2D startPos = pGraph->GetNode(pStartNode->GetId())->GetPosition();
    FVector2D endPos = pGraph->GetNode(pEndNode->GetId())->GetPosition();

    const GridGraph* grid = static_cast<const GridGraph*>(pGraph);
    float cellSize = grid->GetCellSize();

    float dx = std::abs(endPos.X - startPos.X) / cellSize;
    float dy = std::abs(endPos.Y - startPos.Y) / cellSize;

    return HeuristicFunction(dx, dy);
}
