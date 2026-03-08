#include "SpacePartitioning.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);
	
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	Cells.reserve(Rows * Cols);

	for (int row = 0; row < Rows; ++row)
	{
		for (int col = 0; col < Cols; ++col)
		{
			float left = col * CellWidth - SpaceWidth / 2.f;  
			float bottom = row * CellHeight - SpaceHeight / 2.f;	

			Cells.emplace_back(left, bottom, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	int index = PositionToIndex(FVector2D(Agent.GetActorLocation()));
	Cells[index].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	int oldIndex = PositionToIndex(OldPos);
	int newIndex = PositionToIndex(FVector2D(Agent.GetActorLocation()));

	if (oldIndex != newIndex)
	{
		Cells[oldIndex].Agents.remove(&Agent);
		Cells[newIndex].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	FVector2D agentPos = FVector2D(Agent.GetActorLocation());

	FRect query;
	query.Min = agentPos - FVector2D(QueryRadius, QueryRadius);
	query.Max = agentPos + FVector2D(QueryRadius, QueryRadius);

	for (Cell& cell : Cells)
	{
		if (!DoRectsOverlap(cell.BoundingBox, query))
			continue;

		for (ASteeringAgent* other : cell.Agents)
		{
			if (!other || other == &Agent)
				continue;

			FVector2D otherPos = FVector2D(other->GetActorLocation());
			FVector2D delta = otherPos - agentPos;

			if (FMath::Abs(delta.X) > SpaceWidth / 2)
				delta.X -= FMath::Sign(delta.X) * SpaceWidth;
			if (FMath::Abs(delta.Y) > SpaceHeight / 2)
				delta.Y -= FMath::Sign(delta.Y) * SpaceHeight;

			float distSq = delta.SizeSquared();

			if (distSq <= QueryRadius * QueryRadius)
			{
				Neighbors[NrOfNeighbors] = other;
				NrOfNeighbors++;
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	if (!pWorld) return;

	FVector boundaryCorners[4] = {
		FVector(-SpaceWidth / 2.f, -SpaceHeight / 2.f, 5.f),
		FVector(-SpaceWidth / 2.f,  SpaceHeight / 2.f, 5.f),
		FVector(SpaceWidth / 2.f,  SpaceHeight / 2.f, 5.f),
		FVector(SpaceWidth / 2.f, -SpaceHeight / 2.f, 5.f),
	};
	for (int i = 0; i < 4; ++i)
		DrawDebugLine(pWorld, boundaryCorners[i], boundaryCorners[(i + 1) % 4], FColor::Blue, false, -1.f, 0, 3.f);

	for (const Cell& cell : Cells)
	{
		std::vector<FVector2D> points = cell.GetRectPoints();

		int agentCount = cell.Agents.size();
		FColor color = agentCount > 0
			? FColor::MakeRedToGreenColorFromScalar(1.0f / (1.0f + agentCount))
			: FColor(50, 50, 50, 80);

		FVector cellCorners[4]; 
		for (int i = 0; i < 4; ++i)
			cellCorners[i] = FVector(points[i].X, points[i].Y, 0.f);

		for (int i = 0; i < 4; ++i)
			DrawDebugLine(pWorld, cellCorners[i], cellCorners[(i + 1) % 4], color, false, -1.f, 0, 1.5f);

		FVector2D center2D = (cell.BoundingBox.Min + cell.BoundingBox.Max) * 0.5f;
		FVector center3D(center2D.X, center2D.Y, 0.f);

		DrawDebugString(pWorld, center3D, FString::FromInt(agentCount), nullptr,
			agentCount > 0 ? FColor::White : FColor::Green, 0.f, false);  // 0.f not -1.f
	}
}

int CellSpace::PositionToIndex(FVector2D const& Pos) const
{
	float localX = Pos.X + SpaceWidth / 2.f;   
	float localY = Pos.Y + SpaceHeight / 2.f;  

	int col = FMath::Clamp(int(localX / CellWidth), 0, NrOfCols - 1);
	int row = FMath::Clamp(int(localY / CellHeight), 0, NrOfRows - 1);

	return row * NrOfCols + col;
}
bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	return true;
}