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
	
	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	CellOrigin = FVector2D{ -Width / 2.f, -Height / 2.f };

	for (int row = 0; row < Rows; ++row)
	{
		for (int col = 0; col < Cols; ++col)
		{
			float Left = CellOrigin.X + col * CellWidth;
			float Bottom = CellOrigin.Y + row * CellHeight;
			Cells.emplace_back(Left, Bottom, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	int Index = PositionToIndex(Agent.GetPosition());
	if (Index >= 0 && Index < static_cast<int>(Cells.size()))
		Cells[Index].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	int OldIndex = PositionToIndex(OldPos);
	int NewIndex = PositionToIndex(Agent.GetPosition());

	if (OldIndex != NewIndex)
	{
		if (OldIndex >= 0 && OldIndex < static_cast<int>(Cells.size()))
			Cells[OldIndex].Agents.remove(&Agent);

		if (NewIndex >= 0 && NewIndex < static_cast<int>(Cells.size()))
			Cells[NewIndex].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	FVector2D AgentPos = Agent.GetPosition();
	FRect QueryRect;
	QueryRect.Min = { AgentPos.X - QueryRadius, AgentPos.Y - QueryRadius };
	QueryRect.Max = { AgentPos.X + QueryRadius, AgentPos.Y + QueryRadius };

	for (Cell& cell : Cells)
	{
		if (!DoRectsOverlap(QueryRect, cell.BoundingBox))
			continue;

		for (ASteeringAgent* pOther : cell.Agents)
		{
			if (pOther == &Agent) continue;

			float Distance = FVector2D::Distance(AgentPos, pOther->GetPosition());
			if (Distance < QueryRadius)
			{
				Neighbors[NrOfNeighbors] = pOther;
				++NrOfNeighbors;
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
	for (const Cell& cell : Cells)
	{
		FVector Center = FVector(
			(cell.BoundingBox.Min.X + cell.BoundingBox.Max.X) * 0.5f,
			(cell.BoundingBox.Min.Y + cell.BoundingBox.Max.Y) * 0.5f,
			0.f);

		int Count = static_cast<int>(cell.Agents.size());
		if (Count > 0)
		{
			DrawDebugString(pWorld, Center, FString::FromInt(Count), nullptr, FColor::Green, 0.f);
		}
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	int Col = static_cast<int>((Pos.X - CellOrigin.X) / CellWidth);
	int Row = static_cast<int>((Pos.Y - CellOrigin.Y) / CellHeight);

	Col = FMath::Clamp(Col, 0, NrOfCols - 1);
	Row = FMath::Clamp(Row, 0, NrOfRows - 1);

	return Row * NrOfCols + Col;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}