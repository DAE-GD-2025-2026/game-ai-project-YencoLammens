#include "SpacePartitioning.h"
#include "DrawDebugHelpers.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left   = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width  = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left,         bottom          },
		{ left,         bottom + height },
		{ left + width, bottom + height },
		{ left + width, bottom          },
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

	CellWidth  = Width  / Cols;
	CellHeight = Height / Rows;

	CellOrigin = FVector2D(-Width * 0.5f, -Height * 0.5f);

	for (int row = 0; row < Rows; ++row)
		for (int col = 0; col < Cols; ++col)
			Cells.emplace_back(
				CellOrigin.X + col * CellWidth,
				CellOrigin.Y + row * CellHeight,
				CellWidth, CellHeight);
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	int Index = PositionToIndex(Agent.GetPosition());
	Cells[Index].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	//TODO Check if the agent needs to be moved to another cell.
	//TODO Use the calculated index for oldPos and currentPos for this

	int OldIndex = PositionToIndex(OldPos);
	int NewIndex = PositionToIndex(Agent.GetPosition());

	if (OldIndex != NewIndex)
	{
		Cells[OldIndex].Agents.remove(&Agent);
		Cells[NewIndex].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	// TODO Register the neighbors for the provided agent
	// TODO Only check the cells that are within the radius of the neighborhood

	NrOfNeighbors = 0;

	FRect QueryRect;
	FVector2D AgentPos = Agent.GetPosition();
	QueryRect.Min = AgentPos - FVector2D(QueryRadius, QueryRadius);
	QueryRect.Max = AgentPos + FVector2D(QueryRadius, QueryRadius);

	for (Cell& cell : Cells)
	{
		if (!DoRectsOverlap(QueryRect, cell.BoundingBox))
			continue;

		for (ASteeringAgent* pOther : cell.Agents)
		{
			if (!pOther || pOther == &Agent) continue;
			if (FVector2D::Distance(AgentPos, pOther->GetPosition()) <= QueryRadius)
				Neighbors[NrOfNeighbors++] = pOther;
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
		FVector2D Min = cell.BoundingBox.Min;
		FVector2D Max = cell.BoundingBox.Max;
		float Z = 100.f;

		FVector TopLeft    (Min.X, Min.Y, Z);
		FVector TopRight   (Max.X, Min.Y, Z);
		FVector BottomLeft (Min.X, Max.Y, Z);
		FVector BottomRight(Max.X, Max.Y, Z);

		DrawDebugLine(pWorld, TopLeft,     TopRight,    FColor::Blue, false, -1.f, SDPG_Foreground, 2.f);
		DrawDebugLine(pWorld, TopRight,    BottomRight, FColor::Blue, false, -1.f, SDPG_Foreground, 2.f);
		DrawDebugLine(pWorld, BottomRight, BottomLeft,  FColor::Blue, false, -1.f, SDPG_Foreground, 2.f);
		DrawDebugLine(pWorld, BottomLeft,  TopLeft,     FColor::Blue, false, -1.f, SDPG_Foreground, 2.f);

		int AgentCount = cell.Agents.size();
		if (AgentCount > 0)
			DrawDebugString(pWorld,
				FVector((Min + Max) * 0.5f, 120.f),
				FString::FromInt(AgentCount),
				nullptr, FColor::White, 0.f);
	}
}

int CellSpace::PositionToIndex(FVector2D const& Pos) const
{
	int Col = static_cast<int>((Pos.X - CellOrigin.X) / CellWidth);
	int Row = static_cast<int>((Pos.Y - CellOrigin.Y) / CellHeight);
	Col = FMath::Clamp(Col, 0, NrOfCols - 1);
	Row = FMath::Clamp(Row, 0, NrOfRows - 1);
	return Row * NrOfCols + Col;
}

bool CellSpace::DoRectsOverlap(FRect const& RectA, FRect const& RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;

	// If they are not separated, they must overlap
	return true;
}