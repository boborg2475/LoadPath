#pragma once

#include "StaticsTestSupport.h"

namespace loadpath::test
{
inline Box BrickShape(double CentreXM = 0.0, int Course = 0, double Scale = 1.0)
{
	return Box{Vec3{Scale * CentreXM, 0.0, Scale * ((Course + 0.5) * BrickHeightM)}, Vec3{Scale * (BrickLengthM / 2.0), Scale * (BrickWidthM / 2.0), Scale * (BrickHeightM / 2.0)}, Scale * Scale * Scale * BrickMassKg};
}

struct BrickOnGround
{
	Structure Model;
	BodyId Brick;
	InterfaceId Bed;
	ProfileId Profile;
};

inline BrickOnGround MakeBrickOnGround(const StrengthProfile& Profile, double Scale = 1.0)
{
	BrickOnGround F;
	F.Profile = AddProfileOk(F.Model, Profile);
	const BodyId Ground = AddGroundPlane(F.Model);
	F.Brick = AddBox(F.Model, BrickShape(0.0, 0, Scale));
	F.Bed = AddInterfaceOk(F.Model, OnGroundPlane(Ground, F.Brick, BrickShape(0.0, 0, Scale), F.Profile));
	return F;
}

struct TwoBrickStack
{
	Structure Model;
	BodyId Lower;
	BodyId Upper;
	InterfaceId Bed;
	InterfaceId Joint;
};

inline TwoBrickStack MakeTwoBrickStack(const StrengthProfile& Profile, double ShiftXM, double Scale = 1.0)
{
	TwoBrickStack S;
	const ProfileId P = AddProfileOk(S.Model, Profile);
	const BodyId Ground = AddGroundPlane(S.Model);
	S.Lower = AddBox(S.Model, BrickShape(0.0, 0, Scale));
	S.Upper = AddBox(S.Model, BrickShape(ShiftXM, 1, Scale));
	S.Bed = AddInterfaceOk(S.Model, OnGroundPlane(Ground, S.Lower, BrickShape(0.0, 0, Scale), P));
	S.Joint = AddInterfaceOk(S.Model, *StackedFaces(S.Lower, BrickShape(0.0, 0, Scale), S.Upper, BrickShape(ShiftXM, 1, Scale), P));
	return S;
}

inline constexpr double BridgeHalfSpanM = 0.12;

struct ThreeBrickBridge
{
	Structure Model;
	BodyId Left;
	BodyId Right;
	BodyId Top;
	InterfaceId LeftBed;
	InterfaceId RightBed;
	InterfaceId LeftJoint;
	InterfaceId RightJoint;
};

inline ThreeBrickBridge MakeThreeBrickBridge(const StrengthProfile& Profile, double Scale = 1.0)
{
	ThreeBrickBridge B;
	const ProfileId P = AddProfileOk(B.Model, Profile);
	const BodyId Ground = AddGroundPlane(B.Model);
	const Box LeftShape = BrickShape(-BridgeHalfSpanM, 0, Scale);
	const Box RightShape = BrickShape(BridgeHalfSpanM, 0, Scale);
	const Box TopShape = BrickShape(0.0, 1, Scale);
	B.Left = AddBox(B.Model, LeftShape);
	B.Right = AddBox(B.Model, RightShape);
	B.Top = AddBox(B.Model, TopShape);
	B.LeftBed = AddInterfaceOk(B.Model, OnGroundPlane(Ground, B.Left, LeftShape, P));
	B.RightBed = AddInterfaceOk(B.Model, OnGroundPlane(Ground, B.Right, RightShape, P));
	B.LeftJoint = AddInterfaceOk(B.Model, *StackedFaces(B.Left, LeftShape, B.Top, TopShape, P));
	B.RightJoint = AddInterfaceOk(B.Model, *StackedFaces(B.Right, RightShape, B.Top, TopShape, P));
	return B;
}
}
