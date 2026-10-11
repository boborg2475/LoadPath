#include "StaticsTestSupport.h"

#include "loadpath/geometry/BoxContacts.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double Exact = 1e-12;

struct Pair
{
	Structure Model;
	ProfileId Profile;
	BodyId A;
	BodyId B;
	geometry::Box BoxA;
	geometry::Box BoxB;
};

Pair MakePair(const Vec3& CentreA, const Vec3& CentreB, const Vec3& HalfM = Vec3{0.5, 0.5, 0.5})
{
	Pair P;
	P.Profile = AddProfileOk(P.Model, DryJoint(0.6));
	P.BoxA = geometry::Box{CentreA, HalfM};
	P.BoxB = geometry::Box{CentreB, HalfM};
	P.A = AddBox(P.Model, Box{CentreA, HalfM, 1.0});
	P.B = AddBox(P.Model, Box{CentreB, HalfM, 1.0});
	return P;
}

std::optional<InterfaceDesc> ContactOf(const Pair& P)
{
	return geometry::FaceContact(P.A, P.BoxA, P.B, P.BoxB, P.Profile);
}

void CheckVec(const Vec3& Got, const Vec3& Want, const char* What)
{
	INFO(What << ": got " << Got << ", want " << Want);
	CHECK(Length(Minus(Got, Want)) <= Exact);
}

void CheckRectangle(const InterfaceDesc& Desc, const Vec3& Normal, const Vec3& Centre, double HalfAlongX, double HalfAlongY, double HalfAlongZ)
{
	CheckVec(Desc.NormalAToB, Normal, "normal");
	CheckVec(Desc.CentreM, Centre, "centre");
	CHECK(Dot(Cross(Desc.AxisU, Desc.AxisV), Normal) == doctest::Approx(1.0).epsilon(Exact));
	const Vec3 Extent = Plus(Times(Desc.HalfExtentUM, Desc.AxisU), Times(Desc.HalfExtentVM, Desc.AxisV));
	INFO("half extents along x, y, z: " << Vec3{std::abs(Extent.X), std::abs(Extent.Y), std::abs(Extent.Z)});
	CHECK(std::abs(std::abs(Extent.X) - HalfAlongX) <= Exact);
	CHECK(std::abs(std::abs(Extent.Y) - HalfAlongY) <= Exact);
	CHECK(std::abs(std::abs(Extent.Z) - HalfAlongZ) <= Exact);
}

bool HasContact(const std::optional<InterfaceDesc>& Contact)
{
	INFO("FaceContact returned " << std::string(Contact.has_value() ? "a contact" : "nullopt"));
	REQUIRE(Contact.has_value());
	return Contact.has_value();
}
}

TEST_CASE("Geometry.Contact.StackedUnitCubesShareTheWholeFaceWithNormalUp")
{
	Pair P = MakePair(Vec3{0, 0, 0.5}, Vec3{0, 0, 1.5});
	const std::optional<InterfaceDesc> Contact = ContactOf(P);
	if (!HasContact(Contact))
	{
		return;
	}
	const InterfaceDesc& Desc = *Contact;
	CHECK(SameHandle(Desc.BodyA, P.A));
	CHECK(SameHandle(Desc.BodyB, P.B));
	CHECK(SameHandle(Desc.Profile, P.Profile));
	CHECK(Desc.HalfExtentUM == doctest::Approx(0.5).epsilon(Exact));
	CHECK(Desc.HalfExtentVM == doctest::Approx(0.5).epsilon(Exact));
	CheckRectangle(Desc, Vec3{0, 0, 1}, Vec3{0, 0, 1}, 0.5, 0.5, 0.0);
	AddInterfaceOk(P.Model, Desc);
}

TEST_CASE("Geometry.Contact.ShiftedHalfACubeGivesTheOverlapRectangleAtItsCentre")
{
	Pair P = MakePair(Vec3{0, 0, 0.5}, Vec3{0.5, 0, 1.5});
	const std::optional<InterfaceDesc> Contact = ContactOf(P);
	if (!HasContact(Contact))
	{
		return;
	}
	const InterfaceDesc& Desc = *Contact;
	CheckRectangle(Desc, Vec3{0, 0, 1}, Vec3{0.25, 0, 1}, 0.25, 0.5, 0.0);
	CHECK(Desc.HalfExtentUM * Desc.HalfExtentVM == doctest::Approx(0.125).epsilon(Exact));
	AddInterfaceOk(P.Model, Desc);
}

TEST_CASE("Geometry.Contact.GapAndToleranceDecideTouching")
{
	const double Tolerance = 1e-9;
	{
		Pair P = MakePair(Vec3{0, 0, 0.5}, Vec3{0, 0, 1.5 + 2.0 * Tolerance});
		CHECK_FALSE(ContactOf(P).has_value());
	}
	{
		Pair P = MakePair(Vec3{0, 0, 0.5}, Vec3{0, 0, 1.5 + 0.5 * Tolerance});
		const std::optional<InterfaceDesc> Contact = ContactOf(P);
		if (!HasContact(Contact))
		{
			return;
		}
		const InterfaceDesc& Desc = *Contact;
		CHECK(Desc.NormalAToB.Z == doctest::Approx(1.0).epsilon(Exact));
		AddInterfaceOk(P.Model, Desc);
	}
}

TEST_CASE("Geometry.Contact.OverlapInVolumeIsNotAContact")
{
	Pair P = MakePair(Vec3{0, 0, 0.5}, Vec3{0, 0, 1.25});
	CHECK_FALSE(ContactOf(P).has_value());
}

TEST_CASE("Geometry.Contact.SideBySideAlongXPointsFromTheLeftBodyToTheRight")
{
	Pair P = MakePair(Vec3{1, 0, 0.5}, Vec3{0, 0, 0.5});
	const std::optional<InterfaceDesc> Contact = ContactOf(P);
	if (!HasContact(Contact))
	{
		return;
	}
	const InterfaceDesc& Desc = *Contact;
	CHECK(SameHandle(Desc.BodyA, P.B));
	CHECK(SameHandle(Desc.BodyB, P.A));
	CheckRectangle(Desc, Vec3{1, 0, 0}, Vec3{0.5, 0, 0.5}, 0.0, 0.5, 0.5);
	AddInterfaceOk(P.Model, Desc);
}

TEST_CASE("Geometry.Contact.StackAlongYPointsAlongY")
{
	Pair P = MakePair(Vec3{0, 0, 0.5}, Vec3{0, 1, 0.5});
	const std::optional<InterfaceDesc> Contact = ContactOf(P);
	if (!HasContact(Contact))
	{
		return;
	}
	const InterfaceDesc& Desc = *Contact;
	CheckRectangle(Desc, Vec3{0, 1, 0}, Vec3{0, 0.5, 0.5}, 0.5, 0.0, 0.5);
	AddInterfaceOk(P.Model, Desc);
}

TEST_CASE("Geometry.Contact.ArgumentOrderSwapsBodiesAndReversesNothingElse")
{
	Pair P = MakePair(Vec3{0, 0, 0.5}, Vec3{0.5, 0, 1.5});
	const std::optional<InterfaceDesc> Forward = geometry::FaceContact(P.A, P.BoxA, P.B, P.BoxB, P.Profile);
	const std::optional<InterfaceDesc> Backward = geometry::FaceContact(P.B, P.BoxB, P.A, P.BoxA, P.Profile);
	if (!HasContact(Forward) || !HasContact(Backward))
	{
		return;
	}
	const InterfaceDesc& F = *Forward;
	const InterfaceDesc& R = *Backward;
	CHECK(SameHandle(F.BodyA, R.BodyA));
	CHECK(SameHandle(F.BodyB, R.BodyB));
	CheckVec(R.NormalAToB, F.NormalAToB, "normal");
	CheckVec(R.CentreM, F.CentreM, "centre");
	CHECK(std::min(F.HalfExtentUM, F.HalfExtentVM) == doctest::Approx(std::min(R.HalfExtentUM, R.HalfExtentVM)).epsilon(Exact));
	CHECK(std::max(F.HalfExtentUM, F.HalfExtentVM) == doctest::Approx(std::max(R.HalfExtentUM, R.HalfExtentVM)).epsilon(Exact));
}
