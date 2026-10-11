#include "loadpath/StructuralAuthority.h"

#include <utility>

namespace loadpath
{
namespace
{
class CombinedAuthority final : public IStructuralAuthority
{
public:
	CombinedAuthority(std::unique_ptr<IStandingAuthority> InStanding, std::unique_ptr<ICollapseCertificate> InCertificate)
		: Standing(std::move(InStanding))
		, Certificate(std::move(InCertificate))
	{
	}

	AuthorityResult Judge(const Structure&, std::span<const BodyLoad>, const AuthorityOptions&) const override
	{
		return AuthorityResult{};
	}

private:
	std::unique_ptr<IStandingAuthority> Standing;
	std::unique_ptr<ICollapseCertificate> Certificate;
};
}

std::unique_ptr<IStructuralAuthority> MakeCombinedAuthority(std::unique_ptr<IStandingAuthority> Standing, std::unique_ptr<ICollapseCertificate> Certificate)
{
	return std::make_unique<CombinedAuthority>(std::move(Standing), std::move(Certificate));
}
}
