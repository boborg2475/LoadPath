#include "loadpath/Cascade.h"

#include <utility>

namespace loadpath
{
namespace
{
class BrittleCascade final : public ICascade
{
public:
	explicit BrittleCascade(std::unique_ptr<IStructuralAuthority> InAuthority)
		: Authority(std::move(InAuthority))
	{
	}

	CascadeVerdict Run(Structure&, std::span<const BodyLoad>, const CascadeOptions&) const override
	{
		return CascadeVerdict{};
	}

private:
	std::unique_ptr<IStructuralAuthority> Authority;
};
}

std::unique_ptr<ICascade> MakeCascade(std::unique_ptr<IStructuralAuthority> Authority)
{
	return std::make_unique<BrittleCascade>(std::move(Authority));
}
}
