#include "loadpath/CollapseCertificate.h"

namespace loadpath
{
namespace
{
class LimitAnalysisCertificate final : public ICollapseCertificate
{
public:
	CertificateResult Decide(const Structure&, std::span<const BodyLoad>, const CertificateOptions&) const override
	{
		return CertificateResult{};
	}
};
}

std::unique_ptr<ICollapseCertificate> MakeLimitAnalysisCertificate()
{
	return std::make_unique<LimitAnalysisCertificate>();
}
}
