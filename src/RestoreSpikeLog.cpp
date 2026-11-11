#pragma warning(disable : 4996)
#include "loadpath/RestoreSpikeLog.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <process.h>

namespace loadpath::spike
{
namespace
{
std::mutex& Lock()
{
	static std::mutex M;
	return M;
}
std::string& TagRef()
{
	static std::string T;
	return T;
}
std::atomic<int> GPass{0};
FILE* File()
{
	static FILE* F = [] {
		const char* P = std::getenv("LOADPATH_RESTORE_LOG");
		if (P == nullptr || *P == 0)
		{
			return static_cast<FILE*>(nullptr);
		}
		const std::string Path = std::string(P) + "_" + std::to_string(_getpid()) + ".csv";
		return std::fopen(Path.c_str(), "a");
	}();
	return F;
}
}

bool Enabled() { return File() != nullptr; }

void SetTag(const std::string& Tag)
{
	std::lock_guard<std::mutex> G(Lock());
	TagRef() = Tag;
}

void SetCascadePass(int Pass) { GPass = Pass; }

int CascadePass() { return GPass; }

std::string Prefix()
{
	std::string T;
	{
		std::lock_guard<std::mutex> G(Lock());
		T = TagRef();
	}
	if (T.empty())
	{
		const char* E = std::getenv("LOADPATH_RESTORE_TAG");
		T = E ? E : "";
	}
	for (char& C : T)
	{
		if (C == ',')
		{
			C = ';';
		}
	}
	return T + "," + std::to_string(GPass.load());
}

void Line(const std::string& Text)
{
	FILE* F = File();
	if (F == nullptr)
	{
		return;
	}
	std::lock_guard<std::mutex> G(Lock());
	std::fputs(Text.c_str(), F);
	std::fputc('\n', F);
	std::fflush(F);
}
}
