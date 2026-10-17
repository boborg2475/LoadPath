#include "loadpath/Cli.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace
{
class DiskFiles final : public loadpath::IFileStore
{
public:
	std::optional<std::string> Read(std::string_view Path) const override
	{
		std::ifstream File{std::string(Path), std::ios::binary};
		if (!File)
		{
			return std::nullopt;
		}
		return std::string(std::istreambuf_iterator<char>(File), std::istreambuf_iterator<char>());
	}

	bool Write(std::string_view Path, std::string_view Contents) override
	{
		std::ofstream File{std::string(Path), std::ios::binary};
		File.write(Contents.data(), static_cast<std::streamsize>(Contents.size()));
		return static_cast<bool>(File);
	}

	std::vector<std::string> List(std::string_view Dir) const override
	{
		std::vector<std::string> Names;
		std::error_code Error;
		std::filesystem::directory_iterator It(std::filesystem::path(std::string(Dir)), Error);
		const std::filesystem::directory_iterator End;
		while (!Error && It != End)
		{
			if (It->is_regular_file(Error))
			{
				Names.push_back(It->path().filename().string());
			}
			It.increment(Error);
		}
		std::sort(Names.begin(), Names.end());
		return Names;
	}
};
}

int main(int Argc, char** Argv)
{
	const std::vector<std::string_view> Args(Argv + 1, Argv + Argc);
	DiskFiles Files;
	std::string Out;
	std::string Err;
	const int Code = loadpath::RunCli(Args, Files, Out, Err);
	std::fwrite(Out.data(), 1, Out.size(), stdout);
	std::fwrite(Err.data(), 1, Err.size(), stderr);
	return Code;
}
