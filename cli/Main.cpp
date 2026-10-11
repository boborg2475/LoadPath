#include "loadpath/Cli.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

int main(int Argc, char** Argv)
{
	const std::vector<std::string_view> Args(Argv + 1, Argv + Argc);
	std::optional<std::string> Contents;
	if (Args.size() == 1)
	{
		std::ifstream File(Argv[1], std::ios::binary);
		if (File)
		{
			Contents.emplace(std::istreambuf_iterator<char>(File), std::istreambuf_iterator<char>());
		}
	}
	std::string Out;
	std::string Err;
	const std::optional<std::string_view> View = Contents ? std::optional<std::string_view>(*Contents) : std::nullopt;
	const int Code = loadpath::RunCli(Args, View, Out, Err);
	std::fwrite(Out.data(), 1, Out.size(), stdout);
	std::fwrite(Err.data(), 1, Err.size(), stderr);
	return Code;
}
