#pragma once

#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace loadpath
{
// Args excludes the program name; FileContents is nullopt when the file could not be read. Returns the exit code.
int RunCli(std::span<const std::string_view> Args, std::optional<std::string_view> FileContents, std::string& Out, std::string& Err);
}
