#pragma once

#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace loadpath
{
// The CLI's only access to files, so tests can pass an in-memory store.
class IFileStore
{
public:
	virtual ~IFileStore() = default;
	// Nullopt when the file cannot be read.
	virtual std::optional<std::string> Read(std::string_view Path) const = 0;
	virtual bool Write(std::string_view Path, std::string_view Contents) = 0;
	// File names in Dir, sorted.
	virtual std::vector<std::string> List(std::string_view Dir) const = 0;
};

// Args excludes the program name; FileContents is nullopt when the file could not be read. Returns the exit code.
int RunCli(std::span<const std::string_view> Args, std::optional<std::string_view> FileContents, std::string& Out, std::string& Err);

// Adds `scenario <file> [--viz <out>] [--catalogue <path>] [--settings <path>] [--overrides <path>] [--record-passes]`: exit 0 expectation met, 1 not met, 2 the file cannot be run.
// Settings default to data/settings.json (absent: compiled defaults) and overrides to data/settings.local.json (absent is fine); a bad settings file exits 2 naming the key, and --record-passes sets debug.record_passes.
int RunCli(std::span<const std::string_view> Args, IFileStore& Files, std::string& Out, std::string& Err);
}
