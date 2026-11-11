#pragma once

#include <string>

namespace loadpath::spike
{
// Spike-only restore instrumentation, active only when LOADPATH_RESTORE_LOG names a file prefix.
bool Enabled();
void SetTag(const std::string& Tag);
void SetCascadePass(int Pass);
int CascadePass();
void Line(const std::string& Text);
std::string Prefix();
}
