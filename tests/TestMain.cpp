#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "loadpath/RestoreSpikeLog.h"

struct SpikeTestNameListener : doctest::IReporter
{
	explicit SpikeTestNameListener(const doctest::ContextOptions&) {}
	void report_query(const doctest::QueryData&) override {}
	void test_run_start() override {}
	void test_run_end(const doctest::TestRunStats&) override {}
	void test_case_start(const doctest::TestCaseData& In) override { loadpath::spike::SetTag(In.m_name); }
	void test_case_reenter(const doctest::TestCaseData&) override {}
	void test_case_end(const doctest::CurrentTestCaseStats&) override {}
	void test_case_exception(const doctest::TestCaseException&) override {}
	void subcase_start(const doctest::SubcaseSignature&) override {}
	void subcase_end() override {}
	void log_assert(const doctest::AssertData&) override {}
	void log_message(const doctest::MessageData&) override {}
	void test_case_skipped(const doctest::TestCaseData&) override {}
};

REGISTER_LISTENER("spike_test_name", 1, SpikeTestNameListener);
