#pragma once

#include "loadpath/StructureFile.h"

#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

// The one JSON reader, shared by the structure file and the catalogue loader.
namespace loadpath::detail
{
inline constexpr int MaxDepth = 64;
inline constexpr double Infinity = std::numeric_limits<double>::infinity();
inline constexpr double NotANumber = std::numeric_limits<double>::quiet_NaN();

enum class JsonType : std::uint8_t
{
	Null,
	Bool,
	Number,
	String,
	Array,
	Object
};

struct JsonValue
{
	JsonType Type = JsonType::Null;
	int Line = 0;
	bool Bool = false;
	double Number = 0;
	std::string Text;
	std::vector<std::string> Keys;
	std::vector<JsonValue> Items;
};

inline void AppendUtf8(std::string& Out, std::uint32_t Code)
{
	if (Code < 0x80u)
	{
		Out += static_cast<char>(Code);
	}
	else if (Code < 0x800u)
	{
		Out += static_cast<char>(0xC0u | (Code >> 6));
		Out += static_cast<char>(0x80u | (Code & 0x3Fu));
	}
	else if (Code < 0x10000u)
	{
		Out += static_cast<char>(0xE0u | (Code >> 12));
		Out += static_cast<char>(0x80u | ((Code >> 6) & 0x3Fu));
		Out += static_cast<char>(0x80u | (Code & 0x3Fu));
	}
	else
	{
		Out += static_cast<char>(0xF0u | (Code >> 18));
		Out += static_cast<char>(0x80u | ((Code >> 12) & 0x3Fu));
		Out += static_cast<char>(0x80u | ((Code >> 6) & 0x3Fu));
		Out += static_cast<char>(0x80u | (Code & 0x3Fu));
	}
}

inline bool IsHighSurrogate(std::uint32_t Code) { return Code >= 0xD800u && Code <= 0xDBFFu; }

inline bool IsLowSurrogate(std::uint32_t Code) { return Code >= 0xDC00u && Code <= 0xDFFFu; }

inline const JsonValue* Find(const JsonValue& Object, std::string_view Key)
{
	for (std::size_t Index = Object.Keys.size(); Index-- > 0;)
	{
		if (Object.Keys[Index] == Key)
		{
			return &Object.Items[Index];
		}
	}
	return nullptr;
}

class JsonParser
{
public:
	explicit JsonParser(std::string_view Json)
		: Text(Json)
	{
	}

	bool ParseDocument(JsonValue& Root)
	{
		SkipWhitespace();
		if (!Peek('{'))
		{
			Error = {0, "the document is not a JSON object"};
			return false;
		}
		if (!ParseValue(Root, 1))
		{
			return false;
		}
		SkipWhitespace();
		return Pos == Text.size() || Fail("unexpected text after the document");
	}

	FileError Error;

private:
	bool AtEnd() const { return Pos >= Text.size(); }

	bool Peek(char C) const { return !AtEnd() && Text[Pos] == C; }

	bool Consume(char C)
	{
		if (!Peek(C))
		{
			return false;
		}
		++Pos;
		return true;
	}

	bool ConsumeDigits()
	{
		const std::size_t Start = Pos;
		while (!AtEnd() && Text[Pos] >= '0' && Text[Pos] <= '9')
		{
			++Pos;
		}
		return Pos > Start;
	}

	bool Fail(std::string Message)
	{
		Error = {Line, std::move(Message)};
		return false;
	}

	void SkipWhitespace()
	{
		while (!AtEnd() && (Text[Pos] == ' ' || Text[Pos] == '\t' || Text[Pos] == '\r' || Text[Pos] == '\n'))
		{
			if (Text[Pos] == '\n')
			{
				++Line;
			}
			++Pos;
		}
	}

	bool Enter(int Depth) { return Depth <= MaxDepth || Fail("nesting deeper than 64 levels"); }

	bool ParseLiteral(std::string_view Word)
	{
		if (Text.substr(Pos, Word.size()) != Word)
		{
			return Fail("unexpected character");
		}
		Pos += Word.size();
		return true;
	}

	bool ParseValue(JsonValue& Out, int Depth)
	{
		SkipWhitespace();
		if (AtEnd())
		{
			return Fail("unexpected end of input");
		}
		Out.Line = Line;
		const char C = Text[Pos];
		switch (C)
		{
		case '{': return ParseObject(Out, Depth);
		case '[': return ParseArray(Out, Depth);
		case '"': Out.Type = JsonType::String; return ParseString(Out.Text);
		case 't': Out.Type = JsonType::Bool; Out.Bool = true; return ParseLiteral("true");
		case 'f': Out.Type = JsonType::Bool; return ParseLiteral("false");
		case 'n': return ParseLiteral("null");
		default: break;
		}
		if (C == '-' || (C >= '0' && C <= '9'))
		{
			return ParseNumber(Out);
		}
		return Fail("unexpected character");
	}

	bool ParseNumber(JsonValue& Out)
	{
		const std::size_t Start = Pos;
		Consume('-');
		if (!Consume('0') && !ConsumeDigits())
		{
			return Fail("invalid number");
		}
		if (Consume('.') && !ConsumeDigits())
		{
			return Fail("invalid number");
		}
		if (Consume('e') || Consume('E'))
		{
			if (!Consume('+'))
			{
				Consume('-');
			}
			if (!ConsumeDigits())
			{
				return Fail("invalid number");
			}
		}
		Out.Number = NotANumber;
		const std::from_chars_result Parsed = std::from_chars(Text.data() + Start, Text.data() + Pos, Out.Number);
		if (Parsed.ec != std::errc() && !std::isfinite(Out.Number))
		{
			return Fail("number out of range");
		}
		Out.Type = JsonType::Number;
		return true;
	}

	bool ParseString(std::string& Out)
	{
		++Pos;
		while (!AtEnd())
		{
			const char C = Text[Pos++];
			if (C == '"')
			{
				return true;
			}
			if (static_cast<unsigned char>(C) < 0x20u)
			{
				return Fail("control character in a string");
			}
			if (C == '\\')
			{
				if (!ParseEscape(Out))
				{
					return false;
				}
				continue;
			}
			Out += C;
		}
		return Fail("unterminated string");
	}

	bool ParseEscape(std::string& Out)
	{
		if (AtEnd())
		{
			return Fail("unterminated string");
		}
		switch (Text[Pos++])
		{
		case '"': Out += '"'; return true;
		case '\\': Out += '\\'; return true;
		case '/': Out += '/'; return true;
		case 'b': Out += '\b'; return true;
		case 'f': Out += '\f'; return true;
		case 'n': Out += '\n'; return true;
		case 'r': Out += '\r'; return true;
		case 't': Out += '\t'; return true;
		case 'u': return ParseUnicodeEscape(Out);
		default: return Fail("unsupported escape");
		}
	}

	bool ParseHex4(std::uint32_t& Out)
	{
		if (Text.size() - Pos < 4)
		{
			return Fail("invalid \\u escape");
		}
		const char* First = Text.data() + Pos;
		const std::from_chars_result Parsed = std::from_chars(First, First + 4, Out, 16);
		if (Parsed.ec != std::errc() || Parsed.ptr != First + 4)
		{
			return Fail("invalid \\u escape");
		}
		Pos += 4;
		return true;
	}

	bool ParseUnicodeEscape(std::string& Out)
	{
		std::uint32_t Code = 0;
		if (!ParseHex4(Code))
		{
			return false;
		}
		if (IsLowSurrogate(Code))
		{
			return Fail("lone surrogate");
		}
		if (IsHighSurrogate(Code))
		{
			if (!Consume('\\') || !Consume('u'))
			{
				return Fail("lone surrogate");
			}
			std::uint32_t Low = 0;
			if (!ParseHex4(Low))
			{
				return false;
			}
			if (!IsLowSurrogate(Low))
			{
				return Fail("lone surrogate");
			}
			Code = 0x10000u + ((Code - 0xD800u) << 10) + (Low - 0xDC00u);
		}
		AppendUtf8(Out, Code);
		return true;
	}

	bool ParseObject(JsonValue& Out, int Depth)
	{
		Out.Type = JsonType::Object;
		++Pos;
		if (!Enter(Depth))
		{
			return false;
		}
		SkipWhitespace();
		if (Consume('}'))
		{
			return true;
		}
		while (true)
		{
			SkipWhitespace();
			std::string Key;
			if (!Peek('"'))
			{
				return Fail("expected a string key");
			}
			if (!ParseString(Key))
			{
				return false;
			}
			SkipWhitespace();
			if (!Consume(':'))
			{
				return Fail("expected ':'");
			}
			JsonValue Value;
			if (!ParseValue(Value, Depth + 1))
			{
				return false;
			}
			Out.Keys.push_back(std::move(Key));
			Out.Items.push_back(std::move(Value));
			SkipWhitespace();
			if (Consume('}'))
			{
				return true;
			}
			if (!Consume(','))
			{
				return Fail("expected ',' or '}'");
			}
		}
	}

	bool ParseArray(JsonValue& Out, int Depth)
	{
		Out.Type = JsonType::Array;
		++Pos;
		if (!Enter(Depth))
		{
			return false;
		}
		SkipWhitespace();
		if (Consume(']'))
		{
			return true;
		}
		while (true)
		{
			JsonValue Value;
			if (!ParseValue(Value, Depth + 1))
			{
				return false;
			}
			Out.Items.push_back(std::move(Value));
			SkipWhitespace();
			if (Consume(']'))
			{
				return true;
			}
			if (!Consume(','))
			{
				return Fail("expected ',' or ']'");
			}
		}
	}

	std::string_view Text;
	std::size_t Pos = 0;
	int Line = 1;
};

inline const char* ModelErrorText(ModelError Error)
{
	switch (Error)
	{
	case ModelError::None: return "None";
	case ModelError::NonFiniteInput: return "NonFiniteInput";
	case ModelError::NegativeMass: return "NegativeMass";
	case ModelError::DegenerateNormal: return "DegenerateNormal";
	case ModelError::AxesNotOrthonormal: return "AxesNotOrthonormal";
	case ModelError::NonPositiveExtent: return "NonPositiveExtent";
	case ModelError::UnknownBody: return "UnknownBody";
	case ModelError::UnknownProfile: return "UnknownProfile";
	case ModelError::SameBody: return "SameBody";
	case ModelError::BodyRemoved: return "BodyRemoved";
	case ModelError::NegativeStrength: return "NegativeStrength";
	case ModelError::NonPositiveStiffness: return "NonPositiveStiffness";
	}
	return "ModelError(?)";
}

class ElementFields
{
public:
	ElementFields(const JsonValue& InElement, std::string InLabel, FileError& InError)
		: Label(std::move(InLabel))
		, Element(InElement)
		, Error(InError)
	{
	}

	bool Fail(const std::string& Message)
	{
		Error = {Element.Line, Label + ": " + Message};
		return false;
	}

	bool Number(std::string_view Key, double& Out, bool bRequired)
	{
		const JsonValue* Value = nullptr;
		if (!Get(Key, JsonType::Number, bRequired, Value))
		{
			return false;
		}
		if (Value != nullptr)
		{
			Out = Value->Number;
		}
		return true;
	}

	bool Boolean(std::string_view Key, bool& Out)
	{
		const JsonValue* Value = nullptr;
		if (!Get(Key, JsonType::Bool, false, Value))
		{
			return false;
		}
		if (Value != nullptr)
		{
			Out = Value->Bool;
		}
		return true;
	}

	bool String(std::string_view Key, std::string& Out, bool bRequired = true)
	{
		const JsonValue* Value = nullptr;
		if (!Get(Key, JsonType::String, bRequired, Value))
		{
			return false;
		}
		if (Value != nullptr)
		{
			Out = Value->Text;
		}
		return true;
	}

	bool Vector(std::string_view Key, Vec3& Out, bool bRequired)
	{
		const JsonValue* Value = nullptr;
		if (!Get(Key, JsonType::Array, bRequired, Value))
		{
			return false;
		}
		if (Value == nullptr)
		{
			return true;
		}
		const std::vector<JsonValue>& Items = Value->Items;
		if (Items.size() != 3 || Items[0].Type != JsonType::Number || Items[1].Type != JsonType::Number ||
			Items[2].Type != JsonType::Number)
		{
			return Fail(std::string(Key) + " must hold 3 numbers");
		}
		Out = Vec3{Items[0].Number, Items[1].Number, Items[2].Number};
		return true;
	}

	bool Elements(std::string_view Key, const JsonValue*& Out) { return Get(Key, JsonType::Array, false, Out); }

	std::string Label;

private:
	bool Get(std::string_view Key, JsonType Type, bool bRequired, const JsonValue*& Out)
	{
		Out = Find(Element, Key);
		if (Out == nullptr)
		{
			return !bRequired || Fail("missing " + std::string(Key));
		}
		if (Out->Type != Type)
		{
			return Fail(std::string(Key) + " has the wrong type");
		}
		return true;
	}

	const JsonValue& Element;
	FileError& Error;
};

}
