package main

import (
	"strings"
	"testing"
)

func TestSanitizePackageName(t *testing.T) {
	tests := []struct {
		input    string
		expected string
	}{
		{"std_msgs", "std_msgs"},
		{"example-interfaces", "example_interfaces"},
		{"my-package", "my_package"},
	}

	for _, test := range tests {
		result := sanitizePackageName(test.input)
		if result != test.expected {
			t.Errorf("sanitizePackageName(%q) = %q, expected %q", test.input, result, test.expected)
		}
	}
}

func TestGenerateGoMessage(t *testing.T) {
	msg := MessageDefinition{
		Package:  "std_msgs",
		Name:     "String",
		FullName: "std_msgs/String",
		TypeHash: "RIHS01_1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef",
		Fields: []FieldDefinition{
			{
				Name:      "data",
				FieldType: FieldType{Kind: "String"},
				IsArray:   false,
			},
		},
		Constants: []ConstantDefinition{},
	}

	code, err := GenerateGoMessage(msg, "rosz")
	if err != nil {
		t.Fatalf("GenerateGoMessage failed: %v", err)
	}

	codeStr := string(code)

	// Verify the generated code contains expected elements
	expectedElements := []string{
		"package std_msgs",
		"type String struct",
		"Data string",
		"String_TypeName = \"std_msgs::msg::dds_::String_\"",
		"String_TypeHash = \"RIHS01_",
		"func (m *String) TypeName() string",
		"func (m *String) TypeHash() string",
		"func (m *String) SerializeCDR() ([]byte, error)",
		"func (m *String) DeserializeCDR(data []byte) error",
	}

	for _, element := range expectedElements {
		if !strings.Contains(codeStr, element) {
			t.Errorf("Generated code missing expected element: %q", element)
		}
	}
}

func TestFieldToGoType(t *testing.T) {
	tests := []struct {
		field    FieldDefinition
		expected string
	}{
		{
			FieldDefinition{
				FieldType: FieldType{Kind: "String"},
				IsArray:   false,
			},
			"string",
		},
		{
			FieldDefinition{
				FieldType: FieldType{Kind: "Int32"},
				IsArray:   false,
			},
			"int32",
		},
		{
			FieldDefinition{
				FieldType: FieldType{Kind: "Float64"},
				IsArray:   false,
			},
			"float64",
		},
		{
			FieldDefinition{
				FieldType: FieldType{Kind: "Bool"},
				IsArray:   false,
			},
			"bool",
		},
		{
			FieldDefinition{
				FieldType: FieldType{Kind: "Custom", Package: stringPtr("std_msgs"), Name: stringPtr("Header")},
				IsArray:   false,
			},
			"std_msgs.Header",
		},
		{
			FieldDefinition{
				FieldType: FieldType{Kind: "String"},
				IsArray:   true,
				ArraySize: nil,
			},
			"[]string",
		},
		{
			FieldDefinition{
				FieldType: FieldType{Kind: "Int32"},
				IsArray:   true,
				ArrayKind: "fixed",
				ArraySize: uintPtr(10),
			},
			"[10]int32",
		},
	}

	for _, test := range tests {
		result := fieldToGoType(test.field)
		if result != test.expected {
			t.Errorf("fieldToGoType(%+v) = %q, expected %q", test.field, result, test.expected)
		}
	}
}

func TestCapitalize(t *testing.T) {
	tests := []struct {
		input    string
		expected string
	}{
		{"data", "Data"},
		{"count", "Count"},
		{"", ""},
		{"a", "A"},
		{"ABC", "ABC"},
	}

	for _, test := range tests {
		result := capitalize(test.input)
		if result != test.expected {
			t.Errorf("capitalize(%q) = %q, expected %q", test.input, result, test.expected)
		}
	}
}

func stringPtr(s string) *string {
	return &s
}

func uintPtr(u uint) *uint {
	return &u
}

func TestGenerateGoMessageWithAllTypes(t *testing.T) {
	msg := MessageDefinition{
		Package:  "test_msgs",
		Name:     "AllTypes",
		FullName: "test_msgs/AllTypes",
		TypeHash: "RIHS01_abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890",
		Fields: []FieldDefinition{
			{Name: "bool_field", FieldType: FieldType{Kind: "Bool"}, IsArray: false},
			{Name: "int8_field", FieldType: FieldType{Kind: "Int8"}, IsArray: false},
			{Name: "uint8_field", FieldType: FieldType{Kind: "UInt8"}, IsArray: false},
			{Name: "int16_field", FieldType: FieldType{Kind: "Int16"}, IsArray: false},
			{Name: "uint16_field", FieldType: FieldType{Kind: "UInt16"}, IsArray: false},
			{Name: "int32_field", FieldType: FieldType{Kind: "Int32"}, IsArray: false},
			{Name: "uint32_field", FieldType: FieldType{Kind: "UInt32"}, IsArray: false},
			{Name: "int64_field", FieldType: FieldType{Kind: "Int64"}, IsArray: false},
			{Name: "uint64_field", FieldType: FieldType{Kind: "UInt64"}, IsArray: false},
			{Name: "float32_field", FieldType: FieldType{Kind: "Float32"}, IsArray: false},
			{Name: "float64_field", FieldType: FieldType{Kind: "Float64"}, IsArray: false},
			{Name: "string_field", FieldType: FieldType{Kind: "String"}, IsArray: false},
		},
		Constants: []ConstantDefinition{},
	}

	code, err := GenerateGoMessage(msg, "test")
	if err != nil {
		t.Fatalf("GenerateGoMessage failed: %v", err)
	}

	codeStr := string(code)

	// Verify struct fields exist (capitalize() only capitalizes the first letter)
	// The expected pattern is "FieldName type" but gofmt may change spacing
	expectedFieldNames := []string{
		"Bool_field",
		"Int8_field",
		"Uint8_field",
		"Int16_field",
		"Uint16_field",
		"Int32_field",
		"Uint32_field",
		"Int64_field",
		"Uint64_field",
		"Float32_field",
		"Float64_field",
		"String_field",
	}

	for _, fieldName := range expectedFieldNames {
		if !strings.Contains(codeStr, fieldName) {
			t.Errorf("Generated code missing expected field name: %q\nGenerated:\n%s", fieldName, codeStr[:500])
		}
	}

	// Verify math import for float types
	if !strings.Contains(codeStr, `"math"`) {
		t.Error("Generated code should import math for float types")
	}
}

func TestGenerateGoMessageWithArrays(t *testing.T) {
	fixedSize := uint(5)
	msg := MessageDefinition{
		Package:  "test_msgs",
		Name:     "ArrayTypes",
		FullName: "test_msgs/ArrayTypes",
		TypeHash: "RIHS01_1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef",
		Fields: []FieldDefinition{
			{Name: "unbounded_strings", FieldType: FieldType{Kind: "String"}, IsArray: true, ArrayKind: "unbounded"},
			{Name: "fixed_ints", FieldType: FieldType{Kind: "Int32"}, IsArray: true, ArrayKind: "fixed", ArraySize: &fixedSize},
			{Name: "bounded_floats", FieldType: FieldType{Kind: "Float64"}, IsArray: true, ArrayKind: "bounded", ArraySize: &fixedSize},
		},
		Constants: []ConstantDefinition{},
	}

	code, err := GenerateGoMessage(msg, "test")
	if err != nil {
		t.Fatalf("GenerateGoMessage failed: %v", err)
	}

	codeStr := string(code)

	// Verify array fields exist by name
	expectedFieldNames := []string{
		"Unbounded_strings",
		"Fixed_ints",
		"Bounded_floats",
	}

	for _, fieldName := range expectedFieldNames {
		if !strings.Contains(codeStr, fieldName) {
			t.Errorf("Generated code missing expected array field name: %q\nGenerated:\n%s", fieldName, codeStr[:500])
		}
	}

	// Verify array type syntax
	if !strings.Contains(codeStr, "[]string") {
		t.Error("Generated code should contain unbounded string array type")
	}
	if !strings.Contains(codeStr, "[5]int32") {
		t.Error("Generated code should contain fixed int32 array type")
	}
	if !strings.Contains(codeStr, "[]float64") {
		t.Error("Generated code should contain bounded float64 array type")
	}
}

func TestGenerateGoService(t *testing.T) {
	srv := ServiceDefinition{
		Package:  "example_interfaces",
		Name:     "AddTwoInts",
		FullName: "example_interfaces/srv/AddTwoInts",
		TypeHash: "RIHS01_service12345678901234567890123456789012345678901234567890123456",
		Request: MessageDefinition{
			Package:  "example_interfaces",
			Name:     "AddTwoInts_Request",
			FullName: "example_interfaces/srv/AddTwoInts_Request",
			TypeHash: "RIHS01_request12345678901234567890123456789012345678901234567890123456",
			Fields: []FieldDefinition{
				{Name: "a", FieldType: FieldType{Kind: "Int64"}, IsArray: false},
				{Name: "b", FieldType: FieldType{Kind: "Int64"}, IsArray: false},
			},
		},
		Response: MessageDefinition{
			Package:  "example_interfaces",
			Name:     "AddTwoInts_Response",
			FullName: "example_interfaces/srv/AddTwoInts_Response",
			TypeHash: "RIHS01_response1234567890123456789012345678901234567890123456789012",
			Fields: []FieldDefinition{
				{Name: "sum", FieldType: FieldType{Kind: "Int64"}, IsArray: false},
			},
		},
	}

	code, err := GenerateGoService(srv, "test")
	if err != nil {
		t.Fatalf("GenerateGoService failed: %v", err)
	}

	codeStr := string(code)

	// Verify service elements
	expectedElements := []string{
		"package example_interfaces",
		"type AddTwoInts struct{}",
		"AddTwoInts_TypeName = \"example_interfaces::srv::dds_::AddTwoInts_\"",
		"type AddTwoIntsRequest struct",
		"type AddTwoIntsResponse struct",
		"A int64",
		"B int64",
		"Sum int64",
		"func (m *AddTwoIntsRequest) SerializeCDR()",
		"func (m *AddTwoIntsResponse) DeserializeCDR(",
	}

	for _, element := range expectedElements {
		if !strings.Contains(codeStr, element) {
			t.Errorf("Generated service code missing expected element: %q", element)
		}
	}
}

func TestGenerateUnpackField(t *testing.T) {
	// Test that unpack code is generated (not just TODO comments)
	msg := MessageDefinition{
		Package:  "test_msgs",
		Name:     "Simple",
		FullName: "test_msgs/Simple",
		TypeHash: "RIHS01_simple12345678901234567890123456789012345678901234567890123456",
		Fields: []FieldDefinition{
			{Name: "value", FieldType: FieldType{Kind: "Int32"}, IsArray: false},
			{Name: "name", FieldType: FieldType{Kind: "String"}, IsArray: false},
		},
		Constants: []ConstantDefinition{},
	}

	code, err := GenerateGoMessage(msg, "test")
	if err != nil {
		t.Fatalf("GenerateGoMessage failed: %v", err)
	}

	codeStr := string(code)

	// Verify unpack methods are properly generated (not just TODOs)
	if strings.Contains(codeStr, "// TODO: unpack") {
		t.Error("Unpack code should be fully implemented, not contain TODO comments")
	}

	// Check for actual unpack logic
	expectedUnpackElements := []string{
		"binary.LittleEndian.Uint32(data[offset:])",
		"m.Value = int32(",
		"strLen := int(binary.LittleEndian.Uint32(data[offset:]))",
	}

	for _, element := range expectedUnpackElements {
		if !strings.Contains(codeStr, element) {
			t.Errorf("Generated unpack code missing expected element: %q", element)
		}
	}
}
