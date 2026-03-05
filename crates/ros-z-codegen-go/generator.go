package main

import (
	"bytes"
	"fmt"
	"go/format"
	"unicode"
)

type CodeBuilder struct {
	buf        bytes.Buffer
	indent     int
	currentPkg string // current Go package being generated
}

func (b *CodeBuilder) P(format string, args ...interface{}) {
	for i := 0; i < b.indent; i++ {
		b.buf.WriteString("\t")
	}
	fmt.Fprintf(&b.buf, format, args...)
	b.buf.WriteString("\n")
}

func (b *CodeBuilder) In()  { b.indent++ }
func (b *CodeBuilder) Out() { b.indent-- }

func (b *CodeBuilder) Bytes() ([]byte, error) {
	return format.Source(b.buf.Bytes())
}

func GenerateGoMessage(msg MessageDefinition, prefix string) ([]byte, error) {
	currentPkg := sanitizePackageName(msg.Package)
	g := &CodeBuilder{currentPkg: currentPkg}

	// Analyze imports needed
	needsMath := false
	needsBinary := false
	crossPkgImports := map[string]bool{}
	for _, field := range msg.Fields {
		if field.FieldType.Kind == "Float32" || field.FieldType.Kind == "Float64" {
			needsMath = true
		}
		if fieldNeedsBinary(field) {
			needsBinary = true
		}
		if field.FieldType.Kind == "Custom" && field.FieldType.Package != nil {
			pkg := sanitizePackageName(*field.FieldType.Package)
			if pkg != currentPkg {
				crossPkgImports[pkg] = true
			}
		}
		if field.FieldType.Kind == "Time" || field.FieldType.Kind == "Duration" {
			if currentPkg != "builtin_interfaces" {
				crossPkgImports["builtin_interfaces"] = true
				needsBinary = true
			}
		}
	}

	// Package declaration
	g.P("package %s", sanitizePackageName(msg.Package))
	g.P("")

	// Imports
	g.P("import (")
	g.In()
	if needsBinary {
		g.P(`"encoding/binary"`)
	}
	g.P(`"fmt"`)
	if needsMath {
		g.P(`"math"`)
	}
	for pkg := range crossPkgImports {
		g.P(`"%s/generated/%s"`, prefix, pkg)
	}
	g.Out()
	g.P(")")
	g.P("")

	// Generate struct
	generateStructInPkg(g, msg, sanitizePackageName(msg.Package))

	// Generate constants
	generateConstants(g, msg)

	// Generate TypeName method
	generateTypeNameMethod(g, msg)

	// Generate TypeHash method
	generateTypeHashMethod(g, msg)

	// Generate SerializeCDR method
	generateSerializeMethod(g, msg)

	// Generate DeserializeCDR method
	generateDeserializeMethod(g, msg)

	return g.Bytes()
}

func generateStruct(g *CodeBuilder, msg MessageDefinition) {
	generateStructInPkg(g, msg, "")
}

func generateStructInPkg(g *CodeBuilder, msg MessageDefinition, currentPkg string) {
	g.P("// %s is a ROS 2 message type", msg.Name)
	g.P("// Full name: %s", msg.FullName)
	g.P("type %s struct {", msg.Name)
	g.In()

	for _, field := range msg.Fields {
		goType := fieldToGoTypeInPkg(field, currentPkg)
		g.P("%s %s", capitalize(field.Name), goType)
	}

	g.Out()
	g.P("}")
	g.P("")
}

func generateConstants(g *CodeBuilder, msg MessageDefinition) {
	// Use DDS-qualified type name so Zenoh key expressions match rmw_zenoh_cpp and ros-z Rust.
	ddsTypeName := fmt.Sprintf("%s::msg::dds_::%s_", msg.Package, msg.Name)
	g.P("const (")
	g.In()
	g.P("%s_TypeName = %q", msg.Name, ddsTypeName)
	g.P("%s_TypeHash = %q", msg.Name, msg.TypeHash)
	g.Out()
	g.P(")")
	g.P("")

	if len(msg.Constants) > 0 {
		g.P("// Message-specific constants")
		g.P("const (")
		g.In()
		for _, c := range msg.Constants {
			g.P("%s_%s = %s", msg.Name, c.Name, c.Value)
		}
		g.Out()
		g.P(")")
		g.P("")
	}
}

func generateTypeNameMethod(g *CodeBuilder, msg MessageDefinition) {
	g.P("// TypeName returns the full ROS 2 type name")
	g.P("func (m *%s) TypeName() string {", msg.Name)
	g.In()
	g.P("return %s_TypeName", msg.Name)
	g.Out()
	g.P("}")
	g.P("")
}

func generateTypeHashMethod(g *CodeBuilder, msg MessageDefinition) {
	g.P("// TypeHash returns the ROS 2 type hash (RIHS01 format)")
	g.P("func (m *%s) TypeHash() string {", msg.Name)
	g.In()
	g.P("return %s_TypeHash", msg.Name)
	g.Out()
	g.P("}")
	g.P("")
}

func generateSerializeMethod(g *CodeBuilder, msg MessageDefinition) {
	g.P("// SerializeCDR serializes the message to CDR format")
	g.P("func (m *%s) SerializeCDR() ([]byte, error) {", msg.Name)
	g.In()
	g.P("raw := m.packToRaw()")
	g.P("buf := make([]byte, 4, 4+len(raw))")
	g.P("buf[0], buf[1], buf[2], buf[3] = 0x00, 0x01, 0x00, 0x00 // CDR_LE encapsulation header")
	g.P("return append(buf, raw...), nil")
	g.Out()
	g.P("}")
	g.P("")

	// Generate pack method
	g.P("func (m *%s) packToRaw() []byte {", msg.Name)
	g.In()
	g.P("buf := make([]byte, 0, 256)")
	for _, field := range msg.Fields {
		generatePackField(g, field)
	}
	g.P("return buf")
	g.Out()
	g.P("}")
	g.P("")
}

func generateDeserializeMethod(g *CodeBuilder, msg MessageDefinition) {
	g.P("// DeserializeCDR deserializes CDR data into the message")
	g.P("func (m *%s) DeserializeCDR(data []byte) error {", msg.Name)
	g.In()
	g.P("if len(data) < 4 {")
	g.In()
	g.P("return fmt.Errorf(\"CDR data too short: need 4-byte encapsulation header\")")
	g.Out()
	g.P("}")
	g.P("return m.unpackFromRaw(data[4:]) // skip CDR encapsulation header")
	g.Out()
	g.P("}")
	g.P("")

	// Generate unpack method
	g.P("func (m *%s) unpackFromRaw(data []byte) error {", msg.Name)
	g.In()
	g.P("offset := 0")
	if len(msg.Fields) == 0 {
		g.P("_ = offset")
	}
	for _, field := range msg.Fields {
		generateUnpackField(g, field)
	}
	g.P("return nil")
	g.Out()
	g.P("}")
	g.P("")
}

// fieldNeedsBinary returns true if the field requires the "encoding/binary" import.
func fieldNeedsBinary(field FieldDefinition) bool {
	switch field.FieldType.Kind {
	case "String":
		return true // string length uses binary
	case "Int16", "UInt16", "Int32", "UInt32", "Int64", "UInt64", "Float32", "Float64":
		return true // multi-byte numeric types
	case "Custom", "Time", "Duration":
		if field.IsArray {
			return true // array length prefix uses binary
		}
		// Same-package custom single value: packToRaw doesn't use binary in the caller
		return false
	case "Bool", "Int8", "UInt8":
		// Dynamic/bounded arrays have a length prefix that uses binary.
		// Fixed arrays use a simple loop without binary.
		return field.IsArray && field.ArrayKind != "fixed"
	default:
		return false
	}
}

func generatePackField(g *CodeBuilder, field FieldDefinition) {
	fieldAccess := fmt.Sprintf("m.%s", capitalize(field.Name))

	switch field.FieldType.Kind {
	case "String":
		if field.IsArray {
			g.P("// Pack string array: %s", field.Name)
			g.P("{")
			g.In()
			g.P("lenBytes := make([]byte, 4)")
			g.P("binary.LittleEndian.PutUint32(lenBytes, uint32(len(%s)))", fieldAccess)
			g.P("buf = append(buf, lenBytes...)")
			g.P("for _, s := range %s {", fieldAccess)
			g.In()
			g.P("data := []byte(s)")
			g.P("binary.LittleEndian.PutUint32(lenBytes, uint32(len(data)+1))")
			g.P("buf = append(buf, lenBytes...)")
			g.P("buf = append(buf, data...)")
			g.P("buf = append(buf, 0) // null terminator")
			g.Out()
			g.P("}")
			g.Out()
			g.P("}")
		} else {
			g.P("// Pack string: %s", field.Name)
			g.P("{")
			g.In()
			g.P("data := []byte(%s)", fieldAccess)
			g.P("lenBytes := make([]byte, 4)")
			g.P("binary.LittleEndian.PutUint32(lenBytes, uint32(len(data)+1))")
			g.P("buf = append(buf, lenBytes...)")
			g.P("buf = append(buf, data...)")
			g.P("buf = append(buf, 0) // null terminator")
			g.Out()
			g.P("}")
		}
	case "Bool", "Int8", "Int16", "Int32", "Int64", "UInt8", "UInt16", "UInt32", "UInt64", "Float32", "Float64":
		g.P("// Pack %s: %s", field.FieldType.Kind, field.Name)
		g.P("{")
		g.In()
		// Dynamic sequences require a 4-byte length prefix in CDR.
		if field.IsArray && field.ArrayKind != "fixed" {
			g.P("lenBytes := make([]byte, 4)")
			g.P("binary.LittleEndian.PutUint32(lenBytes, uint32(len(%s)))", fieldAccess)
			g.P("buf = append(buf, lenBytes...)")
		}
		generatePrimitivePackCode(g, field)
		g.Out()
		g.P("}")
	case "Custom":
		g.P("// Pack custom type: %s", field.Name)
		g.P("{")
		g.In()
		isCrossPkg := field.FieldType.Package != nil && sanitizePackageName(*field.FieldType.Package) != g.currentPkg
		if field.IsArray {
			// Emit length prefix then iterate over elements.
			g.P("lenBytes := make([]byte, 4)")
			g.P("binary.LittleEndian.PutUint32(lenBytes, uint32(len(%s)))", fieldAccess)
			g.P("buf = append(buf, lenBytes...)")
			g.P("for i := range %s {", fieldAccess)
			g.In()
			if isCrossPkg {
				g.P("if raw, err := %s[i].SerializeCDR(); err == nil { buf = append(buf, raw[4:]...) }", fieldAccess)
			} else {
				g.P("buf = append(buf, %s[i].packToRaw()...)", fieldAccess)
			}
			g.Out()
			g.P("}")
		} else if isCrossPkg {
			// Cross-package: use SerializeCDR and strip the 4-byte CDR header.
			g.P("if raw, err := %s.SerializeCDR(); err == nil { buf = append(buf, raw[4:]...) }", fieldAccess)
		} else {
			g.P("buf = append(buf, %s.packToRaw()...)", fieldAccess)
		}
		g.Out()
		g.P("}")
	}
}

func generatePrimitivePackCode(g *CodeBuilder, field FieldDefinition) {
	fieldAccess := fmt.Sprintf("m.%s", capitalize(field.Name))

	switch field.FieldType.Kind {
	case "Bool":
		if field.IsArray {
			g.P("for _, v := range %s {", fieldAccess)
			g.In()
			g.P("if v { buf = append(buf, 1) } else { buf = append(buf, 0) }")
			g.Out()
			g.P("}")
		} else {
			g.P("if %s { buf = append(buf, 1) } else { buf = append(buf, 0) }", fieldAccess)
		}
	case "Int8":
		if field.IsArray {
			// []int8 is not []byte in Go, must convert element-by-element.
			// Fixed arrays need a loop; dynamic arrays also need element conversion.
			g.P("for _, v := range %s { buf = append(buf, byte(v)) }", fieldAccess)
		} else {
			g.P("buf = append(buf, byte(%s))", fieldAccess)
		}
	case "UInt8":
		if field.IsArray {
			if field.ArrayKind == "fixed" {
				// [N]uint8 (fixed array) needs [:] to convert to slice.
				g.P("buf = append(buf, %s[:]...)", fieldAccess)
			} else {
				// []uint8 == []byte, safe direct append.
				g.P("buf = append(buf, %s...)", fieldAccess)
			}
		} else {
			g.P("buf = append(buf, byte(%s))", fieldAccess)
		}
	case "Int16":
		if field.IsArray {
			g.P("for _, v := range %s {", fieldAccess)
			g.In()
			g.P("b := make([]byte, 2)")
			g.P("binary.LittleEndian.PutUint16(b, uint16(v))")
			g.P("buf = append(buf, b...)")
			g.Out()
			g.P("}")
		} else {
			g.P("b := make([]byte, 2)")
			g.P("binary.LittleEndian.PutUint16(b, uint16(%s))", fieldAccess)
			g.P("buf = append(buf, b...)")
		}
	case "UInt16":
		if field.IsArray {
			g.P("for _, v := range %s {", fieldAccess)
			g.In()
			g.P("b := make([]byte, 2)")
			g.P("binary.LittleEndian.PutUint16(b, v)")
			g.P("buf = append(buf, b...)")
			g.Out()
			g.P("}")
		} else {
			g.P("b := make([]byte, 2)")
			g.P("binary.LittleEndian.PutUint16(b, %s)", fieldAccess)
			g.P("buf = append(buf, b...)")
		}
	case "Int32":
		if field.IsArray {
			g.P("for _, v := range %s {", fieldAccess)
			g.In()
			g.P("b := make([]byte, 4)")
			g.P("binary.LittleEndian.PutUint32(b, uint32(v))")
			g.P("buf = append(buf, b...)")
			g.Out()
			g.P("}")
		} else {
			g.P("b := make([]byte, 4)")
			g.P("binary.LittleEndian.PutUint32(b, uint32(%s))", fieldAccess)
			g.P("buf = append(buf, b...)")
		}
	case "UInt32":
		if field.IsArray {
			g.P("for _, v := range %s {", fieldAccess)
			g.In()
			g.P("b := make([]byte, 4)")
			g.P("binary.LittleEndian.PutUint32(b, v)")
			g.P("buf = append(buf, b...)")
			g.Out()
			g.P("}")
		} else {
			g.P("b := make([]byte, 4)")
			g.P("binary.LittleEndian.PutUint32(b, %s)", fieldAccess)
			g.P("buf = append(buf, b...)")
		}
	case "Int64":
		if field.IsArray {
			g.P("for _, v := range %s {", fieldAccess)
			g.In()
			g.P("b := make([]byte, 8)")
			g.P("binary.LittleEndian.PutUint64(b, uint64(v))")
			g.P("buf = append(buf, b...)")
			g.Out()
			g.P("}")
		} else {
			g.P("b := make([]byte, 8)")
			g.P("binary.LittleEndian.PutUint64(b, uint64(%s))", fieldAccess)
			g.P("buf = append(buf, b...)")
		}
	case "UInt64":
		if field.IsArray {
			g.P("for _, v := range %s {", fieldAccess)
			g.In()
			g.P("b := make([]byte, 8)")
			g.P("binary.LittleEndian.PutUint64(b, v)")
			g.P("buf = append(buf, b...)")
			g.Out()
			g.P("}")
		} else {
			g.P("b := make([]byte, 8)")
			g.P("binary.LittleEndian.PutUint64(b, %s)", fieldAccess)
			g.P("buf = append(buf, b...)")
		}
	case "Float32":
		if field.IsArray {
			g.P("for _, v := range %s {", fieldAccess)
			g.In()
			g.P("b := make([]byte, 4)")
			g.P("binary.LittleEndian.PutUint32(b, math.Float32bits(v))")
			g.P("buf = append(buf, b...)")
			g.Out()
			g.P("}")
		} else {
			g.P("b := make([]byte, 4)")
			g.P("binary.LittleEndian.PutUint32(b, math.Float32bits(%s))", fieldAccess)
			g.P("buf = append(buf, b...)")
		}
	case "Float64":
		if field.IsArray {
			g.P("for _, v := range %s {", fieldAccess)
			g.In()
			g.P("b := make([]byte, 8)")
			g.P("binary.LittleEndian.PutUint64(b, math.Float64bits(v))")
			g.P("buf = append(buf, b...)")
			g.Out()
			g.P("}")
		} else {
			g.P("b := make([]byte, 8)")
			g.P("binary.LittleEndian.PutUint64(b, math.Float64bits(%s))", fieldAccess)
			g.P("buf = append(buf, b...)")
		}
	}
}

func generateUnpackField(g *CodeBuilder, field FieldDefinition) {
	fieldAccess := fmt.Sprintf("m.%s", capitalize(field.Name))

	switch field.FieldType.Kind {
	case "String":
		if field.IsArray {
			g.P("// Unpack string array: %s", field.Name)
			g.P("{")
			g.In()
			g.P("if offset+4 > len(data) { return fmt.Errorf(\"buffer too short for string array length\") }")
			g.P("arrLen := int(binary.LittleEndian.Uint32(data[offset:]))")
			g.P("offset += 4")
			g.P("%s = make([]string, arrLen)", fieldAccess)
			g.P("for i := 0; i < arrLen; i++ {")
			g.In()
			g.P("if offset+4 > len(data) { return fmt.Errorf(\"buffer too short for string length\") }")
			g.P("strLen := int(binary.LittleEndian.Uint32(data[offset:]))")
			g.P("offset += 4")
			g.P("if strLen > 0 {")
			g.In()
			g.P("if offset+strLen > len(data) { return fmt.Errorf(\"buffer too short for string data\") }")
			g.P("%s[i] = string(data[offset:offset+strLen-1]) // exclude null terminator", fieldAccess)
			g.P("offset += strLen")
			g.Out()
			g.P("}")
			g.Out()
			g.P("}")
			g.Out()
			g.P("}")
		} else {
			g.P("// Unpack string: %s", field.Name)
			g.P("{")
			g.In()
			g.P("if offset+4 > len(data) { return fmt.Errorf(\"buffer too short for string length\") }")
			g.P("strLen := int(binary.LittleEndian.Uint32(data[offset:]))")
			g.P("offset += 4")
			g.P("if strLen > 0 {")
			g.In()
			g.P("if offset+strLen > len(data) { return fmt.Errorf(\"buffer too short for string data\") }")
			g.P("%s = string(data[offset:offset+strLen-1]) // exclude null terminator", fieldAccess)
			g.P("offset += strLen")
			g.Out()
			g.P("}")
			g.Out()
			g.P("}")
		}
	case "Bool", "Int8", "Int16", "Int32", "Int64", "UInt8", "UInt16", "UInt32", "UInt64", "Float32", "Float64":
		g.P("// Unpack %s: %s", field.FieldType.Kind, field.Name)
		g.P("{")
		g.In()
		generatePrimitiveUnpackCode(g, field)
		g.Out()
		g.P("}")
	case "Custom":
		g.P("// Unpack custom type: %s", field.Name)
		g.P("{")
		g.In()
		isCrossPkg := field.FieldType.Package != nil && sanitizePackageName(*field.FieldType.Package) != g.currentPkg
		if field.IsArray {
			g.P("if offset+4 > len(data) { return fmt.Errorf(\"buffer too short for array length\") }")
			g.P("arrLen := int(binary.LittleEndian.Uint32(data[offset:]))")
			g.P("offset += 4")
			g.P("%s = make([]%s, arrLen)", fieldAccess, fieldToGoBaseTypeInPkg(field, g.currentPkg))
			g.P("for i := 0; i < arrLen; i++ {")
			g.In()
			if isCrossPkg {
				// Cross-package: prepend fake CDR header, deserialize, then re-serialize to measure size.
				g.P("tmp := append([]byte{0, 1, 0, 0}, data[offset:]...)")
				g.P("if err := %s[i].DeserializeCDR(tmp); err != nil { return err }", fieldAccess)
				g.P("if repack, err := %s[i].SerializeCDR(); err == nil { offset += len(repack) - 4 } else { return err }", fieldAccess)
			} else {
				g.P("if err := %s[i].unpackFromRaw(data[offset:]); err != nil { return err }", fieldAccess)
				g.P("// Note: offset advancement requires knowing nested type size")
			}
			g.Out()
			g.P("}")
		} else if isCrossPkg {
			// Cross-package: prepend fake CDR header, deserialize, then re-serialize to measure size.
			g.P("tmp := append([]byte{0, 1, 0, 0}, data[offset:]...)")
			g.P("if err := %s.DeserializeCDR(tmp); err != nil { return err }", fieldAccess)
			g.P("if repack, err := %s.SerializeCDR(); err == nil { offset += len(repack) - 4 } else { return err }", fieldAccess)
		} else {
			g.P("if err := %s.unpackFromRaw(data[offset:]); err != nil { return err }", fieldAccess)
		}
		g.Out()
		g.P("}")
	case "Time":
		g.P("// Unpack Time: %s", field.Name)
		g.P("{")
		g.In()
		g.P("if offset+8 > len(data) { return fmt.Errorf(\"buffer too short for Time\") }")
		g.P("%s.Sec = int32(binary.LittleEndian.Uint32(data[offset:]))", fieldAccess)
		g.P("offset += 4")
		g.P("%s.Nsec = binary.LittleEndian.Uint32(data[offset:])", fieldAccess)
		g.P("offset += 4")
		g.Out()
		g.P("}")
	case "Duration":
		g.P("// Unpack Duration: %s", field.Name)
		g.P("{")
		g.In()
		g.P("if offset+8 > len(data) { return fmt.Errorf(\"buffer too short for Duration\") }")
		g.P("%s.Sec = int32(binary.LittleEndian.Uint32(data[offset:]))", fieldAccess)
		g.P("offset += 4")
		g.P("%s.Nsec = binary.LittleEndian.Uint32(data[offset:])", fieldAccess)
		g.P("offset += 4")
		g.Out()
		g.P("}")
	}
}

func generatePrimitiveUnpackCode(g *CodeBuilder, field FieldDefinition) {
	fieldAccess := fmt.Sprintf("m.%s", capitalize(field.Name))

	switch field.FieldType.Kind {
	case "Bool":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "bool", 1, func() {
				g.P("%s[i] = data[offset] != 0", fieldAccess)
				g.P("offset++")
			})
		} else {
			g.P("if offset >= len(data) { return fmt.Errorf(\"buffer too short for bool\") }")
			g.P("%s = data[offset] != 0", fieldAccess)
			g.P("offset++")
		}
	case "Int8":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "int8", 1, func() {
				g.P("%s[i] = int8(data[offset])", fieldAccess)
				g.P("offset++")
			})
		} else {
			g.P("if offset >= len(data) { return fmt.Errorf(\"buffer too short for int8\") }")
			g.P("%s = int8(data[offset])", fieldAccess)
			g.P("offset++")
		}
	case "UInt8":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "uint8", 1, func() {
				g.P("%s[i] = data[offset]", fieldAccess)
				g.P("offset++")
			})
		} else {
			g.P("if offset >= len(data) { return fmt.Errorf(\"buffer too short for uint8\") }")
			g.P("%s = data[offset]", fieldAccess)
			g.P("offset++")
		}
	case "Int16":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "int16", 2, func() {
				g.P("%s[i] = int16(binary.LittleEndian.Uint16(data[offset:]))", fieldAccess)
				g.P("offset += 2")
			})
		} else {
			g.P("if offset+2 > len(data) { return fmt.Errorf(\"buffer too short for int16\") }")
			g.P("%s = int16(binary.LittleEndian.Uint16(data[offset:]))", fieldAccess)
			g.P("offset += 2")
		}
	case "UInt16":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "uint16", 2, func() {
				g.P("%s[i] = binary.LittleEndian.Uint16(data[offset:])", fieldAccess)
				g.P("offset += 2")
			})
		} else {
			g.P("if offset+2 > len(data) { return fmt.Errorf(\"buffer too short for uint16\") }")
			g.P("%s = binary.LittleEndian.Uint16(data[offset:])", fieldAccess)
			g.P("offset += 2")
		}
	case "Int32":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "int32", 4, func() {
				g.P("%s[i] = int32(binary.LittleEndian.Uint32(data[offset:]))", fieldAccess)
				g.P("offset += 4")
			})
		} else {
			g.P("if offset+4 > len(data) { return fmt.Errorf(\"buffer too short for int32\") }")
			g.P("%s = int32(binary.LittleEndian.Uint32(data[offset:]))", fieldAccess)
			g.P("offset += 4")
		}
	case "UInt32":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "uint32", 4, func() {
				g.P("%s[i] = binary.LittleEndian.Uint32(data[offset:])", fieldAccess)
				g.P("offset += 4")
			})
		} else {
			g.P("if offset+4 > len(data) { return fmt.Errorf(\"buffer too short for uint32\") }")
			g.P("%s = binary.LittleEndian.Uint32(data[offset:])", fieldAccess)
			g.P("offset += 4")
		}
	case "Int64":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "int64", 8, func() {
				g.P("%s[i] = int64(binary.LittleEndian.Uint64(data[offset:]))", fieldAccess)
				g.P("offset += 8")
			})
		} else {
			g.P("if offset+8 > len(data) { return fmt.Errorf(\"buffer too short for int64\") }")
			g.P("%s = int64(binary.LittleEndian.Uint64(data[offset:]))", fieldAccess)
			g.P("offset += 8")
		}
	case "UInt64":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "uint64", 8, func() {
				g.P("%s[i] = binary.LittleEndian.Uint64(data[offset:])", fieldAccess)
				g.P("offset += 8")
			})
		} else {
			g.P("if offset+8 > len(data) { return fmt.Errorf(\"buffer too short for uint64\") }")
			g.P("%s = binary.LittleEndian.Uint64(data[offset:])", fieldAccess)
			g.P("offset += 8")
		}
	case "Float32":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "float32", 4, func() {
				g.P("%s[i] = math.Float32frombits(binary.LittleEndian.Uint32(data[offset:]))", fieldAccess)
				g.P("offset += 4")
			})
		} else {
			g.P("if offset+4 > len(data) { return fmt.Errorf(\"buffer too short for float32\") }")
			g.P("%s = math.Float32frombits(binary.LittleEndian.Uint32(data[offset:]))", fieldAccess)
			g.P("offset += 4")
		}
	case "Float64":
		if field.IsArray {
			generateArrayUnpack(g, field, fieldAccess, "float64", 8, func() {
				g.P("%s[i] = math.Float64frombits(binary.LittleEndian.Uint64(data[offset:]))", fieldAccess)
				g.P("offset += 8")
			})
		} else {
			g.P("if offset+8 > len(data) { return fmt.Errorf(\"buffer too short for float64\") }")
			g.P("%s = math.Float64frombits(binary.LittleEndian.Uint64(data[offset:]))", fieldAccess)
			g.P("offset += 8")
		}
	}
}

func generateArrayUnpack(g *CodeBuilder, field FieldDefinition, fieldAccess, elemType string, elemSize int, unpackElem func()) {
	switch field.ArrayKind {
	case "fixed":
		g.P("if offset+%d*%d > len(data) { return fmt.Errorf(\"buffer too short for fixed array\") }", *field.ArraySize, elemSize)
		g.P("for i := 0; i < %d; i++ {", *field.ArraySize)
		g.In()
		unpackElem()
		g.Out()
		g.P("}")
	default: // unbounded or bounded
		g.P("if offset+4 > len(data) { return fmt.Errorf(\"buffer too short for array length\") }")
		g.P("arrLen := int(binary.LittleEndian.Uint32(data[offset:]))")
		g.P("offset += 4")
		g.P("%s = make([]%s, arrLen)", fieldAccess, elemType)
		g.P("for i := 0; i < arrLen; i++ {")
		g.In()
		unpackElem()
		g.Out()
		g.P("}")
	}
}

func fieldToGoBaseType(field FieldDefinition) string {
	return fieldToGoBaseTypeInPkg(field, "")
}

func fieldToGoBaseTypeInPkg(field FieldDefinition, currentPkg string) string {
	switch field.FieldType.Kind {
	case "Bool":
		return "bool"
	case "Int8":
		return "int8"
	case "Int16":
		return "int16"
	case "Int32":
		return "int32"
	case "Int64":
		return "int64"
	case "UInt8":
		return "uint8"
	case "UInt16":
		return "uint16"
	case "UInt32":
		return "uint32"
	case "UInt64":
		return "uint64"
	case "Float32":
		return "float32"
	case "Float64":
		return "float64"
	case "String":
		return "string"
	case "Custom":
		pkg := sanitizePackageName(*field.FieldType.Package)
		if pkg == currentPkg {
			return *field.FieldType.Name
		}
		return fmt.Sprintf("%s.%s", pkg, *field.FieldType.Name)
	default:
		return "interface{}"
	}
}

func GenerateGoService(srv ServiceDefinition, prefix string) ([]byte, error) {
	pkgName := sanitizePackageName(srv.Package)
	g := &CodeBuilder{currentPkg: pkgName}
	g.P("package %s", pkgName)
	g.P("")

	// Analyze imports needed
	needsMath := false
	needsBinary := false
	crossPkgImports := map[string]bool{}
	allFields := append(srv.Request.Fields, srv.Response.Fields...)
	for _, field := range allFields {
		if field.FieldType.Kind == "Float32" || field.FieldType.Kind == "Float64" {
			needsMath = true
		}
		if fieldNeedsBinary(field) {
			needsBinary = true
		}
		if field.FieldType.Kind == "Custom" && field.FieldType.Package != nil {
			pkg := sanitizePackageName(*field.FieldType.Package)
			if pkg != pkgName {
				crossPkgImports[pkg] = true
			}
		}
		if field.FieldType.Kind == "Time" || field.FieldType.Kind == "Duration" {
			if pkgName != "builtin_interfaces" {
				crossPkgImports["builtin_interfaces"] = true
				needsBinary = true
			}
		}
	}

	// Imports
	g.P("import (")
	g.In()
	if needsBinary {
		g.P(`"encoding/binary"`)
	}
	g.P(`"fmt"`)
	if needsMath {
		g.P(`"math"`)
	}
	g.P(`"%s/rosz"`, prefix)
	for pkg := range crossPkgImports {
		g.P(`"%s/generated/%s"`, prefix, pkg)
	}
	g.Out()
	g.P(")")
	g.P("")

	g.P("// %s is a ROS 2 service type", srv.Name)
	g.P("// Full name: %s", srv.FullName)
	g.P("")

	// Use DDS-qualified service type name so Zenoh key expressions match rmw_zenoh_cpp and ros-z Rust.
	ddsSvcTypeName := fmt.Sprintf("%s::srv::dds_::%s_", srv.Package, srv.Name)
	g.P("const (")
	g.In()
	g.P("%s_TypeName = %q", srv.Name, ddsSvcTypeName)
	g.P("%s_TypeHash = %q", srv.Name, srv.TypeHash)
	g.Out()
	g.P(")")
	g.P("")

	reqName := srv.Name + "Request"
	respName := srv.Name + "Response"

	// Service marker type implementing the rosz.Service interface
	g.P("// %s represents the service type marker", srv.Name)
	g.P("type %s struct{}", srv.Name)
	g.P("")

	g.P("func (s *%s) TypeName() string { return %s_TypeName }", srv.Name, srv.Name)
	g.P("func (s *%s) TypeHash() string { return %s_TypeHash }", srv.Name, srv.Name)
	g.P("func (s *%s) SerializeCDR() ([]byte, error) { return nil, nil }", srv.Name)
	g.P("func (s *%s) DeserializeCDR(_ []byte) error { return nil }", srv.Name)
	g.P("func (s *%s) GetRequest() rosz.Message { return &%s{} }", srv.Name, reqName)
	g.P("func (s *%s) GetResponse() rosz.Message { return &%s{} }", srv.Name, respName)
	g.P("")

	// Generate Request type
	generateServiceMessage(g, reqName, srv.Request, pkgName)

	// Generate Response type
	generateServiceMessage(g, respName, srv.Response, pkgName)

	return g.Bytes()
}

func generateServiceMessage(g *CodeBuilder, name string, msg MessageDefinition, currentPkg string) {
	// Generate struct
	g.P("// %s is the request/response message for the service", name)
	g.P("type %s struct {", name)
	g.In()
	for _, field := range msg.Fields {
		goType := fieldToGoTypeInPkg(field, currentPkg)
		g.P("%s %s", capitalize(field.Name), goType)
	}
	g.Out()
	g.P("}")
	g.P("")

	// Generate constants
	g.P("const (")
	g.In()
	g.P("%s_TypeName = %q", name, msg.FullName)
	g.P("%s_TypeHash = %q", name, msg.TypeHash)
	g.Out()
	g.P(")")
	g.P("")

	// Generate TypeName method
	g.P("func (m *%s) TypeName() string { return %s_TypeName }", name, name)
	g.P("func (m *%s) TypeHash() string { return %s_TypeHash }", name, name)
	g.P("")

	// Generate SerializeCDR method
	g.P("// SerializeCDR serializes the message to CDR format")
	g.P("func (m *%s) SerializeCDR() ([]byte, error) {", name)
	g.In()
	g.P("raw := m.packToRaw()")
	g.P("buf := make([]byte, 4, 4+len(raw))")
	g.P("buf[0], buf[1], buf[2], buf[3] = 0x00, 0x01, 0x00, 0x00 // CDR_LE encapsulation header")
	g.P("return append(buf, raw...), nil")
	g.Out()
	g.P("}")
	g.P("")

	// Generate pack method
	g.P("func (m *%s) packToRaw() []byte {", name)
	g.In()
	g.P("buf := make([]byte, 0, 256)")
	for _, field := range msg.Fields {
		generatePackField(g, field)
	}
	g.P("return buf")
	g.Out()
	g.P("}")
	g.P("")

	// Generate DeserializeCDR method
	g.P("// DeserializeCDR deserializes CDR data into the message")
	g.P("func (m *%s) DeserializeCDR(data []byte) error {", name)
	g.In()
	g.P("if len(data) < 4 {")
	g.In()
	g.P("return fmt.Errorf(\"CDR data too short: need 4-byte encapsulation header\")")
	g.Out()
	g.P("}")
	g.P("return m.unpackFromRaw(data[4:]) // skip CDR encapsulation header")
	g.Out()
	g.P("}")
	g.P("")

	// Generate unpack method
	g.P("func (m *%s) unpackFromRaw(data []byte) error {", name)
	g.In()
	g.P("offset := 0")
	if len(msg.Fields) == 0 {
		g.P("_ = offset")
	}
	for _, field := range msg.Fields {
		generateUnpackField(g, field)
	}
	g.P("return nil")
	g.Out()
	g.P("}")
	g.P("")
}

func fieldToGoType(field FieldDefinition) string {
	return fieldToGoTypeInPkg(field, "")
}

func fieldToGoTypeInPkg(field FieldDefinition, currentPkg string) string {
	var baseType string

	switch field.FieldType.Kind {
	case "Bool":
		baseType = "bool"
	case "Int8":
		baseType = "int8"
	case "Int16":
		baseType = "int16"
	case "Int32":
		baseType = "int32"
	case "Int64":
		baseType = "int64"
	case "UInt8":
		baseType = "uint8"
	case "UInt16":
		baseType = "uint16"
	case "UInt32":
		baseType = "uint32"
	case "UInt64":
		baseType = "uint64"
	case "Float32":
		baseType = "float32"
	case "Float64":
		baseType = "float64"
	case "String":
		baseType = "string"
	case "Time":
		if currentPkg == "builtin_interfaces" {
			baseType = "Time"
		} else {
			baseType = "builtin_interfaces.Time"
		}
	case "Duration":
		if currentPkg == "builtin_interfaces" {
			baseType = "Duration"
		} else {
			baseType = "builtin_interfaces.Duration"
		}
	case "Custom":
		pkg := sanitizePackageName(*field.FieldType.Package)
		if pkg == currentPkg {
			baseType = *field.FieldType.Name
		} else {
			baseType = fmt.Sprintf("%s.%s", pkg, *field.FieldType.Name)
		}
	default:
		baseType = "interface{}"
	}

	if field.IsArray {
		switch field.ArrayKind {
		case "fixed":
			return fmt.Sprintf("[%d]%s", *field.ArraySize, baseType)
		default:
			return "[]" + baseType
		}
	}

	return baseType
}

// GenerateGoAction generates Go code for a ROS 2 action type.
// It produces the action marker type, Goal/Result/Feedback sub-types with CDR serialization,
// and implements ActionSubServiceHashes for interop with rmw_zenoh_cpp.
func GenerateGoAction(action ActionDefinition, prefix string) ([]byte, error) {
	pkgName := sanitizePackageName(action.Package)
	g := &CodeBuilder{currentPkg: pkgName}
	g.P("package %s", pkgName)
	g.P("")

	// Collect imports needed by sub-messages
	needsMath := false
	needsBinary := false
	crossPkgImports := map[string]bool{}
	var allSubFields []FieldDefinition
	allSubFields = append(allSubFields, action.Goal.Fields...)
	if action.Result != nil {
		allSubFields = append(allSubFields, action.Result.Fields...)
	}
	if action.Feedback != nil {
		allSubFields = append(allSubFields, action.Feedback.Fields...)
	}
	for _, field := range allSubFields {
		if field.FieldType.Kind == "Float32" || field.FieldType.Kind == "Float64" {
			needsMath = true
		}
		if fieldNeedsBinary(field) {
			needsBinary = true
		}
		if field.FieldType.Kind == "Custom" && field.FieldType.Package != nil {
			pkg := sanitizePackageName(*field.FieldType.Package)
			if pkg != pkgName {
				crossPkgImports[pkg] = true
			}
		}
		if field.FieldType.Kind == "Time" || field.FieldType.Kind == "Duration" {
			if pkgName != "builtin_interfaces" {
				crossPkgImports["builtin_interfaces"] = true
				needsBinary = true
			}
		}
	}

	g.P("import (")
	g.In()
	if needsBinary {
		g.P(`"encoding/binary"`)
	}
	g.P(`"fmt"`)
	if needsMath {
		g.P(`"math"`)
	}
	g.P(`"%s/rosz"`, prefix)
	for pkg := range crossPkgImports {
		g.P(`"%s/generated/%s"`, prefix, pkg)
	}
	g.Out()
	g.P(")")
	g.P("")

	g.P("// %s is a ROS 2 action type", action.Name)
	g.P("// Full name: %s", action.FullName)
	g.P("")

	// Action-level constants: TypeName uses the ros-z internal format (not DDS).
	// Sub-service compound hashes are the DDS RIHS01 hashes used by rmw_zenoh_cpp.
	g.P("const (")
	g.In()
	g.P("%s_TypeName = %q", action.Name, action.FullName)
	g.P("%s_TypeHash = %q", action.Name, action.TypeHash)
	g.P("%s_SendGoalHash = %q", action.Name, action.SendGoalHash)
	g.P("%s_GetResultHash = %q", action.Name, action.GetResultHash)
	g.P("%s_CancelGoalHash = %q", action.Name, action.CancelGoalHash)
	g.P("%s_FeedbackMessageHash = %q", action.Name, action.FeedbackMessageHash)
	g.P("%s_StatusHash = %q", action.Name, action.StatusHash)
	g.Out()
	g.P(")")
	g.P("")

	// Action marker type
	g.P("// %s is the action type marker", action.Name)
	g.P("type %s struct{}", action.Name)
	g.P("")

	// Action interface methods
	goalName := action.Name + "Goal"
	resultName := action.Name + "Result"
	feedbackName := action.Name + "Feedback"

	g.P("func (a *%s) TypeName() string              { return %s_TypeName }", action.Name, action.Name)
	g.P("func (a *%s) TypeHash() string              { return %s_TypeHash }", action.Name, action.Name)
	g.P("func (a *%s) SerializeCDR() ([]byte, error) { return nil, nil }", action.Name)
	g.P("func (a *%s) DeserializeCDR(_ []byte) error { return nil }", action.Name)
	g.P("func (a *%s) GetGoal() rosz.Message         { return &%s{} }", action.Name, goalName)
	if action.Result != nil {
		g.P("func (a *%s) GetResult() rosz.Message     { return &%s{} }", action.Name, resultName)
	} else {
		g.P("func (a *%s) GetResult() rosz.Message     { return nil }", action.Name)
	}
	if action.Feedback != nil {
		g.P("func (a *%s) GetFeedback() rosz.Message   { return &%s{} }", action.Name, feedbackName)
	} else {
		g.P("func (a *%s) GetFeedback() rosz.Message   { return nil }", action.Name)
	}
	g.P("")

	// ActionSubServiceHashes interface — provides compound hashes for rmw_zenoh_cpp interop
	g.P("// SendGoalHash returns the compound RIHS01 hash for the SendGoal sub-service.")
	g.P("func (a *%s) SendGoalHash() string        { return %s_SendGoalHash }", action.Name, action.Name)
	g.P("// GetResultHash returns the compound RIHS01 hash for the GetResult sub-service.")
	g.P("func (a *%s) GetResultHash() string       { return %s_GetResultHash }", action.Name, action.Name)
	g.P("// CancelGoalHash returns the compound RIHS01 hash for the CancelGoal sub-service.")
	g.P("func (a *%s) CancelGoalHash() string      { return %s_CancelGoalHash }", action.Name, action.Name)
	g.P("// FeedbackMessageHash returns the compound RIHS01 hash for the FeedbackMessage topic.")
	g.P("func (a *%s) FeedbackMessageHash() string { return %s_FeedbackMessageHash }", action.Name, action.Name)
	g.P("// StatusHash returns the compound RIHS01 hash for the GoalStatusArray topic.")
	g.P("func (a *%s) StatusHash() string          { return %s_StatusHash }", action.Name, action.Name)
	g.P("")

	// Generate Goal sub-type
	generateActionSubMessage(g, goalName, action.Goal, pkgName)

	// Generate Result sub-type (if present)
	if action.Result != nil {
		generateActionSubMessage(g, resultName, *action.Result, pkgName)
	}

	// Generate Feedback sub-type (if present)
	if action.Feedback != nil {
		generateActionSubMessage(g, feedbackName, *action.Feedback, pkgName)
	}

	return g.Bytes()
}

// generateActionSubMessage generates a Goal/Result/Feedback sub-message type for an action.
func generateActionSubMessage(g *CodeBuilder, name string, msg MessageDefinition, currentPkg string) {
	comment := "goal"
	switch {
	case len(name) > 6 && name[len(name)-6:] == "Result":
		comment = "result"
	case len(name) > 8 && name[len(name)-8:] == "Feedback":
		comment = "feedback"
	}
	g.P("// %s is the %s message for the action", name, comment)
	g.P("type %s struct {", name)
	g.In()
	for _, field := range msg.Fields {
		goType := fieldToGoTypeInPkg(field, currentPkg)
		g.P("%s %s", capitalize(field.Name), goType)
	}
	g.Out()
	g.P("}")
	g.P("")

	// Constants (use the message-level full_name and type_hash)
	g.P("const (")
	g.In()
	g.P("%s_TypeName = %q", name, msg.FullName)
	g.P("%s_TypeHash = %q", name, msg.TypeHash)
	g.Out()
	g.P(")")
	g.P("")

	g.P("func (m *%s) TypeName() string { return %s_TypeName }", name, name)
	g.P("func (m *%s) TypeHash() string { return %s_TypeHash }", name, name)
	g.P("")

	// SerializeCDR
	g.P("// SerializeCDR serializes the message to CDR format")
	g.P("func (m *%s) SerializeCDR() ([]byte, error) {", name)
	g.In()
	g.P("raw := m.packToRaw()")
	g.P("buf := make([]byte, 4, 4+len(raw))")
	g.P("buf[0], buf[1], buf[2], buf[3] = 0x00, 0x01, 0x00, 0x00 // CDR_LE encapsulation header")
	g.P("return append(buf, raw...), nil")
	g.Out()
	g.P("}")
	g.P("")

	// packToRaw
	g.P("func (m *%s) packToRaw() []byte {", name)
	g.In()
	g.P("buf := make([]byte, 0, 256)")
	for _, field := range msg.Fields {
		generatePackField(g, field)
	}
	g.P("return buf")
	g.Out()
	g.P("}")
	g.P("")

	// DeserializeCDR
	g.P("// DeserializeCDR deserializes CDR data into the message")
	g.P("func (m *%s) DeserializeCDR(data []byte) error {", name)
	g.In()
	g.P("if len(data) < 4 {")
	g.In()
	g.P("return fmt.Errorf(\"CDR data too short: need 4-byte encapsulation header\")")
	g.Out()
	g.P("}")
	g.P("return m.unpackFromRaw(data[4:]) // skip CDR encapsulation header")
	g.Out()
	g.P("}")
	g.P("")

	// unpackFromRaw
	g.P("func (m *%s) unpackFromRaw(data []byte) error {", name)
	g.In()
	g.P("offset := 0")
	if len(msg.Fields) == 0 {
		g.P("_ = offset")
	}
	for _, field := range msg.Fields {
		generateUnpackField(g, field)
	}
	g.P("return nil")
	g.Out()
	g.P("}")
	g.P("")
}

func capitalize(s string) string {
	if len(s) == 0 {
		return s
	}
	runes := []rune(s)
	runes[0] = unicode.ToUpper(runes[0])
	return string(runes)
}
