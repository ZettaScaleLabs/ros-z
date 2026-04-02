#[test]
fn message_type_info_rejects_v2_only_shapes() {
    let cases = trybuild::TestCases::new();
    cases.compile_fail("tests/ui/message_type_info/*.rs");
}
