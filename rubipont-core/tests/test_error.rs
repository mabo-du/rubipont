use rubipont_core::error::RubipontError;

#[test]
fn error_display_unsupported_format() {
    let err = RubipontError::UnsupportedFormat("xyz".into());
    let msg = format!("{}", err);
    assert!(msg.contains("xyz"), "Error should mention format name, got: {}", msg);
}

#[test]
fn error_display_parse_error() {
    let err = RubipontError::ParseError {
        format: "LAS".into(),
        offset: 256,
        detail: "invalid header signature".into(),
    };
    let msg = format!("{}", err);
    assert!(msg.contains("LAS"));
    assert!(msg.contains("256"));
}

#[test]
fn error_is_std_error() {
    use std::error::Error;
    let err = RubipontError::UnsupportedFormat("test".into());
    let _: &dyn Error = &err; // must implement std::error::Error
}
