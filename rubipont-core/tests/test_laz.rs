use rubipont_core::format;

#[test]
fn laz_detect_extension() {
    assert!(format::laz::detect("laz"));
    assert!(format::laz::detect("LAZ"));
    assert!(!format::laz::detect("las"));
}
