use rubipont_core::format;

#[test]
fn bag_detect_extension() {
    assert!(format::bag::detect("bag"));
    assert!(format::bag::detect("BAG"));
    assert!(!format::bag::detect("mcap"));
}

#[test]
fn bag_rejects_nonexistent() {
    let result = format::bag::BagReader::new(std::path::Path::new("nonexistent.bag"));
    assert!(result.is_err());
}
