//! RAM budget of the link profiles (spec 3.3, 3.4, 8.9): each slot holds one
//! 640-byte payload; the accumulator holds 656 bytes.

use zigbee_zrhp::link::Link;

#[test]
fn link_ram_budget() {
    let sizes = [
        ("ncp 16/16", size_of::<Link<16, 16>>(), 16 + 16),
        ("embedded 8/8", size_of::<Link<8, 8>>(), 8 + 8),
        ("minimal 4/4", size_of::<Link<4, 4>>(), 4 + 4),
        ("desktop 32/32", size_of::<Link<32, 32>>(), 32 + 32),
    ];
    for (name, size, slots) in sizes {
        println!("{name}: {size} B");
        // 640 B payload + ≤ 48 B metadata per slot, 656 B accumulator,
        // ≤ 1 KiB of fixed state.
        assert!(size <= slots * (640 + 48) + 656 + 1024, "{name}: {size} B");
    }
}
