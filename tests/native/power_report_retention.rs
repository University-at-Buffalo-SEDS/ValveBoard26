use sedsnet::{
    config::{DataEndpoint, DataType, RuntimeMemoryConfig},
    router::{Router, RouterConfig},
};
use std::sync::{Arc, Mutex};
fn main() {
    let memory = RuntimeMemoryConfig::new(8192, 16, 512, 1.0).unwrap();
    let router = Router::new_with_clock(
        RouterConfig::new([])
            .with_sender("VB")
            .with_memory_config(memory)
            .unwrap(),
        Box::new(|| 0),
    );
    let sent = Arc::new(Mutex::new(Vec::new()));
    let observer = sent.clone();
    let side = router.add_side_packed("can", move |wire| {
        let p = sedsnet::wire_format::unpack_packet(wire)?;
        observer
            .lock()
            .unwrap()
            .push((p.data_type(), p.timestamp()));
        Ok(())
    });
    let consumers = [
        DataEndpoint::named("GROUND_STATION"),
        DataEndpoint::named("SD_CARD"),
    ];
    let announce = sedsnet::discovery::build_discovery_announce("consumer", 0, &consumers).unwrap();
    router.rx_from_side(&announce, side).unwrap();
    router.drain_queues_with_timeout(0).unwrap();
    router.clear_queues();
    sent.lock().unwrap().clear();
    let voltage = DataType::named("BATTERY_VOLTAGE");
    let current = DataType::named("BATTERY_CURRENT");
    let bulk = DataType::named("FUEL_TANK_PRESSURE");
    router
        .log_queue_ts(voltage, 1001, &[12.4f32])
        .expect("voltage accepted");
    router
        .log_queue_ts(current, 1002, &[-1.3f32])
        .expect("current accepted");
    let mut accepted = 0;
    let mut refused = 0;
    for stamp in 0..128 {
        if router.log_queue_ts(bulk, 2000 + stamp, &[50f32]).is_ok() {
            accepted += 1
        } else {
            refused += 1
        }
    }
    router.drain_queues_with_timeout(0).unwrap();
    let wire = sent.lock().unwrap();
    let v = wire
        .iter()
        .filter(|(ty, t)| *ty == voltage && *t == 1001)
        .count();
    let c = wire
        .iter()
        .filter(|(ty, t)| *ty == current && *t == 1002)
        .count();
    eprintln!(
        "accepted power samples:2; actual CRC-decoded voltage={v} current={c}; bulk accepted={accepted} refused={refused}; unchanged queue budget8192/start512"
    );
    assert!(
        v >= 1 && c >= 1,
        "accepted low-rate power reports were reclaimed before their first CAN attempt"
    );
}
#[unsafe(no_mangle)]
pub extern "C" fn telemetry_lock() {}
#[unsafe(no_mangle)]
pub extern "C" fn telemetry_unlock() {}
