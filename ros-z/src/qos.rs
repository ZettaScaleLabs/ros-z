#[repr(C)]
pub enum QosReliability {
    Reliable,
    BestEffort,
}

#[repr(C)]
pub enum QosHistory {
    KeepLast(usize),
    KeepAll,
}

#[repr(C)]
pub enum QosDurability {
    TransientLocal,
    Volatile,
}

pub struct QosProfile {
    history: QosHistory,
    reliability: QosReliability,
    durability: QosDurability,
}
