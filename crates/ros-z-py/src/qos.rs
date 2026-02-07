use pyo3::prelude::*;
use pyo3::types::PyDict;
use ros_z::qos::{QosProfile, QosReliability, QosDurability, QosHistory, QosLiveliness, Duration};
use std::num::NonZeroUsize;

// ROS 2 QoS presets
pub const QOS_DEFAULT: QosProfile = QosProfile {
    reliability: QosReliability::Reliable,
    durability: QosDurability::Volatile,
    history: QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
    deadline: Duration::INFINITE,
    lifespan: Duration::INFINITE,
    liveliness: QosLiveliness::Automatic,
    liveliness_lease_duration: Duration::INFINITE,
};

pub const QOS_SENSOR_DATA: QosProfile = QosProfile {
    reliability: QosReliability::BestEffort,
    durability: QosDurability::Volatile,
    history: QosHistory::KeepLast(NonZeroUsize::new(5).unwrap()),
    deadline: Duration::INFINITE,
    lifespan: Duration::INFINITE,
    liveliness: QosLiveliness::Automatic,
    liveliness_lease_duration: Duration::INFINITE,
};

pub const QOS_PARAMETERS: QosProfile = QosProfile {
    reliability: QosReliability::Reliable,
    durability: QosDurability::Volatile,
    history: QosHistory::KeepLast(NonZeroUsize::new(1000).unwrap()),
    deadline: Duration::INFINITE,
    lifespan: Duration::INFINITE,
    liveliness: QosLiveliness::Automatic,
    liveliness_lease_duration: Duration::INFINITE,
};

pub const QOS_SERVICES: QosProfile = QosProfile {
    reliability: QosReliability::Reliable,
    durability: QosDurability::Volatile,
    history: QosHistory::KeepLast(NonZeroUsize::new(10).unwrap()),
    deadline: Duration::INFINITE,
    lifespan: Duration::INFINITE,
    liveliness: QosLiveliness::Automatic,
    liveliness_lease_duration: Duration::INFINITE,
};

/// Convert QosProfile to Python dict
pub fn qos_to_pydict(py: Python, qos: &QosProfile) -> PyResult<Py<PyDict>> {
    let dict = PyDict::new_bound(py);

    // Reliability
    let reliability = match qos.reliability {
        QosReliability::Reliable => "reliable",
        QosReliability::BestEffort => "best_effort",
    };
    dict.set_item("reliability", reliability)?;

    // Durability
    let durability = match qos.durability {
        QosDurability::Volatile => "volatile",
        QosDurability::TransientLocal => "transient_local",
    };
    dict.set_item("durability", durability)?;

    // History
    match qos.history {
        QosHistory::KeepLast(depth) => {
            dict.set_item("history", "keep_last")?;
            dict.set_item("depth", depth.get())?;
        }
        QosHistory::KeepAll => {
            dict.set_item("history", "keep_all")?;
        }
    }

    // Liveliness
    let liveliness = match qos.liveliness {
        QosLiveliness::Automatic => "automatic",
        QosLiveliness::ManualByNode => "manual_by_node",
        QosLiveliness::ManualByTopic => "manual_by_topic",
    };
    dict.set_item("liveliness", liveliness)?;

    // Durations (convert to seconds as float)
    if qos.deadline != Duration::INFINITE {
        let secs = qos.deadline.sec as f64 + (qos.deadline.nsec as f64 / 1_000_000_000.0);
        dict.set_item("deadline", secs)?;
    }

    if qos.lifespan != Duration::INFINITE {
        let secs = qos.lifespan.sec as f64 + (qos.lifespan.nsec as f64 / 1_000_000_000.0);
        dict.set_item("lifespan", secs)?;
    }

    if qos.liveliness_lease_duration != Duration::INFINITE {
        let secs = qos.liveliness_lease_duration.sec as f64 + (qos.liveliness_lease_duration.nsec as f64 / 1_000_000_000.0);
        dict.set_item("liveliness_lease_duration", secs)?;
    }

    Ok(dict.unbind())
}

/// Convert Python dict to QosProfile
pub fn qos_from_pydict(dict: &Bound<'_, PyDict>) -> PyResult<QosProfile> {
    let mut qos = QOS_DEFAULT;

    // Reliability
    if let Ok(Some(rel)) = dict.get_item("reliability") {
        let rel_str: String = rel.extract()?;
        qos.reliability = match rel_str.as_str() {
            "reliable" => QosReliability::Reliable,
            "best_effort" => QosReliability::BestEffort,
            _ => return Err(pyo3::exceptions::PyValueError::new_err(
                format!("Invalid reliability: {}", rel_str)
            )),
        };
    }

    // Durability
    if let Ok(Some(dur)) = dict.get_item("durability") {
        let dur_str: String = dur.extract()?;
        qos.durability = match dur_str.as_str() {
            "volatile" => QosDurability::Volatile,
            "transient_local" => QosDurability::TransientLocal,
            _ => return Err(pyo3::exceptions::PyValueError::new_err(
                format!("Invalid durability: {}", dur_str)
            )),
        };
    }

    // History
    if let Ok(Some(hist)) = dict.get_item("history") {
        let hist_str: String = hist.extract()?;
        qos.history = match hist_str.as_str() {
            "keep_last" => {
                let depth: usize = dict.get_item("depth")?
                    .ok_or_else(|| pyo3::exceptions::PyValueError::new_err(
                        "depth required for keep_last history"
                    ))?
                    .extract()?;
                let non_zero_depth = NonZeroUsize::new(depth)
                    .ok_or_else(|| pyo3::exceptions::PyValueError::new_err(
                        "depth must be greater than 0"
                    ))?;
                QosHistory::KeepLast(non_zero_depth)
            }
            "keep_all" => QosHistory::KeepAll,
            _ => return Err(pyo3::exceptions::PyValueError::new_err(
                format!("Invalid history: {}", hist_str)
            )),
        };
    }

    // Liveliness
    if let Ok(Some(live)) = dict.get_item("liveliness") {
        let live_str: String = live.extract()?;
        qos.liveliness = match live_str.as_str() {
            "automatic" => QosLiveliness::Automatic,
            "manual_by_node" => QosLiveliness::ManualByNode,
            "manual_by_topic" => QosLiveliness::ManualByTopic,
            _ => return Err(pyo3::exceptions::PyValueError::new_err(
                format!("Invalid liveliness: {}", live_str)
            )),
        };
    }

    // Deadline (convert from float seconds to Duration)
    if let Ok(Some(deadline)) = dict.get_item("deadline") {
        let secs: f64 = deadline.extract()?;
        qos.deadline = duration_from_secs(secs);
    }

    // Lifespan
    if let Ok(Some(lifespan)) = dict.get_item("lifespan") {
        let secs: f64 = lifespan.extract()?;
        qos.lifespan = duration_from_secs(secs);
    }

    // Liveliness lease duration
    if let Ok(Some(lease)) = dict.get_item("liveliness_lease_duration") {
        let secs: f64 = lease.extract()?;
        qos.liveliness_lease_duration = duration_from_secs(secs);
    }

    Ok(qos)
}

/// Helper function to convert float seconds to Duration
fn duration_from_secs(secs: f64) -> Duration {
    let sec = secs.trunc() as u64;
    let nsec = ((secs.fract() * 1_000_000_000.0).round() as u64).min(999_999_999);
    Duration { sec, nsec }
}
