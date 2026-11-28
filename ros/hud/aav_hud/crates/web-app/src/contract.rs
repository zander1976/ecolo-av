//! The ROS tab: the running graph against `docs/interfaces.yaml`, as
//! `tools/check_graph.py` judges it (the same profiles, the same "must be live" rule).
//! No ROS here: the graph comes in as a list of [`Seen`] topics.
//!
//! | Verdict | When |
//! |---|---|
//! | LIVE | in the contract, a publisher, the contract's type |
//! | MISSING | the profile needs it live, and it has no publisher |
//! | WRONG TYPE | on the graph with a type the contract doesn't give |
//! | QUIET | in the contract, not needed in this profile, no publisher |
//! | NOT IN CONTRACT | on the graph with a publisher, but the contract doesn't list it |

use std::collections::{BTreeMap, HashMap};

use anyhow::{bail, Context as _, Result};
use serde::Serialize;
use serde_yaml_ng::Value;

/// The profiles, as `tools/check_graph.py`'s `PROFILES`.
pub const PROFILES: [&str; 8] = [
    "turtle", "sim", "road", "spine", "car", "virtual", "replay", "bench",
];

/// Never judged: every node has them (check_graph.py's `IGNORE`, the topics).
const IGNORE: [&str; 3] = ["/parameter_events", "/rosout", "/diagnostics"];

/// Never "not in the contract" either: costmap internals and turtlesim's own
/// (check_graph.py's `internal`), and the topics ROS makes inside every action.
const INTERNAL: [&str; 3] = ["/local_costmap/", "/global_costmap/", "/turtle1/"];

/// Whether a topic on the graph is ROS's or a library's own, not ours to list.
fn is_internal(name: &str) -> bool {
    INTERNAL.iter().any(|p| name.starts_with(p)) || name.contains("/_action/")
}

/// The profiles each one runs whole, besides its own (check_graph.py's `INCLUDES`).
fn includes(profile: &str) -> &'static [&'static str] {
    match profile {
        "road" => &["sim"],
        "spine" => &["sim", "road"],
        "virtual" => &["sim", "road", "car"],
        _ => &[],
    }
}

/// One topic of the contract.
#[derive(Debug, Clone, PartialEq)]
pub struct Rule {
    /// Its name.
    pub name: String,
    /// Its type, such as `sensor_msgs/msg/Imu`.
    pub kind: String,
    /// The profiles where it must have a publisher.
    pub live: Vec<String>,
    /// On the car only (`not_virtual:`): the virtual car needn't have it.
    pub not_virtual: bool,
    /// `drop_in: {min_hz}`, if any.
    pub min_hz: Option<f64>,
}

impl Rule {
    /// Whether it must have a publisher in `profile` (check_graph.py's `must_be_live`).
    #[must_use]
    pub fn must_be_live(&self, profile: &str) -> bool {
        self.live.iter().any(|p| {
            let is_car_only = profile == "virtual" && self.not_virtual && p == "car";
            !is_car_only && (p == profile || includes(profile).contains(&p.as_str()))
        })
    }
}

/// The contract's topics, by name.
#[derive(Debug, Clone, Default)]
pub struct Contract {
    rules: BTreeMap<String, Rule>,
}

impl Contract {
    /// Reads `docs/interfaces.yaml`'s `topics:`.
    pub fn from_yaml(text: &str) -> Result<Self> {
        let doc: Value = serde_yaml_ng::from_str(text).context("interfaces.yaml")?;
        let Some(topics) = doc.get("topics").and_then(Value::as_mapping) else {
            bail!("interfaces.yaml has no topics:");
        };
        let mut rules = BTreeMap::new();
        for (name, entry) in topics {
            let name = name.as_str().context("a topic's name")?.to_owned();
            let kind = entry
                .get("type")
                .and_then(Value::as_str)
                .with_context(|| format!("{name}: no type"))?
                .to_owned();
            let live = entry
                .get("live")
                .and_then(Value::as_sequence)
                .map(|s| {
                    s.iter()
                        .filter_map(Value::as_str)
                        .map(str::to_owned)
                        .collect()
                })
                .unwrap_or_default();
            let not_virtual = entry
                .get("not_virtual")
                .is_some_and(|v| !v.is_null() && v.as_bool() != Some(false));
            let min_hz = entry
                .get("drop_in")
                .and_then(|d| d.get("min_hz"))
                .and_then(Value::as_f64);
            rules.insert(
                name.clone(),
                Rule {
                    name,
                    kind,
                    live,
                    not_virtual,
                    min_hz,
                },
            );
        }
        Ok(Self { rules })
    }

    /// How many topics it lists.
    #[cfg(test)]
    #[must_use]
    pub fn len(&self) -> usize {
        self.rules.len()
    }

    /// The topics that must be live in `profile`, by name.
    pub fn must_be_live<'a>(&'a self, profile: &'a str) -> impl Iterator<Item = &'a str> + 'a {
        self.rules
            .values()
            .filter(move |r| r.must_be_live(profile))
            .map(|r| r.name.as_str())
    }

    /// Every topic of the contract and of the graph, judged; the problems first.
    #[must_use]
    pub fn judge(&self, profile: &str, seen: &[Seen]) -> Vec<Row> {
        let seen: HashMap<&str, &Seen> = seen.iter().map(|s| (s.name.as_str(), s)).collect();
        let mut rows: Vec<Row> = self
            .rules
            .values()
            .map(|rule| {
                let on_graph = seen.get(rule.name.as_str());
                let publishers = on_graph.map_or(0, |s| s.publishers);
                let needed = rule.must_be_live(profile);
                let verdict = match on_graph {
                    Some(s) if s.types.iter().any(|t| *t != rule.kind) => Verdict::WrongType,
                    _ if publishers > 0 => Verdict::Live,
                    _ if needed => Verdict::Missing,
                    _ => Verdict::Quiet,
                };
                Row {
                    name: rule.name.clone(),
                    kind: rule.kind.clone(),
                    seen_types: on_graph.map(|s| s.types.clone()).unwrap_or_default(),
                    publishers,
                    needed,
                    min_hz: rule.min_hz,
                    verdict,
                }
            })
            .collect();
        rows.extend(
            seen.values()
                .filter(|s| {
                    s.publishers > 0
                        && !self.rules.contains_key(&s.name)
                        && !IGNORE.contains(&s.name.as_str())
                        && !is_internal(&s.name)
                })
                .map(|s| Row {
                    name: s.name.clone(),
                    kind: s.types.join(", "),
                    seen_types: s.types.clone(),
                    publishers: s.publishers,
                    needed: false,
                    min_hz: None,
                    verdict: Verdict::NotInContract,
                }),
        );
        rows.sort_by(|a, b| {
            a.verdict
                .rank()
                .cmp(&b.verdict.rank())
                .then(a.name.cmp(&b.name))
        });
        rows
    }
}

/// A topic on the running graph.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Seen {
    /// Its name.
    pub name: String,
    /// Its types.
    pub types: Vec<String>,
    /// How many publishers.
    pub publishers: usize,
}

/// What the ROS tab says of a topic.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Verdict {
    /// Needed here, and no publisher.
    Missing,
    /// Not the contract's type.
    WrongType,
    /// Published, but not in the contract.
    NotInContract,
    /// Published, the contract's type.
    Live,
    /// Not needed here, and not published.
    Quiet,
}

impl Verdict {
    fn rank(self) -> u8 {
        match self {
            Self::Missing => 0,
            Self::WrongType => 1,
            Self::NotInContract => 2,
            Self::Live => 3,
            Self::Quiet => 4,
        }
    }

    /// Whether it's a problem (red on the page).
    #[must_use]
    pub fn is_problem(self) -> bool {
        matches!(self, Self::Missing | Self::WrongType)
    }
}

/// One row of the ROS tab.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Row {
    /// The topic.
    pub name: String,
    /// The contract's type (or the graph's, for a topic not in the contract).
    pub kind: String,
    /// The types on the graph.
    pub seen_types: Vec<String>,
    /// Its publishers now.
    pub publishers: usize,
    /// Whether this profile needs it live.
    pub needed: bool,
    /// The contract's floor, Hz (`drop_in`), if any.
    pub min_hz: Option<f64>,
    /// The verdict.
    pub verdict: Verdict,
}

#[cfg(test)]
mod tests {
    use super::*;

    const YAML: &str = r"
topics:
  /sensors/imu/data:
    type: sensor_msgs/msg/Imu
    drop_in: {min_hz: 45}
    live: [sim, car]
  /vehicle/status:
    type: aav_interfaces/msg/VehicleStatus
    live: [turtle, sim, bench]
  /sensors/lidar/packets:
    type: velodyne_msgs/msg/VelodyneScan
    live: [car]
    not_virtual: Ravensim makes the points, not the packets
  /hud/screen_alive:
    type: std_msgs/msg/UInt64
services: {}
";

    fn seen(name: &str, kind: &str, publishers: usize) -> Seen {
        Seen {
            name: name.into(),
            types: vec![kind.into()],
            publishers,
        }
    }

    #[test]
    fn the_profiles_include_as_check_graph_says() {
        let c = Contract::from_yaml(YAML).expect("reads");
        assert_eq!(c.len(), 4);
        let live = |p| c.must_be_live(p).collect::<Vec<_>>();
        assert_eq!(live("car"), ["/sensors/imu/data", "/sensors/lidar/packets"]);
        // road runs the sim whole.
        assert_eq!(live("road"), ["/sensors/imu/data", "/vehicle/status"]);
        // virtual = sim + road + car, less the car-only topics.
        assert_eq!(live("virtual"), ["/sensors/imu/data", "/vehicle/status"]);
        assert!(live("turtle").contains(&"/vehicle/status"));
    }

    #[test]
    fn missing_and_wrong_types_come_first() {
        let c = Contract::from_yaml(YAML).expect("reads");
        let rows = c.judge(
            "sim",
            &[
                seen("/sensors/imu/data", "sensor_msgs/msg/Temperature", 1),
                seen("/extra/thing", "std_msgs/msg/Bool", 2),
                seen(
                    "/follow_path/_action/status",
                    "action_msgs/msg/GoalStatusArray",
                    1,
                ),
                seen("/rosout", "rcl_interfaces/msg/Log", 9),
                seen("/hud/screen_alive", "std_msgs/msg/UInt64", 0),
            ],
        );
        let names: Vec<(&str, Verdict)> =
            rows.iter().map(|r| (r.name.as_str(), r.verdict)).collect();
        assert_eq!(
            names,
            [
                ("/vehicle/status", Verdict::Missing),
                ("/sensors/imu/data", Verdict::WrongType),
                ("/extra/thing", Verdict::NotInContract),
                ("/hud/screen_alive", Verdict::Quiet),
                ("/sensors/lidar/packets", Verdict::Quiet),
            ]
        );
        assert_eq!(rows[1].min_hz, Some(45.0));
        assert!(rows[0].verdict.is_problem() && !rows[2].verdict.is_problem());
    }

    #[test]
    fn a_published_topic_with_the_right_type_is_live() {
        let c = Contract::from_yaml(YAML).expect("reads");
        let rows = c.judge(
            "car",
            &[
                seen("/sensors/imu/data", "sensor_msgs/msg/Imu", 1),
                seen(
                    "/sensors/lidar/packets",
                    "velodyne_msgs/msg/VelodyneScan",
                    1,
                ),
            ],
        );
        assert!(rows
            .iter()
            .filter(|r| r.needed)
            .all(|r| r.verdict == Verdict::Live));
    }

    /// The same rule as tools/check_graph.py: its INCLUDES, PROFILES and IGNORE, word for word.
    #[test]
    fn check_graph_py_says_the_same() {
        let py = include_str!("../../../../../../tools/check_graph.py");
        let line = |start: &str| {
            py.lines()
                .find(|l| l.starts_with(start))
                .unwrap_or_else(|| panic!("check_graph.py has no {start}"))
                .to_owned()
        };
        assert_eq!(
            line("INCLUDES = "),
            "INCLUDES = {'road': ('sim',), 'spine': ('sim', 'road'), 'virtual': ('sim', 'road', 'car')}"
        );
        let profiles: Vec<String> = PROFILES.iter().map(|p| format!("'{p}'")).collect();
        assert_eq!(
            line("PROFILES = "),
            format!("PROFILES = ({})", profiles.join(", "))
        );
        let ignore = py
            .split("IGNORE = {")
            .nth(1)
            .expect("IGNORE")
            .split('}')
            .next()
            .expect("}");
        for t in IGNORE {
            assert!(
                ignore.contains(&format!("'{t}'")),
                "check_graph.py doesn't ignore {t}"
            );
        }
        assert!(py.contains("if profile == 'virtual' and topic.get('not_virtual'):"));
        assert!(py.contains("internal = ('/local_costmap/', '/global_costmap/', '/turtle1/')"));
        assert!(py.contains("'/_action/' not in n"));
    }

    #[test]
    fn the_real_contract_reads() {
        let text = include_str!("../../../../../../docs/interfaces.yaml");
        let c = Contract::from_yaml(text).expect("docs/interfaces.yaml reads");
        assert!(c.len() > 50, "only {} topics", c.len());
        for p in PROFILES {
            // Every profile but replay's bag needs something live.
            if p != "replay" {
                assert!(c.must_be_live(p).next().is_some(), "{p}: nothing live");
            }
        }
    }
}
