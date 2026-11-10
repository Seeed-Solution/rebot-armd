//! Minimal URDF reader: links (inertial), joints (type, origin, axis, limits).
//!
//! Only the elements armd uses are read; visual / collision geometry, transmissions and
//! Gazebo tags are ignored. Parsing uses `roxmltree` (read-only XML tree, no dependencies).

use crate::kinematics::{Mat3, Vec3};

#[derive(Debug, Clone, PartialEq)]
pub struct Inertial {
    /// Mass (kg).
    pub mass: f64,
    /// Centre of mass in the link frame (m).
    pub com: Vec3,
    /// Rotation of the inertia frame in the link frame (URDF `<inertial><origin rpy>`).
    pub rot: Mat3,
    /// ixx, ixy, ixz, iyy, iyz, izz (kg·m²) in the inertia frame.
    pub inertia: [f64; 6],
}

#[derive(Debug, Clone, PartialEq)]
pub struct Link {
    pub name: String,
    pub inertial: Option<Inertial>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType { Revolute, Continuous, Prismatic, Fixed, Floating, Planar }

#[derive(Debug, Clone, PartialEq)]
pub struct Joint {
    pub name: String,
    pub kind: JointType,
    pub parent: String,
    pub child: String,
    /// Origin of the joint frame in the parent link frame.
    pub xyz: Vec3,
    /// Origin rotation, fixed-axis roll / pitch / yaw (rad).
    pub rpy: Vec3,
    /// Joint axis in the joint frame (as written; default 1 0 0).
    pub axis: Vec3,
    /// `<limit lower upper>` (rad or m), when present.
    pub limit: Option<(f64, f64)>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Urdf {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
}

impl Urdf {
    pub fn link(&self, name: &str) -> Option<&Link> { self.links.iter().find(|l| l.name == name) }
    /// The joint whose child is `link`.
    pub fn parent_joint(&self, link: &str) -> Option<&Joint> { self.joints.iter().find(|j| j.child == link) }
    pub fn child_joints<'a>(&'a self, link: &'a str) -> impl Iterator<Item = &'a Joint> + 'a {
        self.joints.iter().filter(move |j| j.parent == link)
    }
}

fn vec3(s: Option<&str>, default: Vec3, what: &str) -> Result<Vec3, String> {
    let Some(s) = s else { return Ok(default); };
    let v: Vec<f64> = s.split_whitespace().map(str::parse).collect::<Result<_, _>>()
        .map_err(|e| format!("{what}: '{s}': {e}"))?;
    match v.as_slice() {
        [x, y, z] if v.iter().all(|x| x.is_finite()) => Ok([*x, *y, *z]),
        _ => Err(format!("{what}: '{s}' is not three finite numbers")),
    }
}

fn num(s: Option<&str>, what: &str) -> Result<f64, String> {
    let s = s.ok_or_else(|| format!("{what} missing"))?;
    let v: f64 = s.trim().parse().map_err(|e| format!("{what}: '{s}': {e}"))?;
    if v.is_finite() { Ok(v) } else { Err(format!("{what}: '{s}' is not finite")) }
}

fn child<'a, 'i>(n: roxmltree::Node<'a, 'i>, tag: &str) -> Option<roxmltree::Node<'a, 'i>> {
    n.children().find(|c| c.has_tag_name(tag))
}

fn origin(n: roxmltree::Node, what: &str) -> Result<(Vec3, Vec3), String> {
    match child(n, "origin") {
        Some(o) => Ok((vec3(o.attribute("xyz"), [0.0; 3], &format!("{what} origin xyz"))?,
                       vec3(o.attribute("rpy"), [0.0; 3], &format!("{what} origin rpy"))?)),
        None => Ok(([0.0; 3], [0.0; 3])),
    }
}

/// Parse URDF text.
pub fn parse(text: &str) -> Result<Urdf, String> {
    let doc = roxmltree::Document::parse(text).map_err(|e| format!("URDF XML: {e}"))?;
    let robot = doc.root_element();
    if !robot.has_tag_name("robot") {
        return Err("URDF root element is not <robot>".into());
    }
    let mut links = Vec::new();
    let mut joints = Vec::new();
    for n in robot.children().filter(|n| n.is_element()) {
        let name = || n.attribute("name").map(str::to_string).ok_or_else(|| format!("<{}> without a name", n.tag_name().name()));
        match n.tag_name().name() {
            "link" => {
                let name = name()?;
                let inertial = match child(n, "inertial") {
                    Some(i) => {
                        let what = format!("link '{name}' inertial");
                        let (com, rpy) = origin(i, &what)?;
                        let mass = num(child(i, "mass").and_then(|m| m.attribute("value")), &format!("{what} mass"))?;
                        if mass < 0.0 { return Err(format!("{what}: negative mass")); }
                        let inertia = match child(i, "inertia") {
                            Some(t) => {
                                let mut v = [0.0; 6];
                                for (k, a) in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"].iter().enumerate() {
                                    v[k] = num(t.attribute(*a), &format!("{what} {a}"))?;
                                }
                                v
                            }
                            None => [0.0; 6],
                        };
                        Some(Inertial { mass, com, rot: crate::kinematics::rpy_to_r(rpy[0], rpy[1], rpy[2]), inertia })
                    }
                    None => None,
                };
                links.push(Link { name, inertial });
            }
            "joint" => {
                let name = name()?;
                let kind = match n.attribute("type") {
                    Some("revolute") => JointType::Revolute,
                    Some("continuous") => JointType::Continuous,
                    Some("prismatic") => JointType::Prismatic,
                    Some("fixed") => JointType::Fixed,
                    Some("floating") => JointType::Floating,
                    Some("planar") => JointType::Planar,
                    other => return Err(format!("joint '{name}': unknown type {other:?}")),
                };
                let link_of = |tag: &str| child(n, tag).and_then(|c| c.attribute("link")).map(str::to_string)
                    .ok_or_else(|| format!("joint '{name}': <{tag} link> missing"));
                let (parent, child_link) = (link_of("parent")?, link_of("child")?);
                let (xyz, rpy) = origin(n, &format!("joint '{name}'"))?;
                let axis = vec3(child(n, "axis").and_then(|a| a.attribute("xyz")), [1.0, 0.0, 0.0], &format!("joint '{name}' axis"))?;
                let limit = match child(n, "limit") {
                    Some(l) if l.attribute("lower").is_some() || l.attribute("upper").is_some() => Some((
                        l.attribute("lower").map_or(Ok(0.0), |_| num(l.attribute("lower"), &format!("joint '{name}' lower")))?,
                        l.attribute("upper").map_or(Ok(0.0), |_| num(l.attribute("upper"), &format!("joint '{name}' upper")))?,
                    )),
                    _ => None,
                };
                joints.push(Joint { name, kind, parent, child: child_link, xyz, rpy, axis, limit });
            }
            _ => {}
        }
    }
    for j in &joints {
        if !links.iter().any(|l| l.name == j.parent) || !links.iter().any(|l| l.name == j.child) {
            return Err(format!("joint '{}' names a link that is not defined", j.name));
        }
        if joints.iter().filter(|k| k.child == j.child).count() > 1 {
            return Err(format!("link '{}' has more than one parent joint", j.child));
        }
    }
    Ok(Urdf { name: robot.attribute("name").unwrap_or("").to_string(), links, joints })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::approx_constant)]
    fn parses_the_dm_urdf() {
        let u = parse(include_str!("../assets/reBot-DevArm_gripper.urdf")).unwrap();
        assert_eq!(u.links.len(), 10);
        assert_eq!(u.joints.len(), 9);
        let j2 = u.joints.iter().find(|j| j.name == "joint2").unwrap();
        assert_eq!(j2.kind, JointType::Revolute);
        assert_eq!(j2.axis, [0.0, 0.0, -1.0]);
        assert_eq!(j2.rpy, [-1.5708, 0.0, 0.0]); // the URDF value
        assert_eq!(u.link("end_link").unwrap().inertial.as_ref().unwrap().mass, 0.55);
    }

    #[test]
    fn rejects_bad_input() {
        assert!(parse("<robot><joint name='a' type='weird'/></robot>").is_err());
        assert!(parse("<robot><link name='a'><inertial><mass value='x'/></inertial></link></robot>").is_err());
        assert!(parse("<robot><link name='a'/><joint name='j' type='fixed'><parent link='a'/><child link='b'/></joint></robot>").is_err());
        assert!(parse("<notrobot/>").is_err());
    }
}
