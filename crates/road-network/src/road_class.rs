//! Road classes: OpenLR's Functional Road Class ([`Frc`]) and Form of Way
//! ([`Fow`]), and how they are inferred from OSM tags.

use serde::{Deserialize, Serialize};

/// Functional Road Class, 0 (most important) to 7. HERE's FC1–FC5 encode as
/// Frc0–Frc4; HERE never uses Frc5–Frc7.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum Frc {
    Frc0 = 0, // motorways
    Frc1 = 1, // major routes between/through cities
    Frc2 = 2, // high-volume roads feeding major routes
    Frc3 = 3, // roads between neighborhoods
    Frc4 = 4, // local/residential
    Frc5 = 5,
    Frc6 = 6,
    Frc7 = 7,
}

impl Frc {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => Frc::Frc0,
            1 => Frc::Frc1,
            2 => Frc::Frc2,
            3 => Frc::Frc3,
            4 => Frc::Frc4,
            5 => Frc::Frc5,
            6 => Frc::Frc6,
            _ => Frc::Frc7,
        }
    }

    /// Map an OSM `highway` tag onto HERE's Frc0–Frc4 range.
    pub fn from_osm_highway(highway: &str) -> Self {
        match highway {
            "motorway" => Frc::Frc0,
            "trunk" => Frc::Frc1,
            "primary" => Frc::Frc2,
            "secondary" => Frc::Frc3,
            "motorway_link" | "trunk_link" | "primary_link" => Frc::Frc3,
            "tertiary" | "secondary_link" | "tertiary_link" | "unclassified" | "residential"
            | "living_street" | "service" | "track" => Frc::Frc4,
            // Not drivable in HERE's network. Frc7 is far enough from Frc0–Frc4
            // that the default ±2 tolerance never matches it.
            _ => Frc::Frc7,
        }
    }

    /// Residential, living_street, service and track. These share Frc4 with
    /// tertiary roads, so A* tells them apart with a cost penalty instead.
    pub fn is_access_road(highway: &str) -> bool {
        matches!(
            highway,
            "residential" | "living_street" | "service" | "track"
        )
    }

    /// Whether the two classes are at most `tolerance` levels apart.
    pub fn is_compatible(&self, other: Frc, tolerance: u8) -> bool {
        let diff = (*self as i8 - other as i8).unsigned_abs();
        diff <= tolerance
    }
}

/// Form of Way (OpenLR spec)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum Fow {
    Undefined = 0,
    Motorway = 1,
    MultipleCarriageway = 2,
    SingleCarriageway = 3,
    Roundabout = 4,
    TrafficSquare = 5,
    SlipRoad = 6,
    Other = 7,
}

impl Fow {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => Fow::Undefined,
            1 => Fow::Motorway,
            2 => Fow::MultipleCarriageway,
            3 => Fow::SingleCarriageway,
            4 => Fow::Roundabout,
            5 => Fow::TrafficSquare,
            6 => Fow::SlipRoad,
            _ => Fow::Other,
        }
    }

    /// Infer FOW from OSM tags. The loader has no `junction` or `oneway`
    /// columns and passes `None` for both.
    pub fn from_osm_tags(
        highway: &str,
        junction: Option<&str>,
        oneway: Option<&str>,
        lanes: Option<u8>,
    ) -> Self {
        if junction == Some("roundabout") {
            return Fow::Roundabout;
        }

        match highway {
            "motorway" => Fow::Motorway,
            "motorway_link" | "trunk_link" | "primary_link" | "secondary_link"
            | "tertiary_link" => Fow::SlipRoad,
            "trunk" | "primary" | "secondary" => {
                // OSM maps dual carriageways as two one-way ways.
                if lanes.unwrap_or(1) >= 4 || oneway == Some("yes") {
                    Fow::MultipleCarriageway
                } else {
                    Fow::SingleCarriageway
                }
            }
            _ => Fow::SingleCarriageway,
        }
    }

    /// How well an edge with this FOW stands in for an LRP with FOW `other`:
    /// 1.0 for an exact match, 0.0 for incompatible. The matrix comes from
    /// TomTom's reference implementation.
    pub fn substitution_score(&self, other: Fow) -> f64 {
        // Rows: LRP FOW. Columns: edge FOW.
        #[rustfmt::skip]
        const FOW_SCORE_MATRIX: [[f64; 8]; 8] = [
            //       Und   Mwy   Mlt   Sgl   Rnd   Tsq   Slp   Oth
            /* Und */ [0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50],
            /* Mwy */ [0.50, 1.00, 0.75, 0.00, 0.00, 0.00, 0.00, 0.00],
            /* Mlt */ [0.50, 0.75, 1.00, 0.75, 0.50, 0.00, 0.00, 0.00],
            /* Sgl */ [0.50, 0.00, 0.75, 1.00, 0.50, 0.50, 0.00, 0.00],
            /* Rnd */ [0.50, 0.00, 0.50, 0.50, 1.00, 0.50, 0.00, 0.00],
            /* Tsq */ [0.50, 0.00, 0.00, 0.50, 0.50, 1.00, 0.00, 0.00],
            /* Slp */ [0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00],
            /* Oth */ [0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00],
        ];

        let wanted_idx = other as usize;
        let actual_idx = *self as usize;
        FOW_SCORE_MATRIX[wanted_idx][actual_idx]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frc_from_osm() {
        assert_eq!(Frc::from_osm_highway("motorway"), Frc::Frc0);
        assert_eq!(Frc::from_osm_highway("trunk"), Frc::Frc1);
        assert_eq!(Frc::from_osm_highway("primary"), Frc::Frc2);
        assert_eq!(Frc::from_osm_highway("secondary"), Frc::Frc3);
        assert_eq!(Frc::from_osm_highway("motorway_link"), Frc::Frc3);
        // FRC4: Local/residential roads
        assert_eq!(Frc::from_osm_highway("tertiary"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("secondary_link"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("tertiary_link"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("unclassified"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("residential"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("living_street"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("service"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("track"), Frc::Frc4);
        // FRC7: Not navigable
        assert_eq!(Frc::from_osm_highway("cycleway"), Frc::Frc7);
    }

    #[test]
    fn test_frc_compatibility() {
        assert!(Frc::Frc2.is_compatible(Frc::Frc2, 1));
        assert!(Frc::Frc2.is_compatible(Frc::Frc3, 1));
        assert!(!Frc::Frc2.is_compatible(Frc::Frc5, 1));
    }

    #[test]
    fn test_fow_substitution_score() {
        assert_eq!(Fow::Motorway.substitution_score(Fow::Motorway), 1.0);
        assert_eq!(
            Fow::SingleCarriageway.substitution_score(Fow::SingleCarriageway),
            1.0
        );

        // Undefined should match anything at 0.5
        assert_eq!(Fow::Undefined.substitution_score(Fow::Motorway), 0.5);
        assert_eq!(Fow::Motorway.substitution_score(Fow::Undefined), 0.5);

        // Some providers encode freeways as MultipleCarriageway.
        assert_eq!(
            Fow::Motorway.substitution_score(Fow::MultipleCarriageway),
            0.75
        );
        assert_eq!(
            Fow::MultipleCarriageway.substitution_score(Fow::Motorway),
            0.75
        );

        // Incompatible types should score 0.0
        assert_eq!(Fow::Roundabout.substitution_score(Fow::Motorway), 0.0);
        assert_eq!(
            Fow::SlipRoad.substitution_score(Fow::SingleCarriageway),
            0.0
        );
    }
}
