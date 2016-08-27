extern crate time;
extern crate cgmath;

use cgmath::prelude::*;

type Vector3 = cgmath::Vector3<f32>;
type Vector4 = cgmath::Vector4<f32>;
type Matrix4 = cgmath::Matrix4<f32>;

#[derive(Debug)]
struct AABB {
    center: Vector3,
    extent: Vector3
}

#[derive(Debug, Clone)]
struct Plane {
    n: Vector3,
    d: f32
}

#[derive(Debug)]
struct Volume {
    planes: Vec<Plane>
}

impl AABB {
    fn from_min_max( mn: Vector3, mx: Vector3 ) -> AABB {
        AABB {
            center: ( mn + mx ) / 2.0,
            extent: ( mx - mn ) / 2.0
        }
    }
}

impl Plane {
    fn new( n: Vector3, d: f32 ) -> Plane {
        Plane {
            n: n,
            d: d
        }
    }

    fn from_vector4( v: &Vector4 ) -> Plane {
        Plane {
            n: v.truncate(),
            d: v[3]
        }
    }
}

impl Default for Volume {
    fn default() -> Volume {
        Volume {
            planes: vec!()
        }
    }
}

impl Volume {
    fn from_planes( planes: &[Plane] ) -> Volume {
        Volume {
            planes: planes.to_vec()
        }
    }

    fn contains( &self, aabb: &AABB ) -> bool {
        for plane in self.planes.iter() {
            let plane_sign = Vector3::new( plane.n[0].signum(), plane.n[1].signum(), plane.n[2].signum() );
            if plane.n.dot( aabb.center + ( aabb.extent.mul_element_wise( plane_sign ) ) ) <= -plane.d {
                return false;
            }
        }

        return true;
    }

    fn transform( &self, mat: &Matrix4 ) -> Volume {
        let tmat = mat.invert().unwrap().transpose();
        let mut planes = Vec::with_capacity( self.planes.len() );
        for src in self.planes.iter() {
            let src4 = Vector4::new( src.n[0], src.n[1], src.n[2], src.d );
            let dst4 = tmat * src4;
            planes.push( Plane::from_vector4( &dst4 ) );
        }

        Volume {
            planes: planes
        }
    }
}

fn main() {
    let volume = Volume::from_planes( &[
        Plane::new( -Vector3::unit_y(), 1.0 ),
        Plane::new( Vector3::unit_y(), 1.0 ),
        Plane::new( Vector3::unit_x(), 0.0 )
    ] );

    let mat = Matrix4::from_angle_z( cgmath::Deg( 45.0 ) ) * Matrix4::from_translation( Vector3::new( 0.0, 0.0, 0.0 ) );

    let volume2 = volume.transform( &mat );

    for y in -10..10 {
        for x in -10..10 {
            let aabb = AABB::from_min_max(
                Vector3::new( x as f32 - 0.2, y as f32 - 0.2, -0.2 ),
                Vector3::new( x as f32 + 0.2, y as f32 + 0.2, 0.2 )
            );
            if volume2.contains( &aabb ) {
                print!( "*" );
            } else {
                print!( " " );
            }
        }
        println!("");
    }
}
