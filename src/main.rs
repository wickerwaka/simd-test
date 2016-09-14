#![feature(test)]
#![feature(step_by)]

#![allow(dead_code)]
#![allow(unused_imports)]

extern crate test;
extern crate time;
extern crate cgmath;
extern crate rand;
extern crate simd;

use std::str::FromStr;
use std::env;
use cgmath::prelude::*;
use simd::{f32x4,u32x4,Simd};
use simd::x86::sse2::Sse2F32x4;

pub type Vector3 = cgmath::Vector3<f32>;
pub type Vector4 = cgmath::Vector4<f32>;
pub type Matrix4 = cgmath::Matrix4<f32>;


fn to_u32( x: f32x4 ) -> u32x4 {
    unsafe { std::mem::transmute( x ) }
}

fn to_f32( x: u32x4 ) -> f32x4 {
    unsafe { std::mem::transmute( x ) }
}


#[derive(Debug, Clone)]
#[repr(C)]
pub struct AABB {
    center: Vector3,
    extent: Vector3
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct Plane {
    n: Vector3,
    d: f32
}

#[derive(Debug)]
pub struct Volume {
    planes: Vec<Plane>
}

extern {
    fn contains_bulk_c( planes: *const Plane, plane_count: usize, aabbs: *const AABB, aabb_count: usize, results: *mut bool );
    fn contains_bulk_c_simd( planes: *const Plane, plane_count: usize, aabbs: *const AABB, aabb_count: usize, results: *mut bool );
}

impl AABB {
    fn from_min_max( mn: Vector3, mx: Vector3 ) -> AABB {
        AABB {
            center: ( mn + mx ) / 2.0,
            extent: ( mx - mn ) / 2.0
        }
    }

    fn new( center: Vector3, extent: Vector3 ) -> AABB {
        AABB {
            center: center,
            extent: extent
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

    fn from_cube( extent: &Vector3 ) -> Volume {
        Volume {
            planes: vec![
                Plane::new( -Vector3::unit_x(), extent[0] ),
                Plane::new( Vector3::unit_x(), extent[0] ),
                Plane::new( -Vector3::unit_y(), extent[1] ),
                Plane::new( Vector3::unit_y(), extent[1] ),
                Plane::new( -Vector3::unit_z(), extent[2] ),
                Plane::new( Vector3::unit_z(), extent[2] )
            ]
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
    
    #[inline(never)]
    fn contains_bulk( &self, aabbs: &[AABB] ) -> Vec<bool> {
        let mut result = Vec::with_capacity( aabbs.len() );

        for idx in 0..aabbs.len() {
            result.push( self.contains( &aabbs[idx] ) );
        }

        result
    }

    fn contains_simd( &self, aabb: &AABB ) -> bool {
        let sign_mask = u32x4::splat( 0x80000000 );

        let num_planes_4 = ( self.planes.len() + 3 ) & !3;
        let mut planes_x = Vec::with_capacity( num_planes_4 );
        let mut planes_y = Vec::with_capacity( num_planes_4 );
        let mut planes_z = Vec::with_capacity( num_planes_4 );
        let mut planes_w = Vec::with_capacity( num_planes_4 );

        let mut it = self.planes.iter().cycle();
        for _ in 0..num_planes_4 {
            let plane = it.next().unwrap();
            planes_x.push( plane.n[0] );
            planes_y.push( plane.n[1] );
            planes_z.push( plane.n[2] );
            planes_w.push( plane.d );
        }

        let extent_x = f32x4::splat( aabb.extent[0] );
        let extent_y = f32x4::splat( aabb.extent[1] );
        let extent_z = f32x4::splat( aabb.extent[2] );
        let center_x = f32x4::splat( aabb.center[0] );
        let center_y = f32x4::splat( aabb.center[1] );
        let center_z = f32x4::splat( aabb.center[2] );

        for pidx in ( 0..num_planes_4 ).step_by(4) {
            let plane_x = f32x4::load( &planes_x, pidx );
            let plane_y = f32x4::load( &planes_y, pidx );
            let plane_z = f32x4::load( &planes_z, pidx );
            let plane_w = f32x4::load( &planes_w, pidx );

            let e_x = to_u32( extent_x ) ^ ( to_u32( plane_x ) & sign_mask );
            let t_x = center_x + to_f32( e_x );
            let e_y = to_u32( extent_y ) ^ ( to_u32( plane_y ) & sign_mask );
            let t_y = center_y + to_f32( e_y );
            let e_z = to_u32( extent_z ) ^ ( to_u32( plane_z ) & sign_mask );
            let t_z = center_z + to_f32( e_z );

            let dot = ( t_x * plane_x ) + ( t_y * plane_y ) + ( t_z * plane_z ) + plane_w;
                
            if dot.move_mask() != 0 {
                return false;
            }
        }

        return true;
    }


    #[inline(never)]
    fn contains_bulk_simd( &self, aabbs: &[AABB] ) -> Vec<bool> {
        let sign_mask = u32x4::splat( 0x80000000 );

        let num_planes_4 = ( self.planes.len() + 3 ) & !3;

        let mut planes_x = Vec::with_capacity( num_planes_4 );
        let mut planes_y = Vec::with_capacity( num_planes_4 );
        let mut planes_z = Vec::with_capacity( num_planes_4 );
        let mut planes_w = Vec::with_capacity( num_planes_4 );

        let mut it = self.planes.iter().cycle();
        for _ in 0..num_planes_4 {
            let plane = it.next().unwrap();
            planes_x.push( plane.n[0] );
            planes_y.push( plane.n[1] );
            planes_z.push( plane.n[2] );
            planes_w.push( plane.d );
        }

        let mut result = Vec::with_capacity( aabbs.len() );
        unsafe{ result.set_len( aabbs.len() ); }

        {
            let mut rs = result.as_mut_slice();

            for idx in 0..aabbs.len() {
                let aabb = &aabbs[idx];
                let extent_x = f32x4::splat( aabb.extent[0] );
                let extent_y = f32x4::splat( aabb.extent[1] );
                let extent_z = f32x4::splat( aabb.extent[2] );
                let center_x = f32x4::splat( aabb.center[0] );
                let center_y = f32x4::splat( aabb.center[1] );
                let center_z = f32x4::splat( aabb.center[2] );

                rs[idx] = true;

                let mut pidx = 0;

                while pidx < num_planes_4 {
                    let plane_x = f32x4::load( &planes_x, pidx );
                    let plane_y = f32x4::load( &planes_y, pidx );
                    let plane_z = f32x4::load( &planes_z, pidx );
                    let plane_w = f32x4::load( &planes_w, pidx );

                    let e_x = to_u32( extent_x ) ^ ( to_u32( plane_x ) & sign_mask );
                    let t_x = center_x + to_f32( e_x );
                    let e_y = to_u32( extent_y ) ^ ( to_u32( plane_y ) & sign_mask );
                    let t_y = center_y + to_f32( e_y );
                    let e_z = to_u32( extent_z ) ^ ( to_u32( plane_z ) & sign_mask );
                    let t_z = center_z + to_f32( e_z );

                    let dot = ( t_x * plane_x ) + ( t_y * plane_y ) + ( t_z * plane_z ) + plane_w;

                    if dot.move_mask() != 0 {
                        rs[idx] = false;
                        break;
                    }
                    pidx += 4;
                }
            }
        }

        result
    }

    fn contains_bulk_c( &self, aabbs: &[AABB] ) -> Vec<bool> {
        let mut results = Vec::with_capacity( aabbs.len() );
        unsafe {
            results.set_len( aabbs.len() );
            contains_bulk_c( self.planes.as_ptr(), self.planes.len(), aabbs.as_ptr(), aabbs.len(), results.as_mut_ptr() );
        }
        results
    }

    fn contains_bulk_c_simd( &self, aabbs: &[AABB] ) -> Vec<bool> {
        let mut results = Vec::with_capacity( aabbs.len() );
        unsafe {
            results.set_len( aabbs.len() );
            contains_bulk_c_simd( self.planes.as_ptr(), self.planes.len(), aabbs.as_ptr(), aabbs.len(), results.as_mut_ptr() );
        }
        results
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
            if volume2.contains_simd( &aabb ) {
                print!( "*" );
            } else {
                print!( " " );
            }
        }
        println!("");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test::Bencher;
    use cgmath::prelude::*;
    use cgmath;
    use rand;
    use rand::distributions::{IndependentSample, Range};

    fn random_aabbs( count : usize ) -> Vec<AABB> {
        let range = Range::new( -1000.0, 1000.0 );
        let mut rng = rand::thread_rng();

        let mut aabbs = Vec::new();
        for _ in 0..count {
            let aabb = AABB::new( Vector3::new(
                    range.ind_sample( &mut rng ), range.ind_sample( &mut rng ), range.ind_sample( &mut rng )
                    ), Vector3::from_value( range.ind_sample( &mut rng ) ) );
            aabbs.push( aabb );
        }

        aabbs
    }

    fn test_volume() -> Volume {
        let mat = Matrix4::from_angle_y( cgmath::Deg( 45.0 ) );
        Volume::from_cube( &Vector3::new( 500.0, 500.0, 500.0 ) ).transform( &mat )
    }

    fn test_all( volume : &Volume, aabb : &AABB ) -> bool {
        let base_result = volume.contains( aabb );
        let simd_result = volume.contains_simd( aabb );
        assert_eq!( base_result, simd_result );

        let mut aabbs = Vec::new();
        aabbs.push( aabb.clone() );
        let bulk_result = volume.contains_bulk( &aabbs );
        assert!( bulk_result.len() == 1 );
        assert_eq!( base_result, bulk_result[0] );

        let bulk_simd_result = volume.contains_bulk_simd( &aabbs );
        assert!( bulk_simd_result.len() == 1 );
        assert_eq!( base_result, bulk_simd_result[0] );

        let bulk_c_result = volume.contains_bulk_c( &aabbs );
        assert!( bulk_c_result.len() == 1 );
        println!( "{:?}", bulk_c_result );
        assert_eq!( base_result, bulk_c_result[0] );

        base_result
    }

    #[test]
    fn inside_cube() {
        let volume = Volume::from_cube( &Vector3::new( 10.0, 10.0, 10.0 ) );
        let aabb = AABB::new( Vector3::new( 5.0, 5.0, 5.0 ), Vector3::new( 1.0, 1.0, 1.0 ) );
        assert_eq!( test_all( &volume, &aabb ), true );
    }

    #[test]
    fn outside_cube() {
        let volume = Volume::from_cube( &Vector3::new( 10.0, 10.0, 10.0 ) );
        let aabb = AABB::new( Vector3::new( 15.0, 15.0, 15.0 ), Vector3::new( 1.0, 1.0, 1.0 ) );
        assert_eq!( test_all( &volume, &aabb ), false );
    }

    #[test]
    fn intersecting_cube() {
        let volume = Volume::from_cube( &Vector3::new( 10.0, 10.0, 10.0 ) );
        let aabb = AABB::new( Vector3::new( 9.5, 9.5, 9.5 ), Vector3::new( 1.0, 1.0, 1.0 ) );
        assert_eq!( test_all( &volume, &aabb ), true );
    }

    #[test]
    fn compare_bulk() {
        let volume = test_volume();
        let aabbs = random_aabbs( 100 );

        let bulk_non_simd = volume.contains_bulk( &aabbs );
        let bulk_with_simd = volume.contains_bulk_simd( &aabbs );

        assert_eq!( bulk_with_simd, bulk_non_simd );
    }


    #[bench]
    fn bench_volume_aabb(b: &mut Bencher) {
        let volume = test_volume();
        let aabbs = random_aabbs( 1000 );

        b.iter(|| {
            let mut v = Vec::with_capacity( aabbs.len() );
            for a in aabbs.iter() {
                v.push( volume.contains( &a ) );
            }
        } );
    }

    #[bench]
    fn bench_volume_aabb_simd(b: &mut Bencher) {
        let volume = test_volume();
        let aabbs = random_aabbs( 1000 );

        b.iter(|| {
            let mut v = Vec::with_capacity( aabbs.len() );
            for a in aabbs.iter() {
                v.push( volume.contains_simd( &a ) );
            }
        } );
    }

    #[bench]
    fn bench_volume_aabb_bulk(b: &mut Bencher) {
        let volume = test_volume();
        let aabbs = random_aabbs( 1000 );

        b.iter(|| volume.contains_bulk( aabbs.as_slice() ) );
    }

    #[bench]
    fn bench_volume_aabb_bulk_c(b: &mut Bencher) {
        let volume = test_volume();
        let aabbs = random_aabbs( 1000 );

        b.iter(|| volume.contains_bulk_c( aabbs.as_slice() ) );
    }

    #[bench]
    fn bench_volume_aabb_bulk_c_simd(b: &mut Bencher) {
        let volume = test_volume();
        let aabbs = random_aabbs( 1000 );

        b.iter(|| volume.contains_bulk_c_simd( aabbs.as_slice() ) );
    }


    #[bench]
    fn bench_volume_aabb_bulk_simd(b: &mut Bencher) {
        let volume = test_volume();
        let aabbs = random_aabbs( 1000 );

        b.iter(|| volume.contains_bulk_simd( aabbs.as_slice() ) );
    }
}
