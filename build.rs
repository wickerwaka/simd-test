extern crate gcc;

fn main() {
    gcc::Config::new()
        .file( "src/aabb.c" )
        .flag( "-std=c99" )
        .flag( "-march=native" )
        .compile( "libaabb.a" );
}

