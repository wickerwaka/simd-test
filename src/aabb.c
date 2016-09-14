#include <stdio.h>
#include <immintrin.h>

typedef struct
{
	float cx, cy, cz;
	float ex, ey, ez;
} AABB;

typedef struct
{
	float x, y, z, w;
} Plane;

void contains_bulk_c( const Plane *planes, size_t plane_count, const AABB *aabbs, size_t aabb_count, _Bool *results )
{
	for( size_t idx = 0; idx < aabb_count; idx++ )
	{
		const AABB *aabb = &aabbs[idx];

		results[idx] = 1;

		for( size_t pidx = 0; pidx < plane_count; pidx++ )
		{
			const Plane *plane = &planes[pidx];
			float extent[3];
			extent[0] = plane->x < 0.0f ? -aabb->ex : aabb->ex;
			extent[1] = plane->y < 0.0f ? -aabb->ey : aabb->ey;
			extent[2] = plane->z < 0.0f ? -aabb->ez : aabb->ez;

			float c[3];
			c[0] = extent[0] + aabb->cx;
			c[1] = extent[1] + aabb->cy;
			c[2] = extent[2] + aabb->cz;

			float d = ( plane->x * c[0] ) + ( plane->y * c[1] ) + ( plane->z * c[2] );
			if( d <= -plane->w )
			{
				results[idx] = 0;
				break;
			}
		}
	}
}

typedef struct
{
	__m128 x, y, z, w;
} PlaneBlock;

void contains_bulk_c_simd( const Plane *_planes, size_t plane_count, const AABB *aabbs, size_t aabb_count, _Bool *results )
{
	size_t plane_count_4 = ( plane_count + 3 ) / 4;
	PlaneBlock *planes = (PlaneBlock *)malloc( sizeof( PlaneBlock ) * plane_count_4 );

	unsigned int sm[] = { 0x80000000, 0x80000000, 0x80000000, 0x80000000 };

	const __m128i sign_mask = _mm_loadu_si128( (__m128i *)&sm );

	for( size_t p = 0; p < plane_count_4; p++ )
	{
		const Plane *plane[4];
		size_t pidx = p * 4;
		for( int i = 0; i < 4; i++ )
		{
			if( ( pidx + i ) < plane_count )
				plane[i] = &_planes[pidx+i];
			else
				plane[i] = &_planes[0];
		}

		planes[p].x = _mm_set_ps( plane[0]->x, plane[1]->x, plane[2]->x, plane[3]->x );
		planes[p].y = _mm_set_ps( plane[0]->y, plane[1]->y, plane[2]->y, plane[3]->y );
		planes[p].z = _mm_set_ps( plane[0]->z, plane[1]->z, plane[2]->z, plane[3]->z );
		planes[p].w = _mm_set_ps( plane[0]->w, plane[1]->w, plane[2]->w, plane[3]->w );
	}


	for( size_t idx = 0; idx < aabb_count; idx++ )
	{
		const AABB *aabb = &aabbs[idx];
		results[idx] = 1;

		const __m128 extent_x = _mm_broadcast_ss( &aabb->ex );
		const __m128 extent_y = _mm_broadcast_ss( &aabb->ey );
		const __m128 extent_z = _mm_broadcast_ss( &aabb->ez );
		const __m128 center_x = _mm_broadcast_ss( &aabb->cx );
		const __m128 center_y = _mm_broadcast_ss( &aabb->cy );
		const __m128 center_z = _mm_broadcast_ss( &aabb->cz );

		for( size_t pidx = 0; pidx < plane_count_4; pidx++ )
		{
			const PlaneBlock *plane = &planes[pidx];

			__m128 dot = plane->w;

			/*
			const __m128 e_x = _mm_xor_ps( extent_x, _mm_and_ps( plane->x, sign_mask ) );
			const __m128 t_x = _mm_add_ps( center_x, e_x );
			dot = _mm_add_ps( _mm_mul_ps( t_x, plane->x ), dot );
			const __m128 e_y = _mm_xor_ps( extent_y, _mm_and_ps( plane->y, sign_mask ) );
			const __m128 t_y = _mm_add_ps( center_y, e_y );
			dot = _mm_add_ps( _mm_mul_ps( t_y, plane->y ), dot );
			const __m128 e_z = _mm_xor_ps( extent_z, _mm_and_ps( plane->z, sign_mask ) );
			const __m128 t_z = _mm_add_ps( center_z, e_z );
			dot = _mm_add_ps( _mm_mul_ps( t_z, plane->z ), dot );
			*/

			const __m128 e_x = (__m128i)extent_x ^ ( (__m128i)plane->x & sign_mask );
			const __m128 t_x = center_x + e_x;
			dot = ( t_x * plane->x ) + dot;
			const __m128 e_y = (__m128i)extent_y ^ ( (__m128i)plane->y & sign_mask );
			const __m128 t_y = center_y + e_y;
			dot = ( t_y * plane->y ) + dot;
			const __m128 e_z = (__m128i)extent_z ^ ( (__m128i)plane->z & sign_mask );
			const __m128 t_z = center_z + e_z;
			dot = ( t_z * plane->z ) + dot;


			if( _mm_movemask_ps( dot ) != 0 )
			{
				results[idx] = 0;
				break;
			}
		}
	}

	free( planes );
}

