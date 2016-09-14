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

void contains_bulk_c_simd( const Plane *planes, size_t plane_count, const AABB *aabbs, size_t aabb_count, _Bool *results )
{
	size_t plane_count_4 = ( plane_count + 3 ) & ~3;
	float *planes_data = (float *)malloc( sizeof( float ) * plane_count_4 * 4 );
	float *planes_x = &planes_data[plane_count_4 * 0];
	float *planes_y = &planes_data[plane_count_4 * 1];
	float *planes_z = &planes_data[plane_count_4 * 2];
	float *planes_w = &planes_data[plane_count_4 * 3];

	unsigned int sm[] = { 0x80000000, 0x80000000, 0x80000000, 0x80000000 };

	const __m128 sign_mask = _mm_loadu_si128( &sm );

	for( size_t p = 0; p < plane_count_4; p++ )
	{
		if( p < plane_count_4 )
		{
			planes_x[p] = planes[p].x;
			planes_y[p] = planes[p].y;
			planes_z[p] = planes[p].z;
			planes_w[p] = planes[p].w;
		}
		else
		{
			planes_x[p] = planes[0].x;
			planes_y[p] = planes[0].y;
			planes_z[p] = planes[0].z;
			planes_w[p] = planes[0].w;
		}
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

		for( size_t pidx = 0; pidx < plane_count_4; pidx+=4 )
		{
			const __m128 plane_x = _mm_loadu_ps( &planes_x[pidx] );
			const __m128 plane_y = _mm_loadu_ps( &planes_y[pidx] );
			const __m128 plane_z = _mm_loadu_ps( &planes_z[pidx] );
			
			__m128 dot = _mm_loadu_ps( &planes_w[pidx] );

			const __m128 e_x = _mm_xor_ps( extent_x, _mm_and_ps( plane_x, sign_mask ) );
			const __m128 t_x = _mm_add_ps( center_x, e_x );
			dot = _mm_add_ps( dot, _mm_mul_ps( t_x, plane_x ) );
			const __m128 e_y = _mm_xor_ps( extent_y, _mm_and_ps( plane_y, sign_mask ) );
			const __m128 t_y = _mm_add_ps( center_y, e_y );
			dot = _mm_add_ps( dot, _mm_mul_ps( t_y, plane_y ) );
			const __m128 e_z = _mm_xor_ps( extent_z, _mm_and_ps( plane_z, sign_mask ) );
			const __m128 t_z = _mm_add_ps( center_z, e_z );
			dot = _mm_add_ps( dot, _mm_mul_ps( t_z, plane_z ) );

			if( _mm_movemask_ps( dot ) != 0 )
			{
				results[idx] = 0;
				break;
			}
		}
	}

	free( planes_data );
}

