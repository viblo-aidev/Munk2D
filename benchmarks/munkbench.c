/* Copyright (c) 2026 Victor Blomqvist
 * Copyright (c) 2007-2024 Scott Lembcke and Howling Moon Software
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "chipmunk/chipmunk.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef MUNK2D_VERSION
#define MUNK2D_VERSION "unknown"
#endif

#define FPS 60.0

#ifndef CP_PI
#define CP_PI 3.14159265358979323846264338327950288
#endif

typedef struct Benchmark Benchmark;

typedef cpSpace *(*BenchmarkInitFunc)(int size, void **state);
typedef void (*BenchmarkUpdateFunc)(cpSpace *space, void *state, cpFloat dt);
typedef void (*BenchmarkDestroyStateFunc)(void *state);

struct Benchmark {
	const char *name;
	int steps;
	int default_size;
	int size_start;
	int size_end;
	int size_inc;
	BenchmarkInitFunc init;
	BenchmarkUpdateFunc update;
	BenchmarkDestroyStateFunc destroy_state;
};

typedef struct TumblerState {
	int count;
	int target_count;
} TumblerState;

typedef struct RunResult {
	const char *benchmark;
	int size;
	double init_time;
	double run_time;
} RunResult;

static double
now_seconds(void)
{
	return (double)clock()/(double)CLOCKS_PER_SEC;
}

static void
usage(const char *argv0)
{
	printf("MunkBench - Munk2D benchmark suite\n\n");
	printf("Usage: %s [-b benchmark ...] [-s size] [--summary-json] [--svg file --step n]\n\n", argv0);
	printf("Options:\n");
	printf("  -b, --benchmarks  Run only the named benchmarks.\n");
	printf("  -s, --size        Size to run. Omit for each benchmark default; use -1 for size sweep.\n");
	printf("  --summary-json    Emit validation checkpoint summaries as JSON instead of timing CSV.\n");
	printf("  --checkpoints     Comma-separated step checkpoints for --summary-json. Default: 0,1,10,100,final.\n");
	printf("  --svg file        Write an SVG snapshot for a single benchmark. Use '-' for stdout.\n");
	printf("  --step n          Step to render for --svg. Default: benchmark final step.\n");
	printf("  -h, --help        Show this help.\n");
}

static cpSpace *
new_space(cpFloat gx, cpFloat gy)
{
	cpSpace *space = cpSpaceNew();
	cpSpaceSetGravity(space, cpv(gx, gy));
	return space;
}

static cpBody *
add_dynamic_body(cpSpace *space, cpVect position)
{
	(void)space;
	cpBody *body = cpBodyNew(0.0, 0.0);
	cpBodySetPosition(body, position);
	return body;
}

static cpBody *
add_static_body(cpSpace *space, cpVect position)
{
	cpBody *body = cpSpaceAddBody(space, cpBodyNewStatic());
	cpBodySetPosition(body, position);
	return body;
}

static cpShape *
add_shape_with_density(cpSpace *space, cpShape *shape, cpFloat density)
{
	cpBody *body = cpShapeGetBody(shape);

	if(density > 0.0){
		cpFloat old_mass = cpBodyGetMass(body);
		cpFloat old_moment = cpBodyGetMoment(body);
		cpVect old_cog = cpBodyGetCenterOfGravity(body);

		cpShapeSetDensity(shape, density);
		cpFloat shape_mass = cpShapeGetMass(shape);
		cpFloat shape_moment = cpShapeGetMoment(shape);
		cpVect shape_cog = cpShapeGetCenterOfGravity(shape);

		cpShapeSetMass(shape, 0.0);

		if(shape_mass > 0.0){
			cpFloat new_mass = old_mass + shape_mass;
			cpVect new_cog = (old_mass > 0.0 ? cpvlerp(old_cog, shape_cog, shape_mass/new_mass) : shape_cog);
			cpFloat new_moment = shape_moment;
			if(old_mass > 0.0){
				new_moment += old_moment + cpvdistsq(old_cog, shape_cog)*(shape_mass*old_mass)/new_mass;
			}

			cpVect position = cpBodyGetPosition(body);
			cpBodySetCenterOfGravity(body, new_cog);
			cpBodySetMass(body, new_mass);
			cpBodySetMoment(body, new_moment);
			cpBodySetPosition(body, position);
		}
	}

	if(cpBodyGetSpace(body) == NULL) cpSpaceAddBody(space, body);
	return cpSpaceAddShape(space, shape);
}

static cpShape *
add_box_shape(cpSpace *space, cpBody *body, cpFloat width, cpFloat height, cpFloat density)
{
	return add_shape_with_density(space, cpBoxShapeNew(body, width, height, 0.0), density);
}

static cpShape *
add_box_shape_bb(cpSpace *space, cpBody *body, cpFloat l, cpFloat b, cpFloat r, cpFloat t, cpFloat density)
{
	cpFloat left = cpfmin(l, r);
	cpFloat right = cpfmax(l, r);
	cpFloat bottom = cpfmin(b, t);
	cpFloat top = cpfmax(b, t);
	return add_shape_with_density(space, cpBoxShapeNew2(body, cpBBNew(left, bottom, right, top), 0.0), density);
}

static cpShape *
add_circle_shape(cpSpace *space, cpBody *body, cpFloat radius, cpVect offset, cpFloat density)
{
	return add_shape_with_density(space, cpCircleShapeNew(body, radius, offset), density);
}

static cpShape *
add_poly_shape(cpSpace *space, cpBody *body, int count, const cpVect *verts, cpFloat density)
{
	return add_shape_with_density(space, cpPolyShapeNew(body, count, verts, cpTransformIdentity, 0.0), density);
}

static cpShape *
add_static_segment(cpSpace *space, cpVect a, cpVect b, cpFloat radius)
{
	return cpSpaceAddShape(space, cpSegmentShapeNew(cpSpaceGetStaticBody(space), a, b, radius));
}

static void
shape_free_wrap(cpSpace *space, cpShape *shape, void *unused)
{
	(void)unused;
	cpSpaceRemoveShape(space, shape);
	cpShapeFree(shape);
}

static void
post_shape_free(cpShape *shape, cpSpace *space)
{
	cpSpaceAddPostStepCallback(space, (cpPostStepFunc)shape_free_wrap, shape, NULL);
}

static void
constraint_free_wrap(cpSpace *space, cpConstraint *constraint, void *unused)
{
	(void)unused;
	cpSpaceRemoveConstraint(space, constraint);
	cpConstraintFree(constraint);
}

static void
post_constraint_free(cpConstraint *constraint, cpSpace *space)
{
	cpSpaceAddPostStepCallback(space, (cpPostStepFunc)constraint_free_wrap, constraint, NULL);
}

static void
body_free_wrap(cpSpace *space, cpBody *body, void *unused)
{
	(void)unused;
	cpSpaceRemoveBody(space, body);
	cpBodyFree(body);
}

static void
post_body_free(cpBody *body, cpSpace *space)
{
	cpSpaceAddPostStepCallback(space, (cpPostStepFunc)body_free_wrap, body, NULL);
}

static void
free_space_children(cpSpace *space)
{
	cpSpaceEachShape(space, (cpSpaceShapeIteratorFunc)post_shape_free, space);
	cpSpaceEachConstraint(space, (cpSpaceConstraintIteratorFunc)post_constraint_free, space);
	cpSpaceEachBody(space, (cpSpaceBodyIteratorFunc)post_body_free, space);
}

static void
default_update(cpSpace *space, void *state, cpFloat dt)
{
	(void)state;
	cpSpaceStep(space, dt);
}

static void
add_pair_update(cpSpace *space, void *state, cpFloat dt)
{
	(void)state;
	for(int i = 0; i < 4; i++) cpSpaceStep(space, dt/4.0);
}

static cpSpace *
init_falling_squares(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, -10.0);
	cpBodySetPosition(cpSpaceGetStaticBody(space), cpv(0.0, -10.0));
	cpSpaceAddShape(space, cpBoxShapeNew(cpSpaceGetStaticBody(space), 100.0*2.0, 3.0*2.0, 0.0));

	for(int i = 0; i < 15; i++){
		cpFloat a = 0.5 + (cpFloat)i/15.0*2.5;
		for(int j = 0; j < size; j++){
			cpBody *body = add_dynamic_body(space, cpv(i*7.0 - 30.0, 2.0*a*(size - j)));
			add_box_shape(space, body, a*2.0, a*2.0, 5.0);
		}
	}

	return space;
}

static cpSpace *
init_falling_circles(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, -10.0);
	cpBodySetPosition(cpSpaceGetStaticBody(space), cpv(0.0, -10.0));
	cpSpaceAddShape(space, cpBoxShapeNew(cpSpaceGetStaticBody(space), 100.0*2.0, 3.0*2.0, 0.0));

	for(int i = 0; i < 15; i++){
		cpFloat a = 0.5 + (cpFloat)i/15.0*2.5;
		for(int j = 0; j < size; j++){
			cpBody *body = add_dynamic_body(space, cpv(i*7.0 + j*0.25 - 100.0, 2.0*a*(size - j)));
			add_circle_shape(space, body, a/2.0, cpvzero, 5.0);
		}
	}

	return space;
}

static void
tumbler_update(cpSpace *space, void *state, cpFloat dt)
{
	TumblerState *tumbler = (TumblerState *)state;
	cpSpaceStep(space, dt);

	if(tumbler->count < tumbler->target_count){
		cpBody *body = add_dynamic_body(space, cpv(0.0, 10.0));
		add_box_shape(space, body, 0.125*2.0, 0.125*2.0, 1.0);
		tumbler->count++;
	}
}

static void
tumbler_destroy_state(void *state)
{
	free(state);
}

static cpSpace *
init_tumbler(int size, void **state)
{
	cpSpace *space = new_space(0.0, -10.0);

	TumblerState *tumbler = (TumblerState *)calloc(1, sizeof(TumblerState));
	if(!tumbler){
		fprintf(stderr, "Out of memory.\n");
		exit(1);
	}
	tumbler->target_count = size;
	*state = tumbler;

	cpBody *body = add_dynamic_body(space, cpv(0.0, 10.0));
	add_shape_with_density(space, cpSegmentShapeNew(body, cpv(-10.0, 10.0), cpv(10.0, 10.0), 0.5), 5.0);
	add_shape_with_density(space, cpSegmentShapeNew(body, cpv(10.0, 10.0), cpv(10.0, -10.0), 0.5), 5.0);
	add_shape_with_density(space, cpSegmentShapeNew(body, cpv(10.0, -10.0), cpv(-10.0, -10.0), 0.5), 5.0);
	add_shape_with_density(space, cpSegmentShapeNew(body, cpv(-10.0, -10.0), cpv(-10.0, 10.0), 0.5), 5.0);

	cpSpaceAddConstraint(space, cpSimpleMotorNew(cpSpaceGetStaticBody(space), body, 0.05*CP_PI));
	cpSpaceAddConstraint(space, cpPinJointNew(cpSpaceGetStaticBody(space), body, cpv(0.0, 10.0), cpv(0.0, 0.0)));

	return space;
}

static cpSpace *
init_add_pair(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, 0.0);
	cpFloat min_x = -9.0/10.0;
	cpFloat max_x = 9.0/10.0;
	cpFloat min_y = 4.0/10.0;
	cpFloat max_y = 6.0/10.0;

	for(int i = 0; i < size; i++){
		cpBody *body = add_dynamic_body(space, cpv(
			min_x + (max_x - min_x)*(cpFloat)i/(cpFloat)size,
			min_y + (max_y - min_y)*(cpFloat)(i % 32)/32.0
		));
		add_circle_shape(space, body, 0.1/10.0, cpvzero, 0.01/10.0);
	}

	cpBody *body = add_dynamic_body(space, cpv(-40.0/10.0, 5.0/10.0));
	add_box_shape(space, body, 1.5*2.0/10.0, 1.5*2.0/10.0, 1.0/10.0);
	cpBodySetVelocity(body, cpv(175.0/10.0, 0.0));

	return space;
}

static cpSpace *
init_mild_n2(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, -10.0);
	add_static_segment(space, cpv(-50.0, 0.0), cpv(50.0, 0.0), 1.0);

	for(int i = 0; i < size; i++){
		cpBody *body = add_dynamic_body(space, cpv(-5.0, -1.0));
		add_box_shape_bb(space, body, -3.0, 3.0, 3.0, 4.0, 2.0);
		add_box_shape_bb(space, body, -3.0, 4.0, -2.0, 0.0, 2.0);
		add_box_shape_bb(space, body, 2.0, 4.0, 3.0, 0.0, 0.0);
	}

	for(int i = 0; i < size; i++){
		cpBody *body = add_dynamic_body(space, cpv(15.0, 1.0));
		cpVect left[] = {cpv(-2.0, 0.0), cpv(1.0, 2.0), cpv(0.0, 4.0)};
		cpVect right[] = {cpv(2.0, 0.0), cpv(-1.0, 2.0), cpv(0.0, 4.0)};
		add_poly_shape(space, body, 3, left, 2.0);
		add_poly_shape(space, body, 3, right, 2.0);
	}

	return space;
}

static cpSpace *
init_n2(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, -10.0);
	for(int i = 0; i < size; i++){
		cpBody *body = add_dynamic_body(space, cpv(i*0.01, -i*0.01));
		add_circle_shape(space, body, 1.0, cpvzero, 1.0);
	}
	return space;
}

static cpSpace *
init_multifixture(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, -10.0);
	add_static_segment(space, cpv(-35.0, 0.0), cpv(35.0, 0.0), 1.0);
	add_static_segment(space, cpv(-36.0, 50.0), cpv(-36.0, 0.0), 1.0);
	add_static_segment(space, cpv(36.0, 50.0), cpv(36.0, 0.0), 1.0);

	for(int i = 0; i < size; i++){
		cpBody *body = add_dynamic_body(space, cpv(-20.0 + (i % 6)*7.0 + (cpFloat)i/10.0, 1.0 + ((cpFloat)i/6.0)*5.0));
		int c = 50 - i/2;
		for(int z = 0; z < c; z++){
			cpFloat bs = 2.0/(cpFloat)c;
			cpFloat ps = 2.0*z*bs + bs;
			add_box_shape_bb(space, body, -bs + ps - 2.0, 3.0, bs + ps - 2.0, 4.0, 2.0/(cpFloat)c);
			add_box_shape_bb(space, body, -2.0, bs + ps, -1.0, -bs + ps, 2.0/(cpFloat)c);
			add_box_shape_bb(space, body, 1.0, bs + ps, 2.0, -bs + ps, 0.0);
		}
	}

	return space;
}

static cpSpace *
init_mostly_static_single_body(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, -10.0);
	cpFloat a = 0.5;
	cpBodySetPosition(cpSpaceGetStaticBody(space), cpv(0.0, -a));
	int n = size;
	int m = size;
	cpVect position = cpv(0.0, 0.0);

	for(int j = 0; j < m; j++){
		position = cpv(-n*a, position.y);
		for(int i = 0; i < n; i++){
			if(abs(j - i) > 3){
				cpSpaceAddShape(space, cpBoxShapeNew2(cpSpaceGetStaticBody(space), cpBBNew(position.x - a, position.y - a, position.x + a, position.y + a), 0.0));
			} else if(i == j){
				cpBody *body = add_dynamic_body(space, position);
				add_circle_shape(space, body, a*2.0, cpvzero, 1.0);
			}
			position = cpvadd(position, cpv(2.0*a, 0.0));
		}
		position = cpvsub(position, cpv(0.0, 2.0*a));
	}

	return space;
}

static cpSpace *
init_mostly_static_multi_body(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, -10.0);
	cpFloat a = 0.5;
	int n = size;
	int m = size;
	cpVect position = cpv(0.0, 0.0);

	for(int j = 0; j < m; j++){
		position = cpv(-n*a, position.y);
		for(int i = 0; i < n; i++){
			if(abs(j - i) > 3){
				cpBody *body = add_static_body(space, position);
				cpSpaceAddShape(space, cpBoxShapeNew(body, a*2.0, a*2.0, 0.0));
			} else if(i == j){
				cpBody *body = add_dynamic_body(space, position);
				add_circle_shape(space, body, a*2.0, cpvzero, 1.0);
			}
			position = cpvadd(position, cpv(2.0*a, 0.0));
		}
		position = cpvsub(position, cpv(0.0, 2.0*a));
	}

	return space;
}

static cpSpace *
init_diagonal(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, -10.0);
	cpFloat a = 0.5;
	int n = size;
	cpFloat m = (cpFloat)size/2.0;
	cpVect position = cpv(0.0, 0.0);

	for(int j = 0; j < (int)m; j++){
		position = cpv(-n*a, position.y);
		for(int i = 0; i < n; i++){
			cpBody *body = add_static_body(space, position);
			cpBodySetAngle(body, CP_PI/4.0);
			cpSpaceAddShape(space, cpBoxShapeNew(body, a, (3.0*j + 1.0)*a, 0.0));
			position = cpvadd(position, cpv(8.0*a, 0.0));
		}
		position = cpvsub(position, cpv(0.0, 8.0*a));
	}

	for(int i = 0; i < 3000; i++){
		cpBody *body = add_dynamic_body(space, cpv(((cpFloat)i/15.0)*2.0 - 75.0, (cpFloat)(i % 15)*2.0 + 50.0));
		add_circle_shape(space, body, 0.5, cpvzero, 1.0);
	}

	return space;
}

static cpSpace *
init_mixed_static_dynamic(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, -10.0);
	int n = 150;
	int m = 150;
	cpVect cntr = cpv((cpFloat)m/2.0, (cpFloat)n/2.0);
	cpFloat a = 0.5;

	for(int j = 0; j < m; j++){
		for(int i = 0; i < n; i++){
			cpVect pos = cpv((cpFloat)i, (cpFloat)j);
			cpVect delta = cpvsub(pos, cntr);
			if(cpvdot(delta, delta) > 67.0*67.0){
				cpSpaceAddShape(space, cpCircleShapeNew(cpSpaceGetStaticBody(space), a, pos));
			}
		}
	}

	for(int i = 0; i < size; i++){
		cpFloat s = (cpFloat)i/(cpFloat)size;
		cpVect pos = cpv(cos(s*30.0)*(s*50.0 + 10.0), sin(s*30.0)*(s*50.0 + 10.0));
		cpBody *body = add_dynamic_body(space, cpvadd(pos, cntr));
		cpBodySetVelocity(body, pos);
		add_circle_shape(space, body, a, cpvzero, 0.5);
	}

	return space;
}

static cpBody *
big_mobile_add_node(cpSpace *space, cpBody *parent, cpVect local_anchor, int depth, int max_depth, cpFloat offset, cpFloat a)
{
	cpFloat density = 20.0;
	cpVect h = cpv(0.0, a);
	cpVect p = cpvsub(cpvadd(cpBodyGetPosition(parent), local_anchor), h);
	cpBody *body = add_dynamic_body(space, p);
	add_box_shape(space, body, 0.25*a*2.0, a*2.0, density + p.x*0.02);

	if(depth == max_depth) return body;

	add_box_shape_bb(space, body, -offset, -0.25*a - a, offset, 0.25*a - a, density);

	cpVect a1 = cpv(offset, -a);
	cpVect a2 = cpv(-offset, -a);
	cpBody *body1 = big_mobile_add_node(space, body, a1, depth + 1, max_depth, 0.5*offset, a);
	cpBody *body2 = big_mobile_add_node(space, body, a2, depth + 1, max_depth, 0.5*offset, a);

	cpConstraint *j1 = cpSpaceAddConstraint(space, cpPinJointNew(body, body1, a1, h));
	cpConstraint *j2 = cpSpaceAddConstraint(space, cpPinJointNew(body, body2, a2, h));
	cpConstraintSetCollideBodies(j1, cpFalse);
	cpConstraintSetCollideBodies(j2, cpFalse);

	return body;
}

static cpSpace *
init_big_mobile(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, -10.0);
	cpBodySetPosition(cpSpaceGetStaticBody(space), cpv(0.0, 20.0));
	cpFloat a = 0.25;
	cpVect h = cpv(0.0, a);
	cpBody *root = big_mobile_add_node(space, cpSpaceGetStaticBody(space), cpvzero, 0, size, 200.0, a);
	cpSpaceAddConstraint(space, cpPinJointNew(cpSpaceGetStaticBody(space), root, cpvzero, h));
	return space;
}

static cpSpace *
init_slow_explosion(int size, void **state)
{
	(void)state;
	cpSpace *space = new_space(0.0, 0.0);
	for(int i = 0; i < size; i++){
		cpFloat s = (cpFloat)i*30.0/(cpFloat)size;
		cpVect pos = cpv(cos(s*30.0)*(s*30.0 + 5.0), sin(s*30.0)*(s*30.0 + 5.0));
		cpBody *body = add_dynamic_body(space, pos);
		cpBodySetVelocity(body, cpvmult(cpBodyGetPosition(body), 0.2));
		add_circle_shape(space, body, 0.5, cpvzero, 0.5);
	}
	return space;
}

static Benchmark benchmarks[] = {
	{"FallingSquares", 1300, 300, 10, 300, 10, init_falling_squares, default_update, NULL},
	{"FallingCircles", 1300, 300, 10, 300, 10, init_falling_circles, default_update, NULL},
	{"Tumbler", 1500, 1000, 50, 1000, 50, init_tumbler, tumbler_update, tumbler_destroy_state},
	{"AddPair", 1000, 2000, 100, 2500, 100, init_add_pair, add_pair_update, NULL},
	{"MildN2", 100, 200, 10, 200, 10, init_mild_n2, default_update, NULL},
	{"N2", 100, 750, 25, 750, 25, init_n2, default_update, NULL},
	{"Multifixture", 500, 100, 5, 100, 5, init_multifixture, default_update, NULL},
	{"MostlyStaticSingleBody", 400, 200, 10, 200, 5, init_mostly_static_single_body, default_update, NULL},
	{"MostlyStaticMultiBody", 400, 200, 10, 200, 5, init_mostly_static_multi_body, default_update, NULL},
	{"Diagonal", 1000, 50, 2, 50, 2, init_diagonal, default_update, NULL},
	{"MixedStaticDynamic", 400, 6000, 100, 6000, 100, init_mixed_static_dynamic, default_update, NULL},
	{"BigMobile", 1000, 11, 1, 11, 1, init_big_mobile, default_update, NULL},
	{"SlowExplosion", 1000, 6000, 100, 6000, 100, init_slow_explosion, default_update, NULL},
};

static int benchmark_count = (int)(sizeof(benchmarks)/sizeof(benchmarks[0]));

static Benchmark *
find_benchmark(const char *name)
{
	for(int i = 0; i < benchmark_count; i++){
		if(strcmp(benchmarks[i].name, name) == 0) return &benchmarks[i];
	}
	return NULL;
}


typedef struct Summary {
	int step;
	int bodies;
	int dynamic_bodies;
	int static_bodies;
	int kinematic_bodies;
	int sleeping_bodies;
	int shapes;
	int constraints;
	int invalid_values;
	cpFloat total_mass;
	cpFloat total_kinetic_energy;
	cpFloat max_linear_velocity;
	cpFloat max_angular_velocity;
	cpFloat sum_position_x;
	cpFloat sum_position_y;
	cpFloat sum_velocity_x;
	cpFloat sum_velocity_y;
	cpBB body_bounds;
	cpBB shape_bounds;
	cpBool has_body_bounds;
	cpBool has_shape_bounds;
} Summary;

static int
is_bad(cpFloat value)
{
	return isnan((double)value) || isinf((double)value);
}

static void
summary_add_bad_vect(Summary *summary, cpVect v)
{
	if(is_bad(v.x)) summary->invalid_values++;
	if(is_bad(v.y)) summary->invalid_values++;
}

static void
summary_body(cpBody *body, void *data)
{
	Summary *summary = (Summary *)data;
	cpBodyType type = cpBodyGetType(body);
	cpVect p = cpBodyGetPosition(body);
	cpVect v = cpBodyGetVelocity(body);
	cpFloat w = cpBodyGetAngularVelocity(body);
	cpFloat mass = cpBodyGetMass(body);
	cpFloat moment = cpBodyGetMoment(body);
	cpFloat speed = cpvlength(v);
	cpFloat angular_speed = cpfabs(w);

	summary->bodies++;
	if(type == CP_BODY_TYPE_DYNAMIC){
		summary->dynamic_bodies++;
		summary->sum_position_x += p.x;
		summary->sum_position_y += p.y;
		summary->sum_velocity_x += v.x;
		summary->sum_velocity_y += v.y;
		summary->total_mass += mass;
		if(!isinf((double)mass)) summary->total_kinetic_energy += 0.5*mass*cpvdot(v, v);
		if(!isinf((double)moment)) summary->total_kinetic_energy += 0.5*moment*w*w;
		if(speed > summary->max_linear_velocity) summary->max_linear_velocity = speed;
		if(angular_speed > summary->max_angular_velocity) summary->max_angular_velocity = angular_speed;

		cpBB bb = cpBBNewForExtents(p, 0.0, 0.0);
		summary->body_bounds = summary->has_body_bounds ? cpBBMerge(summary->body_bounds, bb) : bb;
		summary->has_body_bounds = cpTrue;
	} else if(type == CP_BODY_TYPE_STATIC){
		summary->static_bodies++;
	} else if(type == CP_BODY_TYPE_KINEMATIC){
		summary->kinematic_bodies++;
	}

	if(cpBodyIsSleeping(body)) summary->sleeping_bodies++;
	if(is_bad(mass)) summary->invalid_values++;
	if(is_bad(moment)) summary->invalid_values++;
	if(is_bad(w)) summary->invalid_values++;
	summary_add_bad_vect(summary, p);
	summary_add_bad_vect(summary, v);
}

static void
summary_shape(cpShape *shape, void *data)
{
	Summary *summary = (Summary *)data;
	cpBB bb = cpShapeGetBB(shape);
	summary->shapes++;
	summary->shape_bounds = summary->has_shape_bounds ? cpBBMerge(summary->shape_bounds, bb) : bb;
	summary->has_shape_bounds = cpTrue;
	if(is_bad(bb.l)) summary->invalid_values++;
	if(is_bad(bb.b)) summary->invalid_values++;
	if(is_bad(bb.r)) summary->invalid_values++;
	if(is_bad(bb.t)) summary->invalid_values++;
}

static void
summary_constraint(cpConstraint *constraint, void *data)
{
	(void)constraint;
	Summary *summary = (Summary *)data;
	summary->constraints++;
}

static Summary
collect_summary(cpSpace *space, int step)
{
	Summary summary;
	memset(&summary, 0, sizeof(summary));
	summary.step = step;
	cpSpaceEachBody(space, summary_body, &summary);
	cpSpaceEachShape(space, summary_shape, &summary);
	cpSpaceEachConstraint(space, summary_constraint, &summary);
	return summary;
}


static void
print_number_json(cpFloat value)
{
	if(is_bad(value)){
		printf("null");
	} else {
		printf("%.17g", (double)value);
	}
}

static void
print_bb_json(cpBB bb, cpBool valid)
{
	if(valid){
		printf("[");
		print_number_json(bb.l);
		printf(",");
		print_number_json(bb.b);
		printf(",");
		print_number_json(bb.r);
		printf(",");
		print_number_json(bb.t);
		printf("]");
	} else {
		printf("null");
	}
}

static void
print_summary_json(const Summary *summary)
{
	printf("{\"step\":%d", summary->step);
	printf(",\"bodies\":%d", summary->bodies);
	printf(",\"dynamic_bodies\":%d", summary->dynamic_bodies);
	printf(",\"static_bodies\":%d", summary->static_bodies);
	printf(",\"kinematic_bodies\":%d", summary->kinematic_bodies);
	printf(",\"sleeping_bodies\":%d", summary->sleeping_bodies);
	printf(",\"shapes\":%d", summary->shapes);
	printf(",\"constraints\":%d", summary->constraints);
	printf(",\"invalid_values\":%d", summary->invalid_values);
	printf(",\"total_mass\":");
	print_number_json(summary->total_mass);
	printf(",\"total_kinetic_energy\":");
	print_number_json(summary->total_kinetic_energy);
	printf(",\"max_linear_velocity\":");
	print_number_json(summary->max_linear_velocity);
	printf(",\"max_angular_velocity\":");
	print_number_json(summary->max_angular_velocity);
	printf(",\"sum_position\":[");
	print_number_json(summary->sum_position_x);
	printf(",");
	print_number_json(summary->sum_position_y);
	printf("]");
	printf(",\"sum_velocity\":[");
	print_number_json(summary->sum_velocity_x);
	printf(",");
	print_number_json(summary->sum_velocity_y);
	printf("]");
	printf(",\"body_bounds\":");
	print_bb_json(summary->body_bounds, summary->has_body_bounds);
	printf(",\"shape_bounds\":");
	print_bb_json(summary->shape_bounds, summary->has_shape_bounds);
	printf("}");
}

static int
compare_ints(const void *a, const void *b)
{
	int ia = *(const int *)a;
	int ib = *(const int *)b;
	return (ia > ib) - (ia < ib);
}

static int
parse_checkpoints(const char *arg, int *checkpoints, int max_checkpoints)
{
	if(!arg) return 0;
	char *copy = (char *)malloc(strlen(arg) + 1);
	if(!copy){
		fprintf(stderr, "Out of memory.\n");
		exit(1);
	}
	strcpy(copy, arg);

	int count = 0;
	for(char *token = strtok(copy, ","); token; token = strtok(NULL, ",")){
		if(count >= max_checkpoints){
			fprintf(stderr, "Too many checkpoints; max is %d.\n", max_checkpoints);
			free(copy);
			exit(2);
		}
		checkpoints[count++] = atoi(token);
	}
	free(copy);
	return count;
}

static int
prepare_checkpoints(const Benchmark *benchmark, int *checkpoints, int checkpoint_count, int max_checkpoints)
{
	if(checkpoint_count == 0){
		int defaults[] = {0, 1, 10, 100, benchmark->steps};
		for(size_t i = 0; i < sizeof(defaults)/sizeof(defaults[0]); i++){
			if(defaults[i] <= benchmark->steps) checkpoints[checkpoint_count++] = defaults[i];
		}
	} else {
		for(int i = 0; i < checkpoint_count; i++){
			if(checkpoints[i] < 0) checkpoints[i] = 0;
			if(checkpoints[i] > benchmark->steps) checkpoints[i] = benchmark->steps;
		}
	}

	qsort(checkpoints, (size_t)checkpoint_count, sizeof(int), compare_ints);
	int out = 0;
	for(int i = 0; i < checkpoint_count; i++){
		if(out == 0 || checkpoints[i] != checkpoints[out - 1]) checkpoints[out++] = checkpoints[i];
	}
	(void)max_checkpoints;
	return out;
}

static void
run_summary_json(Benchmark *benchmark, int size, const int *input_checkpoints, int input_checkpoint_count, int max_checkpoints)
{
	int *checkpoints = (int *)calloc((size_t)max_checkpoints, sizeof(int));
	if(!checkpoints){
		fprintf(stderr, "Out of memory.\n");
		exit(1);
	}
	for(int i = 0; i < input_checkpoint_count; i++) checkpoints[i] = input_checkpoints[i];
	int checkpoint_count = prepare_checkpoints(benchmark, checkpoints, input_checkpoint_count, max_checkpoints);
	int checkpoint_index = 0;
	void *state = NULL;

	cpSpace *space = benchmark->init(size, &state);

	printf("{\"benchmark\":\"%s\",\"size\":%d,\"steps\":%d,\"checkpoints\":[", benchmark->name, size, benchmark->steps);
	if(checkpoint_index < checkpoint_count && checkpoints[checkpoint_index] == 0){
		Summary summary = collect_summary(space, 0);
		print_summary_json(&summary);
		checkpoint_index++;
	}

	for(int step = 1; step <= benchmark->steps; step++){
		benchmark->update(space, state, (cpFloat)(1.0/FPS));
		if(checkpoint_index < checkpoint_count && checkpoints[checkpoint_index] == step){
			if(checkpoint_index > 0) printf(",");
			Summary summary = collect_summary(space, step);
			print_summary_json(&summary);
			checkpoint_index++;
		}
	}
	printf("]}");

	free_space_children(space);
	cpSpaceFree(space);
	if(benchmark->destroy_state) benchmark->destroy_state(state);
	free(checkpoints);
}

typedef struct SvgContext {
	FILE *file;
} SvgContext;

static void
svg_color(char *buffer, size_t size, cpSpaceDebugColor color)
{
	int r = (int)(cpfclamp(color.r, 0.0, 1.0)*255.0);
	int g = (int)(cpfclamp(color.g, 0.0, 1.0)*255.0);
	int b = (int)(cpfclamp(color.b, 0.0, 1.0)*255.0);
	snprintf(buffer, size, "#%02x%02x%02x", r, g, b);
}

static cpSpaceDebugColor
svg_color_for_shape(cpShape *shape, cpDataPointer data)
{
	(void)shape;
	(void)data;
	cpSpaceDebugColor color = {0.35f, 0.55f, 0.95f, 0.55f};
	return color;
}

static void
svg_draw_circle(cpVect pos, cpFloat angle, cpFloat radius, cpSpaceDebugColor outline, cpSpaceDebugColor fill, cpDataPointer data)
{
	SvgContext *ctx = (SvgContext *)data;
	char stroke[16], fill_color[16];
	svg_color(stroke, sizeof(stroke), outline);
	svg_color(fill_color, sizeof(fill_color), fill);
	fprintf(ctx->file, "  <circle cx=\"%.17g\" cy=\"%.17g\" r=\"%.17g\" fill=\"%s\" fill-opacity=\"%.3g\" stroke=\"%s\" stroke-width=\"0.03\"/>\n",
		(double)pos.x, (double)-pos.y, (double)radius, fill_color, (double)fill.a, stroke);
	fprintf(ctx->file, "  <line x1=\"%.17g\" y1=\"%.17g\" x2=\"%.17g\" y2=\"%.17g\" stroke=\"%s\" stroke-width=\"0.02\"/>\n",
		(double)pos.x, (double)-pos.y, (double)(pos.x + cos(angle)*radius), (double)-(pos.y + sin(angle)*radius), stroke);
}

static void
svg_draw_segment(cpVect a, cpVect b, cpSpaceDebugColor color, cpDataPointer data)
{
	SvgContext *ctx = (SvgContext *)data;
	char stroke[16];
	svg_color(stroke, sizeof(stroke), color);
	fprintf(ctx->file, "  <line x1=\"%.17g\" y1=\"%.17g\" x2=\"%.17g\" y2=\"%.17g\" stroke=\"%s\" stroke-width=\"0.05\" stroke-linecap=\"round\"/>\n",
		(double)a.x, (double)-a.y, (double)b.x, (double)-b.y, stroke);
}

static void
svg_draw_fat_segment(cpVect a, cpVect b, cpFloat radius, cpSpaceDebugColor outline, cpSpaceDebugColor fill, cpDataPointer data)
{
	SvgContext *ctx = (SvgContext *)data;
	char stroke[16];
	(void)outline;
	svg_color(stroke, sizeof(stroke), fill);
	fprintf(ctx->file, "  <line x1=\"%.17g\" y1=\"%.17g\" x2=\"%.17g\" y2=\"%.17g\" stroke=\"%s\" stroke-opacity=\"%.3g\" stroke-width=\"%.17g\" stroke-linecap=\"round\"/>\n",
		(double)a.x, (double)-a.y, (double)b.x, (double)-b.y, stroke, (double)fill.a, (double)(2.0*radius));
}

static void
svg_draw_polygon(int count, const cpVect *verts, cpFloat radius, cpSpaceDebugColor outline, cpSpaceDebugColor fill, cpDataPointer data)
{
	SvgContext *ctx = (SvgContext *)data;
	char stroke[16], fill_color[16];
	(void)radius;
	svg_color(stroke, sizeof(stroke), outline);
	svg_color(fill_color, sizeof(fill_color), fill);
	fprintf(ctx->file, "  <polygon points=\"");
	for(int i = 0; i < count; i++) fprintf(ctx->file, "%.17g,%.17g ", (double)verts[i].x, (double)-verts[i].y);
	fprintf(ctx->file, "\" fill=\"%s\" fill-opacity=\"%.3g\" stroke=\"%s\" stroke-width=\"0.03\"/>\n", fill_color, (double)fill.a, stroke);
}

static void
svg_draw_dot(cpFloat size, cpVect pos, cpSpaceDebugColor color, cpDataPointer data)
{
	SvgContext *ctx = (SvgContext *)data;
	char fill_color[16];
	svg_color(fill_color, sizeof(fill_color), color);
	fprintf(ctx->file, "  <circle cx=\"%.17g\" cy=\"%.17g\" r=\"%.17g\" fill=\"%s\"/>\n", (double)pos.x, (double)-pos.y, (double)(size/2.0), fill_color);
}

static void
write_svg_snapshot(Benchmark *benchmark, int size, int step, const char *path)
{
	void *state = NULL;
	cpSpace *space = benchmark->init(size, &state);
	if(step < 0 || step > benchmark->steps) step = benchmark->steps;
	for(int i = 0; i < step; i++) benchmark->update(space, state, (cpFloat)(1.0/FPS));

	Summary summary = collect_summary(space, step);
	cpBB bb = summary.has_shape_bounds ? summary.shape_bounds : cpBBNew(-1.0, -1.0, 1.0, 1.0);
	cpFloat margin = cpfmax(1.0, cpfmax(bb.r - bb.l, bb.t - bb.b)*0.05);
	cpFloat min_x = bb.l - margin;
	cpFloat max_x = bb.r + margin;
	cpFloat min_y = -bb.t - margin;
	cpFloat max_y = -bb.b + margin;
	cpFloat width = cpfmax(1.0, max_x - min_x);
	cpFloat height = cpfmax(1.0, max_y - min_y);

	FILE *file = (strcmp(path, "-") == 0 ? stdout : fopen(path, "w"));
	if(!file){
		fprintf(stderr, "Could not open SVG output path: %s\n", path);
		exit(1);
	}

	SvgContext ctx = {file};
	fprintf(file, "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"%.17g %.17g %.17g %.17g\">\n", (double)min_x, (double)min_y, (double)width, (double)height);
	fprintf(file, "  <rect x=\"%.17g\" y=\"%.17g\" width=\"%.17g\" height=\"%.17g\" fill=\"white\"/>\n", (double)min_x, (double)min_y, (double)width, (double)height);
	fprintf(file, "  <title>MunkBench %s size %d step %d</title>\n", benchmark->name, size, step);

	cpSpaceDebugColor outline_color = {0.10f, 0.10f, 0.12f, 1.0f};
	cpSpaceDebugColor constraint_color = {0.80f, 0.25f, 0.25f, 1.0f};
	cpSpaceDebugColor collision_color = {0.90f, 0.20f, 0.20f, 1.0f};
	cpSpaceDebugDrawOptions options = {
		svg_draw_circle,
		svg_draw_segment,
		svg_draw_fat_segment,
		svg_draw_polygon,
		svg_draw_dot,
		CP_SPACE_DEBUG_DRAW_SHAPES,
		outline_color,
		svg_color_for_shape,
		constraint_color,
		collision_color,
		cpTransformIdentity,
		&ctx,
	};
	cpSpaceDebugDraw(space, &options);
	fprintf(file, "</svg>\n");

	if(file != stdout) fclose(file);
	free_space_children(space);
	cpSpaceFree(space);
	if(benchmark->destroy_state) benchmark->destroy_state(state);
}

static RunResult
run_benchmark(Benchmark *benchmark, int size)
{
	RunResult result = {benchmark->name, size, 0.0, 0.0};
	void *state = NULL;

	double init_start_time = now_seconds();
	cpSpace *space = benchmark->init(size, &state);
	double sim_start_time = now_seconds();

	for(int steps = 0; steps < benchmark->steps; steps++){
		benchmark->update(space, state, (cpFloat)(1.0/FPS));
	}

	double end_time = now_seconds();
	result.init_time = sim_start_time - init_start_time;
	result.run_time = end_time - sim_start_time;

	free_space_children(space);
	cpSpaceFree(space);
	if(benchmark->destroy_state) benchmark->destroy_state(state);

	return result;
}

static int
name_selected(Benchmark *benchmark, char **names, int name_count)
{
	if(name_count == 0) return 1;
	for(int i = 0; i < name_count; i++){
		if(strcmp(benchmark->name, names[i]) == 0) return 1;
	}
	return 0;
}

int
main(int argc, char **argv)
{
	int size_arg_set = 0;
	int size_arg = 0;
	int summary_json = 0;
	const char *checkpoints_arg = NULL;
	const char *svg_path = NULL;
	int svg_step = -1;
	char **selected_names = NULL;
	int selected_name_count = 0;

	for(int i = 1; i < argc; i++){
		if(strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0){
			usage(argv[0]);
			return 0;
		} else if(strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--size") == 0){
			if(i + 1 >= argc){
				fprintf(stderr, "Missing value for %s.\n", argv[i]);
				return 2;
			}
			size_arg = atoi(argv[++i]);
			size_arg_set = 1;
		} else if(strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--benchmarks") == 0){
			while(i + 1 < argc && argv[i + 1][0] != '-'){
				selected_names = (char **)realloc(selected_names, sizeof(char *)*(selected_name_count + 1));
				if(!selected_names){
					fprintf(stderr, "Out of memory.\n");
					return 1;
				}
				selected_names[selected_name_count++] = argv[++i];
			}
		} else if(strcmp(argv[i], "--summary-json") == 0){
			summary_json = 1;
		} else if(strcmp(argv[i], "--checkpoints") == 0){
			if(i + 1 >= argc){
				fprintf(stderr, "Missing value for %s.\n", argv[i]);
				return 2;
			}
			checkpoints_arg = argv[++i];
		} else if(strcmp(argv[i], "--svg") == 0){
			if(i + 1 >= argc){
				fprintf(stderr, "Missing value for %s.\n", argv[i]);
				return 2;
			}
			svg_path = argv[++i];
		} else if(strcmp(argv[i], "--step") == 0){
			if(i + 1 >= argc){
				fprintf(stderr, "Missing value for %s.\n", argv[i]);
				return 2;
			}
			svg_step = atoi(argv[++i]);
		} else {
			fprintf(stderr, "Unknown argument: %s\n", argv[i]);
			usage(argv[0]);
			free(selected_names);
			return 2;
		}
	}

	for(int i = 0; i < selected_name_count; i++){
		if(!find_benchmark(selected_names[i])){
			fprintf(stderr, "Unknown benchmark: %s\n", selected_names[i]);
			fprintf(stderr, "Known benchmarks:\n");
			for(int j = 0; j < benchmark_count; j++) fprintf(stderr, "  %s\n", benchmarks[j].name);
			free(selected_names);
			return 2;
		}
	}

	if(svg_path){
		if(selected_name_count != 1){
			fprintf(stderr, "--svg requires exactly one benchmark selected with -b/--benchmarks.\n");
			free(selected_names);
			return 2;
		}
		if(size_arg_set && size_arg == -1){
			fprintf(stderr, "--svg does not support size sweeps.\n");
			free(selected_names);
			return 2;
		}
		Benchmark *benchmark = find_benchmark(selected_names[0]);
		int size = size_arg_set ? size_arg : benchmark->default_size;
		write_svg_snapshot(benchmark, size, svg_step, svg_path);
		free(selected_names);
		return 0;
	}

	if(summary_json){
		if(size_arg_set && size_arg == -1){
			fprintf(stderr, "--summary-json does not support size sweeps.\n");
			free(selected_names);
			return 2;
		}
		int max_checkpoints = 128;
		int checkpoints[128];
		int checkpoint_count = parse_checkpoints(checkpoints_arg, checkpoints, max_checkpoints);
		printf("{\"version\":\"%s\",\"benchmarks\":[", MUNK2D_VERSION);
		int emitted = 0;
		for(int i = 0; i < benchmark_count; i++){
			Benchmark *benchmark = &benchmarks[i];
			if(!name_selected(benchmark, selected_names, selected_name_count)) continue;
			if(emitted++) printf(",");
			int size = size_arg_set ? size_arg : benchmark->default_size;
			run_summary_json(benchmark, size, checkpoints, checkpoint_count, max_checkpoints);
		}
		printf("]}\n");
		free(selected_names);
		return 0;
	}

	printf("version,benchmark,size,init_time,run_time\n");
	for(int i = 0; i < benchmark_count; i++){
		Benchmark *benchmark = &benchmarks[i];
		if(!name_selected(benchmark, selected_names, selected_name_count)) continue;

		if(!size_arg_set){
			RunResult result = run_benchmark(benchmark, benchmark->default_size);
			printf("%s,%s,%d,%.9f,%.9f\n", MUNK2D_VERSION, result.benchmark, result.size, result.init_time, result.run_time);
		} else if(size_arg == -1){
			int step = (benchmark->size_end + 1 - benchmark->size_start)/11;
			if(step <= 0) step = benchmark->size_inc;
			for(int size = benchmark->size_start; size <= benchmark->size_end; size += step){
				RunResult result = run_benchmark(benchmark, size);
				printf("%s,%s,%d,%.9f,%.9f\n", MUNK2D_VERSION, result.benchmark, result.size, result.init_time, result.run_time);
			}
		} else {
			RunResult result = run_benchmark(benchmark, size_arg);
			printf("%s,%s,%d,%.9f,%.9f\n", MUNK2D_VERSION, result.benchmark, result.size, result.init_time, result.run_time);
		}
		fflush(stdout);
	}

	free(selected_names);
	return 0;
}
