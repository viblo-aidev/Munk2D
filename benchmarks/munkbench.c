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
	printf("Usage: %s [-b benchmark ...] [-s size]\n\n", argv0);
	printf("Options:\n");
	printf("  -b, --benchmarks  Run only the named benchmarks.\n");
	printf("  -s, --size        Size to run. Omit for each benchmark default; use -1 for size sweep.\n");
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
	return add_shape_with_density(space, cpBoxShapeNew2(body, cpBBNew(l, b, r, t), 0.0), density);
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
		cpSpaceAddShape(space, cpBoxShapeNew2(body, cpBBNew(2.0, 4.0, 3.0, 0.0), 0.0));
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
			cpSpaceAddShape(space, cpBoxShapeNew2(body, cpBBNew(1.0, bs + ps, 2.0, -bs + ps), 0.0));
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
