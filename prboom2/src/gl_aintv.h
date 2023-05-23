// Copyright (C) 2023 Brian Koropoff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//

// DESCRIPTION:
//	Angular Intervals
//

// Angular intervals are used to represent the view emanating from the player,
// which can be clipped and recombined as it passes through portals in the map.
//
// It's permissible for the start angle to be larger than the end angle,
// (otherwise some ranges could not be represented as a simple pair), so the
// following code handles that case.  Such intervals are called backward as
// opposed to forward.
//
// Intervals with equal start and end angles all represent the same interval,
// but which one is ambiguous: the empty interval (0 degrees magnitude) or the
// full interval (360 degrees magnitude).  Therefore, two members of this class
// are used to represent the empty and full intervals as special cases, and all
// others are disallowed.
//
// Angular intervals use pseudo angles for efficiency.

#ifndef __GL_AINTV__
#define __GL_AINTV__

#include <assert.h>

#include "tables.h"

// An angular interval
typedef struct
{
  angle_t start;
  angle_t end;
} gl_aintv_t;

// Tests if `a` is the special empty interval
static inline dboolean GL_AIntvIsEmpty(const gl_aintv_t* a)
{
  return a->start == 0 && a->end == 0;
}

// Test if `a` is the special full interval
static inline dboolean GL_AIntvIsFull(const gl_aintv_t* a)
{
  return a->start == ANGLE_MAX && a->end == ANGLE_MAX;
}

// Initialize interval
static inline void GL_AIntvInit(gl_aintv_t* intv, angle_t start, angle_t end)
{
  // Equal angles are ambiguous between an empty or full interval, so this
  // function should never be called with them
  assert(start != end);
  intv->start = start;
  intv->end = end;
}

// Compute (superset of) intersection of two angular intervals. If the two
// intervals have two disconnected intersections (which is possible due to the
// modular nature of angles), the resulting interval will be the smallest
// interval large enough to contain both intersections. Returns false if the
// result is empty, for convenience.
dboolean GL_AIntvIntersect(const gl_aintv_t* a, const gl_aintv_t* b, gl_aintv_t* result);

// Compute (superset of) union of two angular intervals. If the two intervals
// have no overlap, the result may be larger than the set-wise union as it's not
// possible for an interval to represent disconnected subsets.
void GL_AIntvUnion(const gl_aintv_t* a, const gl_aintv_t* b, gl_aintv_t* result);

// Compute angular interval from bounding box
void GL_AIntvFromBBox(gl_aintv_t* intv, const fixed_t bbox[4]);

// Compute angular interval corresponding to player view
void GL_AIntvFromView(gl_aintv_t* intv);

#endif
