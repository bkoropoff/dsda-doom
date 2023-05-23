// Copyright (C) 1999 id Software, Lee Killough
// Copyright (C) 1999-2000 Colin Phipps, Florian Schulze
// Copyright (C) 2005-2006 Colin Phipps, Andrey Budko
// Copyright (C) 2023 Brian Koropoff
//
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

#include <assert.h>
#include <math.h>

#include "e6y.h"
#include "m_bbox.h"
#include "r_main.h"
#include "r_state.h"

#include "gl_aintv.h"

// Special value representing the empty interval
static const gl_aintv_t empty_intv = {0, 0};
// Special value representing the full (360 degree) interval
static const gl_aintv_t full_intv = {ANGLE_MAX, ANGLE_MAX};

dboolean GL_AIntvIntersect(const gl_aintv_t* a, const gl_aintv_t* b, gl_aintv_t* result)
{
  gl_aintv_t an;
  gl_aintv_t bn;
  gl_aintv_t r;

  // Handle special cases
  if (GL_AIntvIsEmpty(a) || GL_AIntvIsEmpty(b))
  {
    *result = empty_intv;
    return false;
  }
  else if (GL_AIntvIsFull(a))
  {
    *result = *b;
    return !GL_AIntvIsEmpty(b);
  }
  else if (GL_AIntvIsFull(b))
  {
    *result = *a;
    return !GL_AIntvIsEmpty(a);
  }

  // These should not occur outside of special cases
  assert(a->start != a->end);
  assert(b->start != b->end);

  // Normalize `a` and `b` to be relative to the start of `a`. This ensures `an`
  // is always a forward interval
  an.start = 0;
  an.end = a->end - a->start;
  bn.start = b->start - a->start;
  bn.end = b->end - a->start;

  if (bn.start <= bn.end)
  {
    // `bn` is also a forward angle interval
    if (bn.start >= an.end)
    {
      *result = empty_intv;
      return false;
    }

    r.start = a->start + bn.start;
    r.end = a->start + (an.end < bn.end ? an.end : bn.end);
  }
  else
  {
    // `bn` is a backward angle interval, so it definitely intersects `an` since
    // `an` starts at 0.  Check if it intersects on both sides.
    if (an.end > bn.start)
    {
      // There are two disconnected intersections, so the smallest interval containing
      // them is one of the two inputs.  Return it.
      *result = a->end - a->start < b->end - b->start ? *a : *b;
      return true;
    }

    // Only one intersection, compute it
    r.start = a->start;
    r.end = a->start + (an.end < bn.end ? an.end : bn.end);

    if (r.start == r.end)
    {
      // Unambiguously empty, since the intersection of non-full intervals is
      // always non-full
      *result = empty_intv;
      return false;
    }
  }

  // Empty/full result should have been handled already
  assert(r.end != r.start);
  // The intersection should be equal to or smaller than its operands
  assert(r.end - r.start <= a->end - a->start);
  assert(r.end - r.start <= b->end - b->start);

  *result = r;

  return true;
}

// FIXME: when including a gap between the two intervals, include the smallest
// of the two possible gaps
void GL_AIntvUnion(const gl_aintv_t* a, const gl_aintv_t* b, gl_aintv_t* result)
{
  gl_aintv_t an;
  gl_aintv_t bn;
  gl_aintv_t r;

  // Handle special cases
  if (GL_AIntvIsFull(a) || GL_AIntvIsFull(b))
  {
    *result = full_intv;
    return;
  }
  else if (GL_AIntvIsEmpty(a))
  {
    *result = *b;
    return;
  }
  else if (GL_AIntvIsEmpty(b))
  {
    *result = *a;
    return;
  }

  // These should not occur outside of special cases
  assert(a->start != a->end);
  assert(b->start != b->end);

  // Normalize `a` and `b` to be relative to the start of `a`. This ensures `an`
  // is always a forward interval
  an.start = 0;
  an.end = a->end - a->start;
  bn.start = b->start - a->start;
  bn.end = b->end - a->start;

  if (bn.start < bn.end)
  {
    // `bn` is also a forward angle interval
    // A union of two forward intervals can never be full
    r.start = a->start;
    r.end = a->start + (an.end > bn.end ? an.end : bn.end);
  }
  else
  {
    // `bn` is a backward interval
    if (an.end >= bn.start)
    {
      *result = full_intv;
      return;
    }
    else
    {
      r.start = b->start;
      r.end = a->start + (an.end > bn.end ? an.end : bn.end);
    }
  }

  // Empty/full result should have been handled already
  assert(r.end != r.start);
  // The union should be equal to or larger than its operands
  assert(r.end - r.start >= a->end - a->start);
  assert(r.end - r.start >= b->end - b->start);

  *result = r;

  return;
}

//
// AIntvFromBBox
//
// Computes angle interval from bounding box, based on R_CheckBBox
//

static const int checkcoord[12][4] = // killough -- static const
{
  {3,0,2,1},
  {3,0,2,0},
  {3,1,2,0},
  {0},
  {2,0,2,1},
  {0,0,0,0},
  {3,1,3,0},
  {0},
  {2,0,3,1},
  {2,1,3,1},
  {2,1,3,0}
};

void GL_AIntvFromBBox(gl_aintv_t* intv, const fixed_t bbox[4])
{
  angle_t angle1, angle2;
  int        boxpos;
  const int* check;

  // Find the corners of the box that define the edges from current viewpoint.
  // Note: this differs from the logic in R_CheckBBox by ensuring that the
  // player is considered to be inside the bounding box if they are precisely on
  // any of its edges.  This avoids false negative visibility tests.
  boxpos = (viewx < bbox[BOXLEFT] ? 0 : viewx <= bbox[BOXRIGHT] ? 1 : 2) +
    (viewy > bbox[BOXTOP] ? 0 : viewy >= bbox[BOXBOTTOM] ? 4 : 8);

  if (boxpos == 5)
  {
    // Within the box, so it subtends a full 360 degrees
    *intv = full_intv;
    return;
  }

  check = checkcoord[boxpos];

  angle1 = R_PointToPseudoAngle(bbox[check[0]], bbox[check[1]]);
  angle2 = R_PointToPseudoAngle(bbox[check[2]], bbox[check[3]]);

  if (angle1 == angle2)
  {
    // This can happen if the portal line is axis-aligned and the player is
    // colinear with it
    *intv = empty_intv;
    return;
  }

  GL_AIntvInit(intv, angle2, angle1);
}

// Compute angular interval for player view from fov and view pitch
static void AIntvFromFovPitch(gl_aintv_t* intv, double vp)
{
  double va = viewangle * M_PI / ANG180;
  double halffov = atan(1 / projMatrix[0]);
  // Ray projecting out at half fov angle
  float fovray[4] = {cos(halffov), 0, sin(halffov), 1.0};
  // Pitch rotation matrix
  float pm[16] =
  {
    cos(vp), -sin(vp), 0, 0,
    sin(vp), cos(vp),  0, 0,
    0,       0,        1, 0,
    0,       0,        0, 1
  };
  float pfovray[4];
  double angle;
  angle_t angle1;
  angle_t angle2;

  if (halffov >= M_PI)
  {
    *intv = full_intv;
    return;
  }

  // Rotate fov ray by pitch
  R_MultMatrixVecd(pm, fovray, pfovray);
  // Compute angle of resulting ray in x/z plane
  angle = atan2(pfovray[2], pfovray[0]);

  if (angle >= M_PI)
  {
    *intv = full_intv;
    return;
  }

  // Convert to start/end pseudo angles around view angle
  angle1 = gld_AngleToPseudo(round((va - angle) / M_PI * ANG180));
  angle2 = gld_AngleToPseudo(round((va + angle) / M_PI * ANG180));

  GL_AIntvInit(intv, angle1, angle2);
}

void GL_AIntvFromView(gl_aintv_t* intv)
{
  double vfov = gl_render_fovy * M_PI / 180;
  double vp = viewpitch * M_PI / ANG180;

  // Wrap vp to between -M_PI/2 and M_PI/2
  if (vp > M_PI)
    vp -= 2 * M_PI;

  // Add half of vertical fov with same sign to find angle of lower/upper
  // frustum plane furthest from horizontal.
  if (vp > 0)
    vp += vfov / 2;
  else
    vp -= vfov / 2;

  // If lower/upper frustum plane is at or past vertical, 2D view is 360 degrees
  if ((vp > 0 && vp >= M_PI / 2) ||
      (vp < 0 && vp <= -M_PI / 2))
  {
    *intv = full_intv;
    return;
  }

  // Otherwise, compute from fov and pitch
  // FIXME: using lower/upper frustum plane pitch is a conservative
  // approximation which slightly overestimates effective 2D view at shallow
  // view angles, but only by about 10% at worst
  AIntvFromFovPitch(intv, vp);
}
