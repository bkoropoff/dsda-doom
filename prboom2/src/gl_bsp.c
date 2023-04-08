/* Emacs style mode select   -*- C -*-
 *-----------------------------------------------------------------------------
 *
 *
 *  PrBoom: a Doom port merged with LxDoom and LSDLDoom
 *  based on BOOM, a modified and improved DOOM engine
 *  Copyright (C) 1999 by
 *  id Software, Chi Hoang, Lee Killough, Jim Flynn, Rand Phares, Ty Halderman
 *  Copyright (C) 1999-2000 by
 *  Jess Haas, Nicolas Kalkhof, Colin Phipps, Florian Schulze
 *  Copyright 2005, 2006 by
 *  Florian Schulze, Colin Phipps, Neil Stevens, Andrey Budko
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 *  02111-1307, USA.
 *
 * DESCRIPTION:
 *      BSP traversal, handling of LineSegs for rendering.
 *
 *-----------------------------------------------------------------------------*/

#include <assert.h>

#include "doomstat.h"
#include "m_bbox.h"
#include "p_spec.h"
#include "r_main.h"
#include "r_plane.h"
#include "r_things.h"
#include "r_bsp.h" // cph - sanity checking
#include "gl_struct.h"
#include "dsda/bsp.h"

static unsigned int gl_numdeferred = 0;

//
// e6y: Check whether the player can look beyond this line
//

static dboolean CheckClip(seg_t * seg, sector_t * frontsector, sector_t * backsector)
{
  static sector_t tempsec_back, tempsec_front;

  backsector = R_FakeFlat(backsector, &tempsec_back, NULL, NULL, true);
  frontsector = R_FakeFlat(frontsector, &tempsec_front, NULL, NULL, false);

  // check for closed sectors!
  if (backsector->ceilingheight <= frontsector->floorheight)
  {
    if (seg->sidedef->toptexture == NO_TEXTURE)
      return false;

    if (backsector->ceilingpic == skyflatnum && frontsector->ceilingpic == skyflatnum)
      return false;

    return true;
  }

  if (frontsector->ceilingheight <= backsector->floorheight)
  {
    if (seg->sidedef->bottomtexture == NO_TEXTURE)
      return false;

    // properly render skies (consider door "open" if both floors are sky):
    if (backsector->ceilingpic == skyflatnum && frontsector->ceilingpic == skyflatnum)
      return false;

    return true;
  }

  if (backsector->ceilingheight <= backsector->floorheight)
  {
    // preserve a kind of transparent door/lift special effect:
    if (backsector->ceilingheight < frontsector->ceilingheight)
    {
      if (seg->sidedef->toptexture == NO_TEXTURE)
        return false;
    }
    if (backsector->floorheight > frontsector->floorheight)
    {
      if (seg->sidedef->bottomtexture == NO_TEXTURE)
        return false;
    }
    if (backsector->ceilingpic == skyflatnum && frontsector->ceilingpic == skyflatnum)
      return false;

    if (backsector->floorpic == skyflatnum && frontsector->floorpic == skyflatnum)
      return false;

    return true;
  }

  return false;
}

static void GL_DeferFlats(gl_chunk_t* comp)
{
  if (comp->deferred)
    // Do nothing if chunk is already in deferral list
    return;

  gl_rstate.deferred[gl_numdeferred++] = comp;
  comp->deferred = true;
}

static void ApplyBleed(gl_chunk_t* source, bleed_t* bleed)
{
  bleedtarget_t existing;
  bleedtarget_t* target;
  sourceplane_t* plane;
  dboolean ceiling;

  switch (bleed->type)
  {
  case BLEED_FLOOR_OVER:
    plane = &source->floor;
    if (!plane->plane)
      return;
    if (plane->plane->height <= bleed->target->sector->floorheight)
      return;
    target = &bleed->target->floorover;
    ceiling = false;
    break;
  case BLEED_FLOOR_UNDER:
    plane = &source->floor;
    if (!plane->plane)
      return;
    if (plane->plane->height >= bleed->target->sector->floorheight)
      return;
    target = &bleed->target->floorunder;
    ceiling = false;
    break;
  case BLEED_FLOOR_THROUGH:
    plane = &source->floor;
    if (!plane->plane)
      return;
    if (plane->plane->height != bleed->target->sector->floorheight)
      return;
    target = &bleed->target->floorthrough;
    ceiling = false;
    break;
  case BLEED_CEILING_OVER:
    plane = &source->ceiling;
    if (!plane->plane)
      return;
    if (plane->plane->height >= bleed->target->sector->ceilingheight)
      return;
    target = &bleed->target->ceilingover;
    ceiling = true;
    break;
  case BLEED_CEILING_UNDER:
    plane = &source->ceiling;
    if (!plane->plane)
      return;
    if (plane->plane->height <= bleed->target->sector->ceilingheight)
      return;
    target = &bleed->target->ceilingunder;
    ceiling = true;
    break;
  case BLEED_CEILING_THROUGH:
    plane = &source->ceiling;
    if (!plane->plane)
      return;
    if (plane->plane->height != bleed->target->sector->ceilingheight)
      return;
    target = &bleed->target->ceilingthrough;
    ceiling = true;
    break;
  default:
    // Impossible absent a bug
    abort();
  }

  existing = *target;

  // First or lower-depth candidate always wins
  if (!existing.source || bleed->depth < existing.depth)
  {
    target->source = plane;
    target->depth = bleed->depth;
    GL_DeferFlats(bleed->target);
    return;
  }

  if (bleed->depth > existing.depth)
    return;

  // Pick the bleedier sector
  if ((ceiling && existing.source->sector->ceilingheight > source->sector->ceilingheight) ||
      (!ceiling && existing.source->sector->floorheight < source->sector->floorheight))
  {
    target->source = plane;
    target->depth = bleed->depth;
    GL_DeferFlats(bleed->target);
    return;
  }

  // Among equals, arbitrarily pick the sector with the higher number to avoid
  // dependence on BSP traversal order
  // FIXME: find a better heuristic
  if (source->sector > existing.source->sector &&
      ((ceiling && existing.source->sector->ceilingheight ==
                       source->sector->ceilingheight) ||
       (!ceiling && existing.source->sector->floorheight ==
                        source->sector->floorheight)))
  {
    target->source = plane;
    target->depth = bleed->depth;
    GL_DeferFlats(bleed->target);
  }
}

static void GL_AddLine (subsector_t* sub, seg_t *line)
{
  angle_t  angle1;
  angle_t  angle2;
  line_t* l = line->linedef;
  gl_chunk_t* comp = GL_Chunk(sub->chunk);
  int i;

  // "Activate" chunk flats on first non-self-referencing seg draw
  if (comp->validcount != validcount && l->frontsector != l->backsector)
  {
    comp->validcount = validcount;

    if (comp->floor.plane || comp->ceiling.plane)
    {
      GL_DeferFlats(comp);

      // Apply bleeds
      for (i = 0; i < comp->numbleeds; ++i)
        ApplyBleed(comp, &gl_rstate.bleeds[comp->firstbleed + i]);
    }
  }

  angle1 = R_PointToPseudoAngle(line->v1->x, line->v1->y);
  angle2 = R_PointToPseudoAngle(line->v2->x, line->v2->y);

  // Back side, i.e. backface culling	- read: endAngle >= startAngle!
  if (angle2 - angle1 < ANG180)
  {
    return;
  }
  if (!gld_clipper_SafeCheckRange(angle2, angle1))
  {
    return;
  }

  gl_rstate.map_subsectors[sub - gl_rstate.subsectors] = 1;

  if (!line->backsector)
  {
    gld_clipper_SafeAddClipRange(angle2, angle1);
  }
  else
  {
    if (line->frontsector == line->backsector)
    {
      if (texturetranslation[line->sidedef->midtexture] == NO_TEXTURE)
      {
        //e6y: nothing to do here!
        return;
      }
    }
    if (CheckClip(line, line->frontsector, line->backsector))
    {
      gld_clipper_SafeAddClipRange(angle2, angle1);
    }
  }

  line->linedef->flags |= ML_MAPPED;

  gld_AddWall(line);
}

//
// GL_CheckBBox
// Checks BSP node/subtree bounding box.
// Returns true
//  if some part of the bbox might be visible.
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

// killough 1/28/98: static // CPhipps - const parameter, reformatted
static dboolean GL_CheckBBox(const fixed_t *bspcoord)
{
  angle_t angle1, angle2;
  int        boxpos;
  const int* check;

  // Find the corners of the box
  // that define the edges from current viewpoint.
  boxpos = (viewx <= bspcoord[BOXLEFT] ? 0 : viewx < bspcoord[BOXRIGHT ] ? 1 : 2) +
    (viewy >= bspcoord[BOXTOP ] ? 0 : viewy > bspcoord[BOXBOTTOM] ? 4 : 8);

  if (boxpos == 5)
    return true;

  check = checkcoord[boxpos];

  angle1 = R_PointToPseudoAngle(bspcoord[check[0]], bspcoord[check[1]]);
  angle2 = R_PointToPseudoAngle(bspcoord[check[2]], bspcoord[check[3]]);
  return gld_clipper_SafeCheckRange(angle2, angle1);
}

static void GL_UpdateSourcePlane(gl_chunk_t* comp, sourceplane_t* source, dboolean ceiling)
{
  sector_t* sector;
  int cll, fll;

  if (source->validcount == validcount)
    return;

  sector = source->sector =
      R_FakeFlat(comp->sector, &source->dummysector, &fll, &cll, false);

  if (ceiling)
  {
    source->lightlevel = cll;

    if (sector->ceilingheight > viewz || sector->ceilingpic == skyflatnum ||
        (sector->heightsec != -1 &&
         sectors[sector->heightsec].floorpic == skyflatnum))
    {
      source->plane = &source->dummyplane;
      source->dummyplane.height = sector->ceilingheight;
      source->dummyplane.special = sector->special;
      source->dummyplane.xoffs = sector->ceiling_xoffs;
      source->dummyplane.yoffs = sector->ceiling_yoffs;
      source->dummyplane.xscale = sector->ceiling_xscale;
      source->dummyplane.yscale = sector->ceiling_yscale;
      source->dummyplane.rotation = sector->ceiling_rotation;
      source->dummyplane.lightlevel = cll;
      source->dummyplane.picnum =
          sector->ceilingpic == skyflatnum && sector->sky & PL_SKYFLAT
              ? sector->sky
              : sector->ceilingpic;
    } else
      source->plane = NULL;
  }
  else
  {
    source->lightlevel = fll;

    if (source->sector->floorheight < viewz ||
        (sector->heightsec != -1 &&
         sectors[sector->heightsec].ceilingpic == skyflatnum))
    {
      source->plane = &source->dummyplane;
      source->dummyplane.height = sector->floorheight;
      source->dummyplane.special = sector->special;
      source->dummyplane.xoffs = sector->floor_xoffs;
      source->dummyplane.yoffs = sector->floor_yoffs;
      source->dummyplane.xscale = sector->floor_xscale;
      source->dummyplane.yscale = sector->floor_yscale;
      source->dummyplane.rotation = sector->floor_rotation;
      source->dummyplane.lightlevel = fll;
      source->dummyplane.picnum =
          sector->floorpic == skyflatnum && sector->sky & PL_SKYFLAT
              ? sector->sky
              : sector->floorpic;
    } else
      source->plane = NULL;
  }

  source->validcount = validcount;
}

static void ClearBleedTarget(bleedtarget_t* target)
{
  target->source = NULL;
  target->depth = INT_MAX;
}

// Render flats of activated chunks now that all bleed effects have been
// updated
void GL_RenderDeferred(void)
{
  int i;

  for (i = 0; i < gl_numdeferred; ++i)
  {
    gl_chunk_t* comp = gl_rstate.deferred[i];
    int compnum = comp - gl_rstate.chunks;

    // Clear deferred mark
    comp->deferred = false;

    // Render ordinary planes if activated during BSP traversal

    if (comp->floor.plane && comp->validcount == validcount)
      gld_AddFlat(compnum, false, comp->floor.plane, comp->floor.sector);

    if (comp->ceiling.plane && comp->validcount == validcount)
      gld_AddFlat(compnum, true, comp->ceiling.plane, comp->ceiling.sector);
    
    // Render any over/under planes

    if (comp->floorover.source)
    {
      sourceplane_t* source = comp->floorover.source;
      if (source->plane)
        gld_AddFlat(compnum, false, source->plane, source->sector);
    }

    if (comp->floorunder.source)
    {
      sourceplane_t* source = comp->floorunder.source;
      if (source->plane)
        gld_AddFlat(compnum, false, source->plane, source->sector);
    }

    if (comp->ceilingover.source)
    {
      sourceplane_t* source = comp->ceilingover.source;
      if (source->plane)
        gld_AddFlat(compnum, true, source->plane, source->sector);
    }

    if (comp->ceilingunder.source)
    {
      sourceplane_t* source = comp->ceilingunder.source;
      if (source->plane)
        gld_AddFlat(compnum, true, source->plane, source->sector);
    }

    // If this was activated by bleed-through (and not activated directly),
    // render the bleed source planes

    if (comp->floorthrough.source && comp->validcount != validcount)
    {
      sourceplane_t* source = comp->floorthrough.source;
      // Don't hide underbleeding floor with a different texture
      // FIXME: this is a questionable heuristic
      if (source->plane && (!comp->floorunder.source ||
                            !comp->floorunder.source->sector ||
                            comp->floorunder.source->sector->floorpic ==
                                source->plane->picnum))
        gld_AddFlat(compnum, false, source->plane, source->sector);
    }

    if (comp->ceilingthrough.source && comp->validcount != validcount)
    {
      sourceplane_t* source = comp->ceilingthrough.source;
      // Don't hide underbleeding ceiling with a different texture
      // FIXME: this is a questionable heuristic
      if (source->plane && (!comp->ceilingunder.source ||
                            !comp->ceilingunder.source->sector ||
                            comp->ceilingunder.source->sector->ceilingpic ==
                                source->plane->picnum))
        gld_AddFlat(compnum, true, source->plane, source->sector);
    }

    ClearBleedTarget(&comp->floorunder);
    ClearBleedTarget(&comp->floorover);
    ClearBleedTarget(&comp->floorthrough);
    ClearBleedTarget(&comp->ceilingunder);
    ClearBleedTarget(&comp->ceilingover);
    ClearBleedTarget(&comp->ceilingthrough);

    comp->validcount = validcount;
  }

  gl_numdeferred = 0;
}

// GL_Subsector
// Determine floor/ceiling planes.
// Add sprites of things in sector.
// Draw one or more line segments.
//
// Derived from R_Subsector
static void GL_Subsector(subsector_t* sub)
{
  int         count;
  seg_t       *line;
  gl_chunk_t* comp = GL_Chunk(sub->chunk);

  GL_UpdateSourcePlane(comp, &comp->floor, false);
  GL_UpdateSourcePlane(comp, &comp->ceiling, true);

  if (sub->sector->validcount != validcount)
  {
    // killough 9/18/98: Fix underwater slowdown, by passing real sector
    // instead of fake one. Improve sprite lighting by basing sprite
    // lightlevels on floor & ceiling lightlevels in the surrounding area.
    //
    // 10/98 killough:
    //
    // NOTE: TeamTNT fixed this bug incorrectly, messing up sprite lighting!!!
    // That is part of the 242 effect!!!  If you simply pass sub->sector to
    // the old code you will not get correct lighting for underwater sprites!!!
    // Either you must pass the fake sector and handle validcount here, on the
    // real sector, or you must account for the lighting in some other way,
    // like passing it as an argument.
    sub->sector->validcount = validcount;

    R_AddSprites(sub, (comp->floor.lightlevel + comp->ceiling.lightlevel) / 2);
  }

  // hexen
  if (sub->poly)
  {                           // Render the polyobj in the subsector first
    int polyCount;
    seg_t **polySeg;

    poly_add_line = true;
    poly_frontsector = sub->sector;
    polyCount = sub->poly->numsegs;
    polySeg = sub->poly->segs;
    while (polyCount--)
      GL_AddLine(sub, *polySeg++);
    poly_add_line = false;
    poly_frontsector = NULL;
  }

  count = sub->numrsegs;
  line = &gl_rstate.rsegs[sub->firstrseg];
  while (count--)
  {
    GL_AddLine(sub, line);
    line++;
  }
}

//
// RenderBSPNode
// Renders all subsectors below a given node,
//  traversing subtree recursively.
// Just call with BSP root.
//
// killough 5/2/98: reformatted, removed tail recursion

void GL_RenderBSPNode(int bspnum)
{
  subsector_t* sub;

  while (!(bspnum & NF_SUBSECTOR))  // Found a subsector?
  {
    const node_t *bsp = &gl_rstate.nodes[bspnum];

    // Decide which side the view point is on.
    int side = R_PointOnSide(viewx, viewy, bsp);
    // Recursively divide front space.
    GL_RenderBSPNode(bsp->children[side]);

    // Possibly divide back space.

    if (!GL_CheckBBox(bsp->bbox[side^1]))
      return;

    bspnum = bsp->children[side^1];
  }
  // e6y: support for extended nodes
  sub = &gl_rstate.subsectors[bspnum == -1 ? 0 : bspnum & ~NF_SUBSECTOR];
  GL_Subsector(sub);
}
