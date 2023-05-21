//
// Copyright(C) 2023 Brian Koropoff
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
//	BSP Analysis
//

#include <assert.h>

#include "m_bbox.h"

#include "bspinternal.h"

static unsigned int capportals = 0;
static unsigned int numwalls = 0;

static portal_t* PushPortal(int chunk, seg_t* seg, portal_flags_t flags)
{
  portal_t* portal;

  if (gl_rstate.numportals == capportals)
  {
    if (capportals == 0)
      capportals = 256;
    else
      capportals *= 2;
    gl_rstate.portals =
        Z_Realloc(gl_rstate.portals, sizeof(*gl_rstate.portals) * capportals);
  }

  portal = &gl_rstate.portals[gl_rstate.numportals++];
  portal->chunk = chunk;
  portal->flags = flags;
  M_ClearBox(portal->bbox);
  M_AddToBox(portal->bbox, seg->v1->x, seg->v1->y);
  M_AddToBox(portal->bbox, seg->v2->x, seg->v2->y);

  return portal;
}

// Add portal induced by seg (if any) to chunk
static void AddPortal(gl_chunk_t* chunk, seg_t* seg)
{
  segmeta_t* segmeta = &dsda_gl_rstate.segmeta[seg - gl_rstate.segs];
  seg_t* pseg = segmeta->partner;
  segmeta_t* psegmeta;
  int ochunknum;
  int i;
  portal_flags_t flags = PORTF_NONE;

  if (!pseg)
    // Single-sided line
    return;

  if (seg->sidedef)
  {
    if (seg->sidedef->toptexture == NO_TEXTURE)
      flags |= PORTF_CLEAR_TOP;
    if (seg->sidedef->bottomtexture == NO_TEXTURE)
      flags |= PORTF_CLEAR_BOTTOM;
  }

  psegmeta = &dsda_gl_rstate.segmeta[pseg - gl_rstate.segs];
  ochunknum = psegmeta->subsector->chunk;

  if (ochunknum == chunk - gl_rstate.chunks)
    // Same chunk
    return;

  // Look for an existing portal to the same destination
  for (i = 0; i < chunk->numportals; ++i)
  {
    portal_t* portal = &gl_rstate.portals[chunk->firstportal + i];

    if (portal->chunk == ochunknum)
    {
      // Expand existing portal bounding box to encompass seg
      M_AddToBox(portal->bbox, seg->v1->x, seg->v1->y);
      M_AddToBox(portal->bbox, seg->v2->x, seg->v2->y);
      // Add flags
      portal->flags |= flags;
      return;
    }
  }

  // Create new portal
  PushPortal(ochunknum, seg, flags);
  chunk->numportals++;
}

static void WallInitFromSeg(gl_wall_t* wall, seg_t* seg)
{
  segmeta_t *segmeta = &dsda_gl_rstate.segmeta[seg - gl_rstate.segs];
  segmeta_t* psegmeta =
      segmeta->partner
          ? &dsda_gl_rstate.segmeta[segmeta->partner - gl_rstate.segs]
          : NULL;
  submeta_t* submeta =
      &dsda_gl_rstate.submeta[segmeta->subsector - gl_rstate.subsectors];
  submeta_t* psubmeta =
      psegmeta
          ? &dsda_gl_rstate.submeta[psegmeta->subsector - gl_rstate.subsectors]
          : NULL;
  int side = seg->sidedef == &sides[seg->linedef->sidenum[1]];

  wall->linedef = seg->linedef - lines;
  wall->v1id = (side ? seg->linedef->v2 : seg->linedef->v1) - vertexes;
  wall->v2id = (side ? seg->linedef->v1 : seg->linedef->v2) - vertexes;
  wall->alpha = seg->linedef->alpha;
  wall->frontsec = seg->frontsector - sectors;
  wall->backsec =
      seg->backsector ? seg->backsector - sectors : (unsigned short)-1;
  wall->sidedef = seg->sidedef - sides;

  if (side)
    wall->flags |= GL_WALLF_BACK;
  if (seg->linedef->flags & ML_DONTPEGBOTTOM)
    wall->flags |= GL_WALLF_DONTPEGBOTTOM;
  if (seg->linedef->flags & ML_DONTPEGTOP)
    wall->flags |= GL_WALLF_DONTPEGTOP;
  if (seg->linedef->flags & ML_WRAPMIDTEX)
    wall->flags |= GL_WALLF_WRAPMIDTEX;
  if (submeta->flags & SUBF_MULTISECTOR ||
      (psubmeta && psubmeta->flags & SUBF_MULTISECTOR))
    // Inhibit stencil bleed if subsectors on either side have an ambiguous
    // sector, as we might bleed the wrong floor/ceiling depending
    // non-deterministically on how node building assigned first segments to
    // subsectors.
    wall->flags |= GL_WALLF_NOBLEED;
}

static void AddChunkWall(gl_chunk_t* chunk, seg_t* seg)
{
  int i;
  gl_wall_t* wall;
  uint8_t side;

  if (!seg->linedef)
    return;

  side = seg->sidedef == &sides[seg->linedef->sidenum[1]];

  for (i = 0; i < chunk->numwalls; ++i)
  {
    wall = &gl_rstate.walls[chunk->firstwall + i];

    if (wall->linedef == seg->linedef - lines &&
        (wall->flags & GL_WALLF_BACK) == side)
      return;
  }

  wall = &gl_rstate.walls[numwalls++];
  WallInitFromSeg(wall, seg);

  chunk->numwalls++;

  // Render flats if this is not a purely self-referencing chunk
  // (i.e. a fake sector)
  if (seg->linedef->frontsector != seg->linedef->backsector)
    chunk->flags |= GL_CHUNKF_RENDER_FLATS;
}

static void AnnotateSubsectorRender(gl_chunk_t* chunk, subsector_t* sub)
{
  int i;

  for (i = 0; i < sub->numlines; ++i)
  {
    seg_t* seg = &gl_rstate.segs[sub->firstline + i];

    AddPortal(chunk, seg);
    AddChunkWall(chunk, seg);
  }
}

static void AnnotateChunkRender(chunkmeta_t* chunkmeta)
{
  gl_chunk_t* chunk = GL_Chunk(chunkmeta - dsda_gl_rstate.chunkmeta);
  int subnum;
  subsector_t* sub;
  submeta_t* submeta;

  chunk->firstwall = numwalls;
  chunk->firstportal = gl_rstate.numportals;

  for (subnum = chunkmeta->subsectors; subnum != -1;
       subnum = submeta->chunk_next)
  {
    sub = &gl_rstate.subsectors[subnum];
    submeta = &dsda_gl_rstate.submeta[subnum];

    AnnotateSubsectorRender(chunk, sub);
  }
}

static void AddPolyobjWall(gl_polyobj_t* gpo, seg_t* seg)
{
  int i;
  gl_wall_t* wall;
  uint8_t side;

  assert(seg->linedef);

  side = seg->sidedef == &sides[seg->linedef->sidenum[1]];

  for (i = 0; i < gpo->numwalls; ++i)
  {
    wall = &gl_rstate.walls[gpo->firstwall + i];

    if (wall->linedef == seg->linedef - lines &&
        (wall->flags & GL_WALLF_BACK) == side)
      return;
  }

  wall = &gl_rstate.walls[numwalls++];
  WallInitFromSeg(wall, seg);

  gpo->numwalls++;
}

static void AnnotatePolyobjRender(polyobj_t* po, gl_polyobj_t* gpo)
{
  int i;

  gpo->firstwall = numwalls;

  for (i = 0; i < po->numsegs; ++i)
  {
    seg_t* seg = po->segs[i];

    AddPolyobjWall(gpo, seg);
  }
}

void AnnotateRender(void)
{
  int i;

  // Annotate chunks with render info
  for (i = 0; i < gl_rstate.numchunks; ++i)
    AnnotateChunkRender(&dsda_gl_rstate.chunkmeta[i]);

  // Annotate polyobjs with render info
  gl_rstate.polyobjs = Z_Calloc(sizeof(*gl_rstate.polyobjs), po_NumPolyobjs);
  for (i = 0; i < po_NumPolyobjs; ++i)
    AnnotatePolyobjRender(&polyobjs[i], &gl_rstate.polyobjs[i]);
}

void ClearRender(void)
{
  numwalls = 0;
  if (gl_rstate.walls)
    Z_Free(gl_rstate.walls);

  if (gl_rstate.polyobjs)
    Z_Free(gl_rstate.polyobjs);
  gl_rstate.polyobjs = NULL;
}
