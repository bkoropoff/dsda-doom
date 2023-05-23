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
//	GL BSP/chunk rendering
//

// GL BSP rendering pass.
//
// BSP is sort of a misnomer, as the node tree is only used to locate the chunk
// occupied by the player.  Instead, a portal-based rendering algorithm is used.
// Each chunk is annotated with a list of portals leading to other chunks.  Each
// portal is represented by a bounding box around all two-sided linedefs
// connecting the source and destination.  See dsda/bsprender.c for where this
// happens.
//
// When rendering the initial chunk, the player's view angle interval is clipped
// against the angles subtended by each portal bounding box.  If the result is
// not empty, the destination chunk is potentially visible and is marked for
// rendering using the clipped view.  This proceeds in a breadth-first traversal
// of the chunk graph until complete.
//
// A chunk may be reachable by multiple paths, in which case subsequent paths
// expand the view into the destination chunk and mark it for rendering again if
// needed.  The total view angle interval into the chunk eventually reaches a
// maximum (in the limiting case, a full 360 degrees), at which point no further
// updates are possible.
//
// When a chunk is rendered, all walls that are not backface culled and that
// intersect the view into the chunk are added to the GL draw items list.
//
// The first time a chunk is rendered, it's flats are processed if applicable.
// It's added to a list for deferred flat rendering, and possible flat bleeding
// effects are processed for the chunk (determined in dsda/bspbleed.c), which
// may result in additional chunks being scheduled for deferred flat rendering.
//
// The first time a chunk in a particular sector is rendered, all sprites in the
// sector are added to the GL draw items list.
//
// After all chunks are rendered, the deferral list is processed and flats for
// all chunks and all bleed effects are added to the GL draw items list.

#include <assert.h>

#include "doomstat.h"
#include "m_bbox.h"
#include "p_spec.h"
#include "r_main.h"
#include "r_plane.h"
#include "r_things.h"
#include "r_bsp.h"
#include "gl_struct.h"
#include "e6y.h"
#include "dsda/bsp.h"

// Number of chunks in deferred flat rendering list
static unsigned int numdeferred = 0;
// Number of chunks in visibility processing queue
static unsigned int numvisible;

// Visibility processing queue ring head/tail
static gl_chunk_t** vhead;
static gl_chunk_t** vtail;

static gl_aintv_t playerview;

// Put chunk in deferral list to render its flats later
static void DeferFlats(gl_chunk_t* chunk)
{
  if (chunk->flags & GL_CHUNKF_DEFERRED)
    // Do nothing if chunk is already in deferral list
    return;

  gl_rstate.deferred[numdeferred++] = chunk;
  chunk->flags |= GL_CHUNKF_DEFERRED;
}

static void ApplyBleed(gl_chunk_t* source, bleed_t* bleed)
{
  bleedtarget_t existing;
  bleedtarget_t* target;
  gl_plane_t* plane;
  dboolean ceiling;
  gl_chunk_t* tchunk = &gl_rstate.chunks[bleed->target];

  switch (bleed->type)
  {
  case BLEED_FLOOR_OVER:
    plane = &source->floor;
    if (plane->picnum == -1)
      return;
    if (plane->height <= tchunk->sector->floorheight)
      return;
    target = &tchunk->floorover;
    ceiling = false;
    break;
  case BLEED_FLOOR_UNDER:
    plane = &source->floor;
    if (plane->picnum == -1)
      return;
    if (plane->height >= tchunk->sector->floorheight)
      return;
    target = &tchunk->floorunder;
    ceiling = false;
    break;
  case BLEED_FLOOR_THROUGH:
    plane = &source->floor;
    if (plane->picnum == -1)
      return;
    if (plane->height != tchunk->sector->floorheight)
      return;
    target = &tchunk->floorthrough;
    ceiling = false;
    break;
  case BLEED_CEILING_OVER:
    plane = &source->ceiling;
    if (plane->picnum == -1)
      return;
    if (plane->height >= tchunk->sector->ceilingheight)
      return;
    target = &tchunk->ceilingover;
    ceiling = true;
    break;
  case BLEED_CEILING_UNDER:
    plane = &source->ceiling;
    if (plane->picnum == -1)
      return;
    if (plane->height <= tchunk->sector->ceilingheight)
      return;
    target = &tchunk->ceilingunder;
    ceiling = true;
    break;
  case BLEED_CEILING_THROUGH:
    plane = &source->ceiling;
    if (plane->picnum == -1)
      return;
    if (plane->height != tchunk->sector->ceilingheight)
      return;
    target = &tchunk->ceilingthrough;
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
    DeferFlats(tchunk);
    return;
  }

  if (bleed->depth > existing.depth)
    return;

  // Pick the bleedier sector
  if ((ceiling && existing.source->height > source->sector->ceilingheight) ||
      (!ceiling && existing.source->height < source->sector->floorheight))
  {
    target->source = plane;
    target->depth = bleed->depth;
    DeferFlats(tchunk);
    return;
  }

  // Among equals, arbitrarily pick the source with the higher address to avoid
  // dependence on BSP traversal order
  // FIXME: find a better heuristic
  if (plane > existing.source &&
      ((ceiling && existing.source->height ==
                       source->sector->ceilingheight) ||
       (!ceiling && existing.source->height ==
                        source->sector->floorheight)))
  {
    target->source = plane;
    target->depth = bleed->depth;
    DeferFlats(tchunk);
  }
}

// Determine whether portal is closed by floor/ceiling heights of sectors on
// either side
static dboolean PortalIsClosed(portal_t* portal, sector_t* frontsector,
                               sector_t* backsector)
{
  sector_t dummy_back, dummy_front;

  backsector = R_FakeFlat(backsector, &dummy_back, NULL, NULL, true);
  frontsector = R_FakeFlat(frontsector, &dummy_front, NULL, NULL, false);

  if (backsector->ceilingheight <= frontsector->floorheight)
    // Floor in source sector is above ceiling of destination sector, so we can
    // only see into it if the top texture is clear or both sectors are sky
    // ceilings
    return !(portal->flags & PORTF_CLEAR_TOP) &&
           !(backsector->ceilingpic == skyflatnum &&
             frontsector->ceilingpic == skyflatnum);

  if (frontsector->ceilingheight <= backsector->floorheight)
    // Floor in destination sector is above ceiling of source sector, so we can
    // only see into it if the bottom texture is clear or both sectors are sky
    // ceilings.
    return !(portal->flags & PORTF_CLEAR_BOTTOM) &&
           !(backsector->ceilingpic == skyflatnum &&
             frontsector->ceilingpic == skyflatnum);

  if (backsector->ceilingheight <= backsector->floorheight)
    // Destination sector is closed (e.g. a door), so we can only see into it if
    // we can see into a clear top/bottom texture, or both sectors are sky
    // ceilings/floors.
    return !(backsector->ceilingheight < frontsector->ceilingheight &&
             portal->flags & PORTF_CLEAR_TOP) &&
           !(backsector->floorheight > frontsector->floorheight &&
             portal->flags & PORTF_CLEAR_BOTTOM) &&
           !(frontsector->ceilingpic == skyflatnum &&
             backsector->ceilingpic == skyflatnum) &&
           !(frontsector->floorpic == skyflatnum &&
             backsector->floorpic == skyflatnum);

  return false;
}

// (Maybe) add wall to GL draw items
static void AddWall(gl_wall_t *line, const gl_aintv_t* view)
{
  vertex_t* v1 = &vertexes[line->v1id];
  vertex_t* v2 = &vertexes[line->v2id];
  angle_t angle1;
  angle_t angle2;
  gl_aintv_t intv;

  angle1 = R_PointToPseudoAngle(v1->x, v1->y);
  angle2 = R_PointToPseudoAngle(v2->x, v2->y);

  // Backface culling
  if (angle2 - angle1 < ANG180)
    return;

  // Don't cull polyobj walls, as they may extend beyond the current chunk
  if (!poly_add_line)
  {
    // View culling
    GL_AIntvInit(&intv, angle2, angle1);
    if (!GL_AIntvIntersect(view, &intv, &intv))
      return;
  }

  // Wall is potentially visible
  gld_AddWall(line);
}

// Update GL plane info (safe to call multiple times per frame)
static void UpdatePlane(gl_chunk_t* chunk, gl_plane_t* source, dboolean ceiling)
{
  sector_t dummy;
  sector_t* sector;
  int cll, fll;

  if (source->validcount == validcount)
    return;

  sector = R_FakeFlat(chunk->sector, &dummy, &fll, &cll, false);

  if (ceiling)
  {
    source->lightlevel = cll;

    if (sector->ceilingheight > viewz || sector->ceilingpic == skyflatnum ||
        (sector->heightsec != -1 &&
         sectors[sector->heightsec].floorpic == skyflatnum))
    {
      source->height = sector->ceilingheight;
      source->special = sector->special;
      //source->xoffs = sector->ceiling_xoffs;
      //source->yoffs = sector->ceiling_yoffs;
      source->xscale = sector->ceiling_xscale;
      source->yscale = sector->ceiling_yscale;
      source->rotation = sector->ceiling_rotation;
      source->lightlevel = cll;
      source->picnum =
          sector->ceilingpic == skyflatnum && sector->sky & PL_SKYFLAT
              ? sector->sky
              : sector->ceilingpic;
    } else
      source->picnum = -1;
  }
  else
  {
    source->lightlevel = fll;

    if (sector->floorheight < viewz ||
        (sector->heightsec != -1 &&
         sectors[sector->heightsec].ceilingpic == skyflatnum))
    {
      source->height = sector->floorheight;
      source->special = sector->special;
      //source->xoffs = sector->floor_xoffs;
      //source->yoffs = sector->floor_yoffs;
      source->xscale = sector->floor_xscale;
      source->yscale = sector->floor_yscale;
      source->rotation = sector->floor_rotation;
      source->lightlevel = fll;
      source->picnum =
          sector->floorpic == skyflatnum && sector->sky & PL_SKYFLAT
              ? sector->sky
              : sector->floorpic;
    } else
      source->picnum = -1;
  }

  source->validcount = validcount;
}

static void ClearBleedTarget(bleedtarget_t* target)
{
  target->source = NULL;
  target->depth = UINT_MAX;
}

// Render flats of chunks now that all bleed effects have been updated
static void RenderDeferred(void)
{
  int i;

  for (i = 0; i < numdeferred; ++i)
  {
    gl_chunk_t* chunk = gl_rstate.deferred[i];
    int chunknum = chunk - gl_rstate.chunks;

    // Clear deferred mark
    chunk->flags &= ~GL_CHUNKF_DEFERRED;

    // Render ordinary planes if activated during BSP traversal

    if (chunk->floor.picnum != -1 && chunk->flags & GL_CHUNKF_RENDER_FLATS)
      gld_AddFlat(chunknum, false, &chunk->floor, chunk->sector);

    if (chunk->ceiling.picnum != -1 && chunk->flags & GL_CHUNKF_RENDER_FLATS)
      gld_AddFlat(chunknum, true, &chunk->ceiling, chunk->sector);
    
    // Render any over/under planes

    // Don't bleed "under" sky, as it will override the sky due to how sky
    // is rendered
    if (chunk->floorover.source && chunk->sector->floorpic != skyflatnum)
    {
      gl_plane_t* source = chunk->floorover.source;
      if (source->picnum != -1)
        gld_AddFlat(chunknum, false, source, chunk->sector);
    }

    if (chunk->floorunder.source)
    {
      gl_plane_t* source = chunk->floorunder.source;
      if (source->picnum != -1)
        gld_AddFlat(chunknum, false, source, chunk->sector);
    }

    if (chunk->ceilingover.source)
    {
      gl_plane_t* source = chunk->ceilingover.source;
      if (source->picnum != -1)
        gld_AddFlat(chunknum, true, source, chunk->sector);
    }

    if (chunk->ceilingunder.source && chunk->sector->ceilingpic != skyflatnum)
    {
      gl_plane_t* source = chunk->ceilingunder.source;
      if (source->picnum != -1)
        gld_AddFlat(chunknum, true, source, chunk->sector);
    }

    // If this was deferred by bleed-through (and not directly), render the
    // bleed source planes

    if (chunk->floorthrough.source && !(chunk->flags & GL_CHUNKF_RENDER_FLATS))
    {
      gl_plane_t* source = chunk->floorthrough.source;
      if (source->picnum != -1)
        gld_AddFlat(chunknum, false, source, chunk->sector);
    }

    if (chunk->ceilingthrough.source && !(chunk->flags & GL_CHUNKF_RENDER_FLATS))
    {
      gl_plane_t* source = chunk->ceilingthrough.source;
      if (source->picnum != -1)
        gld_AddFlat(chunknum, true, source, chunk->sector);
    }

    ClearBleedTarget(&chunk->floorunder);
    ClearBleedTarget(&chunk->floorover);
    ClearBleedTarget(&chunk->floorthrough);
    ClearBleedTarget(&chunk->ceilingunder);
    ClearBleedTarget(&chunk->ceilingover);
    ClearBleedTarget(&chunk->ceilingthrough);

    chunk->flatsvalid = validcount;
  }

  numdeferred = 0;
}

static void MarkChunkVisible(gl_chunk_t* chunk)
{
  if (chunk->flags & GL_CHUNKF_VISIBLE)
    // Already in visibile queue
    return;

  chunk->flags |= GL_CHUNKF_VISIBLE;
  *(vtail++) = chunk;
  if (vtail == gl_rstate.visible + gl_rstate.numchunks)
    vtail = gl_rstate.visible;
  numvisible++;
}

static void RenderChunk(gl_chunk_t* chunk)
{
  int i;

  // Render any polyobjects in this chunk
  if (chunk->numpolyobjs)
  {
    // Need to access metadata if polyobjs are present. This is going to hurt
    // cache performance, but there shouldn't be too many polyobjs on a map.
    chunkmeta_t* chunkmeta = &dsda_gl_rstate.chunkmeta[chunk - gl_rstate.chunks];
    int subnum;
    subsector_t* sub;
    submeta_t* submeta;
    gl_polyobj_t* gpo;

    for (subnum = chunkmeta->subsectors;
        subnum != -1;
        subnum = submeta->chunk_next)
    {
      sub = &gl_rstate.subsectors[subnum];
      submeta = &dsda_gl_rstate.submeta[subnum];

      if (!sub->poly)
        continue;

      gpo = &gl_rstate.polyobjs[sub->poly - polyobjs];
      // FIXME: plumb this into calls instead of using global variables
      poly_add_line = true;
      poly_frontsector = sub->sector;
      for (i = 0; i < gpo->numwalls; ++i)
        AddWall(&gl_rstate.walls[gpo->firstwall + i], &chunk->view);
      poly_add_line = false;
      poly_frontsector = NULL;
    }
  }

  // Render walls in this chunk
  for (i = 0; i < chunk->numwalls; ++i)
    AddWall(&gl_rstate.walls[chunk->firstwall + i], &chunk->view);

  // Render chunks across portals
  for (i = 0; i < chunk->numportals; ++i)
  {
    portal_t* portal = &gl_rstate.portals[chunk->firstportal + i];
    gl_chunk_t* pchunk = GL_Chunk(portal->chunk);
    gl_aintv_t newview;

    if (PortalIsClosed(portal, chunk->sector, pchunk->sector))
      // Skip portals where floor/ceiling heights close them off
      continue;

    // Compute angular interval of portal from player position
    GL_AIntvFromBBox(&newview, portal->bbox);

    // Clip view interval so far against portal
    if (!GL_AIntvIntersect(&chunk->view, &newview, &newview))
      // Portal is fully occluded
      continue;

    if (pchunk->viewvalid != validcount)
    {
      // Unvisited chunk, just set the view
      pchunk->view = newview;
      pchunk->viewvalid = validcount;
    }
    else
    {
      // Save original view
      gl_aintv_t orig = pchunk->view;

      // Compute union with previous view
      GL_AIntvUnion(&newview, &pchunk->view, &pchunk->view);

      // Union is approximate, so re-intersect with player view for sanity
      GL_AIntvIntersect(&playerview, &pchunk->view, &pchunk->view);

      // If the view expanded, we need to revisit the chunk to ensure
      // newly-visibile portals are traversed and newly-visible walls are
      // rendered. This process rapidly converges to a fixed point and
      // terminates even for complicated scenes.
      if (pchunk->view.start == orig.start && pchunk->view.end == orig.end)
        // View didn't change, skip portal
        continue;
    }

    // Chunk across portal should be rendered (or rerendered with larger view)
    MarkChunkVisible(pchunk);
  }

  // Update sector
  if (chunk->sector->validcount != validcount)
  {
    chunk->sector->validcount = validcount;

    UpdatePlane(chunk, &chunk->floor, false);
    UpdatePlane(chunk, &chunk->ceiling, true);

    R_AddSprites(chunk->sector,
                 (chunk->floor.lightlevel + chunk->ceiling.lightlevel) / 2);
  }

  // Update flats
  if (chunk->flatsvalid != validcount)
  {
    chunk->flatsvalid = validcount;

    UpdatePlane(chunk, &chunk->floor, false);
    UpdatePlane(chunk, &chunk->ceiling, true);

    if (chunk->flags & GL_CHUNKF_RENDER_FLATS &&
        (chunk->floor.picnum != -1 || chunk->ceiling.picnum != -1))
    {
      DeferFlats(chunk);

      // Apply bleeds
      for (i = 0; i < chunk->numbleeds; ++i)
        ApplyBleed(chunk, &gl_rstate.bleeds[chunk->firstbleed + i]);
    }
  }
}

// Render all chunks in visible queue
static void RenderChunks(void)
{
  while (numvisible)
  {
    // Pop next chunk
    gl_chunk_t* chunk = *(vhead++);

    if (vhead == gl_rstate.visible + gl_rstate.numchunks)
      // Wrap ring head pointer
      vhead = gl_rstate.visible;
    numvisible--;
    chunk->flags &= ~GL_CHUNKF_VISIBLE;

    // Do it
    RenderChunk(chunk);
  }
}

void GL_RenderBSP(void)
{
  // Find chunk containing player using BSP tree
  subsector_t* sub = GL_PointInSubsector(viewx, viewy);
  gl_chunk_t* chunk = GL_Chunk(sub->chunk);

  // Initialize view from player view
  GL_AIntvFromView(&playerview);
  chunk->view = playerview;
  chunk->viewvalid = validcount;

  // Initialise visibile queue ring
  vhead = vtail = gl_rstate.visible;

  // Render chunks starting with initial chunk
  MarkChunkVisible(chunk);
  RenderChunks();

  // Render deferred chunk flats
  RenderDeferred();
}
